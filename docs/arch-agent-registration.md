# Agent Registration & Multi-Rate Capability Architecture

**Document Version**: 2.0  
**Created**: 2024  
**Last Updated**: October 2025

## Overview

This document describes FEAGI's unified agent registration system with multi-rate capability architecture and two-tier registration model. This system addresses the critical temporal pattern replay bug caused by frequency mismatches between agents and FEAGI's processing rate, while providing sophisticated capability-aware polling and clean infrastructure/agent separation.

## Key Architecture Principles

1. **Single Registration Path**: All registrations flow through Registration Manager - no fallbacks
2. **Two-Tier Model**: Infrastructure (bridges) register separately from agents
3. **Transparent Proxying**: Bridges proxy agent registrations without auto-registering agents
4. **Clean Code**: No redundant fallback paths or legacy compatibility layers
5. **Automatic Capability Processing**: Rates extracted transparently from standard registration

## Problem Statement

### Original Issue: Temporal Pattern Replay Bug

**Symptoms:**
- Neural activity continues long after agent disconnection
- Same temporal patterns repeat cyclically 
- FCL (Fire Candidate List) shows persistent neuron activity from "ghost" data

**Root Cause:**
```
Agent Data Rate:    20 Hz (video frames every 50ms)
SHM Polling Rate:   200 Hz (every 5ms) 
FEAGI Burst Rate:   1 Hz (every 1000ms)

Problem: During each 1000ms burst window, SHM poller reads 
the same frame data 200 times, injecting identical temporal 
patterns repeatedly into FCL.
```

**Architecture Flaw:**
The original design used fixed polling rates that were decoupled from agent data rates and FEAGI processing rates, causing massive temporal data buffering and replay.

## Solution Architecture

### Core Principles

1. **Rate Negotiation**: Agents specify desired rates during registration
2. **Capability Isolation**: Each capability (sensory, motor, etc.) has independent rates
3. **Rate Validation**: System prevents capability rates exceeding FEAGI global rate
4. **Synchronized Polling**: Polling frequency matches registered capability rates
5. **Temporal Consistency**: Each data point is processed exactly once

### Architecture Overview

```mermaid
graph TB
    subgraph "Agent Registration"
        Agent[Agent: brain-visualizer] -->|HTTP or via Bridge| RegAPI[/v1/agent/register]
        Bridge[Infrastructure: feagi_bridge] -->|HTTP| RegAPI
    end
    
    subgraph "FEAGI Core - Single Path"
        RegAPI --> RegMgr[Registration Manager]
        RegMgr -->|Process Rates| CapMgr[Capability Rate Manager]
        RegMgr -->|Unified Registry| AgentList[/v1/agent/list]
        RegMgr -->|FQ Coordination| FQSampler[FQ Sampler]
    end
    
    subgraph "Rate-Based Polling"
        CapMgr -->|Agent Rates| SHMPoll[SHM Poller]
        CapMgr -->|Agent Rates| ZMQPoll[ZMQ Poller]
        SHMPoll -->|Fresh Data Only| FCL[Fire Candidate List]
        ZMQPoll -->|Fresh Data Only| FCL
        FCL --> BurstEngine[Burst Engine]
    end
    
    Note1["✅ No fallbacks<br/>✅ Single code path<br/>✅ Clean separation"]
```

### Two-Tier Registration Model

**Tier 1: Infrastructure Registration**
- Purpose: Register connectivity infrastructure (bridges, gateways)
- Agent Type: `"feagi_bridge"`, `"gateway"`, etc.
- Capabilities: Infrastructure-specific (e.g., `{"bridge": {"rate_hz": 60.0}}`)
- Metadata: `{"infrastructure": true, "proxy_mode": "transparent"}`
- Behavior: Does NOT auto-register end agents

**Tier 2: Agent Registration**
- Purpose: Register actual agents (sensors, motors, visualizers)
- Agent Type: `"visualizer"`, `"sensor"`, `"motor"`, etc.
- Capabilities: Agent-specific with rates (e.g., `{"visualization": {"rate_hz": 30.0}}`)
- Metadata: Optional agent-specific info
- Behavior: Can register directly OR through bridge as transparent proxy

## Component Details

### 1. Unified Agent Registration

#### Registration Protocol

**Endpoint**: `/v1/agent/register` (single endpoint handles all registration)

**Request Structure** (existing AgentRegistrationRequest with capability rate support):
```json
{
  "agent_id": "video_agent_001",
  "agent_type": "vision_sensor",
  "agent_data_port": 5000,
  "agent_version": "1.2.0",
  "controller_version": "2.1.0",
  "agent_ip": "192.168.1.100",
  "metadata": {},
  
  "capabilities": {
    "sensory": {
      "rate_hz": 30.0,
      "sensor_type": "camera",
      "resolution": "1920x1080"
    },
    "visualization": {
      "rate_hz": 5.0
    },
    "sensorimotor": {
      "sensory_rate_hz": 15.0,
      "motor_rate_hz": 20.0
    }
  }
}
```

**Response Structure** (standard SuccessResponse):
```json
{
  "success": true,
  "message": "Agent video_agent_001 registered successfully"
}
```

#### Registration Flow (Clean Single Path)

1. **Request Reception**: `/v1/agent/register` endpoint receives `AgentRegistrationRequest`
2. **Capability Rate Extraction**: Automatically extract rates from `capabilities` dict
3. **Registration Manager Processing**: 
   - Validate agent request
   - Create unified registry entry
   - Coordinate with FQ sampler
   - Update State Manager for legacy compatibility
4. **Capability Rate Storage**: Store rates for multi-rate polling coordination
5. **Success Response**: Return standard `SuccessResponse`

**Critical Design Rule**: 
- ❌ NO fallbacks to State Manager if Registration Manager unavailable
- ❌ NO legacy registration paths
- ✅ Registration Manager MUST be initialized during startup
- ✅ If Registration Manager not available → 503 Service Unavailable error

### 2. Capability Types

#### Standard Capability Types

| Type | Description | Typical Rate Range | Default Rate |
|------|-------------|-------------------|--------------|
| `sensory` | Sensor data input | 1-60 Hz | 10 Hz |
| `motor` | Motor command output | 1-100 Hz | 20 Hz |  
| `visualization` | Visual activity data | 1-30 Hz | 5 Hz |
| `control` | Control messages | 0.1-10 Hz | 1 Hz |
| `neurons_stream` | Legacy neuron data | 1-50 Hz | 10 Hz |

#### Capability Rate Specification

```python
class CapabilityRateSpec(BaseModel):
    capability_type: CapabilityType
    requested_rate_hz: float = Field(gt=0, le=1000)
    required: bool = Field(default=True)
    metadata: Optional[Dict[str, Any]] = None
```

### 3. Rate Validation System

#### Validation Rules

1. **Global Rate Constraint**: `capability_rate <= feagi_global_rate`
2. **Range Limits**: `0.1 Hz <= capability_rate <= 1000 Hz`  
3. **FEAGI Rate Limits**: `0.1 Hz <= feagi_rate <= 100 Hz`
4. **Rate Change Limits**: FEAGI rate increases limited to 2x current rate

#### Validation Process

```python
def validate_capability_rates(self, capability_specs: List[CapabilityRateSpec]) -> Dict[CapabilityType, str]:
    """
    Validate capability rates against FEAGI global rate.
    Returns dict mapping capability types to rejection reasons.
    """
    rejections = {}
    
    for spec in capability_specs:
        if spec.requested_rate_hz > self.feagi_global_rate_hz:
            rejections[spec.capability_type] = f"Rate {spec.requested_rate_hz}Hz exceeds FEAGI rate {self.feagi_global_rate_hz}Hz"
    
    return rejections
```

#### Rate Optimization

The system suggests optimal rates that are divisors of the FEAGI global rate for efficiency:

```python
def suggest_optimal_rate(self, capability_type: CapabilityType, requested_rate_hz: float) -> float:
    """Find the largest divisor of FEAGI rate that's <= requested rate."""
    for divisor in [1, 2, 3, 4, 5, 6, 8, 10, 12, 15, 20, 24, 30, 40, 50, 60]:
        candidate_rate = self.feagi_global_rate_hz / divisor
        if candidate_rate <= requested_rate_hz and candidate_rate >= 0.1:
            return candidate_rate
    return requested_rate_hz
```

### 4. Capability Rate Manager

#### Core Responsibilities

- **Rate Storage**: Store per-agent, per-capability rate configurations
- **Polling Coordination**: Determine which agents should be polled when  
- **Rate Updates**: Handle dynamic rate changes during runtime
- **State Persistence**: Integrate with FEAGI's state management system
- **Automatic Integration**: Works transparently with existing registration system

#### Rate Configuration Storage

```python
@dataclass
class AgentCapabilityRate:
    agent_id: str
    capability_type: CapabilityType
    approved_rate_hz: float
    last_poll_time_ns: int = 0
    poll_count: int = 0
    
    @property
    def poll_interval_ns(self) -> int:
        return int(1_000_000_000 / self.approved_rate_hz)
    
    def should_poll_now(self, current_time_ns: int) -> bool:
        if self.last_poll_time_ns == 0:
            return True
        elapsed_ns = current_time_ns - self.last_poll_time_ns
        return elapsed_ns >= self.poll_interval_ns
```

#### Polling Coordination

```python
def get_agents_for_capability_polling(
    self, 
    capability_type: CapabilityType,
    current_time_ns: Optional[int] = None
) -> List[AgentCapabilityRate]:
    """Get agents whose specified capability should be polled now."""
    if current_time_ns is None:
        current_time_ns = time.time_ns()
    
    agents_to_poll = []
    for registry in self._agent_registries.values():
        rate_config = registry.get_capability_rate(capability_type)
        if rate_config and rate_config.should_poll_now(current_time_ns):
            agents_to_poll.append(rate_config)
    
    return agents_to_poll
```

### 5. Smart Polling Implementation

#### Rate-Based SHM Polling

The enhanced SHM polling system replaces fixed-rate polling with capability-aware scheduling:

**Before (Fixed Rate)**:
```python
while self.running:
    # Process all agents every 5ms regardless of their rates
    for agent_id, reader in self._slot_readers.items():
        slot_data = reader.read_latest()
        if slot_data:
            await self._process_neural_payload_bytes(slot_data.data)
    
    await asyncio.sleep(0.005)  # Fixed 5ms = 200Hz
```

**After (Capability-Aware)**:
```python
while self.running:
    current_time_ns = time.time_ns()
    
    # Only poll agents whose rates indicate they should be polled now
    agents_to_poll = capability_manager.get_agents_for_capability_polling(
        CapabilityType.SENSORY, current_time_ns
    )
    
    for agent_id, rate_config in agents_to_poll.items():
        reader = self._slot_readers[agent_id]
        slot_data = reader.read_latest()
        if slot_data:
            await self._process_neural_payload_bytes(slot_data.data)
            capability_manager.mark_capability_polled(
                agent_id, rate_config.capability_type, current_time_ns
            )
    
    # Smart sleep: sleep until next agent needs to be polled
    next_poll_time_ns = self._calculate_next_poll_time(capability_manager, current_time_ns)
    sleep_duration_ms = (next_poll_time_ns - current_time_ns) / 1_000_000.0
    await asyncio.sleep(max(0.001, min(0.1, sleep_duration_ms / 1000.0)))
```

#### Intelligent Sleep Calculation

```python
def _calculate_next_poll_time(self, capability_manager, current_time_ns: int) -> int:
    """Calculate when the next agent should be polled based on registered rates."""
    next_poll_times = []
    
    for agent_id in capability_manager.get_all_registered_agents():
        registry = capability_manager.get_agent_capabilities(agent_id)
        if registry:
            for capability_rate in registry.capabilities.values():
                next_poll = capability_rate.last_poll_time_ns + capability_rate.poll_interval_ns
                if next_poll > current_time_ns:
                    next_poll_times.append(next_poll)
    
    return min(next_poll_times) if next_poll_times else current_time_ns + 10_000_000  # 10ms default
```

### 6. Automatic Capability Rate Processing

#### Transparent Integration

The system automatically processes capability rates from the existing `capabilities` dictionary without requiring API changes:

**Standard Registration**: `/v1/agent/register`  
**Behavior**: Automatically extracts and processes capability rates from existing request format

```python
def _process_agent_capabilities(self, request: AgentRegistrationRequest) -> Optional[List[Any]]:
    """Process agent capabilities and convert to rate specifications."""
    if not request.capabilities:
        return None
        
    capability_specs = []
    seen_capability_types = set()
    
    default_rates = {
        "sensory": 10.0,
        "motor": 20.0,
        "visualization": 5.0,
        "neurons_stream": 10.0,
        "control": 1.0
    }
    
    for cap_name, cap_config in request.capabilities.items():
        # Handle sensorimotor as combined capability
        if cap_name.lower() == "sensorimotor":
            # Split into sensory + motor capabilities
            # Extract rates from sensory_rate_hz, motor_rate_hz, or rate_hz
        
        # Map capability name to standard type
        cap_type = self._map_capability_name_to_type(cap_name)
        
        # Extract rate from config or use default
        final_rate = cap_config.get("rate_hz", default_rates.get(cap_name.lower(), 10.0))
        
        # Add to specs if not duplicate
        if cap_type not in seen_capability_types:
            capability_specs.append(CapabilityRateSpec(...))
            seen_capability_types.add(cap_type)
    
    return capability_specs
```

## Configuration Examples

### Infrastructure Registration (FEAGI Bridge)

```json
{
  "agent_id": "feagi_bridge",
  "agent_type": "feagi_bridge",
  "agent_data_port": 9050,
  "agent_version": "2.0.0",
  "controller_version": "1.0.0",
  "capabilities": {
    "bridge": {
      "rate_hz": 60.0,
      "enabled": true,
      "proxy_types": ["websocket", "zmq"],
      "supports_shm": true
    }
  },
  "metadata": {
    "infrastructure": true,
    "proxy_mode": "transparent",
    "websocket_port": 9050
  }
}
```

### Brain Visualizer Agent

```json
{
  "agent_id": "brain-visualizer",
  "agent_type": "visualizer",
  "agent_data_port": 0,
  "agent_version": "4.3.0",
  "controller_version": "2.0.0",
  "capabilities": {
    "visualization": {
      "rate_hz": 30.0,
      "enabled": true
    }
  },
  "metadata": {
    "request_shared_memory": true
  }
}
```

### High-Speed Vision Agent

```json
{
  "agent_id": "high_speed_camera",
  "agent_type": "vision_sensor",
  "agent_data_port": 5000,
  "agent_version": "1.0.0",
  "controller_version": "2.0.0",
  "capabilities": {
    "sensory": {
      "rate_hz": 60.0,
      "camera_fps": 120,
      "processing_mode": "real_time"
    }
  }
}
```

### Multi-Modal IoT Agent

```json
{
  "agent_id": "iot_sensor_hub",
  "agent_type": "environmental_sensor", 
  "agent_data_port": 5001,
  "agent_version": "1.2.0",
  "controller_version": "2.0.0",
  "capabilities": {
    "sensory": {
      "rate_hz": 1.0,
      "sensors": ["temperature", "humidity", "pressure"]
    },
    "motor": {
      "rate_hz": 0.5,
      "actuators": ["ventilation", "heating"]
    }
  }
}
```

### Robotic Agent with Sensorimotor Capability

```json
{
  "agent_id": "mobile_robot",
  "agent_type": "autonomous_vehicle",
  "agent_data_port": 5002, 
  "agent_version": "2.1.0",
  "controller_version": "2.0.0",
  "capabilities": {
    "sensorimotor": {
      "sensory_rate_hz": 30.0,
      "motor_rate_hz": 50.0,
      "sensors": ["lidar", "camera", "imu"],
      "actuators": ["wheels", "steering", "brakes"]
    },
    "visualization": {
      "rate_hz": 5.0
    }
  }
}
```

## System Integration

### State Manager Integration

Capability rates are stored in the FEAGI State Manager for persistence and system-wide access:

```python
# Store capability rates in agent registry
connected_agents[agent_id]["capability_rates"] = {
    "sensory": {
        "approved_rate_hz": 15.0,
        "poll_interval_ns": 66666666,
        "created_at": 1703123456.789
    }
}
```

### Monitoring and Statistics

The system provides comprehensive monitoring through the capability rate manager:

```python
def get_capability_statistics(self) -> Dict[str, Any]:
    """Get statistics about registered capabilities and polling rates."""
    return {
        "total_agents": len(self._agent_registries),
        "feagi_rate_hz": self._cached_feagi_rate_hz,
        "capabilities_by_type": {
            "sensory": 5,
            "motor": 3,
            "visualization": 2
        },
        "rate_distribution": {
            "1.0": 2,    # 2 capabilities at 1Hz
            "10.0": 4,   # 4 capabilities at 10Hz
            "15.0": 4    # 4 capabilities at 15Hz
        },
        "agents": {
            "agent_001": {
                "capability_count": 2,
                "capabilities": {
                    "sensory": {"rate_hz": 15.0, "poll_count": 1240},
                    "motor": {"rate_hz": 20.0, "poll_count": 1653}
                }
            }
        }
    }
```

## Performance Considerations

### Polling Efficiency

**Before**: Fixed 200Hz polling regardless of need
- **CPU Usage**: High (constant polling)
- **Memory**: Moderate (ring buffers)
- **Network**: High (unnecessary data transfers)
- **Temporal Accuracy**: Poor (data replay)

**After**: Rate-matched polling based on capability needs
- **CPU Usage**: Low (only poll when needed)
- **Memory**: Low (latest-only slots)  
- **Network**: Minimal (fresh data only)
- **Temporal Accuracy**: Excellent (no replay)

### Scalability

The system scales linearly with agent count rather than exponentially with polling frequency:

- **O(1)** time complexity for rate validation
- **O(n)** space complexity for n agents
- **O(log n)** time complexity for next-poll-time calculation
- **Bounded memory usage** with latest-only slots

### Resource Usage

| Component | Memory Usage | CPU Usage | Network Impact |
|-----------|--------------|-----------|----------------|
| Rate Manager | ~1KB per agent | Minimal | None |
| Validation | Constant | Low | None |
| Smart Polling | ~100B per capability | Variable (rate-dependent) | Reduced |
| Legacy Support | None (conversion only) | Minimal | None |

## Error Handling

### Registration Failures

| Error Scenario | HTTP Status | Response Handling |
|----------------|-------------|-------------------|
| Invalid rate format | 400 Bad Request | Validation error details |
| Rate exceeds FEAGI limit | 400 Bad Request | Rate limit explanation |
| Required capability rejected | 400 Bad Request | Specific rejection reasons |
| FEAGI rate change failed | 500 Internal Error | System error message |
| State manager unavailable | 503 Service Unavailable | Service dependency error |

### Runtime Error Recovery

- **Rate Processing Failure**: Continues with standard registration, logs warnings
- **State Persistence Error**: Continues with in-memory rates only
- **Agent Disconnection**: Automatic cleanup of rate configurations
- **Invalid Rate Data**: Uses sensible default rates with warning logs
- **Capability Manager Unavailable**: System continues with default behavior

## Migration Guide

### For Existing Agents

**No changes required** - all existing agents continue working with zero modifications. The system automatically:

1. **Processes existing capabilities**: Extracts rate information from existing `capabilities` dict
2. **Applies intelligent defaults**: Uses sensible default rates for each capability type
3. **Handles special cases**: Automatically splits `sensorimotor` capabilities into separate rates
4. **Maintains compatibility**: All existing API contracts remain unchanged

### For New Agents  

**Enhanced capability specification** (optional):
```json
{
  "capabilities": {
    "sensory": {
      "rate_hz": 15.0,
      "sensor_type": "camera"
    },
    "motor": {
      "rate_hz": 25.0,
      "actuator_type": "servo"
    }
  }
}
```

**Benefits of explicit rates**:
- Optimized polling frequencies for your specific use case
- Better resource utilization
- Reduced temporal pattern replay issues
- More predictable data processing timing

### System Configuration

**FEAGI Configuration** (optional tuning):
```toml
[api.agent_registration]  
# Default rates applied when not specified
default_sensory_rate_hz = 10.0
default_motor_rate_hz = 20.0
default_visualization_rate_hz = 5.0
default_control_rate_hz = 1.0

# Rate limits for validation
max_capability_rate_hz = 1000.0
min_capability_rate_hz = 0.1
```

## Testing

### Unit Tests

```python
def test_rate_validation():
    validator = CapabilityRateValidator(feagi_global_rate_hz=10.0)
    specs = [CapabilityRateSpec(
        capability_type=CapabilityType.SENSORY,
        requested_rate_hz=15.0  # Exceeds global rate
    )]
    rejections = validator.validate_capability_rates(specs)
    assert CapabilityType.SENSORY in rejections

def test_polling_schedule():
    manager = CapabilityRateManager(state_manager)
    # Register agent with 5Hz rate
    manager.register_agent_capabilities("test_agent", [
        CapabilityRateSpec(CapabilityType.SENSORY, 5.0)
    ])
    
    # Should not poll immediately after marking polled
    agents = manager.get_agents_for_capability_polling(CapabilityType.SENSORY)
    assert len(agents) == 0
    
    # Should poll after sufficient time passes
    time.sleep(0.21)  # > 200ms for 5Hz rate
    agents = manager.get_agents_for_capability_polling(CapabilityType.SENSORY) 
    assert len(agents) == 1
```

### Integration Tests

```python
async def test_enhanced_registration_flow():
    request = EnhancedAgentRegistrationRequest(
        agent_id="test_agent",
        capability_rates=[
            CapabilityRateSpec(CapabilityType.SENSORY, 10.0)
        ]
    )
    
    api = EnhancedAgentRegistrationAPI(core_api_service)
    response = await api.register_agent_enhanced(request)
    
    assert response.success
    assert len(response.approved_capabilities) == 1
    assert CapabilityType.SENSORY in response.approved_capabilities
```

### Performance Tests

```python
def test_polling_performance():
    # Test that rate-based polling reduces CPU usage
    # compared to fixed-rate polling
    
    # Setup agents with various rates
    setup_agents_with_rates([1, 5, 10, 15, 20])  # Hz
    
    # Measure CPU usage over time
    cpu_usage = measure_cpu_usage_over_time(duration=60)  # 60 seconds
    
    # Should be significantly lower than 200Hz fixed polling
    assert cpu_usage.average < FIXED_RATE_CPU_BASELINE * 0.3
```

## Security Considerations

### Rate Limit Protection

- **FEAGI Rate Changes**: Limited to 2x current rate to prevent system overload
- **Capability Rate Bounds**: Enforced minimums and maximums prevent abuse
- **Registration Rate Limiting**: Prevent rapid registration/deregistration attacks

### Validation Security

- **Input Sanitization**: All rate values validated for type and range
- **Agent Identity**: Rate configurations tied to authenticated agent IDs
- **State Isolation**: Agent rate configs isolated from each other

### Resource Protection  

- **Memory Bounds**: Rate manager uses bounded data structures
- **CPU Protection**: Smart polling prevents excessive CPU usage
- **Network Limits**: Rate validation prevents network flooding

## Future Enhancements

### Planned Features

1. **Dynamic Rate Adjustment**: Allow rate changes during runtime
2. **Rate Profiles**: Predefined rate configurations for common agent types  
3. **Adaptive Rates**: Automatically adjust rates based on system load
4. **Rate Analytics**: Detailed analysis and optimization recommendations
5. **Rate Scheduling**: Advanced scheduling algorithms for complex scenarios

### Research Areas

1. **Machine Learning Integration**: Predict optimal rates based on data patterns
2. **Resource-Aware Scheduling**: Adjust rates based on system resource availability
3. **Multi-Agent Coordination**: Coordinate rates across related agents
4. **Quality of Service**: Priority-based rate allocation

## Implementation Guidelines

### For FEAGI Core Developers

**Registration Manager Initialization**:
- MUST call `ProcessManager.init_important_processes()` during startup
- Registration Manager is a REQUIRED component, not optional
- If initialization fails → FEAGI should not start (fail-fast)

**No Fallback Policy**:
- `/v1/agent/register` MUST NOT fall back to State Manager
- `/v1/agent/list` MUST NOT fall back to State Manager
- If Registration Manager unavailable → return 503 Service Unavailable
- This forces proper initialization and prevents silent failures

**Code Cleanliness**:
- Remove all legacy registration paths
- Remove all fallback mechanisms
- Single source of truth: Registration Manager
- State Manager updated via Registration Manager for legacy compatibility only

### For Bridge Developers

**Infrastructure Registration**:
- Register bridge itself with `agent_type: "feagi_bridge"`
- Include `metadata.infrastructure: true`
- Specify `proxy_mode: "transparent"`
- Do NOT auto-register end agents

**Transparent Proxying**:
- Agent registration requests pass through unchanged
- Bridge forwards `/v1/agent/register` calls to FEAGI
- FEAGI sees agent registration, not bridge registration
- Bridge can intercept for SHM coordination if needed

### For Agent Developers

**Direct Registration** (agent connects directly to FEAGI):
```python
registration_data = {
    "agent_id": "my_agent",
    "agent_type": "sensor",
    "agent_data_port": 5000,
    "agent_version": "1.0.0",
    "controller_version": "2.0.0",
    "capabilities": {
        "sensory": {"rate_hz": 30.0, "enabled": true}
    }
}
response = requests.post("http://feagi:8000/v1/agent/register", json=registration_data)
```

**Via Bridge** (agent connects to bridge, which forwards to FEAGI):
```python
# Same payload, different endpoint
registration_data = {
    "agent_id": "my_agent",
    "agent_type": "sensor",
    "agent_data_port": 5000,
    "agent_version": "1.0.0",
    "controller_version": "2.0.0",
    "capabilities": {
        "sensory": {"rate_hz": 30.0, "enabled": true}
    }
}
# Bridge proxies this to FEAGI transparently
response = requests.post("http://bridge:8000/v1/agent/register", json=registration_data)
```

## Conclusion

The unified agent registration system with integrated multi-rate capability architecture successfully addresses the temporal pattern replay bug through a clean, single-path implementation. The two-tier infrastructure/agent model ensures clear separation of concerns while the transparent proxying architecture allows flexible deployment topologies.

Key benefits:

- **Eliminates temporal pattern replay bugs** through rate-matched polling
- **Clean single-path architecture** - no fallbacks or redundant code
- **Two-tier registration model** - clear infrastructure/agent separation
- **Transparent proxying** - agents work identically via bridge or direct connection
- **Automatic capability processing** from existing registration format
- **Resource efficiency** through smart polling schedules
- **Fail-fast initialization** - ensures Registration Manager availability
- **Zero agent migration required** - existing agents work unchanged

**Breaking Changes from Legacy System**:
1. Registration Manager is now REQUIRED (not optional)
2. No fallback paths to State Manager
3. Bridge registers as infrastructure, not as agents
4. 503 errors if Registration Manager unavailable (fail-fast, not fail-silent)

The architecture achieves sophisticated rate management and clean separation of concerns through disciplined implementation without fallbacks or legacy compatibility layers. This proves that complex systems can have elegant, maintainable solutions when designed with clear principles.
