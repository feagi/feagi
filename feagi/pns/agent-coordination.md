# Agent Registration & Coordination System

*FEAGI 2.0 - Peripheral Nervous System (PNS) Module*

## Overview

The Agent Registration & Coordination System provides intelligent management of connected agents and automatic coordination with FEAGI's Fire Queue (FQ) samplers. This system eliminates manual intervention for data flow management while maintaining RUST/RTOS compatibility and resource efficiency.

## Architecture

### Core Components

```
┌─────────────────────────────────────────────────────────────────┐
│                    Agent Registration System                     │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ │
│  │   Agent     │ │ Capability  │ │ FQ Sampler  │ │   State     │ │
│  │ Registry    │ │ Detection   │ │ Manager     │ │ Manager     │ │
│  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘ │
├─────────────────────────────────────────────────────────────────┤
│                    Coordination Logic                            │
│  • Agent registers → Capabilities detected → FQ samplers enabled │
│  • Agent deregisters → Check remaining → FQ samplers disabled    │
│  • Thread-safe operations with comprehensive error handling      │
└─────────────────────────────────────────────────────────────────┘
```

### Data Flow Architecture

```
Agent Registration
        ↓
Capability Detection
        ↓
┌─────────────────┐     ┌─────────────────┐
│ Visualization   │     │     Motor       │
│ Capability?     │     │ Capability?     │
└─────────────────┘     └─────────────────┘
        ↓                       ↓
   Enable Viz FQ            Enable Motor FQ
   Sampler (30Hz)           Sampler (100Hz)
        ↓                       ↓
   Port 5562 Data          Port 5564 Data
   (All Cortical)          (OPU Areas Only)
```

## Agent Registration API

### Registration Endpoint

**Route**: `POST /v1/agent/register`

**Request Format**:
```json
{
  "agent_id": "unique_agent_identifier",
  "agent_type": "brain_visualizer|motor_controller|sensory_processor|hybrid",
  "capabilities": {
    "visualization": true|false,
    "motor": true|false,
    "sensory": true|false,
    "output": true|false,
    "sensorimotor": true|false
  },
  "metadata": {
    "version": "2.0.1",
    "platform": "godot_4.2|ros2|custom",
    "description": "Optional agent description"
  }
}
```

**Successful Response**:
```json
{
  "message": "Agent registered successfully",
  "agent_id": "unique_agent_identifier",
  "fq_samplers_enabled": {
    "visualization": true,
    "motor": false
  },
  "coordination_status": "FQ samplers automatically coordinated based on capabilities"
}
```

### Deregistration Endpoint

**Route**: `DELETE /v1/agent/deregister`

**Request Format**:
```json
{
  "agent_id": "unique_agent_identifier"
}
```

**Successful Response**:
```json
{
  "message": "Agent deregistered successfully",
  "agent_id": "unique_agent_identifier",
  "fq_samplers_disabled": {
    "visualization": true,
    "motor": false
  },
  "remaining_agents": {
    "visualization_count": 0,
    "motor_count": 1
  }
}
```

## Capability Detection Logic

### Visualization Capability

**Detection Criteria**:
- `capabilities.visualization = true`

**Action**:
- Enable Visualization FQ Sampler (30Hz sampling rate)
- Provides neural activity data for all cortical areas
- Data published to port 5562

**Use Cases**:
- Brain visualizers (Godot-based, web-based)
- Neural activity monitors
- Real-time brain state displays

### Motor Capability

**Detection Criteria**:
- `capabilities.motor = true` OR
- `capabilities.output = true` OR
- `capabilities.sensorimotor = true`

**Action**:
- Enable Motor FQ Sampler (100Hz sampling rate)
- Provides neural activity data for OPU (Output Processing Unit) areas only
- Data published to port 5564

**Use Cases**:
- Robotic controllers
- Virtual environment motor systems
- Physical actuator interfaces

### Sensory Capability

**Detection Criteria**:
- `capabilities.sensory = true` OR
- `capabilities.sensorimotor = true`

**Action**:
- Registers agent as sensory data provider
- No FQ sampler coordination (sensory is input, not output)
- Agent can send sensory data via sensory stream (port 5558)

## FQ Sampler Coordination

### Visualization FQ Sampler

**Configuration**:
- **Sampling Rate**: 30Hz
- **Data Scope**: All cortical areas
- **Port**: 5562
- **Format**: Full neural activity visualization data

**Enable Logic**:
```python
def should_enable_visualization_fq_sampler():
    return len(agents_with_visualization_capability) > 0
```

**Disable Logic**:
```python
def should_disable_visualization_fq_sampler():
    return len(agents_with_visualization_capability) == 0
```

### Motor FQ Sampler

**Configuration**:
- **Sampling Rate**: 100Hz (matches burst frequency)
- **Data Scope**: OPU areas only
- **Port**: 5564
- **Format**: Motor-optimized neural activity data

**Enable Logic**:
```python
def should_enable_motor_fq_sampler():
    return len(agents_with_motor_capability) > 0
```

**Disable Logic**:
```python
def should_disable_motor_fq_sampler():
    return len(agents_with_motor_capability) == 0
```

## State Management

### Agent Registry Structure

```python
agent_registry = {
    "agents": {
        "agent_id": {
            "agent_type": str,
            "capabilities": dict,
            "metadata": dict,
            "registration_time": datetime,
            "last_seen": datetime,
            "status": "active|inactive"
        }
    },
    "capability_counts": {
        "visualization": int,
        "motor": int,
        "sensory": int
    },
    "fq_sampler_states": {
        "visualization_enabled": bool,
        "motor_enabled": bool
    }
}
```

### Thread Safety

The agent coordination system uses thread-safe operations with proper locking:

```python
class AgentCoordinator:
    def __init__(self):
        self._lock = threading.RLock()

    def register_agent(self, agent_data):
        with self._lock:
            # Thread-safe registration and FQ coordination
            pass

    def deregister_agent(self, agent_id):
        with self._lock:
            # Thread-safe deregistration and cleanup
            pass
```

## Monitoring & Status

### List Active Agents

**Route**: `GET /v1/agent/list`

**Response**:
```json
{
  "agents": [
    {
      "agent_id": "godot_visualizer_001",
      "agent_type": "brain_visualizer",
      "capabilities": ["visualization", "sensory"],
      "status": "active",
      "last_seen": "2025-06-07T12:34:56Z"
    }
  ],
  "summary": {
    "total_agents": 1,
    "visualization_agents": 1,
    "motor_agents": 0,
    "sensory_agents": 1
  }
}
```

### Agent Properties

**Route**: `GET /v1/agent/properties/{agent_id}`

**Response**:
```json
{
  "agent_id": "godot_visualizer_001",
  "agent_type": "brain_visualizer",
  "capabilities": {
    "visualization": true,
    "motor": false,
    "sensory": true
  },
  "metadata": {
    "version": "2.0.1",
    "platform": "godot_4.2"
  },
  "status": "active",
  "registration_time": "2025-06-07T12:30:00Z",
  "last_seen": "2025-06-07T12:34:56Z"
}
```

### FQ Sampler Coordination Status

**Route**: `GET /v1/agent/fq_sampler_status`

**Response**:
```json
{
  "visualization_fq_sampler": {
    "enabled": true,
    "reason": "1 visualization agent(s) connected",
    "agents_requiring": ["godot_visualizer_001"],
    "sampling_rate": "30Hz",
    "port": 5562
  },
  "motor_fq_sampler": {
    "enabled": false,
    "reason": "0 motor agent(s) connected",
    "agents_requiring": [],
    "sampling_rate": "100Hz",
    "port": 5564
  }
}
```

## Implementation Details

### CoreAPIService Integration

The agent coordination system integrates with FEAGI's CoreAPIService:

```python
class CoreAPIService:
    def get_agent_registry_summary(self):
        """Provides agent statistics for FQ sampler decisions"""
        return {
            "visualization_agent_count": count,
            "motor_agent_count": count
        }
```

### Process Manager Integration

FQ sampler enable/disable methods integrate with the process manager:

```python
# Enable samplers when agents register
process_manager.enable_viz_fq_sampler()
process_manager.enable_motor_fq_sampler()

# Disable samplers when no agents remain
process_manager.disable_viz_fq_sampler()
process_manager.disable_motor_fq_sampler()
```

### State Manager Integration

Agent registry data persists in the centralized state manager:

```python
# RUST/RTOS compatible binary structure
state_manager.set_state("connected_agents", agent_registry)
state_manager.set_state("fq_sampler_coordination", coordination_status)
```

## Error Handling

### Registration Errors

**Duplicate Agent ID**:
```json
{
  "error": "Agent with ID 'existing_agent' already registered",
  "error_code": "DUPLICATE_AGENT_ID",
  "suggested_action": "Use different agent_id or deregister existing agent"
}
```

**Invalid Capabilities**:
```json
{
  "error": "Invalid capability specification",
  "error_code": "INVALID_CAPABILITIES",
  "details": "At least one capability must be specified"
}
```

### Deregistration Errors

**Agent Not Found**:
```json
{
  "error": "Agent with ID 'unknown_agent' not found",
  "error_code": "AGENT_NOT_FOUND",
  "suggested_action": "Verify agent_id or check if agent was already deregistered"
}
```

### FQ Sampler Coordination Errors

**Process Manager Unavailable**:
```json
{
  "warning": "FQ sampler coordination failed - process manager unavailable",
  "error_code": "PROCESS_MANAGER_UNAVAILABLE",
  "impact": "Agent registered but FQ samplers not automatically coordinated"
}
```

## Benefits & Features

### Zero Manual Intervention

**Before**: Manual FQ sampler management
```bash
curl -X POST "http://localhost:8000/v1/system/enable_visualization_fq_sampler"
curl -X POST "http://localhost:8000/v1/system/enable_motor_fq_sampler"
```

**After**: Automatic coordination
```python
agent.register(
    agent_id="my_visualizer",
    capabilities={"visualization": True}
)
# Visualization FQ Sampler automatically enabled
```

### Resource Efficiency

- **CPU Usage**: FQ samplers only consume resources when agents actually need data
- **Memory**: Minimal memory overhead for agent registry
- **Network**: Data only flows when consumers are connected

### RUST/RTOS Compatibility

- **Enable/Disable Pattern**: Uses enable/disable rather than create/destroy for better embedded compatibility
- **Binary State Structure**: Agent registry uses binary-compatible data structures
- **Thread Safety**: All operations are thread-safe for multi-threaded environments

### Comprehensive Monitoring

- **Real-time Status**: Live agent and FQ sampler status monitoring
- **Agent Properties**: Detailed agent information and capabilities
- **Coordination Visibility**: Full transparency into agent-sampler relationships

## Migration Guide

### For Agent Developers

**Old Pattern** (Manual FQ management):
```python
# Manual setup required
enable_visualization_fq_sampler()
start_agent()
```

**New Pattern** (Automatic coordination):
```python
# Register with capabilities - FQ samplers auto-coordinated
agent.register(
    agent_id="my_agent",
    agent_type="visualizer",
    capabilities={"visualization": True}
)
```

### For System Administrators

**Old Pattern**: Manual monitoring and enabling of FQ samplers
**New Pattern**: Monitor agent registration status, FQ samplers automatically coordinated

## Future Enhancements

### Planned Features

1. **Agent Heartbeat System**: Automatic detection of disconnected agents
2. **Capability Negotiation**: Dynamic capability registration and updates
3. **Load Balancing**: Multiple agents sharing same capability types
4. **Agent Priorities**: Priority-based FQ sampler resource allocation
5. **Historical Analytics**: Agent connection patterns and usage statistics

### API Extensions

1. **Batch Registration**: Register multiple agents in single operation
2. **Conditional Registration**: Register only if certain conditions met
3. **Agent Groups**: Logical grouping of related agents
4. **Dynamic Reconfiguration**: Change agent capabilities without deregistration

## Troubleshooting

### Common Issues

**FQ Samplers Not Enabling**:
1. Check agent capabilities are properly specified
2. Verify process manager is running
3. Check state manager connectivity

**Agent Registration Failing**:
1. Verify unique agent_id
2. Check required capabilities format
3. Ensure FEAGI core services are running

**Data Not Flowing After Registration**:
1. Confirm FQ sampler coordination status via `/v1/agent/fq_sampler_status`
2. Check ZMQ port connectivity
3. Verify agent is listening on correct port

### Debug Commands

```bash
# Check agent registration status
curl -X GET "http://localhost:8000/v1/agent/list"

# Check FQ sampler coordination
curl -X GET "http://localhost:8000/v1/agent/fq_sampler_status"

# Manual FQ sampler control (fallback)
curl -X POST "http://localhost:8000/v1/system/enable_visualization_fq_sampler"
```
