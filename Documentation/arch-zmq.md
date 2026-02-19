# ZeroMQ Architecture in FEAGI

*Last Updated: May 25, 2025*

## Overview

This document describes the comprehensive ZeroMQ (ZMQ) architecture in FEAGI, implementing a **multi-stream architecture** where each stream has a specific purpose and protocol. This design eliminates protocol confusion, improves performance, and provides clear separation of concerns. A key innovation is the **differentiated FQ sampler** that provides optimized data streams for different consumer types, with **intelligent agent-driven coordination** that automatically manages FQ samplers based on connected agent capabilities.

## Architecture Principles

1. **Multi-Stream Separation**: Dedicated streams for different protocols and purposes
2. **Protocol Isolation**: Each stream only accepts its designated message format
3. **Asynchronous Processing**: Non-blocking message handling for performance
4. **Connection Management**: Centralized connection tracking and lifecycle management
5. **Binary Efficiency**: Optimized binary protocols for high-throughput data exchange
6. **Versioning**: Protocol versioning for backward compatibility
7. **Differentiated Data Delivery**: Optimized sampling behavior for different subscriber types
8. **Agent Registration & Coordination**: Automatic FQ sampler management based on connected agent capabilities
9. **Zero-Manual Intervention**: Intelligent system coordination eliminates need for manual data flow management

## Multi-Stream Architecture Overview

FEAGI implements **Option 4: Separate Dedicated Streams** for optimal performance, maintainability, and protocol separation.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           FEAGI ZMQ Server                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────┐ │
│  │ REQ/REP     │ │ Control     │ │ REST        │ │Visualization│ │ Motor   │ │
│  │ Stream      │ │ Stream      │ │ Stream      │ │ Stream      │ │Stream  │ │
│  │ Port 5555   │ │ Port 5561   │ │ Port 5563   │ │ Port 5562   │ │Port 5564│ │
│  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘ └─────────┘ │
│  │               │               │               │             │           │ │
│  │ Legacy ZMQ    │ Legacy        │ REST API      │ All Neural  │ OPU Only  │ │
│  │ Commands      │ Control       │ Operations    │ Activity    │ Motor     │ │
│  └───────────────┴───────────────┴───────────────┴─────────────┴───────────┘ │
├─────────────────────────────────────────────────────────────────────────────┤
│                      Enhanced FQ Sampler                                     │
│  ┌─────────────────────────────┐   ┌─────────────────────────────────────┐  │
│  │   Visualization Path        │   │         Motor Path                  │  │
│  │ • All Cortical Areas        │   │ • OPU Areas Only                   │  │
│  │ • Configured Sample Rates   │   │ • Burst Frequency                  │  │
│  │ • Rich Data Format          │   │ • Optimized for Real-time          │  │
│  └─────────────────────────────┘   └─────────────────────────────────────┘  │
├─────────────────────────────────────────────────────────────────────────────┤
│                           FEAGI Core Engine                                    │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Stream Specifications

### **REQ/REP Stream (Port 5555)**

**Purpose**: Traditional ZMQ request-reply pattern for legacy command operations.

**Characteristics**:
- **Socket Type**: REP (Reply)
- **Pattern**: Synchronous request-reply
- **Protocol**: Legacy ZMQ command format
- **Use Cases**: Command-line tools, legacy integrations

**Message Format**:
```json
{
  "command": "get_status|set_config|...",
  "parameters": {...},
  "timestamp": 1621234567890
}
```

**Client Example (Rust SDK via Python bindings)**:
```python
import os
from feagi.pns.client import AgentType, FeagiAgentClient

client = FeagiAgentClient(os.environ["FEAGI_AGENT_DESCRIPTOR_B64"], AgentType.SENSORY)
client.configure(
    feagi_host=os.environ["FEAGI_HOST"],
    registration_port=int(os.environ["FEAGI_REGISTRATION_PORT"]),
    sensory_port=int(os.environ["FEAGI_SENSORY_PORT"]),
    motor_port=int(os.environ["FEAGI_MOTOR_PORT"]),
    heartbeat_interval=float(os.environ["FEAGI_HEARTBEAT_INTERVAL_S"]),
    connection_timeout_ms=int(os.environ["FEAGI_CONNECTION_TIMEOUT_MS"]),
    registration_retries=int(os.environ["FEAGI_REGISTRATION_RETRIES"]),
)
client.connect()
```

### **REST Stream (Port 5563) - Primary API Interface**

**Purpose**: Modern REST API operations with HTTP-like semantics over ZMQ. This is the primary interface for all API communication, replacing legacy control protocols.

**Characteristics**:
- **Socket Type**: ROUTER/DEALER
- **Pattern**: Asynchronous request-reply with HTTP semantics
- **Protocol**: REST API format only
- **Use Cases**: Modern applications, web interfaces, API clients, agent management
- **Scope**: Handles all API operations including agent registration, heartbeats, system status, and data operations

#### REST Message Format

**Request Format**:
```json
{
  "route": "/v1/path/to/resource",
  "method": "GET|POST|PUT|DELETE",
  "params": {
    "param1": "value1"
  },
  "query": {
    "filter": "value"
  },
  "body": {
    "property": "value"
  },
  "headers": {
    "authorization": "Bearer token"
  },
  "timestamp": 1621234567890
}
```

**Response Format**:
```json
{
  "status": 200,
  "headers": {
    "content-type": "application/json"
  },
  "body": {
    "property": "value"
  },
  "timestamp": 1621234567890
}
```

#### REST API Examples

**System Health Check**:
```json
// Request
{
  "route": "/v1/system/health_check",
  "method": "GET",
  "timestamp": 1621234567890
}

// Response
{
  "status": 200,
  "headers": {"content-type": "application/json"},
  "body": {"status": "healthy"},
  "timestamp": 1621234567890
}
```

**Agent Registration** (replaces legacy control protocol):
```json
// Request
{
  "route": "/v1/agents/register",
  "method": "POST",
  "body": {
    "agent_id": "my_godot_client",
    "agent_type": "godot_bridge",
    "capabilities": ["sensory", "motor"],
    "protocol_versions": {
      "rest": "1.0",
      "visualization": "1.0"
    }
  },
  "timestamp": 1621234567890
}

// Response
{
  "status": 200,
  "headers": {"content-type": "application/json"},
  "body": {
    "message": "Agent registered successfully",
    "agent_id": "my_godot_client"
  },
  "timestamp": 1621234567890
}
```

**Agent Heartbeat** (replaces legacy control protocol):
```json
// Request
{
  "route": "/v1/agents/heartbeat",
  "method": "POST",
  "body": {
    "agent_id": "my_godot_client"
  },
  "timestamp": 1621234567890
}

// Response
{
  "status": 200,
  "headers": {"content-type": "application/json"},
  "body": {"status": "ok"},
  "timestamp": 1621234567890
}
```

## Agent Registration & FQ Sampler Coordination

### Overview

FEAGI 2.0 introduces an intelligent agent registration system that provides automatic coordination between connected agents and the FQ (Fire Queue) samplers. This eliminates the need for manual intervention to enable data flow for visualization and motor streams.

### Architecture Flow

```
Agent Registration → Capability Detection → Automatic FQ Sampler Enable/Disable → Data Flow
```

### Key Components

1. **Agent Registry**: Centralized tracking of connected agents with capabilities
2. **Capability Detection**: Automatic detection of agent visualization and motor requirements
3. **FQ Sampler Management**: RUST/RTOS compatible enable/disable coordination
4. **Resource Optimization**: Samplers only consume CPU when agents actually need data

### Agent Registration Process

#### 1. Agent Registration
```json
// POST /v1/agent/register
{
  "route": "/v1/agent/register",
  "method": "POST",
  "body": {
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
    }
  }
}
```

**Response with Automatic FQ Sampler Coordination**:
```json
{
  "status": 200,
  "body": {
    "message": "Agent registered successfully",
    "agent_id": "godot_visualizer_001",
    "fq_samplers_enabled": {
      "visualization": true,  // Automatically enabled due to visualization capability
      "motor": false          // Remains disabled (no motor capability)
    }
  }
}
```

#### 2. Capability-Based FQ Sampler Management

**Visualization Capability Detection**:
- Agent registers with `capabilities.visualization = true`
- System automatically enables Visualization FQ Sampler (30Hz, all cortical areas)
- Data flows to port 5562 for visualization consumption

**Motor Capability Detection**:
- Agent registers with `capabilities.motor = true` OR `capabilities.output = true` OR `capabilities.sensorimotor = true`
- System automatically enables Motor FQ Sampler (100Hz, OPU areas only)
- Data flows to port 5564 for motor control consumption

#### 3. Agent Deregistration & Cleanup
```json
// DELETE /v1/agent/deregister
{
  "route": "/v1/agent/deregister",
  "method": "DELETE",
  "body": {
    "agent_id": "godot_visualizer_001"
  }
}
```

**Automatic Cleanup Response**:
```json
{
  "status": 200,
  "body": {
    "message": "Agent deregistered successfully",
    "agent_id": "godot_visualizer_001",
    "fq_samplers_disabled": {
      "visualization": true,  // Disabled if no other visualization agents remain
      "motor": false          // Remains in current state
    }
  }
}
```

### FQ Sampler Coordination Logic

#### Visualization FQ Sampler
- **Enable Condition**: At least one agent with `visualization` capability registered
- **Disable Condition**: No agents with `visualization` capability remain
- **Sampling**: 30Hz, all cortical areas
- **Port**: 5562

#### Motor FQ Sampler
- **Enable Condition**: At least one agent with motor capabilities (`motor`, `output`, or `sensorimotor`)
- **Disable Condition**: No agents with motor capabilities remain
- **Sampling**: 100Hz, OPU areas only
- **Port**: 5564

### Agent Management Endpoints

#### List Active Agents
```json
// GET /v1/agent/list
{
  "route": "/v1/agent/list",
  "method": "GET"
}
```

**Response**:
```json
{
  "status": 200,
  "body": {
    "agents": [
      {
        "agent_id": "godot_visualizer_001",
        "agent_type": "brain_visualizer",
        "capabilities": ["visualization", "sensory"],
        "status": "active",
        "last_seen": "2025-06-07T12:34:56Z"
      },
      {
        "agent_id": "robotic_arm_controller",
        "agent_type": "motor_controller",
        "capabilities": ["motor", "sensorimotor"],
        "status": "active",
        "last_seen": "2025-06-07T12:35:12Z"
      }
    ],
    "summary": {
      "total_agents": 2,
      "visualization_agents": 1,
      "motor_agents": 1
    }
  }
}
```

#### Agent Properties
```json
// GET /v1/agent/properties/{agent_id}
{
  "route": "/v1/agent/properties/godot_visualizer_001",
  "method": "GET"
}
```

#### FQ Sampler Status
```json
// GET /v1/agent/fq_sampler_status
{
  "route": "/v1/agent/fq_sampler_status",
  "method": "GET"
}
```

**Response**:
```json
{
  "status": 200,
  "body": {
    "visualization_fq_sampler": {
      "enabled": true,
      "reason": "1 visualization agent(s) connected",
      "agents_requiring": ["godot_visualizer_001"]
    },
    "motor_fq_sampler": {
      "enabled": true,
      "reason": "1 motor agent(s) connected",
      "agents_requiring": ["robotic_arm_controller"]
    }
  }
}
```

### Benefits of Agent-Driven Coordination

1. **Zero Manual Intervention**: No need to manually enable/disable FQ samplers
2. **Resource Efficiency**: Samplers only consume CPU when agents actually need data
3. **RUST/RTOS Compatibility**: Uses enable/disable pattern rather than create/destroy
4. **Robust Coordination**: Thread-safe operations with comprehensive error handling
5. **Real-time Monitoring**: Full visibility into agent-sampler relationships

### Migration from Manual Management

**Before (Manual)**:
```bash
# Manual intervention required
curl -X POST "http://localhost:8000/v1/system/enable_visualization_fq_sampler"
curl -X POST "http://localhost:8000/v1/system/enable_motor_fq_sampler"
```

**After (Automatic)**:
```python
# Zero manual intervention - automatic coordination
agent_client.register(
    agent_id="<agent_descriptor_b64>",
    capabilities={"visualization": True}
)
# Visualization FQ Sampler automatically enabled
```

**Get Cortical Area**:
```json
// Request
{
  "route": "/v1/connectome/cortical_area/12345",
  "method": "GET",
  "params": {"cortical_id": "12345"},
  "timestamp": 1621234567890
}

// Response
{
  "status": 200,
  "headers": {"content-type": "application/json"},
  "body": {
    "id": "12345",
    "name": "Visual Cortex",
    "type": "sensory",
    "position": [10, 10, 10],
    "dimensions": [10, 10, 1]
  },
  "timestamp": 1621234567890
}
```

### **Visualization Stream (Port 5562) - Enhanced with Differentiated Sampling**

**Purpose**: Real-time neural activity data broadcasting with comprehensive brain state information.

**Characteristics**:
- **Socket Type**: PUB (Publisher)
- **Pattern**: Publish-subscribe (one-to-many)
- **Protocol**: Binary `feagi_bytes` format (FVP - FEAGI Visualization Protocol)
- **Use Cases**: Brain visualization, monitoring dashboards, data analysis
- **Differentiated Behavior**: Samples ALL cortical areas at configurable rates

**Enhanced Features:**
- **Threading-Based Architecture**: RTOS-compatible synchronous threading implementation (May 2025)
- **Enhanced Client Tracking**: Real-time heartbeat monitoring with 30-second timeout
- **Automatic Subscriber Detection**: Monitors client connections and automatically enables/disables FQ sampler
- **Per-Area Sampling Rates**: Configurable sampling frequency per cortical area via `fq_sample_rate` property
- **Rich Data Format**: Includes membrane potentials, thresholds, coordinates, and firing history
- **Efficient Routing**: Only processes visualization-targeted data from FQ sampler
- **Responsive Shutdown**: Fast thread management with less than 2 second shutdown time
- **Production Logging**: Clean, structured logging suitable for production environments

#### Threading Architecture

The visualization stream implements a **threading-based architecture** designed for RTOS compatibility:

**Thread 1: FQ Data Worker**
- Processes fire queue data from FQ sampler
- Handles multiple data formats (tagged, tuple, dict)
- Publishes binary data via ZMQ PUB socket
- Responsive to shutdown signals (200ms intervals)

**Thread 2: Client Cleanup Worker**
- Monitors client heartbeat timeouts (30-second default)
- Removes inactive clients automatically
- Thread-safe client list management
- 250ms responsiveness for shutdown

**Thread 3: Subscriber Monitor Worker**
- Automatically enables/disables FQ sampler based on client presence
- Resource conservation when no clients connected
- 200ms check intervals for responsive shutdown

#### Enhanced Client Lifecycle Management

```python
# Client Connection Flow:
# 1. Client connects to port 5562 for data
# 2. Client sends initial heartbeat via REST API
#    POST /v1/visualization/heartbeat {"client_id": "my_client"}
# 3. FQ sampler automatically enabled when first client connects
# 4. Client maintains heartbeat every 5-15 seconds
# 5. Automatic cleanup after 30 seconds without heartbeat
# 6. FQ sampler disabled when last client disconnects

class VisualizationStream:
    def __init__(self, ...):
        # Enhanced client tracking
        self.client_last_heartbeat = {}
        self.client_heartbeat_timeout = 30
        self._client_lock = threading.Lock()

        # Automatic subscriber management
        self._subscriber_count = 0
        self._fq_sampler_enabled = False

        # Responsive shutdown
        self._stop_event = threading.Event()

    def heartbeat_visualization_client(self, client_id: str) -> None:
        """Enhanced heartbeat method with proper client tracking."""
        current_time = time.time()

        with self._client_lock:  # Thread-safe access
            is_new_client = client_id not in self.client_last_heartbeat
            self.client_last_heartbeat[client_id] = current_time

            if is_new_client:
                logger.info(f"📺 New visualization client connected: {client_id}")
                # Automatic FQ sampler enablement for new clients
                if len(self.client_last_heartbeat) == 1:  # First client
                    self._control_fq_sampler(True)
```

#### REST API Integration

The visualization stream integrates with the REST API for heartbeat management:

**Heartbeat Endpoint**: `POST /v1/visualization/heartbeat`
```json
{
  "client_id": "my_visualization_client"
}
```

**Response**:
```json
{
  "message": "Heartbeat received from client my_visualization_client"
}
```

**Data Format**: Binary `feagi_bytes` with enhanced structure:
```python
# Visualization data includes:
{
    'cortical_ids': ['area1', 'area2', ...],      # Cortical area identifiers
    'x_coords': [x1, x2, ...],                   # Neuron X coordinates
    'y_coords': [y1, y2, ...],                   # Neuron Y coordinates
    'z_coords': [z1, z2, ...],                   # Neuron Z coordinates
    'potentials': [pot1, pot2, ...],             # Membrane potentials
    'thresholds': [thr1, thr2, ...],             # Firing thresholds
    'fire_counts': [fc1, fc2, ...],              # Consecutive fire counts
    'refractory': [ref1, ref2, ...]              # Refractory counters
}
```

**Configuration Examples**:
```python
# Per-area sampling configuration
area.properties['fq_sample_rate'] = 30.0  # 30Hz for this specific area
area.properties['fq_sample_rate'] = 0.0   # Disable sampling for this area

Use the Rust SDK (`feagi-rust-py-libs`) or `feagi-io` to subscribe to the visualization stream and send heartbeats; Python ZMQ bindings are not required.
```

### **Motor Stream (Port 5564) - New Differentiated Real-Time Motor Control**

**Purpose**: High-performance real-time motor control data broadcasting with minimal latency.

**Characteristics**:
- **Socket Type**: PUB (Publisher)
- **Pattern**: Publish-subscribe optimized for real-time control
- **Protocol**: Binary `feagi_bytes` format optimized for motor control (FSMP - FEAGI Sensorimotor Protocol)
- **Use Cases**: Robotic control, real-time motor output, actuator commands
- **Differentiated Behavior**: Samples ONLY OPU (Output Processing Unit) areas at burst frequency

**Enhanced Features:**
- **Real-Time Optimization**: Samples at burst frequency (~100Hz) for minimal control latency
- **OPU Area Detection**: Automatically detects motor-relevant cortical areas using multiple criteria
- **Efficient Data Format**: Streamlined data format optimized for motor control applications
- **Fast Timeout Detection**: Faster heartbeat timeouts for real-time control requirements

**OPU Area Detection Logic**:
```python
# Multiple detection methods for OPU areas:
is_opu = (
    area_type == 'OPU' or               # Explicit type
    area_type == 'OUTPUT' or            # Output type
    area_type == 'MOTOR' or             # Motor type
    'OPU' in area_type or               # Contains OPU
    'OUTPUT' in area_type or            # Contains OUTPUT
    'MOTOR' in area_type or             # Contains MOTOR
    area.id.startswith('opu_') or       # Name prefix
    area.id.startswith('motor_') or     # Motor prefix
    area.id.startswith('output_')       # Output prefix
)
```

**Motor Data Format**: Optimized binary format for real-time control:
```python
# Motor data includes (streamlined for performance):
{
    'cortical_ids': ['motor_left', 'motor_right', ...], # Only OPU areas
    'x_coords': [x1, x2, ...],                         # Motor neuron coordinates
    'y_coords': [y1, y2, ...],                         # Motor neuron coordinates
    'z_coords': [z1, z2, ...],                         # Motor neuron coordinates
    'potentials': [pot1, pot2, ...]                    # Motor activation levels
}
```

**Client Configuration Examples**:

Use the Rust SDK (`feagi-rust-py-libs`) or `feagi-io` to configure motor subscriptions and heartbeats; Python ZMQ bindings are not required.

```python
# Example OPU area configuration in genome
motor_cortex = {
    "id": "motor_cortex",
    "properties": {
        "cortical_type": "OPU",  # Automatically detected for motor sampling
        # No fq_sample_rate needed - uses burst frequency automatically
    }
}
```

## Enhanced FQ Sampler Architecture

### Differentiated Sampling Behavior

The Enhanced FQ Sampler implements **dual-path sampling** optimized for different use cases:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           Fire Queue Provider                                 │
└─────────────────────────┬───────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                      Enhanced FQ Sampler                                     │
│                                                                             │
│  ┌─────────────────────────────┐   ┌─────────────────────────────────────┐  │
│  │   Visualization Path        │   │         Motor Path                  │  │
│  │                             │   │                                     │  │
│  │ • ALL Cortical Areas        │   │ • OPU Areas ONLY                   │  │
│  │ • Per-Area Sample Rates     │   │ • Burst Frequency (~100Hz)         │  │
│  │ • Rich Data (8 fields)      │   │ • Streamlined Data (4 fields)      │  │
│  │ • Tuple Format Output       │   │ • Tagged Dict Format               │  │
│  │ • Heartbeat Detection       │   │ • Fast Timeout Detection           │  │
│  └─────────────┬───────────────┘   └─────────────┬───────────────────────┘  │
└────────────────┼─────────────────────────────────┼─────────────────────────┘
                 │                                 │
                 ▼                                 ▼
┌────────────────────────────────┐    ┌────────────────────────────────┐
│      Visualization Stream      │    │        Motor Stream            │
│      (Port 5562)               │    │        (Port 5564)             │
│                                │    │                                │
│ • PUB Socket                   │    │ • PUB Socket                   │
│ • feagi_bytes Encoding         │    │ • feagi_bytes Encoding         │
│ • All Neural Activity          │    │ • Motor Commands Only          │
│ • Research/Monitoring          │    │ • Real-Time Control            │
│ • 20-60Hz Configurable         │    │ • 100Hz+ Burst-Synchronized    │
└────────────────────────────────┘    └────────────────────────────────┘
```

### Automatic Subscriber Management

**Visualization Stream Subscriber Management**:
```python
class VisualizationStream:
    async def _monitor_subscribers(self):
        """Monitor visualization subscribers and control FQ sampler."""
        while self.running:
            current_count = self._get_subscriber_count()
            if current_count != self._last_subscriber_count:
                should_enable = current_count > 0
                if should_enable != self._fq_sampler_enabled:
                    await self._control_fq_sampler(should_enable)
            await asyncio.sleep(self.subscriber_check_interval)

    async def _control_fq_sampler(self, enable: bool):
        """Enable/disable FQ sampler for visualization."""
        if self.fq_sampler:
            if enable:
                logger.info("🔔 Enabling FQ sampler - visualization clients connected")
                self.fq_sampler.set_visualization_subscribers(True)
            else:
                logger.info("🔕 Disabling FQ sampler - no visualization clients")
                self.fq_sampler.set_visualization_subscribers(False)
```

**Motor Stream Subscriber Management**:
```python
class MotorStream:
    async def _monitor_subscribers(self):
        """Monitor motor subscribers and control FQ sampler."""
        while self.running:
            current_count = self.get_connected_client_count()
            if current_count != self._last_subscriber_count:
                should_enable = current_count > 0
                if should_enable != self._fq_sampler_enabled:
                    await self._control_fq_sampler(should_enable)
            await asyncio.sleep(self.subscriber_check_interval)  # Faster for real-time

    async def _control_fq_sampler(self, enable: bool):
        """Enable/disable FQ sampler for motor control."""
        if self.fq_sampler:
            if enable:
                logger.info("🔔 Enabling FQ sampler for motor - motor clients connected")
                self.fq_sampler.set_motor_subscribers(True)
            else:
                logger.info("🔕 Disabling FQ sampler for motor - no motor clients")
                self.fq_sampler.set_motor_subscribers(False)
```

## Protocol Separation Rules

### **Strict Protocol Enforcement**

Each stream **ONLY** accepts its designated message format:

1. **REST Stream (5563)** - Only accepts messages with `method` and `route` fields
2. **Control Stream (5561)** - Only accepts messages with `message_type` field
3. **REQ/REP Stream (5555)** - Only accepts messages with `command` field
4. **Visualization Stream (5562)** - Only broadcasts binary data

### **Protocol Validation**

**REST Stream Validation**:
```python
def is_valid_rest_message(message):
    return (
        isinstance(message, dict) and
        'method' in message and
        'route' in message and
        isinstance(message['method'], str) and
        isinstance(message['route'], str)
    )
```

**Control Stream Validation**:
```python
def is_valid_control_message(message):
    return (
        isinstance(message, dict) and
        'message_type' in message and
        isinstance(message['message_type'], str)
    )
```

### **Error Handling for Wrong Protocols**

**Sending REST message to Control stream**:
```
Request:  {"method": "GET", "route": "/v1/health"} → Control Stream (5561)
Response: {
  "message_type": "error",
  "error": "REST API messages should be sent to the dedicated REST stream port",
  "timestamp": 1621234567890
}
```

**Sending Control message to REST stream**:
```
Request:  {"message_type": "hello"} → REST Stream (5563)
Response: {
  "status": 400,
  "body": {
    "type": "error",
    "code": "INVALID_REST_FORMAT",
    "message": "Message must contain 'method' and 'route' fields"
  },
  "timestamp": 1621234567890
}
```

## Component Architecture

### ZMQManager

The central coordinator for all ZMQ communications:

```python
class ZMQManager:
    def __init__(self, host="*", req_port=5555, control_port=5561,
                 rest_port=5563, vis_port=5562):
        self.context = zmq.Context.instance()
        self.host = host
        self.ports = {
            "req": req_port,
            "control": control_port,
            "rest": rest_port,
            "visualization": vis_port
        }
        self.connection_manager = ConnectionManager(self.context, **self.ports)
        self.streams = {}
        self._initialize_streams()
```

### Stream Handlers

**REST Stream Handler**:
```python
class RestStreamHandler:
    async def process_rest_message(self, message):
        if not self.is_valid_rest_format(message):
            return self.create_error_response(400, "INVALID_REST_FORMAT")

        # Route to appropriate REST endpoint
        return await self.route_rest_request(
            message['method'],
            message['route'],
            message.get('params', {}),
            message.get('query', {}),
            message.get('body', {})
        )
```

**Control Stream Handler**:
```python
class ControlStreamHandler:
    async def process_control_message(self, message):
        if not self.is_valid_control_format(message):
            return self.create_error_response("INVALID_CONTROL_FORMAT")

        # Handle legacy control messages
        return await self.handle_control_command(
            message['message_type'],
            message.get('agent_id'),
            message.get('data', {})
        )
```

## Performance Characteristics

### Stream Performance

| Stream | Latency | Throughput | Use Case | FQ Sampler Behavior |
|--------|---------|------------|----------|-------------------|
| REST (5563) | ~2-3ms | High | API operations | Not applicable |
| Control (5561) | ~1-2ms | Medium | Agent management | Not applicable |
| Visualization (5562) | ~20-50ms | Very High | Real-time visualization | All areas, configurable rates |
| Motor (5564) | ~5-10ms | High | Real-time motor control | OPU areas only, burst frequency |
| REQ/REP (5555) | ~3-5ms | Low | Legacy commands | Not applicable |

### Performance Benefits

1. **No Message Routing Overhead**: Messages go directly to the right handler
2. **Protocol-Specific Optimizations**: Each stream optimized for its use case
3. **Independent Scaling**: Scale different streams based on load patterns
4. **Reduced Contention**: No competing message types on same endpoint
5. **Differentiated Data Delivery**: Optimized sampling reduces unnecessary processing
6. **Automatic Resource Management**: FQ sampler only runs when subscribers are present

### Enhanced FQ Sampler Performance

**Visualization Path Performance:**
- **Sampling Scope**: All cortical areas (comprehensive brain state)
- **Default Rate**: 20Hz configurable per area (1-60Hz typical range)
- **Data Richness**: 8 data fields per neuron (coordinates, potentials, thresholds, etc.)
- **Latency Target**: 20-50ms (optimized for completeness over speed)
- **Use Cases**: Research analysis, brain monitoring, development visualization

**Motor Path Performance:**
- **Sampling Scope**: OPU areas only (typically 1-10% of total areas)
- **Sampling Rate**: Burst frequency (~100Hz for real-time control)
- **Data Efficiency**: 4 data fields per neuron (coordinates and activation levels)
- **Latency Target**: 5-10ms (optimized for speed over completeness)
- **Use Cases**: Robotic control, real-time actuation, motor feedback loops

## Configuration

### Command Line Arguments

```bash
python -m feagi.main \
  --zmq-req-port 5555 \
  --zmq-control-port 5561 \
  --zmq-rest-port 5563 \
  --zmq-vis-port 5562 \
  --zmq-motor-port 5564
```

### Environment Variables

```bash
export FEAGI_ZMQ_REQ_PORT=5555
export FEAGI_ZMQ_CONTROL_PORT=5561
export FEAGI_ZMQ_REST_PORT=5563
export FEAGI_ZMQ_VIS_PORT=5562
export FEAGI_ZMQ_MOTOR_PORT=5564
```

### Configuration File

```json
{
  "zmq": {
    "host": "127.0.0.1",
    "req_port": 5555,
    "control_port": 5561,
    "rest_port": 5563,
    "vis_port": 5562,
    "motor_port": 5564
  },
  "fq_sampler": {
    "visualization": {
      "default_sample_rate": 20.0,
      "auto_enable_on_subscribers": true,
      "subscriber_check_interval": 1.0,
      "include_membrane_potentials": true,
      "include_coordinates": true,
      "include_firing_history": true
    },
    "motor": {
      "sample_at_burst_frequency": true,
      "auto_detect_opu_areas": true,
      "subscriber_check_interval": 0.5,
      "motor_timeout_seconds": 10.0,
      "optimize_for_latency": true
    }
  }
}
```

### Per-Area FQ Sampler Configuration

```python
# Configure different sampling rates for different cortical areas
cortical_areas = {
    "visual_cortex": {
        "properties": {
            "fq_sample_rate": 30.0,  # High rate for visual analysis
            "cortical_type": "sensory"
        }
    },
    "motor_cortex": {
        "properties": {
            "cortical_type": "OPU",  # Automatically uses burst frequency for motor
            # No fq_sample_rate needed - controlled by motor stream
        }
    },
    "memory_hippocampus": {
        "properties": {
            "fq_sample_rate": 5.0,   # Low rate for memory areas
            "cortical_type": "associative"
        }
    },
    "background_noise": {
        "properties": {
            "fq_sample_rate": 0.0,   # Disable sampling for noise areas
            "cortical_type": "utility"
        }
  }
}
```

## Authentication and Security

### REST Stream Authentication

```json
{
  "headers": {
    "authorization": "Bearer eyJhbGciOiJIUzI1NiIsInR5cCI..."
  }
}
```

### Control Stream Authentication

```json
{
  "message_type": "hello",
  "agent_id": "secure_agent",
  "authentication": {
    "token": "bearer_token_here"
  }
}
```

## Testing and Monitoring

### Stream Connectivity Testing

```bash
# Test all streams are listening
ss -tuln | grep -E "5555|5561|5562|5563|5564"

# Expected output:
tcp LISTEN 127.0.0.1:5555   # REQ/REP
tcp LISTEN 127.0.0.1:5561   # Control
tcp LISTEN 127.0.0.1:5562   # Visualization
tcp LISTEN 127.0.0.1:5563   # REST
tcp LISTEN 127.0.0.1:5564   # Motor
```

### Protocol Validation Testing

Use the Rust SDK or `feagi-io` to validate REST and control streams; Python ZMQ bindings are not required.

### Enhanced FQ Sampler Testing

Use the Rust SDK or `feagi-io` to validate visualization/motor sampling behavior. Python ZMQ bindings are not required.

## Migration Guide

## Recent Changes

### VisualizationStream Threading Enhancement (May 2025)

**Major Enhancement**: Complete rewrite of `VisualizationStream` with threading-based architecture.

**Issues Resolved**:
- Fixed `SimpleVisualizationStream` → `VisualizationStream` import errors
- Eliminated FEAGI shutdown hanging (reduced from 10+ seconds to less than 2 seconds)
- Resolved critical server initialization order dependency in `server.py`
- Cleaned up excessive debugging logs (>90% reduction in log volume)

**New Features**:
- **Threading-Based Architecture**: RTOS-compatible synchronous design with 3 worker threads
- **Enhanced Client Tracking**: Real-time heartbeat monitoring with automatic cleanup
- **REST API Integration**: Heartbeat endpoint `POST /v1/visualization/heartbeat`
- **Responsive Shutdown**: Fast thread management with event-based signaling
- **Production Logging**: Clean, structured logs suitable for production use

**Performance Improvements**:
- Shutdown time: 10+ seconds → less than 2 seconds (80%+ improvement)
- Thread responsiveness: 1000ms+ → 200-250ms (75%+ improvement)
- Error recovery: Manual restart → Automatic socket recreation (100% automated)
- Log volume: >1000 debug lines/min → ~10 info lines/min (90%+ reduction)

**Testing Coverage**:
- Created `tests/api/zmq/test_visualization_stream.py` with 19 comprehensive test cases
- All tests passing, covering threading, client management, FQ sampler control, error handling

**Migration Impact**:
- **Zero breaking changes** - Full backward compatibility maintained
- **Enhanced heartbeat system** - REST API-based client lifecycle management
- **Better monitoring** - Real-time connection status and statistics
- **Production-ready** - Clean logs, proper error handling

### From Unified Stream Architecture

**Before (Unified)**:
```python
# Single control stream handled everything
client = ZMQClient(port=5561)  # Mixed REST and control
```

**After (Separate Streams)**:
```python
# Dedicated streams for each protocol
rest_client = ZMQRestClient(port=5563)     # REST only
control_client = ControlClient(port=5561)  # Control only
```

## Best Practices

### Stream Selection Guidelines

- **REST Stream (5563)**: Modern APIs, web interfaces, bridges
- **Control Stream (5561)**: Agent registration, heartbeats, legacy control
- **Visualization Stream (5562)**: Real-time monitoring, dashboards
- **REQ/REP Stream (5555)**: Command-line tools, simple scripts

### Performance Optimization

1. **Choose the Right Stream**: Use the most appropriate stream for your use case
2. **Batch Operations**: Where possible, batch multiple operations
3. **Connection Pooling**: Reuse connections for multiple operations
4. **Async Operations**: Use async clients for better throughput

## Windows Platform Compatibility

### Windows ZMQ Binding Issues & Solutions

**Issue**: Windows systems may encounter "Permission denied" errors when FEAGI attempts to bind ZMQ sockets to `127.0.0.1` addresses:

```
Failed to start ZMQ services: Permission denied (addr='tcp://127.0.0.1:5558')
```

**Root Cause**: Windows handles loopback interface binding differently than Linux/macOS. Binding to `127.0.0.1` can trigger Windows Firewall or network stack restrictions, even when the port is available.

**Solution Architecture**: FEAGI 2.0 implements an automatic Windows compatibility fix in the Process Manager:

```python
# Windows-specific ZMQ binding fix in process_manager.py
import platform
if platform.system() == "Windows" and zmq_host in ["127.0.0.1", "localhost"]:
    logger.info(f"🪟 Windows detected: Converting ZMQ host '{zmq_host}' to '*' for proper binding")
    zmq_bind_host = "*"  # Bind to all interfaces on Windows only
else:
    # On non-Windows platforms, use the configured host directly
    # ZMQ will handle 0.0.0.0 appropriately on each platform
    zmq_bind_host = zmq_host
```

### Cross-Platform Configuration

**Recommended Configuration** (`feagi_configuration.toml`):

```toml
[api]
host = "0.0.0.0"  # Cross-platform compatible for all interfaces
port = 8080

[zmq]
host = "0.0.0.0"  # Works cross-platform; Windows converts loopback addresses to "*" automatically

[ports]
zmq_req_rep_port = 5555
zmq_pub_sub_port = 5556
zmq_push_pull_port = 5557
zmq_sensory_port = 5558
zmq_control_port = 5559
zmq_visualization_port = 5562
zmq_rest_port = 5563
zmq_motor_port = 5564
```

**Environment Variable Overrides** (Production):
```bash
# Linux/Docker deployment
export FEAGI_ZMQ_HOST=0.0.0.0
export FEAGI_API_HOST=0.0.0.0

# Windows development
export FEAGI_ZMQ_HOST=0.0.0.0
export FEAGI_API_HOST=0.0.0.0

# Kubernetes deployment
export FEAGI_ZMQ_HOST=0.0.0.0  # Service discovery handles routing
export FEAGI_API_HOST=0.0.0.0
```

### Platform-Specific Considerations

#### Windows Development
- **Use `0.0.0.0`**: Always configure hosts as `0.0.0.0` instead of `127.0.0.1`
- **ZMQ Binding**: FEAGI automatically converts loopback addresses to `"*"` for ZMQ socket binding on Windows only
- **Firewall**: Windows Defender may prompt for network access - allow FEAGI through firewall
- **Port Conflicts**: Use `netstat -ano | findstr :5555` to check for port conflicts

#### Linux/macOS Development
- **Standard Configuration**: `0.0.0.0` works universally without conversion
- **Docker Compatibility**: `0.0.0.0` ensures containers can bind properly
- **No Special Handling**: Platform differences are handled automatically - ZMQ uses the configured host directly

#### Container Deployments
- **Docker**: Always use `0.0.0.0` for host binding inside containers
- **Kubernetes**: Use `0.0.0.0` with Service discovery for external access
- **Network Policies**: Configure appropriate NetworkPolicies for ZMQ ports

### Troubleshooting Windows ZMQ Issues

**Error**: `Failed to start ZMQ services: Permission denied`

**Solutions**:
1. **Configuration Fix** (Permanent):
   ```toml
   # In feagi_configuration.toml
   [zmq]
   host = "0.0.0.0"  # NOT "127.0.0.1"
   ```

2. **Environment Override** (Temporary):
   ```bash
   set FEAGI_ZMQ_HOST=0.0.0.0
   python main.py
   ```

3. **Check Port Availability**:
   ```bash
   netstat -ano | findstr :5558
   # Should show no existing connections
   ```

4. **Windows Firewall**:
   - Allow Python/FEAGI through Windows Defender Firewall
   - Or run PowerShell as Administrator: `New-NetFirewallRule -DisplayName "FEAGI ZMQ" -Direction Inbound -Port 5555-5564 -Protocol TCP -Action Allow`

**Error**: `Address already in use`

**Solutions**:
1. **Find Conflicting Process**:
   ```bash
   netstat -ano | findstr :5558
   taskkill /PID <process_id> /F
   ```

2. **Change Ports** (if needed):
   ```toml
   # In feagi_configuration.toml
   [ports]
   zmq_sensory_port = 5568  # Use different port
   ```

### Architecture Compliance

The Windows compatibility fix maintains FEAGI's architecture compliance:

- ✅ **No Hardcoded Fallbacks**: Uses configuration system with `

### ZMQ Asyncio Event Loop Issue

**Problem:** On Windows, Python 3.8+ uses the `ProactorEventLoop` by default, but ZMQ asyncio requires the `SelectorEventLoop` for compatibility.

**Error Symptoms:**
```
ERROR: Proactor event loop does not implement add_reader family of methods required for zmq.
zmq will work with proactor if tornado >= 6.1 can be found.
Use asyncio.set_event_loop_policy(WindowsSelectorEventLoopPolicy()) or install 'tornado>=6.1' to avoid this error.
```

**Solution:** FEAGI automatically detects Windows and sets the appropriate event loop policy:

```python
# In main.py and zmq/server.py
if platform.system() == "Windows":
    try:
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
        print("🪟 Windows detected: Set SelectorEventLoopPolicy for ZMQ compatibility")
    except AttributeError:
        print("⚠️ Warning: WindowsSelectorEventLoopPolicy not available - ZMQ may have issues")
```

**Implementation Details:**
- Applied in `feagi/main.py` during system initialization
- Applied in `feagi/api/zmq/server.py` when creating ZMQ server threads
- Uses `@architecture:acceptable` annotation for Windows-specific platform code
- Gracefully handles older Python versions without WindowsSelectorEventLoopPolicy

This fix ensures cross-platform compatibility while maintaining FEAGI's platform-agnostic architecture principles.
