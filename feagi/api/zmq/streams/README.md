# FEAGI ZMQ Streams

*Last Updated: May 24, 2025*

## Overview

This directory contains the ZeroMQ stream implementations for FEAGI, providing specialized data channels optimized for different use cases. A key innovation is the **differentiated FQ sampler integration** that provides optimized data delivery for different subscriber types.

## Stream Architecture

### Differentiated Data Delivery

FEAGI implements **dual-path FQ sampling** to optimize data delivery:

- **Visualization Path**: Comprehensive brain state monitoring with all cortical areas
- **Motor Path**: Real-time motor control with OPU areas only

## Stream Implementations

### VisualizationStream (`visualization.py`)

**Purpose**: Real-time neural activity broadcasting for brain visualization and analysis.

**Key Features:**
- **Comprehensive Sampling**: Receives data from ALL cortical areas
- **Configurable Rates**: Respects per-area `fq_sample_rate` properties
- **Rich Data Format**: Full neuron state including membrane potentials, thresholds, coordinates
- **Automatic Subscriber Detection**: Enables FQ sampler when visualization clients connect
- **Heartbeat Monitoring**: Tracks client connections and automatically disables sampling when no clients

**Port**: 5562  
**Socket Type**: PUB (Publisher)  
**Protocol**: feagi_bytes binary format  

```python
# Client connection example
import zmq
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5562")
socket.setsockopt(zmq.SUBSCRIBE, b"activity")

# Register for subscriber detection
control_socket = context.socket(zmq.DEALER)
control_socket.connect("tcp://localhost:5561")
heartbeat = {
    "message_type": "heartbeat",
    "agent_id": "visualization_client_001",
    "timestamp": time.time() * 1000
}
control_socket.send_json(heartbeat)
```

### MotorStream (`motor.py`)

**Purpose**: High-performance real-time motor control data broadcasting.

**Key Features:**
- **OPU-Only Sampling**: Receives data from OPU (Output Processing Unit) areas only
- **Burst-Frequency Sampling**: Samples at burst frequency (~100Hz) for minimal latency
- **Optimized Data Format**: Streamlined for real-time motor control applications
- **Fast Subscriber Detection**: Shorter timeouts and faster monitoring for real-time requirements
- **Automatic OPU Detection**: Intelligently identifies motor-relevant cortical areas

**Port**: 5564  
**Socket Type**: PUB (Publisher)  
**Protocol**: feagi_bytes binary format optimized for motor control  

```python
# Client connection example
import zmq
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5564")
socket.setsockopt(zmq.SUBSCRIBE, b"motor")

# Register for subscriber detection
control_socket = context.socket(zmq.DEALER)
control_socket.connect("tcp://localhost:5561")
heartbeat = {
    "message_type": "heartbeat",
    "agent_id": "motor_controller_robot_01", 
    "timestamp": time.time() * 1000
}
control_socket.send_json(heartbeat)
```

**OPU Area Detection:**
Motor stream automatically detects OPU areas using multiple criteria:
- `cortical_type` property: "OPU", "OUTPUT", "MOTOR"
- Area ID prefixes: "opu_", "motor_", "output_"
- Contains keywords: "OPU", "OUTPUT", "MOTOR" in type or name

### SensoryStream (`sensory.py`)

**Purpose**: Handles incoming sensory data from agents.

**Key Features:**
- **Input Processing**: Receives sensory data from external agents
- **Protocol Support**: FSMP (FEAGI Sensorimotor Protocol)
- **Data Validation**: Validates incoming sensory messages
- **Routing**: Routes sensory data to appropriate cortical areas

**Port**: 5558  
**Socket Type**: SUB (Subscriber)  
**Protocol**: FSMP binary format  

### ControlStream (`control.py`)

**Purpose**: Legacy control protocol for agent management.

**Key Features:**
- **Agent Registration**: Handles agent hello/goodbye messages
- **Heartbeat Monitoring**: Tracks agent health and connection status
- **Legacy Support**: Maintains compatibility with existing control protocols
- **Connection Management**: Manages agent lifecycle and state

**Port**: 5561  
**Socket Type**: ROUTER  
**Protocol**: Legacy control message format  

### RestStream (`rest.py`)

**Purpose**: HTTP-like REST API operations over ZMQ.

**Key Features:**
- **REST Semantics**: HTTP-like request/response patterns
- **API Operations**: Standard CRUD operations on FEAGI resources
- **JSON Protocol**: Human-readable message format
- **Modern Interface**: Designed for modern applications and web interfaces

**Port**: 5563  
**Socket Type**: ROUTER  
**Protocol**: REST API message format  

## Enhanced FQ Sampler Integration

### Differentiated Sampling Behavior

The Enhanced FQ Sampler implements **dual-path sampling** optimized for different use cases:

```
Fire Queue Provider
        │
        ▼
Enhanced FQ Sampler
        │
   ┌────┼────┐
   │         │
   ▼         ▼
Visualization Motor
   Path      Path
   │         │
   │         └─► OPU Areas Only
   │            Burst Frequency
   │            Streamlined Data
   │
   └─► All Areas
       Configurable Rates  
       Rich Data Format
   │         │
   ▼         ▼
VisualizationStream  MotorStream
(Port 5562)         (Port 5564)
```

### Automatic Subscriber Management

Both visualization and motor streams implement automatic subscriber detection:

1. **Connection Monitoring**: Track client heartbeats via control stream
2. **Automatic Activation**: Enable FQ sampler when subscribers connect
3. **Resource Conservation**: Disable sampling when no subscribers present
4. **Type-Specific Control**: Independent enable/disable for each stream type

### Performance Characteristics

| Stream Type | Latency Target | Data Volume | Sampling Frequency | Optimization Focus |
|-------------|---------------|-------------|-------------------|-------------------|
| Visualization | 20-50ms | High (All areas) | 1-60Hz configurable | Completeness |
| Motor | 5-10ms | Low (OPU only) | ~100Hz burst-sync | Speed |

## Configuration

### Stream Configuration

```python
# Visualization stream configuration
visualization_config = {
    'auto_enable_on_subscribers': True,
    'subscriber_check_interval': 1.0,
    'include_membrane_potentials': True,
    'include_coordinates': True,
    'include_firing_history': True
}

# Motor stream configuration
motor_config = {
    'auto_enable_on_subscribers': True,
    'subscriber_check_interval': 0.5,  # Faster for real-time
    'motor_timeout_seconds': 10.0,
    'optimize_for_latency': True,
    'include_only_coordinates': True
}
```

### Per-Area FQ Sampler Configuration

```python
# Configure different sampling rates for visualization
cortical_areas = {
    "visual_cortex": {
        "properties": {
            "fq_sample_rate": 30.0,  # High rate for visual analysis
            "cortical_type": "sensory"
        }
    },
    "motor_cortex": {
        "properties": {
            "cortical_type": "OPU",  # Auto-detected for motor sampling
            # Uses burst frequency automatically
        }
    },
    "memory_area": {
        "properties": {
            "fq_sample_rate": 5.0,   # Low rate for memory areas
            "cortical_type": "associative"
        }
    },
    "noise_area": {
        "properties": {
            "fq_sample_rate": 0.0,   # Disable sampling
            "cortical_type": "utility"
        }
    }
}
```

## Testing and Validation

### Test Differentiated Behavior

```python
# Verify that motor and visualization streams receive different data
import zmq
from feagi_bytes import ByteStructureDecoder

ctx = zmq.Context()
decoder = ByteStructureDecoder()

# Connect to both streams
viz_socket = ctx.socket(zmq.SUB)
viz_socket.connect("tcp://localhost:5562")
viz_socket.setsockopt(zmq.SUBSCRIBE, b"activity")

motor_socket = ctx.socket(zmq.SUB)  
motor_socket.connect("tcp://localhost:5564")
motor_socket.setsockopt(zmq.SUBSCRIBE, b"motor")

# Collect and compare data
viz_topic, viz_data = viz_socket.recv_multipart()
motor_topic, motor_data = motor_socket.recv_multipart()

viz_decoded = decoder.decode_neuron_flat(viz_data)
motor_decoded = decoder.decode_neuron_flat(motor_data)

viz_areas = set(viz_decoded['cortical_ids'])
motor_areas = set(motor_decoded['cortical_ids'])

print(f"Visualization areas: {len(viz_areas)}")
print(f"Motor areas: {len(motor_areas)}")
print(f"Motor areas are subset: {motor_areas.issubset(viz_areas)}")
```

## Best Practices

### Client Implementation

1. **Always Send Heartbeats**: Register with control stream for subscriber detection
2. **Handle Disconnections**: Implement proper cleanup when disconnecting
3. **Use Appropriate Stream**: Choose visualization for analysis, motor for control
4. **Monitor Data Flow**: Verify you're receiving expected data types

### Performance Optimization

1. **Configure Sampling Rates**: Set appropriate `fq_sample_rate` for your needs
2. **Use Motor Stream for Control**: Minimize latency with dedicated motor stream
3. **Monitor Subscriber Counts**: Verify automatic activation is working
4. **Handle Backpressure**: Drop data if you can't keep up with the stream

## Related Documentation

- [ZMQ Architecture](../../../docs/arch-zmq.md)
- [NPU Architecture](../../npu/arch-npu.md)
- [FQ Sampler Documentation](../../npu/README.md#fq-sampler)
- [System Architecture](../../../docs/arch-system-overview.md) 