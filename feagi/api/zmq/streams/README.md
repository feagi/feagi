# FEAGI ZMQ Streams

*Last Updated: May 24, 2025*

## Overview

This directory contains the ZeroMQ stream implementations for FEAGI, providing specialized data channels optimized for different use cases. A key innovation is the **differentiated FQ sampler integration** that provides optimized data delivery for different subscriber types.

## Recent Major Updates

### VisualizationStream Enhanced Threading Implementation

The `VisualizationStream` has been **completely rewritten** with a threading-based architecture for optimal RTOS compatibility and reliability. Key improvements include:

**🔧 Threading-Based Architecture:**
- Replaced async-based implementation with synchronous threading approach
- Eliminates async/sync context conflicts that caused previous issues
- RTOS-compatible design for future migration

**👥 Enhanced Client Tracking:**
- Real-time heartbeat monitoring with 30-second timeout
- Thread-safe client management with proper locking
- Automatic cleanup of disconnected clients

**🔄 Automatic FQ Sampler Control:**
- Enables FQ sampler automatically when visualization clients connect
- Disables FQ sampler when no clients present (resource conservation)
- Independent control per stream type

**⚡ Responsive Shutdown:**
- Improved thread management with 5-second timeout per thread
- Replaces `time.sleep()` with `Event.wait()` for faster response
- Frequent stop signal checks (200-250ms intervals)

**🛡️ Enhanced Error Handling:**
- ZMQ socket corruption detection and automatic recreation
- Graceful handling of malformed data
- Production-ready logging with appropriate levels

## Stream Architecture

### Differentiated Data Delivery

FEAGI implements **dual-path FQ sampling** to optimize data delivery:

- **Visualization Path**: Comprehensive brain state monitoring with all cortical areas
- **Motor Path**: Real-time motor control with OPU areas only

## Stream Implementations

### VisualizationStream (`visualization.py`)

**Purpose**: Real-time neural activity broadcasting for brain visualization and analysis.

**Key Features:**
- **Threading-Based Implementation**: Reliable RTOS-compatible design
- **Enhanced Client Tracking**: Real-time heartbeat monitoring and timeout management
- **Comprehensive Sampling**: Receives data from ALL cortical areas
- **Configurable Rates**: Respects per-area `fq_sample_rate` properties
- **Rich Data Format**: Full neuron state including membrane potentials, thresholds, coordinates
- **Automatic Subscriber Detection**: Enables FQ sampler when visualization clients connect
- **Standby Mode**: Intelligently pauses processing when genome not loaded
- **Production Logging**: Clean, structured logging suitable for production environments

**Port**: 5562  
**Socket Type**: PUB (Publisher)  
**Protocol**: feagi_bytes binary format  
**Threading Model**: 3 worker threads (data processing, client cleanup, subscriber monitoring)

```python
# Enhanced client connection example with heartbeat
import zmq
import time
import threading

def visualization_client_example():
    # Set up data subscription
    context = zmq.Context()
    data_socket = context.socket(zmq.SUB)
    data_socket.connect("tcp://localhost:5562")
    data_socket.setsockopt(zmq.SUBSCRIBE, b"activity")
    
    # Set up heartbeat via REST API (recommended approach)
    import requests
    client_id = "my_visualization_client"
    
    def send_heartbeat():
        while True:
            try:
                response = requests.post(
                    "http://localhost:8000/v1/visualization/heartbeat",
                    json={"client_id": client_id}
                )
                print(f"Heartbeat response: {response.status_code}")
                time.sleep(5)  # Send every 5 seconds
            except Exception as e:
                print(f"Heartbeat error: {e}")
                time.sleep(5)
    
    # Start heartbeat thread
    heartbeat_thread = threading.Thread(target=send_heartbeat, daemon=True)
    heartbeat_thread.start()
    
    # Process visualization data
    while True:
        try:
            topic, data = data_socket.recv_multipart()
            if topic == b"activity":
                # Process neural activity data
                print(f"Received {len(data)} bytes of neural data")
        except KeyboardInterrupt:
            break
    
    context.term()
```

**Client Lifecycle Management:**
1. **Connection**: Client connects to visualization port
2. **Registration**: Send initial heartbeat via REST API (`POST /v1/visualization/heartbeat`)
3. **Automatic FQ Enablement**: FQ sampler enabled when first client connects
4. **Heartbeat Maintenance**: Send heartbeat every 5-15 seconds (timeout = 30 seconds)
5. **Automatic Cleanup**: Client removed after 30 seconds without heartbeat
6. **Automatic FQ Disablement**: FQ sampler disabled when last client disconnects

**Thread Architecture:**
- **Main Thread**: Socket management and initialization
- **FQ Data Worker**: Processes fire queue data and publishes to ZMQ
- **Client Cleanup Worker**: Monitors heartbeat timeouts and removes inactive clients
- **Subscriber Monitor Worker**: Controls FQ sampler based on active client count

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
- **Visualization Heartbeat Endpoint**: `POST /v1/visualization/heartbeat`

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

1. **Connection Monitoring**: Track client heartbeats via REST API
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
    'motor_latency_target_ms': 10,
    'opu_detection_patterns': ['opu_', 'motor_', 'output_']
}
```

### TOML Configuration Example

```toml
[feagi.api.zmq.streams]
enabled = ["visualization", "motor", "sensory", "control", "rest"]

[feagi.api.zmq.streams.visualization]
port = 5562
auto_enable_on_subscribers = true
subscriber_check_interval = 1.0
client_heartbeat_timeout = 30

[feagi.api.zmq.streams.motor]
port = 5564
auto_enable_on_subscribers = true
subscriber_check_interval = 0.5
motor_latency_target_ms = 10
```

## Threading and RTOS Compatibility

### Design Principles

The threading-based approach follows these principles:

1. **Synchronous Operations**: No async/await complexity
2. **Dedicated Threads**: Each responsibility gets its own thread
3. **Responsive Shutdown**: Fast response to stop signals
4. **Thread Safety**: Proper locking for shared data
5. **Resource Management**: Clean startup and shutdown

### Thread Management

```python
# Example of responsive thread pattern used throughout
def worker_thread(self):
    while self.running and not self._stop_event.is_set():
        try:
            # Quick exit check
            if self._stop_event.is_set():
                break
            
            # Do work with short timeouts
            data = self.queue.get(timeout=0.05)
            self.process_data(data)
            
        except Empty:
            # Use wait() instead of sleep() for responsive shutdown
            if self._stop_event.wait(timeout=0.2):
                break
            continue
        except Exception as e:
            self.logger.error(f"Worker error: {e}")
            # Brief pause on error, still responsive
            if self._stop_event.wait(timeout=0.1):
                break
```

## Migration and Compatibility

### Backward Compatibility

The new VisualizationStream maintains full API compatibility:

- All public methods preserved
- Same initialization parameters
- Compatible with existing client code
- Legacy tuple and dict data formats supported

### Migration Notes

**From Previous VisualizationStream:**
- No client code changes required
- Improved reliability and performance
- Better error handling and logging
- Responsive shutdown (no more hanging)

**Key Behavioral Changes:**
- Heartbeat tracking now functional (previously stubbed)
- FQ sampler auto-control now working
- Production-ready logging levels
- Thread-safe client management

## Troubleshooting

### Common Issues

**FQ Sampler Not Enabling:**
- Verify clients send heartbeats via `POST /v1/visualization/heartbeat`
- Check logs for client connection messages
- Ensure `auto_enable_on_subscribers` is True

**Slow Shutdown:**
- Updated implementation should shutdown in less than 2 seconds
- If hanging, check for blocking operations in custom code
- Monitor thread join timeouts in logs

**Missing Data:**
- Verify genome is loaded (stream enters standby mode otherwise)
- Check FQ sampler queue connection
- Monitor queue size and processing logs

**Client Timeout Issues:**
- Default timeout is 30 seconds
- Ensure heartbeat interval < 30 seconds
- Check network connectivity and REST API access

### Debug Logging

Enable debug logging for detailed information:

```python
import logging
logging.getLogger('feagi.api.zmq.streams.visualization').setLevel(logging.DEBUG)
```

## Performance Monitoring

### Statistics

All streams provide runtime statistics:

```python
# Get visualization stream stats
stats = viz_stream.get_stats()
print(f"Messages sent: {stats['data_sent']}")
print(f"Bytes sent: {stats['bytes_sent']}")
print(f"Message rate: {stats['messages_per_second']:.2f} msg/s")
print(f"Connected clients: {viz_stream.get_connected_client_count()}")
```

### Monitoring Endpoints

- **Stream Status**: `GET /v1/system/zmq_status`
- **Client Count**: Available via stream statistics
- **Performance Metrics**: Built into each stream implementation

## Related Documentation

- [FQ Sampler Documentation](../../npu/README.md#fq-sampler)
- [Protocol Documentation](../protocols/README.md)
- [Core API Documentation](../core/README.md)
- [ZMQ Server Documentation](../README.md)

## Testing

Comprehensive test coverage includes:

- **Unit Tests**: `tests/api/zmq/test_visualization_stream.py`
- **Integration Tests**: `tests/api/zmq/test_rest_integration.py`
- **Performance Tests**: `tests/performance/test_zmq_streams.py`
- **Thread Safety Tests**: Multi-threaded client simulation 