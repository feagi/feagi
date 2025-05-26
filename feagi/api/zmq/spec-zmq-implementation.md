# ZMQ Implementation for FEAGI 2.1

This document summarizes the implementation of the ZeroMQ (ZMQ) communication architecture for FEAGI 2.1, as outlined in the [ZMQ Architecture Document](zmq_arch.md).

## Components Implemented

### 1. Core ZMQ Manager

The `ZMQManager` class in `feagi/api/zmq/manager.py` provides:

- Process-priority-aware socket configurations (Critical, Important, Background)
- Topic-based routing system (control, healthcheck, burst, fcl, motor, sensory, visualization)
- Socket creation with priority-specific settings
- Connection management for agent registration/deregistration
- Port allocation and cleanup

### 2. Protocol Streams

#### Control Stream (FCP)

The `ControlStream` class in `feagi/api/zmq/streams/control.py` implements:

- ROUTER-DEALER pattern for reliable messaging
- Protocol translation between binary and internal formats
- Message handling for agent registration, deregistration, heartbeats, etc.
- Command routing to CoreAPIService

#### Sensorimotor Stream (FSMP)

The `SensorimotorStream` class in `feagi/api/zmq/streams/sensorimotor.py` implements:

- Efficient binary exchange of sensory input and motor output data
- Channel-based communication for different sensory/motor modalities
- Real-time data streaming optimized for low latency

#### Visualization Stream (FVP)

The `VisualizationStream` class in `feagi/api/zmq/streams/visualization.py` implements:

- Neural activity data streaming for visualization
- Brain structure information exchange
- Optimized for larger datasets with appropriate compression

### 3. Binary Protocol Serialization

The `BinarySerializer` class in `feagi/api/protocols/binary.py` implements:

- Binary serialization/deserialization for all protocols
- Protocol header format with protocol ID and version
- Protocol-specific message formats:
  - FCP: command type, message length, payload
  - FVP: frame type, timestamp, data length, payload
  - FSMP: channel ID, timestamp, data length, payload

### 4. ZMQ Server and Client

- `ZmqServer` in `feagi/api/zmq/server.py` manages all ZMQ streams and patterns
- `ZmqClient` in `feagi/api/zmq/client.py` provides client-side connectivity
- Proper event loop management and resource cleanup
- Standalone mode for testing and development

### 5. Communication Patterns

- `RequestReplyManager`: Traditional CRUD operations
- `PubSubManager`: Event broadcasting and subscription
- `PushPullManager`: High-throughput data processing

## Testing

Tests have been implemented for:

- Binary protocol serialization (`tests/api/zmq/test_binary_protocol.py`)
- Control stream functionality (`tests/api/zmq/test_control_stream.py`)

## Usage Examples

### Starting the ZMQ Server

```python
from feagi.api.zmq.server import ZmqServer
from feagi.core import create_core_api

# Create core API
core_api = create_core_api({})

# Create and start ZMQ server
server = ZmqServer(
    core_api=core_api,
    host="0.0.0.0",  # Listen on all interfaces
    req_rep_port=5555,
    pub_sub_port=5556,
    push_pull_port=5557,
    sensorimotor_port=5558,
    control_port=5559,
    vis_base_port=5560
)

# Start the server
server.start()
```

### Connecting as a Client

```python
from feagi.api.zmq.client import ZmqClient

# Create and connect client
client = ZmqClient(
    host="localhost",
    req_port=5555,
    pub_port=5556,
    push_port=5557,
    stream_port=5558
)

# Connect to server
client.connect()

# Subscribe to events
client.subscribe("system.metrics", callback=handle_metrics)

# Send a request
response = client.request("get_status")

# Clean up
client.disconnect()
```

## Future Work

1. **Security**: Implement ZMQ CURVE authentication for encrypted communication
2. **Monitoring**: Add comprehensive metrics for ZMQ connections and performance
3. **Optimization**: Further tune socket options for specific deployment scenarios
4. **Documentation**: Create detailed API documentation for ZMQ components
5. **Integration Testing**: Develop end-to-end tests for the complete ZMQ stack 