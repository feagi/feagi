# ZeroMQ Architecture for FEAGI 2.1

## Overview

This document outlines the ZeroMQ (ZMQ) communication architecture for FEAGI 2.1, detailing the protocols, connection management, message formats, and implementation details required for efficient agent communication.

## Communication Architecture

```
┌───────────────────┐                   ┌───────────────────────────────────────┐
│  External Agent   │                   │                FEAGI                  │
│                   │                   │                                       │
│  ┌─────────────┐  │                   │  ┌─────────┐    ┌───────────────┐    │
│  │ ZMQ Client  │◄─┼───────────────────┼─►│   API   │◄──►│ Protocol      │    │
│  └─────────────┘  │                   │  │ Gateway │    │ Translator    │    │
│                   │                   │  └────┬────┘    └──────┬────────┘    │
└───────────────────┘                   │       │               │             │
                                        │       │               │             │
                                        │       ▼               ▼             │
                                        │  ┌────────────────────────────────┐ │
                                        │  │         CoreAPIService         │ │
                                        │  └────────────────────────────────┘ │
                                        └───────────────────────────────────────┘
```

FEAGI uses ZeroMQ as the primary communication protocol between:
- FEAGI core and external agents (robots, sensors, monitors)
- Internal components requiring high-performance messaging
- Visualization systems requiring real-time neural activity data

## FEAGI Communication Protocols

FEAGI 2.1 implements three specialized binary protocols for different communication needs:

### 1. FEAGI Control Protocol (FCP)

- **Purpose**: Administrative and management commands
- **Scope**: Agent registration, configuration, status updates
- **Data Types**: Commands, responses, configuration properties
- **Direction**: Bidirectional
- **Characteristics**: Low frequency, high reliability

### 2. FEAGI Visualization Protocol (FVP)

- **Purpose**: Neural activity data for visualization
- **Scope**: Brain state, neural activity, connectivity visualization
- **Data Types**: Activation patterns, FCL data, structural information
- **Direction**: Primarily FEAGI → Agent
- **Characteristics**: High frequency, optimized for large datasets

### 3. FEAGI Sensorimotor Protocol (FSMP)

- **Purpose**: Sensory input and motor output
- **Scope**: Sensor data, motor commands, proprioceptive feedback
- **Data Types**: Binary sensory arrays, motor activation patterns
- **Direction**: Bidirectional (sensory: Agent → FEAGI, motor: FEAGI → Agent)
- **Characteristics**: Real-time, low latency, optimized for streaming

## Protocol Versioning

Each protocol supports versioning to ensure backward compatibility:

```
┌─────────────────────────────────────────────────────────────┐
│                     Message Format                          │
├───────────┬───────────┬────────────────────────────────────┐
│ Protocol  │ Version   │                                    │
│ Identifier│ Number    │         Protocol Payload           │
│ (1 byte)  │ (1 byte)  │                                    │
├───────────┴───────────┴────────────────────────────────────┤
│                                                            │
│                  Protocol-Specific Data                    │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

### Version Negotiation

1. During agent registration, the agent declares supported protocol versions
2. FEAGI selects the highest compatible version for each protocol
3. This version selection is stored with the agent registration
4. All subsequent messages use the negotiated versions

### Version Management

Protocol implementations follow a plugin architecture:
- Each protocol version is implemented as a separate class
- A protocol factory creates the appropriate version handler
- New versions can be added without modifying existing code

## API Gateway

The API Gateway serves as the entry point for all ZMQ communication:

1. **Responsibilities**:
   - Message routing
   - Protocol translation
   - Authentication and authorization
   - Rate limiting and throttling
   - Connection management

2. **Flow**:
   - Incoming: ZMQ → Gateway → Protocol Translation → CoreAPIService
   - Outgoing: CoreAPIService → Protocol Translation → Gateway → ZMQ

## Protocol Translation Layer

The Protocol Translation Layer converts between binary protocol messages and FEAGI's internal data structures:

```python
class ProtocolTranslator:
    def __init__(self):
        self.translators = {
            'FCP': self._create_protocol_handler('FCP'),
            'FVP': self._create_protocol_handler('FVP'),
            'FSMP': self._create_protocol_handler('FSMP')
        }
    
    def _create_protocol_handler(self, protocol_type):
        return {
            '1': self._get_handler_class(protocol_type, '1'),
            '2': self._get_handler_class(protocol_type, '2'),
            # Add new versions as they're developed
        }
    
    def decode(self, binary_data):
        """Convert binary protocol data to internal FEAGI format"""
        protocol_id = binary_data[0]
        version = binary_data[1]
        payload = binary_data[2:]
        
        protocol_type = self._get_protocol_type(protocol_id)
        translator = self.translators[protocol_type][version]
        return translator.decode(payload)
    
    def encode(self, data, protocol_type, version):
        """Convert internal FEAGI data to binary protocol format"""
        translator = self.translators[protocol_type][version]
        payload = translator.encode(data)
        
        protocol_id = self._get_protocol_id(protocol_type)
        return bytes([protocol_id, int(version)]) + payload
```

## Topic Structure

FEAGI 2.1 uses a simple topic structure with five primary communication channels that correspond to the protocols:

1. **Control** (FCP)
   - Purpose: Command and control messages between FEAGI and agents
   - Examples: Start/stop commands, configuration changes
   - Direction: Bidirectional (FEAGI → Agent, Agent → FEAGI)

2. **Healthcheck/Status** (FCP)
   - Purpose: Monitoring system health and connectivity
   - Examples: Heartbeats, resource utilization, error reports
   - Direction: Bidirectional

3. **Motor** (FSMP)
   - Purpose: Neural activity related to motor functions
   - Examples: Movement commands, actuator signals
   - Direction: Primarily FEAGI → Agent

4. **Sensory** (FSMP)
   - Purpose: Incoming sensory data from agents
   - Examples: Camera input, microphone data, touch sensors
   - Direction: Primarily Agent → FEAGI

5. **Visualization** (FVP)
   - Purpose: Data for visualizing brain activity
   - Examples: Neuron firing patterns, connection strengths
   - Direction: Primarily FEAGI → Agent (monitoring tools)

## Protocol Details

### 1. FCP (FEAGI Control Protocol)

Binary structure for version 1:
```
┌───────────┬───────────┬───────────┬────────────┬─────────────────┐
│ Protocol  │ Version   │ Command   │ Message    │                 │
│ ID (0x01) │ (0x01)    │ Type      │ Length     │ Message Payload │
│ 1 byte    │ 1 byte    │ 1 byte    │ 4 bytes    │ Variable        │
└───────────┴───────────┴───────────┴────────────┴─────────────────┘
```

Command types:
- 0x01: Register
- 0x02: Deregister
- 0x03: Configure
- 0x04: Status Request
- 0x05: Status Response
- 0x06: Heartbeat
- ...

### 2. FVP (FEAGI Visualization Protocol)

Binary structure for version 1:
```
┌───────────┬───────────┬───────────┬────────────┬────────┬─────────────────┐
│ Protocol  │ Version   │ Frame     │ Timestamp  │ Data   │                 │
│ ID (0x02) │ (0x01)    │ Type      │ (ms)       │ Length │ Data Payload    │
│ 1 byte    │ 1 byte    │ 1 byte    │ 8 bytes    │ 4 bytes│ Variable        │
└───────────┴───────────┴───────────┴────────────┴────────┴─────────────────┘
```

Frame types:
- 0x01: Neuron Activations
- 0x02: Connection Strengths
- 0x03: Area Summary
- 0x04: Global Stats
- ...

### 3. FSMP (FEAGI Sensorimotor Protocol)

Binary structure for version 1:
```
┌───────────┬───────────┬───────────┬────────────┬────────┬─────────────────┐
│ Protocol  │ Version   │ Channel   │ Timestamp  │ Data   │                 │
│ ID (0x03) │ (0x01)    │ ID        │ (ms)       │ Length │ Data Payload    │
│ 1 byte    │ 1 byte    │ 2 bytes   │ 8 bytes    │ 4 bytes│ Variable        │
└───────────┴───────────┴───────────┴────────────┴────────┴─────────────────┘
```

Channel organization:
- 0x0000-0x7FFF: Sensory channels
- 0x8000-0xFFFF: Motor channels

## Connection Management

### ZMQ Socket Factory

FEAGI 2.1 implements a ZMQConnectionFactory to centralize connection management:

```python
class ZMQConnectionFactory:
    def create_publisher(self, address: str, bind: bool = True) -> ZMQPublisher:
        """Create a ZMQ publisher socket"""
        pass
        
    def create_subscriber(self, address: str, topics: List[str] = None, bind: bool = False) -> ZMQSubscriber:
        """Create a ZMQ subscriber socket"""
        pass
        
    def create_req_socket(self, address: str, bind: bool = False) -> ZMQRequester:
        """Create a ZMQ REQ socket"""
        pass
        
    def create_rep_socket(self, address: str, bind: bool = True) -> ZMQReplier:
        """Create a ZMQ REP socket"""
        pass
```

### Connection Lifecycle

1. **Initialization**: ZMQ Context is created during FEAGI startup
2. **Creation**: Sockets are created on-demand via the factory
3. **Monitoring**: Active connections are tracked and monitored
4. **Termination**: Connections are explicitly closed when agents deregister
5. **Cleanup**: ZMQ Context is terminated during FEAGI shutdown

## Socket Types

### ZMQPublisher

```python
class ZMQPublisher:
    def __init__(self, context: zmq.Context, address: str, bind: bool = True):
        self.socket = context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 0)  # Unlimited queue
        if bind:
            self.socket.bind(address)
        else:
            self.socket.connect(address)
            
    def send(self, message: Any, topic: str = "") -> None:
        """Send a message to all subscribers of a topic"""
        compressed_data = self._compress(message)
        self.socket.send_multipart([topic.encode('utf-8'), compressed_data])
        
    def close(self) -> None:
        """Close the socket"""
        self.socket.close()
        
    def _compress(self, data: Any) -> bytes:
        """Compress data using lz4"""
        serialized = pickle.dumps(data)
        return lz4.frame.compress(serialized)
```

### ZMQSubscriber

```python
class ZMQSubscriber:
    def __init__(self, context: zmq.Context, address: str, topics: List[str] = None, bind: bool = False):
        self.socket = context.socket(zmq.SUB)
        
        # Subscribe to specified topics or all if none specified
        if not topics:
            self.socket.setsockopt(zmq.SUBSCRIBE, b'')  # Subscribe to all topics
        else:
            for topic in topics:
                self.socket.setsockopt(zmq.SUBSCRIBE, topic.encode('utf-8'))
        
        self.socket.setsockopt(zmq.CONFLATE, 1)  # Only keep latest message
        if bind:
            self.socket.bind(address)
        else:
            self.socket.connect(address)
            
    def receive(self, timeout: int = 0) -> Optional[Tuple[str, Any]]:
        """Receive a message from publisher"""
        try:
            if timeout > 0:
                if self.socket.poll(timeout) == 0:
                    return None
                    
            topic_bytes, data = self.socket.recv_multipart(flags=zmq.NOBLOCK)
            topic = topic_bytes.decode('utf-8')
            message = self._decompress(data)
            return (topic, message)
            
        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                return None
            raise
            
    def close(self) -> None:
        """Close the socket"""
        self.socket.close()
        
    def _decompress(self, data: bytes) -> Any:
        """Decompress data using lz4"""
        decompressed = lz4.frame.decompress(data)
        return pickle.loads(decompressed)
```

## Agent Communication Flow

### Registration with Protocol Negotiation

1. Agent connects via REST API to register
2. Agent declares supported protocol versions for FCP, FVP, and FSMP
3. FEAGI selects compatible protocol versions
4. FEAGI assigns communication parameters (port, address)
5. ZMQ connections are established using the assigned parameters
6. Protocol translators are initialized with negotiated versions
7. Agent begins regular communication

### Normal Operation

1. Agent sends sensor data via ZMQ using the "sensory" topic with FSMP protocol
2. Gateway translates binary FSMP to internal data structures
3. CoreAPIService processes the data through neural network
4. CoreAPIService sends response via internal API
5. Gateway translates internal data to binary FCP, FVP or FSMP as appropriate
6. Data is sent back to agent via ZMQ with appropriate topic

### Deregistration

1. Agent requests deregistration via REST API or FCP protocol
2. FEAGI closes and cleans up ZMQ connections
3. Agent is removed from registry

## Port Management

FEAGI 2.1 implements a port manager to handle port allocation:

```python
class ZMQPortManager:
    def __init__(self, min_port: int = 40001, max_port: int = 40050):
        self.min_port = min_port
        self.max_port = max_port
        self.used_ports = set()
        
    def get_available_port(self) -> int:
        """Get an available port in the range"""
        for port in range(self.min_port, self.max_port + 1):
            if port not in self.used_ports:
                self.used_ports.add(port)
                return port
        raise RuntimeError("No available ports")
        
    def release_port(self, port: int) -> None:
        """Release a used port"""
        if port in self.used_ports:
            self.used_ports.remove(port)
```

## Performance Considerations

1. **Binary Protocols**: Custom binary protocols significantly reduce overhead compared to text-based formats
2. **Message Compression**: Compressed protocol reduces network bandwidth requirements
3. **High Water Mark**: Set to unlimited (0) to avoid message loss
4. **Conflation**: Enabled for subscribers to only process latest messages when overwhelmed
5. **Topic Filtering**: Used for efficient message routing
6. **Asynchronous Processing**: Prevents blocking on I/O operations

## Security Considerations

1. **Authentication**: Implement ZMQ CURVE authentication for encrypted communication
2. **Network Isolation**: Restrict ZMQ connections to specific network interfaces
3. **Input Validation**: Validate all binary messages before processing
4. **Resource Limits**: Implement limits on message sizes and connection counts
5. **Protocol Validation**: Verify protocol integrity before processing messages

## Integration with CoreAPIService

The CoreAPIService will manage ZMQ connections through a dedicated ZMQManager:

```python
class CoreAPIService:
    def __init__(self, connectome_manager, state_manager):
        # ...
        self._zmq_manager = ZMQManager()
        self._protocol_translator = ProtocolTranslator()
        
    def register_agent(self, agent_id, agent_type, protocol_versions, ...):
        # Determine compatible protocol versions
        compatible_versions = self._determine_compatible_versions(protocol_versions)
        
        # Create ZMQ connection
        zmq_connection = self._zmq_manager.create_connection(
            agent_id=agent_id,
            agent_type=agent_type,
            address=agent_router_address,
            protocols=compatible_versions
        )
        
        # Store connection and protocol information in agent registry
        # ...
```

## Implementation Roadmap

1. Define binary protocol specifications for FCP, FVP, and FSMP
2. Implement protocol translators with versioning support
3. Develop API Gateway with routing and protocol translation
4. Implement ZMQConnectionFactory and core socket wrapper classes
5. Implement ZMQPortManager for port allocation
6. Integrate ZMQ connection management with agent registration/deregistration
7. Add protocol version negotiation during registration
8. Add monitoring and metrics for ZMQ connections
9. Implement security features
10. Develop comprehensive tests for ZMQ communication and protocol handling 