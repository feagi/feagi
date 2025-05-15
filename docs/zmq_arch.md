# ZeroMQ Architecture for FEAGI 2.1

## Overview

This document outlines the ZeroMQ (ZMQ) communication architecture for FEAGI 2.1, detailing the protocols, connection management, message formats, and implementation details required for efficient agent communication.

## Communication Architecture

```
┌───────────────────┐                   ┌───────────────────────────────────────┐
│  External Agent   │                   │                FEAGI                  │
│                   │                   │                                       │
│  ┌─────────────┐  │                   │  ┌─────────────┐  ┌─────────────────┐ │
│  │ ZMQ Client  │◄─┼───────────────────┼─►│ ZMQ Router  │◄─┤ByteStructure    │ │
│  │ (DEALER)    │  │                   │  │ Server      │  │Translator       │ │
│  └─────────────┘  │                   │  └─────────────┘  └─────────┬───────┘ │
│                   │                   │         ▲                   │         │
└───────────────────┘                   │         │                   ▼         │
                                        │  ┌──────────────────────────────────┐ │
                                        │  │           CoreAPIService         │ │
                                        │  └──────────────────────────────────┘ │
                                        └───────────────────────────────────────┘
```

FEAGI uses ZeroMQ as the primary communication protocol between:
- FEAGI core and external agents (robots, sensors, monitors)
- Internal components requiring high-performance messaging
- Visualization systems requiring real-time neural activity data

## FEAGI Communication Protocols

FEAGI 2.1 implements specialized protocols for different communication needs:

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

## Custom Byte Structures Serialization

FEAGI uses custom binary byte structures for serialization across all protocols, offering significant benefits:

1. **Performance Optimization**: Custom binary formats tailored specifically for neural data transmission
2. **Memory Efficiency**: Direct control over memory layout and minimized overhead
3. **Specialized Structures**: Different structures optimized for different data types (JSON, images, neuron potentials)
4. **Compatibility**: Simple header-based approach that works across multiple programming languages
5. **Lightweight**: Minimal external dependencies, reducing deployment complexity

### Byte Structure Format

All byte structures follow a common pattern with a universal header:

```
┌─────────────────┬─────────────────┬─────────────────────────┐
│ Structure Type  │ Version Number  │      Message Data      │
│    (1 byte)     │    (1 byte)     │    (variable size)     │
└─────────────────┴─────────────────┴─────────────────────────┘
```

### Structure Types

1. **JSON (ID: 1)**: For non-performance-critical operations (admin, configuration)
2. **Raw Image (ID: 8)**: Efficient visual data transmission
3. **Multi-Holder (ID: 9)**: Container for multiple structures
4. **Neuron Flat Format (ID: 10)**: Optimized for single-area neuron data
5. **Neuron Categories (ID: 11)**: Optimized for multi-area neuron data

## Protocol Versioning

Each byte structure type has its own versioning to ensure backward compatibility:

### Version Negotiation

1. During agent registration, the agent declares supported structure versions
2. FEAGI registers these capabilities in the client registry
3. When creating messages, FEAGI selects the highest mutually supported version
4. The version byte in each message header identifies the format version

### Version Management

- Version registry tracks supported versions for each structure type:
  ```python
  SUPPORTED_VERSIONS = {
      ByteStructureID.JSON: [1],
      ByteStructureID.RAW_IMAGE: [1],
      ByteStructureID.MULTI_HOLDER: [1],
      ByteStructureID.NEURON_FLAT: [1],
      ByteStructureID.NEURON_CATEGORIES: [1],
  }
  ```
- Version-specific encoder and decoder methods handle different format versions
- New versions can be added without breaking compatibility with older clients

## ROUTER-DEALER Pattern for Multiple Clients

FEAGI uses the ZMQ ROUTER-DEALER pattern to efficiently manage multiple simultaneous clients:

### Server-Side Architecture

1. **Socket Patterns**
   - **ROUTER sockets**: FEAGI binds ROUTER sockets to specific ports for each protocol
     - Control port (FCP): 5559
     - Sensorimotor port (FSMP): 5558
     - Visualization port (FVP): 5560

2. **Client Identity Management**
   - ZMQ's ROUTER socket automatically tracks client identity
   - Each message includes an identity frame that FEAGI uses to route responses
   - Agent ID (from handshake) is mapped to ZMQ identity for application-level tracking

### Client-Side Architecture

1. **Socket Patterns**
   - **DEALER sockets**: Clients connect using DEALER sockets
   - Client identity is automatically managed by ZMQ

2. **Message Flow**
   - Client initiates connection and handshake
   - After handshake, client can send messages using appropriate protocol
   - Server responds using client identity from ROUTER socket

## Connection Management

The `ConnectionManager` class handles all aspects of client connections:

```python
class ConnectionManager:
    def __init__(self, context, control_port, sensorimotor_port, visualization_port):
        # Create ROUTER sockets for each protocol
        self.control_socket = context.socket(zmq.ROUTER)
        self.control_socket.bind(f"tcp://*:{control_port}")
        
        self.sensorimotor_socket = context.socket(zmq.ROUTER)
        self.sensorimotor_socket.bind(f"tcp://*:{sensorimotor_port}")
        
        self.visualization_socket = context.socket(zmq.ROUTER)
        self.visualization_socket.bind(f"tcp://*:{visualization_port}")
        
        # Store client information
        self.connections = {}  # agent_id -> connection_info
    
    def register_client(self, agent_id, zmq_id, supported_protocols):
        """Register a new client connection"""
        self.connections[agent_id] = {
            "zmq_id": zmq_id,
            "protocols": supported_protocols,
            "last_active": time.time(),
            "message_count": {"sent": 0, "received": 0}
        }
    
    async def send_message(self, agent_id, protocol_type, message):
        """Send a message to a specific client"""
        client_info = self.connections.get(agent_id)
        if not client_info:
            return False
            
        zmq_id = client_info["zmq_id"]
        
        if protocol_type == "fcp":
            await self.control_socket.send_multipart([zmq_id, b"", message])
        elif protocol_type == "fsmp":
            await self.sensorimotor_socket.send_multipart([zmq_id, b"", message])
        elif protocol_type == "fvp":
            await self.visualization_socket.send_multipart([zmq_id, b"", message])
            
        return True
```

## Protocol Translation Layer

The ByteStructure Translation Layer converts between binary byte structures and FEAGI's internal data structures:

```python
class ByteStructureTranslator:
    def __init__(self):
        # Initialize encoder and decoder
        self.encoder = ByteStructureEncoder()
        self.decoder = ByteStructureDecoder()
        
        # Default versions and client capabilities registry
        self.default_versions = {
            ByteStructureID.JSON: 1,
            ByteStructureID.NEURON_FLAT: 1,
            # ...other structures
        }
        self.client_capabilities = {}
    
    def decode_message(self, message_data):
        """Decode a byte structure message"""
        try:
            # Check if data is compressed
            if is_compressed(message_data):
                message_data = self.decoder.decompress(message_data)
                
            # Get structure type from header
            structure_id, version = get_structure_info(message_data)
            
            # Decode based on structure type
            if structure_id == ByteStructureID.JSON:
                return self.decoder.decode_json(message_data)
            elif structure_id == ByteStructureID.NEURON_FLAT:
                return {"message_type": "neuron_data", 
                        "data": self.decoder.decode_neuron_flat(message_data)}
            # ...other structure types
        except Exception as e:
            raise ValueError(f"Failed to decode message: {e}")
    
    def create_neuron_data_message(self, cortical_data, client_id=None):
        """Create a neuron data message with appropriate version"""
        # Determine best structure type and version for the data
        if len(cortical_data) > 1:
            structure_id = ByteStructureID.NEURON_CATEGORIES
        else:
            structure_id = ByteStructureID.NEURON_FLAT
            
        # Get appropriate version based on client capabilities
        version = self.get_supported_version(client_id, structure_id)
        
        # Create and return the encoded message
        # ...
```

## Message Handling

The server uses asynchronous message handlers for each protocol:

```python
class MessageHandler:
    def __init__(self, connection_manager, translator, protocol_type):
        self.connection_manager = connection_manager
        self.translator = translator
        self.protocol_type = protocol_type
        
        # Get appropriate socket based on protocol type
        if protocol_type == "fcp":
            self.socket = connection_manager.control_socket
        elif protocol_type == "fsmp":
            self.socket = connection_manager.sensorimotor_socket
        elif protocol_type == "fvp":
            self.socket = connection_manager.visualization_socket
    
    async def _handle_messages(self):
        while self.running:
            # Receive message parts: [client_id, empty_frame, message_data]
            message_parts = await self.socket.recv_multipart()
            client_id, empty, message_data = message_parts
            
            # Look up client by ZMQ identity
            agent_id, client_info = self.connection_manager.get_client_by_zmq_id(client_id)
            
            # Process the message
            decoded_message = self.translator.decode_message(message_data)
            response_data = await self._process_message(agent_id, decoded_message)
            
            # Send response if needed
            if response_data:
                await self.socket.send_multipart([client_id, b"", response_data])
```

## Agent Communication Flow

### Handshake Protocol

The handshake process establishes a connection and negotiates protocol versions:

1. **Hello**: Agent sends hello message with agent ID and type
2. **Welcome**: FEAGI responds with welcome message with server ID
3. **Capabilities**: Agent sends capabilities with supported protocols and structure versions
4. **Configuration**: FEAGI responds with configuration with server settings

```
┌──────────┐                            ┌──────────┐
│  Agent   │                            │  FEAGI   │
└────┬─────┘                            └────┬─────┘
     │                                       │
     │ Hello Message                         │
     │─────────────────────────────────────► │
     │                                       │
     │ Welcome Message                       │
     │◄───────────────────────────────────── │
     │                                       │
     │ Capabilities Message                  │
     │─────────────────────────────────────► │
     │                                       │
     │ Configuration Message                 │
     │◄───────────────────────────────────── │
     │                                       │
```

### Normal Operation

After handshake completion:

1. Agent sends data using appropriate protocol and socket
2. FEAGI processes the message and sends any necessary responses
3. ZMQ automatically handles routing using client identity

## Performance Considerations

1. **Custom Byte Structures**: Optimized binary formats for different data types
2. **Neuron Data Optimization**: Specialized formats for neural potential data
3. **ROUTER-DEALER Pattern**: Enables concurrent handling of multiple clients
4. **Asynchronous Processing**: Non-blocking I/O with asyncio for maximum throughput
5. **Optional Compression**: Deflate compression for bandwidth optimization
6. **Minimal Overhead**: Direct binary encoding with no intermediate representation

## Security Considerations

1. **Authentication**: Implement ZMQ CURVE authentication for encrypted communication
2. **Network Isolation**: Restrict ZMQ connections to specific network interfaces
3. **Input Validation**: Validate all byte structure messages before processing
4. **Resource Limits**: Implement limits on message sizes and connection counts
5. **Client Activity Tracking**: Monitor and clean up inactive connections

## Implementation 

The FEAGI ZMQ architecture consists of the following components:

1. **ZMQRouterServer**: Main server class integrating all components
2. **ConnectionManager**: Handles client connections and message routing
3. **MessageHandlers**: Protocol-specific message handlers
4. **ByteStructureTranslator**: Encodes/decodes messages using byte structures
5. **ByteStructureEncoder/Decoder**: Low-level binary encoding/decoding

The implementation allows for:
- Handling multiple simultaneous clients
- Protocol version negotiation
- Efficient message routing
- Asynchronous message processing
- Client activity monitoring and cleanup

## Implementation Status

1. ✅ Implement custom byte structures for all protocols
2. ✅ Implement ConnectionManager with ROUTER socket support
3. ✅ Implement ByteStructureTranslator, Encoder, and Decoder
4. ✅ Implement MessageHandlers for each protocol
5. ✅ Implement ZMQRouterServer integrating all components
6. ✅ Add version negotiation during handshake
7. ✅ Add monitoring and metrics for connections
8. ⏳ Implement security features
9. ⏳ Develop comprehensive end-to-end tests 