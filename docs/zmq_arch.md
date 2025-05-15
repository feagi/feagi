# ZeroMQ Architecture for FEAGI 2.1

## Overview

This document outlines the ZeroMQ (ZMQ) communication architecture for FEAGI 2.1, detailing the protocols, connection management, message formats, and implementation details required for efficient agent communication.

## Communication Architecture

```
┌───────────────────┐                   ┌───────────────────────────────────────┐
│  External Agent   │                   │                FEAGI                  │
│                   │                   │                                       │
│  ┌─────────────┐  │                   │  ┌─────────────┐  ┌─────────────────┐ │
│  │ ZMQ Client  │◄─┼───────────────────┼─►│ ZMQ Router  │◄─┤Protocol         │ │
│  │ (DEALER)    │  │                   │  │ Server      │  │Translator       │ │
│  └─────────────┘  │                   │  └─────────────┘  │(Cap'n Proto)    │ │
│                   │                   │         ▲         └─────────┬───────┘ │
└───────────────────┘                   │         │                   │         │
                                        │         ▼                   ▼         │
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

## Serialization with Cap'n Proto

FEAGI uses Cap'n Proto as its serialization format for all protocols, offering significant benefits:

1. **Zero-Copy Deserialization**: Cap'n Proto uses a binary format that doesn't require deserialization, which significantly improves performance.
2. **Faster Serialization**: Cap'n Proto is consistently faster than Protocol Buffers, especially for large data structures.
3. **Memory Efficiency**: The memory representation is the same as the serialized format, reducing memory overhead.
4. **Forward/Backward Compatibility**: Cap'n Proto has robust schema evolution support, allowing for seamless upgrades.
5. **Built-in Schema Validation**: Cap'n Proto schemas enforce type safety and structure validation.

## Protocol Versioning

Each protocol supports versioning to ensure backward compatibility:

### Version Negotiation

1. During agent registration, the agent declares supported protocol versions
2. FEAGI selects the highest compatible version for each protocol
3. This version selection is stored with the agent registration
4. All subsequent messages use the negotiated versions

### Version Management

Protocol implementations follow a plugin architecture:
- Each protocol version is implemented in a separate directory (v1, v2, etc.)
- Schema files are organized by protocol and version
- New versions can be added without modifying existing code

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

The Protocol Translation Layer converts between Cap'n Proto messages and FEAGI's internal data structures:

```python
class ProtocolTranslator:
    def __init__(self, schema_path):
        # Load Cap'n Proto schemas
        self.constants_schema = capnp.load(os.path.join(schema_path, "common/constants.capnp"))
        self.handshake_schema = capnp.load(os.path.join(schema_path, "handshake/v1/handshake.capnp"))
        self.fcp_schema = capnp.load(os.path.join(schema_path, "fcp/v1/fcp.capnp"))
        self.fsmp_schema = capnp.load(os.path.join(schema_path, "fsmp/v1/fsmp.capnp"))
        self.fvp_schema = capnp.load(os.path.join(schema_path, "fvp/v1/fvp.capnp"))
    
    def decode_message(self, message_data, protocol_type):
        """Decode a Cap'n Proto message"""
        if protocol_type == "handshake":
            return self.handshake_schema.HandshakeMessage.from_bytes(message_data)
        elif protocol_type == "fcp":
            return self.fcp_schema.FCPMessage.from_bytes(message_data)
        elif protocol_type == "fsmp":
            return self.fsmp_schema.FSMPMessage.from_bytes(message_data)
        elif protocol_type == "fvp":
            return self.fvp_schema.FVPMessage.from_bytes(message_data)
```

## Message Handling

The server uses asynchronous message handlers for each protocol:

```python
class MessageHandler:
    def __init__(self, connection_manager, schema_loader, protocol_type):
        self.connection_manager = connection_manager
        self.protocol_type = protocol_type
        
        # Get appropriate socket based on protocol type
        if protocol_type == "fcp":
            self.socket = connection_manager.control_socket
        elif protocol_type == "fsmp":
            self.socket = connection_manager.sensorimotor_socket
        elif protocol_type == "fvp":
            self.socket = connection_manager.visualization_socket
            
        # Load protocol schema
        self.schema = schema_loader()
    
    async def _handle_messages(self):
        while self.running:
            # Receive message parts: [client_id, empty_frame, message_data]
            message_parts = await self.socket.recv_multipart()
            client_id, empty, message_data = message_parts
            
            # Look up client by ZMQ identity
            agent_id, client_info = self.connection_manager.get_client_by_zmq_id(client_id)
            
            # Process the message
            decoded_message = self._decode_message(message_data)
            response_data = await self._process_message(agent_id, decoded_message)
            
            # Send response if needed
            if response_data:
                encoded_response = self._encode_response(response_data)
                await self.socket.send_multipart([client_id, b"", encoded_response])
```

## Agent Communication Flow

### Handshake Protocol

The handshake process establishes a connection and negotiates protocol versions:

1. **Hello**: Agent sends `HelloMessage` with agent ID and type
2. **Welcome**: FEAGI responds with `WelcomeMessage` with server ID
3. **Capabilities**: Agent sends `CapabilitiesMessage` with supported protocols
4. **Configuration**: FEAGI responds with `ConfigurationMessage` with server settings

```
┌──────────┐                            ┌──────────┐
│  Agent   │                            │  FEAGI   │
└────┬─────┘                            └────┬─────┘
     │                                       │
     │ HelloMessage                          │
     │─────────────────────────────────────► │
     │                                       │
     │ WelcomeMessage                        │
     │◄───────────────────────────────────── │
     │                                       │
     │ CapabilitiesMessage                   │
     │─────────────────────────────────────► │
     │                                       │
     │ ConfigurationMessage                  │
     │◄───────────────────────────────────── │
     │                                       │
```

### Normal Operation

After handshake completion:

1. Agent sends data using appropriate protocol and socket
2. FEAGI processes the message and sends any necessary responses
3. ZMQ automatically handles routing using client identity

## Performance Considerations

1. **Cap'n Proto Serialization**: Zero-copy deserialization significantly improves performance
2. **ROUTER-DEALER Pattern**: Enables concurrent handling of multiple clients without blocking
3. **Asynchronous Processing**: Non-blocking I/O with asyncio for maximum throughput
4. **High Water Mark**: Set to unlimited (0) to avoid message loss in high-traffic scenarios
5. **Client Identification**: Efficient routing using ZMQ's built-in identity tracking

## Security Considerations

1. **Authentication**: Implement ZMQ CURVE authentication for encrypted communication
2. **Network Isolation**: Restrict ZMQ connections to specific network interfaces
3. **Input Validation**: Validate all Cap'n Proto messages before processing
4. **Resource Limits**: Implement limits on message sizes and connection counts
5. **Client Activity Tracking**: Monitor and clean up inactive connections

## Implementation 

The FEAGI ZMQ architecture consists of the following components:

1. **ZMQRouterServer**: Main server class integrating all components
2. **ConnectionManager**: Handles client connections and message routing
3. **MessageHandler**: Protocol-specific message handlers
4. **ProtocolTranslator**: Loads Cap'n Proto schemas and handles message conversion

The implementation allows for:
- Handling multiple simultaneous clients
- Protocol version negotiation
- Efficient message routing
- Asynchronous message processing
- Client activity monitoring and cleanup

## Implementation Roadmap

1. Implement Cap'n Proto schemas for all protocols
2. Implement ConnectionManager with ROUTER socket support
3. Implement ProtocolTranslator for Cap'n Proto schemas
4. Implement MessageHandlers for each protocol
5. Implement ZMQRouterServer integrating all components
6. Add version negotiation during handshake
7. Add monitoring and metrics for connections
8. Implement security features
9. Develop comprehensive tests 