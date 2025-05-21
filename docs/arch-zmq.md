# ZeroMQ Architecture in FEAGI

*Last Updated: May 15, 2025*

## Overview

This document describes the ZeroMQ (ZMQ) architecture in FEAGI, which provides high-performance, asynchronous messaging between FEAGI components and external agents. The architecture is designed to support multiple communication patterns, protocols, and client types while maintaining performance and scalability.

## Architecture Principles

1. **Protocol Separation**: Different communication needs use dedicated protocols and sockets
2. **Asynchronous Processing**: Non-blocking message handling for performance
3. **Connection Management**: Centralized connection tracking and lifecycle management
4. **Binary Efficiency**: Optimized binary protocols for high-throughput data exchange
5. **Versioning**: Protocol versioning for backward compatibility

## Component Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                           FEAGI Core                                │
└───────────────────────────┬─────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         CoreAPIService                              │
└───────────────────────────────┬─────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        ZMQManager                                   │
│                                                                     │
│  ┌─────────────────┐  ┌────────────┐  ┌───────────┐  ┌───────────┐  │
│  │ControlStream    │  │SensoryStream│  │MotorStream│  │VisStream  │  │
│  │(FCP Protocol)   │  │(FSMP Input) │  │(FSMP Out) │  │(FVP)      │  │
│  └────────┬────────┘  └────────┬────┘  └─────┬─────┘  └─────┬─────┘  │
│           │                    │              │              │       │
│  ┌────────┴────────┐  ┌────────┴────┐  ┌─────┴─────┐  ┌─────┴─────┐  │
│  │ ROUTER Socket   │  │ PULL Socket │  │PUB Socket │  │PUB Socket │  │
│  └────────┬────────┘  └────────┬────┘  └─────┬─────┘  └─────┬─────┘  │
└───────────┼─────────────────────┼──────────────┼────────────┼────────┘
            │                     │              │            │
            │                     │              │            │
┌───────────┼─────────────────────┼──────────────┼────────────┼────────┐
│           │                     │              │            │        │
│  ┌────────┴────────┐  ┌────────┴────┐  ┌──────┴──────┐ ┌───┴───────┐ │
│  │ DEALER Socket   │  │ PUSH Socket │  │ SUB Socket  │ │SUB Socket │ │
│  └────────┬────────┘  └─────────┬───┘  └──────┬──────┘ └────┬──────┘ │
│           │                     │             │              │       │
│  ┌────────┴────────┐  ┌─────────┴───┐  ┌──────┴──────┐ ┌────┴──────┐ │
│  │ControlClient    │  │SensoryClient │  │MotorClient  │ │VizClient  │ │
│  │(FCP Protocol)   │  │(FSMP Input)  │  │(FSMP Output)│ │(FVP)      │ │
│  └─────────────────┘  └─────────────┘  └─────────────┘ └───────────┘ │
│                                                                      │
│                         External Agent                               │
└──────────────────────────────────────────────────────────────────────┘
```

## Key Components

### 1. ZMQManager

The central coordinator for all ZMQ communications:

- Manages socket creation and configuration
- Allocates ports for different protocols
- Tracks active connections
- Handles connection lifecycle (creation, monitoring, cleanup)
- Implements security features (authentication, encryption)

```python
class ZMQManager:
    def __init__(self, host="*", control_port=5559, sensory_port=5558, 
                 motor_port=5564, visualization_port=5560):
        self.context = zmq.Context.instance()
        self.host = host
        self.ports = {
            "control": control_port,
            "sensory": sensory_port,
            "motor": motor_port,
            "visualization": visualization_port
        }
        self.connection_manager = ConnectionManager(self.context, **self.ports)
        self.streams = {}
        self._initialize_streams()
```

### 2. Protocol Streams

Specialized stream handlers for each protocol:

#### ControlStream (FCP)
- Handles administrative messages
- Manages agent registration and configuration
- Processes control commands and status updates
- Implements the FEAGI Control Protocol (FCP)

#### SensoryStream (FSMP Input)
- Processes sensory input data from agents to FEAGI
- Optimized for high-frequency, low-latency data reception
- Implements the input part of FEAGI Sensorimotor Protocol (FSMP)

#### MotorStream (FSMP Output)
- Delivers motor output commands from FEAGI to agents
- Optimized for efficient command broadcasting
- Implements the output part of FEAGI Sensorimotor Protocol (FSMP)

#### VisualizationStream (FVP)
- Streams neural activity data
- Provides connectome structure information
- Optimized for larger data payloads
- Implements the FEAGI Visualization Protocol (FVP)

### 3. ConnectionManager

Manages client connections and message routing:

- Tracks active client connections
- Routes messages to appropriate handlers
- Monitors client activity and health
- Cleans up inactive connections

```python
class ConnectionManager:
    def __init__(self, context, control_port, sensory_port, motor_port, visualization_port):
        # Create sockets for each stream
        self.control_socket = context.socket(zmq.ROUTER)
        self.control_socket.bind(f"tcp://*:{control_port}")
        
        self.sensory_socket = context.socket(zmq.PULL)
        self.sensory_socket.bind(f"tcp://*:{sensory_port}")
        
        self.motor_socket = context.socket(zmq.PUB)
        self.motor_socket.bind(f"tcp://*:{motor_port}")
        
        self.visualization_socket = context.socket(zmq.PUB)
        self.visualization_socket.bind(f"tcp://*:{visualization_port}")
        
        # Store client information
        self.connections = {}  # agent_id -> connection_info
```

### 4. MessageHandlers

Protocol-specific message processors:

- Decode incoming messages
- Process messages according to protocol rules
- Generate appropriate responses
- Handle protocol-specific errors

```python
class MessageHandler:
    def __init__(self, connection_manager, translator, protocol_type):
        self.connection_manager = connection_manager
        self.translator = translator
        self.protocol_type = protocol_type
        
        # Get appropriate socket based on protocol type
        if protocol_type == "fcp":
            self.socket = connection_manager.control_socket
        elif protocol_type == "fsmp_sensory":
            self.socket = connection_manager.sensory_socket
        elif protocol_type == "fsmp_motor":
            self.socket = connection_manager.motor_socket
        elif protocol_type == "fvp":
            self.socket = connection_manager.visualization_socket
```

### 5. ByteStructureTranslator

Converts between binary protocol messages and internal data structures:

- Encodes internal data to binary protocol format
- Decodes binary protocol messages to internal format
- Handles protocol versioning and compatibility
- Validates message structure and content

## Communication Patterns

### 1. ROUTER-DEALER Pattern

The primary communication pattern used for all protocols:

- **Server (ROUTER)**: Handles multiple clients with unique identities
- **Client (DEALER)**: Connects to server with persistent identity
- **Asynchronous**: Non-blocking send/receive operations
- **Bidirectional**: Both sides can initiate communication
- **Reliable**: Messages are delivered in order and without loss

### 2. Publish-Subscribe Pattern (Optional)

Used for one-to-many broadcasting of neural activity:

- **Server (PUB)**: Broadcasts activity data to multiple subscribers
- **Client (SUB)**: Subscribes to specific topics/areas of interest
- **Unidirectional**: Server to client only
- **Filtered**: Clients receive only data they subscribe to

## Performance Considerations

### Socket Configuration

- **High Water Mark (HWM)**: Limits queue sizes to prevent memory issues
- **Linger**: Controls socket behavior during shutdown
- **Timeouts**: Prevents blocking operations from hanging indefinitely
- **TCP Keepalive**: Detects dead connections

### Message Processing

- **Asynchronous Handlers**: Non-blocking message processing
- **Thread Pool**: Dedicated threads for CPU-intensive operations
- **Batching**: Combines multiple small messages when appropriate
- **Zero-Copy**: Minimizes data copying for large messages

### Resource Management

- **Context Sharing**: Single ZMQ context across all sockets
- **Connection Pooling**: Reuses connections when possible
- **Graceful Shutdown**: Proper socket cleanup on exit
- **Monitoring**: Tracks socket performance and health

## Security Considerations

### Authentication

- **CURVE**: Elliptic curve cryptography for authentication
- **Certificates**: Public/private key pairs for identity verification
- **Access Control**: Whitelist of authorized clients

### Encryption

- **Transport Encryption**: Secure communication channel
- **Message Encryption**: Optional payload encryption for sensitive data

### Network Security

- **Interface Binding**: Restricts connections to specific network interfaces
- **Port Security**: Firewall rules to protect ZMQ ports
- **Rate Limiting**: Prevents DoS attacks

## Related Documentation

- [Protocol Specification](spec-protocols.md)
- [IPC Architecture](arch-ipc.md)
- [System Overview](arch-system-overview.md) 