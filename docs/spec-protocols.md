# FEAGI Communication Protocols Specification

*Last Updated: May 15, 2025*

## Overview

This document specifies the communication protocols used by FEAGI for interaction with agents, visualization tools, and internal components. FEAGI implements three primary protocols, each optimized for specific communication needs:

1. **FEAGI Control Protocol (FCP)**: Administrative and management commands
2. **FEAGI Visualization Protocol (FVP)**: Neural activity data for visualization
3. **FEAGI Sensorimotor Protocol (FSMP)**: Sensory input and motor output

## Protocol Architecture

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

All protocols share a common binary message structure with protocol-specific payloads:

```
┌─────────────────┬─────────────┬────────────────┬────────────────────┐
│ Protocol Header │ Message Type│ Message Length │      Payload       │
│    (4 bytes)    │  (1 byte)   │    (4 bytes)   │   (variable len)   │
└─────────────────┴─────────────┴────────────────┴────────────────────┘
```

## 1. FEAGI Control Protocol (FCP)

### Purpose
Administrative and management communication between FEAGI and agents.

### Message Types

| Type ID | Name | Description |
|---------|------|-------------|
| 0x01 | HELLO | Initial connection request from agent |
| 0x02 | WELCOME | Server acknowledgment of connection |
| 0x03 | CAPABILITIES | Agent capabilities and supported protocol versions |
| 0x04 | CONFIG | Configuration data from server |
| 0x05 | REGISTER | Agent registration request |
| 0x06 | REGISTER_RESPONSE | Registration confirmation |
| 0x07 | HEARTBEAT | Connection keep-alive |
| 0x08 | DISCONNECT | Graceful disconnection notice |
| 0x09 | ERROR | Error notification |
| 0x0A | COMMAND | Control command |
| 0x0B | COMMAND_RESPONSE | Command response |
| 0x0C | STATUS | Status update |

### Message Structure

#### HELLO Message
```
┌─────────────────┬─────────────┬────────────────┬───────────────┬──────────────┐
│ Protocol Header │ HELLO (0x01)│ Message Length │   Agent ID    │  Agent Type  │
│    (4 bytes)    │  (1 byte)   │    (4 bytes)   │ (variable len)│(variable len)│
└─────────────────┴─────────────┴────────────────┴───────────────┴──────────────┘
```

#### CAPABILITIES Message
```
┌─────────────────┬─────────────────┬────────────────┬───────────────────────────┐
│ Protocol Header │ CAPABILITIES    │ Message Length │  Supported Protocol       │
│    (4 bytes)    │ (0x03)(1 byte)  │    (4 bytes)   │  Versions (JSON)          │
└─────────────────┴─────────────────┴────────────────┴───────────────────────────┘
```

### Versioning

- Current version: 1.0
- Version negotiation occurs during handshake
- Backward compatibility maintained for one major version

## 2. FEAGI Visualization Protocol (FVP)

### Purpose
Streaming neural activity data for visualization and monitoring.

### Message Types

| Type ID | Name | Description |
|---------|------|-------------|
| 0x01 | ACTIVITY_FRAME | Neural activity snapshot |
| 0x02 | STRUCTURE_DATA | Connectome structure information |
| 0x03 | METRICS | Performance metrics |
| 0x04 | SUBSCRIPTION | Data subscription request |
| 0x05 | SUBSCRIPTION_RESPONSE | Subscription confirmation |

### Message Structure

#### ACTIVITY_FRAME Message
```
┌─────────────────┬─────────────────┬────────────────┬────────────┬────────────────┐
│ Protocol Header │ ACTIVITY_FRAME  │ Message Length │ Timestamp  │ Activity Data  │
│    (4 bytes)    │ (0x01)(1 byte)  │    (4 bytes)   │  (8 bytes) │ (binary array) │
└─────────────────┴─────────────────┴────────────────┴────────────┴────────────────┘
```

### Performance Considerations

- Binary encoding for efficiency
- Optional compression for large datasets
- Configurable frame rates
- Selective area monitoring

## 3. FEAGI Sensorimotor Protocol (FSMP)

### Purpose
High-performance exchange of sensory input and motor output data.

### Message Types

| Type ID | Name | Description |
|---------|------|-------------|
| 0x01 | SENSORY_DATA | Incoming sensory data |
| 0x02 | MOTOR_DATA | Outgoing motor commands |
| 0x03 | CHANNEL_CONFIG | Channel configuration |
| 0x04 | CHANNEL_SUBSCRIBE | Channel subscription |
| 0x05 | PROPRIOCEPTION | Proprioceptive feedback |

### Message Structure

#### SENSORY_DATA Message
```
┌─────────────────┬─────────────────┬────────────────┬────────────┬───────────┬────────────────┐
│ Protocol Header │ SENSORY_DATA    │ Message Length │ Timestamp  │ Channel ID│ Sensory Data   │
│    (4 bytes)    │ (0x01)(1 byte)  │    (4 bytes)   │  (8 bytes) │ (2 bytes) │ (binary array) │
└─────────────────┴─────────────────┴────────────────┴────────────┴───────────┴────────────────┘
```

### Channel Types

| Channel ID Range | Type | Description |
|-----------------|------|-------------|
| 0x0001-0x00FF | Visual | Visual input data |
| 0x0100-0x01FF | Auditory | Audio input data |
| 0x0200-0x02FF | Tactile | Touch/pressure data |
| 0x0300-0x03FF | Motor | Motor output data |
| 0x0400-0x04FF | Proprioceptive | Position/movement feedback |
| 0x0500-0x05FF | Custom | User-defined channels |

## Implementation

### Protocol Translation Layer

The ByteStructure Translation Layer converts between binary byte structures and FEAGI's internal data structures:

```python
class ByteStructureTranslator:
    def __init__(self):
        self.protocol_handlers = {
            ProtocolID.FCP: FCPProtocolHandler(),
            ProtocolID.FVP: FVPProtocolHandler(),
            ProtocolID.FSMP: FSMPProtocolHandler()
        }

    def encode_message(self, protocol_id, message_type, payload):
        """Encode a message into binary format"""
        handler = self.protocol_handlers.get(protocol_id)
        if not handler:
            raise ValueError(f"Unknown protocol ID: {protocol_id}")

        return handler.encode(message_type, payload)

    def decode_message(self, binary_data):
        """Decode binary message into structured format"""
        if len(binary_data) < 5:  # Minimum header size
            raise ValueError("Message too short")

        protocol_id = int.from_bytes(binary_data[0:4], byteorder='big')
        handler = self.protocol_handlers.get(protocol_id)
        if not handler:
            raise ValueError(f"Unknown protocol ID: {protocol_id}")

        return handler.decode(binary_data)
```

### ZMQ Implementation

FEAGI uses ZeroMQ (ZMQ) as the transport layer for all protocols:

- **Pattern**: ROUTER-DEALER for bidirectional communication
- **Sockets**: Separate sockets for each protocol
- **Addressing**: Client identification via ZMQ identities
- **Concurrency**: Asynchronous message handling

### Connection Flow

1. **Handshake**:
   - Agent sends HELLO message with agent ID and type
   - FEAGI responds with WELCOME message
   - Agent sends CAPABILITIES with supported protocol versions
   - FEAGI sends CONFIG with server settings

2. **Registration**:
   - Agent sends REGISTER message with details
   - FEAGI responds with REGISTER_RESPONSE
   - Connection established

3. **Communication**:
   - Agent and FEAGI exchange protocol-specific messages
   - Periodic HEARTBEAT messages maintain connection

4. **Disconnection**:
   - Either side can send DISCONNECT message
   - Resources are cleaned up

## Security Considerations

1. **Authentication**: ZMQ CURVE authentication for encrypted communication
2. **Network Isolation**: Restrict ZMQ connections to specific network interfaces
3. **Input Validation**: Validate all byte structure messages before processing
4. **Resource Limits**: Implement limits on message sizes and connection counts
5. **Client Activity Tracking**: Monitor and clean up inactive connections

## Related Documentation

- [IPC Architecture](arch-ipc.md)
- [API Gateway](../feagi/api/README.md)
- [System Overview](arch-system-overview.md)
