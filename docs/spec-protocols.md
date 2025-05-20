# FEAGI Communication Protocol Specifications

This document specifies the communication protocols used by FEAGI and implemented in the FEAGI Connector.

## Overview

FEAGI uses three main communication protocols:

1. **FCP (FEAGI Control Protocol)**: For agent registration, heartbeats, and control
2. **FSMP (FEAGI Sensorimotor Protocol)**: For exchanging sensory and motor data
3. **FVP (FEAGI Visualization Protocol)**: For exchanging brain visualization data

All protocols are implemented over ZeroMQ transport, with specific message formats and patterns.

## FCP (FEAGI Control Protocol)

### Transport

- **Pattern**: REQ/REP
- **Port**: 5555
- **Format**: Multipart messages

### Message Format

#### Request Format

```
[auth_token, content_type, request_data]
```

- `auth_token`: Authentication token (empty if none)
- `content_type`: Content type (e.g., "application/json")
- `request_data`: JSON-encoded request with `command` field

#### Response Format

```
[content_type, response_data]
```

- `content_type`: Content type (e.g., "application/json")
- `response_data`: Response data

### Commands

| Command | Description | Parameters | Response |
|---------|-------------|------------|----------|
| `ping` | Health check | None | `{"status": "ok"}` |
| `get_status` | Get simulation status | None | Status object |
| `get_configuration` | Get FEAGI configuration | None | Configuration object |
| `get_performance` | Get performance metrics | None | Performance metrics object |
| `get_cortical_areas` | Get cortical areas | None | Cortical areas object |
| `start_simulation` | Start the simulation | None | `{"status": "success"}` |
| `stop_simulation` | Stop the simulation | None | `{"status": "success"}` |
| `list_genomes` | List available genomes | None | `{"genomes": [...]}` |
| `load_genome` | Load a genome | `{"name": "genome_name"}` | `{"status": "success"}` |

## FSMP (FEAGI Sensorimotor Protocol)

### Transport

- **Pattern**: DEALER/ROUTER
- **Port**: 5558
- **Format**: Multipart messages with empty delimiter frame

### Message Types

#### Agent Registration

1. **Hello Message**:
   ```
   [b"", json_hello_message]
   ```
   
   Where `json_hello_message` is:
   ```json
   {
     "message_type": "hello",
     "agent_id": "agent-123",
     "agent_type": "vision",
     "sensory_channels": [1, 2, 3],
     "motor_channels": [101, 102]
   }
   ```

2. **Welcome Message**:
   ```
   [b"", json_welcome_message]
   ```
   
   Where `json_welcome_message` is:
   ```json
   {
     "message_type": "welcome",
     "agent_id": "agent-123",
     "status": "ok"
   }
   ```

#### Sensory Data

```
[b"", json_header, binary_data]
```

Where `json_header` is:
```json
{
  "message_type": "sensory_data",
  "cortical_area": "visual_cortex",
  "timestamp": 1234567890
}
```

And `binary_data` is either:
- Raw bytes (e.g., image data)
- Encoded neuron data using feagi_bytes format

#### Motor Data

```
[b"", json_header, binary_data]
```

Where `json_header` is:
```json
{
  "message_type": "motor_data",
  "channel_id": 101,
  "timestamp": 1234567890
}
```

#### Heartbeat

```
[b"", json_heartbeat_message]
```

Where `json_heartbeat_message` is:
```json
{
  "message_type": "heartbeat",
  "agent_id": "agent-123",
  "timestamp": 1234567890
}
```

### Channel IDs

| Channel ID | Name | Type | Description |
|------------|------|------|-------------|
| 1 | VISION | Sensory | Visual input |
| 2 | AUDIO | Sensory | Audio input |
| 3 | TACTILE | Sensory | Touch input |
| 4 | PROPRIOCEPTION | Sensory | Body position |
| 5 | OLFACTORY | Sensory | Smell input |
| 6 | GUSTATORY | Sensory | Taste input |
| 7 | TEXT | Sensory | Text input |
| 101 | MOTOR_ARM | Motor | Arm movement |
| 102 | MOTOR_LEG | Motor | Leg movement |
| 103 | MOTOR_HAND | Motor | Hand movement |
| 104 | MOTOR_SPEECH | Motor | Speech output |
| 105 | MOTOR_EYE | Motor | Eye movement |

## FVP (FEAGI Visualization Protocol)

### Transport

- **Pattern**: DEALER/ROUTER
- **Port**: 5560
- **Format**: Multipart messages with empty delimiter frame

### Message Types

#### Agent Registration

Same as FSMP, but with `agent_type` set to "visualization".

#### Activity Request

```
[b"", json_request_message]
```

Where `json_request_message` is:
```json
{
  "message_type": "activity_request",
  "agent_id": "viz-123",
  "timestamp": 1234567890
}
```

#### Activity Data

```
[b"", json_header, binary_data]
```

Where `json_header` is:
```json
{
  "message_type": "activity_data",
  "timestamp": 1234567890
}
```

And `binary_data` is encoded neuron activity data using feagi_bytes format.

#### Structure Request

```
[b"", json_request_message]
```

Where `json_request_message` is:
```json
{
  "message_type": "structure_request",
  "agent_id": "viz-123",
  "timestamp": 1234567890
}
```

#### Structure Data

```
[b"", json_header, binary_data]
```

Where `json_header` is:
```json
{
  "message_type": "structure_data",
  "timestamp": 1234567890
}
```

And `binary_data` is JSON-encoded brain structure data.

## Binary Data Formats

### Neuron Data

Neuron data is encoded using the feagi_bytes library, which provides:

- `ByteStructureEncoder`: For encoding neuron data
- `ByteStructureDecoder`: For decoding neuron data

The format encodes neuron coordinates and activation values efficiently.

## Error Handling

### Error Messages

Error messages are JSON objects with:

```json
{
  "message_type": "error",
  "error_code": 123,
  "error_message": "Description of the error"
}
```

### Error Codes

| Code | Name | Description |
|------|------|-------------|
| 1 | INVALID_MESSAGE | Message format is invalid |
| 2 | UNKNOWN_AGENT | Agent is not registered |
| 3 | INVALID_CORTICAL_AREA | Cortical area does not exist |
| 4 | INVALID_CHANNEL | Channel ID is not valid |
| 5 | INTERNAL_ERROR | Internal FEAGI error | 