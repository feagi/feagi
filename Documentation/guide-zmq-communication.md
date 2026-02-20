# FEAGI ZMQ Communication Guide

This document explains the ZMQ communication patterns used by FEAGI and how the FEAGI Connector library implements them.

## ZMQ Ports and Patterns

FEAGI uses several ZMQ ports, each with a specific communication pattern:

| Port | Pattern | Purpose | Notes |
|------|---------|---------|-------|
| 5555 | REQ/REP | Command API | For commands like health checks, getting status, etc. |
| 5558 | DEALER/ROUTER | Sensorimotor data | For sending sensory data and receiving motor data |
| 5562 | DEALER/ROUTER | Visualization data | For receiving neural activity visualization |

## Command API (REQ/REP Pattern)

The Command API uses the ZMQ REQ/REP pattern on port 5555.

### Message Format

Messages must be formatted as multipart messages:

1. **Request format**: `[auth_token, content_type, request_data]`
   - `auth_token`: Authentication token (empty if none)
   - `content_type`: Content type (e.g., "application/json")
   - `request_data`: JSON-encoded request with `command` field

2. **Response format**: `[content_type, response_data]`
   - `content_type`: Content type (e.g., "application/json")
   - `response_data`: Response data

### Example Request

```python
# Create a REQ socket
socket = context.socket(zmq.REQ)
socket.connect(f"tcp://{host}:5555")

# Create request
request = {
    'command': 'ping',
    'id': 1,
    'timestamp': int(time.time() * 1000),
    'params': {}
}
request_bytes = json.dumps(request).encode('utf-8')

# Send request
socket.send_multipart([
    b"",                            # Auth token (empty)
    b"application/json",            # Content type
    request_bytes                   # Request data
])

# Receive response
response_parts = socket.recv_multipart()
content_type = response_parts[0].decode('utf-8')
response_data = response_parts[1]
```

### Important REQ/REP Considerations

1. **Strict send-receive pattern**: REQ sockets must alternate between sending and receiving
2. **New socket for each request**: Create a new socket for each request to avoid state issues
3. **Timeouts**: Set appropriate timeouts to handle unresponsive servers
4. **Socket closure**: Always close sockets after use to prevent resource leaks

## Sensorimotor API (DEALER/ROUTER Pattern)

The Sensorimotor API uses the ZMQ DEALER/ROUTER pattern on port 5558.

### Message Format

Messages must include an empty delimiter frame:

1. **Agent registration**: `[b"", json_registration_data]`
2. **Sensory data**: `[b"", json_header, binary_data]`
3. **Motor data reception**: `[b"", json_header, binary_data]`

### Agent Registration Process

1. **Hello message**: Agent sends identity and capabilities
   ```python
   hello_msg = {
       "message_type": "hello",
       "agent_id": agent_id,
       "agent_type": agent_type,
       "sensory_channels": [1, 2, 3],  # Supported sensory channels
       "motor_channels": [101, 102]    # Supported motor channels
   }
   
   socket.send_multipart([
       b"",  # Empty delimiter frame
       json.dumps(hello_msg).encode('utf-8')
   ])
   ```

2. **Welcome message**: FEAGI acknowledges the agent
   ```python
   welcome_frames = socket.recv_multipart()
   # Skip empty delimiter frame
   welcome_data = welcome_frames[1]
   welcome_msg = json.loads(welcome_data.decode('utf-8'))
   ```

### Sending Sensory Data

```python
# Prepare header
header = {
    "message_type": "sensory_data",
    "cortical_area": "visual_cortex",
    "timestamp": int(time.time() * 1000)
}

# Send message
socket.send_multipart([
    b"",  # Empty delimiter frame
    json.dumps(header).encode('utf-8'),
    binary_data  # Raw bytes or encoded neuron data
])
```

### Receiving Motor Data

```python
frames = socket.recv_multipart()
# Skip empty delimiter frame
header_data = frames[1]
motor_data = frames[2]

header = json.loads(header_data.decode('utf-8'))
if header.get("message_type") == "motor_data":
    channel_id = header.get("channel_id")
    # Process motor data...
```

## Visualization API (DEALER/ROUTER Pattern)

The Visualization API uses the ZMQ DEALER/ROUTER pattern on port 5562.

### Message Format

Similar to the Sensorimotor API, messages must include an empty delimiter frame:

1. **Registration**: `[b"", json_registration_data]`
2. **Data request**: `[b"", json_request_data]`
3. **Data reception**: `[b"", json_header, binary_data]`

### Requesting Visualization Data

```python
# Request neural activity data
request_msg = {
    "message_type": "activity_request",
    "agent_id": agent_id,
    "timestamp": int(time.time() * 1000)
}

socket.send_multipart([
    b"",  # Empty delimiter frame
    json.dumps(request_msg).encode('utf-8')
])
```

### Receiving Visualization Data

```python
frames = socket.recv_multipart()
# Skip empty delimiter frame
header_data = frames[1]
viz_data = frames[2]

header = json.loads(header_data.decode('utf-8'))
message_type = header.get("message_type")

if message_type == "activity_data":
    # Decode neural activity data
    decoder = ByteStructureDecoder()
    activity = decoder.decode_neuron_data(viz_data)
    # Process activity data...
elif message_type == "structure_data":
    # Decode brain structure data
    structure = json.loads(viz_data.decode('utf-8'))
    # Process structure data...
```

## Common ZMQ Issues and Solutions

### Socket State Errors

**Issue**: "Operation cannot be accomplished in current state"  
**Solution**: Create a new socket for each REQ/REP request

### Timeout Errors

**Issue**: "Resource temporarily unavailable"  
**Solution**: Set appropriate timeouts and handle timeout exceptions

```python
socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5 second timeout
try:
    response = socket.recv_multipart()
except zmq.error.Again:
    # Handle timeout
```

### Message Format Errors

**Issue**: Messages not being received correctly  
**Solution**: Ensure correct message framing, especially the empty delimiter frame for DEALER/ROUTER

### Connection Issues

**Issue**: Cannot connect to FEAGI  
**Solution**: Verify that FEAGI is running and listening on the correct ports

```python
socket.setsockopt(zmq.CONNECT_TIMEOUT, 1000)  # 1 second connect timeout
```

## Best Practices

1. **Use the FEAGI Connector**: The connector library handles all these details for you
2. **Proper error handling**: Always handle exceptions and timeouts
3. **Resource management**: Close sockets when done
4. **Message framing**: Always include the empty delimiter frame for DEALER/ROUTER
5. **Logging**: Log messages at the byte level for debugging 