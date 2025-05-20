# FEAGI 2.1 ZMQ Communication Guide

This document summarizes the ZMQ communication patterns used in FEAGI 2.1 and how to properly connect to them.

## ZMQ Ports and Patterns

FEAGI 2.1 uses several ZMQ ports, each with a specific communication pattern:

| Port | Pattern | Purpose | Notes |
|------|---------|---------|-------|
| 5555 | REQ/REP | REST API | For REST API calls like health checks, getting genome, etc. |
| 5556 | PUB/SUB | Status updates | For subscribing to FEAGI status updates |
| 5557 | PUSH/PULL | General messages | For one-way messages from FEAGI |
| 5558 | DEALER/ROUTER | Sensorimotor data | For sending sensory data to FEAGI |
| 5560 | DEALER/ROUTER | Visualization data | For receiving neural activity visualization |

## Key Insights

1. **Different socket patterns**: Each port requires a specific ZMQ pattern and message format:
   - Port 5555 (REQ/REP): For command-based API - working correctly
   - Port 5558 (DEALER/ROUTER): For sensorimotor data - connectivity issue
   - Port 5560 (DEALER/ROUTER): For visualization data - not tested

2. **REQ/REP command pattern (port 5555) format**:
   - Messages must be formatted as `[auth_token, content_type, request_data]`
   - The request_data must include a `command` field (e.g., "ping", "get_status", "get_performance")
   - The server responds with `[content_type, response_data]`
   - Working commands include: "ping", "get_status", "get_performance"

3. **DEALER/ROUTER pattern issues**:
   - Port 5558 is listening but not responding to handshake messages
   - This may indicate that the FEAGI sensorimotor server component is not fully activated
   - The empty delimiter frame is required in messages: `[b"", json_data, binary_data]`

## Recommended Solutions

1. **For REQ/REP (port 5555) - Command API**:
   - Use the `FeagiCommandClient` from `feagi_zmq_client.py` for reliable communication
   - Create a new socket for each request to avoid state issues
   - Always format messages correctly with auth_token and content_type frames
   - Include the required command field in your request

2. **For DEALER/ROUTER (port 5558) - Sensorimotor Data**:
   - Validate FEAGI's configuration to ensure the sensorimotor server is active
   - Try restarting FEAGI with explicit sensorimotor port settings
   - Ensure all messages include the empty delimiter frame
   - Follow the correct handshake protocol (hello → welcome → capabilities → ack)

3. **For both patterns**:
   - Use proper error handling and timeouts
   - Close sockets properly after use
   - Log messages at the byte level for debugging

## Next Steps

1. Consult the FEAGI documentation on ZMQ server configuration
2. Check if there are specific startup flags needed for the sensorimotor server
3. Consider using the REST API for functions that are critical, as it appears more stable
4. Implement a fallback mechanism to use REST API when ZMQ fails

## Common Errors and Solutions

| Error | Cause | Solution |
|-------|-------|----------|
| "Unknown structure type: 123" | Incorrect binary encoding of sensory data | Use JSON or proper feagi_bytes encoding |
| "Operation cannot be accomplished in current state" | REQ socket used more than once without receiving | Create new socket for each request |
| "Resource temporarily unavailable" | Timeout waiting for response | Check that server is running and port is correct |
| "Raw application/json response" | FEAGI responding with content-type header | Parse raw response bytes |

## Client Implementations

We've provided three client implementations in `zmq_client.py`:

1. `FeagiReqRepClient`: For making REST API calls on port 5555
2. `FeagiSensoryClient`: For sending sensory data on port 5558
3. `FeagiVizClient`: For receiving visualization data on port 5560

### Example Usage

```python
# REST API client
rest_client = FeagiReqRepClient(host="127.0.0.1", port=5555)
status = rest_client.get('/v1/status')
print(f"FEAGI status: {status}")

# Sensory client
sensory_client = FeagiSensoryClient(host="127.0.0.1", port=5558)
sensory_client.connect()
sensory_client.send_sensory_data([[1, 2, 0], [3, 4, 0]], cortical_id="iv00CC")
sensory_client.disconnect()
```

## Lesson Learned

The primary lesson from this investigation is that ZMQ socket patterns must be strictly followed:

1. **Proper socket types**: Use the correct socket type for each port/pattern
2. **Correct message framing**: Include empty delimiter frames for DEALER/ROUTER
3. **Socket state management**: REQ/REP sockets must alternate between sending and receiving
4. **Data formatting**: Use the expected data format for each port/connection
5. **Socket lifecycle**: Create new sockets when needed to avoid state errors

By following these guidelines, you can successfully communicate with FEAGI 2.1 using ZMQ. 