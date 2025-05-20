# FEAGI Connector

Client-side integration library for agents connecting to FEAGI.

## Overview

FEAGI Connector provides a simple, high-level API for agents to connect to FEAGI (Flexible & Extensible Artificial General Intelligence) and exchange data using its communication protocols. It abstracts the underlying transport details and provides an async-first interface.

## Features

- Async API for modern Python applications
- Agent registration and discovery
- Sensory data transmission to FEAGI
- Motor data reception from FEAGI
- Neural activity visualization data
- Connection management with heartbeats
- ZeroMQ transport for efficient communication

## Installation

```bash
pip install feagi_connector
```

Or install from source:

```bash
git clone https://github.com/feagi/feagi
cd feagi/feagi_connector
pip install -e .
```

## Quick Usage

```python
import asyncio
from feagi_connector import FeagiClient
from feagi_connector.protocols import FSMPChannel

async def main():
    # Create a client
    client = FeagiClient(
        host="localhost",
        agent_id="my-agent",
        agent_type="example"
    )
    
    # Connect to FEAGI
    connected = await client.connect()
    if not connected:
        print("Failed to connect to FEAGI")
        return
        
    # Register a callback for motor data
    await client.register_motor_callback(handle_motor_data)
    
    # Send some sensory data
    image_data = bytes([0x80] * (10 * 10))  # 10x10 grayscale image
    await client.send_sensory_data(FSMPChannel.VISION, image_data)
    
    # Keep the connection alive for a while
    await asyncio.sleep(10)
    
    # Disconnect
    await client.disconnect()
    
def handle_motor_data(channel_id, data):
    print(f"Received motor data on channel {channel_id}: {len(data)} bytes")

if __name__ == "__main__":
    asyncio.run(main())
```

## Documentation

For complete usage documentation, see:

- [FEAGI Connector Usage Guide](guide-connector-usage.md) - Comprehensive guide to using the connector
- [API Reference](https://feagi.org/docs/api/feagi-connector) - API reference documentation
- [Example Agent](example.py) - Full example of an agent implementation

## Integration with FEAGI

FEAGI Connector is designed to work with FEAGI's communication protocols:

- **FSMP (FEAGI Sensorimotor Protocol)**: For exchanging sensory and motor data
- **FVP (FEAGI Visualization Protocol)**: For receiving neural activity and structure data
- **FCP (FEAGI Control Protocol)**: For agent registration and control

## Development

### Running Tests

```bash
cd feagi_connector
pytest
```

### Building Documentation

```bash
cd feagi_connector
mkdocs build
```

## License

Apache License 2.0

# FEAGI 2.1 ZMQ Communication

This directory contains client implementations for FEAGI 2.1's ZMQ communication patterns.

## ZMQ Communication Patterns in FEAGI 2.1

FEAGI 2.1 uses several ZMQ communication patterns on different ports:

| Port | Pattern | Purpose | Message Format |
|------|---------|---------|----------------|
| 5555 | REQ/REP | Command-based API | `[auth_token, content_type, request_data]` |
| 5558 | DEALER/ROUTER | Sensorimotor data | Binary data with custom framing |
| 5560 | DEALER/ROUTER | Visualization data | Binary data with custom framing |

## Fixed REQ/REP Client

The `feagi_req_rep_fixed.py` file contains a properly implemented REQ/REP client for communicating with FEAGI's command API on port 5555. Key points:

1. REQ/REP sockets **must** follow a strict send-receive pattern
2. Messages must be properly formatted as a 3-part multipart message: `[auth_token, content_type, request_data]`
3. Requests must include a `command` field, such as `ping` or `get_status`
4. The REST-style API paths are mapped to specific commands

## Usage Example

```python
from feagi_req_rep_fixed import FeagiReqRepFixedClient

# Create a client
client = FeagiReqRepFixedClient(host="127.0.0.1", port=5555)

# Check if FEAGI is running
response = client.get('/v1/system/health_check')
print(f"Health check response: {response}")

# Get simulation status
response = client.get('/v1/status')
print(f"Status: {response}")
```

## Supported Commands

The following commands are supported by FEAGI's REQ/REP server:

- `ping` - Health check
- `get_status` - Get simulation status
- `get_configuration` - Get FEAGI configuration
- `get_performance` - Get performance metrics

## Debugging Tips

If you encounter communication issues:

1. Ensure you're using the correct port (5555 for REQ/REP)
2. Make sure each socket only sends once then receives once
3. Use proper multipart message formatting
4. Include the required `command` field in your requests
5. Close sockets after each request to avoid state issues

## Other Communication Channels

For sensory input and visualization data, see the other clients in this directory which use the DEALER/ROUTER pattern. 