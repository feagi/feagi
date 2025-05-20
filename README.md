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
- Designed for future Rust implementation

## Installation

```bash
pip install feagi_connector
```

Or install from source:

```bash
git clone https://github.com/feagi/feagi_connector
cd feagi_connector
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

- [FEAGI Connector Usage Guide](docs/guide-connector-usage.md) - Comprehensive guide to using the connector
- [API Reference](https://feagi.github.io/feagi_connector) - API reference documentation
- [Example Agent](examples/example_agent.py) - Full example of an agent implementation

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