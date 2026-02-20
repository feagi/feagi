# FEAGI Connector

FEAGI Connector is a client-side library that enables agents to connect to FEAGI (Flexible & Extensible Artificial General Intelligence) and exchange data using its communication protocols.

## Features

- **Async API**: Modern async/await patterns for efficient I/O
- **Agent Registration**: Register your agent with FEAGI
- **Sensory Data Transmission**: Send sensory inputs to FEAGI
- **Motor Data Reception**: Receive motor outputs from FEAGI
- **Visualization**: Receive neural activity and brain structure data
- **Connection Management**: Maintain connection with heartbeats
- **ZeroMQ Transport**: Efficient binary communication
- **Rust-compatible**: Designed for future Rust implementation

## Installation

```bash
pip install feagi_connector
```

## Quick Example

```python
import asyncio
from feagi_connector import FeagiClient
from feagi_connector.protocols import FSMPChannel

async def main():
    # Create a client
    client = FeagiClient(host="localhost")
    
    # Connect to FEAGI
    connected = await client.connect()
    if not connected:
        print("Failed to connect to FEAGI")
        return
    
    # Register a callback for motor data
    await client.register_motor_callback(handle_motor_data)
    
    # Send sensory data
    image_data = bytes([0x80] * (10 * 10))  # 10x10 grayscale image
    await client.send_sensory_data(FSMPChannel.VISION, image_data)
    
    # Wait for a while
    await asyncio.sleep(10)
    
    # Disconnect
    await client.disconnect()

def handle_motor_data(channel_id, data):
    print(f"Received motor data on channel {channel_id}: {len(data)} bytes")

if __name__ == "__main__":
    asyncio.run(main())
```

## Documentation Sections

- **[Getting Started](guide-connector-usage.md)**: Learn how to use the FEAGI Connector
- **[ZMQ Communication](guide-zmq-communication.md)**: Understand the ZMQ communication patterns
- **[Architecture Overview](arch-connector-overview.md)**: Explore the connector's architecture
- **[Protocol Specifications](spec-protocols.md)**: Details of the communication protocols
- **[API Reference](api/client.md)**: Complete API documentation
- **[Example Agent](examples/example_agent.md)**: Full example implementation

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the Apache License 2.0. 