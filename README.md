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

## Rust Integration

FEAGI Connector provides optional high-performance Rust implementations for computationally intensive operations through the `feagi-data-processing` package.

### Installation with Rust Support

To install FEAGI Connector with Rust support:

```bash
# Install with Rust support
pip install "feagi_connector[rust]"

# Or install full version with all extras
pip install "feagi_connector[full]"
```

### Explicit Implementation Selection

FEAGI Connector uses a fully explicit approach for accessing implementations - Python and Rust implementations are kept separate with clear naming, giving you maximum control and clarity:

```python
# Import Python implementations directly (always available)
from feagi_connector.utils.processing import (
    decode_neuron_potential_xyz_python
)

# Import Rust implementations directly (will raise ImportError if not available)
from feagi_connector.utils.rust_processing import (
    decode_neuron_potential_xyz_rust
)

# Helper for checking availability if needed
from feagi_connector.utils import is_rust_available

# Example function that lets the caller choose which implementation to use
def process_data(data: bytes, use_rust: bool = True):
    if use_rust:
        try:
            # Use Rust implementation
            return decode_neuron_potential_xyz_rust(data)
        except ImportError:
            print("Rust implementation not available, falling back to Python")
            return decode_neuron_potential_xyz_python(data)
    else:
        # Use Python implementation
        return decode_neuron_potential_xyz_python(data)
```

This explicit approach offers several benefits:

1. **Maximum Performance**: No runtime overhead from conditional checks
2. **Code Clarity**: It's always clear which implementation you're using
3. **Full Control**: You decide exactly when to use each implementation
4. **Easier Debugging**: Clear code paths simplify troubleshooting
5. **Simple Error Handling**: ImportErrors are handled where appropriate

The module provides the `is_rust_available()` helper function to check for Rust availability, but it's used only when needed - not on every function call.

### Performance Benefits

The Rust implementation provides significant performance improvements, particularly for:

- Byte structure parsing and manipulation
- Neuron data processing
- Vision data processing

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