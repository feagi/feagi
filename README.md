# FEAGI Connector

**Complete SDK library** for building agents that connect to FEAGI.

## Overview

FEAGI Connector is a complete client-side SDK that provides a simple, high-level API for building agents that connect to FEAGI (Flexible & Extensible Artificial General Intelligence). It handles communication protocols, sensorimotor data processing, device management, and provides extensible frameworks for custom agent development.

**This is a pure library** - it does not contain standalone applications. For complete agent examples and reference implementations, see the `simple_agent` project.

## Features

### Core Communication
- Async API for modern Python applications
- Agent registration and discovery
- Sensory data transmission to FEAGI
- Motor data reception from FEAGI
- Neural activity visualization data
- Connection management with heartbeats
- ZeroMQ transport for efficient communication

### Sensorimotor Processing
- **CapabilitiesManager**: JSON-based device configuration
- **MotorProcessor**: Generic motor command processing with extensible device handlers
- **Connection state management**: Standard connection lifecycle tracking
- **Agent logging**: Structured logging for agents and neuron data

### Performance & Compatibility
- Optional Rust implementations for performance-critical operations
- Designed for future full Rust implementation
- Cross-platform compatibility

## Installation

### From PyPI (when published):
```bash
pip install feagi_connector
```

### Development Installation (current):
```bash
# Install in development mode from local directory
cd feagi_connector
pip install -e .
```

### From Source:
```bash
git clone https://github.com/feagi/feagi_connector
cd feagi_connector
pip install -e .
```

## Quick Usage

```python
from feagi_connector import FeagiClient
from feagi_connector.protocols import FSMPChannel

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

# Disconnect
await client.disconnect()

def handle_motor_data(channel_id, data):
    print(f"Received motor data on channel {channel_id}: {len(data)} bytes")
```

**For complete agent implementations with continuous sensorimotor loops, see the `simple_agent` project.**

## Advanced Usage

### Using Sensorimotor Processing Components

```python
from feagi_connector import (
    FeagiClient,
    CapabilitiesManager,
    MotorProcessor,
    setup_agent_logging
)

# Set up logging
logger, neuron_logger = setup_agent_logging()

# Load device capabilities
capabilities = CapabilitiesManager("capabilities.json")
capabilities.load_capabilities()

# Set up motor processing
motor_processor = MotorProcessor()

# Create FEAGI client
client = FeagiClient(host="localhost", agent_id="my-agent")
await client.connect()

# Register motor callback that uses the motor processor
async def handle_motor_data(channel_id, data):
    # Process motor commands through the generic processor
    await motor_processor.process_motor_commands(data, capabilities)

await client.register_motor_callback(handle_motor_data)
```

### Custom Device Handlers

```python
from feagi_connector import MotorProcessor

motor_processor = MotorProcessor()

# Register custom device handler
async def handle_my_custom_device(device_id: str, config: dict, neuron_data: dict):
    print(f"Custom device {device_id} received: {neuron_data}")
    # Your custom device control logic here

motor_processor.register_device_handler("my_device_type", handle_my_custom_device)
```

## Documentation

For complete usage documentation, see:

- [FEAGI Connector Usage Guide](docs/guide-connector-usage.md) - Comprehensive guide to using the connector
- [API Reference](https://feagi.github.io/feagi_connector) - API reference documentation
- **[Simple Agent Project](../simple_agent/)** - Complete agent examples and reference implementations

## Integration with FEAGI

FEAGI Connector is designed to work with FEAGI's communication protocols:

- **FSMP (FEAGI Sensorimotor Protocol)**: For exchanging sensory and motor data
- **FVP (FEAGI Visualization Protocol)**: For receiving neural activity and structure data
- **FCP (FEAGI Control Protocol)**: For agent registration and control

## Rust Integration

FEAGI Connector provides optional high-performance Rust implementations for computationally intensive operations through the `feagi-rust-py-libs` package.

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