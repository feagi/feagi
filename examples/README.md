# FEAGI Connector Examples

**Note: FEAGI Connector is now a pure SDK library. For complete agent examples and reference implementations, please see the `simple_agent` project.**

This directory contains only SDK usage documentation and basic code snippets.

## SDK Usage Examples

### Basic Connection Example

```python
from feagi_connector import FeagiClient
from feagi_connector.protocols import FSMPChannel

# Create client
client = FeagiClient(host="localhost", agent_id="my-agent")

# Connect
await client.connect()

# Send sensory data  
await client.send_sensory_data(FSMPChannel.VISION, image_bytes)

# Register motor callback
await client.register_motor_callback(handle_motor_data)

# Disconnect
await client.disconnect()
```

### Performance Comparison (Python vs Rust)

```python
# Explicit implementation selection
from feagi_connector.utils.processing import encode_neuron_potential_xyz_python
from feagi_connector.utils.rust_processing import encode_neuron_potential_xyz_rust

# Use Python implementation
py_encoded = encode_neuron_potential_xyz_python(neuron_data)

# Use Rust implementation (5-20x faster)
rust_encoded = encode_neuron_potential_xyz_rust(neuron_data)
```

## Complete Agent Examples

For complete, runnable agent implementations that demonstrate:
- Robot control agents
- IoT sensor agents  
- Vision processing agents
- Custom agent extensions

**See the `simple_agent` project**, which serves as the reference implementation showing how to use FEAGI Connector in real applications. 