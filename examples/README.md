# FEAGI Connector Examples

**Note: FEAGI Connector is now a pure SDK library. For complete agent examples and reference implementations, please see the `simple_agent` project.**

This directory contains SDK usage documentation and code examples demonstrating the new Rust-powered API.

## Requirements

Install with video support:
```bash
pip install --extra-index-url https://test.pypi.org/simple/ feagi-connector[video]
```

## SDK Usage Examples

### New API: Segmented Vision with Gaze Control (v0.0.73+)

See `segmented_vision_gaze_example.py` for a complete example using the new Rust data structures.

```python
import feagi_rust_py_libs as frpl
from feagi_connector import FeagiAgentConnector

# Create image frames using Rust types
color_space = frpl.connector_core.data.descriptors.ColorSpace.Linear
memory_order = frpl.connector_core.data.descriptors.MemoryOrderLayout.WidthsHeightsChannels
image_frame = frpl.connector_core.data.ImageFrame.new_from_array(
    numpy_array, color_space, memory_order
)

# Create gaze properties
eccentricity = frpl.connector_core.data.Percentage2D(
    frpl.connector_core.data.Percentage.new_from_0_1(0.2),
    frpl.connector_core.data.Percentage.new_from_0_1(0.2)
)

# Register segmented vision sensor
agent.sensors.image_camera_with_peripheral.register(
    cortical_group, num_channels, input_props, segment_props, gaze
)

# Write image data
agent.sensors.image_camera_with_peripheral.write(
    cortical_group, channel, image_frame
)

# Get structured byte container
byte_container = agent.sensor_get_byte_container()
neuron_data = byte_container.try_create_new_struct_from_index(0)
```

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