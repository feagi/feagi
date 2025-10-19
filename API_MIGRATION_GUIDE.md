# FEAGI Connector API Migration Guide

## Version 0.1.0 with feagi-rust-py-libs v0.0.73

This guide documents the new API for working with FEAGI Connector, which now uses `feagi-rust-py-libs` v0.0.73 and `feagi-data-processing` v0.0.50-beta.43.

### Installation

```bash
# Standard installation
pip install --extra-index-url https://test.pypi.org/simple/ feagi-connector

# Install with video support
pip install --extra-index-url https://test.pypi.org/simple/ feagi-connector[video]

# Install with all extras (video, REST API)
pip install --extra-index-url https://test.pypi.org/simple/ feagi-connector[full]
```

## Key API Changes

### 1. Segmented Vision Registration

**New API** uses Rust data structures for type safety and performance:

```python
import feagi_rust_py_libs as frpl
from feagi_connector import FeagiAgentConnector

# Create resolution descriptors
input_resolution = frpl.connector_core.data.descriptors.ImageXYResolution(1280, 720)
center_resolution = frpl.connector_core.data.descriptors.ImageXYResolution(128, 128)
peripheral_resolution = frpl.connector_core.data.descriptors.ImageXYResolution(64, 64)

# Create gaze properties
eccentricity = frpl.connector_core.data.Percentage2D(
    frpl.connector_core.data.Percentage.new_from_0_1(0.2),
    frpl.connector_core.data.Percentage.new_from_0_1(0.2)
)

modulation = frpl.connector_core.data.Percentage2D(
    frpl.connector_core.data.Percentage.new_from_0_1(0.2),
    frpl.connector_core.data.Percentage.new_from_0_1(0.2)
)

# Create output resolutions
output_resolutions = frpl.connector_core.data.descriptors.SegmentedXYImageResolutions.create_with_same_sized_peripheral(
    center_resolution,
    peripheral_resolution
)

gaze = frpl.connector_core.data.descriptors.GazeProperties(eccentricity, modulation)

# Create segment properties
segment_properties = frpl.connector_core.data.descriptors.SegmentedImageFrameProperties(
    output_resolutions,
    color_channel_layout,
    color_channel_layout,
    color_space
)

# Register segmented vision sensor
agent.sensors.image_camera_with_peripheral.register(
    cortical_group,
    number_of_channels,
    input_image_properties,
    segment_properties,
    gaze
)
```

### 2. Image Frame Creation

**New API** uses Rust ImageFrame objects:

```python
import numpy as np
import feagi_rust_py_libs as frpl

# Create image data
image_data = np.ones((1280, 720, 3), dtype=np.uint8) * 128

# Define image properties
color_space = frpl.connector_core.data.descriptors.ColorSpace.Linear
memory_order = frpl.connector_core.data.descriptors.MemoryOrderLayout.WidthsHeightsChannels

# Create ImageFrame
image_frame = frpl.connector_core.data.ImageFrame.new_from_array(
    image_data,
    color_space,
    memory_order
)

# Get properties
image_properties = image_frame.get_image_frame_properties()
channel_layout = image_frame.channel_layout
```

### 3. Writing Sensor Data

**New API** uses `.write()` method (alias for `.store()`):

```python
# Write image data to sensor
agent.sensors.image_camera_with_peripheral.write(
    cortical_group,
    device_channel,
    image_frame
)
```

### 4. Gaze Motor Registration

**New API** for gaze motor control:

```python
# Register gaze motor
agent.motors.gaze.register(
    cortical_group,
    number_of_channels,
    number_z_neurons
)

# Read gaze data
gaze_data = agent.motors.gaze.read_cache(cortical_group, channel)

# Read post-processed gaze (as Percentage4D)
gaze_percentage = agent.motors.gaze.read_post_processed(cortical_group, device_channel)
```

### 5. Byte Container Access

**New API** provides structured access to encoded data:

```python
# Get byte container with structured access
byte_container = agent.sensor_get_byte_container()

if byte_container:
    # Get number of structures
    num_structs = byte_container.number_contained_structures
    
    # Extract individual structures
    for i in range(num_structs):
        neuron_data = byte_container.try_create_new_struct_from_index(i)
        # Process neuron_data...

# Or get raw bytes
raw_bytes = agent.get_most_recent_sensor_bytes()
```

### 6. Stage Property Updates

**New API** for dynamic stage updates:

```python
# Update stage properties (e.g., gaze parameters)
agent.sensors.image_camera_with_peripheral.update_stage(
    cortical_group,
    device_channel,
    stage_index,
    new_stage_properties
)
```

## API Compatibility Matrix

| Feature | Old API | New API (v0.0.73+) | Status |
|---------|---------|-------------------|--------|
| Basic sensor registration | ✓ | ✓ | Both supported |
| Segmented vision | Limited | ✓ Full support | New API recommended |
| Gaze control | ❌ | ✓ | New API only |
| Rust ImageFrame | ❌ | ✓ | New API only |
| Byte containers | ❌ | ✓ | New API only |
| Stage updates | ❌ | ✓ | New API only |

## Underlying Rust Library Changes

### feagi-data-processing v0.0.50-beta.43

Key changes in the Rust data structures:

1. **Module renamed**: `neurons` → `neuron_voxels`
2. **Types renamed**:
   - `NeuronXYZP` → `NeuronVoxelXYZP`
   - `NeuronXYZPArrays` → `NeuronVoxelXYZPArrays`
   - `CorticalMappedXYZPNeuronData` → `CorticalMappedXYZPNeuronVoxels`
3. **Method renamed**: 
   - `try_update_from_byte_slice` → `try_deserialize_and_update_self_from_byte_slice`

These changes are automatically handled by the feagi-connector wrapper.

## Complete Example

See `examples/segmented_vision_gaze_example.py` for a complete working example.

## Troubleshooting

### ImportError: feagi_rust_py_libs not found

Reinstall feagi-connector:
```bash
pip install --extra-index-url https://test.pypi.org/simple/ --upgrade --force-reinstall feagi-connector
```

### AttributeError: 'module' object has no attribute 'connector_core'

Upgrade to feagi-rust-py-libs v0.0.73 or later:
```bash
pip install --extra-index-url https://test.pypi.org/simple/ --upgrade feagi-rust-py-libs>=0.0.73
```

### byte_container returns None

Ensure feagi-rust-py-libs is properly installed and you've written data to sensors before calling `sensor_get_byte_container()`.

## Migration Checklist

- [ ] Update dependencies to use feagi-rust-py-libs v0.0.73+
- [ ] Import `feagi_rust_py_libs as frpl` in your code
- [ ] Update sensor registration to use Rust data structures
- [ ] Replace custom image processing with Rust ImageFrame objects
- [ ] Update gaze control code to use new motor API
- [ ] Replace raw byte access with byte container API where beneficial
- [ ] Test with the new API using provided examples

## Getting Help

For questions or issues:
- Check the examples in `feagi-connector/examples/`
- Review the sample code in `archive_del/feagi-connector/Python_Connector/sample/`
- Refer to the Rust library documentation: https://docs.rs/feagi-rust-py-libs

