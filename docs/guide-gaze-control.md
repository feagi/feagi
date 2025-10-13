# FEAGI Connector Gaze Control Guide

This guide explains how to use the enhanced segmented vision processing with gaze control capabilities in FEAGI Connector 2.0, updated for `feagi-rust-py-libs` version 0.0.66+.

## Overview

The updated FEAGI Connector includes advanced gaze control functionality that allows agents to dynamically adjust their visual attention based on motor feedback from FEAGI. This is implemented using the latest `feagi_rust_py_libs` 0.0.66+ patterns and provides:

- **Dynamic Gaze Control**: Adjust visual focus in real-time
- **Segmented Vision Processing**: 3x3 vision segmentation with center/peripheral regions
- **Motor-Vision Integration**: Bidirectional communication between vision and motor systems
- **Advanced Gaze Properties**: Eccentricity and modularity control for natural attention mechanisms

## New in feagi-rust-py-libs 0.0.66+

The latest version introduces several API improvements:

- **New API Path**: `frpl.connector_core.data.*` instead of `frpl.data_structures.data.*`
- **Stage Updates**: Dynamic stage property updates via `update_stage()` method
- **Percentage4D**: Motor data now returns `Percentage4D` objects with `.get_as_0_1()` accessor
- **Factory Method**: `FeagiAgentConnector.create_dummy_connector()` for testing
- **Stage Properties**: New `ImageSegmentorStageProperties` for pipeline configuration

The connector maintains backward compatibility with fallbacks to older API paths.

## Key Components

### SegmentedVisionProcessor

Enhanced with gaze control capabilities:

```python
from feagi_connector import SegmentedVisionProcessor

processor = SegmentedVisionProcessor(
    cortical_group_index=0,
    center_dims=(128, 128),
    peripheral_dims=(64, 64),
    eccentricity=(0.2, 0.2),    # Focus area size (0-1)
    modularity=(0.2, 0.2),      # Modulation parameters (0-1)
    gaze_position=(0.5, 0.5),   # Initial gaze (0-1, center)
    number_of_channels=1
)

# Process frames with gaze
sensor_bytes = processor.process_frame(bgr_frame)

# Update gaze dynamically
processor.update_gaze(0.3, 0.7)  # Move gaze to lower-left
```

### GazeMotorProcessor

Handles motor feedback for gaze control:

```python
from feagi_connector import GazeMotorProcessor

gaze_motor = GazeMotorProcessor(
    cortical_group_index=0,
    num_channels=10,
    gaze_resolution=8
)

# Register with FEAGI motor system
gaze_motor.register_gaze_motor()

# Create gaze control neurons
mapped_neurons = gaze_motor.create_gaze_neurons(
    gaze_x=0.7, 
    gaze_y=0.3, 
    intensity=1.0
)

# Process motor feedback
motor_bytes = get_motor_data_from_feagi()
gaze_position = gaze_motor.process_motor_bytes(motor_bytes)
if gaze_position:
    processor.update_gaze(*gaze_position)
```

### FeagiAgentConnector Integration

The agent connector now exposes motor functionality:

```python
from feagi_connector import FeagiAgentConnector

# Create connector (using existing patterns)
agent = create_agent_connector()

# Access motor functionality
agent.motors.gaze.register(
    cortical_group=0, 
    num_channels=10, 
    gaze_resolution=8
)

# Process motor data
agent.motors.read_bytes_into_motor(motor_bytes)

# Read gaze cache
gaze_data = agent.motors.gaze.read_cache(0, 0)
```

## Gaze Control Parameters

### Eccentricity
Controls the size of the focus area:
- `(0.1, 0.1)`: Very focused attention
- `(0.2, 0.2)`: Moderate focus (default)
- `(0.5, 0.5)`: Wide attention area

### Modularity
Controls the modulation of attention:
- Lower values: Sharp attention boundaries
- Higher values: Softer attention transitions

### Gaze Position
Coordinates are normalized (0.0 to 1.0):
- `(0.0, 0.0)`: Top-left corner
- `(0.5, 0.5)`: Center (default)
- `(1.0, 1.0)`: Bottom-right corner

## Integration Patterns

### Vision-Motor Loop

```python
async def vision_motor_loop(agent, vision_processor, gaze_motor):
    while True:
        # 1. Capture frame
        frame = capture_camera_frame()
        
        # 2. Process with current gaze
        sensor_bytes = vision_processor.process_frame(frame)
        
        # 3. Send to FEAGI
        await send_to_feagi(sensor_bytes)
        
        # 4. Receive motor feedback
        motor_bytes = await receive_from_feagi()
        
        # 5. Process gaze commands
        if motor_bytes:
            agent.motors.read_bytes_into_motor(motor_bytes)
            gaze_pos = gaze_motor.process_motor_bytes(motor_bytes)
            
            if gaze_pos:
                # 6. Update vision gaze
                vision_processor.update_gaze(*gaze_pos)
                print(f"Updated gaze to: {gaze_pos}")
        
        await asyncio.sleep(0.033)  # ~30 FPS
```

### Creating Gaze Neurons

```python
from feagi_connector import create_gaze_control_neurons

# Create smooth gaze control neurons
neurons = create_gaze_control_neurons(
    gaze_x=0.6,      # 60% to the right
    gaze_y=0.4,      # 40% down
    intensity=0.8,   # 80% activation
    resolution=8     # 8x8 motor resolution
)

# Use with motor system
mapped_neurons = map_neurons_to_cortical_area(neurons, gaze_cortical_id)
motor_bytes = encode_neurons_to_bytes(mapped_neurons)
```

## Best Practices

### Performance

- **Update gaze sparingly**: Only when motor feedback indicates significant changes
- **Use appropriate resolution**: Higher gaze resolution = more precise but more computation
- **Batch processing**: Process multiple frames before updating gaze for smoother operation

### Integration

- **Coordinate spaces**: Ensure gaze coordinates match your vision system's expectations
- **Timing**: Allow time for gaze updates to take effect before processing next frame
- **Fallbacks**: Handle cases where motor feedback is unavailable

### Debugging

```python
# Enable debug logging
import logging
logging.getLogger("feagi_connector").setLevel(logging.DEBUG)

# Monitor gaze changes
print(f"Current gaze: {vision_processor.gaze}")
print(f"Gaze properties: {vision_processor.gaze_properties}")

# Check motor registration
if gaze_motor._registered:
    print(f"Gaze motor registered: group={gaze_motor.group_index}")
```

## Example Applications

1. **Autonomous Navigation**: Gaze follows interesting objects or path markers
2. **Interactive Agents**: Gaze tracks user movements or points of interest  
3. **Visual Search**: Systematic scanning with learned gaze patterns
4. **Attention Networks**: Training FEAGI to develop natural attention behaviors

## Migration from Previous Versions

If updating from earlier versions:

1. **Add gaze parameters** to SegmentedVisionProcessor constructor
2. **Create GazeMotorProcessor** instance for motor integration
3. **Update motor callbacks** to handle gaze data
4. **Use `image_camera_with_peripheral`** registration pattern

```python
# Before
processor = SegmentedVisionProcessor(group, center_dims, per_dims)

# After  
processor = SegmentedVisionProcessor(
    group, center_dims, per_dims,
    eccentricity=(0.2, 0.2),
    modularity=(0.2, 0.2),
    gaze_position=(0.5, 0.5)
)
```

## See Also

- [Example: segmented_vision_with_gaze.py](../examples/segmented_vision_with_gaze.py)
- [Architecture Overview](arch-connector-overview.md)
- [Protocol Specifications](spec-protocols.md)
