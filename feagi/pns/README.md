# FEAGI Peripheral Nervous System (PNS) Module

The Peripheral Nervous System (PNS) module provides sensorimotor interfaces between FEAGI and the external world, allowing for embodied cognition through various sensory inputs and motor outputs.

## Overview

The PNS module serves as a bridge between FEAGI's internal neural processing and external environments. It handles:

- Processing sensory inputs (vision, audio, proprioception, etc.)
- Generating motor outputs (motion control, speech, etc.)
- Encoding/decoding between physical data and neural activity patterns
- Managing input/output channels for agent embodiment

## Components

### Vision System

The vision component processes visual inputs, converting them into neural signals that FEAGI can process:

```python
from feagi.pns import vision

# Process an image into neural spikes
image_data = load_image("scene.jpg")
neuron_activity = vision.process_visual_input(
    image=image_data,
    cortical_area_id="visual_cortex"
)
```

Key features:
- RGB image processing
- Feature extraction and encoding
- Visual attention mechanisms
- Configurable receptor fields

### Motor Control

The motor component translates neural activity into physical actions:

```python
from feagi.pns import motor

# Convert neural activity to motor commands
motor_commands = motor.decode_motor_activity(
    activity_data=neuron_data,
    motor_mapping=motor_mapping_config
)
```

### Agent Integration

The PNS module enables seamless integration with various embodiment platforms:

- Robotic systems
- Virtual environments
- Simulated physics engines
- Real-time control systems

## Configuration

Each sensorimotor system can be configured through mapping configurations that define how physical data maps to neural representations:

```python
vision_config = {
    "input_dimensions": (640, 480),
    "cortical_mapping": "visual_cortex",
    "receptive_field_size": 10,
    "encoding_scheme": "rate_based"
}
```

## Integration with FEAGI

The PNS module integrates with other FEAGI components:

- **ZMQ Interface**: Communicates with external agents using the FSMP protocol
- **BDU**: Coordinates with the Brain Development Unit to create sensorimotor pathways
- **NPU**: Sends encoded sensory data to the Neural Processing Unit
- **Agent Registration**: Automatic coordination between connected agents and FQ samplers (see [agent-coordination.md](agent-coordination.md))

### Agent Registration & Coordination System

FEAGI 2.0 introduces an intelligent agent registration system that provides automatic coordination between connected agents and Fire Queue (FQ) samplers. This eliminates manual intervention for data flow management:

- **Automatic FQ Sampler Management**: Samplers enable/disable based on agent capabilities
- **Resource Efficiency**: CPU usage only when agents actually need data
- **Zero Manual Intervention**: Agents register with capabilities, system handles coordination
- **RUST/RTOS Compatibility**: Enable/disable patterns for embedded systems

For detailed information, see the [Agent Coordination Documentation](agent-coordination.md).

## Development

### Adding New Sensory Systems

To add a new sensory system:

1. Create a new module in the PNS package
2. Implement the required encoder/decoder interfaces
3. Register with the sensory registry
4. Add appropriate configuration options

### Adding New Motor Systems

To add a new motor system:

1. Create a motor interface module
2. Implement neural activity decoders
3. Register with the motor command registry
4. Add appropriate configuration options

## Future Directions

Planned enhancements for the PNS module:

- Advanced auditory processing
- Tactile sensation support
- Proprioception and balance systems
- Enhanced temporal encoding for sensory data 