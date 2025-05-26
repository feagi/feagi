# PNS Module Architecture

This document describes the architectural design of FEAGI's Peripheral Nervous System (PNS) module.

## Architectural Overview

The PNS module is designed as the interface layer between FEAGI's internal neural processing and external environments. It follows these key architectural principles:

1. **Bidirectional Mapping**: Converts between physical sensor data and neural representations
2. **Configurable Encoders/Decoders**: Provides flexible processing of various sensory modalities
3. **Real-time Processing**: Optimized for timely sensorimotor processing
4. **Embodiment Abstraction**: Supports multiple types of physical or virtual embodiments

## System Components

### Sensory System Architecture

```
┌─────────────────────┐     ┌─────────────────────┐     ┌─────────────────┐
│  External Sensors   │     │  Sensory Encoders   │     │ Neural Activity  │
│                     │     │                     │     │                  │
│ - Cameras           │────►│ - Vision Processing │────►│ - FCL Updates   │
│ - Microphones       │     │ - Audio Processing  │     │ - Spike Patterns │
│ - Proprioceptors    │     │ - Tactile Encoding  │     │                  │
└─────────────────────┘     └─────────────────────┘     └─────────────────┘
```

### Motor System Architecture

```
┌─────────────────────┐     ┌─────────────────────┐     ┌─────────────────┐
│  Neural Activity    │     │   Motor Decoders    │     │ External Motors  │
│                     │     │                     │     │                  │
│ - Motor Area FCLs   │────►│ - Pattern Decoding  │────►│ - Actuators     │
│ - Command Sequences │     │ - Command Generation│     │ - Speakers       │
│                     │     │ - Timing Control    │     │ - Displays       │
└─────────────────────┘     └─────────────────────┘     └─────────────────┘
```

## Vision System

The vision processing system is one of the most developed components in the PNS module. It handles:

### Vision Components

1. **Central Vision**: High-resolution processing of the center of the visual field
   - Configurable resolution for detailed processing
   - Color vision support (RGB channels)
   
2. **Peripheral Vision**: Lower-resolution processing of the peripheral visual field
   - Divided into 8 peripheral regions (top-right, top-left, etc.)
   - Optimized for motion detection and large-scale features

3. **Visual Enhancement**: Configurable image processing parameters
   - Brightness adjustment
   - Contrast enhancement
   - Shadow processing
   - Change thresholds for motion detection

4. **Visual Modulation**: Controls for visual attention
   - Eccentricity parameters for focus control
   - Modulation parameters for sensitivity adjustment
   - Flicker period for temporal processing

### Vision Processing Flow

```
1. Image acquisition from external source
2. Pre-processing (resizing, color conversion if needed)
3. Image enhancement (brightness, contrast, etc.)
4. Splitting into central and peripheral regions
5. Feature extraction for each region
6. Neural encoding based on extracted features
7. FCL updates to relevant visual cortical areas
```

## Motor System

The motor system translates neural activity patterns into motor commands for embodiment:

### Motor System Components

1. **Pattern Recognition**: Interpreting neural activity patterns
2. **Command Generation**: Converting neural patterns to physical commands
3. **Timing Control**: Ensuring appropriate timing of motor actions
4. **Feedback Integration**: Incorporating sensory feedback into motor control

## Protocol Integration

The PNS module integrates with external systems using standardized protocols:

1. **FSMP (FEAGI Sensorimotor Protocol)**: For exchanging sensory and motor data
   - Binary message format for efficiency
   - Channel-based organization for different sensory modalities
   - Reliable delivery mechanisms
   
2. **ZeroMQ Transport**: For high-performance message passing
   - PUB/SUB pattern for sensor data broadcasts
   - REQ/REP pattern for command responses
   - Configurable synchronous/asynchronous communication

## Configuration Architecture

The PNS module uses a hierarchical configuration system:

```
┌─────────────────────┐
│  Global PNS Config  │
│                     │
│ - Common Parameters │
│ - Channel Settings  │
└──────────┬──────────┘
           │
           ▼
┌──────────┴──────────┐
│ Modality Configs    │
├─────────────────────┤
│ - Vision Config     │
│ - Audio Config      │
│ - Motor Config      │
└──────────┬──────────┘
           │
           ▼
┌──────────┴──────────┐
│ Processing Configs  │
├─────────────────────┤
│ - Enhancement       │
│ - Encoding Schemes  │
│ - Timing Parameters │
└─────────────────────┘
```

The vision configuration includes:

```python
{
    "central_vision_resolution": (width, height),
    "peripheral_vision_resolution": (width, height),
    "flicker_period": period_ms,
    "color_vision": bool,
    "eccentricity": [eccen_x, eccen_y],
    "modulation": [mod_x, mod_y],
    "brightness": brightness_value,
    "contrast": contrast_value,
    "shadows": shadows_value,
    "pixel_change_limit": threshold_value
}
```

## Cortical Area Integration

The PNS module maps to specific cortical areas in the FEAGI connectome:

- **Central Vision**: Maps to cortical area "iv00_C"
- **Peripheral Vision**: Maps to cortical areas ["iv00TR", "iv00TL", "iv00TM", "iv00MR", "iv00ML", "iv00BR", "iv00BL", "iv00BM"]
- **Vision Enhancement**: Maps to "ov_enh" (output visual enhancement)
- **Vision Thresholds**: Maps to "ovtune"
- **Eccentricity Control**: Maps to "ov_ecc"
- **Modulation Control**: Maps to "ov_mod"
- **Vision Flipping**: Maps to "ovflph" (horizontal) and "ovflpv" (vertical)
- **Vision Flicker**: Maps to "o_blnk"

## Performance Considerations

The PNS module incorporates these performance optimizations:

1. **Pre-processed Mappings**: Pre-compute mappings between physical and neural representations
2. **Batch Processing**: Process sensory data in optimized batches
3. **Asynchronous Processing**: Handle slow sensory processing in separate threads
4. **Optimized Encodings**: Use efficient binary encodings for sensor data

## Error Handling

The PNS module includes robust error handling:

1. **Sensor Failure Detection**: Detect and report sensor failures
2. **Graceful Degradation**: Continue operation with reduced capabilities when sensors fail
3. **Data Validation**: Validate incoming sensory data before processing
4. **Fallback Modes**: Default to safe operation modes when processing fails

## Future Architecture

The planned architectural extensions include:

1. **Multi-modal Integration**: Cross-modal sensory integration
2. **Predictive Processing**: Incorporating predictive models for sensorimotor anticipation
3. **Adaptive Encoding**: Dynamically adjusting encoding parameters based on context
4. **Distributed Sensory Processing**: Supporting distributed sensor networks 