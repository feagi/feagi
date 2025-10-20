# FEAGI Inference Engine

A standalone Rust-based neural processing engine for FEAGI (Framework for Evolutionary Artificial General Intelligence). The inference engine provides real-time neural inference with online learning capabilities, enabling FEAGI to run independently without Python dependencies.

## Overview

The FEAGI Inference Engine is a high-performance, standalone application that:

- **Loads pre-trained connectomes** from serialized brain files
- **Processes sensory input** from video files and converts them to neural activations
- **Executes neural bursts** with synaptic propagation and neural dynamics
- **Extracts motor output** from active neurons in motor cortical areas
- **Supports online learning** with dynamic synapse and neuron modification
- **Runs at configurable frequencies** (default: 50Hz burst rate)

This enables FEAGI to operate in resource-constrained environments, embedded systems, and applications where Python overhead is undesirable.

## Key Features

### Neural Processing
- **Real-time burst execution** with configurable frequency (1-1000Hz)
- **Online learning** - synapses and neurons can be modified during runtime
- **Connectome persistence** - save/load complete brain state
- **Multi-cortical area support** - vision, motor, and custom cortical regions

### Video Input Processing
- **Multiple video formats** supported via FFmpeg (MP4, AVI, MOV, etc.)
- **Automatic frame resizing** for performance optimization
- **Configurable frame sampling** to control processing load
- **Video looping** for continuous operation
- **XYZP coordinate encoding** - pixels mapped to 3D voxel space with intensity

### Motor Output
- **Standard FEAGI data format** using feagi-data-processing structures
- **Real-time extraction** from fire queue
- **Multi-area motor control** support
- **Verbose output** with neuron coordinates and activation levels

### Platform Support
- **Cross-platform** - Linux, macOS, Windows
- **Embedded-ready** - designed for migration to RTOS
- **Docker-compatible** - can run in containers
- **No Python required** - pure Rust implementation

## Prerequisites

- **Rust toolchain** 1.70+ (install from https://rustup.rs/)
- **FFmpeg libraries** (for video processing)
  - Ubuntu/Debian: `sudo apt-get install libavcodec-dev libavformat-dev libavutil-dev libswscale-dev`
  - macOS: `brew install ffmpeg`
  - Windows: Download from https://ffmpeg.org/download.html

## Building

Build the inference engine from the FEAGI workspace:

```bash
cd feagi_core/feagi-rust
cargo build --release --bin feagi-inference-engine
```

The binary will be created at:
```
target/release/feagi-inference-engine
```

## Usage

### Basic Usage

Run the inference engine with a pre-trained connectome:

```bash
./feagi-inference-engine --connectome path/to/brain.connectome
```

### With Video Input

Process video input and inject as sensory data:

```bash
./feagi-inference-engine \
  --connectome brain.connectome \
  --video input.mp4 \
  --vision-cortical-area ipu_vision
```

### Full Configuration

```bash
./feagi-inference-engine \
  --connectome brain.connectome \
  --video input.mp4 \
  --vision-cortical-area ipu_vision \
  --motor-cortical-areas "opu_motor_left,opu_motor_right" \
  --burst-hz 50 \
  --resize 64x64 \
  --frame-skip 1 \
  --loop-video \
  --auto-save
```

## Command-Line Options

### Required Parameters
- `--connectome <PATH>` - Path to serialized connectome file to load

### Video Input
- `--video <PATH>` - Path to video file for visual input (optional)
- `--vision-cortical-area <NAME>` - Cortical area ID for vision (default: "ipu_vision")
- `--resize <WxH>` - Resize video frames (e.g., "64x64", "128x128")
- `--frame-skip <N>` - Process every Nth frame (default: 1)
- `--loop-video` - Loop video playback indefinitely (default: true)

### Neural Processing
- `--burst-hz <N>` - Burst frequency in Hz (default: 50)
- `--motor-cortical-areas <NAMES>` - Comma-separated motor area IDs (default: "opu_motor")

### Persistence
- `--auto-save` - Auto-save connectome on shutdown (default: true)
- `--checkpoint-interval <SECONDS>` - Periodic checkpoint interval (0 = disabled)

### Debugging
- `--verbose` - Enable verbose logging
- `--help` - Display all options

## Architecture

### Data Flow

```
┌─────────────┐
│ Video Input │
└──────┬──────┘
       │ Frame-by-frame
       ▼
┌─────────────────────┐
│ Sensory Injection   │
│ (XYZP Coordinates)  │
└──────┬──────────────┘
       │ Inject into FCL
       ▼
┌─────────────────────┐
│  Neural Processing  │
│  - Neural Dynamics  │
│  - Synaptic Prop.   │
│  - Online Learning  │
└──────┬──────────────┘
       │ Fire Queue
       ▼
┌─────────────────────┐
│  Motor Extraction   │
│ (XYZP from FQ)      │
└──────┬──────────────┘
       │ Encode & Output
       ▼
┌─────────────────────┐
│   Motor Output      │
│ (Console/SHM/File)  │
└─────────────────────┘
```

### Components

- **Video Reader** - FFmpeg-based video decoder with frame extraction
- **Sensory Injector** - Converts pixels to XYZP voxel coordinates
- **RustNPU** - Core neural processing unit with burst execution
- **Motor Extractor** - Extracts and encodes motor neuron activations
- **Connectome Serializer** - Saves/loads complete brain state

## Connectome Files

### Creating a Connectome

Connectomes are created by running FEAGI with Python and then serializing the trained brain:

```python
# In Python FEAGI
from feagi_burst_engine import RustNPU

# Train your brain...
# ...

# Export connectome
connectome = npu.export_connectome()
save_connectome(connectome, "trained_brain.connectome")
```

### Connectome Format

Connectome files contain:
- **Neuron arrays** - All neuron properties and states
- **Synapse arrays** - All synaptic connections and weights
- **Cortical mappings** - Area ID to name mappings
- **Runtime state** - Burst count, fire ledger, etc.
- **Metadata** - Timestamp, description, source information

Files use binary serialization (bincode) for fast loading and compact storage.

## Performance Optimization

### Video Processing
- Use `--resize` to reduce frame resolution (e.g., 64x64 for faster processing)
- Set `--frame-skip 2` or higher to sample every Nth frame
- The inference engine automatically skips dim pixels below activation threshold

### Burst Frequency
- Default 50Hz is suitable for most applications
- Increase to 100Hz+ for faster reaction times
- Decrease to 20-30Hz for lower CPU usage

### Memory Usage
- Connectome size depends on neuron/synapse count
- Typical brain: 10K neurons, 100K synapses = ~10MB connectome file
- Runtime memory: 2-3x connectome size

## Use Cases

### Robotics
Run FEAGI inference engine on embedded controllers (Raspberry Pi, NVIDIA Jetson) for real-time robot control with vision processing.

### Edge Computing
Deploy neural processing at the edge without Python/GPU requirements, enabling AI in resource-constrained environments.

### Research & Development
Rapid iteration on neural architectures with fast Rust execution and Python-free deployment for production systems.

### Autonomous Systems
Integrate with ROS, RTOS, or custom control systems for autonomous vehicles, drones, and industrial automation.

### Simulation & Testing
Run headless simulations at high speed for training and validation without visualization overhead.

## Limitations & Future Work

### Current Limitations
- Motor output is printed to console (no hardware integration yet)
- Cortical area IDs are simplified (need proper genome mapping)
- No runtime genome modification (connectome must be pre-built)
- Single video input (no multi-modal sensory fusion)

### Planned Features
- Motor output to hardware interfaces (GPIO, I2C, serial)
- Real-time genome modification and cortical area creation
- Multi-modal input (audio, tactile, proprioception)
- Distributed processing across multiple inference engines
- WebAssembly compilation for browser-based inference

## Troubleshooting

### "Failed to load connectome"
Ensure the connectome file exists and was created with compatible FEAGI version. Connectome format versioning ensures backward compatibility.

### "No valid neurons found for XYZP coordinates"
The vision cortical area in the connectome may not match the CLI parameter. Verify cortical area names match between genome and CLI arguments.

### Low Performance / High CPU
Reduce video resolution with `--resize`, increase `--frame-skip`, or lower `--burst-hz`. Monitor neuron count - very large brains require more processing power.

### FFmpeg Errors
Ensure FFmpeg libraries are properly installed. On Linux, verify `libavcodec`, `libavformat`, `libavutil`, and `libswscale` are available.

## Contributing

The FEAGI Inference Engine is part of the FEAGI 2.0 architecture. Contributions should follow the project's Rust/RTOS compatibility guidelines.

See the main FEAGI repository for contribution guidelines and architecture documentation.

## License

Copyright 2025 Neuraville Inc.  
Licensed under the Apache License, Version 2.0

## Related Documentation

- [FEAGI Core README](../../README.md) - Main FEAGI documentation
- [Connectome Serialization](../feagi-connectome-serialization/) - Connectome file format
- [Burst Engine](../feagi-burst-engine/) - Neural processing engine
- [Architecture Guide](/feagi_core/docs/) - FEAGI 2.0 architecture overview

