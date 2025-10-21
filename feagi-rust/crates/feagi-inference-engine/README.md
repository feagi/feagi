# FEAGI Inference Engine

A standalone Rust-based neural processing engine for FEAGI (Framework for Evolutionary Artificial General Intelligence). The inference engine provides real-time neural inference with online learning capabilities, completely decoupled from I/O and agent implementations.

## Overview

The FEAGI Inference Engine is a high-performance, standalone application that:

- **Loads pre-trained connectomes** from serialized brain files
- **Executes neural bursts** with synaptic propagation and neural dynamics
- **Communicates via ZMQ** with external agents for sensory input and motor output
- **Supports online learning** with dynamic synapse and neuron modification
- **Runs at configurable frequencies** (default: 50Hz burst rate)
- **Agent-agnostic** - works with any agent that implements the ZMQ protocol

This enables FEAGI to operate in resource-constrained environments, embedded systems, and applications where Python overhead is undesirable, while maintaining complete separation between neural processing and I/O handling.

## Key Features

### Neural Processing
- **Real-time burst execution** with configurable frequency (1-1000Hz)
- **Online learning** - synapses and neurons can be modified during runtime
- **Connectome persistence** - save/load complete brain state
- **Multi-cortical area support** - unlimited custom cortical regions

### Agent Communication (ZMQ)
- **Agent registration** - dynamic discovery and capability negotiation
- **Sensory input** - receive data from external agents (vision, audio, sensors, etc.)
- **Motor output** - publish commands to external agents (motors, actuators, etc.)
- **Multi-agent support** - multiple agents can connect simultaneously
- **Language-agnostic** - agents can be written in any language with ZMQ support

### Architecture
- **Decoupled design** - inference engine has zero knowledge of agent implementations
- **Pure NPU core** - no I/O coupling, no hardware dependencies
- **Scalable** - distribute agents across processes, machines, or networks
- **Testable** - easy to mock and test with simple ZMQ clients

### Platform Support
- **Cross-platform** - Linux, macOS, Windows
- **Embedded-ready** - designed for migration to RTOS
- **Docker/Kubernetes** - container-native with service discovery
- **No Python required** - pure Rust implementation

## Prerequisites

- **Rust toolchain** 1.70+ (install from https://rustup.rs/)
- **ZMQ library** (for agent communication)
  - Ubuntu/Debian: `sudo apt-get install libzmq3-dev`
  - macOS: `brew install zeromq`
  - Windows: Download from https://zeromq.org/download/

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

The engine will start and wait for agents to connect via ZMQ.

### With External Agents

Start the inference engine, then connect agents:

**Terminal 1 - Inference Engine:**
```bash
./feagi-inference-engine --connectome brain.connectome --burst-hz 50
```

**Terminal 2 - Video Agent:**
```bash
cd video_agent_rust
python video_agent.py --video input.mp4 --inference-host localhost:5000
```

**Terminal 3 - Motor Agent (example):**
```bash
python motor_agent.py --inference-host localhost:5000
```

### Full Configuration

```bash
./feagi-inference-engine \
  --connectome brain.connectome \
  --burst-hz 50 \
  --checkpoint-interval 300 \
  --auto-save \
  --verbose
```

## Command-Line Options

### Required Parameters
- `--connectome <PATH>` - Path to serialized connectome file to load

### Neural Processing
- `--burst-hz <N>` - Burst frequency in Hz (default: 50)

### Persistence
- `--auto-save` - Auto-save connectome on shutdown (default: true)
- `--checkpoint-interval <SECONDS>` - Periodic checkpoint interval (0 = disabled)

### Debugging
- `--verbose` - Enable verbose logging
- `--help` - Display all options

### ZMQ Endpoints (TODO - Not Yet Implemented)
- `--registration-port <PORT>` - Agent registration endpoint (default: 5000)
- `--sensory-port <PORT>` - Sensory input endpoint (default: 5555)
- `--motor-port <PORT>` - Motor output endpoint (default: 5556)

## Architecture

### Data Flow

```
┌──────────────────┐         ZMQ REQ/REP         ┌────────────────────┐
│  External Agents │◄──────► (Registration)      │  FEAGI Inference   │
│                  │                               │  Engine (Rust)     │
│  - Video Agent   │         ZMQ PUSH             │                    │
│  - Audio Agent   │────────► (Sensory Input)    │  ┌──────────────┐  │
│  - Sensor Agent  │                               │  │   RustNPU    │  │
│  - Motor Agent   │         ZMQ SUB              │  │              │  │
│  - etc...        │◄──────── (Motor Output)      │  │ - Burst Loop │  │
│                  │                               │  │ - Learning   │  │
└──────────────────┘                               │  │ - Synapses   │  │
                                                   │  └──────────────┘  │
                                                   │                    │
                                                   │  ┌──────────────┐  │
                                                   │  │  Connectome  │  │
                                                   │  │   Storage    │  │
                                                   │  └──────────────┘  │
                                                   └────────────────────┘
```

### Components

- **Agent Registry** (TODO) - Manages agent registration and capabilities
- **ZMQ Communication** (TODO) - Handles multi-agent messaging
- **RustNPU** - Core neural processing unit with burst execution
- **Connectome Serializer** - Saves/loads complete brain state
- **External Agents** - Separate processes handling I/O (see `video_agent_rust/`)

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

### Burst Frequency
- Default 50Hz is suitable for most applications
- Increase to 100Hz+ for faster reaction times
- Decrease to 20-30Hz for lower CPU usage on embedded systems

### Agent Configuration
- Preprocess sensory data in agents before sending to reduce bandwidth
- Agents handle all I/O blocking - inference engine never blocks
- Multiple agents can run on separate machines for distributed processing

### Memory Usage
- Connectome size depends on neuron/synapse count
- Typical brain: 10K neurons, 100K synapses = ~10MB connectome file
- Runtime memory: 2-3x connectome size
- ZMQ messaging uses minimal memory overhead

## Use Cases

### Robotics
Run FEAGI inference engine on embedded controllers (Raspberry Pi, NVIDIA Jetson) with agents handling sensors, cameras, and actuators over ZMQ.

### Edge Computing
Deploy pure neural processing at the edge, with agents running on higher-powered machines or cloud for I/O-intensive tasks.

### Distributed Systems
Run inference engine in Docker/Kubernetes with agents as separate microservices, enabling horizontal scaling and fault tolerance.

### Research & Development
Rapid iteration on neural architectures - modify agents without touching neural core, or vice versa.

### Multi-Modal Processing
Connect multiple agents (vision, audio, tactile, proprioception) simultaneously, each handling their own I/O and preprocessing.

## Limitations & Future Work

### Current Limitations
- **ZMQ agent registration not yet implemented** (agents cannot register dynamically)
- **No sensory input processing** - agents can't yet send data
- **No motor output publishing** - agents can't receive commands
- Connectome must be pre-built (no runtime genome modification)

### Planned Features (High Priority)
- **Agent registry** - REQ/REP socket for dynamic agent registration
- **Sensory input** - PULL socket for receiving agent data
- **Motor output** - PUB socket for publishing motor commands
- **Agent health monitoring** - detect and handle agent disconnections

### Future Enhancements
- Real-time genome modification and cortical area creation
- Distributed processing across multiple inference engines
- WebAssembly compilation for browser-based inference
- Performance metrics and monitoring endpoints

## Troubleshooting

### "Failed to load connectome"
Ensure the connectome file exists and was created with compatible FEAGI version. Connectome format versioning ensures backward compatibility.

### "Agent registration not implemented" (Current State)
The ZMQ agent registration system is not yet implemented. The inference engine currently only runs the NPU burst loop. See the "Planned Features" section above.

### Agents Can't Connect
1. Verify ZMQ library is installed (`libzmq3-dev` on Ubuntu, `zeromq` on macOS)
2. Check firewall rules if running agents on different machines
3. Ensure port numbers match between engine and agents (defaults: 5000/5555/5556)

### Low Performance / High CPU
Lower `--burst-hz` or optimize the connectome size. Monitor neuron/synapse counts - very large brains require more processing power. Agent I/O does not impact inference engine performance (decoupled design).

## Contributing

The FEAGI Inference Engine is part of the FEAGI 2.0 architecture. Contributions should follow the project's Rust/RTOS compatibility guidelines.

See the main FEAGI repository for contribution guidelines and architecture documentation.

## License

Copyright 2025 Neuraville Inc.  
Licensed under the Apache License, Version 2.0

## Related Documentation

- [FEAGI Core README](../../README.md) - Main FEAGI documentation
- [Video Agent](../../../../video_agent_rust/) - Example Python video agent implementation
- [Connectome Serialization](../feagi-connectome-serialization/) - Connectome file format
- [Burst Engine](../feagi-burst-engine/) - Neural processing engine
- [Architecture Guide](/feagi_core/docs/) - FEAGI 2.0 architecture overview

## Example Agents

See the `video_agent_rust/` directory for a complete example of a ZMQ-based agent that reads video files and communicates with the inference engine. This serves as a template for creating additional agents (audio, sensors, motor controllers, etc.).

