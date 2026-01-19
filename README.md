# FEAGI Python SDK

**Complete SDK for building FEAGI agents, controlling the neural engine, and creating marketplace packages**

Version 2.0.0 - Framework for Evolutionary Artificial General Intelligence

## Overview

FEAGI SDK provides everything needed to:
- Build agents for robots, simulators, and embedded devices
- Start and stop the FEAGI neural engine from Python
- Manipulate genomes and connectomes at runtime
- Create packages for the FEAGI marketplace

## Installation

```bash
pip install feagi
```

Optional extras:
- `bv`: Brain Visualizer runtime
- `video`: OpenCV video helpers
- `bluetooth`: Bluetooth support
- `test`: Test dependencies
- `dev`: Dev tooling
- `docs`: Documentation tooling
- `full`: Common extras combined

For video processing support:
```bash
pip install feagi[video]
```

For Brain Visualizer (optional runtime):
```bash
pip install feagi[bv]
feagi bv start --config feagi_configuration.toml
```

Start FEAGI from Python CLI:
```bash
feagi start --config feagi_configuration.toml --genome my_genome.json
# or
feagi start --config feagi_configuration.toml --connectome trained_brain.connectome
```

Start Brain Visualizer from Python:
```python
from feagi_bv import BrainVisualizer

bv = BrainVisualizer()
bv.load_config("feagi_configuration.toml")
pid = bv.start()
```

## Quick Start

### Basic Agent

```python
from feagi.agent import BaseAgent
from feagi.pns import FeagiAgentClient, AgentType

class MyRobotAgent(BaseAgent):
    def initialize_hardware(self):
        # Connect to your robot/simulator
        pass
    
    def map_sensors(self, hw_data):
        # Convert robot sensors → FEAGI format
        return {"camera": image_bytes}
    
    def map_motors(self, feagi_output):
        # Convert FEAGI commands → robot format
        return motor_commands

# Run the agent
agent = MyRobotAgent("my-robot-01")
await agent.connect()
await agent.run()
```

### Direct PNS Communication

```python
from feagi.pns import FeagiAgentClient, AgentType

# Create client
client = FeagiAgentClient("my-agent", AgentType.SENSORY)
client.configure(feagi_host="localhost")
await client.connect()

# Send sensory data
await client.send_sensory_data(sensor_data)
```

## Architecture

```
feagi/
├── engine/          # Engine control (start/stop FEAGI)
├── agent/           # Agent framework (BaseAgent templates)
├── genome/          # Runtime genome manipulation
├── connectome/      # Runtime connectome operations
├── packaging/       # Build marketplace packages
├── pns/             # Peripheral Nervous System (communication)
└── cli/             # Command-line tools
```

## Modules

### `feagi.engine`
Start and stop the FEAGI neural engine from Python.
- `FeagiEngine` - Control local FEAGI instance
- PyO3 bindings to Rust `feagi::FeagiInstance`

**Status:** Planned for Phase 2

### `feagi.agent`
Agent framework with base classes and templates.
- `BaseAgent` - Abstract base class
- `SDKRobotAgent` - For SDK-based robots (Cozmo, NAO)
- `SimulatorAgent` - For physics simulators (Webots, Gazebo)
- `EmbeddedAgent` - For embedded devices (ESP32, Arduino)
- `VirtualAgent` - For game engines (Unity, Unreal)

**Status:** ✅ BaseAgent implemented

### `feagi.genome`
Manipulate genomes on a running FEAGI instance via REST API.
- `GenomeAPI` - Add/remove cortical areas, modify parameters
- `GenomeLoader` - Load genome from local file
- `GenomeValidator` - Validate genome structure

**Status:** Planned for Phase 3

### `feagi.connectome`
Download and upload connectomes via REST API.
- `ConnectomeAPI` - Snapshot/restore trained brain state
- Download from running instance for backup
- Upload pre-trained connectomes

**Status:** Planned for Phase 3

### `feagi.packaging`
Build marketplace packages locally.
- `PackageBuilder` - Create `.feagi-personality`, `.feagi-firmware`, etc.
- `PackageValidator` - Validate package structure
- `ManifestGenerator` - Generate manifest.json

**Status:** Planned for Phase 4

### `feagi.pns`
Peripheral Nervous System - Communication layer.
- `FeagiAgentClient` - Modern Rust-backed client (RECOMMENDED)
- `CapabilitiesManager` - Device capability management
- `MotorProcessor` - Motor command processing
- `VisionProcessor` - Vision data processing
- `SegmentedVisionProcessor` - Advanced segmented vision
- Supports ZMQ and WebSocket transports

**Status:** ✅ Fully implemented

### `feagi.cli`
Command-line tools for development.
- `feagi create-agent` - Scaffold new agent from template
- `feagi build-package` - Build marketplace package locally

**Status:** Planned for Phase 4

## Development Workflow

### 1. Create Agent
```bash
# Scaffold new agent (future)
feagi create-agent cozmo --template sdk_robot

# Edit generated agent code
vim cozmo_agent.py
```

### 2. Test Locally
```python
# Run agent with local FEAGI
python cozmo_agent.py --genome ~/.feagi/genomes/vision_nav.json
```

### 3. Build Package
```bash
# Create marketplace package (future)
feagi build-package \
    --genome vision_nav.json \
    --docs README.md \
    --output vision_nav_v1.feagi-personality
```

### 4. Upload via UI
Upload the package through Neurorobotics Studio or FEAGI Desktop.

## Examples

See `examples/` directory for complete agent implementations:
- Basic sensory agent
- SDK robot agent (Cozmo example)
- Simulator agent (Webots example)
- Vision processing examples

## Documentation

- [API Reference](https://docs.feagi.org/api)
- [Agent Development Guide](https://docs.feagi.org/agent-guide)
- [Marketplace Guide](https://docs.feagi.org/marketplace)

## What's New in 2.x

**Current architecture highlights:**
- ✅ Renamed from `feagi_connector` to `feagi`
- ✅ Modular structure: `engine`, `agent`, `genome`, `connectome`, `packaging`, `pns`, `cli`
- ✅ No legacy code, no deprecated APIs, no fallbacks
- ✅ Modern `BaseAgent` framework for agent development
- ✅ `feagi.pns` module for Peripheral Nervous System communication
- ✅ Prepared for engine control (PyO3 bindings in Phase 2)
- ✅ Prepared for runtime genome/connectome manipulation (Phase 3)
- ✅ Prepared for marketplace package building (Phase 4)

**Breaking Changes:**
- Package renamed: `feagi_connector` → `feagi`
- All legacy clients removed (`FeagiClient`, `FeagiAgentConnector`)
- Only modern `FeagiAgentClient` supported
- Python 3.10+ required
- Imports changed: `from feagi_connector import X` → `from feagi.pns import X`

## Migration from 1.x / feagi_connector

```python
# Old (feagi_connector)
from feagi_connector import FeagiAgentClient

# New (feagi 2.x)
from feagi.pns import FeagiAgentClient
```

## Requirements

- Python 3.10+
- `feagi_rust_py_libs` (for Rust-backed performance)
- NumPy 1.20+
- PyZMQ 24.0+
- aiohttp 3.9+

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## License

Apache-2.0 - See [LICENSE.txt](LICENSE.txt) for details.

## Links

- **Homepage**: https://feagi.org
- **Repository**: https://github.com/Neuraville/FEAGI-2.0
- **Issues**: https://github.com/Neuraville/FEAGI-2.0/issues
- **Neurorobotics Studio**: https://neurorobotics.studio
- **FEAGI Desktop**: Download from https://feagi.org

## Authors

Neuraville Inc. - <feagi@neuraville.com>

Copyright 2016-2025 Neuraville Inc. All Rights Reserved.
