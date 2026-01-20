# FEAGI Python SDK

**Build AI agents that learn like biological brains**

[![PyPI](https://img.shields.io/pypi/v/feagi)](https://pypi.org/project/feagi/) [![Python](https://img.shields.io/pypi/pyversions/feagi)](https://pypi.org/project/feagi/) [![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

---

## Installation Options

**🎨 Full Experience (Recommended for Development):**
```bash
pip install feagi
```
Includes Brain Visualizer for real-time 3D neural activity visualization (~196MB)

**⚡ Slim/Core (Recommended for Production):**
```bash
pip install feagi-core
```
SDK only without Brain Visualizer - perfect for containers, CI/CD, and inference (~5MB)

> **Note:** Both packages use identical imports (`from feagi import ...`)

---

## What is FEAGI?

**FEAGI (Framework for Evolutionary Artificial General Intelligence)** is a biologically inspired, modular neural execution engine designed for **embodied AI and robotics**. FEAGI enables spiking-neural-circuit-driven perception, cognition, and control across simulated and physical embodiments, with a strong emphasis on **real-time interaction, modularity, and cross-platform deployment**.

FEAGI serves as the core neural runtime behind **Neurorobotics Studio**, powering a growing ecosystem of reusable neural components ("brains"), tools, and integrations for robotics and physical AI.

### The FEAGI Python SDK

The FEAGI Python SDK provides the tools you need to:

- **Connect robots and devices** to FEAGI's neural network
- **Build learning agents** for robots, simulators, and games
- **Visualize neural activity** in real-time with Brain Visualizer
- **Control and manage FEAGI** from Python code
- **Interface with diverse embodiments** through standardized communication protocols

---

## Key Concepts

* **Neuromorphic by Design** – FEAGI is built as a neuromorphic framework inspired by biological neural computation. While it currently runs on conventional CPUs and GPUs, **native support for neuromorphic hardware is a near-term roadmap item**, enabling direct execution on event-driven, spike-based accelerators as they mature.

* **Embodied Intelligence First** – FEAGI is designed to control bodies (robots, agents, simulations), not just process static data.

* **Spiking Neural Networks (SNNs)** – Uses event-driven neuron firing rather than frame-based inference.

* **Modular Neural Architecture** – Neural circuits can be composed like building blocks (Lego-like micro-circuits).

* **Real-Time Closed Loop** – Continuous perception → cognition → action loop.

* **Cross-Simulator & Hardware Support** – One brain, many bodies.

---

## Core Capabilities

### Neural Processing

* Large-scale spiking neural networks
* Sparse connectivity and event-driven execution
* Fire Candidate Lists (FCL) and optimized firing pipelines
* Supports long-term and short-term memory constructs

### Modular Brain Design

* Pre-packaged micro-circuits (sensory, motor, cognitive)
* Community-developed and shareable neural components
* Easy composition into larger functional brains

### Embodiment Integration

FEAGI supports diverse embodiments through its flexible communication architecture:

* **Physical Robots** – AMRs, manipulators, service robots, drones
* **Virtual Simulations** – Webots, Gazebo, Unity, custom simulators
* **Game Engines** – Real-time game AI and NPCs
* **IoT Devices** – Sensors, actuators, embedded systems
* **Research Platforms** – Custom experimental setups

See the [embodiment-controllers repository](https://github.com/feagi/embodiment-controllers) for available integrations.

### Performance & Portability

* CPU multiprocessing support
* GPU acceleration paths
* Designed for future support of:
  * Vulkan
  * WebGPU
  * WebAssembly (WASM)
* Sparse matrix and bit-packed data representations

---

## Typical Architecture

```
[Sensors]
    ↓
[FEAGI Sensory Cortical Areas]
    ↓
[Cognitive / Associative Circuits]
    ↓
[Motor Cortical Areas]
    ↓
[Actuators]
```

FEAGI runs as the **neural execution engine**, while adapters (built with this SDK) translate between FEAGI's neural signals and the embodiment's sensors and actuators.

---

## Example Use Cases

* Autonomous mobile robots (AMRs)
* Service robots (cleaning, logistics, inspection)
* Robotic manipulation and dexterous hands
* AI-driven simulation agents
* Research in biologically inspired AI
* Education and rapid prototyping of embodied intelligence
* Game AI that adapts to players
* Educational neuroscience simulations

---

## Quick Start

Get started with FEAGI in just 2 lines:

```bash
pip install "feagi[bv]"
feagi bv start
```

That's it! This installs FEAGI with Brain Visualizer, creates default configuration automatically, and launches the visualizer.

> **Note for zsh users (macOS default):** Use quotes around `"feagi[bv]"` to avoid shell glob expansion errors.

**For detailed installation options, configuration, and platform-specific notes, see [DEPLOY.md](./DEPLOY.md).**

### Build Your First Agent

```python
from feagi.agent import BaseAgent

class MyRobotAgent(BaseAgent):
    def initialize_hardware(self):
        # Connect to your robot/simulator
        pass
    
    def map_sensors(self, hw_data):
        # Send sensor data to FEAGI
        return {"camera": image_bytes}
    
    def map_motors(self, feagi_output):
        # Control motors from FEAGI commands
        return motor_commands

# Run it
agent = MyRobotAgent("my-robot")
await agent.connect()
await agent.run()
```

---

## Documentation

- **[Deployment Guide](./DEPLOY.md)** - Complete installation, configuration, and platform-specific notes
- [Getting Started Guide](https://docs.feagi.org/getting-started)
- [Agent Development Tutorial](https://docs.feagi.org/agents)
- [API Reference](https://docs.feagi.org/api)
- [Examples](./examples/)

---

<details>
<summary><b>Advanced Usage</b></summary>

### Configuration Management

Initialize FEAGI environment with default configuration:

```bash
feagi init
```

This creates:
- Configuration: `~/.feagi/config/feagi_configuration.toml`
- Genomes directory: `~/Documents/FEAGI/Genomes/` (macOS/Windows) or `~/FEAGI/genomes/` (Linux)
- Connectomes directory: `~/Documents/FEAGI/Connectomes/` or `~/FEAGI/connectomes/`
- Logs and cache directories

**For complete configuration options and customization, see [DEPLOY.md](./DEPLOY.md).**

### Optional Extras

Install additional features as needed:

```bash
# Video processing (OpenCV)
pip install "feagi[video]"

# Bluetooth support
pip install "feagi[bluetooth]"

# All extras
pip install "feagi[full]"
```

> **zsh users:** Always use quotes around package names with brackets.

### Direct PNS Communication

For low-level control over FEAGI communication:

```python
from feagi.pns import FeagiAgentClient, AgentType

client = FeagiAgentClient("my-agent", AgentType.SENSORY)
client.configure(feagi_host="localhost", feagi_api_port=8000)
await client.connect()

# Send sensory data
await client.send_sensory_data({
    "camera": image_data,
    "lidar": distance_readings
})

# Receive motor commands
motor_data = await client.receive_motor_data()
```

### Start FEAGI Engine from Python

```python
from feagi.engine import FeagiEngine

engine = FeagiEngine()
engine.load_config()  # Uses default config
engine.load_genome("my_brain.json")  # Loads from genomes directory
engine.start()
```

Or from command line:

```bash
feagi start --config ~/.feagi/config/feagi_configuration.toml --genome my_brain.json
```

### SDK Architecture

```
feagi/
├── agent/           # Agent framework (BaseAgent)
├── pns/             # Peripheral Nervous System (communication)
├── engine/          # Engine control
├── config/          # Configuration management
├── paths/           # Cross-platform path utilities
├── cli/             # Command-line tools
├── genome/          # Runtime genome manipulation (coming soon)
├── connectome/      # Brain state management (coming soon)
└── packaging/       # Marketplace packages (coming soon)
```

### Migration from 1.x

If you're upgrading from `feagi_connector`:

```python
# Old (feagi_connector)
from feagi_connector import FeagiAgentClient

# New (feagi 2.x)
from feagi.pns import FeagiAgentClient
```

**Breaking changes:**
- Package renamed: `feagi_connector` → `feagi`
- Python 3.10+ required
- Legacy APIs removed

</details>

---

## Examples

See [`examples/`](./examples/) for complete agent implementations:
- Basic sensory agent
- Robot agent (SDK-based)
- Simulator agent (Webots)
- Vision processing

---

## Neurorobotics Studio

FEAGI's official desktop application, **Neurorobotics Studio**, provides an integrated development environment for building and deploying neural brains:

* Visual brain design and editing tools
* Brain marketplace for sharing and discovering neural components
* Embodiment management and configuration
* Experiment orchestration and monitoring
* Community-driven neural circuit library

While this SDK enables programmatic access and custom integrations, Neurorobotics Studio offers a complete graphical workflow for those who prefer visual development tools.

---

## Design Philosophy

* **Biology-inspired, not biology-constrained**
* **Performance-aware from day one**
* **Composable over monolithic**
* **Embodiment-agnostic intelligence**
* **Developer-first robotics AI**

---

## Roadmap Highlights

* Expanded GPU and Vulkan backends
* WASM/WebGPU execution for browser-based robotics
* Standardized neural component formats
* Advanced long-term memory mechanisms
* Deeper autonomy stack integrations
* Native neuromorphic hardware support

---

## Community & Support

- **Discord**: [Join our community](https://discord.gg/PTVC8fyGN8)
- **Issues**: [Report bugs](https://github.com/feagi/feagi-python-sdk/issues)
- **Neurorobotics Studio**: [Cloud platform](https://neurorobotics.studio)
- **Homepage**: [feagi.org](https://feagi.org)

---

## Contributing

Contributions are welcome! This includes:

* Neural circuit modules
* Performance optimizations
* New embodiment adapters
* Documentation and examples
* Agent implementations and examples

Please follow the contribution guidelines in [CONTRIBUTING.md](CONTRIBUTING.md).

---

## Requirements

- Python 3.10 or higher
- Works on Linux, macOS, and Windows

---

## License

Apache 2.0 - See [LICENSE](LICENSE) for details.

**Copyright 2016-2025 Neuraville Inc. All Rights Reserved.**

---

## About Neuraville

FEAGI is developed by **Neuraville**, a company focused on democratizing robotics and enabling the next generation of embodied AI through modular, biologically inspired intelligence systems.
