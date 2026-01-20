# FEAGI Python SDK

**Build AI agents that learn like biological brains**

[![PyPI](https://img.shields.io/pypi/v/feagi)](https://pypi.org/project/feagi/) [![Python](https://img.shields.io/pypi/pyversions/feagi)](https://pypi.org/project/feagi/) [![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## What is FEAGI?

FEAGI (Framework for Evolutionary Artificial General Intelligence) is a brain-inspired AI engine that learns through experience, not training data. The FEAGI Python SDK lets you:

- **Connect robots and devices** to FEAGI's neural network
- **Build learning agents** for robots, simulators, and games
- **Visualize neural activity** in real-time with Brain Visualizer
- **Control FEAGI** from Python code

**Applications:**
- Autonomous robots that learn from interaction
- Game AI that adapts to players
- Research in neuromorphic computing
- Educational neuroscience simulations

---

## Quick Start

### Install FEAGI SDK with Brain Visualizer

```bash
pip install feagi[bv]
```

That's it! This installs everything you need.

### Launch Brain Visualizer

```bash
feagi bv start --config feagi_configuration.toml
```

Brain Visualizer will open and connect to your FEAGI instance automatically.

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

- [Getting Started Guide](https://docs.feagi.org/getting-started)
- [Agent Development Tutorial](https://docs.feagi.org/agents)
- [API Reference](https://docs.feagi.org/api)
- [Examples](./examples/)

---

<details>
<summary><b>Advanced Usage</b></summary>

### Optional Extras

Install additional features as needed:

```bash
# Video processing (OpenCV)
pip install feagi[video]

# Bluetooth support
pip install feagi[bluetooth]

# All extras
pip install feagi[full]
```

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

### Start FEAGI from Python

```bash
feagi start --config feagi_configuration.toml --genome my_brain.json
```

### Python API for Brain Visualizer

```python
from feagi_bv import BrainVisualizer

bv = BrainVisualizer()
bv.load_config("feagi_configuration.toml")
pid = bv.start()
```

### SDK Architecture

```
feagi/
├── agent/           # Agent framework (BaseAgent)
├── pns/             # Peripheral Nervous System (communication)
├── engine/          # Engine control
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

## Community & Support

- **Discord**: [Join our community](https://discord.gg/PTVC8fyGN8)
- **Issues**: [Report bugs](https://github.com/feagi/feagi-python-sdk/issues)
- **Neurorobotics Studio**: [Cloud platform](https://neurorobotics.studio)
- **Homepage**: [feagi.org](https://feagi.org)

---

## Contributing

We welcome contributions! See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

---

## Requirements

- Python 3.10 or higher
- Works on Linux, macOS, and Windows

---

## License

Apache 2.0 - See [LICENSE](LICENSE) for details.

**Copyright 2016-2025 Neuraville Inc. All Rights Reserved.**
