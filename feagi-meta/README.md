# FEAGI - Framework for Evolutionary Artificial General Intelligence

Complete FEAGI SDK with Brain Visualizer included.

## Installation

```bash
pip install feagi
```

This installs:
- ✅ **feagi-core** - SDK for building agents and controlling FEAGI
- ✅ **Brain Visualizer** - Real-time 3D neural activity visualization

## Quick Start

```bash
# Start FEAGI with barebones genome
feagi start

# Launch Brain Visualizer
feagi bv start
```

## When to Use This Package

Use `feagi` (this package) when you want:
- 🎨 Visual development and debugging
- 📊 Real-time neural activity monitoring
- 🎓 Learning and tutorials
- 🎮 Interactive demos

## Alternative: feagi-core (Slim)

For production deployments, CI/CD, or when you don't need visualization:

```bash
pip install feagi-core
```

The `feagi-core` package:
- 📦 **Smaller** - ~5MB vs ~196MB
- 🚀 **Faster installs** - Great for containers
- 🔧 **Same imports** - All code examples work identically
- ⚡ **Perfect for** - Production, inference-only, edge devices, CI/CD

## Imports Work Identically

Both packages use the same import namespace:

```python
from feagi.agent import FeagiAgent
from feagi.pns import PNSClient
from feagi.engine import FeagiEngine
```

## Features

- **Agent SDK** - Build sensory/motor agents for robotics and simulations
- **Engine Control** - Start/stop FEAGI neural engine programmatically
- **PNS Client** - Connect to FEAGI's Peripheral Nervous System
- **Brain Visualizer** - 3D real-time visualization (included in this package)
- **CLI Tools** - `feagi` and `feagi bv` commands

## Documentation

- [Full Documentation](https://docs.feagi.org)
- [API Reference](https://docs.feagi.org/api)
- [Tutorials](https://docs.feagi.org/tutorials)

## License

Apache License 2.0
