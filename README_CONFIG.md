# FEAGI 2.0: Standardized Configuration System

## ✅ What Was Implemented

We've created a **standardized configuration loader** for all FEAGI agents, following FEAGI 2.0 architecture principles.

### Files Created/Modified

#### New Files in feagi-connector:
1. **`feagi_connector/utils/config_loader.py`** - Standard config loader module
2. **`agent_config.toml.template`** - Template for all agents
3. **`docs/CONFIGURATION.md`** - Complete documentation

#### Updated Files:
1. **`feagi_connector/__init__.py`** - Exports config loader functions
2. **`feagi_connector/pyproject.toml`** - Added `toml` dependency
3. **`video_agent/agent.py`** - Now uses standardized config loader
4. **`video_agent/config.toml`** - Updated with correct ports

## 🎯 Benefits

### For Agent Developers:
- ✅ **Simple** - Just call `load_agent_config()`, no boilerplate
- ✅ **Consistent** - Same pattern across all agents
- ✅ **Template** - Copy `agent_config.toml.template` and customize

### For FEAGI Architecture:
- ✅ **Single source of truth** - No hardcoded defaults
- ✅ **Fail-fast** - Missing config = immediate error (not silent failure)
- ✅ **Cross-platform** - Works in Docker, K8s, embedded, desktop
- ✅ **Rust-compatible** - Prepares for future RTOS migration

## 📦 Installation

### 1. Update feagi-connector

```bash
cd /Users/nadji/code/FEAGI-2.0/feagi-connector
pip install -e .
```

This will install the `toml` dependency automatically.

### 2. For Existing Agents

If you have an existing virtual environment:

```bash
cd your_agent/
source your_venv/bin/activate
pip install toml  # Or reinstall feagi-connector with: pip install -e ../feagi-connector
```

## 🚀 Usage

### Quick Start for New Agent

```bash
# 1. Copy template
cp feagi-connector/agent_config.toml.template my_agent/config.toml

# 2. Edit config.toml with your settings
nano my_agent/config.toml

# 3. Use in agent code
```

```python
from feagi_connector import load_agent_config, FeagiAgentClient

# Load config (auto-detects config.toml in agent directory)
config = load_agent_config()

# Create client with config values
client = FeagiAgentClient(
    agent_id="my-agent",
    feagi_host=config["feagi"]["host"],
    registration_port=config["feagi"]["registration_port"],
    sensory_port=config["feagi"]["sensory_port"],
    agent_type=AgentType.Sensory
)
```

### With CLI Overrides

```python
import argparse
from feagi_connector import load_agent_config, merge_cli_args

# Load base config
config = load_agent_config()

# Parse CLI with config defaults
parser = argparse.ArgumentParser()
parser.add_argument("--feagi-host", default=config["feagi"]["host"])
args = parser.parse_args()

# Merge CLI overrides
config = merge_cli_args(config, args)
```

## 📖 API Reference

### Exported Functions

All available from `feagi_connector`:

```python
from feagi_connector import (
    load_agent_config,      # Load config.toml
    validate_feagi_config,  # Validate required fields
    get_config_template,    # Get template string
    merge_cli_args,         # Merge CLI overrides
)
```

See [`docs/CONFIGURATION.md`](docs/CONFIGURATION.md) for complete API documentation.

## 🔧 Configuration Structure

### Minimal config.toml

```toml
[feagi]
host = "127.0.0.1"
registration_port = 5563  # Rust PNS registration (ZMQ REQ/REP)
sensory_port = 5558       # Rust PNS sensory input (ZMQ PUSH)

[agent]
log_level = "INFO"
```

### Full Example (video_agent)

```toml
[feagi]
host = "127.0.0.1"
registration_port = 5563
sensory_port = 5558

[agent]
log_level = "INFO"

[video]
path = "driving.mp4"
frequency = 30.0

[capabilities]
cortical_area = "iic400"
```

## ⚡ Next Steps

### For Agent Developers

1. **Copy template** to your agent directory:
   ```bash
   cp feagi-connector/agent_config.toml.template your_agent/config.toml
   ```

2. **Update your agent** to use the loader:
   ```python
   from feagi_connector import load_agent_config
   config = load_agent_config()
   ```

3. **Remove hardcoded defaults** from your code

### For Testing

```bash
cd video_agent
python agent.py  # Uses config.toml automatically
python agent.py --feagi-host 192.168.1.100  # Override specific values
```

## 🎓 Architecture Principles

This implementation follows FEAGI 2.0 architecture rules:

1. **No hardcoded network addresses** - All from config
2. **No hardcoded timeouts** - All from config (future enhancement)
3. **Single source of truth** - config.toml only
4. **Fail-fast** - No silent fallbacks
5. **Cross-platform** - Same config works everywhere

See: [FEAGI 2.0 Architecture Compliance Rules](../feagi_core/CONTRIBUTING.md)

## 📚 Documentation

- **Full Guide**: [`docs/CONFIGURATION.md`](docs/CONFIGURATION.md)
- **Template**: [`agent_config.toml.template`](agent_config.toml.template)
- **Example**: [`video_agent/config.toml`](../video_agent/config.toml)
- **Source**: [`feagi_connector/utils/config_loader.py`](feagi_connector/utils/config_loader.py)

## 🐛 Troubleshooting

### "ModuleNotFoundError: No module named 'toml'"

**Solution**: Reinstall feagi-connector:
```bash
cd feagi-connector
pip install -e .
```

### "Configuration file not found"

**Solution**: Create config.toml from template:
```bash
cp feagi-connector/agent_config.toml.template your_agent/config.toml
```

### Wrong ports (connection failed)

**Make sure you're using FEAGI 2.0 ports**:
- `registration_port = 5563` (NOT 30001)
- `sensory_port = 5558` (NOT 3000)

---

**Status**: ✅ Ready for use by all FEAGI agents

