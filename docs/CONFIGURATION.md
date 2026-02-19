# FEAGI Agent Configuration Guide

## FEAGI 2.0 Architecture: Single Source of Truth

Following FEAGI 2.0 architecture principles, **ALL agents must use configuration files** as their single source of truth. No hardcoded defaults are allowed in agent code.

## Quick Start

### 1. Create config.toml

Copy the template to your agent directory:

```bash
cp examples/agent_config.toml.template your_agent/config.toml
```

Edit `config.toml` with your settings:

```toml
[feagi]
host = "127.0.0.1"
registration_port = 5563  # Rust PNS registration
sensory_port = 5558       # Rust PNS sensory input

  [feagi.sensory_socket]
  send_hwm = 1
  linger_ms = 0
  immediate = true

[agent]
log_level = "INFO"

[capabilities]
# Your agent-specific configuration here
```

### 2. Load Configuration in Your Agent

```python
from feagi_connector import load_agent_config, FeagiAgentClient

# Load config.toml from agent's directory
config = load_agent_config()

# Access configuration
feagi_host = config["feagi"]["host"]
registration_port = config["feagi"]["registration_port"]

# Create FEAGI client with config values
# NOTE: agent_id must be a base64 AgentDescriptor (48-byte payload).
client = FeagiAgentClient(
    agent_id="<agent_descriptor_b64>",
    feagi_host=feagi_host,
    registration_port=registration_port,
    agent_type=AgentType.Sensory
)
```

### 3. CLI Overrides (Optional)

Configuration file is the default, CLI arguments are **overrides only**:

```python
import argparse
from feagi_connector import load_agent_config, merge_cli_args

# Load base config
config = load_agent_config()

# Parse CLI args with config defaults
parser = argparse.ArgumentParser()
parser.add_argument("--feagi-host", default=config["feagi"]["host"])
parser.add_argument("--registration-port", type=int, 
                   default=config["feagi"]["registration_port"])
args = parser.parse_args()

# Merge CLI overrides into config
config = merge_cli_args(config, args)
```

## Configuration Structure

### Required Sections

#### [feagi]
Connection settings for FEAGI server:

```toml
[feagi]
host = "127.0.0.1"              # FEAGI host address
registration_port = 5563         # Rust PNS registration (ZMQ REQ/REP)
sensory_port = 5558              # Rust PNS sensory input (ZMQ PUSH/PULL)

  [feagi.sensory_socket]
  send_hwm = 1                   # ZMQ PUSH high-water mark (must be >= 0)
  linger_ms = 0                  # Drop unsent frames on shutdown
  immediate = true               # Fail fast if FEAGI not ready (real-time safety)
```

**Port Reference (FEAGI 2.0 Rust PNS)**:
- **5563**: Agent registration and heartbeat (ZMQ REQ/REP)
- **5558**: Sensory data input (ZMQ PUSH)
- **5562**: Motor data output (ZMQ PULL)  
- **5555**: Visualization data (ZMQ PUB)

### Optional Sections

#### [agent]
Agent-specific settings:

```toml
[agent]
id = "my-agent-1"               # Optional: Agent ID (auto-generated if omitted)
log_level = "INFO"              # DEBUG, INFO, WARNING, ERROR
```

#### [capabilities]
Agent capabilities (customizable per agent type):

```toml
# Vision agent example
[capabilities.vision]
cortical_area = "iic400"
width = 64
height = 64
frequency = 30.0

# Motor agent example
[capabilities.motor]
dof = 6                         # degrees of freedom
control_mode = "position"       # position, velocity, torque
```

## API Reference

### load_agent_config()

Load configuration from TOML file (fail-fast if missing).

```python
def load_agent_config(config_path: Optional[str] = None) -> Dict[str, Any]
```

**Parameters**:
- `config_path` (optional): Path to config file. If None, looks for `config.toml` in calling script's directory.

**Returns**: Dictionary containing configuration

**Raises**: `SystemExit` if config is missing or invalid

**Example**:
```python
from feagi_connector import load_agent_config

# Auto-detect config.toml in agent directory
config = load_agent_config()

# Or specify custom path
config = load_agent_config("/path/to/custom_config.toml")
```

### validate_feagi_config()

Validate that configuration contains required FEAGI connection fields.

```python
def validate_feagi_config(config: Dict[str, Any]) -> bool
```

**Parameters**:
- `config`: Configuration dictionary

**Returns**: True if valid

**Raises**: `ValueError` if required fields are missing

**Example**:
```python
from feagi_connector import load_agent_config, validate_feagi_config

config = load_agent_config()
validate_feagi_config(config)  # Raises ValueError if invalid
```

### merge_cli_args()

Merge CLI arguments into configuration (CLI overrides config.toml).

```python
def merge_cli_args(config: Dict[str, Any], args: Any) -> Dict[str, Any]
```

**Parameters**:
- `config`: Base configuration from TOML
- `args`: Parsed `argparse.Namespace` with CLI arguments

**Returns**: Updated configuration with CLI overrides applied

**Example**:
```python
import argparse
from feagi_connector import load_agent_config, merge_cli_args

config = load_agent_config()

parser = argparse.ArgumentParser()
parser.add_argument("--feagi-host")
parser.add_argument("--agent-id")
args = parser.parse_args()

# Merge CLI overrides
config = merge_cli_args(config, args)
```

### get_config_template()

Get a standard FEAGI agent configuration template string.

```python
def get_config_template() -> str
```

**Returns**: String containing template TOML configuration

**Example**:
```python
from feagi_connector import get_config_template

# Generate template and save
template = get_config_template()
with open("config.toml", "w") as f:
    f.write(template)
```

## Architecture Principles

### Why Configuration Files?

Following FEAGI 2.0 architecture, configuration files ensure:

1. ✅ **Single Source of Truth** - One place for all settings
2. ✅ **No Hardcoded Defaults** - Eliminates hidden assumptions
3. ✅ **Cross-Platform Compatibility** - Works in Docker, K8s, embedded, desktop
4. ✅ **Deterministic Behavior** - Same config = same behavior
5. ✅ **Easy Deployment** - Just change config.toml, no code changes
6. ✅ **Future Rust Migration** - Compatible with RTOS requirements

### Fail-Fast Philosophy

**Bad (Hidden Failures)**:
```python
# ❌ DON'T: Hardcoded defaults hide misconfiguration
host = os.getenv("FEAGI_HOST", "localhost")  # WRONG!
port = int(os.getenv("FEAGI_PORT", "30001"))  # WRONG!
```

**Good (Explicit Failures)**:
```python
# ✅ DO: Fail immediately if config is missing
config = load_agent_config()  # Exits with error if no config.toml
host = config["feagi"]["host"]  # Fails if host not in config
```

### CLI Override Pattern

**Configuration Hierarchy**:
1. **config.toml** - Base configuration (required)
2. **CLI arguments** - Override specific values (optional)
3. **Environment variables** - Only for deployment automation (optional)

**Example**:
```bash
# Use config.toml defaults
python agent.py

# Override specific values
python agent.py --feagi-host 192.168.1.100

# Deployment automation (CI/CD)
FEAGI_HOST=prod-server python agent.py
```

## Migration Guide

### Old Pattern (DEPRECATED)

```python
# ❌ OLD: Hardcoded defaults everywhere
def __init__(self, feagi_host="localhost", port=30001, ...):
    self.host = feagi_host
    self.port = port
```

### New Pattern (FEAGI 2.0)

```python
# ✅ NEW: Config file is the single source of truth
from feagi_connector import load_agent_config

config = load_agent_config()

def __init__(self, config):
    self.host = config["feagi"]["host"]
    self.port = config["feagi"]["registration_port"]
```

## Examples

See complete examples:
- [`video_agent/agent.py`](../../video_agent/agent.py) - Vision agent with config
- [`simple_agent/agent.py`](../../simple_agent/agent.py) - Minimal agent with config
- [`agent_config.toml.template`](../examples/agent_config.toml.template) - Standard template

## Troubleshooting

### "Configuration file not found"

**Error**:
```
ERROR: Configuration file not found: /path/to/agent/config.toml
FEAGI 2.0 Architecture Requirement:
  - All agents must have a config.toml file
```

**Solution**:
```bash
cd your_agent/
cp ../examples/agent_config.toml.template config.toml
# Edit config.toml with your settings
```

### "Missing required field"

**Error**:
```
ValueError: Missing required field: feagi.registration_port
```

**Solution**: Add the missing field to your `config.toml`:
```toml
[feagi]
host = "127.0.0.1"
registration_port = 5563  # <-- Add this
sensory_port = 5558
```

### Port Confusion

**FEAGI 2.0 uses different ports than legacy FEAGI**:

| Service | FEAGI 2.0 (Rust PNS) | Legacy (Python) |
|---------|---------------------|-----------------|
| Registration | **5563** (REQ/REP) | 30001 (HTTP) |
| Sensory | **5558** (PUSH) | 3000 (HTTP) |
| Motor | **5562** (PULL) | 3000 (HTTP) |
| Visualization | **5555** (PUB) | 30003 (ZMQ) |

**Make sure your config.toml uses FEAGI 2.0 ports!**

## See Also

- [FEAGI 2.0 Architecture Rules](../../feagi_core/CONTRIBUTING.md)
- [Agent Client Documentation](./AGENT_CLIENT.md)
- [Configuration System (feagi-core)](../../feagi_core/feagi/config/toml_loader.py)

