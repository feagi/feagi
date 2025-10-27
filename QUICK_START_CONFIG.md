# Quick Start: Standardized Configuration System

## What You Asked For

> "I want you to have load config as a standard helper in feagi-connector so all agents can have a simple toml template"

## What Was Built ✅

### 1. Standard Configuration Loader

**File**: `feagi_connector/utils/config_loader.py`

```python
from feagi_connector import load_agent_config

# That's it! Auto-detects config.toml in your agent directory
config = load_agent_config()
```

### 2. Universal Template

**File**: `agent_config.toml.template`

```toml
[feagi]
host = "127.0.0.1"
registration_port = 5563  # Rust PNS
sensory_port = 5558

[agent]
log_level = "INFO"

[capabilities]
# Your agent-specific config here
```

### 3. Automatic Export

Just import from `feagi_connector`:

```python
from feagi_connector import (
    load_agent_config,      # Load config.toml
    validate_feagi_config,  # Validate structure
    get_config_template,    # Get template
    merge_cli_args,         # CLI overrides
)
```

### 4. Complete Documentation

- **Full Guide**: `docs/CONFIGURATION.md` (comprehensive)
- **Overview**: `README_CONFIG.md` (quick reference)
- **This File**: `QUICK_START_CONFIG.md` (quickest start)

## Usage (3 Steps)

### Step 1: Copy Template

```bash
cp feagi-connector/agent_config.toml.template my_agent/config.toml
```

### Step 2: Edit config.toml

```toml
[feagi]
host = "127.0.0.1"
registration_port = 5563
sensory_port = 5558

[my_settings]
# Add your agent-specific config
value = 123
```

### Step 3: Use in Agent

```python
from feagi_connector import load_agent_config

config = load_agent_config()

# Access values
host = config["feagi"]["host"]
my_value = config["my_settings"]["value"]
```

## Already Integrated

✅ **video_agent** - Already updated to use the new system!

See: `/Users/nadji/code/FEAGI-2.0/video_agent/agent.py`

## Architecture Benefits

Following FEAGI 2.0 principles:

1. ✅ **Single source of truth** - config.toml only
2. ✅ **No hardcoded defaults** - Fail-fast if config missing
3. ✅ **Cross-platform** - Works everywhere
4. ✅ **Consistent** - All agents use same pattern
5. ✅ **Template-driven** - Easy onboarding

## Next: Test Video Agent

The video agent is now ready to test with the new configuration system:

```bash
cd video_agent
source video_venv/bin/activate
pip install toml  # One-time install
python agent.py driving.mp4
```

It will:
1. Load `config.toml` (host, ports)
2. Apply CLI overrides (if any)
3. Connect to FEAGI on correct ports (5563, 5558)

## For Other Agents

Just copy the pattern from `video_agent/agent.py`:

```python
from feagi_connector import load_agent_config

# In main()
config = load_agent_config()

# In argparse
parser.add_argument("--feagi-host", default=config["feagi"]["host"])
```

Done! 🎉
