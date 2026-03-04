# FEAGI Python SDK Deployment Guide

Complete guide to installing, configuring, and deploying FEAGI across different platforms and use cases.

---

## Table of Contents

- [Quick Start (2 Lines)](#quick-start-2-lines)
- [Installation Options](#installation-options)
- [Directory Structure](#directory-structure)
- [Configuration](#configuration)
- [Platform-Specific Notes](#platform-specific-notes)
- [Advanced Deployment](#advanced-deployment)
- [Troubleshooting](#troubleshooting)

---

## Quick Start (3 Lines)

The fastest way to get started with FEAGI and Brain Visualizer:

```bash
pip install feagi
feagi start
feagi bv start
```

That's it! This will:
1. Install FEAGI SDK and Brain Visualizer
2. Create default configuration automatically
3. Launch Brain Visualizer connected to your FEAGI instance

**What happens behind the scenes:**
- Default configuration is created in `~/.feagi/config/feagi_configuration.toml`
- Directory structure is initialized (genomes, connectomes, logs, cache)
- Brain Visualizer launches with sensible defaults

---

## Installation Options

### Full (Recommended for Development)

```bash
pip install feagi
```

Includes Brain Visualizer for real-time neural activity visualization (~196MB). **Recommended for most users.**

Use this for:
- Visual development and debugging
- Learning FEAGI
- Interactive demos
- Tutorials

### Slim/Core (Recommended for Production)

```bash
pip install feagi-core
```

SDK only without Brain Visualizer (~5MB). Use this for:
- Production deployments
- Docker containers
- CI/CD pipelines
- Inference-only systems
- Edge devices
- Server deployments without GUI

> **Note:** Both packages use identical imports (`from feagi import ...`)

### Add Optional Features

With either package, add extra features:

```bash
# Video processing (OpenCV)
pip install feagi-core[video]  # or feagi[video]

# Bluetooth support
pip install feagi-core[bluetooth]  # or feagi[bluetooth]

# Everything
pip install feagi[full]
```

---

## Directory Structure

FEAGI uses a hybrid approach for organizing files:
- **System files** (config, logs, cache) in hidden directory
- **User content** (genomes, connectomes) in visible directory

### Linux

```
Hidden (System):
~/.feagi/
├── config/
│   └── feagi_configuration.toml    # FEAGI configuration
├── logs/                            # Runtime logs
└── cache/                           # Temporary data

Visible (User Content):
~/FEAGI/
├── genomes/                         # Brain templates
└── connectomes/                     # Trained brain states
```

**Example paths:**
- Config: `/home/username/.feagi/config/feagi_configuration.toml`
- Genomes: `/home/username/FEAGI/genomes/`
- Logs: `/home/username/.feagi/logs/`

### macOS

```
Hidden (System):
~/.feagi/
├── config/
│   └── feagi_configuration.toml
├── logs/
└── cache/

Visible (User Content):
~/Documents/FEAGI/
├── Genomes/                         # Capitalized on macOS
└── Connectomes/
```

**Example paths:**
- Config: `/Users/username/.feagi/config/feagi_configuration.toml`
- Genomes: `/Users/username/Documents/FEAGI/Genomes/`
- Logs: `/Users/username/.feagi/logs/`

### Windows

```
Hidden (System):
C:\Users\username\.feagi\
├── config\
│   └── feagi_configuration.toml
├── logs\
└── cache\

Visible (User Content):
C:\Users\username\Documents\FEAGI\
├── Genomes\
└── Connectomes\
```

**Example paths:**
- Config: `C:\Users\username\.feagi\config\feagi_configuration.toml`
- Genomes: `C:\Users\username\Documents\FEAGI\Genomes\`
- Logs: `C:\Users\username\.feagi\logs\`

### Directory Philosophy

**Why this structure?**
- **Hidden directories** (`.feagi`) keep system files out of the way
- **Visible directories** (`FEAGI` or `Documents/FEAGI`) make user content easily accessible
- User content (genomes/connectomes) can be:
  - Easily browsed and managed
  - Backed up to cloud storage (Dropbox, Google Drive)
  - Version controlled (Git)
  - Shared with collaborators
  - Copied to USB drives

---

## Configuration

### Initialization

Initialize FEAGI environment and create default configuration:

```bash
feagi init
```

This creates all directories and generates default `feagi_configuration.toml`.

**Output:**
```
FEAGI environment initialized successfully!

Configuration: ~/.feagi/config/feagi_configuration.toml
Genomes:       ~/Documents/FEAGI/Genomes/
Connectomes:   ~/Documents/FEAGI/Connectomes/
Logs:          ~/.feagi/logs/
Cache:         ~/.feagi/cache/

Next steps:
  1. Edit configuration if needed:
     ~/.feagi/config/feagi_configuration.toml
  2. Start Brain Visualizer:
     feagi bv start
```

### Configuration Options

The `feagi init` command supports several options:

```bash
# Generate config only (no directory creation)
feagi init --config-only

# Overwrite existing configuration
feagi init --force

# Generate config in custom location
feagi init --output ./my_config.toml
```

### Configuration File Structure

Default `feagi_configuration.toml`:

```toml
[api]
host = "127.0.0.1"
port = 8000

[websocket]
host = "127.0.0.1"
visualization_port = 8080
sensory_port = 5558
motor_port = 5564

[burst_engine]
# Burst duration in seconds (0.01 = 10ms bursts)
burst_duration = 0.01

# Maximum bursts to execute (0 = unlimited)
max_bursts = 0

# Enable GPU acceleration if available
gpu_enabled = false

[performance]
# Number of worker threads for parallel processing
worker_threads = 4

# Enable performance profiling
profiling_enabled = false

[logging]
# Log level: debug, info, warning, error
level = "info"

# Enable file logging
file_logging = true

[connectome]
# Maximum neuron space allocation
neuron_space = 1000000

# Maximum synapse space allocation
synapse_space = 10000000
```

### Common Configuration Modifications

#### 1. Network Configuration (Docker/Container)

For Docker deployments, bind to all interfaces:

```toml
[api]
host = "0.0.0.0"  # Accept connections from any IP
port = 8000
```

#### 2. Development Configuration (Local Only)

For local development, bind to localhost:

```toml
[api]
host = "127.0.0.1"  # Local connections only
port = 8000
```

#### 3. Performance Tuning

For large brains or high-throughput scenarios:

```toml
[burst_engine]
burst_duration = 0.005  # 5ms bursts (faster)

[performance]
worker_threads = 8  # More parallel processing

[connectome]
neuron_space = 10000000     # 10M neurons
synapse_space = 100000000   # 100M synapses
```

#### 4. GPU Acceleration

Enable GPU processing (requires compatible hardware):

```toml
[burst_engine]
gpu_enabled = true
```

#### 5. Debug Mode

Enable detailed logging for troubleshooting:

```toml
[logging]
level = "debug"
file_logging = true
profiling_enabled = true
```

---

## Platform-Specific Notes

### Linux

#### Permissions

No special permissions required. All directories are in user space.

#### Hidden Files

The `.feagi` directory is hidden by default. To view:

```bash
ls -la ~/
```

Or enable "Show Hidden Files" in your file manager.

#### Distribution-Specific Notes

**Ubuntu/Debian:**
```bash
# Install Python and pip first
sudo apt update
sudo apt install python3 python3-pip

# Then install FEAGI
pip3 install feagi[bv]
```

**Fedora/RHEL:**
```bash
sudo dnf install python3 python3-pip
pip3 install feagi[bv]
```

### macOS

#### Permissions

No admin privileges required. Uses standard user directories.

#### Hidden Files

The `.feagi` directory is hidden. To view in Finder:
- Press `Cmd + Shift + .` to toggle hidden files

Or use Terminal:
```bash
ls -la ~/
```

#### macOS-Specific Configuration

Brain Visualizer on macOS may require accessibility permissions for certain features. Grant when prompted.

### Windows

#### Permissions

No administrator privileges required.

#### Hidden vs Visible

Note that `.feagi` directory (with leading dot) is **visible** in Windows Explorer by default, unlike Linux/macOS. This is normal Windows behavior - it's treated as a regular directory name.

To truly hide it (optional):
```powershell
attrib +h C:\Users\YourUsername\.feagi
```

#### Windows-Specific Configuration

If using Windows Subsystem for Linux (WSL), install FEAGI inside WSL for better compatibility:

```bash
# Inside WSL
pip install feagi[bv]
```

---

## Advanced Deployment

### Network Binding (0.0.0.0 vs 127.0.0.1)

By default, FEAGI binds to `127.0.0.1` for local-only development to avoid
OS firewall prompts and unintentional external access. For container or
remote client access, bind to all interfaces:

```toml
[api]
host = "0.0.0.0"  # Accept connections from any IP
port = 8000

[websocket]
host = "0.0.0.0"  # Required for remote BV/agent connections
```

When using `0.0.0.0`, ensure your firewall rules allow only the intended
ports and networks.

### Custom Configuration Location

Use a custom configuration file:

```bash
# Generate custom config
feagi init --output ./my_project/config.toml

# Start with custom config
feagi bv start --config ./my_project/config.toml
```

### Python API Usage

For programmatic control:

```python
from feagi.config import init_feagi_environment
from feagi.engine import FeagiEngine

# Initialize environment
env = init_feagi_environment()

# Start FEAGI with default config
engine = FeagiEngine()
engine.load_config()  # Uses default config
engine.load_genome("my_brain.json")
engine.start()
```

### Working with Genomes

#### Storing Genomes

Save genomes in the standard directory:

**Linux:**
```bash
cp my_brain.json ~/FEAGI/genomes/
```

**macOS/Windows:**
```bash
cp my_brain.json ~/Documents/FEAGI/Genomes/
```

#### Loading Genomes

Load genomes by filename (auto-resolved from genomes directory):

```python
from feagi.engine import FeagiEngine

engine = FeagiEngine()
engine.load_genome("my_brain.json")  # Looks in ~/Documents/FEAGI/Genomes/
engine.start()
```

Or use absolute path:

```python
engine.load_genome("/path/to/genome.json")
```

### Working with Connectomes

#### Saving Trained Brains

Connectomes are saved brain states (trained neural networks). Save them to the standard directory for easy access:

**Linux:**
```bash
mv trained_v1.connectome ~/FEAGI/connectomes/
```

**macOS/Windows:**
```bash
mv trained_v1.connectome ~/Documents/FEAGI/Connectomes/
```

#### Loading Connectomes

Load by filename:

```python
from feagi.engine import FeagiEngine

engine = FeagiEngine()
engine.load_connectome("trained_v1.connectome")  # Auto-resolved
engine.start()
```

### Docker Deployment

For containerized deployment:

**Dockerfile example:**
```dockerfile
FROM python:3.10-slim

# Install FEAGI
RUN pip install feagi[full]

# Initialize environment
RUN python -c "from feagi.config import init_feagi_environment; init_feagi_environment()"

# Copy your configuration (optional)
COPY feagi_configuration.toml /root/.feagi/config/

# Expose ports
EXPOSE 8000 8080 5558 5564

# Start FEAGI
CMD ["feagi", "start", "--config", "/root/.feagi/config/feagi_configuration.toml", "--genome", "/data/genome.json"]
```

**docker-compose.yml example:**
```yaml
version: '3.8'
services:
  feagi:
    image: python:3.10-slim
    command: >
      bash -c "pip install feagi[full] &&
               feagi init &&
               feagi start --genome /data/genome.json"
    ports:
      - "8000:8000"
      - "8080:8080"
      - "5558:5558"
      - "5564:5564"
    volumes:
      - ./genomes:/data
      - feagi-data:/root/.feagi
volumes:
  feagi-data:
```

### Multi-User / Server Deployment

For server environments with multiple users:

Each user gets their own isolated environment:
- Config: `~user/.feagi/config/`
- Genomes: `~user/FEAGI/genomes/` (Linux) or `~user/Documents/FEAGI/Genomes/` (macOS/Windows)

No conflicts between users.

### Production Deployment Checklist

- [ ] Initialize FEAGI environment: `feagi init`
- [ ] Review and customize configuration
- [ ] Test with sample genome
- [ ] Configure firewall rules for ports 8000, 8080, 5558, 5564
- [ ] Set up log rotation for `~/.feagi/logs/`
- [ ] Configure backup for `~/Documents/FEAGI/` (user content)
- [ ] Document custom configuration for team
- [ ] Set up monitoring (optional)

---

## Troubleshooting

### Config File Not Found

**Problem:** `Config file not found: feagi_configuration.toml`

**Solution:**
```bash
# Initialize environment to create default config
feagi init
```

### Permission Denied

**Problem:** `Permission denied` when creating directories

**Solution:**
- All FEAGI directories are in user space and require no special permissions
- Ensure you're running as regular user (not root)
- Check home directory permissions: `ls -ld ~/`

### Brain Visualizer Won't Start

**Problem:** Brain Visualizer fails to launch

**Solutions:**

1. **Check installation:**
```bash
pip list | grep feagi
# Should show: feagi, feagi-bv-{platform}
```

2. **Reinstall Brain Visualizer:**
```bash
pip install --force-reinstall feagi[bv]
```

3. **Check configuration:**
```bash
cat ~/.feagi/config/feagi_configuration.toml
# Verify [api] and [websocket] sections exist
```

4. **Run with explicit config:**
```bash
feagi bv start --config ~/.feagi/config/feagi_configuration.toml
```

### Genome/Connectome Not Found

**Problem:** `Genome file not found: my_brain.json`

**Solutions:**

1. **Check file location:**
```bash
# Linux
ls ~/FEAGI/genomes/

# macOS/Windows
ls ~/Documents/FEAGI/Genomes/
```

2. **Use absolute path:**
```python
engine.load_genome("/full/path/to/genome.json")
```

3. **Copy to standard directory:**
```bash
# Linux
cp genome.json ~/FEAGI/genomes/

# macOS/Windows
cp genome.json ~/Documents/FEAGI/Genomes/
```

### Port Already in Use

**Problem:** `Address already in use: 8000`

**Solutions:**

1. **Check what's using the port:**
```bash
# Linux/macOS
lsof -i :8000

# Windows (PowerShell)
netstat -ano | findstr :8000
```

2. **Change port in config:**
```toml
[api]
port = 8001  # Use different port
```

3. **Stop conflicting service:**
```bash
# Find and kill the process
kill <PID>
```

### Viewing Hidden Directories

**Problem:** Can't find `.feagi` directory

**Solutions:**

**Linux:**
```bash
ls -la ~/
```

**macOS Finder:**
- Press `Cmd + Shift + .` to toggle hidden files

**Windows:**
- `.feagi` is visible by default in Explorer
- Navigate to `C:\Users\YourUsername\.feagi\`

---

## Getting Help

- **Documentation**: [https://github.com/feagi/feagi/tree/main/docs](https://github.com/feagi/feagi/tree/main/docs)
- **Discord**: [Join our community](https://discord.gg/PTVC8fyGN8)
- **Issues**: [GitHub Issues](https://github.com/feagi/feagi-python-sdk/issues)
- **Homepage**: [feagi.org](https://feagi.org)

---

**Copyright 2016-2025 Neuraville Inc. All Rights Reserved.**
