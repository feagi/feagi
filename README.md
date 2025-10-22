# FEAGI Core

FEAGI (Framework for Evolutionary Artificial General Intelligence) Core is the main neural simulation engine that provides brain-like processing capabilities.

## Installation

### For Users (Recommended)

Install FEAGI from PyPI with pre-built binaries - **no Rust toolchain required**:

```bash
# 1. Create and activate virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# 2. Install FEAGI (includes pre-compiled Rust extensions + all dependencies)
pip install feagi

# 3. Start FEAGI
python -m feagi.main
```

**What gets installed automatically:**
- FEAGI core (pre-compiled Rust extensions included)
- All Python dependencies (FastAPI, NumPy, PyTorch, etc.)
- No manual dependency installation needed!

**Supported Platforms:**
- Linux (x86_64): Ubuntu 20.04+, CentOS 7+, Debian 10+
- macOS: Intel (10.9+) and Apple Silicon (11.0+)
- Windows: Windows 10+ (64-bit)
- Python: 3.8, 3.9, 3.10, 3.11, 3.12

### For Developers (From Source)

If you're developing FEAGI or on an unsupported platform:

**Prerequisites:**
- Python 3.8+
- Rust toolchain (install from https://rustup.rs/)

```bash
# 1. Clone the repository
git clone https://github.com/neuraville/feagi.git
cd feagi/feagi_core

# 2. Create and activate virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# 3. Install in development mode (builds Rust automatically)
pip install -e .

# 4. Start FEAGI
python -m feagi.main
```

The Rust extensions will be compiled automatically during `pip install -e .`

### Verify Installation

Once FEAGI starts, verify it's running:
```bash
# Check health endpoint
curl http://localhost:8000/v1/system/health_check

# Expected response: {"status":"healthy",...}
```

FEAGI is now ready to accept connections on:
- **REST API**: `http://localhost:8000` (web interface)
- **ZMQ Sensory**: `tcp://localhost:5558` (agent input)
- **ZMQ Motor**: `tcp://localhost:5564` (agent output)

## Quick Start

### Using Pre-built Package (No Rust Required)
```bash
# One command installs everything (FEAGI + all dependencies)
pip install feagi

# Ready to run immediately
python -m feagi.main
```

### Using Docker
```bash
docker pull feagi/feagi:latest
docker run -p 8000:8000 -p 5558:5558 -p 5564:5564 feagi/feagi:latest
```

### Default Ports
- **FastAPI REST API**: Port 8000 (HTTP)
- **ZMQ REST API**: Port 5563 (primary API interface)
- **ZMQ Visualization**: Port 5562 (neural data)
- **ZMQ Motor**: Port 5564 (motor output)
- **ZMQ Sensory**: Port 5558 (sensory input)

## Architecture

FEAGI Core implements a **Rust/RTOS compatible architecture** with:

- **Singleton ConnectomeManager**: Centralized brain state management
- **Direct Task Spawning**: No subprocess boundaries for optimal performance
- **Multi-Stream ZMQ**: Dedicated communication channels for different protocols
- **Memory-Mapped State**: Zero-copy data synchronization
- **Configuration-Driven**: All hosts and ports from TOML configuration

### Communication Protocols

**REST API over ZMQ (Port 5563)**: Primary interface for all API operations including:
- Agent registration and heartbeat
- Brain state queries and modifications
- System status and control
- Configuration management

**Specialized Data Streams**:
- **Visualization Stream (5562)**: Real-time neural activity broadcasting
- **Motor Stream (5564)**: High-frequency motor command output
- **Sensory Stream (5558)**: Sensory data input processing

## Configuration

FEAGI uses `feagi_configuration.toml` for all settings:

```toml
[api]
host = "0.0.0.0"
port = 8000

[ports]
zmq_rest_port = 5563        # Primary API interface
zmq_visualization_port = 5562
zmq_motor_port = 5564
zmq_sensory_port = 5558

[zmq]
host = "0.0.0.0"
```

Environment overrides available:
- `FEAGI_API_HOST`, `FEAGI_API_PORT`
- `FEAGI_ZMQ_HOST`

## Troubleshooting

### No Pre-built Wheel Available

If you see an error like:
```
ERROR: Could not find a version that satisfies the requirement feagi
```

Your platform may not have a pre-built wheel. Install Rust and build from source:

```bash
# Install Rust toolchain
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env

# Install FEAGI (will compile Rust extensions)
pip install feagi --no-binary feagi
```

### Verify Rust Extensions Loaded

```bash
python -c "from feagi import feagi_rust; print('✅ Rust extensions working')"
```

### Installation Takes Long Time

If installation is compiling from source (5-10 minutes), you likely don't have a pre-built wheel for your platform. This is expected on:
- ARM Linux (except aarch64)
- Alpine Linux (musl libc)
- FreeBSD, OpenBSD
- Python versions older than 3.8

## Development

### Running Tests
```bash
# Architecture compliance tests
pytest tests/system/test_architecture_compliance.py -v

# Unit tests
pytest tests/unit/ -v

# Integration tests
pytest tests/integration/ -v
```

### Debug Mode
```bash
# Enable API debugging
python -m feagi.main --debug-api

# Enable neural processing debugging
python -m feagi.main --debug-npu

# Full debug mode
python -m feagi.main --debug-api --debug-npu --log-level DEBUG
```

### Building Rust Extensions Manually

For development work on the Rust components:

```bash
cd feagi-rust
cargo build --release --workspace

# Copy extension to Python package
# Linux/macOS:
cp target/release/libfeagi_rust.so ../feagi/feagi_rust.so
# Windows:
copy target\release\feagi_rust.dll ..\feagi\feagi_rust.pyd
```

## Documentation

- [Usage Guide](docs/guide-usage.md) - Complete usage instructions
- [Architecture Guide](docs/arch-zmq.md) - ZMQ communication architecture
- [Embedded Mode](docs/arch-embedded-mode.md) - Resource-constrained deployments
- [Configuration Reference](docs/configuration.md) - Complete configuration options

## License

Copyright 2025 Neuraville Inc. Licensed under the Apache License, Version 2.0.
