# FEAGI Core

FEAGI (Framework for Evolutionary Artificial General Intelligence) Core is the main neural simulation engine that provides brain-like processing capabilities.

## Quick Start

### Installation
```bash
# Install FEAGI core with dependencies
pip install -e .

# Start FEAGI with default configuration
python -m feagi.main
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

## Documentation

- [Usage Guide](docs/guide-usage.md) - Complete usage instructions
- [Architecture Guide](docs/arch-zmq.md) - ZMQ communication architecture
- [Embedded Mode](docs/arch-embedded-mode.md) - Resource-constrained deployments
- [Configuration Reference](docs/configuration.md) - Complete configuration options

## License

Copyright 2025 Neuraville Inc. Licensed under the Apache License, Version 2.0.
