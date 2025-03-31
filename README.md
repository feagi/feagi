# FEAGI

**F**ramework for **E**volutionary **A**rtificial **G**eneral **I**ntelligence

## Overview

FEAGI is a Python-based framework for developing and deploying artificial general intelligence models. It provides a flexible architecture that combines Python's ease of use with Rust's performance for computationally intensive operations.

## Features

- **FastAPI Backend**: High-performance API server for model deployment and interaction
- **ZMQ Messaging**: Efficient inter-component communication using ZeroMQ
- **Hybrid Architecture**: Core functionality in Python with performance-critical components in Rust
- **Extensible Design**: Easily add new model types, training methods, and inference strategies
- **Comprehensive Testing**: Built with pytest to ensure reliability and stability
- **Well-Documented**: Complete API documentation and usage examples
- **Memory Profiling**: Built-in tools for tracking memory usage
- **Benchmarking**: Performance measurement tools for identifying bottlenecks

## Installation

```bash
pip install feagi
```

## Running FEAGI

FEAGI provides multiple ways to run the system depending on your needs:

### Unified Command (Recommended)

The simplest way to run FEAGI is with the unified command that starts both the API and ZMQ servers:

```bash
# Start both API and ZMQ servers with default settings
feagi

# Custom configuration
feagi --api-port 8080 --zmq-pub-port 5566 --zmq-sub-port 5567
```

### Component-Specific Commands

You can also run specific components individually:

```bash
# Run only the API server
feagi --api-only
feagi-api

# Run only the ZMQ server
feagi --zmq-only
feagi-zmq
```

### Advanced Configuration

```bash
# See all available options
feagi --help
feagi-api --help
feagi-zmq --help

# Run with custom host and port
feagi --api-host 0.0.0.0 --api-port 8888

# Run ZMQ with specific topics
feagi --zmq-topics neural metrics heartbeat system
```

### Python Module Usage

You can also run FEAGI as Python modules:

```bash
# Full system
python -m feagi.main

# Individual components
python -m feagi.api.server
python -m feagi.zmq.server
```

## Programming Interface

```python
from feagi import create_feagi

# Initialize a new FEAGI instance
feagi = create_feagi()

# Access components
resource_mgr = feagi["resource_mgr"]
```

## Documentation

Comprehensive documentation is available at [docs/](./docs/).

## Development

### Setup Development Environment

```bash
# Clone the repository
git clone https://github.com/yourusername/feagi.git
cd feagi

# Create a virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install development dependencies
pip install -e .
```

### Optional: Rust Integration

For performance-critical components, FEAGI can leverage Rust:

```bash
# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Install the project with Rust extensions
pip install -e .
```

### Running Tests

```bash
pytest
```

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details. 