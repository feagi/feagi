# FEAGI

**F**ramework for **E**volutionary **A**rtificial **G**eneral **I**ntelligence

## Overview

FEAGI is a Python-based framework for developing and deploying artificial general intelligence models. It provides a flexible architecture that combines Python's ease of use with Rust's performance for computationally intensive operations.

## Features

- **FastAPI Backend**: High-performance API server for model deployment and interaction
- **Hybrid Architecture**: Core functionality in Python with performance-critical components in Rust
- **Extensible Design**: Easily add new model types, training methods, and inference strategies
- **Comprehensive Testing**: Built with pytest to ensure reliability and stability
- **Well-Documented**: Complete API documentation and usage examples

## Installation

```bash
pip install feagi
```

## Quick Start

```python
from feagi import FEAGI

# Initialize a new FEAGI instance
feagi = FEAGI()

# Create a simple model
model = feagi.create_model("simple_model")

# Train the model
model.train(data, epochs=10)

# Make predictions
predictions = model.predict(test_data)
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
pip install -e ".[dev]"
```

### Running Tests

```bash
pytest
```

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details. 