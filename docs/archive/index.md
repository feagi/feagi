# FEAGI Documentation

Welcome to the FEAGI documentation. FEAGI (**F**ramework for **E**volutionary **A**rtificial **G**eneral **I**ntelligence) is a Python-based framework for developing and deploying artificial general intelligence models.

## Overview

FEAGI provides a flexible architecture that combines Python's ease of use with Rust's performance for computationally intensive operations. It was designed to be modular, extensible, and easy to use while still providing the performance required for training and deploying AI models.

## Installation

You can install FEAGI using pip:

```bash
pip install feagi
```

For development installations (including optional dependencies):

```bash
pip install "feagi[dev]"
```

> **Note:** Use quotes for zsh users (macOS default shell).

## Quick Start

Here's a simple example of how to use FEAGI:

```python
from feagi import FEAGI

# Initialize a new FEAGI instance
feagi = FEAGI()

# Create a simple model
model = feagi.create_model("simple_model")

# Train the model with some data
training_data = [
    {"features": [1.0, 2.0, 3.0], "label": 0},
    {"features": [4.0, 5.0, 6.0], "label": 1},
]
model.train(training_data, epochs=10)

# Make predictions
test_data = [{"features": [1.1, 2.1, 3.1]}]
predictions = model.predict(test_data)
print(f"Predictions: {predictions}")
```

For more detailed examples, see the [Examples](examples.md) section.

## Core Components

FEAGI consists of several core components:

- **FEAGI**: The main entry point for creating and managing AI models.
- **Model**: Represents an AI model in the framework.
- **API**: A FastAPI-based API for interacting with FEAGI models via HTTP.
- **Rust Extensions**: Performance-critical operations implemented in Rust.

## API Documentation

For detailed API documentation, see the [API Reference](api_reference.md).

## Contributing

Contributions are welcome! See the [Contributing Guide](contributing.md) for more information.
