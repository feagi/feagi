# FEAGI Test Utilities

This directory contains shared utility functions that can be used across all FEAGI test suites (unit, integration, performance).

## Available Utilities

### Backend Utilities (`backend_utils.py`)

Functions for backend detection and system information:

- `is_webgpu_available()`: Check if WebGPU backend is available on this system
- `is_cuda_available()`: Check if CUDA backend is available on this system
- `get_available_backends()`: Get a list of all available backend names
- `get_system_info()`: Get detailed information about the current system

### Usage

To use these utilities in your tests:

```python
# Add the FEAGI root to path to ensure imports work correctly
import sys
import os

# Makes tests directory available in sys.path
tests_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if tests_path not in sys.path:
    sys.path.insert(0, tests_path)

# Import specific functions
from tests.utils.backend_utils import is_webgpu_available, get_system_info

# Use in your code
if is_webgpu_available():
    # Use WebGPU backend
    pass
    
# Get system information
system_info = get_system_info()
```

## Benefits of Shared Utilities

- **Maintainability**: Centralized implementation reduces code duplication
- **Consistency**: All tests use the same detection and system info methods
- **Extensibility**: Easy to add new utility functions when needed 