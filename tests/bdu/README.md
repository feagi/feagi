# BDU (Brain Development Unit) Tests

This directory contains tests for the Brain Development Unit (BDU) module of FEAGI. The BDU is responsible for the construction and development of the neural structure based on genome specifications.

## Test Structure

```
bdu/
  ├── unit/              # Unit tests for individual BDU components
  │   ├── test_connectome_manager.py       # Tests for the main ConnectomeManager
  │   ├── test_connectome_manager_gpu.py   # Tests for GPU-accelerated ConnectomeManager
  │   ├── test_array_backend.py           # Tests for array backend abstractions
  │   ├── test_mixed_precision.py         # Tests for mixed precision operations
  │   └── test_multi_gpu.py               # Tests for multi-GPU distribution
  ├── integration/       # Integration tests within BDU
  │   └── test_connectome_manager.py      # Integration tests for ConnectomeManager
  └── performance/       # Performance benchmarks
      └── test_synaptogenesis_performance.py  # Performance tests for synapse creation
```

## Running Tests

### CPU-only Tests

To run only tests that don't require GPU capabilities (useful for development or CI environments without GPUs):

```bash
# Use the provided script
./run_non_gpu_tests.sh

# With additional pytest arguments
./run_non_gpu_tests.sh -v --no-header
```

### Unit Tests

```bash
# Run all unit tests
python -m pytest unit/

# Run specific test file
python -m pytest unit/test_connectome_manager.py

# Run a specific test
python -m pytest unit/test_connectome_manager.py::test_create_neuron
```

### Performance Tests

```bash
# Run all performance tests
python -m pytest performance/

# Run with benchmarking information
python -m pytest --benchmark-sort=name performance/
```

## Guidelines for BDU Tests

1. **Test isolation**: Each test should be independent and not rely on the state from other tests.
2. **Fixture usage**: Use fixtures to set up test environments, particularly for the connectome and test neurons.
3. **GPU vs CPU**: Mark GPU-specific tests appropriately so they can be skipped in environments without GPU support.
4. **Performance baselines**: Include clear performance expectations in benchmark tests.

## Test Coverage

BDU tests should cover:

- Creation and management of cortical areas
- Neuron creation and deletion
- Synapse formation and pruning
- Correct membrane potential updates
- Proper neuron firing behavior
- Correct serialization and deserialization
- Array backend abstractions
- GPU acceleration (when available)
- Multi-GPU distribution (when available) 