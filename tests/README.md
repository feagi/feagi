# FEAGI Tests

This directory contains all tests for the FEAGI project. Tests are organized according to the following structure:

## Test Organization

Tests are organized by module and test type:

```
tests/
  ├── bdu/                # Brain Development Unit tests
  │   ├── unit/           # Unit tests for BDU components
  │   ├── integration/    # Integration tests within BDU module
  │   └── performance/    # Performance benchmarks for BDU components
  ├── npu/                # Neural Processing Unit tests
  │   ├── unit/           # Unit tests for NPU components
  │   ├── integration/    # Integration tests within NPU module
  │   └── performance/    # Performance benchmarks for NPU components
  ├── api/                # API tests
  │   ├── unit/           # Unit tests for API components
  │   └── integration/    # Integration tests within API module
  └── system/             # Cross-module system tests
      ├── integration/    # Integration tests across multiple modules
      └── performance/    # System-level performance tests
```

## Test Types

- **Unit Tests**: Tests for individual components in isolation
- **Integration Tests**: Tests for interactions between components within a module
- **Performance Tests**: Benchmarks for performance-critical components
- **System Tests**: Tests that span multiple modules

## Running Tests

### Module-specific Tests

To run tests for a specific module:

```bash
# Run all BDU tests
python -m pytest feagi_core/tests/bdu

# Run only BDU unit tests
python -m pytest feagi_core/tests/bdu/unit
```

### Run BDU Non-GPU Tests

We provide a script to run all non-GPU tests for the BDU module:

```bash
# Run all non-GPU BDU tests
./feagi_core/tests/bdu/run_non_gpu_tests.sh

# Run with verbose output
./feagi_core/tests/bdu/run_non_gpu_tests.sh -v
```

### Run System Integration Tests

For system-level integration tests:

```bash
# Run all system integration tests
./feagi_core/tests/system/run_integration_tests.sh

# Run with verbose output
./feagi_core/tests/system/run_integration_tests.sh -v
```

## Test Guidelines

1. Always place tests in the appropriate module directory
2. Use proper test fixtures and setup/teardown
3. Write descriptive test names that explain what is being tested
4. Include both happy path and edge case tests
5. For performance tests, include baseline metrics and clear pass/fail criteria

## Test Development Guidelines

1. **Right Test, Right Place**: Place tests in the appropriate directory based on their purpose and scope.

2. **Fast Unit Tests**: Keep unit tests fast and resource-efficient. Use small data structures and avoid creating large arrays or matrices.

3. **Mocks and Fixtures**: Use pytest fixtures and mocks to avoid expensive setup/teardown operations.

4. **Test Isolation**: Each test should be independent and not rely on the state from previous tests.

5. **Performance Testing**: When testing performance, define clear metrics and thresholds.

6. **Backend Testing**: Use the backend wrapper to ensure tests run with the appropriate backend.

7. **Rust Binding Tests**: When testing Python-Rust bindings:
   - Use feature detection to handle missing Rust components gracefully
   - Test functionality parity between Python and Rust implementations
   - Verify that data conversions between Python and Rust work correctly

8. **Cross-Backend Testing**: When appropriate, test data transfer between different backends (e.g., WebGPU to CUDA).

9. **Logging Control**: Reduce logging levels in tests to avoid excessive output that slows down execution.

## Running Tests with Coverage

```bash
# Install pytest-cov if not already installed
pip install pytest-cov

# Run tests with coverage
pytest --cov=feagi tests/unit/

# Generate HTML coverage report
pytest --cov=feagi --cov-report=html tests/
```

## Debugging During Testing

FEAGI provides specialized debugging flags that can be helpful during test development and troubleshooting. For comprehensive debugging information, see the [FEAGI Debugging Guide](../docs/guide-how-to-debug.md).

### Quick Debug Commands for Testing

```bash
# NPU debugging during tests
FEAGI_DEBUG_NPU=1 python -m pytest tests/npu/ -v -s

# API debugging during tests
FEAGI_DEBUG_API=1 python -m pytest tests/api/ -v -s

# Combined debugging
FEAGI_DEBUG_API=1 FEAGI_DEBUG_NPU=1 python -m pytest -v -s

# Test mode with debugging
python -m feagi.main --test --debug-npu --test-duration 30
```

The debug flags provide valuable information during test development:
- `--debug-npu`: Shows real-time neuron firing data and fire queue contents
- `--debug-api`: Shows HTTP request/response details and API middleware operations

## Using Test Markers

We use pytest markers to categorize tests:

```python
@pytest.mark.performance
def test_large_scale_operation():
    # Performance-intensive test

@pytest.mark.slow
def test_time_consuming_operation():
    # Slow test that should be skipped in quick runs

@pytest.mark.cuda
def test_cuda_specific_function():
    # Test that only runs on CUDA backend

@pytest.mark.wgpu
def test_wgpu_specific_function():
    # Test that only runs on WebGPU backend

@pytest.mark.rust
def test_rust_implementation():
    # Test for Rust implementation

@pytest.mark.binding
def test_rust_python_binding():
    # Test for Python-Rust binding
```

To run tests with specific markers:

```bash
pytest -m performance
pytest -m "not slow"
pytest -m "wgpu and not slow"
pytest -m "binding and not cuda"
```
