# FEAGI Test Framework

This directory contains test code for FEAGI, organized into a comprehensive structure that separates tests by their purpose, scope, and resource requirements.

## Test Organization

### Unit Tests (`tests/unit/`)
Fast, isolated tests of individual components without external dependencies. These verify that each unit of code works correctly in isolation.

- `bdu/`: Brain Dynamics Unit tests
- `npu/`: Neural Processing Unit tests
- `utils/`: Utility function tests
- `config/`: Configuration system tests
- `models/`: Model system tests

### Integration Tests (`tests/integration/`)
Tests that verify multiple components work correctly together.

- `bdu/`: BDU integration tests
- `npu/`: NPU integration tests
- `workflows/`: Tests for cross-component workflows

### API Tests (`tests/api/`)
Tests that verify the external API interfaces work correctly.

- `rest/`: REST API tests
- `grpc/`: gRPC API tests
- `websocket/`: WebSocket API tests
- `zmq/`: ZeroMQ communication tests

### Rust Tests (`tests/rust/`)
Tests specifically for the Rust implementations and Python-Rust bindings.

- `unit/`: Unit tests for Rust components
- `integration/`: Integration tests for Rust components
- `binding/`: Tests that verify Python-Rust bindings work correctly

### Performance Tests (`tests/performance/`)
Tests that verify the system's performance characteristics, scalability, and behavior under load.

- `bdu/`: BDU performance and scalability tests
- `npu/`: NPU performance and scalability tests
- `system/`: Whole-system performance tests

### Backend Tests (`tests/backends/`)
Tests specific to different computational backends.

- `cpu/`: Tests for CPU-specific behavior
- `cuda/`: Tests for CUDA GPU backends
- `wgpu/`: Tests for WebGPU backends
- `tensorrt/`: Tests for TensorRT acceleration

### End-to-End Tests (`tests/e2e/`)
Tests that verify complete workflows from input to output, simulating real-world usage.

## Running Tests

### Running Unit Tests

Unit tests should be run frequently during development as they're fast and focused:

```bash
# Run all unit tests
pytest tests/unit/

# Run specific unit tests
pytest tests/unit/bdu/
```

### Running Integration Tests

```bash
# Run all integration tests
pytest tests/integration/

# Run specific integration tests
pytest tests/integration/bdu/
```

### Running API Tests

```bash
# Run all API tests
pytest tests/api/

# Run specific API tests
pytest tests/api/rest/
```

### Running Rust Tests

```bash
# Run all Rust-related tests
pytest tests/rust/

# Run only Python-Rust binding tests
pytest tests/rust/binding/

# Run only Rust unit tests
pytest tests/rust/unit/
```

### Running Performance Tests

These tests are resource-intensive and slow. Run them less frequently:

```bash
# Run all performance tests
pytest tests/performance/

# Run with specific markers
pytest -m performance
```

### Running Backend-specific Tests

```bash
# Run tests for a specific backend
pytest tests/backends/cuda/

# Run WebGPU backend tests
pytest tests/backends/wgpu/

# Run with a specific backend configuration
FEAGI_BACKEND=cuda pytest tests/unit/
FEAGI_BACKEND=wgpu pytest tests/unit/
```

### Running End-to-End Tests

```bash
# Run all end-to-end tests
pytest tests/e2e/
```

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