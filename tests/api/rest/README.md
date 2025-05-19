# FEAGI REST API Tests

This directory contains tests for the FEAGI REST API. The tests are organized using pytest and follow a structured approach to maximize performance and reusability.

## Test Framework Design

The test framework is designed with the following principles:

1. **Version-Based Organization**: Tests are organized into version-specific directories (v1, v2) to support backward compatibility.
2. **Session-Scoped Initialization**: Expensive components like the ConnectomeManager are initialized only once per test session.
3. **Test Grouping via Markers**: Tests are grouped by API functionality to share initialization and mocks.
4. **Client Factory Pattern**: Specialized test clients can be created while reusing expensive components.
5. **Complete Mocking**: All external dependencies are mocked to avoid actual initialization and I/O operations.
6. **Client Caching**: Client instances are cached by group to minimize re-initialization.

## Test Structure

- `/v1/` and `/v2/`: API version-specific test directories
- `conftest.py`: Version-specific shared fixtures and mocking logic
- `test_*_api.py`: Individual test files for different API endpoints
- `run_patched_tests.py`: Version-specific script to run tests with lightweight mocks
- `run_api_tests.py`: Main dispatcher script to run tests across all versions

### Test Groups

Tests are organized into the following groups:

- `burst_engine`: Tests for the Burst Engine API
- `brain_state`: Tests for the Brain State API
- `genome`: Tests for the Genome API
- `mapping`: Tests for the Cortical Mapping API
- `region`: Tests for the Brain Region API
- `default`: Default group for tests that don't fit elsewhere

## Running Tests

### Using run_api_tests.py

The easiest way to run tests is using the `run_api_tests.py` script:

```bash
# Run all v1 tests
python run_api_tests.py

# Run all v2 tests
python run_api_tests.py --version v2

# Run all tests for both v1 and v2
python run_api_tests.py --version all

# Run tests with verbose output
python run_api_tests.py -v

# Run tests for a specific API (e.g., genome)
python run_api_tests.py -p genome

# Run tests for a specific group
python run_api_tests.py -g brain_state

# Run tests in parallel (4 processes)
python run_api_tests.py -j 4

# List available test groups
python run_api_tests.py --list-groups

# Use the patched conftest with lightweight mocks for faster testing
python run_api_tests.py --use-patched
```

### Using pytest directly

You can also run tests using pytest directly:

```bash
# Run all tests
pytest

# Run tests for a specific API
pytest test_genome_api.py

# Run tests from a specific group
pytest -m "api_group('brain_state')"
```

## Adding New Tests

### Adding Tests to Existing Files

1. Add your test function to the appropriate test file
2. Make sure to use the appropriate client fixture
3. Use the pattern `assert response.status_code in (200, 404)` to allow for unimplemented endpoints

Example:

```python
def test_new_endpoint(genome_client):
    """Test description."""
    response = genome_client.get("/v1/some/endpoint")
    assert response.status_code in (200, 404)
    
    if response.status_code == 200:
        data = response.json()
        assert "expected_field" in data
```

### Creating a New Test Group

1. Add a new client fixture in `conftest.py`
2. Update the `use_appropriate_client` fixture to include your new group

Example:

```python
@pytest.fixture(scope="module")
def new_group_client(client_factory):
    """Client specialized for new group tests."""
    def _customize_core_api(mock):
        # Add group-specific mock customizations
        mock.some_method.return_value = {"key": "value"}
    
    return client_factory("new_group", core_api_customizer=_customize_core_api)

# Then update use_appropriate_client
@pytest.fixture(autouse=True)
def use_appropriate_client(request):
    # ... existing code ...
    elif group_name == "new_group":
        request.getfixturevalue("new_group_client")
```

## Best Practices

1. **Mock Expensive Operations**: Always mock external services and I/O operations
2. **Use Client Fixtures**: Use the appropriate client fixture for your test group
3. **Handle Missing Endpoints**: Use `assert response.status_code in (200, 404)` to handle missing endpoints
4. **Group Related Tests**: Keep related tests in the same file and group
5. **Reuse Mock Data**: Define mock data in conftest.py for reuse across tests
6. **Use Session-Scoped Fixtures**: For expensive setup operations

## Performance Tips

- Use the `-j` option with `run_api_tests.py` to run tests in parallel
- Group tests appropriately to minimize re-initialization
- Use client caching to avoid recreating clients
- Make sure tests are independent and can run in any order
- Keep test setup minimal and focused on the specific test case 