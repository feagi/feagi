# System-Level Tests

This directory contains system-level tests that validate FEAGI's architecture, integration, and compliance with design principles.

## Architecture Compliance Tests

### `test_architecture_compliance.py`

Validates that FEAGI follows OS/platform agnostic principles essential for cross-platform deployment:

#### **Host Configuration Compliance**
- **No hardcoded network hosts**: Tests that `127.0.0.1`, `localhost` are not hardcoded
- **Required explicit configuration**: All hosts must come from TOML configuration or environment variables
- **Environment variable overrides**: Tests that `FEAGI_API_HOST`, `FEAGI_ZMQ_HOST` properly override empty config

#### **Timeout Configuration Compliance**
- **Configurable timeouts**: All system timeouts must be configurable via TOML
- **No hardcoded timing**: Tests elimination of hardcoded values like `time.sleep(5)`, `timeout=30`
- **Deployment flexibility**: Supports different timeout requirements for embedded vs cloud environments

#### **Port Configuration Compliance**
- **Explicit port specification**: All ports must be explicitly configured in TOML
- **Port validation**: Tests that invalid port values are rejected
- **No auto-discovery**: Eliminates port conflict auto-resolution for predictable deployment

#### **TOML Configuration Structure**
- **Empty host defaults**: Validates that `feagi_configuration.toml` requires explicit host configuration
- **Timeout sections**: Ensures all required timeout configurations are present
- **Cross-platform compatibility**: Tests configuration loading on different platforms

## Running Architecture Compliance Tests

```bash
# Run all architecture compliance tests
pytest tests/system/test_architecture_compliance.py -v

# Run specific compliance marker tests
pytest -m architecture -v
pytest -m compliance -v

# Run architecture tests with detailed output
pytest tests/system/test_architecture_compliance.py -v -s

# Run only host configuration compliance tests
pytest tests/system/test_architecture_compliance.py::TestHostConfigurationCompliance -v
```

## Test Categories

The tests are organized into several compliance categories:

### Unit Tests (`@pytest.mark.unit`)
- Configuration validation
- Host requirement enforcement
- Timeout configuration loading
- Port specification validation

### Integration Tests (`@pytest.mark.integration`)
- TOML file structure validation
- Environment variable override testing
- Cross-platform data directory handling
- End-to-end configuration loading

## Architecture Validation Results

When all tests pass, FEAGI is validated to be:

✅ **Platform Agnostic** - No hardcoded OS-specific paths or behaviors
✅ **Network Configurable** - All hosts and ports come from explicit configuration
✅ **Deployment Flexible** - Configurable timeouts for different environments
✅ **Container Ready** - Works in Docker, Kubernetes, and cloud environments
✅ **Embedded Compatible** - Configurable for resource-constrained systems

## Adding New Compliance Tests

When adding new architecture compliance tests:

1. **Follow the existing pattern**: Use the established test class structure
2. **Add appropriate markers**: Use `@pytest.mark.unit` or `@pytest.mark.integration`
3. **Test both positive and negative cases**: Ensure invalid configurations are rejected
4. **Document the compliance requirement**: Explain what architecture principle is being validated
5. **Update this README**: Document new test categories and their purpose

## Integration with CI/CD

These tests should be run in continuous integration to prevent architecture regressions:

```yaml
# Example CI configuration
- name: "Architecture Compliance Tests"
  run: |
    pytest tests/system/test_architecture_compliance.py -v
    pytest -m architecture --tb=short
```

## Related Documentation

- `/docs/coding_guidelines.md` - Overall coding standards
- `/docs/deployment.md` - Deployment configuration guide
- `feagi_configuration.toml` - Main configuration file
- `/docs/architecture.md` - System architecture overview

## Test Structure

```
system/
  ├── integration/           # System integration tests
  │   ├── test_neuroembryogenesis.py      # Tests for brain development process
  │   └── test_synaptogenesis.py          # Tests for synapse formation across modules
  └── performance/           # System-level performance benchmarks
      └── (future performance tests)
```

## Running Tests

### Integration Tests

```bash
# Use the provided script
./run_integration_tests.sh

# With additional pytest arguments
./run_integration_tests.sh -v
```

## Purpose of System Tests

System-level tests differ from module-specific tests in that they:

1. **Cross module boundaries**: Test interactions between BDU, NPU, API and other modules
2. **Exercise complete workflows**: Test end-to-end processes like genome loading through brain development
3. **Validate system behavior**: Ensure the system behaves correctly as a whole

## Test Guidelines

1. **Focused scope**: While these tests span multiple modules, each should still have a focused purpose
2. **Minimal dependencies**: Minimize external dependencies to make tests more reliable
3. **Realistic scenarios**: Test scenarios should reflect actual usage patterns
4. **Clear expectations**: Define clear pass/fail criteria

## System Test Coverage

The system tests aim to cover:

- End-to-end brain development from genome to functioning brain
- Cross-module interactions like BDU to NPU handoff
- Integration of neurogenesis and synaptogenesis
- Complete workflows from API through core functionality
- System resiliency and error handling

## Adding New System Tests

When adding new system tests:

1. Place the test in the appropriate subdirectory based on its purpose (integration, performance)
2. Focus on testing cross-module functionality rather than internal details
3. Use descriptive test names that explain what system behavior is being tested
4. Consider adding the test to the run script if it should be run regularly
