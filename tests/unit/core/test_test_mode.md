# Test Mode Tests

This directory contains tests for the FEAGI Test Mode module (`feagi/test_mode.py`).

## Purpose

The Test Mode module provides a way to run FEAGI in a test mode that:

1. Loads a genome
2. Generates and injects synthetic sensory data
3. Monitors neural activity
4. Reports test results

These tests verify that the Test Mode functionality works correctly by:

- Testing the `FeagiTestRunner` class methods
- Testing the `run_test_mode` function
- Verifying correct integration with FEAGI components
- Ensuring proper handling of test parameters

## Test Files

- `test_test_mode.py`: Unit tests for the test_mode module
- `../../../integration/workflows/test_test_mode_integration.py`: Integration tests for the test_mode module

## Running the Tests

### Unit Tests

To run the unit tests for the test_mode module:

```bash
cd feagi_core
pytest tests/unit/core/test_test_mode.py -v
```

### Integration Tests

To run the integration tests for the test_mode module:

```bash
cd feagi_core
pytest tests/integration/workflows/test_test_mode_integration.py -v
```

Note that some integration tests may be skipped by default if they require a fully functional FEAGI environment. These tests are marked with `@pytest.mark.skip` and include a reason.

## Test Coverage

The tests cover:

- Loading genomes
- Sensory data generation
- Neural activity monitoring
- Thread management
- Test result reporting
- Parameter handling

## Mocking Strategy

These tests use a combination of:

- Mocked core components for unit tests
- Partially mocked components for integration tests
- Time manipulation to speed up tests

Most external dependencies are mocked to ensure tests run quickly and reliably.
