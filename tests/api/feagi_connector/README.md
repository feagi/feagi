# FEAGI Connector Tests

This directory contains tests for the FEAGI connector library, which provides a Python client for connecting to and interacting with FEAGI.

## Test Files

- `test_basic_connection.py`: Tests basic connectivity with a running FEAGI instance, including agent registration, heartbeat, and deregistration.

## Running the Tests

To run these tests, make sure you have a FEAGI instance running, then:

```bash
# Run all FEAGI connector tests
pytest tests/api/feagi_connector/

# Run a specific test
pytest tests/api/feagi_connector/test_basic_connection.py
```

## Requirements

- A running FEAGI instance
- The `feagi_connector` Python package installed 