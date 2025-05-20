# FEAGI Connector Tests

## Status

**TEMPORARILY DISABLED**: These tests are currently disabled as feagi_connector is planned to be moved to the FEAGI Bridge project.

Once feagi_connector is moved to FEAGI Bridge, these tests will be re-enabled and updated to import from `feagi_bridge import feagi_connector`.

## Test Coverage

These tests verify the basic functionality of the FEAGI connector, including:

- Basic connectivity to FEAGI
- Protocol handling
- Message serialization/deserialization

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