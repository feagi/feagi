# FEAGI Bytes Tests

This directory contains tests for the FEAGI bytes library, which provides binary serialization and deserialization for FEAGI data protocols.

## Test Files

- `test_connection.py`: Tests connection to FEAGI using the FEAGI bytes library for binary serialization.
- `test_simple.py`: A simplified test for verifying FEAGI connection with minimal dependencies.

## Running the Tests

To run these tests, make sure you have a FEAGI instance running, then:

```bash
# Run all FEAGI bytes tests
pytest tests/api/feagi_bytes/

# Run a specific test
pytest tests/api/feagi_bytes/test_connection.py
```

## Requirements

- A running FEAGI instance
- The `feagi_bytes` Python package installed 