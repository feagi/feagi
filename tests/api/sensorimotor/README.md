# FEAGI Sensorimotor Tests

This directory contains tests for FEAGI's sensorimotor protocol, which handles sensory data injection and motor data extraction.

## Test Files

- `test_binary_protocol.py`: A simple test for binary sensorimotor data transmission to FEAGI.
- `test_full_sensorimotor.py`: A comprehensive test for the sensorimotor subsystem, including binary data injection, FCL integration, and feedback.
- `test_fcl_injection.py`: Specifically tests and validates that sensory data is properly injected into the FCL by sending data and then verifying the neuron activations using the visualization API.

## Running the Tests

To run these tests, make sure you have a FEAGI instance running, then:

```bash
# Run all sensorimotor tests
pytest tests/api/sensorimotor/

# Run a specific test
pytest tests/api/sensorimotor/test_binary_protocol.py

# Run the FCL injection validation test
pytest tests/api/sensorimotor/test_fcl_injection.py
```

## Requirements

- A running FEAGI instance
- The `feagi_bytes` Python package installed
- The `feagi_connector` Python package installed
- ZeroMQ (with Python bindings)
