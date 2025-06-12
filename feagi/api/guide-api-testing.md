# FEAGI Testing Guide

This document outlines how to test the FEAGI REST API and how to troubleshoot common issues.

## Common Server Issues

When running the FEAGI server, you might encounter the following issues:

1. **Port conflicts**: The main API server uses port 8000 by default, and ZMQ uses ports 5555-5558 and 5560. If any of these ports are already in use, the server will fail to start.

2. **ZMQ server initialization issues**: There are known issues with the ZMQ server initialization, which can cause errors like "Task got Future attached to a different loop". This happens because of asyncio event loop problems.

3. **Core API object not initialized**: The server might show errors like "'NoneType' object has no attribute 'get_system_metrics'" because the core FEAGI object is not properly initialized.

## Running Tests Safely

To safely run the API tests without the full server initialization, you can use the provided test utilities:

### Using the Test Scripts

1. **Run the API tests script**:
   ```bash
   ./run_api_tests.sh
   ```
   This script will kill any existing FEAGI processes, check for port conflicts, and run the Burst Engine API tests.

2. **Run the test server**:
   ```bash
   ./run_feagi_test_server.py
   ```
   This runs a version of the FEAGI server with properly mocked dependencies, which avoids the initialization errors.

### Manual Testing

You can also run the tests manually:

1. Ensure all ports are free:
   ```bash
   pkill -f "python -m feagi.main"
   lsof -i :8000,8001,5555,5556,5557,5558,5560
   ```

2. Run the specific test:
   ```bash
   cd tests/api/rest
   python -m pytest -v test_burst_engine_api.py
   ```

## Fixing Server Issues

### Port Conflicts

If you encounter port conflicts, you can either:

1. Free the ports by stopping the processes using them:
   ```bash
   lsof -i :8000  # Identify processes
   kill <PID>     # Kill the process
   ```

2. Use different ports:
   ```bash
   export FEAGI_PORT=8002  # For the test server
   ./run_feagi_test_server.py
   ```

### ZMQ and Asyncio Issues

The ZMQ server has issues with asyncio event loops. The test server script works around these by mocking the ZMQ server and avoiding initialization of actual ZMQ sockets.

For development and testing, use the test server script which mocks these dependencies.

### Proper API Testing

The test fixtures in `tests/api/rest/conftest.py` provide mock implementations of the core API services, which allow the tests to run without initializing the full FEAGI backend.

## API Endpoints

The main API endpoints that are tested include:

- `/api/v1/burst_engine/config` - Get and update burst engine configuration
- `/api/v1/burst_engine/stats` - Get burst engine statistics

Check the API documentation for more details on available endpoints and their usage.

## Adding New Tests

When adding new API tests:

1. Ensure proper mocking of dependencies in `conftest.py`
2. Add any required mock methods to `MockFEAGI` in `run_feagi_test_server.py`
3. Structure tests to use the mock core API service
4. Run tests individually to verify they work before integrating

## Debugging API and NPU Issues

FEAGI provides specialized debugging flags for troubleshooting. For comprehensive debugging information, see the [FEAGI Debugging Guide](../../docs/guide-how-to-debug.md).

### Quick Debug Commands

```bash
# API debugging
python -m feagi.main --debug-api

# NPU debugging
python -m feagi.main --debug-npu

# Combined debugging
python -m feagi.main --debug-api --debug-npu --log-level DEBUG

# Test mode with debugging
python -m feagi.main --test --debug-npu --test-duration 30
```

## Common Error Messages and Solutions

### Error: "Address already in use"
**Solution**: Kill existing processes using the port:
```bash
pkill -f "python -m feagi.main"
```

### Error: "Task got Future attached to a different loop"
**Solution**: Use the mock ZMQ server implementation in the test server script.

### Error: "'NoneType' object has no attribute X"
**Solution**: Ensure proper mocking of the core API service and all required methods.

## Running FEAGI In Production

For production environments, resolving the ZMQ and asyncio issues is recommended. This involves:

1. Ensuring consistent event loop usage throughout the codebase
2. Properly initializing the ZMQ server with proper context management
3. Using a structured startup and shutdown sequence for the server components

Please refer to the main FEAGI documentation for production deployment guidelines.
