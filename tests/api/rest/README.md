# FEAGI REST API Tests

This directory contains tests for the FEAGI REST API endpoints.

## Test Structure

The tests are organized by API domain:

- `test_routes.py` - Basic API structure tests
- `test_system_api.py` - System configuration endpoints
- `test_simulation_api.py` - Simulation control endpoints
- `test_region_api.py` - Brain region management endpoints
- `test_insights_api.py` - Analytics and visualization endpoints
- `test_cortical_mapping_api.py` - Cortical mapping endpoints
- `test_burst_engine_api.py` - Burst engine configuration endpoints
- `test_inputs_api.py` - Input sources and stimulation endpoints
- `test_genome_api.py` - Genome management endpoints (from legacy implementation)
- `test_brain_api.py` - Brain state endpoints (from legacy implementation)

## Running Tests

To run all API tests:

```bash
python run_api_tests.py
```

To run tests for a specific domain:

```bash
python run_api_tests.py -p region  # Runs test_region_api.py
```

To run tests in verbose mode:

```bash
python run_api_tests.py -v
```

You can also use pytest directly:

```bash
python -m pytest -v test_routes.py
```

## Current Status

The basic API structure tests (test_routes.py) are passing, which confirms that the router registration is working correctly. However, most of the functional tests are failing due to several issues:

1. **Endpoint Path Mismatches**: The actual endpoint paths in the implementation don't match what the tests expect.
2. **Missing Core Methods**: The FEAGI core object is missing methods required by endpoints.
3. **ZMQ Server Issues**: There are initialization and shutdown problems with the ZMQ server.

## Next Steps

To fix the failing tests:

1. **API Path Standardization**:
   - Update router paths to be consistent across all modules
   - Follow REST API best practices (e.g., plural resource names, consistent casing)

2. **Core Implementation**:
   - Implement missing methods in the FEAGI core class
   - Create proper error handling for missing methods

3. **Testing Infrastructure**:
   - Develop better mock objects that provide consistent behavior
   - Update tests to match the actual implementation
   - Add integration tests for error handling

4. **Documentation**:
   - Add OpenAPI/Swagger documentation for all endpoints
   - Create API reference documentation
   - Update the implementation checklist 

## API Compatibility Changes

The following changes have been made to ensure compatibility between the API implementation and the tests:

1. **Router Path Consistency**:
   - Updated the burst_engine router to use `/config` instead of `/configuration` endpoints
   - Ensured all routers use consistent plural/singular naming

2. **Dependencies and Middleware**:
   - Added standard response models for error handling
   - Added dependency check functions (`check_active_genome`, `check_brain_running`, `check_burst_engine`)
   - Added CORS middleware with proper configuration

3. **Router Tags and Organization**:
   - Updated router tags to match the legacy API structure (all caps)
   - Organized endpoint paths to follow standard RESTful patterns

4. **Core API Implementation**:
   - Added placeholder implementations for missing API methods:
     - `get_input_sources()`, `get_input_source()`, `register_input_source()`, etc.
     - `get_burst_engine_config()`, `update_burst_engine_config()`, `get_burst_engine_stats()`
     - `stimulate_cortical_area()`

5. **Endpoint Parameters**:
   - Updated parameter names to be consistent with the core API implementation
   - Fixed inconsistencies like `target_cortical_area` vs `target_area_id`

These changes ensure that the API structure is consistent with the test expectations while maintaining the modular organization of the new implementation. 