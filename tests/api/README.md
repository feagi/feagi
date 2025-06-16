# FEAGI API Tests

This directory contains tests for the FEAGI API components.

## Structure

- `core/` - Tests for the Core API
- `rest/` - Tests for the REST API
- `zmq/` - Tests for the ZMQ API
- `feagi_connector/` - Tests for the FEAGI Connector (currently disabled)

## Important Notes

### FEAGI Connector

The feagi_connector tests are currently disabled as the feagi_connector module is planned to be moved to a separate FEAGI Bridge project. This will resolve circular dependency issues between the main FEAGI application and the connector library.

Once feagi_connector is moved to FEAGI Bridge, we will update the tests to import from `feagi_bridge import feagi_connector` rather than directly from the FEAGI codebase.

## Running Tests

```bash
# Run all API tests (except disabled ones)
pytest tests/api

# Run specific test category
pytest tests/api/rest
```
