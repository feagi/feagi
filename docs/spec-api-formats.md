# FEAGI API Formats Specification

*Last Updated: May 15, 2025*

## Overview

This specification describes the API response formats and compatibility features of the FEAGI API. The FEAGI API is designed to support both legacy clients (v1) and newer integrations (v2+) through a dual response format strategy.

## Response Formats

### V1 API (Legacy Format)

Responses from `/v1/*` endpoints maintain backward compatibility with existing integrations. These responses vary in structure by endpoint and return data directly without a standardized wrapper.

Example V1 Response:
```json
{
  "loaded": true,
  "genome_counter": 5,
  "load_time": 0.234
}
```

### V2 API (Standardized Format)

All `/v2/*` endpoints use a standardized response structure:

#### Success Response Format

```json
{
  "success": true,
  "data": {
    "loaded": true,
    "genome_counter": 5,
    "load_time": 0.234
  },
  "message": "Optional human-readable message",
  "metadata": { 
    "additional_info": "any metadata"
  },
  "timestamp": "2023-05-20T12:34:56.789Z"
}
```

#### Error Response Format

```json
{
  "success": false,
  "message": "Error description",
  "error_code": "ERROR_CODE",
  "metadata": { "additional_error_details": {} },
  "timestamp": "2023-05-20T12:34:56.789Z"
}
```

## URL Structure and Compatibility

### Path Format Standardization

FEAGI 2.1 supports both snake_case (legacy) and kebab-case (modern) path segments:

- `/v1/cortical_area` and `/v1/cortical-area` are both valid
- `/v1/burst_engine` and `/v1/burst-engine` are both valid

This ensures compatibility with both existing and new client implementations.

### Legacy Endpoint Support

The following legacy endpoints are directly supported:

#### Cortical Areas

- GET `/v1/cortical_areas` - List all cortical areas
- GET `/v1/cortical_area/{area_id}` - Get a specific cortical area
- GET `/v1/cortical_area_types` - List all cortical area types

#### Genome

- GET `/v1/genome/file_name` - Get the current genome filename
- POST `/v1/genome/upload/barebones` - Upload a barebones genome

#### Burst Engine

- GET `/v1/burst_engine/config` - Get burst engine configuration

## Implementation Details

### Response Format Middleware

The API server implements middleware that automatically standardizes responses based on the API version:

1. V1 routes (`/v1/*`) maintain their original response formats to ensure backward compatibility.
2. V2+ routes (`/v2/*` and above) use the standardized response format.
3. Special cases can bypass standardization using the `raw_response()` utility.

```python
@app.middleware("http")
async def standardize_response_format(request, call_next):
    """
    Middleware that standardizes API responses.
    - Skips standardization for v1 routes
    - Applies standardization to v2+ routes
    - Honors raw_response() markers 
    """
    # Implementation details...
```

### Response Utilities

When developing FEAGI extensions, use these utilities:

- `success_response(data, message, metadata)`: Creates a standardized success response
- `error_response(message, error_code, metadata)`: Creates a standardized error response
- `raw_response(data)`: Bypasses standardization when needed

Example usage:
```python
from feagi.api.rest.response_utils import success_response, error_response

@app.get("/v2/genome/status")
async def get_genome_status():
    try:
        status = await get_status()
        return success_response(
            data=status,
            message="Genome status retrieved successfully"
        )
    except Exception as e:
        return error_response(
            message=str(e),
            error_code="GENOME_STATUS_ERROR"
        )
```

## Response Types

### Common Response Fields

| Field | Type | Description |
|-------|------|-------------|
| `success` | boolean | Indicates if the request was successful |
| `data` | any | Main response payload (for success responses) |
| `message` | string | Human-readable message |
| `error_code` | string | Machine-readable error code (for error responses) |
| `metadata` | object | Additional contextual information |
| `timestamp` | string | ISO 8601 timestamp |

### Error Codes

Standard error codes follow this format: `RESOURCE_OPERATION_ERROR`

Examples:
- `GENOME_NOT_FOUND`: Genome resource not found
- `CORTICAL_AREA_CREATION_FAILED`: Failed to create cortical area
- `BURST_ENGINE_CONFIG_INVALID`: Invalid burst engine configuration

## Versioning Strategy

### API Version Lifecycle

1. **V1**: Legacy API, maintained for backward compatibility. No new features.
2. **V2**: Current API with standardized responses. Active development.
3. **Future versions**: Will be introduced for breaking changes.

### Version Transition Guidelines

- New integrations should use V2 endpoints.
- Legacy integrations can continue using V1 endpoints.
- Migration guides will be provided when new versions are released.

## Interprocess Communication

The API server and core FEAGI components communicate through:

1. **Shared Memory**: Data structures like the connectome are shared between processes using memory-mapped files.
2. **Event-based Updates**: The API service receives updates through an event system when data changes.
3. **Priority-based Process Management**: Process priorities ensure burst engine operations take precedence over API handling.

## Related Documentation

- [IPC Architecture](arch-ipc.md)
- [Shared Memory Protocol](spec-shared-memory.md)
- [ZMQ Architecture](arch-zmq.md) 