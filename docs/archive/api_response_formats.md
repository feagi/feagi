# FEAGI API Response Formats

FEAGI implements a dual response format strategy to support both legacy clients and newer integrations.

## V1 API (Legacy Format)

Responses from `/v1/*` endpoints maintain backward compatibility with existing integrations. These responses vary in structure by endpoint.

Example:
```json
{
  "loaded": true,
  "genome_counter": 5,
  "load_time": 0.234
}
```

## V2 API (Standardized Format)

All `/v2/*` endpoints use a standardized response structure:

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

Error responses follow this structure:

```json
{
  "success": false,
  "message": "Error description",
  "error_code": "ERROR_CODE",
  "metadata": { "additional_error_details": {} },
  "timestamp": "2023-05-20T12:34:56.789Z"
}
```

## Integration Guide

When building new integrations with FEAGI:

1. For future-proof applications, use the `/v2/*` endpoints and handle the standardized response format.
2. For maintaining compatibility with existing applications, continue using the `/v1/*` endpoints.

## Response Format Utilities

When developing FEAGI extensions, use these utilities:

- `success_response(data, message, metadata)`: Creates a standardized success response
- `error_response(message, error_code, metadata)`: Creates a standardized error response
- `raw_response(data)`: (Advanced) Bypasses standardization when needed
