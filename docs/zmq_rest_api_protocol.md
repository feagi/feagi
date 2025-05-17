# ZMQ REST API Protocol

This document defines the REST API protocol over ZeroMQ (ZMQ) for FEAGI 2.1, which allows clients to interact with FEAGI using a standard REST API format over ZMQ connections.

## Overview

The ZMQ REST API protocol is designed to mirror the HTTP-based REST API exactly, allowing clients to use the same API format regardless of the transport mechanism.

When FEAGI receives a ZMQ message in the REST API format, it processes it exactly as if it were an HTTP request to the corresponding REST API endpoint.

## Message Format

### Request Format

Requests sent over ZMQ should be JSON objects with the following structure:

```json
{
  "route": "/v1/path/to/resource",
  "method": "GET",
  "params": {
    "param1": "value1"
  },
  "query": {
    "filter": "value"
  },
  "body": {
    "property": "value"
  },
  "headers": {
    "authorization": "Bearer token"
  },
  "timestamp": 1621234567890
}
```

Fields:
- `route` (required): The API route, matching the REST API routes (e.g., `/v1/status`)
- `method` (required): The HTTP method (GET, POST, PUT, DELETE)
- `params` (optional): Path parameters, used for routes with parameter placeholders (e.g., `/v1/connectome/cortical_area/{cortical_id}`)
- `query` (optional): Query parameters, equivalent to URL query parameters
- `body` (optional): Request body for POST, PUT requests
- `headers` (optional): HTTP-style headers, including authentication if needed
- `timestamp` (optional): Request timestamp in milliseconds since epoch

### Response Format

Responses from FEAGI will be JSON objects with the following structure:

```json
{
  "status": 200,
  "headers": {
    "content-type": "application/json"
  },
  "body": {
    "property": "value"
  },
  "timestamp": 1621234567890
}
```

Fields:
- `status`: HTTP status code (200, 404, 500, etc.)
- `headers`: Response headers
- `body`: Response body
- `timestamp`: Response timestamp in milliseconds since epoch

### Error Responses

Error responses will have a status code >= 400 and an error object in the body:

```json
{
  "status": 404,
  "headers": {
    "content-type": "application/json"
  },
  "body": {
    "type": "error",
    "code": "ERROR_404",
    "message": "Resource not found"
  },
  "timestamp": 1621234567890
}
```

## Authentication

Authentication is handled through the `headers` field, mirroring HTTP authentication:

```json
{
  "headers": {
    "authorization": "Bearer eyJhbGciOiJIUzI1NiIsInR5cCI..."
  }
}
```

## Examples

### Getting System Health

Request:
```json
{
  "route": "/v1/system/health_check",
  "method": "GET",
  "timestamp": 1621234567890
}
```

Response:
```json
{
  "status": 200,
  "headers": {
    "content-type": "application/json"
  },
  "body": {
    "status": "healthy"
  },
  "timestamp": 1621234567890
}
```

### Getting a Specific Cortical Area

Request:
```json
{
  "route": "/v1/connectome/cortical_area/12345",
  "method": "GET",
  "params": {
    "cortical_id": "12345"
  },
  "timestamp": 1621234567890
}
```

Response:
```json
{
  "status": 200,
  "headers": {
    "content-type": "application/json"
  },
  "body": {
    "id": "12345",
    "name": "Visual Cortex",
    "type": "sensory",
    "position": [10, 10, 10],
    "dimensions": [10, 10, 1]
  },
  "timestamp": 1621234567890
}
```

### Updating Configuration

Request:
```json
{
  "route": "/v1/system/configuration",
  "method": "PUT",
  "body": {
    "burst_rate": 60
  },
  "timestamp": 1621234567890
}
```

Response:
```json
{
  "status": 200,
  "headers": {
    "content-type": "application/json"
  },
  "body": {
    "status": "success",
    "message": "Configuration updated successfully"
  },
  "timestamp": 1621234567890
}
```

## Protocol Versioning

The ZMQ REST API protocol follows the same versioning as the REST API, with version included in the route (e.g., `/v1/status`).

## Integration with Existing ZMQ Infrastructure

The ZMQ REST API protocol coexists with other ZMQ protocols in FEAGI. When a message is received:

1. FEAGI checks if the message is in the REST API format
2. If so, it's processed using the REST API adapter
3. If not, it's processed using the existing ZMQ protocol handlers

This allows for gradual migration to the REST API format while maintaining backward compatibility. 