# FEAGI API Module

*Last Updated: May 15, 2025*

## Overview

The API module is the primary interface between FEAGI and external components. It provides RESTful HTTP endpoints, WebSocket connections, and high-performance ZeroMQ (ZMQ) streams for communicating with FEAGI.

## Architecture

The API module is structured around a Service-Oriented Architecture:

```
┌─────────────────────┐      ┌──────────────────┐      ┌──────────────────┐
│                     │      │                  │      │                  │
│  REST API Routes    │─────▶│  CoreAPIService  │─────▶│  ConnectomeManager
│  WebSocket Handlers │      │                  │      │                  │
│  ZeroMQ Streams     │      │                  │      │                  │
│                     │      │                  │      │                  │
└─────────────────────┘      └──────────────────┘      └──────────────────┘
```

All interfaces (REST, WebSocket, ZMQ) must follow the **critical architectural rule** that client-facing endpoints never directly access the ConnectomeManager but always go through the CoreAPIService layer.

## Key Components

### Core Components

- **core/**: Core API service layer that implements business logic
  - **services/**: Service implementations (CoreAPIService)
  - **models/**: Data models used across the API

### Interface Implementations

- **rest/**: FastAPI-based HTTP REST API
  - **routers/**: API route definitions organized by version (v1, v2)
  - **common/**: Shared utilities for REST endpoints
  - **static/**: Static assets for web interfaces

- **zmq/**: ZeroMQ-based communication streams
  - **streams/**: Implementation of specific stream types
    - **sensorimotor.py**: Handles sensory input and motor output
    - **visualization.py**: Handles visualization data streaming
  - **server.py**: Central ZMQ server implementation
  - **connection_manager.py**: Manages ZMQ socket lifecycle

### Protocols

- **protocols/**: Communication protocol implementations
  - **byte_structures/**: Binary protocol structure definitions
  - **translator.py**: Converts between binary and Python objects
  - **constants.py**: Protocol constants and enumerations

### Shared Components

- **utils/**: Shared utilities for API implementations
- **shared_memory/**: Shared memory management for efficient IPC
- **gateway/**: API gateway functionalities

## Usage

The API module is used both internally by FEAGI components and externally by client applications:

### Internal Usage

Internal FEAGI components use the API services directly:

```python
from feagi.api.core.services import CoreAPIService

# Get a service instance
service = CoreAPIService(connectome_manager, state_manager)

# Use the service
service.get_cortical_area_list()
```

### External Usage

External clients interact with FEAGI through exposed endpoints:

- **REST**: `http://hostname:8000/v1/...` endpoints
- **WebSocket**: `ws://hostname:8000/ws/...` connections
- **ZeroMQ**: Dedicated ZMQ streams on configured ports

## Dependencies

- **FastAPI**: For REST API implementation
- **PyZMQ**: For ZeroMQ streams
- **Pydantic**: For data validation and serialization

## Related Documentation

- [API Architecture Decision Record](../../docs/adr-api-refactoring.md)
- [API Formats Specification](../../docs/spec-api-formats.md)
- [API Usage Guide](guide-api-usage.md) 