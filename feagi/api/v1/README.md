# FEAGI v1 API - Transport Agnostic Architecture

## 🎯 **Overview**

The FEAGI v1 API implements a **transport-agnostic architecture** that provides identical behavior across multiple transport protocols (HTTP REST, ZMQ, gRPC, etc.). This design ensures that clients get the same responses regardless of how they connect to FEAGI.

## 🏗️ **Architecture**

```
┌─────────────────────┐    ┌─────────────────────┐
│   HTTP Clients      │    │   ZMQ Clients       │
│  (Brain Visualizer) │    │  (Godot Bridge)     │
└─────────┬───────────┘    └─────────┬───────────┘
          │                          │
          ▼                          ▼
┌─────────────────────┐    ┌─────────────────────┐
│  FastAPI Adapter    │    │   ZMQ Adapter       │
│  (HTTP Transport)   │    │  (ZMQ Transport)    │
└─────────┬───────────┘    └─────────┬───────────┘
          │                          │
          └──────────┬─────────────────┘
                     ▼
          ┌─────────────────────┐
          │   v1 Business Logic │
          │  (Transport-Agnostic)│
          └─────────┬───────────┘
                    ▼
          ┌─────────────────────┐
          │  Core API Service   │
          └─────────────────────┘
```

## 📁 **Directory Structure**

```
/api/
├── v1/                          # 🎯 Transport-agnostic business logic
│   ├── __init__.py
│   ├── schemas.py               # Shared request/response models
│   ├── system.py                # System API business logic
│   ├── genome.py                # Genome API business logic (future)
│   ├── connectome.py            # Connectome API business logic (future)
│   └── README.md                # This file
│
├── transport/                   # 🚀 Transport-specific adapters
│   ├── __init__.py
│   ├── fastapi_adapter.py       # HTTP REST transport
│   └── zmq_adapter.py           # ZMQ transport
│
└── rest/routers/v1/            # 🔄 Legacy FastAPI routers (delegating to transport)
    ├── system.py               # Now imports from transport adapter
    ├── genome.py
    └── connectome.py
```

## 🔑 **Key Principles**

### 1. **Single Source of Truth**
- All API endpoint definitions live in `/v1/`
- Business logic is transport-agnostic
- Identical behavior across all transports

### 2. **Transport Adapters**
- Each transport protocol has its own adapter
- Adapters translate protocol-specific requests to v1 API calls
- Adapters translate v1 responses to protocol-specific formats

### 3. **Schema-Driven**
- All requests and responses use Pydantic models
- Shared schemas ensure consistency
- Type safety across the entire API

### 4. **Error Handling**
- Consistent error responses across transports
- Transport adapters handle protocol-specific error formatting
- Business logic raises standard Python exceptions

## 📖 **Usage Examples**

### Creating a New API Module

```python
# 1. Define schemas in /v1/schemas.py
class MyRequest(BaseModel):
    name: str
    value: int

class MyResponse(BaseModel):
    result: str
    timestamp: datetime

# 2. Implement business logic in /v1/my_module.py
class MyModuleAPI:
    def __init__(self, core_api_service: CoreAPIService):
        self.core_api_service = core_api_service
    
    def process_request(self, request: MyRequest) -> MyResponse:
        # Pure business logic here
        result = self.core_api_service.do_something(request.name, request.value)
        return MyResponse(result=result, timestamp=datetime.now())

# 3. Add to FastAPI adapter
@router.post("/my_endpoint", response_model=MyResponse)
async def my_endpoint(request: MyRequest, api: MyModuleAPI = Depends(get_my_module_api)):
    return api.process_request(request)

# 4. Add to ZMQ adapter
async def _handle_my_endpoint(self, params, query, body, headers):
    request = MyRequest(**body)
    return self.my_module_api.process_request(request)
```

### Transport-Specific Behavior

**HTTP (FastAPI):**
```http
POST /v1/system/user_preferences
Content-Type: application/json

{
    "adv_mode": true,
    "ui_magnification": 1.5
}
```

**ZMQ:**
```json
{
    "route": "/v1/system/user_preferences",
    "method": "POST",
    "body": {
        "adv_mode": true,
        "ui_magnification": 1.5
    }
}
```

Both return identical responses:
```json
{
    "status": "success",
    "message": "User preferences updated successfully"
}
```

## 🔄 **Migration Guide**

### From Legacy FastAPI Routers

1. **Extract Business Logic**
   - Move endpoint logic from FastAPI router to v1 module
   - Remove FastAPI dependencies from business logic
   - Use Pydantic models for requests/responses

2. **Update Router**
   - Import transport adapter router
   - Remove duplicate endpoint definitions

3. **Test Compatibility**
   - Verify HTTP endpoints work identically
   - Test ZMQ endpoints provide same responses

### Adding New Transport

1. **Create Transport Adapter**
   - Implement protocol-specific message parsing
   - Translate to v1 API calls
   - Format responses for protocol

2. **Register Route Handlers**
   - Map protocol routes to v1 API methods
   - Handle protocol-specific parameters

3. **Test Compatibility**
   - Ensure identical behavior to existing transports
   - Verify error handling consistency

## ✅ **Testing Strategy**

### Unit Tests
- Test v1 business logic independently
- Mock CoreAPIService dependencies
- Focus on business rules and edge cases

### Integration Tests
- Test each transport adapter against v1 API
- Verify identical responses across transports
- Test error handling consistency

### End-to-End Tests
- Real clients connecting via different transports
- Compare responses for identical requests
- Performance benchmarking

## 🚀 **Benefits**

1. **Consistency**: Identical behavior across all transport protocols
2. **Maintainability**: Single source of truth for business logic
3. **Extensibility**: Easy to add new transport protocols
4. **Testing**: Business logic can be tested independently
5. **Performance**: Each transport optimized for its protocol
6. **Migration**: Clear path from v1 to future versions

## 🔮 **Future Expansion**

### Planned v1 Modules
- `genome.py` - Genome management API
- `connectome.py` - Connectome manipulation API
- `neuron.py` - Individual neuron operations
- `monitoring.py` - System monitoring and metrics

### Additional Transports
- gRPC adapter for high-performance clients
- WebSocket adapter for real-time updates
- GraphQL adapter for flexible queries

### v2 Architecture
When ready for v2:
```
/api/
├── v1/                    # Stable v1 API
├── v2/                    # New v2 API with breaking changes
└── transport/
    ├── fastapi_adapter_v1.py
    ├── fastapi_adapter_v2.py
    └── zmq_adapter_v1.py
```

## 🤝 **Contributing**

When adding new endpoints:

1. **Always start with v1 business logic**
2. **Use shared schemas for consistency**
3. **Add to all relevant transport adapters**
4. **Write tests for business logic first**
5. **Verify transport compatibility**
6. **Update documentation**

This architecture ensures that FEAGI's API remains consistent, maintainable, and extensible as the system grows. 