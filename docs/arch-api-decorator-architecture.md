# FEAGI Decorator-Based API Architecture

*Created: May 23, 2025*
*Status: **Current Implementation***

## 🎯 Overview

FEAGI's v1 API has been completely redesigned using a **decorator-based architecture** that provides a single source of truth for all endpoint definitions. This eliminates code duplication across transport protocols and ensures perfect consistency between FastAPI REST, ZMQ, and future transport implementations.

## 🏗️ Architecture Principles

### 1. **Single Source of Truth**
All API endpoint definitions exist **only once** in v1 API modules using the `@endpoint` decorator. No duplication across transports.

### 2. **Transport Agnostic Business Logic**
Business logic is completely separated from transport concerns, making it reusable across HTTP, ZMQ, WebSocket, gRPC, and future protocols.

### 3. **Auto-Generated Transport Layers**
Transport-specific implementations (FastAPI routers, ZMQ handlers) are automatically generated from decorated endpoints.

### 4. **Perfect Protocol Consistency**
All transport protocols provide identical responses for the same requests since they use the same underlying business logic.

## 📁 Directory Structure

```
feagi/api/
├── v1/                          # 🎯 SINGLE SOURCE OF TRUTH
│   ├── decorators.py           # @endpoint decorator and registry
│   ├── schemas.py              # Shared Pydantic request/response models
│   ├── system.py               # System API endpoints (15 endpoints)
│   ├── genome.py               # Genome API endpoints (18 endpoints)
│   ├── cortical_area.py        # Cortical Area API endpoints (26 endpoints)
│   ├── connectome.py           # Connectome API endpoints (17 endpoints)
│   ├── burst_engine.py         # Burst Engine API endpoints (11 endpoints)
│   ├── neuroplasticity.py      # Neuroplasticity API endpoints (7 endpoints)
│   ├── region.py               # Region API endpoints (5 endpoints)
│   ├── morphology.py           # Morphology API endpoints (5 endpoints)
│   ├── monitoring.py           # Monitoring API endpoints (3 endpoints)
│   ├── simulation.py           # Simulation API endpoints (3 endpoints)
│   ├── feagi_agent.py          # FEAGI Agent API endpoints (3 endpoints)
│   ├── insights.py             # Insights API endpoints (2 endpoints)
│   ├── training.py             # Training API endpoints (3 endpoints)
│   ├── cortical_mapping.py     # Cortical Mapping API endpoints (2 endpoints)
│   ├── network.py              # Network API endpoints (2 endpoints)
│   ├── inputs.py               # Input API endpoints (2 endpoints)
│   ├── outputs.py              # Output API endpoints (2 endpoints)
│   └── evolution.py            # Evolution API endpoints (2 endpoints)
│
├── transport/                   # 🔄 AUTO-GENERATED LAYERS
│   └── universal_fastapi.py    # Universal FastAPI wrapper
│
├── core/                        # 🛠️ BUSINESS LOGIC
│   └── services/
│       └── core_api_service.py # CoreAPIService implementation
│
└── rest/                        # 🌐 FASTAPI APPLICATION
    └── app.py                  # Main FastAPI application
```

## 🎨 Decorator-Based Pattern

### Endpoint Registration

Every API endpoint is defined using the module-specific decorator:

```python
# In feagi/api/v1/system.py
from .decorators import create_module_decorator

# Create the system-specific decorator
system_endpoint = create_module_decorator('system')

class SystemAPI:
    def __init__(self, core_api_service: CoreAPIService):
        self.core_api_service = core_api_service

    @system_endpoint('GET', '/health', response_model=HealthResponse)
    async def get_health_check(self) -> HealthResponse:
        """Get system health status."""
        try:
            status = self.core_api_service.get_system_health()
            return HealthResponse(
                success=True,
                status="healthy",
                message="System is operational",
                services=status.get("services", {})
            )
        except Exception as e:
            raise ValueError(f"Health check failed: {str(e)}")
```

### Universal Transport Generation

The universal wrapper automatically creates transport-specific implementations:

```python
# In feagi/api/transport/universal_fastapi.py
def get_system_router() -> APIRouter:
    """Auto-generate FastAPI router from decorated endpoints."""
    registry = get_endpoint_registry()
    return _create_router_for_module('system', registry)

def _create_router_for_module(module_name: str, registry: EndpointRegistry) -> APIRouter:
    """Create FastAPI router with automatic route generation."""
    router = APIRouter()
    endpoints = registry.get_endpoints_by_module(module_name)
    
    for endpoint_id, endpoint_info in endpoints.items():
        # Auto-generate FastAPI route from decorator metadata
        _add_fastapi_route(router, endpoint_info)
    
    return router
```

### Transport Integration

The main FastAPI application directly uses the universal wrapper:

```python
# In feagi/api/rest/app.py
from feagi.api.transport.universal_fastapi import get_system_router

app.include_router(
    get_system_router(),  # 🎯 Direct integration - no router files!
    prefix="/v1/system",
    tags=["SYSTEM"]
)
```

## 🔍 How It Works

### 1. **Decorator Registration**
```python
@system_endpoint('GET', '/health', response_model=HealthResponse)
```
- Decorator captures method metadata (HTTP method, path, response model)
- Registers endpoint in the global `EndpointRegistry`
- Associates endpoint with the 'system' module

### 2. **Universal Wrapper Generation**
```python
def get_system_router() -> APIRouter:
```
- Retrieves all 'system' endpoints from registry
- Auto-generates FastAPI routes with proper parameter handling
- Preserves all metadata (tags, response models, dependencies)

### 3. **Transport Implementation**
```python
app.include_router(get_system_router())
```
- FastAPI application includes auto-generated router
- No manual route definitions needed
- Perfect consistency guaranteed

## 🎯 Benefits Achieved

### ✅ **Zero Code Duplication**
- Endpoint logic exists in exactly one place
- Transport layers are auto-generated
- Changes to business logic automatically propagate to all transports

### ✅ **Perfect Transport Consistency**
- All protocols (HTTP, ZMQ, future) use identical business logic
- Same request → same response across all transports
- No possibility of transport-specific inconsistencies

### ✅ **Simplified Maintenance**
- Add new endpoint: implement once, works everywhere
- Modify endpoint: change once, updates everywhere
- No transport-specific router files to maintain

### ✅ **Type Safety Throughout**
- Pydantic models ensure type consistency
- Decorator metadata provides compile-time validation
- Auto-generated routes preserve all type information

### ✅ **Clean Architecture**
- Business logic completely separated from transport concerns
- Service layer provides consistent data access
- Clear boundaries between presentation, business, and data layers

## 📊 Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           CLIENT APPLICATIONS                           │
├─────────────────┬─────────────────┬─────────────────┬─────────────────┤
│   HTTP Clients  │   ZMQ Clients   │  Future Clients │  Test Clients   │
│ (Brain Viz,etc) │ (Godot Bridge)  │ (gRPC, WS, etc) │ (pytest, etc)   │
└─────────┬───────┴─────────┬───────┴─────────┬───────┴─────────┬───────┘
          │                 │                 │                 │
          ▼                 ▼                 ▼                 ▼
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
│  FastAPI        │ │  ZMQ Handler    │ │  Future         │ │  Direct API     │
│  Auto-Generated │ │  Auto-Generated │ │  Auto-Generated │ │  Access         │
│  Router         │ │  Endpoints      │ │  Adapters       │ │                 │
└─────────┬───────┘ └─────────┬───────┘ └─────────┬───────┘ └─────────┬───────┘
          │                   │                   │                   │
          └─────────────┬─────────────┬───────────────────────────────┘
                        │             │
                        ▼             ▼
                ┌─────────────────────────────────┐
                │     UNIVERSAL WRAPPER           │
                │ (universal_fastapi.py, etc)     │
                │                                 │
                │ • Auto-generates transport      │
                │   specific implementations      │
                │ • Handles parameter inspection  │
                │ • Preserves metadata           │
                └─────────────┬───────────────────┘
                              │
                              ▼
                ┌─────────────────────────────────┐
                │        v1 API MODULES           │
                │     🎯 SINGLE SOURCE OF TRUTH   │
                │                                 │
                │ @system_endpoint decorators     │
                │ @genome_endpoint decorators     │
                │ @connectome_endpoint decorators │
                │ ... (15+ modules)               │
                └─────────────┬───────────────────┘
                              │
                              ▼
                ┌─────────────────────────────────┐
                │      CORE API SERVICE           │
                │                                 │
                │ • Business logic               │
                │ • Data validation              │
                │ • Error handling               │
                │ • State management             │
                └─────────────┬───────────────────┘
                              │
                              ▼
                ┌─────────────────────────────────┐
                │    CONNECTOME MANAGER           │
                │         DATA LAYER              │
                └─────────────────────────────────┘
```

## 🛠️ Implementation Examples

### Adding a New Endpoint

1. **Define in v1 module:**
```python
@system_endpoint('POST', '/restart', response_model=SystemResponse)
async def restart_system(self, force: bool = False) -> SystemResponse:
    """Restart the FEAGI system."""
    result = self.core_api_service.restart_system(force=force)
    return SystemResponse(success=True, message="System restarted")
```

2. **That's it!** The endpoint is automatically available on:
   - HTTP: `POST /v1/system/restart`
   - ZMQ: `{"action": "restart_system", "force": true}`
   - Future transports: Auto-generated

### Adding a New Module

1. **Create v1 module file:**
```python
# feagi/api/v1/new_module.py
new_module_endpoint = create_module_decorator('new_module')

class NewModuleAPI:
    @new_module_endpoint('GET', '/info')
    async def get_info(self): ...
```

2. **Add to universal wrapper:**
```python
# feagi/api/transport/universal_fastapi.py
def get_new_module_router() -> APIRouter:
    return _create_router_for_module('new_module', get_endpoint_registry())
```

3. **Include in app:**
```python
# feagi/api/rest/app.py
app.include_router(get_new_module_router(), prefix="/v1/new_module")
```

## 🧪 Testing Strategy

### Unit Testing
- Test v1 API classes directly without transport layer
- Mock CoreAPIService for isolated testing
- Validate business logic independently

### Integration Testing
- Test auto-generated routers against actual endpoints
- Verify transport compatibility
- Validate end-to-end workflows

### Transport Testing
- Ensure HTTP and ZMQ return identical responses
- Test parameter handling across transports
- Validate error handling consistency

## 🚀 Migration Results

### Before (Legacy Architecture)
- **~2000+ lines** of duplicated router code
- 18+ router files with duplicate endpoint definitions
- Inconsistencies between transport protocols
- Manual maintenance of multiple implementations

### After (Decorator Architecture)  
- **~400 lines** of clean, auto-generated router code
- Single source of truth for all endpoints
- Perfect transport consistency guaranteed
- Zero maintenance overhead for transport layers

### Endpoints Successfully Migrated
- **System**: 15 endpoints
- **Genome**: 18 endpoints
- **Cortical Area**: 26 endpoints
- **Connectome**: 17 endpoints
- **Burst Engine**: 11 endpoints
- **Neuroplasticity**: 7 endpoints
- **Region**: 5 endpoints
- **Morphology**: 5 endpoints
- **Monitoring**: 3 endpoints
- **Simulation**: 3 endpoints
- **FEAGI Agent**: 3 endpoints
- **Insights**: 2 endpoints
- **Training**: 3 endpoints
- **Cortical Mapping**: 2 endpoints
- **Network**: 2 endpoints
- **Inputs**: 2 endpoints
- **Outputs**: 2 endpoints
- **Evolution**: 2 endpoints

**Total: 130+ endpoints** across 18 modules

## 🛡️ Architectural Compliance & Violation Prevention

### 🚨 **Critical Warning: Architectural Violations**

This section was created after discovering serious architectural violations where different transport protocols (HTTP vs ZMQ) were returning different responses for the same endpoints. This is **STRICTLY FORBIDDEN** and violates the Single Source of Truth principle.

### **Core Compliance Rules**

#### 1. **Single Source of Truth (SSOT) Enforcement**
```
✅ CORRECT: One v1 API endpoint → Multiple transport adapters
❌ FORBIDDEN: Multiple implementations of the same endpoint
```

All API endpoints MUST have exactly ONE implementation in the v1 API modules. Transport adapters provide PURE DELEGATION only.

#### 2. **Pure Delegation Pattern**

**✅ Correct Transport Adapter:**
```python
async def _handle_get_cortical_area_id_list(self, params, query, body, headers):
    """Handler that delegates to v1 API"""
    return self.cortical_area_api.get_cortical_area_id_list()
```

**❌ Forbidden Custom Implementation:**
```python
async def _handle_get_cortical_area_id_list(self, params, query, body, headers):
    """VIOLATION: Custom implementation"""
    return self.core_api_service.get_cortical_areas()  # ⚠️ WRONG!
```

#### 3. **Response Format Consistency**

ALL transports MUST return identical responses for identical requests:
- v1 API defines response format
- All transports use same v1 API method
- **Result**: Perfect consistency across transports

### **Violation Prevention Protocols**

#### **Pre-Implementation Checklist**

1. **Does this endpoint already exist in v1 API?**
   - If YES: Use existing endpoint, never duplicate
   - If NO: Add to appropriate v1 API module

2. **Are you implementing in a transport adapter?**
   - ❌ STOP: Move implementation to v1 API
   - ✅ OK: Only if purely delegating to v1 API

3. **Are response formats identical across all transports?**
   - Test with both HTTP (Swagger) and ZMQ clients
   - Responses must be byte-for-byte identical

#### **Code Review Requirements**

ALL pull requests MUST pass these checks:

1. **No duplicate endpoint implementations**
   ```bash
   # Search for duplicate route definitions
   grep -r "cortical_area_id_list" feagi_core/feagi/api/
   # Should only appear in v1 API and transport delegation
   ```

2. **No custom business logic in transport adapters**
   ```python
   # ❌ REJECT: Any handler with custom logic
   def _handle_xyz(self):
       return self.core_api_service.some_method()  # VIOLATION
   
   # ✅ APPROVE: Pure delegation
   def _handle_xyz(self):
       return self.xyz_api.some_method()  # CORRECT
   ```

3. **Response format testing**
   ```bash
   # Test same endpoint on different transports
   curl http://localhost:8000/v1/system/health_check    # HTTP
   zmq_client.py GET /v1/system/health_check           # ZMQ
   # Responses must be identical
   ```

#### **Immediate Actions on Violation Detection**

1. **STOP all development**
2. **Identify all violated endpoints**
3. **Remove custom implementations**
4. **Ensure v1 API has required endpoints**
5. **Update transport adapters to pure delegation**
6. **Test consistency across all transports**

### **Common Pitfalls & Solutions**

#### **Pydantic Schema Field Name Mismatches**

**Issue**: Transport adapters creating Pydantic request objects with incorrect field names.

**Example Error**:
```
1 validation error for CorticalIdListRequest
cortical_ids
  Field required [type=missing, input_value={'cortical_id_list': ['...']}, input_type=dict]
```

**Root Cause**: Request body contains `cortical_id_list` but schema expects `cortical_ids`.

**❌ Incorrect**:
```python
# Wrong field name used
request = CorticalIdListRequest(cortical_id_list=cortical_id_list)  # FAILS
```

**✅ Correct**:
```python
# Use exact schema field name
request = CorticalIdListRequest(cortical_ids=cortical_id_list)  # WORKS
```

**Prevention**: Always verify Pydantic schema field names in `/feagi_core/feagi/api/v1/schemas.py` before creating request objects.

### **Architecture Compliance KPIs**

Success metrics for architectural compliance:

- ✅ **100% endpoint consistency** across all transports
- ✅ **Zero custom implementations** in transport adapters  
- ✅ **Single point of truth** for each endpoint
- ✅ **Automatic transport registration** for new endpoints

### **Legacy Migration Strategy**

#### **Identify Legacy Violations**
- Custom handlers in transport adapters
- Direct core service calls in transport adapters
- Response format inconsistencies

#### **Migration Steps**
1. Move business logic to appropriate v1 API module
2. Update transport adapters to pure delegation
3. Remove legacy implementations
4. Test endpoint consistency

## 🔮 Future Enhancements

### Planned Transport Support
1. **gRPC Adapter**: High-performance binary protocol
2. **WebSocket Adapter**: Real-time bidirectional communication
3. **GraphQL Adapter**: Flexible query-based API

### Advanced Features
1. **Rate Limiting**: Per-endpoint rate limiting via decorators
2. **Caching**: Automatic response caching based on endpoint metadata
3. **Metrics**: Built-in performance monitoring and analytics
4. **API Versioning**: Support for v2, v3+ APIs using same pattern

### Developer Experience
1. **Auto-Documentation**: Generate comprehensive API docs from decorators
2. **Client Generation**: Auto-generate SDKs for multiple languages
3. **Mock Generation**: Auto-generate mock servers for testing

## 📚 Related Documentation

- [CoreAPIService Architecture](arch-core-api-service.md)
- [API Response Formats](spec-api-formats.md)
- [ZMQ Integration Guide](zmq_rest_api_protocol.md)
- [Testing Strategy](plan-testing-strategy.md)

---

*This document reflects the current state of FEAGI's API architecture as of May 2025. For the most up-to-date information, refer to the actual implementation in `/feagi/api/v1/` and `/feagi/api/transport/`.* 