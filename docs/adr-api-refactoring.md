# Architecture Decision Record: API Refactoring

*Last Updated: May 15, 2025*

## Status

Completed

## Context

FEAGI's API layer had evolved organically over time, resulting in inconsistent patterns and direct dependencies on low-level components. This created several issues:

1. API endpoints directly accessed the ConnectomeManager, bypassing proper service abstraction
2. Some routes referenced undefined global variables
3. Error handling was inconsistent and often duplicated across endpoints
4. No clear architectural boundaries between presentation, business logic, and data layers

## Decision

We decided to implement a comprehensive API refactoring based on the Service Layer pattern, introducing `CoreAPIService` as the central service component. The key architectural principle established was:

**CRITICAL RULE**: API endpoints must NEVER directly access the ConnectomeManager. All access to connectome data must go through the CoreAPIService layer.

## Implementation Approach

The refactoring was implemented in four phases:

### Phase 1: Service Layer Development
- Audit of all API endpoints to identify direct ConnectomeManager access
- Implementation of all required methods in CoreAPIService
- Comprehensive service methods for all CRUD operations

### Phase 2: API Endpoint Refactoring
- Replaced direct ConnectomeManager injections with CoreAPIService
- Converted undefined global references to explicit service usage
- Standardized error handling across all endpoints
- Implemented proper input validation

### Phase 3: Unified Response Format
- Implemented consistent response formatting in CoreAPIService
- Ensured all endpoints use standardized response formats
- Applied proper HTTP status codes for different scenarios

### Phase 4: Testing and Validation
- Integration tests for refactored endpoints
- Runtime checks to prevent direct ConnectomeManager access

## Implementation Schedule

The refactoring was prioritized into three categories:

### Priority 1: Core Operation Endpoints
- Burst Engine endpoints
- Connectome endpoints
- Cortical Area endpoints

### Priority 2: Genome Management Endpoints
- Genome endpoints
- Morphology endpoints
- Neuroplasticity endpoints

### Priority 3: Additional Categories
- Input endpoints
- Region endpoints
- Cortical Mapping endpoints
- Evolution endpoints
- Insights endpoints
- Network endpoints
- Simulation endpoints
- System endpoints
- Agent endpoints

## Key Architectural Improvements

### Service Layer Benefits
- **Isolation**: The CoreAPIService acts as an intermediary between API endpoints and the state/core components
- **Centralized Logic**: Common operations are now managed in a single location
- **Consistent Error Handling**: Standardized approach to handling exceptions
- **Enhanced Testing**: Service methods can be tested independently from API routes

### Dependency Management
- Direct access to `state_manager` and `connectome_manager` is now encapsulated
- Each router gets an instance of CoreAPIService via a dependency injection pattern
- CoreAPIService constructor validates that components are properly initialized

### State Manager Integration
- CoreAPIService maintains tight integration with the state manager
- CoreAPIService properly handles data synchronization and caching

### Specialized Dependency Checks
- Added state-specific dependencies to verify system state before operations
- Created checks for genome, connectome, burst engine, and other critical components

## Implementation Pattern

All refactored routers now follow this consistent pattern:

1. Get a CoreAPIService instance:
```python
def get_api_service():
    connectome_manager = state_manager.get_connectome()
    if not connectome_manager:
        # Create a minimal version if not available
        connectome_manager = ConnectomeManager()
    return CoreAPIService(connectome_manager=connectome_manager, state_manager=state_manager)
```

2. Use CoreAPIService methods in API endpoints:
```python
@router.get("/endpoint")
async def endpoint():
    api_service = get_api_service()
    try:
        result = api_service.method_name()
        return result
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
```

## Consequences

### Positive
- Improved code maintainability and reduced duplication
- Better separation of concerns between API, business logic, and data layers
- More consistent error handling and response formats
- Enhanced testability of both API endpoints and service methods
- Simplified future development of new endpoints
- Better foundation for Rust migration path

### Negative
- Required significant refactoring effort
- Introduced some additional abstraction complexity
- Added slight performance overhead (though negligible in practice)

## Verification

The refactoring was verified through:
- Comprehensive integration testing of refactored endpoints
- Runtime assertion checks for architectural compliance
- Manual testing of key endpoints
- Performance benchmark comparisons

## Related Documentation
- [System Overview](arch-system-overview.md)
- [API Response Formats](spec-api-formats.md) 