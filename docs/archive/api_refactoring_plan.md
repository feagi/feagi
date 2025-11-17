# API Refactoring Plan

This document outlines the plan for refactoring FEAGI's API routes to enforce the strict architectural rule that **API endpoints must never directly access the ConnectomeManager** and must instead go through CoreAPIService.

## Current Architecture Issues

1. Many API routes directly use `connectome = Depends(get_connectome)` which bypasses CoreAPIService
2. Some routes reference an undefined global `connectome` variable
3. The pattern is inconsistently applied across different endpoint modules
4. Error handling is inconsistent and duplicated across endpoints

## Refactoring Approach

### Phase 1: Service Layer Completeness

1. Ensure CoreAPIService provides all methods needed by API endpoints
   - Audit all current API endpoints and identify direct ConnectomeManager access
   - Add missing methods to CoreAPIService to cover all ConnectomeManager operations

2. Key methods to implement in CoreAPIService:
   - Complete set of CRUD operations for cortical areas
   - Comprehensive neuron and synapse management
   - Morphology and mapping operations
   - All operations needed for genome management

### Phase 2: API Endpoint Refactoring

For each API router module:

1. Replace direct ConnectomeManager injections:
   ```python
   # BEFORE
   @router.get("/endpoint")
   async def endpoint(connectome: ConnectomeManager = Depends(get_connectome)):
       # Direct access to connectome

   # AFTER
   @router.get("/endpoint")
   async def endpoint(core_api: CoreAPIService = Depends(get_core_api)):
       # Access via service layer
   ```

2. Convert undefined global `connectome` references to explicit service usage

3. Standardize error handling using CoreAPIService's error handling mechanisms

4. Add proper input validation using CoreAPIService validation methods

### Phase 3: Unified Response Format

1. Implement consistent response formatting in CoreAPIService
2. Ensure all API endpoints use these standardized response formats
3. Implement proper HTTP status codes for different error scenarios

### Phase 4: Testing and Validation

1. Write integration tests for each refactored endpoint
2. Verify all endpoints work with the new architecture
3. Add runtime checks to prevent direct ConnectomeManager access

## Implementation Schedule

### Priority 1: Core Operation Endpoints
- `/burst_engine/*` endpoints
- `/cortical_area/*` endpoints
- `/connectome/*` endpoints

### Priority 2: Genome Management
- `/genome/*` endpoints
- `/morphology/*` endpoints

### Priority 3: Peripheral Operation Endpoints
- All remaining endpoints

## Monitoring and Enforcement

1. Create linting rules to detect direct ConnectomeManager imports in API modules
2. Add runtime assertions to verify endpoint dependencies
3. Document the pattern in all API module docstrings

## Additional Architecture Improvements

1. Implement request/response logging middleware
2. Add API rate limiting to protect against abuse
3. Implement request validation middleware
4. Add comprehensive API documentation
