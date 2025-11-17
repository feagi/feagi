# FEAGI Refactoring Plan

This document outlines the plan for refactoring the FEAGI codebase to improve organization, consistency, and maintainability.

## Current Structure Issues

1. **Inconsistent Module Organization**:
   - Mix of old and new code styles
   - Inconsistent naming conventions
   - Unclear boundaries between modules

2. **Incomplete Documentation**:
   - Missing module-level docstrings
   - Inconsistent function documentation
   - Lack of usage examples

3. **Mixed Dependency Management**:
   - Multiple requirements files
   - Inconsistent version specifications

## Refactoring Goals

1. **Standardize Module Structure**:
   - Each module should follow a consistent organization pattern
   - Clear separation of concerns between modules
   - Consistent naming conventions

2. **Improve Documentation**:
   - Add comprehensive docstrings to all modules
   - Document all public APIs
   - Include usage examples for key components

3. **Consolidate Dependency Management**:
   - Single source of truth for dependencies
   - Clear separation of core vs. optional dependencies
   - Version pinning strategy

## Standardized Module Structure

All FEAGI modules should follow this structure:

```
feagi/
├── <module_name>/
│   ├── __init__.py         # Exports public API, version info
│   ├── constants.py        # Module-specific constants
│   ├── exceptions.py       # Custom exceptions
│   ├── interfaces.py       # Abstract classes/protocols
│   ├── models.py           # Data models using Pydantic
│   ├── <core_component>.py # Implementation files
```

## Module Documentation Standard

Each module should include:

1. **Module-level docstring**:
   ```python
   """
   FEAGI <Module Name>

   This module provides <brief description>.

   Key components:
   - Component1: Description
   - Component2: Description

   Example usage:
   ```python
   from feagi.<module> import Component
   # Basic usage example
   ```
   """
   ```

2. **Class/function documentation**:
   ```python
   def function_name(param1: type, param2: type) -> return_type:
       """
       Short description of function purpose.

       Args:
           param1: Description of param1
           param2: Description of param2

       Returns:
           Description of return value

       Raises:
           ExceptionType: When and why this exception occurs
       """
   ```

## Implementation Plan

### Phase 1: Analysis and Planning
- [x] Document current structure issues
- [x] Define standardized module structure
- [x] Define documentation standards
- [ ] Identify high-priority modules for refactoring

### Phase 2: Core Infrastructure Refactoring
- [ ] Refactor `feagi/core` to follow standard structure
- [ ] Refactor `feagi/config` to follow standard structure
- [ ] Update main module imports and initialization

### Phase 3: Component Refactoring
- [ ] Refactor NPU components
- [ ] Refactor BDU components
- [ ] Refactor PNS components
- [ ] Refactor API components

### Phase 4: Test and Documentation Updates
- [ ] Update tests to reflect new structure
- [ ] Add missing tests for refactored components
- [ ] Complete documentation for all refactored modules

## Migration Strategy

To minimize disruption during refactoring:

1. Implement changes in feature branches
2. Use deprecation warnings for changing APIs
3. Maintain backward compatibility where possible
4. Add comprehensive tests before and after refactoring
5. Update documentation simultaneously with code changes

## Progress Tracking

| Module | Structure Refactored | Documentation Updated | Tests Updated | Complete |
|--------|----------------------|----------------------|---------------|----------|
| core   | □                    | □                    | □             | □        |
| config | □                    | □                    | □             | □        |
| npu    | □                    | □                    | □             | □        |
| bdu    | □                    | □                    | □             | □        |
| pns    | □                    | □                    | □             | □        |
| api    | □                    | □                    | □             | □        |
