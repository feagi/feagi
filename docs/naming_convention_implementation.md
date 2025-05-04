# Implementation of Consistent Naming Convention for Cortical Areas

This document summarizes the changes made to establish consistent naming conventions for cortical area identifiers throughout the FEAGI codebase.

## Background

FEAGI uses two distinct types of identifiers for cortical areas:

1. **`cortical_id`**: A 6-character unique identifier from the genome (e.g., "iv00_C")
2. **`cortical_idx`**: An auto-incremented integer ID used internally for efficient indexing

Previously, the codebase inconsistently used terms like `area_id`, `i`, and `area_id_int` to refer to the internal integer ID. This caused confusion and made code maintenance difficult.

## Changes Made

### File: `docs/naming_convention.md`

- Updated documentation to clearly explain the two types of identifiers
- Added code examples showing the correct usage of `cortical_idx` vs. the deprecated `area_id`
- Added warnings about the deprecated terms
- Improved the explanation of the mapping between identifier types

### File: `feagi/npu/fcl_manager.py`

- Changed type definition from `AreaId = int` to `CorticalIdx = int`
- Renamed all parameters, variables, and comments that used `area_id` to `cortical_idx`
- Updated all function signatures to use `cortical_idx` instead of `area_id`
- Updated class attribute and dictionary keys to use the new naming convention
- Improved comments and docstrings to use consistent terminology
- Updated example usage functions to demonstrate the proper naming convention

### File: `feagi/bdu/connectome_manager.py`

- Updated the `add_cortical_area` method to use `cortical_idx` in its implementation
- Kept the external parameter name `area_id` for backward compatibility, but added comments explaining this
- Improved docstrings to clearly indicate that the `area_id` parameter represents `cortical_idx`
- Enhanced code comments to make the meaning of each identifier clearer

### File: `feagi/api/rest/routers/v1/cortical_area.py`

- Updated Path parameter descriptions to clearly indicate that `area_id` is a string representation of `cortical_idx`
- Improved function docstrings to use consistent terminology
- Fixed parameter naming in function signatures for consistency

## Backward Compatibility

To maintain backward compatibility with existing code:

1. Public API methods continue to accept parameters named `area_id` but add clarifying comments
2. Internal implementations use `cortical_idx` consistently
3. API routers continue to use URL paths with `/area_id/` but clarify that this represents `cortical_idx`

## Future Work

Remaining tasks to fully implement the naming convention:

1. Update additional modules that use `area_id` such as:
   - Burst Engine implementation
   - Remaining REST API endpoints
   - Test files and utilities
2. Eventually refactor the public API to use `cortical_idx` as parameter names, with deprecation warnings for `area_id`

## Guidelines for New Code

When writing new code or modifying existing modules:

1. Always use `cortical_id` for the 6-character genome identifier
2. Always use `cortical_idx` for the integer-based internal ID
3. Never use `area_id` or other variations for new code
4. When modifying existing functions, add appropriate comments when backward compatibility requires keeping `area_id` parameter names

By following these guidelines, we'll gradually achieve complete consistency throughout the codebase. 