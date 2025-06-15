# BDU Connectivity Module Architecture

## Overview
The connectivity module is responsible for defining and implementing the rules that govern how neurons connect between cortical areas during brain development.

## Module Responsibilities

### Core Principle: Single Responsibility
Each module has ONE clear responsibility and does NOT duplicate functionality from other modules.

## Module Structure

### 1. `synapse_rule.py` (Base Classes)
**RESPONSIBILITY**: Define base interfaces for synapse formation rules
- `SynapseRule` - Base class for all synapse rules
- **NO IMPLEMENTATION DETAILS** - only interfaces

### 2. `function_rules.py` (Function-based Rules)
**RESPONSIBILITY**: Mathematical function-based synapse rules
- `OneToOneRule` - One-to-one position mapping
- `DistanceBasedRule` - Distance-based connections

### 3. `vector_rules.py` (Vector-based Rules)
**RESPONSIBILITY**: Vector operation-based synapse rules
- `RandomRule` - Random connection patterns

### 4. `pattern_rules.py` (Pattern-based Rules)
**RESPONSIBILITY**: Pattern matching-based synapse rules
- `PatternRule` - Pattern-based connections

### 5. `synaptogenesis_rules.py` (Core Functions)
**RESPONSIBILITY**: Core synaptogenesis functions and morphology implementations
- `find_candidate_neurons()` - Main entry point for finding connection targets
- `syn_projector()`, `syn_expander_x()`, etc. - Morphology-specific functions
- **NO DUPLICATE CLASSES** - only functions

### 6. `connectivity_rules.py` (High-level Rules)
**RESPONSIBILITY**: High-level connectivity patterns between cortical areas
- `ConnectivityRule` - Base for area-to-area connectivity
- `ProbabilisticRule`, `DistanceBasedConnectivityRule` - Implementations

### 7. `cortical_mappings.py` (Mapping Management)
**RESPONSIBILITY**: Manage mappings between cortical areas
- `CorticalMapping` - Base mapping class
- `DirectMapping`, `ProjectionMapping` - Specific mapping types
- `CorticalMappingRestrictionsRegistry` - Validation rules

### 8. `mapping_utils.py` (Utilities)
**RESPONSIBILITY**: Utility functions for mapping operations
- Helper functions for mapping calculations
- **NO BUSINESS LOGIC** - only utilities

## Architecture Rules

### STRICT PROHIBITIONS
1. **NO DUPLICATE CLASSES** - Each class exists in exactly ONE file
2. **NO CIRCULAR IMPORTS** - Clear dependency hierarchy
3. **NO FALLBACKS** - Imports work or fail clearly
4. **NO BUSINESS LOGIC IN UTILS** - Utils are pure functions only

### Import Hierarchy (Top to Bottom)
```
synapse_rule.py (base interfaces)
    ↑
function_rules.py, vector_rules.py, pattern_rules.py (implementations)
    ↑
synaptogenesis_rules.py (core functions)
    ↑
connectivity_rules.py (high-level rules)
    ↑
cortical_mappings.py (mapping management)
    ↑
mapping_utils.py (utilities)
```

### Performance Requirements
- **Rust/SIMD/GPU/RTOS friendly** - No dynamic behavior
- **Sub-millisecond synapse creation** - Optimized algorithms
- **Spatial indexing** - O(1) position lookups
- **Memory efficient** - Minimal allocations

## Current Violations to Fix

### CRITICAL VIOLATIONS
1. **Duplicate SynapseRule class** in `synaptogenesis_rules.py` (lines 1416+)
2. **Mixed responsibilities** - synaptogenesis_rules.py contains both functions AND classes
3. **Import order violations** - E402 errors throughout
4. **Circular import potential** - synaptogenesis_rules/__init__.py importing from parent

### CLEANUP PLAN
1. ✅ Remove duplicate SynapseRule class from synaptogenesis_rules.py
2. ✅ Move OneToOneRule, DistanceBasedRule to function_rules.py
3. ✅ Move RandomRule to vector_rules.py
4. ✅ Create PatternRule in pattern_rules.py
5. ✅ Fix all import order violations (E402)
6. ✅ Remove synaptogenesis.py (legacy duplicate)
7. ✅ Ensure clean import hierarchy

## Testing Strategy
- Each module tested independently
- No mocking of components under test
- Performance benchmarks for critical paths
- Integration tests for end-to-end synaptogenesis

## Migration Notes
- This cleanup maintains API compatibility
- All existing functionality preserved
- Performance improvements expected
- Cleaner separation of concerns
