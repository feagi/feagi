# BDU Connectivity Module Architecture

## Overview
The connectivity module is responsible for defining and implementing the rules that govern how neurons connect between cortical areas during brain development.

## Current Module Structure

### Core Architecture
The connectivity module follows a clean separation of concerns with specialized rule modules:

```
feagi/bdu/connectivity/
├── rules/                           # Rule implementations by type
│   ├── __init__.py                 # Package exports
│   ├── functions.py                # Function-based morphology rules (syn_* functions)
│   ├── patterns.py                 # Pattern-based connectivity rules
│   └── vectors.py                  # Vector-based connectivity rules
├── synaptogenesis.py               # Main orchestration logic
├── cortical_mappings.py            # Cortical mapping utilities
├── mapping_utils.py                # Mapping helper functions
└── __init__.py                     # Package exports
```

## Module Responsibilities

### 1. `synaptogenesis.py` (Main Orchestration)
**RESPONSIBILITY**: Core synaptogenesis orchestration and entry points
- `find_candidate_neurons()` - Main entry point for finding connection targets
- `neighbor_finder()` - Spatial neighbor finding utilities
- Position utilities (`linearize_position`, `delinearize_position`)
- Expression evaluation utilities
- **Imports and orchestrates** rule implementations from `rules/` package

### 2. `rules/functions.py` (Function-based Morphologies)
**RESPONSIBILITY**: Function-based morphology implementations (`syn_*` functions)
- `syn_projector()` - Complex topology-preserving projections
- `syn_expander_x()` - Coordinate expansion mappings
- `syn_reducer_x()` - Coordinate reduction mappings
- `syn_randomizer()` - Random position selection
- `syn_lateral_pairs_x()` - Lateral connection patterns
- `syn_block_connection()` - Block-to-block mappings
- `syn_memory()` - Memory register operations
- `syn_last_to_first()` - Last-to-first neuron connections

### 3. `rules/vectors.py` (Vector-based Rules)
**RESPONSIBILITY**: Vector operation-based connectivity rules
- `match_vectors()` - Core vector matching function
- `apply_vector_offset()` - Vector offset calculations
- `validate_vector_position()` - Position validation
- `generate_vector_candidates()` - Candidate position generation
- Expression evaluation utilities for vector operations

### 4. `rules/patterns.py` (Pattern-based Rules)
**RESPONSIBILITY**: Pattern matching-based connectivity rules
- `check_pattern_validity()` - Pattern validation
- `find_source_coordinates()` - Source coordinate generation from patterns
- `find_destination_coordinates()` - Destination coordinate generation from patterns
- `define_subregions()` - Subregion definition for targeted synaptogenesis
- Pattern matching utilities and helpers

### 5. `cortical_mappings.py` (Mapping Management)
**RESPONSIBILITY**: Manage mappings between cortical areas
- `CorticalMapping` - Base mapping class
- `DirectMapping`, `ProjectionMapping` - Specific mapping types
- `CorticalMappingRestrictionsRegistry` - Validation rules

### 6. `mapping_utils.py` (Utilities)
**RESPONSIBILITY**: Utility functions for mapping operations
- `get_detailed_cortical_map()` - Build cortical area mapping dictionaries
- `build_power_connections()` - Create power connections for cortical areas
- **Pure utility functions** - no business logic

## Architecture Principles

### Design Rules
1. **Function-based Morphologies**: Morphology rules are implemented as `syn_*` functions, not classes
2. **Clear Separation**: Each rule type has its own module (functions, patterns, vectors)
3. **Single Responsibility**: Each module has one clear purpose
4. **Import Hierarchy**: Clean dependency flow without circular imports
5. **Performance Optimized**: Rust/SIMD/GPU/RTOS friendly implementations

### Import Hierarchy
```
rules/ (specialized implementations)
    ↑
synaptogenesis.py (orchestration)
    ↑
cortical_mappings.py (high-level mappings)
    ↑
mapping_utils.py (utilities)
```

### Function-based Morphology Architecture
The system uses a function-based approach where:
- **Morphology types** like "projector" directly call `syn_projector()` function
- **Dynamic dispatch** by function name (e.g., `neuron_morphology == "projector"` → `syn_projector()`)
- **No OOP classes** for morphology rules - pure functions for performance

## Rule Types

### 1. Function-based Morphologies
- Called directly by name from morphology definitions
- Examples: `syn_projector`, `syn_expander_x`, `syn_randomizer`
- High-performance implementations optimized for speed

### 2. Vector-based Morphologies
- Use `match_vectors()` function with vector parameters
- Support both tuple vectors `(1,0,0)` and algebraic expressions `"x+1,y,z"`
- Morphology definitions contain vector lists

### 3. Pattern-based Morphologies
- Use pattern matching with `find_destination_coordinates()`
- Support wildcards (`*`), conditionals (`?`), and exclusions (`!`)
- Complex pattern transformations between source and destination areas

## Performance Characteristics
- **Sub-millisecond synapse creation** for typical morphologies
- **Memory efficient** - minimal allocations during synaptogenesis
- **Spatial indexing ready** - position-based operations optimized
- **Rust migration friendly** - static typing and minimal dynamic behavior

## Testing Strategy
- Each rule module tested independently
- Integration tests for synaptogenesis orchestration
- Performance benchmarks for critical morphology functions
- No mocking of components under test (per project rules)

## Recent Refactoring (Completed)
✅ **Function Migration**: Moved `syn_*` functions from `synaptogenesis_rules.py` to `rules/functions.py`
✅ **Vector Functions**: Moved `match_vectors` to `rules/vectors.py`
✅ **Pattern Functions**: Moved pattern functions to `rules/patterns.py`
✅ **Clean Architecture**: Established clear separation of concerns
✅ **Import Structure**: Fixed all import dependencies and circular import issues
✅ **Package Organization**: Created proper `rules/` package with clean exports
