# NPU Module Dependencies

*Generated: December 20, 2024*
*Last Updated: After FCL Manager consolidation cleanup*

## Overview

This document maps the internal dependencies within the FEAGI NPU module to help maintain clean architecture and avoid circular imports.

## Dependency Graph

```
fcl_manager.py (Core - No internal dependencies)
    ↑
    ├── gpu_fcl_adapter.py
    ├── burst_engine.py
    └── optimized_integration.py

special_area_handler.py (Core - No internal dependencies)
    ↑
    ├── fcl_injection_service.py
    └── burst_engine.py

fcl_injection_service.py
    ↑
    └── burst_engine.py

optimized_structures.py (Core - No internal dependencies)
    ↑
    └── optimized_integration.py
```

## Core Modules (No Internal Dependencies)

These modules can be imported safely by any other NPU module:

1. **`fcl_manager.py`**
   - Provides: `FCLManager`, `BitMap`, `NeuronId`, `CorticalIdx`, `MembraneUpdate`
   - Used by: gpu_fcl_adapter, burst_engine, optimized_integration

2. **`special_area_handler.py`**
   - Provides: `SpecialAreaHandler`, area detection logic
   - Used by: fcl_injection_service, burst_engine

3. **`optimized_structures.py`**
   - Provides: optimized data structures for performance
   - Used by: optimized_integration

## Dependent Modules

### Level 1 Dependencies
- **`gpu_fcl_adapter.py`** → `fcl_manager.py`
- **`fcl_injection_service.py`** → `special_area_handler.py`
- **`optimized_integration.py`** → `optimized_structures.py`, `fcl_manager.py`

### Level 2 Dependencies
- **`burst_engine.py`** → `special_area_handler.py`, `fcl_injection_service.py`

## Import Guidelines

### ✅ Safe Import Patterns

```python
# Core modules - always safe to import
from feagi.npu.fcl_manager import FCLManager, BitMap
from feagi.npu.special_area_handler import SpecialAreaHandler
from feagi.npu.optimized_structures import OptimizedNeuron

# Level 1 modules - safe to import from non-NPU code
from feagi.npu.gpu_fcl_adapter import create_gpu_accelerated_fcl
from feagi.npu.fcl_injection_service import FCLInjectionService

# Level 2 modules - use cautiously
from feagi.npu.burst_engine import BurstEngine
```

### ❌ Circular Import Risks

Avoid these patterns:
- `fcl_manager.py` importing from any other NPU module
- `special_area_handler.py` importing from other NPU modules
- Cross-dependencies between level 1 modules

### 🔧 Dependency Resolution Strategies

If you need to introduce new dependencies:

1. **Dependency Injection**: Pass dependencies as parameters rather than importing
2. **Lazy Loading**: Import inside functions when needed
3. **Interface Abstraction**: Use protocols/interfaces to break direct dependencies
4. **Factory Pattern**: Use factory functions for complex object creation

## Refactoring History

### December 2024 - FCL Manager Consolidation
- **Issue**: `FCLManager` and `EnhancedFCLManager` were duplicate classes
- **Solution**: Consolidated into single `FCLManager` with all enhanced features
- **Impact**: Simplified dependency graph, maintained backward compatibility with aliases
- **Files Updated**:
  - `fcl_manager.py` - consolidated classes
  - `gpu_fcl_adapter.py` - updated imports to use `FCLManager`

## Validation Commands

To check for dependency issues:

```bash
# Check for circular imports
python3 -c "from feagi.npu import fcl_manager, special_area_handler, burst_engine"

# Validate core modules have no internal dependencies
grep -r "from feagi.npu\." feagi/npu/fcl_manager.py  # Should be empty
grep -r "from feagi.npu\." feagi/npu/special_area_handler.py  # Should be empty

# Check import patterns
grep -r "from feagi\.npu\." feagi/npu/*.py
```

## Future Considerations

- **GPU Backend**: Consider extracting GPU functionality to separate backend module
- **Performance**: Monitor for performance impact of dependency changes
- **Testing**: Update test isolation when dependency structure changes
- **Documentation**: Keep this file updated when adding new modules or dependencies
