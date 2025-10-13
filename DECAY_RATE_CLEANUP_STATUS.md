# Decay Rate to Leak Coefficient Migration Status

## Completed Changes

### 1. API Response (✅ DONE)
- **File**: `feagi/api/core/services/core_api_service.py:729`
  - Changed: `properties["decay_rate"]` → `properties["leak_coefficient"]`
  
- **File**: `feagi/api/v1/schemas.py:659`
  - Changed: `decay_rate: float` → `leak_coefficient: float`

### 2. Public API Methods (✅ DONE)
- **File**: `feagi/bdu/connectome_manager.py`
  - `create_neuron()` - parameter renamed
  - `add_neuron()` - parameter renamed
  - `add_neurons()` - parameter renamed
  - `batch_create_neurons()` - parameter renamed

### 3. Rust Code (✅ DONE)
- Removed dead legacy `feagi-rust/src/lib.rs` entirely
- Updated comment in `feagi-rust/crates/feagi-burst-engine/src/npu.rs:939`

### 4. Brain Visualizer (✅ DONE)
- Added percentage-to-float conversion for `neuron_leak_coefficient` and `neuron_leak_variability`

### 5. API Validation (✅ DONE)
- Added 0.0-1.0 range validation for leak parameters in `parameter_updater.py`

## Remaining Work

### Internal Implementation Details
The following are internal implementation details that don't affect the public API but need cleanup:

1. **Constants** (`feagi/bdu/connectome_manager.py:38`)
   - `DECAY_RATE = "decay_rate"` - Keep for backward compatibility in legacy property getters/setters

2. **Legacy Property Access** (`feagi/bdu/connectome_manager.py:1557, 1611`)
   - `get_neuron_property("decay_rate")` - Maps to leak_coefficient for compatibility
   - `set_neuron_property("decay_rate", value)` - Maps to leak_coefficient for compatibility
   - **Decision**: Keep these for backward compatibility with old tests/code

3. **Internal Helper** (`feagi/bdu/connectome_manager.py:7449`)
   - `_create_neuron_via_npu(decay_rate=...)` - Internal method, needs parameter rename

4. **Genome Service** (`feagi/api/core/services/genome_service.py:2389`)
   - `base_decay_rate = 1.0 - (new_area.get("leak_coefficient", 0) / 100.0)`
   - This is converting from genome's leak_coefficient (0-100) to internal value
   - **INCORRECT FORMULA**: Should be `leak_coefficient / 100.0` (not `1.0 - ...`)

5. **Test Files** (all in `tests/`)
   - Multiple test files still use `decay_rate` parameter
   - These will break when trying to create neurons
   - Need to update all test calls to use `leak_coefficient`

## Critical Fix Needed

### Genome Service Bug
**File**: `feagi/api/core/services/genome/genome_service.py:2389`

**Current (WRONG)**:
```python
base_decay_rate = 1.0 - (new_area.get("leak_coefficient", 0) / 100.0)
```

**Should be**:
```python
base_leak_coefficient = new_area.get("leak_coefficient", 0) / 100.0
```

**Reason**: 
- Old `decay_rate` was inverse: `decay_rate = 1.0 - leak`
- New `leak_coefficient` is direct: `leak = leak / 100.0`
- The genome stores leak as 0-100, needs to convert to 0.0-1.0

## Test Cleanup Strategy

All tests using `decay_rate` need to be updated to use `leak_coefficient`:

```python
# Old
connectome.add_neuron(decay_rate=0.5)

# New
connectome.add_neuron(leak_coefficient=0.5)
```

**Note**: The semantics have changed!
- Old `decay_rate=0.9` meant 10% loss per burst
- New `leak_coefficient=0.1` means 10% loss per burst

## Next Steps

1. ✅ Fix genome service bug (line 2389)
2. ✅ Update `_create_neuron_via_npu` parameter
3. ✅ Update all test files to use `leak_coefficient`
4. ⚠️ **Decision needed**: Keep legacy property getters/setters for compatibility or remove entirely?

---
**Date**: 2025-10-10
**Status**: Migration 80% complete, tests need updating

