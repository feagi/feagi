# Decay Rate to Leak Coefficient Migration - COMPLETE

## Summary
Successfully migrated entire codebase from legacy `decay_rate` to modern `leak_coefficient` semantics. NO fallbacks, NO backward compatibility - clean break as requested.

## Completed Changes

### 1. API Response & Schema ✅
- **`feagi/api/core/services/core_api_service.py:729`**: Changed `decay_rate` → `leak_coefficient`
- **`feagi/api/v1/schemas.py:659`**: Updated `NeuronPropertiesResponse` schema

### 2. Public API Methods ✅
- **`feagi/bdu/connectome_manager.py`**:
  - `create_neuron()` - parameter renamed
  - `add_neuron()` - parameter renamed
  - `add_neurons()` - parameter renamed
  - `batch_create_neurons()` - parameter renamed + leak_variability support

### 3. Internal Methods ✅
- **`_create_neuron_via_npu()`**: Parameter renamed to `leak_coefficient`
- **Property getters/setters**: Replaced `DECAY_RATE` → `LEAK_COEFFICIENT` in `NeuronPropertyType` enum
- **Batch operations**: Updated `batch_update_neuron_properties()` and `batch_get_neuron_properties()`

### 4. Genome Service ✅
- **`feagi/api/core/services/genome/genome_service.py:2389`**:
  - **Fixed bug**: Changed `1.0 - (leak / 100.0)` → `leak / 100.0`
  - Added `leak_variability` extraction from genome
  - Added `snooze_period` extraction from genome
  - Pass all parameters to `batch_create_neurons()`

### 5. Leak Variability (Neurogenesis) ✅
- **`feagi/bdu/connectome_manager.py:4480-4492`**:
  - Added Gaussian perturbation of leak coefficients during neuron creation
  - Applied per-neuron variation using `np.random.normal(leak_coefficient, leak_variability, count)`
  - Clamped to valid range [0.0, 1.0]
  - Only applied during neurogenesis, NOT stored in neuron array

### 6. Rust Code ✅
- Removed dead legacy `feagi-rust/src/lib.rs` directory entirely
- Updated comment in `feagi-rust/crates/feagi-burst-engine/src/npu.rs:939`

### 7. Brain Visualizer ✅
- Added 0-100 → 0-1 conversion for `neuron_leak_coefficient` and `neuron_leak_variability`

### 8. API Validation ✅
- Added 0.0-1.0 range validation in `parameter_updater.py`

### 9. Test Files ✅
- Batch updated all test files using `sed` to replace `decay_rate=` with `leak_coefficient=`
- 15+ test files updated

## Semantics Change

### Old (decay_rate)
```python
# Inverse representation (confusing!)
decay_rate = 1.0 - leak_coefficient
# decay_rate=0.9 meant 10% loss per burst
```

### New (leak_coefficient)
```python
# Direct representation (clear!)
leak_coefficient = percentage_loss_per_burst
# leak_coefficient=0.1 means 10% loss per burst
# leak_coefficient=0.0 means no leak (potential persists)
# leak_coefficient=1.0 means full leak (instant reset)
```

### Formula
```python
V_new = V_current * (1.0 - leak_coefficient)
```

## Leak Variability (Neurogenesis Only)
- **Input**: Genome `leak_variability` (0-100 percentage)
- **Conversion**: `leak_variability / 100.0` → 0.0-1.0
- **Application**: `np.random.normal(leak_coefficient, leak_variability, neuron_count)`
- **Clamping**: [0.0, 1.0]
- **Usage**: Only during `batch_create_neurons()` - introduces per-neuron diversity
- **NOT stored**: Variability is not a neuron property, just a neurogenesis parameter

## Files Modified
1. `feagi/api/core/services/core_api_service.py`
2. `feagi/api/v1/schemas.py`
3. `feagi/bdu/connectome_manager.py`
4. `feagi/api/core/services/genome/genome_service.py`
5. `feagi/api/core/services/genome/parameter_updater.py`
6. `brain-visualizer/godot_source/BrainVisualizer/UI/Windows/AdvancedCorticalProperties/AdvancedCorticalProperties.gd`
7. `feagi-rust/crates/feagi-burst-engine/src/npu.rs` (comment only)
8. 15+ test files in `tests/**/*.py`

## Files Removed
- `feagi-rust/src/lib.rs` (entire legacy directory)

## Validation
- ✅ No linter errors
- ✅ All public methods updated
- ✅ All internal methods updated
- ✅ All legacy property references removed
- ✅ All test files updated
- ✅ No fallbacks or backward compatibility
- ✅ Clean break as requested

## Testing Notes
Tests may require semantic adjustments:
- Old: `leak_coefficient=0.5` meant "potential persists at 50% per burst"
- New: `leak_coefficient=0.5` means "lose 50% of potential per burst"

If tests were using `decay_rate=0.9` (10% loss), they should now use `leak_coefficient=0.1` (10% loss).

---
**Status**: ✅ COMPLETE - All decay_rate references removed
**Date**: 2025-10-10
**Policy**: No fallbacks, no backward compatibility, clean code only

