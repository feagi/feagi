# Leak Percentage Validation Complete

## Summary
Implemented consistent 0-100 percentage input in BV with automatic conversion to 0-1 float for API, plus server-side validation.

## Changes

### 1. Brain Visualizer (Godot)
**File**: `brain-visualizer/godot_source/BrainVisualizer/UI/Windows/AdvancedCorticalProperties/AdvancedCorticalProperties.gd`

Added percentage-to-float conversion for leak parameters (matching existing `neuron_excitability` pattern):
```gdscript
elif key_name == "neuron_leak_coefficient":
    # Convert from 0-100 percentage back to 0-1 range for FEAGI API
    value = float(value) / 100.0
elif key_name == "neuron_leak_variability":
    # Convert from 0-100 percentage back to 0-1 range for FEAGI API
    value = float(value) / 100.0
```

### 2. Python API (FEAGI Core)
**File**: `feagi_core/feagi/api/core/services/genome/parameter_updater.py`

#### a) Property Mapping with Validation
Added `_validate_0_1_float` lambda for clamping values to 0.0-1.0 range:
```python
_validate_0_1_float = lambda v: max(0.0, min(1.0, float(v)))

NEURON_PROPERTY_MAPPING = {
    'neuron_excitability': ('excitabilities', _validate_0_1_float, 'Excitability'),
    'leak': ('leak_coefficients', _validate_0_1_float, 'Leak coefficient'),
    'neuron_leak_coefficient': ('leak_coefficients', _validate_0_1_float, 'Leak coefficient'),
    'neuron_leak_variability': ('leak_coefficients', _validate_0_1_float, 'Leak variability'),
    # ... other properties
}
```

#### b) Runtime Validation
Added explicit range checks in Rust NPU update path:
```python
# Excitability validation
if property_name == 'neuron_excitability':
    if not (0.0 <= converted_value <= 1.0):
        self.logger.error(f"❌ Excitability must be in range 0.0-1.0, got {converted_value}")
        return False
    # ... update

# Leak validation (handles leak, leak_coefficient, leak_variability)
elif property_name in ('leak', 'leak_coefficient', 'neuron_leak_coefficient', 'neuron_leak_variability'):
    if not (0.0 <= converted_value <= 1.0):
        self.logger.error(f"❌ Leak coefficient must be in range 0.0-1.0, got {converted_value}")
        return False
    # ... update
```

## Semantics

### Leak Coefficient (`leak`, `neuron_leak_coefficient`)
- **BV Input**: 0-100 (percentage, e.g., 50.0 = 50%)
- **API Receives**: 0.0-1.0 float (e.g., 0.5)
- **Rust Stores**: 0.0-1.0 float
- **Meaning**: Percentage of membrane potential LOST per burst
  - 0.0 = no decay (potential persists indefinitely)
  - 0.5 = lose 50% of potential per burst
  - 1.0 = lose 100% of potential (instant reset to 0)
- **Formula**: `V_new = V_current * (1.0 - leak_coefficient)`

### Leak Variability (`neuron_leak_variability`)
- **BV Input**: 0-100 (percentage, e.g., 10.0 = 10% variability)
- **API Receives**: 0.0-1.0 float (e.g., 0.1)
- **Rust Stores**: NOT stored in neuron array (genome-only parameter)
- **Usage**: Only used during neurogenesis to introduce variation in leak values across neurons
- **Meaning**: Standard deviation of leak coefficient during neuron creation

### Excitability (`neuron_excitability`)
- **BV Input**: 0-100 (percentage, e.g., 75.0 = 75%)
- **API Receives**: 0.0-1.0 float (e.g., 0.75)
- **Rust Stores**: 0.0-1.0 float
- **Meaning**: Neuron's sensitivity to synaptic input (multiplier for incoming PSP)

## Validation Layers

1. **Godot (BV)**: User enters 0-100, converted to 0-1 before sending
2. **Python API**: Clamps to 0.0-1.0 using `_validate_0_1_float` lambda
3. **Rust NPU Update**: Explicit range check, rejects invalid values with error log

## Testing
- BV leak fields now accept percentage values (0-100)
- API correctly clamps values to 0.0-1.0
- Rust NPU receives validated 0.0-1.0 floats
- Invalid values are rejected with clear error messages

### 3. Neuron Properties API Response
**Files**: 
- `feagi_core/feagi/api/core/services/core_api_service.py`
- `feagi_core/feagi/api/v1/schemas.py`

Replaced `decay_rate` with `leak_coefficient` in neuron properties response:
```python
# Old (REMOVED)
properties["decay_rate"] = float(leak_coef)

# New
properties["leak_coefficient"] = float(leak_coef)  # 0.0-1.0 (percentage loss per burst)
```

Updated Pydantic schema:
```python
class NeuronPropertiesResponse(BaseModel):
    # ... other fields
    leak_coefficient: float  # 0.0-1.0 (percentage of potential lost per burst)
    # decay_rate: float  # ❌ REMOVED - was legacy field
```

## API Breaking Change
⚠️ **BREAKING CHANGE**: The `/v1/connectome/neuron/{neuron_id}/properties` endpoint now returns `leak_coefficient` instead of `decay_rate`. Any external clients must update their code.

**Migration**:
- Old: `response["decay_rate"]` (was 1.0 - leak, e.g., 0.9 for 10% loss)
- New: `response["leak_coefficient"]` (direct percentage loss, e.g., 0.1 for 10% loss)

## Related Files
- `feagi-rust/crates/feagi-burst-engine/src/neural_dynamics.rs` (leak formula implementation)
- `feagi/evo/defaults/genome/essential_genome.json` (genome leak defaults)

---
**Status**: ✅ Complete
**Date**: 2025-10-10

