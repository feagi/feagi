# Leak Coefficient Fix - Complete

## Problem
Neurons were being created with `leak_coefficient: 1.0` instead of `0.0` despite genome having `leak_c: 0`.

## Root Cause
Two legacy code paths in `genome_service.py` were still using the OLD `decay_rate` formula:
```python
decay_rate = 1.0 - (leak_c / 100.0)
```

This was converting `leak_c: 0` → `decay_rate: 1.0`, which was then being passed to `batch_create_neurons` as if it were `leak_coefficient`.

## Files Fixed

### 1. `/feagi/api/core/services/genome/genome_service.py`

**Line 6315** (Expansion function):
- **Before**: `decay_rate=1.0 - (properties.get("leak_c", 0) / 100.0)`
- **After**: `leak_coefficient=properties.get("leak_c", properties.get("leak_coefficient", 0)) / 100.0`
- Added `leak_variability` and `snooze_period` parameters

**Lines 6611-6641** (Neuron creation with variability):
- **Removed**: Entire `decay_rate` calculation block including variability
- **After**: Simplified to `leak_coefficient` and `leak_variability` (variability now applied inside `batch_create_neurons`)

### 2. `/feagi/bdu/connectome_manager.py`

**Lines 3465-3492** (Property extraction):
- **Removed**: Dead code that calculated `avg_decay_rate` and converted it back to `leak_coefficient`
- **After**: Direct property extraction from cortical area properties (genome-derived)

## Verification

After restarting FEAGI and loading the genome:
- Neurons in `iic000` should have `leak_coefficient: 0.0` (genome has `leak_c: 0`)
- Neurons in `_power` should have `leak_coefficient: 0.1` (genome has `leak_c: 10`)

## Test Command
```bash
curl -s "http://127.0.0.1:8000/v1/connectome/neuron/16438/properties" | python3 -m json.tool | grep leak_coefficient
```

Expected: `"leak_coefficient": 0`

## Status
✅ All `decay_rate` references removed from `genome_service.py`
✅ All neuron creation paths now use `leak_coefficient` (0.0-1.0 range)
✅ Leak variability correctly applied during neurogenesis
✅ Ready for testing with fresh genome load

