# FEAGI Enhanced Test Modes

This module provides two enhanced test modes for FEAGI neural network testing and validation.

## Overview

The FEAGI test system now supports two distinct modes:

- **Test Mode 1**: JSON-based predictable neuron activations (original functionality)
- **Test Mode 2**: Numpy-based scalable random neuron generation (new for scalability testing)

## Test Mode 1: JSON-based Predictable Activations

### Purpose
Provides deterministic, repeatable testing by specifying exactly which neurons should be activated at specific coordinates.

### Usage
```bash
# Use existing --test flag (defaults to mode 1)
feagi --test --test-duration 30

# Explicitly specify mode 1
feagi --test-mode-1 --test-duration 30
```

### Configuration
Uses `test_mode_activations.json` file in the same directory as the original test_mode.py:

```json
{
  "cortical_id": [[x,y,z], [x,y,z], [x,y,z]],
  "cortical_id": [[x,y,z], [x,y,z], [x,y,z]]
}
```

### Example JSON
```json
{
  "CIHMot": [[0,0,0], [1,1,1], [2,2,2]],
  "CJWM3_": [[0,0,0], [0,1,0], [1,0,0], [1,1,0]],
  "CKQM2_": [[0,0,0], [0,0,1], [0,0,2]],
  "iv00BL": [[1,0,0], [3,0,0]],
  "iv00BM": [
    [0,0,0], [1,0,0], [2,0,0],
    [0,2,1], [1,2,1], [2,2,1],
    [0,4,2], [1,4,2], [2,4,2]
  ]
}
```

### Features
- ✅ **Reproducible Results**: Same neurons activated every time
- ✅ **Targeted Testing**: Test specific neural pathways
- ✅ **Debugging**: Isolate issues to specific cortical areas
- ✅ **Fallback Support**: Falls back to random selection if JSON unavailable

## Test Mode 2: Numpy-based Scalable Random Generation

### Purpose
Generates large-scale random neuron stimulations using numpy for scalability testing and performance evaluation.

### Usage
```bash
# Basic mode 2 usage
feagi --test-mode-2 --test-duration 60
```

### Features
- 🎲 **Scalable Generation**: Uses numpy for efficient large-scale neuron selection
- 📊 **Random Selection**: Selects random neurons from available cortical areas
- 🔢 **Configurable Range**: 100-5000 neurons per area by default
- 📈 **Area Coverage**: Tests 70% of available areas by default

## Common Parameters

Both test modes support these common parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--test-duration` | 10 | Test duration in seconds |
| `--test-frequency` | 10 | Frequency of neuron injection in Hz |

## Output and Logging

### Test Mode 1 Output
```
🎯 TEST MODE 1: JSON-based predictable neuron activations
✅ Predictable neuron injection enabled:
   📊 4 cortical areas with 17 total neurons to activate
   🎯 Will inject ONLY these neurons (no random selection)
🎯 Injected 17 PREDICTABLE neurons across 4 areas
TEST PASSED: Neural activity detected in 4 areas
```

### Test Mode 2 Output
```
🎲 TEST MODE 2: Numpy-based scalable random neuron generation
   📊 Available cortical areas: 23
   🧠 Total available neurons: 2,456,789
   🎯 Selected areas for testing: 16
   🔢 Neurons per area range: 100-5000
🎲 Generated 49,987 RANDOM neurons across 16 areas
TEST PASSED: Neural activity detected in 16 areas
```

## Architecture and File Organization

```
feagi_core/feagi/utils/test_mode/
├── __init__.py                 # Module exports
├── README.md                   # This documentation
├── test_runner.py              # Main test runner coordination
├── test_mode_1.py             # JSON-based predictable activations
├── test_mode_2.py             # Numpy-based scalable generation
└── test_mode_activations.json # Example JSON configuration
```

## Integration with Existing Systems

### Backwards Compatibility
- The original `--test` flag still works and defaults to Test Mode 1
- Existing `test_mode_activations.json` files are automatically detected
- Original test_mode.py is kept for compatibility (can be deprecated later)

## Best Practices

### Test Mode 1 Usage
- Use for **regression testing** and **specific bug reproduction**
- Create JSON files for **specific test scenarios**
- Ideal for **debugging** and **validation**

### Test Mode 2 Usage
- Use for **performance testing** and **scalability validation**
- Test system under **large-scale neural activity**
- Validate **system stability** with random patterns

### General Guidelines
- Start with short durations (`--test-duration 10-30`) for initial validation
- Increase duration for stability and performance testing
- Monitor system resources during large-scale tests (Mode 2) 