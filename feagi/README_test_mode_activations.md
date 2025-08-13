# FEAGI Test Mode - Predictable Neuron Activations

## Overview

The FEAGI test mode now supports **predictable neuron activations** through a JSON configuration file. This allows for deterministic, repeatable testing by specifying exactly which neurons should be activated at specific coordinates.

## How It Works

1. **Automatic Detection**: When test mode starts, it automatically looks for `test_mode_activations.json` in the same folder as `test_mode.py`
2. **Predictable Mode**: If the file exists and contains valid data, test mode switches to predictable activation mode
3. **Deterministic Testing**: Only the neurons specified in the JSON file will be activated - no random selection occurs

## JSON File Format

Create a file named `test_mode_activations.json` with the following structure:

```json
{
  "cortical_id": [[x,y,z], [x,y,z], [x,y,z]],
  "cortical_id": [[x,y,z], [x,y,z], [x,y,z]],
  "cortical_id": [[x,y,z], [x,y,z], [x,y,z]]
}
```

### Example

```json
{
  "CIHMot": [[0,0,0], [1,1,1], [2,2,2]],
  "CJWM3_": [[0,0,0], [0,1,0], [1,0,0], [1,1,0]],
  "CKQM2_": [[0,0,0], [0,0,1], [0,0,2]],
  "CKYM2_": [[1,1,1], [2,2,2], [3,3,3], [4,4,4], [5,5,5]]
}
```

## Usage

### Enable Predictable Mode
1. Create `test_mode_activations.json` in the `feagi_core/feagi/` folder
2. Add your desired cortical areas and neuron coordinates
3. Run FEAGI with test mode as usual

### Disable Predictable Mode
1. Delete or rename `test_mode_activations.json`
2. Test mode will automatically fall back to random neuron selection

## Log Output

When predictable mode is active, you'll see logs like:
```
✅ Predictable neuron injection enabled:
   📊 4 cortical areas with 17 total neurons to activate
   🎯 Will inject ONLY these neurons (no random selection)
🎯 Injected 17 PREDICTABLE neurons across 4 areas
```

When using random mode:
```
No test_mode_activations.json found - using random neuron injection
🎲 Injected 1247 RANDOM neurons across 23 areas
```

## Validation

The system validates:
- ✅ JSON file format and structure
- ✅ Cortical area IDs exist in the loaded genome
- ✅ Coordinate format is `[x,y,z]` with numeric values
- ✅ Neurons exist at the specified coordinates

Invalid entries are logged as warnings but don't stop the test.

## Benefits

- **Reproducible Results**: Same neurons activated every time
- **Targeted Testing**: Test specific neural pathways
- **Debugging**: Isolate issues to specific cortical areas
- **Visualization Testing**: Generate predictable data for visualization validation
- **Performance Testing**: Consistent load for benchmarking

## File Location

Place the JSON file here:
```
feagi_core/feagi/test_mode_activations.json
```

The file should be in the same directory as `test_mode.py`.
