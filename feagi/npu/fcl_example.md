# FCL (Fire Candidate List) Example

This file provides examples of FCL usage in the NPU module.

## Overview

The FCL (Fire Candidate List) is a core data structure in FEAGI that represents which neurons are firing at any given point in time. It uses roaring bitmaps for efficient storage and retrieval of sparse firing patterns.

## Examples

```python
# Get the global FCL for the current burst
current_activity = fcl_manager.get_global_fcl()

# Get FCL history for the past 3 timesteps
recent_activity = fcl_manager.get_neurons_fired_in_last_n_steps(3)

# Get FCL for a specific cortical area
visual_cortex_activity = fcl_manager.get_area_fcl("visual_cortex")
```

## Performance Considerations

The FCL is designed for high-performance operations:

- Roaring bitmap representation for memory efficiency
- Optimized set operations (union, intersection)
- Parallel processing of different FCL sections

## Integration with GPU Acceleration

When using GPU acceleration, the FCL is efficiently transferred between CPU and GPU memory to minimize data transfer overhead. 