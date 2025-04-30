# FCL (Fire Candidate List) Structure Example

## Overview

The FCL Manager maintains a temporal history of neuron firing patterns. With a window size of 3, it keeps track of neurons that fired in the current timestep and the previous 2 timesteps.

## Data Structure

The FCL uses the following primary data structures:

1. **Global FCL History**: A circular array of bitmaps, each representing all neurons that fired in a particular timestep.
2. **Area-Based FCL History**: A dictionary mapping area IDs to their own circular arrays of bitmaps.
3. **Membrane Update Queue**: A collection of pending membrane potential changes to be applied.

## Example with Window Size 3

Let's walk through an example with a window size of 3 and two cortical areas:
- Area 100: A visual processing area
- Area 200: A motor control area

### Initial State (t=0)

```
global_fcl_history = [
    BitMap(),  # t=0 (current)
    BitMap(),  # t-1
    BitMap()   # t-2
]

area_fcl_history = {
    100: [BitMap(), BitMap(), BitMap()],  # Visual area FCLs for t=0, t-1, t-2
    200: [BitMap(), BitMap(), BitMap()]   # Motor area FCLs for t=0, t-1, t-2
}

current_window_index = 0  # Points to the current timestep's position in the circular array
current_timestep = 0
```

### After Some Activity (t=5)

After some simulation steps, neurons have fired:

```
global_fcl_history = [
    BitMap([15, 16, 17]),             # t=5 (current); neurons 15, 16, 17 are firing
    BitMap([10, 11, 12, 13]),         # t=4; neurons 10-13 fired
    BitMap([5, 6, 7])                 # t=3; neurons 5-7 fired
]

area_fcl_history = {
    100: [
        BitMap([15, 16]),             # Area 100 neurons firing at t=5
        BitMap([10, 11]),             # Area 100 neurons that fired at t=4
        BitMap([5, 6])                # Area 100 neurons that fired at t=3
    ],
    200: [
        BitMap([17]),                 # Area 200 neurons firing at t=5
        BitMap([12, 13]),             # Area 200 neurons that fired at t=4
        BitMap([7])                   # Area 200 neurons that fired at t=3
    ]
}

current_window_index = 0  # Currently pointing at t=5
current_timestep = 5
```

### Advancing to t=6

When we advance to the next timestep:

1. The window slides forward (circular index increments)
2. The new current slot is cleared
3. Membrane updates from firing neurons at t=5 are processed
4. New firing neurons for t=6 are determined

```
global_fcl_history = [
    BitMap([10, 11, 12, 13]),         # t=3 (will be overwritten)
    BitMap([15, 16, 17]),             # t=5 becomes t-1
    BitMap([])                        # t=6 (current) - empty until neurons fire
]

area_fcl_history = {
    100: [
        BitMap([10, 11]),             # Will be overwritten
        BitMap([15, 16]),             # t=5 becomes t-1
        BitMap([])                    # Empty until area 100 neurons fire at t=6
    ],
    200: [
        BitMap([12, 13]),             # Will be overwritten
        BitMap([17]),                 # t=5 becomes t-1
        BitMap([])                    # Empty until area 200 neurons fire at t=6
    ]
}

current_window_index = 1  # Currently pointing at t=6
current_timestep = 6
```

### Membrane Update Processing

During the update:

1. Neurons that fired at t=5 (indices 15, 16, 17) influence their post-synaptic targets
2. Membrane potential updates are queued for affected neurons
3. These updates are processed to determine which neurons fire at t=6
4. Newly firing neurons are added to the current FCL (t=6)

```
# Example membrane update queue at t=6:
membrane_updates = [
    (20, 0.8),  # Neuron 20 gets +0.8 potential
    (21, 1.2),  # Neuron 21 gets +1.2 potential
    (22, 0.5)   # Neuron 22 gets +0.5 potential
]

# After processing, if neurons 21 and 30 fire:
global_fcl_history = [
    BitMap([10, 11, 12, 13]),         # t=3 (will be overwritten at t=7)
    BitMap([15, 16, 17]),             # t=5 becomes t-1
    BitMap([21, 30])                  # Neurons that fired at t=6
]
```

## Key Operations

### Getting neurons that fired in the last N steps

To find all neurons that fired in the last 2 timesteps (t=5 and t=6):

```python
# Union of the current FCL and the previous FCL
result = global_fcl_history[current_window_index] | global_fcl_history[(current_window_index - 1) % 3]
# Result: BitMap([15, 16, 17, 21, 30])
```

### Getting consistently active neurons

To find neurons that fired consistently across the last 3 timesteps:

```python
# Intersection of all FCLs in the window
result = global_fcl_history[0] & global_fcl_history[1] & global_fcl_history[2]
# Result: BitMap() (empty - no neurons fired consistently)
```

### Getting area-specific firing patterns

To find neurons in area 100 that fired at the current timestep:

```python
result = area_fcl_history[100][current_window_index]
# If neuron 21 is in area 100: BitMap([21])
```

## Bitmap Benefits

The use of Roaring Bitmaps provides:

1. **Memory efficiency**: Only stores indices of neurons that fired
2. **Fast set operations**: Union, intersection, etc. are highly optimized
3. **Compression**: RoaringBitmap automatically compresses ranges of consecutive integers

This makes the FCL Manager efficient even with millions of neurons and long history windows. 