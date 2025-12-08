# Neuron Activation to Percentage Mapping

## Overview

FEAGI uses a **3D coordinate system (XYZP)** to encode motor commands. The **Z-coordinate** directly determines the percentage value in linear encoding mode.

## Coordinate System

Each neuron has coordinates: `(X, Y, Z, Potential)`

- **X-coordinate**: Maps to **channel** (motor index)
  - X=0,1 → Channel 0 (first motor)
  - X=2,3 → Channel 1 (second motor)
  - X=4,5 → Channel 2 (third motor)
  - etc.

- **Y-coordinate**: Must be **0** for motor outputs (1D encoding)

- **Z-coordinate**: Determines the **percentage value** (0 to 100)
  - Z=0 → 0%
  - Z=50 → 50%
  - Z=90 → 90%
  - Z=99 → 99% (max, since z_resolution=100)

- **Potential**: Must be > 0.0 for the neuron to be counted

## Direction Encoding (SignedPercentage)

For bidirectional control (positive/negative), FEAGI uses **two X-coordinates per channel**:

- **Even X** (0, 2, 4, ...): Positive direction
- **Odd X** (1, 3, 5, ...): Negative direction

### Example for Channel 0:
- X=0, Z=90 → Positive 90%
- X=1, Z=90 → Negative 90%
- X=0, Z=1 → Positive 1%
- X=1, Z=1 → Negative 1%

## Decoding Formula

The decoder calculates:

```rust
positive = average(Z-coordinates of positive neurons) / z_max_depth
negative = average(Z-coordinates of negative neurons) / z_max_depth
final_percentage = positive - negative  // Ranges from -1.0 to 1.0
```

Where:
- `z_max_depth = 100` (set in `brain_output.py` line 206)
- If multiple neurons activate, their Z-coordinates are **averaged**

## Current Setup

Your controller uses:
- **z_resolution = 100** (Z-coordinates range from 0 to 99)
- **Linear encoding** (percentage_positioning = 0)

## How to Get 90% Output

To get **90% positive output** on Channel 0:

1. **Activate neurons at X=0, Z=90**
   - X=0 (even → positive direction)
   - Z=90 (90% of z_max_depth)
   - Y=0 (required)
   - Potential > 0.0 (required)

2. **Result**: 
   - `positive = 90 / 100 = 0.90`
   - `negative = 0` (no neurons at X=1)
   - `final_percentage = 0.90 - 0 = 0.90` (90%)

## Why You're Getting 1%

Your logs show `raw=-0.0100` (1%), which means:

- Neurons are activating at **Z=1** (or very low Z-coordinates)
- `positive = 1 / 100 = 0.01` (1%)
- Or `negative = 1 / 100 = 0.01`, giving `final = 0 - 0.01 = -0.01` (-1%)

## How to Fix: Activate Neurons at Higher Z-Coordinates

### In Brain Visualizer or FEAGI UI:

1. **Select your motor cortical area** (e.g., `b3BzZQQAAAA=`)

2. **Activate neurons at Z=90** instead of Z=1:
   - For Channel 0, positive: Activate neurons at **(X=0, Y=0, Z=90)**
   - For Channel 0, negative: Activate neurons at **(X=1, Y=0, Z=90)**

3. **Multiple neurons**: If you activate multiple neurons, their Z-coordinates are averaged:
   - Neurons at Z=85, Z=90, Z=95 → Average = 90 → 90%

### Example Neuron Activations:

| Desired Output | Channel | X-coordinate | Z-coordinate | Result |
|----------------|---------|--------------|--------------|--------|
| +90% | 0 | 0 (even) | 90 | 90% positive |
| -90% | 0 | 1 (odd) | 90 | 90% negative |
| +50% | 0 | 0 (even) | 50 | 50% positive |
| +1% | 0 | 0 (even) | 1 | 1% positive (current) |
| +90% | 1 | 2 (even) | 90 | 90% on second motor |
| +90% | 2 | 4 (even) | 90 | 90% on third motor |

## Channel Mapping Summary

For a controller with N motors:

- **Channel 0**: X=0 (positive), X=1 (negative)
- **Channel 1**: X=2 (positive), X=3 (negative)
- **Channel 2**: X=4 (positive), X=5 (negative)
- **Channel N**: X=2N (positive), X=2N+1 (negative)

## Visual Representation

```
Cortical Area (Z-depth = 100, Y-height = 1)

Channel 0:          Channel 1:          Channel 2:
X=0 (pos) X=1(neg) X=2 (pos) X=3(neg) X=4 (pos) X=5(neg)
   |         |         |         |         |         |
Z=99 ────────┴─────────┴─────────┴─────────┴─────────┴── 99% (max)
   |         |         |         |         |         |
Z=90 ────────┴─────────┴─────────┴─────────┴─────────┴── 90% ⭐ Target
   |         |         |         |         |         |
Z=50 ────────┴─────────┴─────────┴─────────┴─────────┴── 50%
   |         |         |         |         |         |
Z=1  ────────┴─────────┴─────────┴─────────┴─────────┴── 1% (current)
   |         |         |         |         |         |
Z=0  ────────┴─────────┴─────────┴─────────┴─────────┴── 0% (center)
```

## Key Takeaways

1. **Z-coordinate = Percentage**: Z=90 → 90%, Z=1 → 1%
2. **X-coordinate = Channel + Direction**: Even X = positive, Odd X = negative
3. **Multiple neurons average**: Activating Z=85, Z=90, Z=95 averages to 90%
4. **To get 90%**: Activate neurons at **Z=90** (or average to 90)
5. **Current issue**: Neurons activating at Z=1 → only 1% output

## Debugging Tips

If you're still getting 1%:

1. **Check neuron Z-coordinates** in FEAGI logs or Brain Visualizer
2. **Verify cortical area** has `z_resolution=100` (should be automatic)
3. **Ensure neurons have potential > 0.0** (they must be "firing")
4. **Check X-coordinates** match your target channel (X=0,1 for channel 0, etc.)

