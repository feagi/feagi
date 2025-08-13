# Cortical Area Module Documentation

## Overview
The **Cortical Area Module** manages CRUD operations against cortical area objects including the creation of cortical
areas, reading cortical properties, updating cortical properties, and deleting them. Creation of a cortical area entails
the initialization of neuron properties which will be inherited by neurons during neuron creation in the given cortical
area.

This module also enables multi-cortical area actions where the above mentioned CRUD operations can be performed against
multiple cortical area at the same time.


## Types
There are 4 types of cortical areas:
1. Input Processing Unit (IPU)
2. Output Processing Unit (OPU)
3. Interconnect
4. Memory

`templates.py` captures the unique properties for each cortical area type and subtype.

### Input Processing Unit (IPU) Area
IPU cortical areas are synonymous to brain areas representing sensory neurons and responsible for feeding sensory information to the rest of the
brain.

### Output Processing Unit (OPU) Area
OPU cortical areas are synonymous to brain areas representing motor neurons and responsible for taking motor commands from the brain to
peripherals.

### Interconnect Area
Interconnect cortical areas are generic and responsible for connecting various areas of the brain together.

### Memory Area
Memory area is a special cortical area without any specific 3D topology and capable of storing information in the form of newly
created neurons.


## Data Structures


### **1. `CorticalArea` (Individual Cortical Representation)**
Each cortical area is instantiated with a **cortical ID, type, and spatial dimensions**. It manages neurons within its assigned voxel space.

- **`cortical_id`**: Unique ID for the cortical area.
- **`cortical_type`**: Type of cortical area (e.g., `interconnect`, `memory`, `ipu`, `opu`).
- **`width, height, depth`**: Defines voxel space dimensions.
- **`neurons_per_voxel`**: Number of neurons per voxel.
- **`total_neurons`**: Total neurons assigned to the cortical area.
- **`neuron_start_idx, neuron_end_idx`**: Tracks assigned neurons in `global_neuron_array`.



Cortical area properties:
- label ()
- FEAGI ID (str) [Limited to 6 characters]
- local ID (int32)
- type (int8)  [IPU, OPU, Interconnect, Memory]
- brain region id (int32)
- dimensions (int32, int32, int32)
- 3D location (int32, int32, int32)
- 2D location (int32, int32)
- neuron block capacity (int32)
- neuron firing threshold
- neuron firing threshold increment (int8, int8, int8)
- neuron firing threshold limit
- neuron degeneracy constant (int32)
- neuron psp (float)
- neuron psp uniformity (bool)
- neuron psp max (int32)
- neuron refractory period (int16)
- neuron refractory period overwrite
- neuron leak coefficient
- neuron leak variability
- neuron consecutive fire count
- neuron snooze period
- neuron excitability
- neuron membrane potential accumulation (bool)
- neuron membrane potential driven post synaptic potential
- neuron synapse attractivity (int32)
- memory neuron temporal depth
- memory neuron initial lifespan
- memory neuron lifespan growth rate (int32)
- memory neuron longterm conversion threshold
- **load shedding (__shed)**: If set to true, this cortical area will be skipped (its FCL content dropped) during simulation bursts when the system is unable to maintain the desired burst frequency. Default is false. This property allows selective load reduction under high computational stress.

# Cortical Area Implementation

This document describes the implementation of cortical areas in FEAGI, with a focus on the memory-efficient representation of neurons within 3D spaces of varying dimensions.

## Challenges and Requirements

1. **Varying Dimensions**: Cortical areas can have a wide range of dimensions, from small 3D volumes to extremely large 1D arrays (e.g., 1×1×1000000).

2. **Multiple Neurons Per Voxel**: Each voxel in a cortical area can contain multiple neurons.

3. **Memory Efficiency**: The system must efficiently handle billions of potential neurons without excessive memory overhead.

4. **Fast Lookups**: Operations like finding all neurons in a position or retrieving a neuron's position must be efficient.

5. **Serialization Support**: The representation must support efficient saving and loading.

## Bitmap-Based Neuron Management

To address these challenges, we implement a bitmap-based approach that scales efficiently for varying cortical area dimensions while supporting multiple neurons per voxel.

### Key Data Structures

The implementation uses the following data structures:

#### 1. Neuron Storage (Structure of Arrays)

```python
# Core neuron properties using Structure of Arrays pattern
membrane_potentials = np.zeros(max_neurons, dtype=np.float32)
thresholds = np.zeros(max_neurons, dtype=np.float32)
coordinates_x = np.zeros(max_neurons, dtype=np.uint32)
coordinates_y = np.zeros(max_neurons, dtype=np.uint32)
coordinates_z = np.zeros(max_neurons, dtype=np.uint32)
neuron_indices = np.zeros(max_neurons, dtype=np.uint32)  # Index within voxel
area_ids = np.zeros(max_neurons, dtype=np.uint32)
is_active = np.zeros(max_neurons, dtype=bool)
```

#### 2. Neuron ID Mapping

```python
# Mapping from neuron ID to array index
neuron_id_to_index = {}  # neuron_id -> array_index

# Mapping from array index to neuron ID
index_to_neuron_id = np.zeros(max_neurons, dtype=np.int64)
```

#### 3. Bitmap-Based Position Tracking

```python
# Adaptive tracking of neurons based on cortical area characteristics
area_lookup_tables = {}  # area_id -> appropriate lookup structure

# Bitmap tracking of occupied voxels per area
occupied_voxels = {}  # area_id -> BitMap of linearized positions

# Bitmap tracking of neurons per position
position_to_neurons = {}  # (area_id, linearized_pos) -> BitMap of neuron IDs
```

#### 4. Neuron Position Lookup

```python
# Reverse lookup: neuron ID to position
neuron_to_position = {}  # neuron_id -> (area_id, x, y, z, neuron_index)
```

### Implementation Strategies

For each cortical area, we select an appropriate implementation strategy based on its characteristics:

#### 1. Small Regular Areas (≤ 100³ voxels)

For small cortical areas with regular dimensions, we use a direct array-based lookup:

```python
# 4D array: x, y, z, neuron_index -> neuron_id
lookup = np.zeros((width, height, depth, max_neurons_per_voxel), dtype=np.int64)
```

This approach provides the fastest lookups with reasonable memory consumption.

#### 2. Large Regular Areas (> 100³ voxels)

For large areas with regular dimensions, we use a position linearization with bitmap tracking:

```python
# Store only occupied positions
occupied_positions = BitMap()  # Bitmap of linearized positions that contain neurons

# For each occupied position, store a list/bitmap of neuron IDs
position_neurons = {}  # linearized_position -> BitMap of neuron IDs
```

#### 3. Areas with Extreme Dimensions

For areas with one or more extreme dimensions (e.g., 1×1×1000000), we use a specialized sparse representation:

```python
# Track only populated dimensions
dimension_occupancy = {
    'x': BitMap(),  # Occupied x positions
    'y': BitMap(),  # Occupied y positions
    'z': BitMap()   # Occupied z positions
}

# Use hierarchical linearization to manage extreme dimensions
position_mapping = {}  # (x_block, y_block, z_block) -> {local_position -> neuron_IDs}
```

### Core Operations

#### Creating a Neuron

```python
def create_neuron(area_id, position, neuron_index=0):
    """Create a neuron at the specified position with the given index."""
    # 1. Find available array index
    array_index = get_next_available_index()

    # 2. Generate a unique neuron ID (sequential or derived from a counter)
    neuron_id = next_neuron_id
    next_neuron_id += 1

    # 3. Store neuron properties in the arrays
    store_neuron_properties(array_index, area_id, position, neuron_index)

    # 4. Update mapping dictionaries
    neuron_id_to_index[neuron_id] = array_index
    index_to_neuron_id[array_index] = neuron_id

    # 5. Update position tracking based on area type
    update_position_tracking(area_id, position, neuron_index, neuron_id)

    # 6. Store reverse mapping
    neuron_to_position[neuron_id] = (area_id, *position, neuron_index)

    return neuron_id
```

#### Finding Neurons by Position

```python
def get_neurons_at_position(area_id, position):
    """Get all neurons at a specific position."""
    # Strategy depends on the area's tracking method
    if area_id in small_regular_areas:
        # Direct array lookup
        return get_neurons_from_array(area_id, position)
    else:
        # Bitmap-based lookup
        linearized_pos = linearize_position(area_id, position)
        pos_key = (area_id, linearized_pos)
        return list(position_to_neurons.get(pos_key, BitMap()))
```

#### Finding a Neuron's Position

```python
def get_neuron_position(neuron_id):
    """Get the position and details of a specific neuron."""
    if neuron_id in neuron_to_position:
        return neuron_to_position[neuron_id]
    return None
```

## Memory Footprint Analysis

Let's analyze the memory requirements for a 1000×1000×1000 cortical area with 1 neuron per voxel (1 billion neurons total):

### 1. Core Neuron Properties (Structure of Arrays)

For 1 billion neurons:
- `membrane_potentials`: 1B × 4 bytes = 4GB
- `thresholds`: 1B × 4 bytes = 4GB
- `coordinates_x/y/z`: 3 × 1B × 4 bytes = 12GB
- `neuron_indices`: 1B × 4 bytes = 4GB
- `area_ids`: 1B × 4 bytes = 4GB
- `is_active`: 1B × 1 byte = 1GB

**Subtotal: 29GB**

### 2. Neuron ID Mapping

- `neuron_id_to_index`: 1B entries × ~24 bytes per entry = 24GB
- `index_to_neuron_id`: 1B × 8 bytes = 8GB

**Subtotal: 32GB**

### 3. Bitmap-Based Position Tracking

For a fully populated 1000×1000×1000 area:
- `occupied_voxels`: Using Roaring Bitmap, ~2MB (since all positions are occupied, compression is minimal)
- `position_to_neurons`: For 1B positions with 1 neuron each
  - Key storage: 1B × ~16 bytes = 16GB
  - Value storage (bitmaps): 1B × ~8 bytes = 8GB

**Subtotal: ~24GB**

### 4. Neuron Position Lookup

- `neuron_to_position`: 1B entries × ~40 bytes per entry = 40GB

**Subtotal: 40GB**

### 5. Optimizations

With optimizations:

1. **Sparse Population**: If only 10% of voxels are populated, the position tracking reduces to ~2.4GB

2. **Array-Based Lookup**: For regular dimensions, we can use a 3D array instead of dictionaries:
   - 1000×1000×1000 array of int64: 8GB

3. **Compression**: Applying compression techniques like run-length encoding for positions

4. **Memory-Mapped Storage**: Using memory-mapped files for portions of the data

**Estimated Total with Optimizations: ~60-80GB**

### Scaling Characteristics

The memory usage scales primarily with:
1. Number of active neurons (not potential neurons)
2. Density of neuron distribution in the cortical space

For extreme cases (1×1×1000000):
- If sparsely populated (e.g., 1%), the memory requirements are dominated by the active neuron count, not the potential space
- The bitmap approach efficiently handles this case by tracking only occupied positions

## Implementation Recommendations

1. **Adaptive Strategy Selection**: Choose the representation strategy based on area dimensions and expected density

2. **Lazy Initialization**: Only allocate memory for lookup structures when needed

3. **Hierarchical Bitmaps**: For extreme dimensions, use a hierarchical approach to break down the space

4. **Memory Mapping**: Use memory mapping for persistent storage of large arrays

5. **Custom Serialization**: Implement efficient serialization specifically for bitmap structures

This implementation approach provides an excellent balance of memory efficiency, computational performance, and flexibility for handling the diverse requirements of FEAGI's cortical areas.
