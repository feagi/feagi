# FEAGI Data Structures Architecture

*Last Updated: January 15, 2025*

This document describes the comprehensive data structures architecture used in FEAGI 2.0, focusing on the Python implementation, performance optimization, and future SIMD/GPU optimization and Rust/RTOS migration.

## Overview

FEAGI employs a comprehensive data structure strategy:
1. **General Neural Data Structures**: For brain simulation and connectivity
2. **High-Performance Transmission Structures**: For real-time neural data extraction and network transmission
3. **Spatial Indexing Structures**: For efficient neuron location management and spatial queries

All structures are designed with Rust/RTOS compatibility and GPU optimization in mind.

### Spatial Hash System

FEAGI uses a **Morton encoding spatial hash system** for efficient neuron location management:

- **Location:** `feagi/bdu/morton_spatial_hash.py`, `feagi/bdu/spatial_hash.py`
- **Type:** Morton encoding + Roaring bitmaps with direct API access
- **Purpose:** Provides 95%+ memory savings for sparse neural genomes while maintaining spatial locality
- **Performance:** O(log N) region queries, microsecond multi-area operations
- **Compatibility:** 100% backward compatible through compatibility methods
- **Multiple Neurons per Coordinate:** Fixed critical bug - now supports multiple neurons at same coordinate
- **State Manager Integration:** Automatic registration of coordinate limits for system-wide validation
- **Cortical Area Validation:** Prevents creation of areas exceeding Morton encoding limits (21-bit per dimension)

For detailed information, see [Morton Spatial Hash Architecture](arch-morton-spatial-hash.md).

---

## Part I: Core Neural Data Structures

### 1. NeuronArray (Enhanced with Embedded Optimizations)
- **Location:** `feagi/bdu/models/neuron.py`
- **Type:** Enhanced array structure with cache-aligned storage and SIMD operations
- **Purpose:** Stores membrane potentials, firing states, and other per-neuron properties with integrated embedded optimizations for 10M+ neuron capacity.
- **GPU/SIMD Suitability:**
    - **Highly optimized** for SIMD and GPU with cache-aligned memory (64-byte alignment)
    - **Block-sparse matrices** for efficient connectivity representation
    - **SIMD-vectorized operations** using Numba JIT compilation
    - **Zero-allocation paths** for embedded deployment
- **Current Features:**
    - ✅ **Cache-aligned arrays** for optimal SIMD performance
    - ✅ **Embedded optimization methods** integrated by default
    - ✅ **10M neuron capacity** with 15Hz target performance
    - ✅ **Automatic backend selection** (Rust/SIMD/NumPy)

### 2. FireCandidateList (FCL)
- **Location:** `feagi/npu/fcl_manager.py`, `feagi/npu/optimized_integration.py`
- **Type:** `numpy.ndarray` (int32) or Python list (fallback)
- **Purpose:** Holds indices of neurons that are candidates to fire in the next step.
- **GPU/SIMD Suitability:**
    - When using NumPy arrays, can be processed in parallel.
    - Rust extension provides further speedup; fallback is slower but compatible.
- **Current Limitations:**
    - Fallback implementation is not as SIMD-friendly. Migration to always use arrays is recommended.

### 3. Connectome (Enhanced with Block-Sparse Matrices)
- **Location:** `feagi/bdu/connectome_manager.py`, `feagi/bdu/models/neuron.py`
- **Type:**
    - **Enhanced**: Block-sparse matrices (64×64 blocks) with cache-aligned storage
    - **Legacy Support**: Dict-based structures for backward compatibility
    - **Optimized**: Flat arrays for pre/post indices and weights (NumPy arrays)
- **Purpose:** Represents synaptic connections between neurons with embedded optimization for high-performance processing.
- **GPU/SIMD Suitability:**
    - ✅ **Block-sparse matrices** optimized for cache locality and SIMD operations
    - ✅ **64×64 blocks** fit perfectly in L1 cache (16KB)
    - ✅ **Automatic backend selection** for optimal performance
    - ✅ **Zero-allocation paths** for embedded deployment
- **Current Features:**
    - ✅ **Cache-friendly memory access** patterns for embedded systems
    - ✅ **SIMD-optimized connectivity processing** integrated by default
    - ✅ **10M+ neuron connectivity** with 15Hz target performance

### 4. Synapse
- **Location:** `feagi/bdu/connectome_manager.py`, `feagi/npu/optimized_integration.py`
- **Type:**
    - Python: Class with fields (pre, post, weight, delay, etc.)
    - Optimized: Structured NumPy arrays or separate arrays per field.
- **Purpose:** Encapsulates a single synaptic connection.
- **GPU/SIMD Suitability:**
    - Structured arrays or SoA (Structure of Arrays) are best for GPU.
    - Ongoing migration from OOP to SoA for all critical simulation paths.

### 5. Propagation & Update Functions
- **Location:** `feagi/npu/optimized_integration.py`, `feagi/npu/fcl_manager.py`
- **Type:** Vectorized functions operating on arrays.
- **Purpose:** Handles propagation of activations, updates of membrane potentials, and firing logic.
- **GPU/SIMD Suitability:**
    - All major functions are written to operate on arrays, ready for GPU offload.
    - Some edge-case logic still uses Python loops; these are being refactored.

---

## Part II: High-Performance Neural Data Transmission

### System Requirements

FEAGI neural data transmission is optimized for:

1. **Network structure:** Neurons exist in large 3D cortical areas. Each cortical area has a unique integer ID internally, but must be represented as a fixed 6-character ASCII string ID for transmission.

2. **Sparse firing:** At each timestep, only a sparse subset of neurons fire in each cortical area. These are stored as roaring bitmaps keyed by cortical area IDs.

3. **Data to transmit:** For each firing neuron, extract:
   - Coordinates (x, y, z) as int32 arrays
   - Membrane potentials (p) as float32 arrays

4. **Performance focus:**
   - No concern for human readability.
   - Maximum memory and transmission efficiency.
   - Avoid costly serialization formats like JSON.
   - Minimize CPU/GPU data copying.
   - Exploit SIMD/SoA layouts.

### Internal Representation (SoA)

Store neuron properties in Structure of Arrays (SoA) form to optimize vectorized access and GPU usage:
- `x: np.ndarray[int32]` shape=(capacity,)
- `y: np.ndarray[int32]` shape=(capacity,)
- `z: np.ndarray[int32]` shape=(capacity,)
- `p: np.ndarray[float32]` shape=(capacity,)

Keep membrane potentials and coordinates in separate contiguous arrays to benefit from SIMD and GPU-friendly memory layout.

### Firing Neurons Representation

Use **Roaring Bitmaps** per cortical area to represent sparse sets of firing neuron indices efficiently:

```python
{
  100: roaring_bitmap_of_firing_neuron_ids_in_area_100,
  200: roaring_bitmap_of_firing_neuron_ids_in_area_200,
  ...
}
```

### Data Extraction Process

1. **Iterate over cortical areas with firing neurons:**
   * For each cortical area integer ID, obtain the corresponding roaring bitmap of firing neurons.

2. **Convert cortical area integer ID to 6-character ASCII string ID:**
   * Use a fixed, lossless mapping (e.g., base-36 or custom encoding) to convert int → 6-letter string.
   * This 6-byte string will be included in the data packet header.

3. **Extract neuron data efficiently:**
   * Convert roaring bitmap to a NumPy array of neuron indices.
   * Use these indices to slice the SoA arrays (`x`, `y`, `z`, `p`) **directly** — no loops.
   * This yields four NumPy arrays per cortical area, each holding data only for firing neurons.

### Data Transmission Format (Raw Bytes)

#### Motivation

* Avoid text-based formats like JSON to eliminate serialization overhead.
* Transmit compact, binary data streams for lowest latency and bandwidth use.
* Allow direct memory-mapped reception on the other side without parsing text.

#### Packet Structure per Cortical Area

| Field               | Size (bytes) | Description                                    |
| ------------------- | ------------ | ---------------------------------------------- |
| Cortical Area ID    | 6            | ASCII string representing cortical area        |
| Number of Neurons   | 4            | uint32 number of firing neurons in this packet |
| x-coordinates       | 4 \* N       | int32 array of x coordinates                   |
| y-coordinates       | 4 \* N       | int32 array of y coordinates                   |
| z-coordinates       | 4 \* N       | int32 array of z coordinates                   |
| membrane potentials | 4 \* N       | float32 array of membrane potentials           |

Where N is the number of firing neurons in that cortical area.

#### Implementation Notes

* Use `numpy.ndarray.tobytes()` for zero-copy conversion of arrays to raw bytes.
* Prepend the 6-byte ASCII cortical area ID and 4-byte neuron count as a fixed-length header.
* Concatenate header + raw byte arrays into a single byte buffer per cortical area.
* Send packets sequentially or in parallel over the network.
* On the receiver side:
  * Parse header (6 bytes + 4 bytes).
  * Read exact sizes for `x, y, z, p` arrays.
  * Reconstruct NumPy arrays using `np.frombuffer()` without copying.

---

## Memory Alignment Guidelines

Different backends have specific alignment requirements:

### WGPU
- Most scalar types require 4-byte alignment
- Vectors (vec2/vec3/vec4) require 8/16-byte alignment
- Arrays within storage buffers should start at offsets divisible by their element size
- Use `wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST` for read-write buffers

### SIMD (CPU)
- AVX/AVX2: Prefer 32-byte alignment for optimal performance
- AVX-512: Prefer 64-byte alignment (aligns with cache line size)
- Use `np.zeros(shape, dtype=np.float32, order='C')` in NumPy to ensure C-contiguous arrays
- In Rust, use `#[repr(align(32))]` for AVX-friendly structs

### CUDA/CuPy
- Most operations work with standard alignment, but prefer 16-byte alignment
- Ensure pitched memory for 2D arrays (handled automatically by most APIs)

## Buffer Transfer Strategies

For efficient CPU-GPU transfers:

### Zero-Copy When Possible
- For WGPU: Use `MAPPED_AT_CREATION` for buffers that are initialized once and rarely updated
- For large buffers that change rarely, use `wgpu::BufferUsages::MAP_WRITE` to avoid full transfers

### Staging Buffers for Frequent Updates
For frequently updated data, use a staging buffer pattern:
```python
# Example for WGPU-py
staging_buffer = device.create_buffer_with_data(data=neuron_data, usage=wgpu.BufferUsages.COPY_SRC)
device.queue.write_buffer(destination_buffer, 0, staging_buffer)
```

### Batch Updates
- Collect multiple small updates and perform them in a single operation
- For neuron state updates, prefer updating entire arrays rather than individual neurons
- Use the command encoder pattern for batched GPU operations

## WGSL Shader Design for Neural Simulation

For WGPU compute shaders:

### SoA in Buffers, AoS in Workgroups
- Use SoA layout in storage buffers:
  ```wgsl
  @group(0) @binding(0) var<storage, read_write> membrane_potentials: array<f32>;
  @group(0) @binding(1) var<storage, read_write> thresholds: array<f32>;
  ```
- For workgroup memory, consider AoS for localized processing:
  ```wgsl
  var<workgroup> local_neurons: array<Neuron, 256>;
  ```

### Workgroup Size Optimization
- Use workgroup size of 64-256 for most neural operations
- For operations on neuron groups, align workgroup size with typical neural circuit size
- Example:
  ```wgsl
  @compute @workgroup_size(256)
  fn update_neurons(@builtin(global_invocation_id) id: vec3<u32>) {
      let neuron_id = id.x;
      if (neuron_id >= arrayLength(&membrane_potentials)) { return; }

      // Process neuron state
      membrane_potentials[neuron_id] = membrane_potentials[neuron_id] * decay_factor;
  }
  ```

### Neuron Dynamics Kernels
- Split complex operations into multiple kernels (e.g., separate decay, integration, firing)
- Use atomic operations for fire candidate list updates:
  ```wgsl
  @group(0) @binding(4) var<storage, read_write> fire_bitmap: array<atomic<u32>>;

  fn set_fire_candidate(neuron_id: u32) {
      let word_index = neuron_id / 32u;
      let bit_position = neuron_id % 32u;
      let mask = 1u << bit_position;
      atomicOr(&fire_bitmap[word_index], mask);
  }
  ```

## Atomics and Synchronization

For parallel updates in neural simulation:

### Fire Candidate List Updates
- Use atomic operations to update fire candidate bitmaps/lists
- In Rust/CPU: Use AtomicU32 for bitmap operations
- In WGPU: Use atomic storage buffers and operations

### Synapse Updates
For learning rules that modify synaptic weights:
- **Option 1**: Use atomic operations (may be slower but safer)
- **Option 2**: Partition synapses to avoid conflicts
- **Option 3**: Use double-buffering for weight updates

### Barrier Synchronization
Between simulation phases (e.g., after all neurons processed, before synapse updates):
- In WGPU:
  ```wgsl
  workgroupBarrier(); // For synchronization within a workgroup
  ```
- In Rust/CPU parallel code:
  ```rust
  // Using rayon or similar
  let (phase1, phase2) = rayon::join(
      || process_neurons(neurons),
      || prepare_synapse_updates(synapses)
  );
  ```

### Phase Separation
Neural simulation is naturally phased - use this for synchronization:
1. Update membrane potentials
2. Barrier
3. Determine firing neurons
4. Barrier
5. Propagate signals
6. Barrier
7. Apply plasticity rules

---

## Current Implementation Status

The following table tracks identified deviations between the architecture specification and the current implementation:

| ID | Component | Current Implementation | Specification | Impact | Status |
|----|-----------|------------------------|---------------|--------|--------|
| 1 | Coordinate Storage | Separate arrays for x, y, z coordinates | Separate arrays for x, y, z coordinates | Optimal SIMD/GPU efficiency | ✅ Completed |
| 2 | Coordinate Data Type | Both implementations use u32/uint32 | int32/uint32 for all coordinates | Full compatibility | ✅ Completed |
| 3 | Binary Format | Minimal header with ID and neuron count | Minimal header with ID and neuron count | Optimal packet size | ✅ Completed |
| 4 | Data Extraction | Vectorized operations with NumPy | Vectorized operations with NumPy | High performance | ✅ Completed |
| 5 | Data Conversion | Zero-copy approaches with `tobytes()` and `frombuffer()` | Zero-copy operations | No memory copying overhead | ✅ Completed |
| 6 | JSON Fallback | Binary format only | Binary format only | Consistent performance | ✅ Completed |
| 7 | Packet Structure | Fixed structure as defined | Fixed structure as defined | Compatibility ensured | ✅ Completed |

---

## Recommendations for Future Development

### Full Array Migration
- Ensure all neuron and synapse properties are stored in contiguous arrays (no dicts/lists for critical paths).

### SoA over AoS
- Prefer Structure of Arrays (SoA) for all per-synapse and per-neuron data for best GPU memory access.

### PyTorch/CuPy Backend
- Abstract array backend to allow easy switching between NumPy, PyTorch, and CuPy for CPU/GPU.

### Remove Legacy Dicts
- Refactor BDU and any remaining NPU code to eliminate dict-based storage in favor of arrays.

### Batch Operations
- All updates, propagation, and thresholding should be performed in batch using vectorized ops.

### Testing
- Add/expand tests to ensure array-based and fallback implementations are always in sync.

### Rust Migration Readiness
- Design data structures with Rust's ownership model in mind
- Use fixed-size arrays where possible
- Minimize dynamic allocations
- Prefer explicit lifetimes over garbage collection

## Related Documentation

- [System Overview](arch-system-overview.md)
- [GPU Architecture](arch-gpu.md)
- [Rust/RTOS Migration Guide](arch-rust-rtos-migration.md)
- [Performance Optimization](arch-gpu-optimization.md)
