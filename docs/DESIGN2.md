# Design Document: Brain Simulation Architecture - Revised

## 1. Overview & Goals

This document outlines the proposed architecture and design considerations for a modular brain simulation application. The primary goals are:

- **Accurate Simulation:** Model a brain composed of distinct cortical areas, voxels within areas, and individual neurons with specific properties and connections.
- **Dynamic Behavior:** Support the creation, update, and deletion of cortical areas, neurons, and synapses during the simulation's lifetime, with dynamic memory growth.
- **High Performance:** Achieve efficient execution by leveraging both CPU SIMD capabilities and GPU acceleration, with optimizations for sparse activation patterns.
- **Portability & Flexibility:** Avoid vendor lock-in (specifically NVIDIA/CUDA) by supporting cross-platform GPU execution, with WebGPU as a primary target, compatible even with mobile/embedded GPUs like Snapdragon/Adreno (given appropriate drivers).
- **Maintainability:** Employ clear data structures and strategies for managing dynamic data.

## 2. Core Architecture Principles

### 2.1. Hierarchy

The simulation represents the brain hierarchically:

- **Cortical Areas:** Distinct regions with unique IDs and properties (e.g., dimensions).
- **Voxels:** Each area is logically divided into a grid of voxels. Voxel coordinates are relative to the parent area.
- **Neurons:** Voxels can contain zero, one, or many neurons. Each neuron belongs to a single area and voxel (implicitly via coordinates) and has a unique ID and multiple biophysical properties.
- **Synapses:** Connections between neurons that transmit signals, with properties like weight and delay.

### 2.2. Data Layout: Structure of Arrays (SoA)

To optimize for both CPU SIMD and GPU parallelism, a **Structure of Arrays (SoA)** data layout will be used for neuron and synapse data.

- **Rationale:**
  - **SIMD:** CPUs operate efficiently on contiguous data. Storing all membrane potentials together allows loading multiple values into wide SIMD registers (SSE, AVX, NEON) for parallel computation.
  - **GPU:** GPUs achieve high memory bandwidth via coalesced memory access. Threads processing adjacent neurons access contiguous memory locations when properties are stored in separate, parallel arrays, significantly improving performance compared to accessing scattered data in an Array of Structures (AoS) layout.

- **Implementation:** All neurons across all areas will be stored in a flattened structure represented by multiple parallel, contiguous arrays (e.g., NumPy arrays on the CPU, mirrored as `wgpu` buffers on the GPU). One array will exist for each neuron property.

Additionally, a separation between **dynamic** (frequently changing) and **static** (rarely changing) neuron properties will be maintained to optimize memory updates, especially for GPU transfers.

### 2.3. Memory Management Strategy

The simulation will use a sophisticated memory management system with several optimizations:

1. **Slotted Memory Allocation System:**
   - Large preallocated arrays with fixed-size slots
   - An efficient slot manager keeps track of occupied and free slots
   - Atomic batch operations for creating or removing multiple neurons or synapses

2. **Dynamic Growth Strategy:**
   - Initial capacity will be generous but not excessive
   - When capacity nears a threshold (e.g., 80%), a background compaction task prepares for expansion
   - Arrays are grown by a geometric factor (e.g., 1.5x) to amortize resizing costs
   - Growth operations never occur during critical simulation steps

3. **Sparse Activation Tracking:**
   - Maintain dedicated arrays of indices for active neurons/synapses
   - Only process active elements during simulation steps
   - Periodically rebuild the active indices array when activation patterns change significantly

### 2.4. Effective Activity Check

Component activity during simulation steps must check hierarchical conditions:

- A neuron's processing requires:
  - The neuron's own `is_active` flag must be True
  - The parent area's `is_active` flag must also be True

- A synapse's processing requires:
  - The synapse's own `is_active` flag must be True
  - Both pre- and post-synaptic neurons must be active

All conditions must be verified before processing during CPU SIMD operations or GPU/WebGPU kernel execution.

## 3. Data Structures

### 3.1. Cortical Area Management

```python
# --- Area Management ---
max_areas = 1024  # Maximum number of cortical areas

# Area properties
area_indices = np.arange(max_areas, dtype=np.int32)
area_is_active = np.zeros(max_areas, dtype=np.bool_)
area_dimensions = np.zeros((max_areas, 3), dtype=np.int32)  # [width, height, depth]
area_names = np.array([''] * max_areas, dtype=object)  # For debugging/visualization

# Area metadata
area_neuron_counts = np.zeros(max_areas, dtype=np.int32)
area_voxel_counts = np.zeros(max_areas, dtype=np.int32)

# Memory management for areas
next_area_id = 0
free_area_slots = []
```

### 3.2. Neuron SoA Arrays

```python
# --- Configuration ---
initial_max_neurons = 1_000_000  # Initial capacity

# --- Neuron Metadata & Status (SoA) ---
neuron_indices = np.arange(initial_max_neurons, dtype=np.int32)     # Slot index
area_ids = np.zeros(initial_max_neurons, dtype=np.uint32)           # Parent area ID
voxel_x = np.zeros(initial_max_neurons, dtype=np.int16)             # Relative X coord
voxel_y = np.zeros(initial_max_neurons, dtype=np.int16)             # Relative Y coord
voxel_z = np.zeros(initial_max_neurons, dtype=np.int16)             # Relative Z coord
is_active = np.zeros(initial_max_neurons, dtype=np.bool_)           # Neuron soft delete flag
neuron_type = np.zeros(initial_max_neurons, dtype=np.uint8)         # E.g., excitatory/inhibitory

# --- Neuron Dynamic Properties (SoA) ---
membrane_potentials = np.zeros(initial_max_neurons, dtype=np.float32)
refractory_timers = np.zeros(initial_max_neurons, dtype=np.int16)
adaptation_currents = np.zeros(initial_max_neurons, dtype=np.float32)
spike_buffer = np.zeros(initial_max_neurons, dtype=np.bool_)        # Records spikes for current step

# --- Neuron Static Properties (SoA) ---
firing_thresholds = np.zeros(initial_max_neurons, dtype=np.float32)
resting_potentials = np.zeros(initial_max_neurons, dtype=np.float32)
membrane_time_constants = np.zeros(initial_max_neurons, dtype=np.float32)
refractory_periods = np.zeros(initial_max_neurons, dtype=np.int16)
# ... other static property arrays ...

# --- Active Neuron Tracking ---
max_active_neurons = initial_max_neurons  # Could be smaller depending on expected sparsity
active_neuron_indices = np.zeros(max_active_neurons, dtype=np.int32)
active_neuron_count = 0

# --- Memory Management ---
class NeuronSlotManager:
    def __init__(self, capacity):
        self.capacity = capacity
        self.next_available_index = 0
        self.free_slots = []
        
    def allocate_slot(self):
        """Allocate a single slot."""
        if self.free_slots:
            return self.free_slots.pop()
        else:
            if self.next_available_index >= self.capacity:
                raise MemoryError("No available neuron slots")
            slot = self.next_available_index
            self.next_available_index += 1
            return slot
            
    def allocate_batch(self, count):
        """Allocate multiple slots at once."""
        # Implementation that optimizes allocation
        # ...
        
    def free_slot(self, index):
        """Free a single slot."""
        self.free_slots.append(index)
        
    def free_batch(self, indices):
        """Free multiple slots at once."""
        self.free_slots.extend(indices)
        
    def get_usage_stats(self):
        """Return stats about memory usage."""
        used = self.next_available_index - len(self.free_slots)
        return {
            'capacity': self.capacity,
            'used': used,
            'free': self.capacity - used,
            'fragmentation': len(self.free_slots) / self.capacity if self.capacity > 0 else 0
        }

# Initialize slot manager
neuron_slot_manager = NeuronSlotManager(initial_max_neurons)
```

### 3.3. Synapse SoA Arrays

```python
# --- Synapse Configuration ---
initial_max_synapses = 100_000_000  # Initial capacity (100x neurons)

# --- Synapse Core Data (SoA) ---
synapse_indices = np.arange(initial_max_synapses, dtype=np.int32)
pre_neuron_indices = np.zeros(initial_max_synapses, dtype=np.int32)   # Source neuron
post_neuron_indices = np.zeros(initial_max_synapses, dtype=np.int32)  # Target neuron
is_active = np.zeros(initial_max_synapses, dtype=np.bool_)

# --- Synapse Dynamic Properties ---
weights = np.zeros(initial_max_synapses, dtype=np.float32)           # Current weight
delay_timers = np.zeros(initial_max_synapses, dtype=np.int16)        # Current delay (countdown)

# --- Synapse Static Properties ---
base_weights = np.zeros(initial_max_synapses, dtype=np.float32)      # Initial/baseline weight
base_delays = np.zeros(initial_max_synapses, dtype=np.int16)         # Delay in timesteps
synapse_types = np.zeros(initial_max_synapses, dtype=np.uint8)       # E.g., AMPA, NMDA, etc.
plasticity_rules = np.zeros(initial_max_synapses, dtype=np.uint8)    # Which learning rule applies

# --- Active Synapse Tracking ---
max_active_synapses = initial_max_synapses  # Could be smaller depending on expected sparsity
active_synapse_indices = np.zeros(max_active_synapses, dtype=np.int32)
active_synapse_count = 0

# --- Synapse Memory Management ---
synapse_slot_manager = SlotManager(initial_max_synapses)  # Similar implementation as for neurons
```

### 3.4. Connectivity Acceleration Structures

```python
# --- Efficient Synapse Lookup ---

# For each neuron, store start index and count in the forward/backward connectivity arrays
outgoing_synapse_start_indices = np.zeros(initial_max_neurons, dtype=np.int32)
outgoing_synapse_counts = np.zeros(initial_max_neurons, dtype=np.int32)
incoming_synapse_start_indices = np.zeros(initial_max_neurons, dtype=np.int32)
incoming_synapse_counts = np.zeros(initial_max_neurons, dtype=np.int32)

# Ordered arrays of synapse indices for efficient lookup
# These are maintained in sync with the main synapse arrays
outgoing_synapses = np.zeros(initial_max_synapses, dtype=np.int32)  # Synapses by pre-neuron
incoming_synapses = np.zeros(initial_max_synapses, dtype=np.int32)  # Synapses by post-neuron
```

## 4. Core Operations

### 4.1. Dynamic Component Creation/Deletion

```python
def create_neurons_batch(area_id, voxel_positions, properties, count):
    """
    Create multiple neurons at once with specified properties.
    
    Args:
        area_id: The cortical area ID
        voxel_positions: Array of (x,y,z) positions, shape (count, 3)
        properties: Dict of property arrays, each of length count
        count: Number of neurons to create
    
    Returns:
        Array of neuron indices that were created
    """
    # Allocate slots and update basic properties
    indices = neuron_slot_manager.allocate_batch(count)
    for i, idx in enumerate(indices):
        area_ids[idx] = area_id
        voxel_x[idx], voxel_y[idx], voxel_z[idx] = voxel_positions[i]
        is_active[idx] = True
        
    # Set all specified properties
    for prop_name, values in properties.items():
        prop_array = globals()[prop_name]  # Get the corresponding property array
        for i, idx in enumerate(indices):
            prop_array[idx] = values[i]
            
    # Update area metadata
    area_neuron_counts[area_id] += count
    
    # Update active neuron tracking if the area is active
    if area_is_active[area_id]:
        # Add to active neurons if there's space
        # This is a simplified version; real implementation would handle overflow
        if active_neuron_count + count <= max_active_neurons:
            active_neuron_indices[active_neuron_count:active_neuron_count+count] = indices
            active_neuron_count += count
    
    return indices
```

### 4.2. Simulation Step Logic and Temporal Activation Tracking

```python
# Basic simulation step
def simulation_step():
    """Execute one step of the brain simulation."""
    global timestep
    
    # 1. Process active neurons (CPU version)
    for i in range(active_neuron_count):
        neuron_idx = active_neuron_indices[i]
        
        # Skip if area is inactive
        area_id = area_ids[neuron_idx]
        if not area_is_active[area_id]:
            continue
            
        # Neuron dynamics
        if refractory_timers[neuron_idx] > 0:
            refractory_timers[neuron_idx] -= 1
            continue
            
        # Decay membrane potential
        tau = membrane_time_constants[neuron_idx]
        rest = resting_potentials[neuron_idx]
        membrane_potentials[neuron_idx] += (rest - membrane_potentials[neuron_idx]) / tau
        
        # Check for spike
        threshold = firing_thresholds[neuron_idx]
        if membrane_potentials[neuron_idx] >= threshold:
            spike_buffer[neuron_idx] = True
            membrane_potentials[neuron_idx] = rest
            refractory_timers[neuron_idx] = refractory_periods[neuron_idx]
    
    # 2. Process synapses with active pre-synaptic neurons
    process_active_synapses()
    
    # 3. Apply synaptic inputs and reset spike buffer
    apply_synaptic_inputs()
    
    # 4. Update activation history using Roaring Bitmaps
    update_activation_history(timestep)
    
    # 5. Reset spike buffer for next step
    spike_buffer.fill(False)
    
    # 6. Periodically update active neuron/synapse lists if activation patterns have changed
    if timestep % UPDATE_ACTIVE_LISTS_INTERVAL == 0:
        rebuild_active_lists()
    
    # Increment timestep
    timestep += 1
```

#### 4.2.1. Roaring Bitmap for Temporal Activation Tracking

The simulation must efficiently track neuron activations across multiple timesteps to support temporal pattern analysis, plasticity mechanisms, and visualization. For this critical functionality, we've selected **Roaring Bitmaps** as our primary data structure.

**Technology Selection Rationale:**

Roaring Bitmaps represent a hybrid data structure that combines the advantages of various set representations, making them ideal for tracking sparse activation patterns in neural simulations:

1. **Compact Memory Representation:** Neural activations are typically very sparse (0.1-5% active), making traditional dense arrays inefficient. Roaring Bitmaps use a two-level indexing scheme that dynamically selects the optimal internal representation based on data density:
   - Sparse ranges use sorted arrays of 16-bit integers
   - Dense ranges use highly efficient bitmap containers
   - Very dense ranges use run-length encoding

2. **Operation Performance:** Set operations (union, intersection, difference) are computationally intensive but critical for analyzing activation patterns. Roaring Bitmaps implement these operations with container-aware algorithms that can be:
   - Up to 100x faster than traditional bit arrays
   - Up to 5x faster than specialized tree-based structures
   - SIMD-accelerated on supported hardware

3. **Time Window Analysis:** Maintaining a rolling window of activations requires efficient data rotation, clearing, and queries. Roaring Bitmaps support these operations with minimal overhead.

4. **GPU Compatibility:** While our primary implementation uses CPU-based PyRoaring for clarity, there are compatible GPU-based bitmap libraries (RAPIDS cuDF's GPU bitmaps) that follow similar principles for future integration.

**Implementation:**

```python
import pyroaring  # For CPU implementation
# For GPU, a specialized implementation or wrapper would be needed

# Create a rolling window of activations
ACTIVATION_WINDOW_SIZE = 20  # Number of timesteps to maintain in history
activation_history = [pyroaring.BitMap() for _ in range(ACTIVATION_WINDOW_SIZE)]
current_window_index = 0

def update_activation_history(current_timestep):
    """Update the rolling window of activations with neurons active in the current timestep."""
    global current_window_index
    
    # Clear the oldest bitmap and reuse it for current timestep
    current_index = current_timestep % ACTIVATION_WINDOW_SIZE
    activation_history[current_index].clear()
    
    # Add all currently spiking neurons to the bitmap
    for i in range(active_neuron_count):
        neuron_idx = active_neuron_indices[i]
        if spike_buffer[neuron_idx]:
            activation_history[current_index].add(neuron_idx)
    
    current_window_index = current_index

def get_neurons_active_in_last_n_steps(n_steps):
    """
    Get neurons that were active in any of the last n timesteps.
    Returns a Roaring Bitmap containing their indices.
    """
    if n_steps > ACTIVATION_WINDOW_SIZE:
        n_steps = ACTIVATION_WINDOW_SIZE
    
    result = pyroaring.BitMap()
    current_index = current_window_index
    
    for i in range(n_steps):
        step_index = (current_index - i) % ACTIVATION_WINDOW_SIZE
        result |= activation_history[step_index]  # Fast union operation
    
    return result

def get_consistently_active_neurons(n_steps):
    """
    Get neurons that were active in ALL of the last n timesteps.
    Returns a Roaring Bitmap containing their indices.
    """
    if n_steps > ACTIVATION_WINDOW_SIZE:
        n_steps = ACTIVATION_WINDOW_SIZE
    
    current_index = current_window_index
    step_index = current_index % ACTIVATION_WINDOW_SIZE
    result = activation_history[step_index].copy()
    
    for i in range(1, n_steps):
        step_index = (current_index - i) % ACTIVATION_WINDOW_SIZE
        result &= activation_history[step_index]  # Fast intersection operation
    
    return result

def get_activation_delta(start_time, end_time):
    """
    Get neurons that became active between start and end times.
    Returns a Roaring Bitmap containing their indices.
    """
    if end_time - start_time > ACTIVATION_WINDOW_SIZE:
        raise ValueError("Time difference exceeds history window size")
    
    start_index = start_time % ACTIVATION_WINDOW_SIZE
    end_index = end_time % ACTIVATION_WINDOW_SIZE
    
    # Neurons active at end but not at start (newly activated)
    return activation_history[end_index] - activation_history[start_index]  # Fast difference

def get_activation_xor(time1, time2):
    """
    Get neurons that were active at either time1 or time2, but not both.
    Useful for detecting changes in activation patterns.
    """
    idx1 = time1 % ACTIVATION_WINDOW_SIZE
    idx2 = time2 % ACTIVATION_WINDOW_SIZE
    
    return activation_history[idx1] ^ activation_history[idx2]  # Fast XOR operation

def count_active_neurons(bitmap):
    """
    Efficiently count the number of active neurons in a bitmap.
    Much faster than iterating through all neurons.
    """
    return len(bitmap)  # Optimized cardinality operation in Roaring
```

**Performance Implications:**

The Roaring Bitmap implementation provides several key advantages for the simulation:

1. **Temporal Pattern Analysis:** The ability to efficiently query neurons active across various time windows enables sophisticated analysis of activity propagation, synchronization patterns, and bursting behaviors. This is crucial for validating the biological plausibility of the simulation.

2. **Memory Efficiency:** For a simulation with 1 million neurons and 1% activation rate, traditional bit arrays would require 125KB per timestep regardless of activity. Roaring Bitmaps typically compress this by 10-100x, requiring only 1-10KB for the same data, allowing much longer temporal windows to be maintained.

3. **Computational Efficiency:** Set operations that would normally require O(n) time regardless of activation density are optimized to O(k) time, where k is the compressed size. For sparse activations, this translates to orders of magnitude speedup in temporal analysis.

4. **Plasticity Support:** Synaptic plasticity mechanisms like STDP require efficient tracking of pre-post activation timing relationships. The fast time-window queries enable these biologically critical mechanisms to be implemented with minimal performance overhead.

**Integration with Simulation Steps:**

The activation history tracking is fully integrated into the main simulation loop, with minimal overhead added to each step. The simulation maintains a fixed-size rolling window that automatically discards older data as new activations are recorded.

### 4.3. GPU Implementation Considerations

GPU acceleration is critical for achieving high-performance neural simulation. The WebGPU standard provides a cross-platform solution that avoids vendor lock-in while delivering excellent performance.

**WebGPU for Neural Simulation:**

WebGPU offers several advantages over vendor-specific alternatives:

1. **Cross-platform Compatibility:** Unlike CUDA (NVIDIA-only) or Metal (Apple-only), WebGPU code can run on multiple hardware platforms with appropriate drivers, including mobile GPUs.

2. **Modern Pipeline Design:** WebGPU's compute pipeline design aligns well with the highly parallel nature of neural simulation, with explicit control over shader stages and memory barriers.

3. **Performance Tradeoffs:** While raw performance may be slightly lower than vendor-optimized solutions in some cases, the portability benefits and broader hardware support outweigh this limitation for most simulation scenarios.

The following WebGPU shader illustrates how neuron dynamics are implemented:

```python
# WebGPU shader for neuron update (pseudocode)
"""
@group(0) @binding(0) var<storage, read> area_is_active: array<u32>;
@group(0) @binding(1) var<storage, read> area_ids: array<u32>;
@group(0) @binding(2) var<storage, read_write> membrane_potentials: array<f32>;
@group(0) @binding(3) var<storage, read> resting_potentials: array<f32>;
@group(0) @binding(4) var<storage, read> membrane_time_constants: array<f32>;
@group(0) @binding(5) var<storage, read> firing_thresholds: array<f32>;
@group(0) @binding(6) var<storage, read_write> spike_buffer: array<u32>;
@group(0) @binding(7) var<storage, read_write> refractory_timers: array<u32>;
@group(0) @binding(8) var<storage, read> refractory_periods: array<u32>;
@group(0) @binding(9) var<storage, read> active_neuron_indices: array<u32>;
@group(0) @binding(10) var<uniform> active_neuron_count: u32;

@compute @workgroup_size(256)
fn update_neurons(@builtin(global_invocation_id) global_id: vec3<u32>) {
    // Only process valid active neurons
    let thread_id = global_id.x;
    if (thread_id >= active_neuron_count) {
        return;
    }
    
    // Get actual neuron index from active list
    let neuron_idx = active_neuron_indices[thread_id];
    
    // Check if parent area is active
    let area_id = area_ids[neuron_idx];
    if (area_is_active[area_id] == 0) {
        return;
    }
    
    // Refractory period check
    if (refractory_timers[neuron_idx] > 0) {
        refractory_timers[neuron_idx] -= 1;
        return;
    }
    
    // Update membrane potential
    let tau = membrane_time_constants[neuron_idx];
    let rest = resting_potentials[neuron_idx];
    membrane_potentials[neuron_idx] += (rest - membrane_potentials[neuron_idx]) / tau;
    
    // Check for spike
    let threshold = firing_thresholds[neuron_idx];
    if (membrane_potentials[neuron_idx] >= threshold) {
        spike_buffer[neuron_idx] = 1;
        membrane_potentials[neuron_idx] = rest;
        refractory_timers[neuron_idx] = refractory_periods[neuron_idx];
    }
}
"""
```

**Note on Integration with Roaring Bitmaps:**

While the GPU component performs the core neural dynamics calculations, Roaring Bitmap operations for temporal activation tracking primarily run on the CPU in the current design. This separation of concerns allows each processor to focus on its strengths:

1. **GPU:** Massive parallel computation of neuron dynamics and synaptic transmission
2. **CPU:** Complex data structure manipulation and temporal pattern analysis

Future iterations may explore GPU-accelerated bitmap libraries to further optimize this pattern where appropriate.

## 5. Performance Optimizations

### 5.1. Sparse Activation & Updates

The simulation leverages the biological property that neural activity is typically sparse (1-10% of neurons active):

1. **Active Lists Maintenance:**
   - Active neurons and synapses are tracked in dedicated index arrays
   - Only these active components are processed during simulation steps
   - Periodic rebuilding of active lists when activation patterns change

2. **Two-phase Processing:**
   - First phase: Neuron dynamics and spike generation
   - Second phase: Only process synapses from neurons that spiked

3. **Early Termination:**
   - Skip inactive areas entirely
   - Skip neurons in refractory period

### 5.2. Memory Access Patterns

1. **GPU Memory Coalescing:**
   - Structure of Arrays layout ensures threads in a warp/wavefront access adjacent memory
   - Active indices are sorted to improve memory locality
   
2. **CPU Cache Optimization:**
   - Properties are processed in separate passes to maximize cache line utilization
   - SIMD-friendly data layout supports vectorized operations

3. **Transfer Optimization:**
   - Only dynamic properties are transferred between CPU-GPU each step
   - Static properties are transferred once or infrequently

### 5.3. Workload Distribution

1. **Dynamic Kernel Selection:**
   - Different compute kernels for different activity densities
   - Sparse kernels for <1% activity
   - Dense kernels for >10% activity
   
2. **Spatial Partitioning:**
   - Group neurons by cortical area for improved locality
   - Process areas with similar activity patterns together

## 6. Future Extensions

1. **Multi-scale Integration:**
   - Support for different temporal resolutions in different areas
   - Integration with lower-fidelity whole-brain models

2. **Advanced Plasticity:**
   - STDP (Spike-Timing-Dependent Plasticity)
   - Homeostatic scaling
   - Structural plasticity (dynamic synapse creation/pruning)

3. **Visualization & Analysis:**
   - Real-time activity visualization
   - Network analysis tools
   - Integration with neuroimaging data

4. **Biophysical Detail:**
   - Compartmental neuron models (dendrites, soma, axon)
   - Ion channel dynamics
   - Glial cell interactions