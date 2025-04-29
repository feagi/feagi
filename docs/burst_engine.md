# Burst Engine Implementation

The Burst Engine is a core component of the Neural Processing Unit (NPU) in FEAGI that handles all activities related to neuron firing, including updating membrane potentials, managing fire candidate lists, and handling cortical stimulation.

## Fire Candidate List (FCL) Queue

The Fire Candidate List contains the neurons that should fire simultaneously across the entire brain. The act of firing all neurons within the FCL is called a "burst" of neuron firing. The FCL Queue maintains a history of recent FCLs across multiple consecutive bursts, enabling temporal pattern analysis and visualization.

### Roaring Bitmaps Technology

For efficient FCL implementation, FEAGI uses **Roaring Bitmaps** as the primary data structure to represent neuron activation patterns. This technology offers several advantages:

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

### Basic FCL Implementation

A simple FCL implementation using Roaring Bitmaps looks like this:

```python
import numpy as np
import pyroaring  # For CPU implementation
# For GPU, a specialized implementation or wrapper would be needed

# Create a rolling window of fire candidate lists
ACTIVATION_WINDOW_SIZE = 20  # Number of timesteps to maintain in history
fcl_history = [pyroaring.BitMap() for _ in range(ACTIVATION_WINDOW_SIZE)]
current_window_index = 0

def update_fcl_history(current_timestep, fire_candidate_list):
    """Update the rolling window of FCLs with neurons that fired in the current timestep."""
    global current_window_index
    
    # Clear the oldest bitmap and reuse it for current timestep
    current_index = current_timestep % ACTIVATION_WINDOW_SIZE
    fcl_history[current_index].clear()
    
    # Add all neurons in the current FCL to the bitmap
    for neuron_idx in fire_candidate_list:
        fcl_history[current_index].add(neuron_idx)
    
    current_window_index = current_index

def get_neurons_fired_in_last_n_steps(n_steps):
    """
    Get neurons that fired in any of the last n timesteps.
    Returns a Roaring Bitmap containing their indices.
    """
    if n_steps > ACTIVATION_WINDOW_SIZE:
        n_steps = ACTIVATION_WINDOW_SIZE
    
    result = pyroaring.BitMap()
    current_index = current_window_index
    
    for i in range(n_steps):
        step_index = (current_index - i) % ACTIVATION_WINDOW_SIZE
        result |= fcl_history[step_index]  # Fast union operation
    
    return result

def get_consistently_active_neurons(n_steps):
    """
    Get neurons that fired in ALL of the last n timesteps.
    Returns a Roaring Bitmap containing their indices.
    """
    if n_steps > ACTIVATION_WINDOW_SIZE:
        n_steps = ACTIVATION_WINDOW_SIZE
    
    current_index = current_window_index
    step_index = current_index % ACTIVATION_WINDOW_SIZE
    result = fcl_history[step_index].copy()
    
    for i in range(1, n_steps):
        step_index = (current_index - i) % ACTIVATION_WINDOW_SIZE
        result &= fcl_history[step_index]  # Fast intersection operation
    
    return result

def get_fcl_delta(start_time, end_time):
    """
    Get neurons that became active between start and end times.
    Returns a Roaring Bitmap containing their indices.
    """
    if end_time - start_time > ACTIVATION_WINDOW_SIZE:
        raise ValueError("Time difference exceeds history window size")
    
    start_index = start_time % ACTIVATION_WINDOW_SIZE
    end_index = end_time % ACTIVATION_WINDOW_SIZE
    
    # Neurons active at end but not at start (newly activated)
    return fcl_history[end_index] - fcl_history[start_index]  # Fast difference

def get_fcl_xor(time1, time2):
    """
    Get neurons that fired at either time1 or time2, but not both.
    Useful for detecting changes in firing patterns.
    """
    idx1 = time1 % ACTIVATION_WINDOW_SIZE
    idx2 = time2 % ACTIVATION_WINDOW_SIZE
    
    return fcl_history[idx1] ^ fcl_history[idx2]  # Fast XOR operation

def count_firing_neurons(bitmap):
    """
    Efficiently count the number of firing neurons in a bitmap.
    Much faster than iterating through all neurons.
    """
    return len(bitmap)  # Optimized cardinality operation in Roaring
```

## Hierarchical Fire Candidate List Implementation

To efficiently track both neurons and their cortical area associations, FEAGI implements a hierarchical FCL data structure. This approach preserves the area information without requiring expensive lookups during queries.

The hierarchical FCL maintains:
1. A global bitmap of all firing neurons
2. Area-specific bitmaps organized in a dictionary (area_id → bitmap)
3. Temporal history for both global and area-specific activations

This design enables:
- Fast global queries across all neurons
- Efficient area-based filtering without table lookups
- Temporal pattern analysis within specific brain regions

```python
from collections import defaultdict

class HierarchicalFCL:
    def __init__(self, window_size=20):
        """Initialize a hierarchical FCL that tracks neurons with cortical areas."""
        self.window_size = window_size
        # Main FCL history - all neurons regardless of area
        self.global_fcl_history = [pyroaring.BitMap() for _ in range(window_size)]
        # Area-specific history - area_id → list of bitmaps
        self.area_fcl_history = defaultdict(lambda: [pyroaring.BitMap() for _ in range(window_size)])
        self.current_window_index = 0
        
    def update_fcl(self, current_timestep, neurons_by_area):
        """Update FCL with neurons firing in current timestep, preserving area info."""
        current_index = current_timestep % self.window_size
        
        # Clear the oldest bitmaps
        self.global_fcl_history[current_index].clear()
        
        # Update area-specific FCLs
        for area_id, neuron_ids in neurons_by_area.items():
            # Convert to bitmap if needed
            area_bitmap = pyroaring.BitMap(neuron_ids) if not isinstance(neuron_ids, pyroaring.BitMap) else neuron_ids
            
            # Update area-specific bitmap
            if area_id not in self.area_fcl_history:
                self.area_fcl_history[area_id] = [pyroaring.BitMap() for _ in range(self.window_size)]
            self.area_fcl_history[area_id][current_index].clear()
            self.area_fcl_history[area_id][current_index] |= area_bitmap
            
            # Update global bitmap
            self.global_fcl_history[current_index] |= area_bitmap
        
        self.current_window_index = current_index
    
    def get_global_fcl(self, timestep=None):
        """Get complete FCL for a specific timestep."""
        index = self.current_window_index if timestep is None else timestep % self.window_size
        return self.global_fcl_history[index].copy()
    
    def get_area_fcl(self, area_id, timestep=None):
        """Get FCL for a specific cortical area."""
        index = self.current_window_index if timestep is None else timestep % self.window_size
        if area_id not in self.area_fcl_history:
            return pyroaring.BitMap()
        return self.area_fcl_history[area_id][index].copy()
    
    def get_fcl_by_area(self, timestep=None):
        """Return dictionary mapping each cortical area to its firing neurons."""
        index = self.current_window_index if timestep is None else timestep % self.window_size
        result = {}
        for area_id, fcl_history in self.area_fcl_history.items():
            if not fcl_history[index].is_empty():
                result[area_id] = fcl_history[index].copy()
        return result
    
    def get_neurons_fired_in_last_n_steps(self, n_steps, area_ids=None):
        """Get neurons that fired in any of the last n timesteps, optionally filtered by area."""
        if n_steps > self.window_size:
            n_steps = self.window_size
        
        # Use global or area-filtered approach
        if area_ids is None:
            result = pyroaring.BitMap()
            for i in range(n_steps):
                step_index = (self.current_window_index - i) % self.window_size
                result |= self.global_fcl_history[step_index]
        else:
            result = pyroaring.BitMap()
            for area_id in area_ids:
                if area_id in self.area_fcl_history:
                    for i in range(n_steps):
                        step_index = (self.current_window_index - i) % self.window_size
                        result |= self.area_fcl_history[area_id][step_index]
        
        return result
        
    def get_consistently_active_neurons(self, n_steps, area_ids=None):
        """Get neurons that fired in ALL of the last n timesteps."""
        if n_steps > self.window_size:
            n_steps = self.window_size
        
        if n_steps <= 0:
            return pyroaring.BitMap()
        
        # Initialize result with neurons from the first relevant timestep
        first_step_index = (self.current_window_index) % self.window_size
        
        if area_ids is None:
            # Start with all neurons from first timestep
            result = self.global_fcl_history[first_step_index].copy()
            
            # Intersect with each subsequent timestep
            for i in range(1, n_steps):
                step_index = (self.current_window_index - i) % self.window_size
                result &= self.global_fcl_history[step_index]
        else:
            # Filter by specified areas
            result = pyroaring.BitMap()
            filtered_neurons = self.get_neurons_by_areas(area_ids, timestep=first_step_index)
            result |= filtered_neurons
            
            # Intersect with each subsequent timestep
            for i in range(1, n_steps):
                step_index = (self.current_window_index - i) % self.window_size
                filtered_step = self.get_neurons_by_areas(area_ids, timestep=step_index)
                result &= filtered_step
        
        return result
```

### Benefits of the Hierarchical FCL

1. **Optimized Queries:** Both global and area-specific queries benefit from the bitmap's efficient operations:
   - Area-filtered temporal analyses (find neurons in area X that fired in last N steps)
   - Cross-area pattern detection (areas with synchronous firing)

2. **Efficient Memory Use:** Only stores bitmaps for active cortical areas, avoiding allocation for silent brain regions

3. **Cortical Analysis:** Enables investigation of information flow between brain regions by monitoring activation patterns across specific area sets

4. **Visualization Support:** Provides direct mapping of activities to brain regions for more meaningful visualizations

The original bitmap-based FCL implementation still shows its advantages for global operations, while this hierarchical extension adds area-based capabilities with minimal computational overhead.

## FCL Manager

At the heart of the burst engine lies the FCL manager that is responsible for:
- Reading FCL content
- Initiating the firing of neurons
- Providing a copy of the current FCL content for transmission to the peripheral nervous system (PNS) and data visualization

The FCL Manager interfaces with the Hierarchical FCL implementation to manage the flow of neuron activations throughout the simulation.

## FCL Sampler

The FCL Sampler is responsible for:
- Invoking the FCL manager at a configurable frequency
- Generating motor commands based on neural activity
- Providing data for brain visualization
- Controlling sampling rate to balance performance and visual fidelity

### Visualization Sampling

To maintain visualization performance for high-frequency simulations, the FCL Sampler supports two sampling modes:

- **Ratio-based sampling** (0.0-1.0): Randomly selects a percentage of FCLs to visualize
  - Example: 0.1 means visualize ~10% of FCLs
  
- **Frequency-based sampling** (>1.0): Caps visualization at a maximum frequency (Hz)
  - Example: 30 means visualize at most 30 FCLs per second

## Neuron Firing Dynamics

### Neuron Models

FEAGI supports different neuron models that define the behavioral dynamics of neurons:

1. **Leaky Integrate-and-Fire (LIF)** - The primary model historically used in FEAGI, featuring:
   - Membrane potential that decays over time
   - Firing threshold
   - Refractory period
   - Simple but computationally efficient

2. **Custom Models** - FEAGI 2.0 is designed to enable supporting multiple neuron models, including:
   - Izhikevich model elements for more biologically realistic behavior
   - Customizable dynamics parameters
   - Model-specific properties 

## WebGPU Implementation

As part of the NPU's GPU acceleration strategy, the Burst Engine can leverage WebGPU for cross-platform GPU acceleration. Below is an example of a WebGPU compute shader for neuron dynamics:

```wgsl
@group(0) @binding(0) var<storage, read> area_is_active: array<u32>;
@group(0) @binding(1) var<storage, read> area_ids: array<u32>;
@group(0) @binding(2) var<storage, read_write> membrane_potentials: array<f32>;
@group(0) @binding(3) var<storage, read> resting_potentials: array<f32>;
@group(0) @binding(4) var<storage, read> membrane_time_constants: array<f32>;
@group(0) @binding(5) var<storage, read> firing_thresholds: array<f32>;
@group(0) @binding(6) var<storage, read_write> fcl_buffer: array<u32>;
@group(0) @binding(7) var<storage, read_write> refractory_timers: array<u32>;
@group(0) @binding(8) var<storage, read> refractory_periods: array<u32>;
@group(0) @binding(9) var<storage, read> active_neuron_indices: array<u32>;
@group(0) @binding(10) var<uniform> active_neuron_count: u32;

@compute @workgroup_size(256)
fn update_neurons(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let thread_id = global_id.x;
    if (thread_id >= active_neuron_count) {
        return;
    }
    
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
        fcl_buffer[neuron_idx] = 1;
        membrane_potentials[neuron_idx] = rest;
        refractory_timers[neuron_idx] = refractory_periods[neuron_idx];
    }
}
```

This shader demonstrates several important aspects of the NPU's design:

1. **Sparse Processing**: Only active neurons (those in `active_neuron_indices`) are processed
2. **Area-Based Filtering**: Early termination for neurons in inactive cortical areas
3. **Refractory Period Handling**: Neurons in refractory periods are skipped from further computation
4. **Memory Layout**: Uses Structure of Arrays (SoA) for optimal memory access patterns
5. **Workgroup Optimization**: Uses a workgroup size of 256 for efficient GPU utilization

By using WebGPU, FEAGI achieves high-performance neural simulation across multiple platforms without vendor lock-in, making advanced brain simulation more accessible and portable. 