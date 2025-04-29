# Master Design Document for FEAGI 2.0

## Project Overview

The Framework for Evolutionary Artificial General Intelligence (FEAGI) 

## Features

- Multi-backend support (CPU/GPU)
- FastAPI-based REST API
- High-performance ZMQ messaging for bidirectional communication
- Modular architecture and seperation of compute intensive operations (neural processing) with async operations (neural development, data visualization, app management)
- Pathway for evolutionary optimization
- Configurable visualization sampling for high-frequency simulations


FEAGI uses a message-based architecture for its BurstEngine implementation, providing improved reliability and performance. This architecture prevents server hangs during heavy load and provides better timeout handling for neuron injection operations.




## Project Structure
```
./
├── feagi/                  # Main package
│   └── core/               # Core functionality and resource management
│       ├── resource_mgr.py # ZMQ handler
│       ├── zmq/            # ZMQ handler
│       ├── sec/            # Placeholder for authentication and encryption modules
│       └── api/            # API
│           └── routes/     # routes structured by version
│               ├── v1/     # 
│               └── v2/     #
│   ├── bdu/                # Brain Developmental Unit
│   ├── npu/                # Neural Processing Unit
│   ├── evo/                # Evolutionary algorithms 
│   ├── pns/                # Peripherial Nervous System and sensorimotor IO modules
│   └── viz/                # Visualization data transformation
├── tests/                  # Unit, integration, and functional tests
├── docs/                   # Documentation
├── examples/               # Example scripts
├── Dockerfile              # Container definition
└── requirements.txt        # Dependencies
```

## Compute Resource Strategy
- A subset of operations under Neural Processing Unit associated with neuron firing will be designed 
to support both CPU and GPU backends. The rest of the application will be running on CPU.
- The entire application needs to be designed in such way that can run as a highly parallel and performant fashion
- The code must be written in a format, so it can be easily ported into Rust in the future.
- Quality of Service priority should be assigned to various tasks ensuring critical process needs


## Security Considerations
### Authentication
FEAGI API and ZMQ will be equipped with authentication enabling secure communication on all communication methods.

### Encryption
Encryption can negatively impact the transmission of sensorimotor data by adding latency but might be essential for 
select use-cases. Both API and ZMQ support the encryption option and measures should be made to make encryption 
available as an option in the future.

## Design Requirements

Major modules: 
- Core (CORE)
  - API/Webserver (API)  
  - ZMQ message handler
- Peripheral Nervous System (PNS)
  - Sensory processor
  - Motor processor
- Brain Developmental Unit (BDU)
- Neural Processing Unit (NPU)
  - Burst Engine

- Memory & Learning Unit (MLU)
- Evolutionary Unit (EVO)
- Sleep (SLP)
  - Memory consolidation
  - Neural development
  

### REST API
- Should be organized in version folders to enable future maintainability. Routes should be defined enabling endpoints 
to be organized by functional area.

### ZMQ Handler
Initiates a ZMQ server with the ability to support multiple topics enabling multithreaded communication to and from 
FEAGI. 

### Resource Manager
FEAGI consists of many independent processes with various requirements such as some running on CPU vs. others suitable 
for GPU, some being mission-critical and highly time sensitive such neuron firing process vs sleep or api 
handling. The resource manager is responsible for starting, terminating, and orchestrating all FEAGI processes and 
initialization of critical data structures.

### Brain Development Unit (BDU)

The Brain Developmental Unit (BDU) facilitates methods and processes that enables the creation of:
- cortical areas
- neurons
- brain regions
- connectivity rules
- cortical mappings
- synapses

#### Cortical Areas
Cortical areas are 3D meshes that represent an ensemble of neurons with common properties. Each cortical area is 
consisted of distinct neuron blocks where each could house one or more neurons.

Refer to `cortical_area.md` for additional details.


#### Neurons and Global Neuron Array
FEAGI is built to support up to billions of neurons. This requires a data structure that can support the storage of such 
large neuron count in a very efficient way. When neurons are generated, depending on the cortical area they belong 
they will inherit the properties of given cortical area. 

The definition for each individual neuron along with dynamic firing variables are captured within the Global Neuron 
Array (GNA). 

Considering the scalability requirements, the GNA data structure could grow considerably large. It is crucial to 
maintain a minimal footprint on GAN while allowing high performance CRUD operations against it.

Refer to `neuron.md` for additional details.

##### Structure of Arrays (SoA) for Global Neuron Array

To optimize for both CPU SIMD capabilities and GPU acceleration, the Global Neuron Array (GNA) uses a Structure of Arrays (SoA) data layout instead of traditional Array of Structures (AoS) approach.

**Benefits of SoA Layout for Neural Simulation:**

- **SIMD Optimization:** Modern CPUs can process multiple data elements simultaneously using SIMD instructions (AVX, SSE, NEON). SoA layout allows loading multiple neuron properties (like membrane potentials) into SIMD registers for parallel computation.

- **Memory Coalescing:** GPU threads achieve maximum performance when accessing contiguous memory locations. SoA ensures that adjacent threads processing different neurons access adjacent memory locations when reading/writing the same property.

- **Cache Efficiency:** Processing properties in separate arrays maximizes CPU cache line utilization when algorithms need to process only certain neuron properties.

- **Selective Updates:** Separating static properties (rarely changing) from dynamic properties (frequently changing) reduces CPU-GPU transfer overhead, as only modified arrays need to be synchronized.

**Implementation:** In practice, the GNA is organized as multiple parallel arrays, with one array per neuron property. The index of a neuron serves as the common key across all property arrays:

```python
# Example SoA implementation for the Global Neuron Array
import numpy as np

area_ids = np.zeros(max_neurons, dtype=np.uint32)       # Parent cortical area ID
positions = np.zeros((max_neurons, 3), dtype=np.int16)  # 3D coordinates
membrane_potentials = np.zeros(max_neurons, dtype=np.float32)
firing_thresholds = np.zeros(max_neurons, dtype=np.float32)
refractory_timers = np.zeros(max_neurons, dtype=np.int16)
# ... other neuron properties ...
```

##### Memory Management Strategy for GNA

The Global Neuron Array uses a sophisticated memory management system to efficiently handle dynamic neuron creation, deletion, and simulation operations:

1. **Slotted Allocation System:**
   - Pre-allocated arrays with fixed-size slots for neuron properties
   - A slot manager tracks occupied and free slots for efficient reuse
   - Support for atomic batch operations when creating or removing multiple neurons

2. **Dynamic Growth Strategy:**
   - Initial capacity is set to a reasonable size based on expected workload
   - When capacity nears a threshold (e.g., 80%), a background compaction task prepares for expansion
   - Arrays are grown by a geometric factor (e.g., 1.5x) to amortize resizing costs
   - Growth operations are scheduled to avoid disrupting critical simulation steps

3. **Sparse Activation Tracking:**
   - Dedicated data structures track which neurons are active, avoiding the need to process all neurons
   - Active neuron indices are stored in compact arrays for efficient processing
   - Periodic rebuilding of active neuron lists adapts to changing activation patterns

This memory management approach balances performance, memory efficiency, and dynamic capacity requirements for large-scale neural simulations.

#### Brain Regions
Brain regions define a collection of cortical areas and represent a functional structure with distinct inputs and 
outputs. Brain regions can be exported as independent genomes and can be imported as a functional building block.

Brain region properties:
- region label
- region id
- region description
- region formation date
- region uuid
- region developer
- region members
- region parent
- region inputs
- region outputs

#### Connectivity Rules
A connectivity rule defines how a neuron can find its destination.

Connectivity rule type:
- Vector: a 3D vector that would translate the location of a neuron to find its destination
- Pattern: collection of wildcards and instructions on how a source and destination neuron could relate to eachother
- Function: a connectivity rule described by a function

Refer to `connectivity_rule.md` for additional details.

#### Cortical Mappings
The relationship defined between a source and destination cortical area (can be self) that defines how neurons are 
supposed to connect to each other.

Cortical mapping properties:
- connectivity rule id
- post synaptic potential multiplier
- inhibitory flag (bool)
- plasticity flag (bool)
- plasticity type
- plasticity decay
- plasticity coefficient (int32)

Refer to `cortical_mapping.md` for additional details.

#### Synapses
Defines the connection between two neurons. Synaptic properties, in addition to the source and destination data, 
includes properties inherited from cortical mapping that leads to their generation.

FEAGI is built to support up to billions of neurons which can lead

Refer to `synapse.md` for additional details.

##### Synapse Memory Organization

For efficient storage and processing of potentially billions of synapses, FEAGI uses a Structure of Arrays (SoA) approach similar to the neuron representation:

```python
# Example SoA implementation for synapses
import numpy as np

pre_neuron_indices = np.zeros(max_synapses, dtype=np.int32)   # Source neuron
post_neuron_indices = np.zeros(max_synapses, dtype=np.int32)  # Target neuron
weights = np.zeros(max_synapses, dtype=np.float32)            # Current weight
delays = np.zeros(max_synapses, dtype=np.int16)               # Delay in timesteps
# ... other synapse properties ...
```

To facilitate fast neuron-to-synapse lookups, specialized connectivity acceleration structures are maintained:

```python
# For each neuron, store quick references to its synapses
import numpy as np

outgoing_synapse_start_indices = np.zeros(max_neurons, dtype=np.int32)
outgoing_synapse_counts = np.zeros(max_neurons, dtype=np.int32)
incoming_synapse_start_indices = np.zeros(max_neurons, dtype=np.int32)
incoming_synapse_counts = np.zeros(max_neurons, dtype=np.int32)
```

These acceleration structures enable O(1) access to a neuron's incoming and outgoing connections, which is critical for efficient signal propagation during simulation.

#### Connectome
The collection of all data structures representing the brain is called connectome that includes 
- neurons
- synapses
- connectivity rules
- cortical areas
- brain regions
- cortical mappings


#### Connectome Manager
Connectome manager is the process responsible for: 
- initialization of connectome data structures e.g. neurons, synapses, cortical area, brain regions, and mappings
- performing CRUD (create, read, update, delete) operations against elements within each connectome data structure

- preservation and revival of the connectome. The preservation involves the
serialization and storage of the connectome and the revival would involve the reverse process where the artificial brain
can return to its functional state while preserving its integrity including all learning.


### Sensorimotor IO

#### Spike Encoding and Decoding
The conversion of sensory data into spikes (neural encoding) and neuronal activities into motor actions 
(neural decoding) is discussed here. 

FEAGI leverages temporal encoding.

### Burst Engine
The Burst Engine handles all activities related to neuron firing that includes:
- Updating membrane potentials
- Management of the fire candidate list queue
- Management of cortical stimulation queue


#### Fire Candidate List Queue
The Fire Candidate List contains the list of neurons that ought to be fired simultaneously across the entire brain.

The act of firing all neurons within the fire candidate list is called a burst of neuron firing or "burst" for short.

Fire Candidate List Queue keeps track of fire candidate list across N number of consecutive bursts where the value N is 
set based on FEAGI configuration parameters and can be overwritten using API.

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

**Performance Benefits:**

The Roaring Bitmap implementation provides several key advantages for the FCL management:

1. **Temporal Pattern Analysis:** Efficiently query neurons that fired across various time windows, enabling analysis of activity propagation, synchronization patterns, and bursting behaviors.

2. **Memory Efficiency:** For simulations with millions of neurons but sparse firing, Roaring Bitmaps compress the FCL representation by 10-100x compared to dense arrays, allowing longer temporal windows to be maintained.

3. **Computational Efficiency:** Set operations are optimized to work directly on the compressed representation, providing orders of magnitude speedup for temporal analyses compared to operations on raw arrays.

4. **Plasticity Support:** Fast time-window queries enable efficient implementation of timing-dependent plasticity mechanisms with minimal overhead.

#### FCL Manager
At the heart of the burst engine lies the FCL manager that is responsible for reading FCL content and initiating the 
firing of neurons. FCL Manager can be tapped by the FCL sampler to provide a copy of the current FCL content for 
transmission to the peripheral nervous system (PNS) and data visualization.

#### FCL Sampler
Responsible for invoking the FCL manager at a configurable frequency for the purpose generating motor commands and brain 
visualization.

### Neuron Firing Dynamics

#### Neuron Models
Neuron model defines the behavioral dynamics of a neuron and dictates what leads to the firing of a neuron, how it 
fires, and how it behaves. FEAGI has been traditionally supporting only Leaky Integrate and Fire (LIF) neuron model 
while taking elements of Izhikevich neuron model. FEAGI 2.0 is being designed to enable the option of supporting 
multiple neuron models but one at a time. 

### Neural Processing Unit (NPU) Implementation

The Neural Processing Unit is responsible for the core simulation of neuron dynamics and synapse operations. It is the most computationally intensive component of FEAGI and is designed to leverage both CPU and GPU acceleration.

#### CPU/GPU Acceleration Strategy

The NPU employs a dual-path design for neural processing:

1. **CPU Path:**
   - Leverages SIMD instructions (AVX/SSE/NEON) for parallel processing
   - Optimized for systems without dedicated GPUs
   - Uses cache-friendly memory access patterns
   - Well-suited for sparse activation patterns with complex logic

2. **GPU Path (WebGPU):**
   - Cross-platform GPU acceleration without vendor lock-in
   - Compatible with most modern GPUs including mobile/embedded devices
   - Enables massive parallelization for neuron and synapse updates
   - Example WebGPU compute shader for neuron dynamics:

The following WebGPU shader illustrates how neuron dynamics are implemented:

```python
# WebGPU shader for neuron update 
# NOTE: This is pseudocode representing a WGSL shader, not executable Python
"""
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
"""
```

**Rationale for WebGPU:**
- Avoids vendor lock-in compared to CUDA (NVIDIA) or Metal (Apple)
- Provides a modern API with explicit memory control
- Works across desktop, mobile, and potentially embedded platforms
- Enables future web-based visualization and deployment options
- Balances performance and portability requirements

#### Performance Optimizations

The NPU incorporates several key performance optimizations:

1. **Sparse Processing:**
   - Leverages the fact that neural activity is naturally sparse (1-10% active)
   - Maintains dedicated lists of active neurons/synapses
   - Only processes active components during simulation
   - Early termination for inactive areas and refractory neurons

2. **Memory Access Patterns:**
   - GPU computations use coalesced memory access for maximum throughput
   - CPU code organizes data for cache-friendly access
   - Property arrays are processed in stages to maximize cache hits
   - Minimizes CPU-GPU transfers by separating static and dynamic properties

3. **Workload Distribution:**
   - Dynamically selects kernels based on activation density
   - Groups neurons by cortical area for improved locality
   - Balances work across available compute resources
   - Two-phase processing separates neuron dynamics from synapse operations

These optimizations enable FEAGI to efficiently simulate large-scale neural networks even on modest hardware, making advanced brain simulations more accessible.

### Templates
A master template file called `templates.py` will capture all default parameters for FEAGI related to brain development.


# Path to Rust Migration
There is a strong incentive to make FEAGI available in Rust. The entire codebase needs to be written in such way 
allowing for a smooth Rust migration.

Best Practices for Writing Python-to-Rust-Compatible Code

| **Best Practice**           | **Why?**                                      | **Example**                                   |
|-----------------------------|----------------------------------------------|----------------------------------------------|
| **Use Type Hints (`mypy`)** | Rust requires explicit types                 | `def add(a: int, b: int) -> int:`           |
| **Avoid Pythonic OOP**      | Rust uses **structs + traits**, not classes  | Use `dataclass` instead of deep class trees |
| **Use Functional Programming** | Rust favors **immutability & pure functions** | `map(), filter()` instead of loops          |
| **Use NumPy (or Fixed-Size Lists)** | Rust prefers **arrays or Vec<T>**    | Prefer `np.ndarray` over `list`             |
| **Avoid Global State**      | Rust forces **explicit state management**    | Use function arguments instead of globals   |
| **Use PyO3 for Gradual Migration** | Rust functions can be called in Python | Compile Rust into a Python module           |


## Visualization Architecture
FEAGI uses a decoupled visualization approach:

1. The `feagi.viz.network_vis` module transforms neural network data into a serializable format
2. This data is sent via ZMQ to a separate application (PNS Bridge)
3. The PNS Bridge connects to a Godot application for 3D visualization as well as sensorimotor controller enabling
embodiment integration.

This architecture allows for:
- Real-time visualization of large networks
- Decoupling of simulation and visualization resources
- Flexible 3D representation of neural activity
- Distributed operation across multiple machines


### Fire Candidate List Visualization

The `FCLVisualizer` component provides real-time visualization of neuron firing activity by:

1. Subscribing to fire candidate list (FCL) updates from the burst engine
2. Applying configurable sampling to handle high-frequency simulations
3. Converting FCL data to the format expected by the visualization system
4. Sending the processed data to the network visualizer

**Visualization Sampling**:

The FCL visualizer supports two sampling modes to maintain visualization performance:

- **Ratio-based sampling** (0.0-1.0): Randomly selects a percentage of FCLs to visualize
  - Example: 0.1 means visualize ~10% of FCLs
  
- **Frequency-based sampling** (>1.0): Caps visualization at a maximum frequency (Hz)
  - Example: 30 means visualize at most 30 FCLs per second

This sampling can be configured via the `--vis-sampling` command-line parameter:
```
# Visualize every FCL (no sampling)
python -m feagi.main --vis-sampling 1.0

# Visualize at most 30 FCLs per second
python -m feagi.main --vis-sampling 30
```


## Implementation Strategy
Given the enormous size of this project it is vital to follow a solid design strategy as outlined below:
1. Initialize project skeleton and folder structure enabling a highly modular project with clear path to rust migration including:
   1. Folder structure
   2. API server
   3. ZMQ server
   4. Resource manager
2. Create the following functional modules in order and ensure every single functional module has an associated pytest folder. Additionally, each functional module should have an associated API route dedicated to it enabling REST management.
   1. Brain Developmental Unit
      1. Connectome Manager
      2. Cortical area module
      3. Neuron module
      4. Connectivity rules module
      5. cortical mappings module
      6. synapse module
   2. Burst Engine
      1. Burst Engine Manager
      2. FCL Manager
      3. FCL Sampler
   3. PNS message broker
   4. Memory and Learning Manager
   5. Async Stem Cell Manager
   6. Sleep Manager


## Benchmarking
It is crucial to be able to collect performance and resource consumption metrics during FEAGI operation to ensure:
1. Enable optimization
2. Prevent regression

### Benchmarking Strategy
TBD


## Testing
All unit and functional tests will be written using pytest framework.

### Test Strategy
TBD

## Configuration

Edit configuration to customize:
- CPU core allocation
- GPU utilization
- Network parameters
- API settings
- Visualization preferences (sampling rate, disabled/enabled)

## Future Development

The NPU modules are designed with a clean interface to facilitate future reimplementation in Rust for enhanced performance.

Additional planned extensions include:

1. **Multi-scale Integration:**
   - Support for different temporal resolutions in different cortical areas
   - Integration with lower-fidelity whole-brain models
   - Bridging between detailed and abstract simulations

2. **Advanced Plasticity Mechanisms:**
   - Spike-Timing-Dependent Plasticity (STDP)
   - Homeostatic scaling
   - Structural plasticity (dynamic synapse creation/pruning)
   - Neuromodulation effects on learning