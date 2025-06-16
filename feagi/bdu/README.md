# FEAGI Brain Development Unit (BDU)

*Last Updated: May 15, 2025*

## Overview

The Brain Development Unit (BDU) is responsible for managing the structure of the brain in FEAGI. This includes:

- Creation and management of brain structures (neurons, synapses, cortical areas)
- Implementation of neurodevelopmental processes (neurogenesis, synaptogenesis)
- Handling connectivity patterns between areas
- Providing an efficient, vectorized representation of the connectome for GPU-accelerated neural simulation

## Key Components

### ConnectomeManager

The `ConnectomeManager` is responsible for storing and manipulating connections between neurons. It provides methods for:

- Adding and removing neurons
- Creating and modifying synapses
- Querying connectivity
- Managing neuron properties

### ConnectomeManagerGPU

The `ConnectomeManagerGPU` is an optimized implementation of `ConnectomeManager` designed for SIMD and GPU acceleration. Key features include:

- Structure of Arrays (SoA) pattern for neuron properties
- Sparse matrix representation of connectivity
- Vectorized operations for membrane potential updates
- Batch operations for neuron property updates
- Support for multiple array backends (NumPy, PyTorch, CuPy, WebGPU)

### WebGPU Integration

The `ConnectomeManagerWebGPU` class provides WebGPU acceleration for the ConnectomeManagerGPU, allowing:

- Execution of neural updates on the GPU
- WGSL shader-based computation
- Efficient buffer transfers using staging buffers
- Optimal memory alignment for SIMD and GPU operations

### Array Backend Abstraction

The `ArrayBackend` class provides a unified interface for different array backends:

- NumPy (CPU)
- PyTorch (CPU/CUDA)
- CuPy (CUDA)
- WebGPU (WebGPU)

This allows for transparent switching between backends based on available hardware and performance requirements.

### NeuroEmbryogenesis

The `NeuroEmbryogenesis` component handles the translation of genomic information to connectome structures:

```python
from feagi.bdu import NeuroEmbryogenesis

# Create a neuroembryogenesis instance
embryo = NeuroEmbryogenesis(connectome_manager)

# Develop brain from genome
embryo.develop_brain_from_genome(genome)
```

### Core Models

- **CorticalArea**: Represents 3D meshes of neurons with common properties
- **Neuron/NeuronArray**: Represents individual neurons and efficient arrays of neurons
- **BrainRegion**: Represents functional structures comprising multiple cortical areas
- **SynapseManager**: Manages synaptic connections between neurons

### Connectivity Components

- **ConnectivityRule**: Defines how neurons connect between areas
- **CorticalMapping**: Manages spatial relationships between different cortical areas
- **SynaptogenesisRule**: Manages rules for synapse formation during development
- **SynapseRule**: Defines specific rules for synapse properties and behavior

## Directory Structure

```
feagi/bdu/
│
├── connectome_manager.py        # Main ConnectomeManager class
├── __init__.py                  # Package exports
│
├── models/                      # Data models
│   ├── neuron.py                # Neuron data model
│   ├── synapse.py               # Synapse data model
│   ├── cortical_area.py         # Cortical area data model
│   └── brain_region.py          # Brain region data model
│
├── connectivity/                # Connectivity-related modules
│   ├── connectivity_rules.py    # Rules for inter-area connections
│   ├── cortical_mappings.py     # Spatial mappings between areas
│   ├── synaptogenesis.py        # Rules for synapse formation
│   └── synapse_rule.py          # Specific synapse rules
│
├── embryogenesis/               # Development of brain from genome
│   └── neuroembryogenesis.py    # Genome to connectome translation
│
├── utils/                       # Shared utilities
│   ├── position.py              # Position calculations
│   ├── linearization.py         # Linearization operations
│   └── validation.py            # Validation utilities
│
└── docs/                        # Module documentation
    ├── bdu_design.md            # Design documentation
    ├── connectivity_rule.md     # Connectivity rules documentation
    └── connectome.md            # Connectome management documentation
```

## Key Features

### 1. Neuron Management

- **Efficient Storage**: Optimized array-based storage for billions of neurons
- **Fast Access**: Constant-time access to neuron properties
- **Vectorized Operations**: NumPy-based operations for performance
- **Sparse Representation**: Memory-efficient storage for active neurons

### 2. Synapse Management

- **Sparse Matrix Representation**: Memory-efficient storage for trillions of potential connections
- **Specialized Algorithms**: Fast operations on synaptic connections
- **Dynamic Connectivity**: Runtime modification of synaptic properties

### 3. Cortical Area Operations

- **Multi-dimensional Areas**: Support for 1D, 2D, and 3D cortical areas
- **Coordinate Transformations**: Mapping between different coordinate spaces
- **Area Properties**: Management of area-specific neuron properties

### 4. Development Capabilities

- **Genome-Driven**: Neural structures built from genomic specifications
- **Rule-Based Development**: Algorithmic construction of neural circuitry
- **Growth and Adaptation**: Dynamic modification of neural structures

## Usage Examples

### Creating a Cortical Area

```python
# Create a 2D cortical area of size 64x64x1
area_properties = {
    "name": "Visual Input",
    "dimensions": {"x": 64, "y": 64, "z": 1},
    "coordinates": {"x": 100, "y": 100, "z": 0},
    "cortical_type": "sensory",
    "neuron_params": {"threshold": 0.5, "leak": 0.1}
}
area_id = connectome.create_cortical_area(area_properties)
```

### Creating Cortical Mappings

```python
# Create a mapping between two cortical areas
mapping = {
    "source_area_id": "source_id",
    "destination_area_id": "dest_id",
    "mapping_type": "topographic",
    "connection_pattern": "one-to-one",
    "plasticity_params": {"learning_rate": 0.01}
}
mapping_id = connectome.create_cortical_mapping(mapping)
```

### Developing from Genome

```python
# Load a genome file
with open("genome.json", "r") as f:
    genome = json.load(f)

# Develop brain from genome
embryo = NeuroEmbryogenesis(connectome)
embryo.develop_brain_from_genome(genome)
```

## Performance Considerations

- Use batch operations when creating or updating multiple neurons/synapses
- Prefer vectorized operations over loops for array manipulations
- Cache frequent lookups to avoid repeated calculations
- For large-scale operations, use the specialized methods designed for performance

## Related Documentation

- [BDU Architecture](arch-bdu.md)
- [Connectome Management](docs/connectome.md)
- [Connectivity Rules](docs/connectivity_rule.md)
- [System Architecture](../../docs/arch-system-overview.md)

## ConnectomeManager Implementations

The BDU offers two implementations of the ConnectomeManager:

1. **Standard ConnectomeManager**: The original implementation using dictionary-based data structures for neuron and synapse storage.
2. **GPU-optimized ConnectomeManager**: A new implementation using NumPy arrays and sparse matrices for efficient data processing and transfer to GPU memory.

## GPU Optimization Strategy

### Memory Layout

- **Structure of Arrays (SoA):** All neuron properties are stored in separate contiguous arrays for efficient SIMD and GPU processing.
- **Memory Alignment:** Arrays are aligned to 64-byte boundaries for optimal SIMD (AVX-512) and cache line performance.
- **Sparse Matrix Format:** Synaptic connectivity is stored in CSR (Compressed Sparse Row) format for efficient memory usage and fast traversal.

### Sparse Matrix Format Consistency

The connectome manager uses specific sparse matrix formats for different access patterns:

- **CSR (Compressed Sparse Row)**: Used for outgoing connections to efficiently access all post-synaptic targets of a neuron. This format is optimized for row-based access patterns and is used when:
  - Simulating neuron firing and signal propagation (accessing all targets of a firing neuron)
  - Querying outgoing connections for a specific neuron
  - Performing batch operations on outgoing connections

- **CSC (Compressed Sparse Column)**: Used for incoming connections to efficiently access all pre-synaptic sources to a neuron. This format is optimized for column-based access patterns and is used when:
  - Querying incoming connections for a specific neuron
  - Computing convergent signals to a neuron
  - Performing learning operations that require knowledge of all synapses targeting a neuron

- **Automatic Format Conversion**: The system automatically converts between formats as needed, based on the operation being performed:
  - `_ensure_csr_format_outgoing()`: Ensures the outgoing matrix is in CSR format
  - `_ensure_csc_format_incoming()`: Ensures the incoming matrix is in CSC format
  - `_convert_to_lil_if_needed()`: Converts to LIL (List of Lists) format for efficient modifications

This approach ensures optimal performance for both kinds of operations while maintaining memory efficiency.

### Multi-GPU Support

The BDU architecture includes support for multi-GPU operation through a partition-based approach:

- **Domain Decomposition**: The brain is partitioned into regions that can be processed independently on different GPUs
  - Spatial partitioning based on cortical areas
  - Workload-balanced partitioning based on neuron and synapse density

- **Communication Strategy**:
  - Uses NCCL (for PyTorch/CUDA) or custom message passing (for WebGPU) for inter-GPU communication
  - Fire Candidate Lists (FCLs) are exchanged between partitions at synchronization points
  - Minimizes data transfer by only communicating active neurons (FCLs) rather than all neurons

- **Synchronization Model**:
  - Bulk Synchronous Parallel (BSP) approach with customizable synchronization frequency
  - Each GPU processes its partition independently, then synchronizes at specific intervals
  - Adjustable trade-off between accuracy and performance through sync frequency settings

- **Load Balancing**:
  - Dynamic partition adjustment based on runtime performance metrics
  - Migration of highly connected neural clusters to balance workloads
  - Automated performance optimization through self-tuning parameters

To enable multi-GPU operation, initialize with the desired configuration:

```python
from feagi.bdu.connectome_manager_gpu import ConnectomeManagerGPU
from feagi.bdu.multi_gpu import MultiGPUConfig

# Configure multi-GPU settings
multi_gpu_config = MultiGPUConfig(
    enabled=True,
    num_devices=2,  # Number of GPUs to use
    partition_method="cortical_areas",  # or "balanced"
    sync_frequency=10  # Synchronize every 10 timesteps
)

# Create connectome manager with multi-GPU support
connectome = ConnectomeManagerGPU(
    max_neurons=1_000_000,
    multi_gpu_config=multi_gpu_config
)
```

### Vectorized Operations

- **Batch Processing:** Operations are performed on entire arrays rather than individual neurons.
- **SIMD-friendly:** Computations are structured to enable SIMD acceleration on CPU.
- **GPU-compatible:** Array operations can be offloaded to GPU for parallel processing.

### Backend Abstraction

The code supports multiple computational backends:

- **NumPy:** For CPU-only environments
- **PyTorch:** For CUDA GPU acceleration
- **CuPy:** Alternative CUDA acceleration
- **WebGPU:** For cross-platform GPU acceleration, including browsers

### GPU Buffer Management

- **Staging Buffers:** Used for efficient CPU-GPU transfers
- **Double Buffering:** Minimizes stalls due to data dependencies
- **Aligned Memory:** Ensures optimal memory access patterns on GPU

### Mixed Precision Support

The backend abstraction layer provides support for various precision modes to optimize performance:

- **FP32 (Full Precision)**: Standard 32-bit floating point for maximum accuracy
- **FP16 (Half Precision)**: 16-bit floating point for faster computation and reduced memory usage
- **INT8 (Quantized)**: 8-bit integer for extremely efficient inference on supported hardware
- **Mixed Precision**: Automatic selection of precision based on operation needs

To use different precision modes:

```python
from feagi.bdu.models.array_backend import ArrayBackend, BackendType, PrecisionType

# Create a half-precision GPU backend
backend = ArrayBackend(BackendType.PYTORCH, PrecisionType.FP16)

# Create arrays with the selected precision
zeros = backend.zeros((1000, 1000))  # Will use float16 dtype

# Use mixed precision for maximum performance
mixed_backend = ArrayBackend(BackendType.PYTORCH, PrecisionType.MIXED)
# This will use FP16 where appropriate and FP32 where needed for stability
```

Benefits of mixed precision:
- Up to 2-3x performance increase for large networks
- Reduced memory footprint allowing larger models
- Hardware-accelerated mixed precision on modern GPUs

### WebGPU Shader Design

- **Workgroup Size:** Using 256 threads per workgroup for optimal GPU utilization
- **SoA in Buffers:** Maintaining the Structure of Arrays pattern in storage buffers
- **AoS in Workgroups:** Using Array of Structures for local workgroup memory
- **Atomic Operations:** Using atomic operations for concurrent updates to neurons

## Performance

The GPU-optimized implementation provides significant performance improvements for large-scale neural networks:

- **Vectorized Operations:** Batch updates are significantly faster than iterative updates
- **Sparse Matrix Operations:** Efficient traversal of synaptic connections
- **Multiple Backend Support:** Can leverage the fastest available hardware
- **WebGPU Acceleration:** Enables browser-based GPU acceleration

## Usage

The GPU-optimized ConnectomeManager can be used as a drop-in replacement for the standard implementation:

```python
from feagi.bdu.connectome_manager_gpu import ConnectomeManagerGPU

# Create an instance
connectome = ConnectomeManagerGPU(max_neurons=10_000_000)

# Use the same API as the standard ConnectomeManager
area_id = connectome.add_cortical_area("Visual Cortex", (100, 100, 10), (0, 0, 0))
neuron_id = connectome.create_neuron(area_id, (50, 50, 5))
```

You can also convert from the standard implementation to the GPU-optimized version:

```python
# Start with standard implementation
connectome = ConnectomeManager(max_neurons=10_000_000)

# ... create neurons, synapses, etc. ...

# Convert to GPU-optimized implementation
gpu_connectome = connectome.to_gpu_optimized()

# Continue using the GPU-optimized version
gpu_connectome.update_membrane_potentials()
```

For maximum performance with larger networks, it's recommended to use batch operations whenever possible:

```python
# Batch create synapses
synapse_specs = [(pre_id1, post_id1, weight1), (pre_id2, post_id2, weight2), ...]
connectome.batch_create_synapses(synapse_specs)

# Update membrane potentials for all neurons at once
fired_neuron_ids = connectome.update_membrane_potentials()
```

## Future Work

Future development will focus on:

1. **WebGPU Support**: Implementing full WebGPU compatibility using WGSL shaders
2. **Custom CUDA Kernels**: Developing specialized CUDA kernels for critical operations
3. **Multi-GPU Support**: Distributing computation across multiple GPUs
4. **Mixed-Precision Training**: Support for FP16 and INT8 operations for improved performance
5. **Rust Integration**: Integrating with Rust-based backend for even better performance

## Benchmarking

A benchmarking script is provided in `tests/performance/bdu/benchmark_backends.py` to compare the performance of different backends. Run it with:

```bash
python -m tests.performance.bdu.benchmark_backends
```

This generates performance comparisons between NumPy, PyTorch, CuPy, and WebGPU for different neuron counts.

## Tests

Tests for the BDU module are located in the `tests/unit/bdu/` directory. Run them with:

```bash
pytest tests/unit/bdu/
```

## Future Improvements

- **SIMD Intrinsics:** Direct use of SIMD intrinsics for critical paths
- **Custom CUDA Kernels:** Specialized kernels for neuronal operations
- **Multi-GPU Support:** Distribute computation across multiple GPUs
- **Rust Migration:** Port critical components to Rust for further performance gains
