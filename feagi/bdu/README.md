# FEAGI Brain Development Unit (BDU)

*Last Updated: May 15, 2025*

## Overview

The Brain Development Unit (BDU) is responsible for creating and managing the connectome - the complete neural structure of FEAGI. It translates genome specifications into neural structures and provides comprehensive APIs for manipulating neurons, synapses, cortical areas, and brain regions.

## Key Components

### ConnectomeManager

The `ConnectomeManager` serves as the primary entry point and coordinator for all connectome operations:

```python
from feagi.bdu import ConnectomeManager

# Create a ConnectomeManager instance
connectome = ConnectomeManager()

# Initialize neuron and synapse arrays
connectome.initialize_arrays()

# Access neurons and synapses
neuron_data = connectome.get_neuron(neuron_id)
synapses = connectome.get_outgoing_connections(neuron_id)
```

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

The GPU-optimized implementation follows these key principles:

### 1. Structure of Arrays (SoA) Pattern

Instead of storing neurons as individual objects or dictionaries, we store neuron properties in columnar format as contiguous arrays:

```python
# Instead of:
self.neurons = {
    neuron_id: {
        "membrane_potential": 0.0,
        "threshold": 1.0,
        # ...other properties
    }
}

# We use:
self.membrane_potentials = np.zeros(max_neurons, dtype=np.float32)
self.thresholds = np.ones(max_neurons, dtype=np.float32)
# ...other properties as arrays
```

This allows for:
- Vectorized operations on properties (e.g., decay all membrane potentials at once)
- Efficient memory layout for cache performance
- Direct transfer to GPU memory
- SIMD-friendly data structures

### 2. Sparse Matrix Synapse Representation

Synaptic connections are stored using sparse matrices instead of nested dictionaries:

```python
# Instead of:
self.connections = {
    pre_neuron_id: {
        post_neuron_id: weight
    }
}

# We use:
self.outgoing_matrix = sparse.lil_matrix((max_neurons, max_neurons), dtype=np.float32)
self.incoming_matrix = sparse.lil_matrix((max_neurons, max_neurons), dtype=np.float32)
```

Benefits include:
- Efficient memory usage for large connectomes
- Specialized formats for different operations (CSR for row access, CSC for column access)
- Vectorized matrix operations for signal propagation
- Compatible with GPU sparse matrix libraries

### 3. Boolean Masks for Grouping

To represent groupings (e.g., neurons in a cortical area), we use boolean masks instead of sets:

```python
# Instead of:
self.area_neuron_map = {
    area_id: {neuron_id1, neuron_id2, ...}
}

# We use:
self.area_neuron_masks = {
    area_id: np.zeros(max_neurons, dtype=np.bool_)
}
# And set True for members: self.area_neuron_masks[area_id][index] = True
```

This enables:
- Vectorized filtering and selection
- Fast intersection and union operations
- Compact memory representation

### 4. Memory-Aligned Types

We use memory-aligned types that are efficient for SIMD and GPU processing:
- `float32` for most neuron properties (membrane potentials, weights)
- `int32` for indices and counters
- `bool_` for boolean flags and masks

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

For maximum performance with larger networks, it's recommended to use batch operations whenever possible:

```python
# Batch create synapses
synapse_specs = [(pre_id1, post_id1, weight1), (pre_id2, post_id2, weight2), ...]
connectome.batch_create_synapses(synapse_specs)

# Update membrane potentials for all neurons at once
fired_neuron_ids = connectome.update_membrane_potentials()
```

## GPU Support

The implementation is designed to be compatible with GPU acceleration through NumPy and PyTorch:

1. **NumPy Arrays**: All neuron properties are stored in NumPy arrays that can be transferred to GPU memory.
2. **Sparse Matrices**: Synaptic connections use SciPy sparse matrices, which can be converted to GPU-compatible formats.
3. **PyTorch Integration**: The `NeuronArray` class includes methods for transferring data to and from GPU memory.

Future work will focus on implementing full WebGPU compatibility and optimizing for WGSL shaders. 