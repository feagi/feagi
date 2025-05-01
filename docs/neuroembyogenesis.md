# Neuroembryogenesis Technical Documentation

## Overview

The neuroembryogenesis module is a core component of FEAGI that transforms genomic instructions (genotype) into a functional brain structure (phenotype). Inspired by biological neural development, it models the process of neural tube formation in embryos, where genetic instructions guide the creation of a complex, interconnected brain.

This module is responsible for reading the genome, creating the physical structure of the brain (neurons and their positions), and establishing connections between neurons (synapses) according to the genetic blueprint.

## Core Development Processes

The neuroembryogenesis process follows a biologically-inspired, sequential development pattern:

1. **Corticogenesis**: Creation of cortical area definitions
2. **Voxelogenesis**: Establishing the 3D spatial framework for neuron placement
3. **Neurogenesis**: Generation of neurons within each cortical area
4. **Synaptogenesis**: Formation of synaptic connections between neurons

### Development Modes

The system supports two primary development modes:

- **New Development**: Creates a brain structure from scratch based on genomic instructions
- **Reincarnation**: Reloads an existing connectome, preserving memories and learning from a previous instance

## Key Components

### Spatial Organization (Voxelogenesis)

The `voxelogenesis` function creates the spatial framework for neuron placement using a voxel-based system. Each cortical area is divided into 3D voxels, which serve as containers for neurons:

```python
voxel_dict = {
    "cortical_area_1": {
        (0, 0, 0): {"neuron_1_id", "neuron_2_id"},
        (0, 0, 1): {"neuron_3_id"},
        (0, 0, 2): {"neuron_4_id", "neuron_5_id"},
    },
    "cortical_area_2": { ... }
}
```

The dimensions of each cortical area are defined in the genome's blueprint, specifying the x, y, and z dimensions through the `block_boundaries` property.

### Neuron Creation (Neurogenesis)

The `neurogenesis` function populates each voxel with neurons according to the genome specifications. The key parameters that influence this process include:

- **Cortical area dimensions**: Defined by `block_boundaries` in the genome
- **Neuron density**: Controlled by `per_voxel_neuron_cnt` in the genome
- **Neuron properties**: Inherited from the cortical area definition

Each neuron is assigned unique properties and positioned within its voxel, with the actual creation delegated to the `neuron.create_neuron()` function.

### Connection Formation (Synaptogenesis)

The `synaptogenesis` and `build_synapses` functions establish connections between neurons based on:

1. **Source cortical area**: The origin of the connections
2. **Destination areas**: Defined in the `cortical_mapping_dst` property
3. **Connection patterns**: Specified by morphologies in the genome

The process involves:

- Identifying source and destination neurons
- Applying morphological rules to determine which neurons should connect
- Creating synapses with properties defined in the genome (strength, plasticity, etc.)

The actual synapse creation is delegated to the `synapse.neighbor_builder()` function.

## Brain Development Process

The main development flow is orchestrated by the `develop()` function, which:

1. Initializes the brain structure with empty cortical areas
2. Performs voxelogenesis for each cortical area
3. Executes neurogenesis to create neurons (skipping memory areas initially)
4. Conducts synaptogenesis to establish connections
5. Generates statistics about the brain (neuron count, synapse count)
6. Updates the genome with these statistics
7. Evaluates the structural fitness of the brain

## Plasticity Management

The `generate_plasticity_dict()` function extracts plasticity information from the genome to create a dictionary that maps:

- Source cortical areas
- Destination areas with plastic connections
- Direction of plasticity (efferent or afferent)

This supports the runtime plasticity mechanisms that allow synaptic weights to be modified during brain operation.

## Supporting Utilities

### Cortical Area Management
- `cortical_list()`: Retrieves all cortical areas from the genome
- `cortical_sub_group_members()`: Finds all areas belonging to a specific group
- `cortical_name_list()` and `cortical_name_to_id()`: Map between human-readable names and internal IDs

### Analytics and Evaluation
- `synapse_count()`: Counts synapses between specific cortical areas
- `connectome_structural_fitness()`: Evaluates the quality of the developed brain
- `build_cortical_map()`: Creates a graph representation of cortical connectivity

## Technical Implementation Details

### Data Structures

The primary data structures used are:

1. **Brain**: A hierarchical dictionary representing the entire brain
   ```
   brain = {
     "cortical_area_1": {
       "neuron_id_1": {
         "properties": {...},
         "neighbors": {...}
       },
       ...
     },
     ...
   }
   ```

2. **Voxel Dictionary**: Maps 3D coordinates to neurons
   ```
   voxel_dict = {
     "cortical_area_1": {
       (x, y, z): {neuron_id_1, neuron_id_2, ...},
       ...
     },
     ...
   }
   ```

3. **Intercortical Mapping**: Records connections between cortical areas
   ```
   [(source_area, destination_area, synapse_count), ...]
   ```

### Performance Considerations

The neuroembryogenesis process can be computationally intensive, especially for larger brain structures. The implementation includes:

- Timing measurements for performance monitoring
- Optional disk persistence for partial results
- Support for targeted development of specific cortical areas

### Integration with Other Systems

The module integrates with:

1. **Genome System**: Reads genetic instructions
2. **Connectome**: Stores the resulting brain structure
3. **Statistics**: Tracks brain complexity metrics
4. **Persistence**: Can save/load brain state

## Future Directions

Potential enhancements to the neuroembryogenesis system include:

1. **Parallel Development**: Further optimization of parallel processing for faster brain generation
2. **Dynamic Growth**: Supporting runtime neurogenesis for adaptive brain structures
3. **Enhanced Validation**: More sophisticated structural fitness evaluation
4. **Visualization Integration**: Better tools for visualizing the development process

## Conclusion

The neuroembryogenesis module translates abstract genetic descriptions into concrete neural architectures, serving as the bridge between the genome system and the operational connectome. By following biologically-inspired developmental principles, it creates brain structures that balance genetic determinism with stochastic variability, enabling both innate capabilities and learning potential. 