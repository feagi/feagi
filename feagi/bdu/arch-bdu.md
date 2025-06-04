# BDU Architecture

*Last Updated: May 15, 2025*

## Overview

The Brain Development Unit (BDU) is responsible for creating and managing the connectome - the complete neural structure of FEAGI. The architecture is designed with the following principles:

1. **Centralized Coordination**: ConnectomeManager serves as the primary entry point and coordinator
2. **Specialized Components**: Dedicated modules handle specific domains
3. **Genome-Driven Development**: Neural structures are derived from genomic specifications
4. **Performance Optimization**: Critical operations are optimized for neural simulation

## Component Architecture

![BDU Architecture Diagram](../../../docs/assets/bdu-architecture.png)

### ConnectomeManager

The central coordinator for all connectome operations, providing:

#### Neuron Operations
- Create, read, update, delete neurons
- Batch operations and vectorized access
- Neuron properties management
- Position management and spatial queries
- Optimized array-based storage structures

#### Synapse Operations
- Create, read, update, delete synapses
- Connectivity queries and graph traversal
- Batch operations for performance
- Sparse matrix representations for memory efficiency
- Vectorized synaptic operations

#### Cortical Area Operations
- Area lifecycle management
- Dimensional operations and spatial relationships
- Area properties management
- Inter-area connectivity

#### Brain Region Operations
- Region hierarchy management
- Region-area associations
- Hierarchical queries and traversal

### Neuroembryogenesis

Handles the translation of genomic information to connectome structures:

- Genome interpretation and processing
- Developmental processes implementation
- Progressive neural development
- Staging of developmental processes

### Connectivity Module

Manages rules that govern how neural elements connect:

#### Connectivity Rules
- Probabilistic connection rules
- Distance-based connection rules
- Feature-based connection rules
- Rule application algorithms

#### Cortical Mappings
- Topological mappings
- Functional mappings
- Coordinate transformations
- Cross-dimensional mappings

#### Synaptogenesis
- Synapse formation rules
- Growth cone guidance
- Activity-dependent synaptogenesis
- Competitive synapse formation

## Interface Design

The BDU provides a clean, comprehensive API organized by domain:

### Neuron Operations
```python
# CREATE
create_neuron(area_id, position, **properties)
batch_create_neurons(area_id, positions, **properties)

# READ
get_neuron(neuron_id)
get_neuron_property(neuron_id, property_name)
get_neurons_by_area(area_id)
query_neurons_by_criteria(criteria)

# UPDATE
update_neuron(neuron_id, updates)
set_neuron_property(neuron_id, property_name, value)
move_neuron(neuron_id, new_position)

# DELETE
delete_neuron(neuron_id)
delete_neurons_batch(neuron_ids)
```

### Synapse Operations
```python
# CREATE
create_synapse(pre_neuron_id, post_neuron_id, weight, **properties)
batch_create_synapses(synapse_specs)

# READ
get_synapse(pre_neuron_id, post_neuron_id)
get_outgoing_connections(neuron_id)
get_incoming_connections(neuron_id)

# UPDATE
update_synapse(pre_neuron_id, post_neuron_id, updates)
update_synapse_weight(pre_neuron_id, post_neuron_id, weight)

# DELETE
remove_synapse(pre_neuron_id, post_neuron_id)
delete_all_connections(neuron_id, direction)
```

### Cortical Area Operations
```python
# CREATE
create_cortical_area(properties)

# READ
get_cortical_area(area_id)
get_cortical_areas()
get_cortical_area_property(area_id, property_name)

# UPDATE
update_cortical_area(area_id, updates)
set_cortical_area_property(area_id, property_name, value)

# DELETE
delete_cortical_area(area_id)
```

## Memory Management

The BDU implements specialized memory management techniques:

### Structure of Arrays (SoA) Layout

To optimize for both CPU SIMD capabilities and potential GPU acceleration, the Global Neuron Array (GNA) uses a Structure of Arrays (SoA) data layout:

```
# Array of Structures (AoS) - Traditional but less efficient
neurons = [
    {id: 1, position_x: 10, position_y: 20, threshold: 0.5, ...},
    {id: 2, position_x: 11, position_y: 20, threshold: 0.5, ...},
    ...
]

# Structure of Arrays (SoA) - More efficient for vectorized operations
neuron_coordinates_x = [10, 11, ...]
neuron_coordinates_y = [20, 20, ...]
neuron_thresholds = [0.5, 0.5, ...]
```

Benefits of this approach:
- SIMD optimization for parallel computation
- Better memory coalescing for GPU operations
- Improved cache efficiency for property-specific operations
- Selective updates of only modified properties

### Slotted Allocation System

The Global Neuron Array uses an efficient slotted allocation system:

1. Pre-allocated arrays with fixed-size slots
2. Slot manager tracking occupied and free slots
3. Support for atomic batch operations
4. Dynamic growth strategy to handle scaling

### Sparse Activation Tracking

For efficient processing of active neurons:

1. Dedicated data structures track which neurons are active
2. Active neuron indices stored in compact arrays
3. Periodic rebuilding of active neuron lists to adapt to changing activation patterns

## Related Documentation

- [BDU Module README](README.md)
- [Connectome Management](docs/connectome.md)
- [Connectivity Rules](docs/connectivity_rule.md)
- [System Architecture](../../docs/arch-system-overview.md) 