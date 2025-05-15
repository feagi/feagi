# BDU Architecture Design

## Overview

The Brain Development Unit (BDU) is responsible for creating and managing the connectome - the complete neural structure of FEAGI. The architecture is designed with the following principles:

1. **Centralized Coordination**: ConnectomeManager serves as the primary entry point and coordinator
2. **Specialized Components**: Dedicated modules handle specific domains
3. **Genome-Driven Development**: Neural structures are derived from genomic specifications
4. **Performance Optimization**: Critical operations are optimized for neural simulation

## Folder Structure

```
feagi/bdu/
│
├── __init__.py                  # Package exports
├── connectome_manager.py        # Main ConnectomeManager class - primary entry point
├── neuroembryogenesis.py        # Translation of genome to connectome
│
├── utils/                       # Shared utilities
│   ├── __init__.py
│   ├── position.py              # Position calculation utilities
│   ├── linearization.py         # Linearization/delinearization operations
│   ├── validation.py            # Connectome validation utilities
│   └── metrics.py               # Performance and statistics utilities
│
├── connectivity/                # Connectivity-related modules
│   ├── __init__.py
│   ├── connectivity_rules.py    # Rules governing inter-area connections
│   ├── cortical_mappings.py     # Spatial mappings between cortical areas
│   └── synaptogenesis.py        # Rules for synapse formation
│
└── models/                      # Data models (if needed)
    ├── __init__.py
    ├── neuron.py                # Neuron data model
    ├── synapse.py               # Synapse data model
    ├── cortical_area.py         # Cortical area data model
    └── brain_region.py          # Brain region data model
```

## Component Responsibilities

### ConnectomeManager (connectome_manager.py)

The central coordinator for all connectome operations, providing:

1. **Neuron CRUD Operations**
   - Create, read, update, delete neurons
   - Batch operations
   - Neuron properties management
   - Position management
   - Query operations

2. **Synapse CRUD Operations**
   - Create, read, update, delete synapses
   - Synapse properties management
   - Connectivity queries
   - Batch operations
   - Specialized sparse matrix representations for memory efficiency
   - Vectorized operations for performance
   - Optimized storage structures

3. **Cortical Area CRUD Operations**
   - Create, read, update, delete cortical areas
   - Area properties management
   - Dimensional operations
   - Area queries

4. **Brain Region Operations**
   - Region hierarchy management
   - Region-area associations
   - Hierarchical queries

5. **Connectivity Rule Operations**
   - Create, read, update, delete connectivity rules
   - Rule application and execution
   - Rule-based queries and analysis

6. **Cortical Mapping Operations**
   - Create, read, update, delete spatial mappings between areas
   - Coordinate transformations
   - Mapping properties management
   - Topology management

7. **Synaptogenesis Rule Operations**
   - Create, read, update, delete synaptogenesis rules
   - Rule application during development
   - Rule-based connection formation

8. **Simulation Operations**
   - Membrane potential updates
   - Neural firing control
   - Simulation timestep management

9. **Serialization/Deserialization**
   - Save and load operations
   - Import/export functionality

10. **Development Operations**
    - Interface with neuroembryogenesis.py
    - Genome-based development coordination

### Neuroembryogenesis (neuroembryogenesis.py)

Handles the translation of genomic information to connectome structures:

1. **Genome Interpretation**
   - Read genomic instructions
   - Interface with GenomeProcessor (from evo module)

2. **Developmental Processes**
   - Area development from genes
   - Neuron generation from genes
   - Connectivity establishment from genes
   - Morphogenesis processes

### Connectivity Rules (connectivity/connectivity_rules.py)

Manages rules that govern how neurons connect between areas:

1. **Rule Representation**
   - Probabilistic connection rules
   - Distance-based connection rules
   - Feature-based connection rules

2. **Rule Application**
   - Efficient rule execution
   - Dynamic rule application during development

### Cortical Mappings (connectivity/cortical_mappings.py)

Manages spatial relationships between different cortical areas:

1. **Mapping Types**
   - Topological mappings
   - Functional mappings
   - Projection mappings

2. **Coordinate Transformations**
   - Transform coordinates between areas
   - Handle dimensionality differences

### Synaptogenesis Rules (connectivity/synaptogenesis.py)

Manages rules for synapse formation during development:

1. **Formation Rules**
   - Activity-dependent rules
   - Growth-factor rules
   - Hebbian/anti-Hebbian rules

2. **Plasticity Rules**
   - Strengthening/weakening rules
   - Pruning rules

## ConnectomeManager Interface Design

The ConnectomeManager presents a clean, comprehensive API organized by domain:

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
add_cortical_area(area_id, name, area_type, dimensions, position, **properties)
add_custom_cortical_area(name, coordinates, dimensions, **properties)

# READ
get_cortical_area(area_id)
get_cortical_area_by_name(name)
get_cortical_name_list()

# UPDATE
update_cortical_area(area_id, updates)
resize_cortical_area(area_id, new_dimensions)

# DELETE
delete_cortical_area(area_id, delete_neurons=False)
```

### Brain Region Operations
```python
# CREATE
create_brain_region(name, parent_id=None, **properties)

# READ
get_brain_region(region_id)
get_areas_in_region(region_id)
get_region_hierarchy()

# UPDATE
update_brain_region(region_id, updates)
move_region_in_hierarchy(region_id, new_parent_id)

# DELETE
delete_brain_region(region_id, recursive=False)
```

### Connectivity Rule Operations
```python
# CREATE
create_connectivity_rule(source_area_id, target_area_id, rule_type, parameters)
register_custom_rule(rule_name, rule_function)

# READ
get_connectivity_rule(rule_id)
get_rules_for_areas(source_area_id, target_area_id)
list_available_rule_types()

# UPDATE
update_connectivity_rule(rule_id, parameters)
enable_disable_rule(rule_id, enabled=True)

# DELETE
delete_connectivity_rule(rule_id)
clear_rules_for_area(area_id)
```

### Cortical Mapping Operations
```python
# CREATE
create_cortical_mapping(source_area_id, target_area_id, mapping_type, **params)
create_topological_mapping(source_area_id, target_area_id, topology_params)

# READ
get_cortical_mapping(mapping_id)
get_mappings_for_area(area_id, direction="both")
transform_coordinates(mapping_id, source_coordinates)

# UPDATE
update_cortical_mapping(mapping_id, updates)
update_mapping_parameters(mapping_id, parameters)

# DELETE
delete_cortical_mapping(mapping_id)
delete_all_mappings_for_area(area_id)
```

### Synaptogenesis Rule Operations
```python
# CREATE
create_synaptogenesis_rule(rule_type, parameters)
register_custom_synaptogenesis_rule(rule_name, rule_function)

# READ
get_synaptogenesis_rule(rule_id)
list_available_synaptogenesis_rules()

# UPDATE
update_synaptogenesis_rule(rule_id, parameters)
enable_disable_synaptogenesis_rule(rule_id, enabled=True)

# DELETE
delete_synaptogenesis_rule(rule_id)
```

### Simulation Operations
```python
update_membrane_potentials(current_timestep=None)
run_simulation_steps(steps=1)
inject_activity(neuron_ids, values=None)
```

### Neuroembryogenesis Operations
```python
develop_from_genome(genome_id, development_config=None)
develop_cortical_area(area_spec, region_id=None)
develop_neuron_population(area_id, population_spec)
develop_connectivity(source_id, target_id, connectivity_spec)
apply_synaptogenesis_rules(rule_ids=None)
```

## Implementation Strategy

1. **Phase 1: Refactor ConnectomeManager**
   - Consolidate synapse management directly into ConnectomeManager
   - Reorganize methods by domain
   - Ensure full CRUD capabilities
   - Clean up parameter handling

2. **Phase 2: Optimize Critical Paths**
   - Identify performance bottlenecks
   - Implement specialized synapse data structures
   - Optimize neuron array operations

3. **Phase 3: Enhance Embryogenesis**
   - Improve integration with genome processor
   - Enhance developmental capabilities
   - Implement synaptogenesis rule system

4. **Phase 4: Add Missing Capabilities**
   - Enhance cortical mapping
   - Implement connectivity rule system
   - Improve region management
   - Add missing operations identified earlier

## Integration with Other FEAGI Components

The BDU interacts with other FEAGI components as follows:

1. **EVO Module**: Provides genomic information that BDU translates into connectome structures
2. **NPU Module**: Consumes the connectome for neural simulation
3. **API Module**: Exposes connectome operations through REST/gRPC interfaces
4. **VIZ Module**: Reads connectome state for visualization 