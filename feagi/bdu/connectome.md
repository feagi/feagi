# FEAGI Connectome Manager

## Overview
The **Connectome Manager** is the core component responsible for managing the neural connectome in FEAGI. It provides efficient, scalable, and thread-safe management of neurons, synapses, and cortical areas, supporting both biological realism and high-performance simulation.

---

## Architecture
- **Centralized State:** The ConnectomeManager maintains all neuron and synapse data structures, using a Structure of Arrays (SoA) approach for memory efficiency and fast access.
- **Thread Safety:** All operations are thread-safe using internal locks, supporting concurrent simulation and API access.
- **Hierarchical FCL:** Integrates a hierarchical Fire Candidate List (FCL) manager for tracking neuron firing events, supporting both global and area-specific queries, and rolling window analysis.
- **Synapse Management:** Delegates all synapse storage and plasticity logic to a dedicated SynapseManager, supporting both static and plastic synapses.
- **Cortical Area Abstraction:** Each cortical area is represented as a `CorticalArea` dataclass, with unique IDs, names, types, dimensions, and properties.
- **Position Tracking:** Efficiently tracks neuron positions using bitmaps and hierarchical lookup tables for fast spatial queries, even in extremely large or sparse areas.
- **Serialization:** Supports saving and restoring the entire brain state (neurons, synapses, areas) to/from disk in a highly compressed format.

---

## Key Features
- **CRUD Operations:**
  - Add, query, and delete neurons and synapses.
  - Create, query, and manage cortical areas.
- **Efficient Queries:**
  - Query neurons by area, position, threshold, membrane potential, last fired, or multiple combined criteria.
  - Retrieve neurons with the highest membrane potential, or statistical summaries.
- **Batch Operations:**
  - Batch creation of neurons for rapid initialization.
  - Batch updates for membrane potential decay.
- **FCL Sampling:**
  - Maintains a rolling window of firing events for temporal analysis and live visualization.
- **Serialization/Deserialization:**
  - Save the entire connectome (including all neurons, synapses, and areas) to disk.
  - Restore a preserved connectome to resume simulation or analysis.
- **Extensible:**
  - Designed for future expansion (e.g., region management, memory areas, advanced plasticity).

---

## Example Usage
```python
from feagi.bdu.connectome_manager import connectome

# Add a cortical area
area = connectome.add_cortical_area(
    area_id=1,
    name="Visual Cortex",
    area_type="sensory",
    dimensions=(100, 100, 10),
    position=(0, 0, 0)
)

# Create a neuron
neuron_id = connectome.create_neuron(
    area_id=1,
    position=(10, 10, 2),
    threshold=1.0,
    refractory_period=5,
    decay_rate=0.9,
    resting_potential=0.0
)

# Create a synapse
connectome.create_synapse(
    pre_neuron_id=neuron_id,
    post_neuron_id=another_neuron_id,
    weight=0.5
)

# Query neurons by area
neurons = connectome.get_neurons_by_area(1)

# Serialize brain state
connectome.serialize_brain_state("brain_state.npz")

# Restore brain state
connectome.deserialize_brain_state("brain_state.npz")
```

---

## Advanced Topics
- **Fire Candidate List (FCL):**
  - The FCL manager tracks which neurons have fired in each timestep, supporting efficient event-driven simulation and visualization.
  - Supports both global and per-area FCL queries, and a rolling window for temporal analysis.
- **Spatial Queries:**
  - Efficiently query neurons by 3D position, ranges, or area, using optimized data structures for both dense and sparse connectomes.
- **Plasticity:**
  - SynapseManager supports both static and plastic synapses, with configurable plasticity coefficients and decay.
- **Extensibility:**
  - The architecture is designed for future features such as region management, memory areas, and advanced learning rules.

---

## Best Practices
- Use the global `connectome` singleton for all connectome operations to ensure consistent state.
- For high-performance or batch operations, use the provided batch methods.
- Always serialize the brain state before shutdown to preserve training progress.

---

## See Also
- `feagi.bdu.synapse_manager.SynapseManager` for synapse logic
- `feagi.npu.fcl_manager.HierarchicalFCL` for FCL logic
- FEAGI API documentation for endpoints interacting with the connectome 