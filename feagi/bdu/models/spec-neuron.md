# NeuronModule Documentation

## Overview
The **Neuron Module** manages neuron CRUD operations including the creation of neurons, reading neuron properties,
updating neuron parameters, and deleting neurons.

The **NeuronArrayManager** class maintains global neuron storage, while **Neuron** handles specific neuron instantiation
and interactions.

## Data Structures

### **1. `NeuronArrayManager` (Global Configuration)**
This class maintains global neuron storage and handles dynamic memory allocation.

- **`GLOBAL_NEURON_ID`**: Tracks the next available neuron ID.
- **`MAX_NEURONS`**: Maximum number of neurons allocated.
- **`MEMORY_NEURON_CAPACITY`**: Initial capacity for memory cortical areas.
- **`global_neuron_array`**: Stores all neurons (NumPy, CuPy, or shared memory).
- **`memory_neuron_array`**: Stores dynamically allocated memory neurons.
- **`lock`**: Ensures thread safety in multi-threaded environments.
- **`shared_mem`**: Enables shared memory for multi-process execution.


### **1. `NeuronArrayManager.initialize()`**
- Determines the backend (`numpy`, `cupy`, `shared_memory`) via `BackendConfig`.
- Allocates `global_neuron_array` and `memory_neuron_array` accordingly.

### **2. `NeuronArrayManager.allocate_neurons(count)`**
- Thread-safe method to allocate neuron IDs.

### **3. `NeuronArrayManager.expand_memory_neurons(new_capacity)`**
- Dynamically expands memory cortical neuron storage.
