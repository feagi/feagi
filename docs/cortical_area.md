# Cortical Area Module Documentation

## Overview
The **Cortical Area Module** manages CRUD operations against cortical area objects including the creation of cortical 
areas, reading cortical properties, updating cortical properties, and deleting them. Creation of a cortical area entails 
the initialization of neuron properties which will be inherited by neurons during neuron creation in the given cortical 
area.

This module also enables multi-cortical area actions where the above mentioned CRUD operations can be performed against 
multiple cortical area at the same time.


## Types
There are 4 types of cortical areas:
1. Input Processing Unit (IPU)
2. Output Processing Unit (OPU)
3. Interconnect
4. Memory

`templates.py` captures the unique properties for each cortical area type and subtype. 

### Input Processing Unit (IPU) Area
IPU cortical areas are synonymous to brain areas representing sensory neurons and responsible for feeding sensory information to the rest of the 
brain.

### Output Processing Unit (OPU) Area
OPU cortical areas are synonymous to brain areas representing motor neurons and responsible for taking motor commands from the brain to 
peripherals.

### Interconnect Area
Interconnect cortical areas are generic and responsible for connecting various areas of the brain together.

### Memory Area
Memory area is a special cortical area without any specific 3D topology and capable of storing information in the form of newly 
created neurons.


## Data Structures


### **1. `CorticalArea` (Individual Cortical Representation)**
Each cortical area is instantiated with a **cortical ID, type, and spatial dimensions**. It manages neurons within its assigned voxel space.

- **`cortical_id`**: Unique ID for the cortical area.
- **`cortical_type`**: Type of cortical area (e.g., `interconnect`, `memory`, `ipu`, `opu`).
- **`width, height, depth`**: Defines voxel space dimensions.
- **`neurons_per_voxel`**: Number of neurons per voxel.
- **`total_neurons`**: Total neurons assigned to the cortical area.
- **`neuron_start_idx, neuron_end_idx`**: Tracks assigned neurons in `global_neuron_array`.



Cortical area properties:
- label ()
- FEAGI ID (str) [Limited to 6 characters]
- local ID (int32)
- type (int8)  [IPU, OPU, Interconnect, Memory]
- brain region id (int32)
- dimensions (int32, int32, int32)
- 3D location (int32, int32, int32)
- 2D location (int32, int32)
- neuron block capacity (int32) 
- neuron firing threshold
- neuron firing threshold increment (int8, int8, int8)
- neuron firing threshold limit
- neuron degeneracy constant (int32) 
- neuron psp (float)
- neuron psp uniformity (bool)
- neuron psp max (int32)
- neuron refractory period (int16)
- neuron refractory period overwrite
- neuron leak coefficient 
- neuron leak variability 
- neuron consecutive fire count
- neuron snooze period
- neuron excitability
- neuron membrane potential accumulation (bool)
- neuron membrane potential driven post synaptic potential
- neuron synapse attractivity (int32)
- memory neuron temporal depth
- memory neuron initial lifespan
- memory neuron lifespan growth rate (int32)
- memory neuron longterm conversion threshold