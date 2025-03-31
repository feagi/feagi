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

#### Synapses
A synapse 


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
