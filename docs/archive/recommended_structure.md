# FEAGI Recommended Project Structure

This document outlines the recommended organizational structure for the FEAGI codebase, designed to improve maintainability, separation of concerns, and adherence to good architectural practices.

## High-Level Structure

```
feagi/
├── __init__.py               # Package initialization
├── config/                   # Configuration management
├── core/                     # Core framework components
├── npu/                      # Neural Processing Unit
├── bdu/                      # Brain Developmental Unit
├── pns/                      # Peripheral Nervous System
├── viz/                      # Visualization components
├── api/                      # API interfaces
├── evo/                      # Evolutionary Unit
├── utils/                    # Shared utilities
└── tests/                    # Tests for all modules
```

## Detailed Structure

### 1. Core Module (`feagi/core/`)

The core module contains foundational components used throughout FEAGI:

```
core/
├── __init__.py               # Module initialization
├── resource_mgr.py           # Resource management
├── backend/                  # Backend abstraction & implementation
│   ├── __init__.py
│   ├── interface.py          # Common interface definition
│   ├── cpu.py                # CPU backend implementation
│   ├── gpu_webgpu.py         # WebGPU implementation
│   ├── gpu_cuda.py           # CUDA-specific implementation
│   └── gpu_metal.py          # Metal-specific implementation
├── process_mgr/              # Process management
│   ├── __init__.py
│   ├── controller.py         # Process controller
│   ├── monitor.py            # Process monitoring
│   └── lifecycle.py          # Process lifecycle management
├── ipc/                      # Inter-process communication
│   ├── __init__.py
│   ├── shared_memory.py      # Shared memory implementation
│   ├── message_queue.py      # Message queue implementation
│   └── zmq/                  # ZeroMQ implementation
│       ├── __init__.py
│       ├── server.py         # ZMQ server implementation
│       └── client.py         # ZMQ client implementation
├── data/                     # Core data structures
│   ├── __init__.py
│   ├── neuron.py             # Neuron data structures
│   ├── cortical_area.py      # Cortical area data structures
│   └── synapse.py            # Synapse data structures
├── storage/                  # Persistence and serialization
│   ├── __init__.py
│   ├── serializer.py         # Data serialization
│   └── checkpoint.py         # Checkpoint management
└── security/                 # Security components
    ├── __init__.py
    └── auth.py               # Authentication & authorization
```

### 2. Neural Processing Unit (`feagi/npu/`)

The NPU handles neural simulation and processing:

```
npu/
├── __init__.py               # Module initialization
├── burst_engine.py           # Core burst engine
├── fcl_manager.py            # Fire Candidate List management
├── connectome/               # Connectome management
│   ├── __init__.py
│   ├── manager.py            # Connectome manager
│   └── operations.py         # Connectome operations
├── learning/                 # Learning and plasticity
│   ├── __init__.py
│   ├── stdp.py               # Spike-Timing-Dependent Plasticity
│   ├── homeostatic.py        # Homeostatic scaling
│   └── consolidation.py      # Memory consolidation
└── gpu/                      # GPU-specific implementations
    ├── __init__.py
    ├── compute_shaders.py    # Compute shaders for WebGPU
    └── kernels/              # GPU kernel implementations
        ├── __init__.py
        ├── neuron_update.py  # Neuron update kernels
        └── synapse_update.py # Synapse update kernels
```

### 3. Brain Developmental Unit (`feagi/bdu/`)

The BDU manages brain structure development:

```
bdu/
├── __init__.py               # Module initialization
├── stem_cell_manager.py      # Stem cell management
├── growth/                   # Neural growth
│   ├── __init__.py
│   ├── neurogenesis.py       # Neuron creation
│   └── synaptogenesis.py     # Synapse creation
└── pruning/                  # Neural pruning
    ├── __init__.py
    └── synaptic_pruning.py   # Synapse pruning
```

### 4. Peripheral Nervous System (`feagi/pns/`)

The PNS handles external I/O:

```
pns/
├── __init__.py               # Module initialization
├── message_broker.py         # Message brokering
├── adapters/                 # I/O adapters
│   ├── __init__.py
│   ├── vision.py             # Visual input
│   ├── audio.py              # Audio I/O
│   └── motor.py              # Motor output
└── protocols/                # Communication protocols
    ├── __init__.py
    └── serialization.py      # Data serialization
```

### 5. API (`feagi/api/`)

The API module provides interfaces for external communication:

```
api/
├── __init__.py               # Module initialization
├── server.py                 # API server
├── schema/                   # API schemas
│   ├── __init__.py
│   ├── requests.py           # Request models
│   └── responses.py          # Response models
└── routes/                   # API routes
    ├── __init__.py
    ├── v1/                   # API v1
    │   ├── __init__.py
    │   ├── cortical_areas.py # Cortical area endpoints
    │   └── runtime.py        # Runtime endpoints
    └── v2/                   # API v2
        ├── __init__.py
        └── ...
```

### 6. Evolutionary Unit (`feagi/evo/`)

The EVO module handles evolutionary algorithms:

```
evo/
├── __init__.py               # Module initialization
├── population.py             # Population management
├── selection.py              # Selection algorithms
├── mutation.py               # Mutation operations
└── fitness/                  # Fitness evaluation
    ├── __init__.py
    └── evaluators.py         # Fitness evaluators
```

### 7. Visualization (`feagi/viz/`)

The VIZ module handles visualization components:

```
viz/
├── __init__.py               # Module initialization
├── renderer.py               # Core rendering
└── data_preparation.py       # Visualization data preparation
```

### 8. Utilities (`feagi/utils/`)

General utilities used across modules:

```
utils/
├── __init__.py               # Module initialization
├── logger.py                 # Logging utilities
├── errors.py                 # Error definitions
├── data_structures.py        # Common data structures
├── benchmarking.py           # Performance benchmarking
├── memory_profiler.py        # Memory profiling
└── version_checker.py        # Version checking
```

### 9. Configuration (`feagi/config/`)

Configuration management:

```
config/
├── __init__.py               # Module initialization
├── defaults.py               # Default configuration
├── validation.py             # Configuration validation
└── loader.py                 # Configuration loading
```

### 10. Tests (`feagi/tests/`)

Tests for all modules:

```
tests/
├── __init__.py               # Module initialization
├── fixtures/                 # Test fixtures
│   ├── __init__.py
│   └── common.py             # Common fixtures
├── unit/                     # Unit tests
│   ├── __init__.py
│   ├── test_core/            # Tests for core module
│   ├── test_npu/             # Tests for NPU
│   └── ...                   # Other unit tests
└── integration/              # Integration tests
    ├── __init__.py
    └── ...                   # Integration test modules
```

## Migration Strategy

To transition from the current project structure to the recommended one:

1. **Identify Duplicated Code**
   - Resolve conflicts between `feagi/zmq/` and `feagi/core/zmq/`
   - Consolidate API implementations

2. **Module Reorganization**
   - Move related functionality into appropriate modules
   - Ensure proper imports and dependencies

3. **Implement Missing Components**
   - Add configuration module
   - Enhance backend abstraction

4. **Update Documentation**
   - Update all import paths
   - Document module purposes and interfaces

5. **Update Tests**
   - Reorganize tests to match new structure
   - Expand test coverage

## Implementation Priorities

1. Core module reorganization
2. Configuration module implementation
3. Backend abstraction layer
4. ZMQ consolidation
5. Test restructuring
