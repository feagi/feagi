# FEAGI Design Principles

This document outlines the core architectural design principles and strategies for FEAGI (Framework for Evolutionary Artificial General Intelligence).

## Core Architecture Principles

### Modular Design

FEAGI is built with a modular architecture that separates concerns and enables independent development:

- **Core Module**: Resource management, configuration, and backend selection
- **Neural Processing Unit (NPU)**: Handles neuron firing dynamics and simulation
- **Brain Development Unit (BDU)**: Manages brain structure and connectivity
- **Peripheral Nervous System (PNS)**: Handles sensorimotor I/O
- **API Layer**: Provides REST and ZeroMQ interfaces

This modular approach enables:
- Independent development of components
- Easier testing and maintenance
- Flexibility in deployment configurations
- Clear separation of compute-intensive vs. async operations

### Multi-Backend Support

FEAGI is designed to run efficiently on different hardware configurations:

- **CPU Backend**: Optimized for systems without dedicated GPUs
- **GPU Backend**: Leverages WebGPU for cross-platform acceleration
- **Dynamic Backend Selection**: Chooses optimal backend based on available hardware

### Performance-First Design

Performance considerations are built into every aspect of FEAGI:

- **Structure of Arrays (SoA)**: Optimized data layout for SIMD and GPU processing
- **Sparse Processing**: Only active neurons and synapses are processed
- **Memory Optimization**: Cache-friendly access patterns and minimal data transfers
- **Workload Distribution**: Dynamic balancing across available compute resources

## Data Structure Principles

### Global Neuron Array (GNA)

The GNA uses a Structure of Arrays (SoA) approach for optimal performance:

- **SIMD Optimization**: Enables parallel processing of neuron properties
- **Memory Coalescing**: Adjacent memory access for GPU threads
- **Cache Efficiency**: Maximizes CPU cache utilization
- **Selective Updates**: Reduces CPU-GPU transfer overhead

### Fire Candidate List (FCL)

The FCL implements a temporal model with specific roles:

- **FCL[t]**: Current timestep (accumulation phase)
- **FCL[t-1]**: Previous timestep (firing phase)
- **FCL[t-2...t-window]**: Historical record for pattern tracking

### Synapse Management

Synapse data structures are optimized for:

- **Fast Lookups**: O(1) access to neuron connections
- **Memory Efficiency**: Compact representation for billions of synapses
- **Parallel Processing**: Efficient batch operations on GPU

## Process Principles

### Two-Phase Neural Processing

Neural simulation follows a two-phase approach:

1. **Firing Phase**: Process neurons in FCL[t-1], queue membrane potential updates
2. **Accumulation Phase**: Apply updates, determine new firing neurons for FCL[t]

### Decoupled Visualization

Visualization is decoupled from simulation:

- **Sampling Strategies**: Ratio-based or frequency-based sampling
- **Data Transformation**: Conversion to visualization-friendly formats
- **ZeroMQ Transport**: Efficient binary messaging to visualization clients

## Implementation Strategies

### Path to Rust Migration

FEAGI is designed with future Rust migration in mind:

| Best Practice | Implementation Approach |
|---------------|-------------------------|
| Type Hints | Use Python type annotations for all functions |
| Avoid Deep OOP | Prefer composition over inheritance |
| Functional Style | Use immutable data and pure functions where possible |
| Fixed-Size Arrays | Use NumPy arrays instead of dynamic lists |
| Explicit State | Avoid global state, pass state explicitly |
| PyO3 Integration | Design for gradual migration to Rust |

### Security Considerations

Security is built into FEAGI's design:

- **Authentication**: Support for secure API and ZMQ communication
- **Optional Encryption**: Available for sensitive deployments
- **Quality of Service**: Priority assignment for critical processes

## Project Structure

```
./
├── feagi/                  # Main package
│   ├── core/               # Core functionality and resource management
│   ├── bdu/                # Brain Developmental Unit
│   ├── npu/                # Neural Processing Unit
│   ├── evo/                # Evolutionary Unit
│   ├── pns/                # Peripheral Nervous System
│   └── viz/                # Visualization data transformation
├── tests/                  # Unit, integration, and functional tests
├── docs/                   # Documentation
├── examples/               # Example scripts
└── requirements.txt        # Dependencies
```

## Memory Management Strategy

FEAGI employs sophisticated memory management:

1. **Slotted Allocation System**:
   - Pre-allocated arrays with fixed-size slots
   - Efficient slot reuse for neuron creation/deletion

2. **Dynamic Growth Strategy**:
   - Geometric growth factor for arrays
   - Background compaction for efficient expansion

3. **Sparse Activation Tracking**:
   - Compact representation of active neurons
   - Periodic rebuilding based on activation patterns

## Performance Optimizations

Key performance optimizations include:

1. **Sparse Processing**:
   - Processing only active components
   - Early termination for inactive areas

2. **Memory Access Patterns**:
   - Coalesced memory access for GPU
   - Cache-friendly organization for CPU

3. **Workload Distribution**:
   - Dynamic kernel selection
   - Grouping by cortical area for locality

These principles guide FEAGI's development to ensure it remains performant, maintainable, and adaptable to future needs.
