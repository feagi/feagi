# FEAGI Architecture Overview

This document provides a high-level overview of FEAGI's architecture, describing how major components interact to form a cohesive system for neuromorphic artificial intelligence.

## System Architecture

FEAGI (Flexible & Extensible Artificial General Intelligence) is built as a modular, distributed system with clearly defined boundaries between components. The architecture emphasizes:

- **Modularity**: Components that can be developed, tested, and upgraded independently
- **Scalability**: Ability to scale from small embedded systems to large distributed deployments
- **Performance**: Optimization for neural processing across various hardware configurations
- **Flexibility**: Support for multiple protocols, interfaces, and computational backends

## Core Components

```
┌──────────────────────────────────────────────────────────────┐
│                        FEAGI Core                            │
├──────────┬───────────┬───────────┬───────────┬───────────────┤
│   API    │    BDU    │    NPU    │    PNS    │  Core System  │
│  Layer   │           │           │           │               │
└────┬─────┴─────┬─────┴─────┬─────┴─────┬─────┴───────┬───────┘
     │           │           │           │             │
     ▼           ▼           ▼           ▼             ▼
┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌───────────┐
│ Agents/ │ │ Genome/ │ │ Neural  │ │ Sensory │ │ Resource  │
│ Clients │ │Connectome│ │Processing│ │ I/O    │ │Management │
└─────────┘ └─────────┘ └─────────┘ └─────────┘ └───────────┘
```

### API Layer

The API layer provides interfaces for communication with FEAGI, including:

- REST API for configuration and monitoring
- ZeroMQ messaging for high-performance data exchange
- Protocol adapters for various communication patterns
- Authentication and security enforcement

### Brain Development Unit (BDU)

The BDU manages the structure of the artificial neural network:

- Genome management (blueprint for neural architecture)
- Connectome generation and modification
- Cortical area definition and configuration
- Synaptogenesis rules and neural connectivity

### Neural Processing Unit (NPU)

The NPU executes the neural simulation:

- Neural activity computation
- Membrane potential modeling
- Spike propagation
- Learning and plasticity mechanisms
- Hardware-accelerated processing

### Peripheral Nervous System (PNS)

The PNS handles interfacing with the outside world:

- Sensory data encoding (vision, audio, etc.)
- Motor output decoding
- Agent embodiment interfaces
- I/O protocol management

### Core System

The Core system provides foundational infrastructure:

- Process and resource management
- State tracking and synchronization
- Shared memory interfaces
- Backend abstractions (CPU, GPU, etc.)
- Security and authentication

## Data Flow

The primary data flows in FEAGI are:

1. **Configuration Flow**: External configuration → API → Component configuration
2. **Development Flow**: BDU → Genome modifications → Connectome updates → NPU
3. **Processing Flow**: Sensory input → PNS → NPU → PNS → Motor output
4. **State Flow**: Components → Core state manager → API → External monitoring

## Genome-Connectome Architecture

The Genome-Connectome architecture separates the neural network's structure definition from its runtime implementation:

- **Genome**: The specification or blueprint for the neural network structure
- **Connectome**: The actual implementation of the neural network in memory

This separation enables:
- Versioning and tracking of neural architectures
- Dynamic modification of network structure
- Efficient memory representation of active networks

## Computational Model

FEAGI employs several computational approaches:

- **Spiking Neural Networks**: Temporal information encoding using discrete spikes
- **Fire Candidate Lists (FCLs)**: Efficient tracking of active neurons
- **Structure of Arrays (SoA)**: Data layout optimized for parallel processing
- **Memory-Mapped State**: High-performance state tracking with near-zero overhead

## Deployment Architecture

FEAGI can be deployed in several configurations:

- **Standalone**: All components run in a single process
- **Distributed**: Components distributed across multiple processes or machines
- **Embedded**: Reduced functionality for resource-constrained environments
- **Hybrid**: Mix of local and remote components

## Integration Architecture

FEAGI integrates with external systems through:

- **FEAGI Bytes**: Lightweight protocol for sensorimotor data exchange
- **FEAGI Connector**: Client library for agent development
- **WebSocket Interface**: Browser-based visualization and interaction
- **REST API**: Configuration, monitoring, and integration with external tools

## Further Reading

For more detailed information on specific components, see:
- [System Design Principles](arch-design-principles.md)
- [ZeroMQ Architecture](arch-zmq.md)
- [Genome-Connectome Architecture](arch-genome-connectome.md)
- [GPU Architecture](arch-gpu.md)
- [IPC Architecture](arch-ipc.md)
- [State Management](arch-state-management.md) 