# FEAGI Hybrid CPU/GPU Architecture

*Last Updated: May 15, 2025*

## Overview

This document outlines the architecture for FEAGI that optimizes for specific GPU-accelerated operations while maintaining CPU efficiency for management tasks, providing RTOS compatibility, and enabling a clean Rust migration path.

## Core Architecture Principles

1. **Targeted Acceleration**: GPU acceleration focused on computation-intensive operations (neuron firing, synaptogenesis)
2. **Hybrid Memory Model**: Clear separation between management data (CPU) and computational data (GPU)
3. **Lock-Free Synchronization**: Minimize blocking operations for RTOS compatibility
4. **Ownership-Based Design**: Structure for Rust's ownership system from the beginning
5. **Strict API Layering**: API endpoints must never directly access lower-level components

## API Architectural Constraints

### Strict Service Layer Separation

```
┌─────────────────────┐
│ REST API Endpoints  │ ◄── Only communicates with CoreAPIService
└────────┬────────────┘
         │
         ▼
┌─────────────────────┐
│   CoreAPIService    │ ◄── All business logic and error handling
└────────┬────────────┘
         │
         ▼
┌─────────────────────┐
│ ConnectomeManager   │ ◄── Data access and manipulation
└─────────────────────┘
```

**CRITICAL RULE**: API endpoints must NEVER directly access the ConnectomeManager. All access to connectome data must go through the CoreAPIService layer.

Benefits of this architectural constraint:
- **Single Point of Entry**: All connectome operations funnel through a controlled interface
- **Consistent Error Handling**: Centralized error processing and validation
- **Proper Transaction Management**: Operations can be organized into atomic units
- **Business Logic Encapsulation**: Logic remains in the service layer where it belongs
- **Improved Testability**: Easier to mock services for endpoint testing
- **Enhanced Security**: CoreAPIService can implement access controls

## Key Process Integration

### Process Interaction Flow

```
┌──────────────────┐      ┌───────────────────┐      ┌────────────────┐
│                  │      │                   │      │                │
│  REST API        │─────▶│  CoreAPIService   │─────▶│ ConnectomeManager
│  Endpoints       │      │                   │      │                │
│                  │      │                   │      │                │
└──────────────────┘      └───────────────────┘      └────────┬───────┘
                                                            │
┌──────────────────┐      ┌───────────────────┐            │
│                  │      │                   │            │
│  FCL Manager     │◀────▶│  Burst Engine     │◀───────────┘
│  (Roaring Bitmap)│      │                   │
│                  │      │                   │
└──────────────────┘      └───────────────────┘

```

The FCL Manager maintains a Fire Candidate List using a Roaring Bitmap for memory-efficient storage. The Burst Engine handles the neuron firing process, accessing membrane potentials from the ConnectomeManager, updating them, and assessing if neurons meet firing criteria.

### Process Responsibilities

1. **FCL_Manager**: 
   - Maintains the Fire Candidate List using Roaring bitmaps
   - GPU-accelerated for efficient set operations on large neuron populations
   - Provides fast lookup for neurons that need processing

2. **Burst Engine**: 
   - Processes firing neurons from FCL
   - Calculates membrane potential changes in connected neurons
   - Applies thresholds, leak, and refractory period logic
   - Primary target for GPU acceleration

3. **Membrane Potential Updates**:
   - Fast, zero-copy transfer of updated membrane potentials
   - Atomic operations for synchronization
   - Batched updates for performance

## Operation-Specific Optimization

### CPU-Based Operations
- Cortical area CRUD operations
- Genome management
- Configuration and setup
- API handling
- Monitoring and diagnostics

### GPU-Accelerated Operations
- FCL processing with Roaring bitmaps
- Neuron membrane potential updates
- Activation propagation
- Synapse weight adjustments
- Possibly synaptogenesis (during development)

## Hybrid Memory Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Management Layer (CPU)                   │
│                                                             │
│  • Cortical area management                                 │
│  • Configuration/metadata                                   │
│  • Runtime control                                          │
│  • API handling                                             │
└─────────────────────────────┬───────────────────────────────┘
                              │
                              │ Transfer Layer
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                Computational Layer (CPU/GPU)                │
│                                                             │
│  • FCL management (GPU)                                     │
│  • Neuron firing (GPU)                                      │
│  • Synaptic signal propagation (GPU)                        │
│  • Membrane potential updates (GPU)                         │
│  • Potentially synaptogenesis (GPU)                         │
└─────────────────────────────────────────────────────────────┘
```

### Memory Management Strategy

- **CPU-Owned Data**:
  - Cortical area definitions
  - Brain structure metadata
  - Configuration parameters
  - Genome data
  
- **GPU-Optimized Data**:
  - Roaring bitmaps for FCL
  - Neuron activation states
  - Membrane potentials
  - Synaptic weights
  - Firing candidate lists

## WebGPU Integration

### WebGPU-Compatible Design

- Roaring bitmap operations implemented in WGSL
- Buffer-based design compatible with WebGPU's binding model
- WGSL compute shaders for cross-platform support
- Workgroup-optimized algorithms for diverse hardware
- Explicit pipeline state management

```
┌──────────────────────┐    ┌───────────────────────┐
│                      │    │                       │
│  WebGPU Compute      │    │  WebGPU Storage       │
│  Pipeline            │◄───┤  Buffers              │
│                      │    │  (FCL, Membranes)     │
└──────────────┬───────┘    └───────────▲───────────┘
               │                        │
               │                        │
               ▼                        │
┌──────────────────────┐    ┌───────────┴───────────┐
│                      │    │                       │
│  CPU Shadow          ├───►│  Buffer               │
│  Arrays              │    │  Management           │
│                      │    │                       │
└──────────────────────┘    └───────────────────────┘
```

## Related Documentation
- [System Overview](arch-system-overview.md)
- [IPC Architecture](arch-ipc.md)
- [State Management](arch-state-management.md) 