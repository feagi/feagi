# FEAGI Hybrid CPU/GPU Architecture

This document outlines a proposed architecture for FEAGI that optimizes for specific GPU-accelerated operations while maintaining CPU efficiency for management tasks, providing RTOS compatibility, and enabling a clean Rust migration path.

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

This constraint is mandatory for all new code and existing code should be refactored to comply with this pattern.

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

The FCL Manager maintains a Fire Candidate List using a Roaring Bitmap for memory-efficient storage. The Burst Engine handles the neuron firing process, accessing membrane potentials from the ConnectomeManager, updating them, and assessing if neurons meet firing criteria. This process must happen through the CoreAPIService to maintain architectural integrity.

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

4. **Core API Service & ConnectomeManager**:
   - Receives membrane potential updates from Burst Engine
   - Updates connectome data structures accordingly
   - Maintains consistent state across the system

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

- **Transfer Strategy**:
  - Zero-copy for FCL and membrane potentials when possible
  - Bulk synchronization at specific sync points
  - Explicit transfer boundaries with versioning
  - Atomic operations for state consistency

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

### WebGPU-Specific Optimizations

- **FCL Processing Kernels**:
  - Specialized WGSL shaders for Roaring bitmap operations
  - Batch processing of firing neurons
  - Parallel computation of membrane potential updates

- **Memory Coherence**:
  - Explicit barriers to ensure visibility of writes
  - Double-buffering for read/write operations
  - Atomic counters for synchronization points

- **Workgroup Optimization**:
  - Dynamic sizing based on hardware capabilities
  - Shared memory usage for frequently accessed FCL segments
  - Warp/wavefront-aware algorithms for SIMD execution

## Synchronization Mechanisms

### Bounded-Wait Operations

- Atomic operations for coordination between FCL_Manager and Burst Engine
- Lock-free updates of membrane potentials
- Explicit sync points with timeout guarantees for RTOS
- Versioned data structures for detection of update conflicts
- Bounded execution guarantees on all operations

### Concurrent Data Structures

- Lock-free queues for FCL updates
- Wait-free read operations for real-time guarantees
- Single-producer, multiple-consumer patterns for scaling
- Priority inheritance for critical sections

## API Layer Architecture

### Hybrid API Design

- Management operations processed directly on CPU
- FCL and computational operations batched for GPU
- Clear boundaries between management and computational APIs
- Explicit transaction model for multi-step operations

### Performance-Optimized Interfaces

```
┌───────────────────────────┐
│                           │
│  External API Interface   │
│  (CPU Processing)         │
│                           │
└───────────┬───────────────┘
            │
            │ Management Operations
            ▼
┌───────────────────────────┐        ┌───────────────────────┐
│                           │        │                       │
│  ConnectomeManager        │◄──────►│  FCL_Manager &        │
│  (CPU-based)              │        │  Burst Engine (GPU)   │
│                           │        │                       │
└───────────────────────────┘        └───────────────────────┘
```

- CPU-efficient APIs for management operations
- Batch processing for GPU-bound operations on FCL and membrane potentials
- Zero-copy interfaces for high-throughput data exchange
- Clear ownership semantics for memory resources

## RTOS Compatibility

### Deterministic Operations

- CPU-side operations with bounded execution time
- Asynchronous GPU dispatching with timeout controls
- Prioritized scheduling of FCL processing
- Maximum latency guarantees for membrane potential updates

### Resource Management

- Preallocated memory pools for FCL Roaring bitmaps
- Memory fences with explicit ordering semantics
- Cached architecture status for bounded-time responses
- Timeout mechanisms for all blocking operations

## Rust Migration Path

### Progressive Migration Strategy

1. **Core Type Definitions**: Start with shared data structures (FCL, membrane potentials)
2. **Rust Management Layer**: Replace CPU-side components with Rust
3. **Computational Kernel Migration**: Port WebGPU compute shaders for FCL processing
4. **Full API Implementation**: Complete Rust implementation of public APIs

### FFI Boundary Design

```
┌────────────────────┐      ┌────────────────────┐
│                    │      │                    │
│  Python Frontend   │◄────►│  Rust Management   │
│  (API, Web, UI)    │      │  Layer             │
│                    │      │                    │
└────────────────────┘      └──────────┬─────────┘
                                       │
                                       │
                                       ▼
                            ┌────────────────────┐
                            │                    │
                            │  Rust+WGPU         │
                            │  FCL & Burst       │
                            │                    │
                            └────────────────────┘
```

- C-compatible interfaces between language boundaries
- Roaring bitmap implementations usable from both Rust and Python
- Clear data ownership at FFI boundaries
- Progressive component replacement
- Parallel implementations during transition

## Implementation Priorities

1. **GPU-Accelerate FCL Operations**: Implement Roaring bitmap operations in WebGPU
2. **Implement Membrane Potential Update Flow**: Create efficient CPU-GPU communication channel
3. **Build Zero-Copy Transfer Layer**: For FCL and membrane potential data
4. **Optimize Memory Layouts**: Ensure data structures are aligned for both CPU and GPU access
5. **Define Clear Process Boundaries**: Ensure clean separation between FCL_Manager, Burst Engine, and API Service

## Benchmarking and Validation

- FCL processing performance metrics
- Membrane potential update latency measurements
- Comparative benchmarking with/without GPU acceleration
- RTOS compliance validation for critical paths
- Memory usage optimization for Roaring bitmaps
- Cross-platform compatibility testing

## Fallback Strategy

- Pure CPU implementation for FCL processing and burst engine
- Capability detection for various acceleration levels
- Progressive feature utilization based on hardware support
- Configuration-driven feature enabling/disabling

## Refactoring Plan and Implementation Roadmap

### Phase 1: ConnectomeManager and CoreAPIService Integration (1-2 weeks)

1. **API Boundary Definition**
   - Define clear interfaces between CoreAPIService and ConnectomeManager
   - Create proper type definitions for all shared data structures
   - Document API contracts and error handling expectations

2. **ConnectomeManager Refactoring**
   - Add missing methods required by CoreAPIService (e.g., get_area_for_neuron)
   - Implement batch operations for all performance-critical paths
   - Add proper error handling and validation

3. **CoreAPIService Adaptation**
   - Update all direct attribute access to use proper API methods
   - Convert string/int ID handling to be consistent
   - Implement proper error handling and validation
   - Add comprehensive logging

### Phase 2: Memory Management Optimization (2-3 weeks)

1. **Shared Memory Architecture**
   - Implement zero-copy shared memory regions for neuron states
   - Create memory-mapped interfaces for FCL and membrane potentials
   - Set up proper locking and synchronization primitives

2. **Batch Operation Implementation**
   - Refactor single-neuron operations to use batch processing
   - Implement vectorized operations where possible
   - Add support for incremental result streaming

3. **Memory Layout Optimization**
   - Reorganize data structures for optimal cache usage
   - Align memory for SIMD operations
   - Implement memory pooling for neuron/synapse structures

### Phase 3: FCL and Burst Engine Acceleration (3-4 weeks)

1. **WebGPU Integration Setup**
   - Set up WebGPU device and adapter initialization
   - Create buffer management system
   - Implement capability detection and fallbacks

2. **FCL GPU Acceleration**
   - Port Roaring bitmap operations to WGSL
   - Implement parallel set operations
   - Create efficient transfer mechanisms between CPU and GPU

3. **Burst Engine Optimization**
   - Implement neuron firing computations in WebGPU
   - Create efficient synaptic signal propagation
   - Set up membrane potential updates with proper synchronization

### Phase 4: Testing and Optimization (2-3 weeks)

1. **Benchmarking Framework**
   - Set up performance measurement infrastructure
   - Create comparative benchmarks (CPU vs. GPU)
   - Implement automated regression testing

2. **Optimization Rounds**
   - Identify and address performance bottlenecks
   - Fine-tune WebGPU workgroups and memory usage
   - Optimize synchronization points

3. **Cross-Platform Validation**
   - Test on various GPU architectures
   - Validate on browsers and native platforms
   - Ensure fallbacks work properly

### Phase 5: Documentation and Refactoring (1-2 weeks)

1. **Code Cleanup**
   - Remove deprecated code paths
   - Ensure consistent naming and patterns
   - Address any technical debt

2. **Documentation**
   - Create architectural documentation
   - Document all APIs and interfaces
   - Add usage examples and benchmarks

3. **Final Integration**
   - Ensure all components work together seamlessly
   - Final performance validation
   - Release preparation 