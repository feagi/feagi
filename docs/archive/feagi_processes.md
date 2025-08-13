# FEAGI Process Architecture

*Last Updated: January 15, 2025*

This document outlines the system workflow and architecture for FEAGI (Framework for Evolutionary Artificial General Intelligence), with a focus on **Rust/RTOS compatible** process management, performance, and quality of service.

## Overview

FEAGI employs a **task-based architecture** designed for high performance, mission-critical reliability, and seamless Rust migration. The architecture is built on four principles:

1. **Direct Task Spawning**: Critical computation uses async tasks instead of subprocess boundaries
2. **Singleton Pattern**: Mission-critical components use singleton instances for consistency
3. **Memory-Mapped State**: Shared memory for zero-copy data access across tasks
4. **Priority-based Resource Allocation**: System resources allocated based on task criticality

## Task Types and Priorities

FEAGI tasks are categorized into three priority levels:

### Priority 1 (Critical - Real-time)
These tasks handle the core neural simulation and must maintain real-time performance:

1. **Burst Engine**: Manages neuron firing dynamics, threshold detection, and refractory periods
2. **ConnectomeManager (Singleton)**: Single source of truth for neuron and synapse data structures
3. **FCL Manager**: Maintains the Fire Candidate List and provides efficient queries
4. **Memory & Learning Manager**: Applies plasticity rules to synaptic weights

### Priority 2 (Important - Near Real-time)
These tasks handle important but less time-critical operations:

1. **FQSampler**: Periodically extracts data from the Fire Queue for visualization and motor output
2. **ZMQ Server**: Manages communication with peripherals and external systems (no subprocess)
3. **Resource Manager**: Monitors and allocates system resources based on task demands

### Priority 3 (Background - Best Effort)
These tasks handle optional or background operations as direct async tasks:

1. **REST API Service**: Provides API endpoints for monitoring and control (direct dependency injection)
2. **Stem Cell Manager**: Handles neurogenesis and synaptogenesis
3. **Sleep Manager**: Manages memory consolidation during inactive periods

## Process Manager (Rust/RTOS Compatible)

The Process Manager is responsible for:

1. **Task Creation**: Launches async tasks with appropriate resources and parameters
2. **Task Monitoring**: Monitors task health and performance without subprocess overhead
3. **Resource Allocation**: Distributes computing resources based on priority
4. **Fault Tolerance**: Restarts failed tasks and maintains system integrity

### Key Rust/RTOS Compatibility Features

#### Singleton Architecture
```python
# Mission-critical singleton pattern (translates directly to Rust std::sync::Once)
class ConnectomeManager:
    _instance = None

    @classmethod
    def instance(cls, config_or_max_neurons=10_000_000, max_synapses=100_000_000):
        if cls._instance is None:
            cls._instance = cls(config_or_max_neurons, max_synapses)
        return cls._instance
```

#### Direct Task Spawning
```python
# RUST/RTOS COMPATIBLE: Direct async task (no subprocess boundaries)
def run_api_service():
    loop = asyncio.new_event_loop()
    app = create_rest_app_direct(config)  # Direct dependency injection
    uvicorn.run(app, host=config['host'], port=config['port'])

# In Rust: tokio::spawn(async move { ... })
api_thread = threading.Thread(target=run_api_service, daemon=True)
api_thread.start()
```

#### Memory-Mapped State
```python
# RUST/RTOS COMPATIBLE: Memory-mapped state (translates to memmap2 crate)
class FeagiStateManager:
    def __init__(self, path: Optional[str] = None):
        self._mmap = mmap.mmap(file.fileno(), self.TOTAL_SIZE)
        # Direct memory access - perfect for Rust migration
```

## Resource Management

FEAGI employs a dynamic resource management strategy optimized for Rust/RTOS:

1. **CPU Allocation**:
   - Priority 1 tasks receive dedicated CPU cores
   - Priority 2 tasks share remaining cores
   - Priority 3 tasks use opportunistic scheduling

2. **Memory Allocation**:
   - Memory allocated based on neural network size
   - Shared memory-mapped files for inter-task communication
   - Zero-copy data structures for performance

3. **GPU Utilization**:
   - Primary allocation to Burst Engine for neural dynamics
   - Secondary allocation to visualization and processing
   - Dynamic time-sharing for non-critical computations

4. **State Synchronization**:
   - Memory-mapped state files for instant synchronization
   - Atomic state updates for thread safety
   - No environment variable dependencies

## System Workflow

The typical FEAGI system workflow follows these steps:

1. **Initialization**:
   - **ConnectomeManager Singleton** initialized first
   - **Memory-mapped state** created and shared
   - Critical Priority 1 tasks launched with singleton access
   - Priority 2 and 3 tasks started with direct dependency injection

2. **Runtime Operation**:
   - **Burst Engine** continuously processes neural activity using singleton ConnectomeManager
   - **FCL Manager** maintains and updates firing history in shared memory
   - **Memory & Learning Manager** applies plasticity rules via singleton interface
   - **FQSampler** periodically extracts data for visualization and motor control
   - **ZMQ Server** handles communication without subprocess boundaries

3. **Development and Learning**:
   - **Stem Cell Manager** runs as async task for neurogenesis
   - **Memory & Learning Manager** continuously updates synaptic weights
   - **Sleep Manager** activates as async task for memory consolidation

4. **Monitoring and Management**:
   - **REST API Service** provides monitoring via direct dependency injection
   - **Resource Manager** monitors task health without subprocess polling

## Deployment Considerations

FEAGI supports multiple deployment scenarios with Rust/RTOS compatibility:

1. **Single Machine**:
   - All tasks run in one process space with shared memory
   - Resource allocation optimized for available hardware
   - Perfect for development and embedded systems

2. **Distributed Deployment**:
   - Critical tasks run on high-performance compute nodes
   - Memory-mapped state shared across node boundaries
   - Suitable for large-scale simulations

3. **RTOS Deployment**:
   - Direct task-based architecture compatible with RTOS
   - No subprocess dependencies
   - Predictable memory usage and timing

## Rust Migration Benefits

The new architecture provides immediate benefits for Rust migration:

### Performance Improvements
- **No subprocess overhead**: Direct task communication
- **Zero-copy data access**: Memory-mapped state
- **Singleton consistency**: Single source of truth

### Memory Safety
- **Controlled shared state**: Memory-mapped files
- **No environment variable dependencies**: Direct injection
- **Predictable resource usage**: Task-based allocation

### Concurrency Model
- **Python threads** → **Rust tokio tasks**
- **Memory-mapped files** → **Rust memmap2 crate**
- **Singleton pattern** → **Rust std::sync::Once**

## Implementation Status

The Rust/RTOS compatible architecture is implemented in phases:

1. **Phase 1: Singleton Architecture** ✅
   - ConnectomeManager singleton implementation
   - ProcessManager using singleton ConnectomeManager
   - Memory-mapped state synchronization

2. **Phase 2: Direct Task Spawning** ✅
   - REST API service as direct async task
   - ZMQ server without subprocess boundaries
   - FQSampler with direct dependency injection

3. **Phase 3: Subprocess Elimination** ✅
   - All environment variable IPC removed
   - Direct memory sharing implementation
   - Clean task monitoring without subprocess polling

4. **Phase 4: Rust Integration** 📋
   - Core data structures in Rust
   - FFI bindings for Python interop
   - Full async runtime with tokio

## Related Documentation

- [Architecture Overview](../arch-system-overview.md)
- [IPC Architecture](../arch-ipc.md)
- [GPU Architecture](../arch-gpu.md)
- [State Management](../arch-state-manager.md)
