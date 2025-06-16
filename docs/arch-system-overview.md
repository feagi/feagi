# FEAGI System Architecture Overview

*Last Updated: January 15, 2025*

## Project Overview

The Framework for Evolutionary Artificial General Intelligence (FEAGI) is designed to provide a flexible and extensible platform for developing brain-inspired artificial general intelligence.

FEAGI 2.0 employs a **Rust/RTOS compatible architecture** designed for high performance, mission-critical reliability, and seamless future migration. The architecture is built on four core principles:

1. **Singleton Pattern**: Critical components use singleton instances for consistent state and performance
2. **Direct Task Spawning**: Async tasks instead of subprocess boundaries for efficiency
3. **Shared Memory State**: Memory-mapped state management for zero-copy data access
4. **Priority-based Resource Allocation**: System resources allocated based on process criticality

## Core Features

- **Multi-backend support** (CPU/GPU)
- **FastAPI-based REST API** with direct dependency injection
- **High-performance ZMQ messaging** for bidirectional communication
- **Task-based architecture** with priority-based resource allocation
- **Singleton ConnectomeManager** for mission-critical reliability
- **Memory-mapped state management** for zero-copy data access
- **Rust/RTOS compatibility** for future migration
- **Embedded performance optimizations** integrated into core architecture (10M neurons at 15Hz)
- **SIMD-optimized neural processing** with cache-aligned memory and block-sparse matrices
- **Evolutionary optimization** capabilities
- **Configurable visualization sampling** for high-frequency simulations

FEAGI uses a direct task spawning architecture for improved reliability and performance. This eliminates subprocess boundaries, provides instant state synchronization, and ensures mission-critical consistency.

## Architecture Principles

### Singleton Architecture

Critical components use singleton patterns for mission-critical reliability:

```python
# ConnectomeManager - Mission-critical singleton
class ConnectomeManager:
    _instance = None

    @classmethod
    def instance(cls, config_or_max_neurons=10_000_000, max_synapses=100_000_000):
        if cls._instance is None:
            cls._instance = cls(config_or_max_neurons, max_synapses)
        return cls._instance

# ProcessManager uses singleton ConnectomeManager
def __init__(self):
    from feagi.bdu.connectome_manager import ConnectomeManager
    self._connectome_manager = ConnectomeManager.instance()
```

### Task-Based Process Management

**Direct Task Spawning (Rust/RTOS Compatible)**:
```python
# NEW: Direct async task spawning (Rust/RTOS compatible)
def run_api_service():
    loop = asyncio.new_event_loop()
    app = create_rest_app_direct(config)  # Direct dependency injection
    uvicorn.run(app, host=config['host'], port=config['port'])

# In Rust: tokio::spawn(async move { ... })
api_thread = threading.Thread(target=run_api_service, daemon=True)
api_thread.start()
```

### Memory-Mapped State Management

```python
# RUST/RTOS COMPATIBLE: Memory-mapped state (translates to Rust memmap2)
class FeagiStateManager:
    def __init__(self, path: Optional[str] = None):
        self._mmap = mmap.mmap(file.fileno(), self.TOTAL_SIZE)
        # Direct memory access - perfect for Rust migration
```

### Burst Engine Lifecycle

FEAGI 2.0 implements a strict burst engine lifecycle that enforces system integrity:

**Engine-First Design Principle**: The burst engine MUST be started before any genome can be loaded.

**State Management**:
- `UNAVAILABLE` → Engine not started (FEAGI launch state)
- `READY` → Engine running and processing neurons
- `ON_HOLD` → Engine alive but paused (new feature for debugging/analysis)
- `FAILED/ERROR` → Engine error states

**Workflow**:
1. FEAGI launches with burst engine `UNAVAILABLE`
2. Genome load request triggers automatic burst engine start
3. If engine start fails → genome load is rejected (fail-fast)
4. If engine start succeeds → genome loading proceeds
5. Post-load: engine remains `READY`, with optional hold/resume control

This design ensures that neural processing is always available when a genome is loaded, preventing inconsistent system states. See [Burst Engine Lifecycle Architecture](arch-burst-engine-lifecycle.md) for complete details.

## Task Priority Levels

### Priority 1 (Critical - Real-time)
These components run with highest priority and use singleton patterns:

1. **Burst Engine**: Neuron firing dynamics with direct ConnectomeManager access
2. **ConnectomeManager (Singleton)**: Single source of truth for brain state
3. **FCL Manager**: Fire Candidate List with Roaring bitmaps
4. **Memory & Learning Manager**: Plasticity rules application

### Priority 2 (Important - Near Real-time)
These services handle important but less time-critical operations:

1. **FQSampler**: Fire Queue sampling for visualization/motor output
2. **ZMQ Server**: Inter-process communication without subprocess boundaries
3. **Resource Manager**: Dynamic resource allocation and monitoring

### Priority 3 (Background - Best Effort)
These services run as async tasks in the same process space:

1. **REST API Service**: Direct dependency injection, no subprocess
2. **Stem Cell Manager**: Neurogenesis and synaptogenesis
3. **Sleep Manager**: Memory consolidation during inactive periods

## Project Structure

```
./
├── feagi/                  # Main package
│   ├── core/               # Core functionality and resource management
│   │   ├── state_manager.py  # Memory-mapped state management
│   │   └── __init__.py     # Core API with singleton integration
│   ├── process_manager.py  # Rust/RTOS compatible task management
│   ├── bdu/                # Brain Developmental Unit
│   │   └── connectome_manager.py  # Singleton ConnectomeManager
│   ├── npu/                # Neural Processing Unit
│   ├── evo/                # Evolutionary Unit
│   ├── pns/                # Peripheral Nervous System and sensorimotor IO modules
│   ├── api/                # API implementations
│   │   ├── rest/           # REST API with direct dependency injection
│   │   └── zmq/            # ZMQ server without subprocess boundaries
│   └── viz/                # Visualization data transformation
├── tests/                  # Unit, integration, and functional tests
├── docs/                   # Documentation
├── examples/               # Example scripts
└── requirements.txt        # Dependencies
```

## Compute Resource Strategy

- A subset of operations under Neural Processing Unit associated with neuron firing will be designed to support both CPU and GPU backends. The rest of the application will be running on CPU.
- The entire application is designed to run in a highly parallel and performant fashion
- **Rust/RTOS compatibility**: Code written for seamless migration to Rust and RTOS environments
- Quality of Service priority is assigned to various tasks ensuring critical processes have sufficient resources
- **Mission-critical reliability**: Singleton patterns eliminate state inconsistency
- **Zero-copy data access**: Memory-mapped state for instant synchronization

## Security Considerations

### Authentication
FEAGI API and ZMQ are equipped with authentication enabling secure communication on all communication methods.

### Encryption
Encryption can negatively impact the transmission of sensorimotor data by adding latency but might be essential for select use-cases. Both API and ZMQ support encryption as an option.

## Major System Modules

### Core (CORE)
- **State Manager**: Memory-mapped state management for instant synchronization
- **Process Manager**: Rust/RTOS compatible task spawning and management
- **API Services**: Direct dependency injection without subprocess boundaries

### Peripheral Nervous System (PNS)
- Sensory processor
- Motor processor

### Brain Developmental Unit (BDU)
- **ConnectomeManager (Singleton)**: Mission-critical brain state management
- Neural structure development

### Neural Processing Unit (NPU)
- **Burst Engine**: Direct access to singleton ConnectomeManager
- Neural computation with priority-based scheduling

### Memory & Learning Unit (MLU)
- Learning mechanisms

### Evolutionary Unit (EVO)
- Evolutionary optimization

### Sleep (SLP)
- Memory consolidation
- Neural development

## Key Components

### REST API
- **Direct dependency injection**: No environment variable dependencies
- **Rust/RTOS compatible**: Ready for future migration
- Organized in version folders to enable future maintainability
- Routes defined enabling endpoints to be organized by functional area

### ZMQ Handler
- **No subprocess boundaries**: Direct task integration
- Initiates a ZMQ server with the ability to support multiple topics enabling multithreaded communication to and from FEAGI
- **Memory-mapped state access**: Instant synchronization with other services

### Process Manager
- **Task-based architecture**: Direct async task spawning instead of subprocesses
- **Priority-based scheduling**: Critical/Important/Background task priorities
- **Singleton integration**: Uses singleton ConnectomeManager for consistency
- **Rust/RTOS compatible**: Ready for seamless migration

## Rust/RTOS Compatibility

The architecture is designed for seamless Rust migration:

### Memory Management
- **Python**: Memory-mapped files with `mmap`
- **Rust**: Direct translation to `memmap2` crate

### Concurrency
- **Python**: `threading.Thread` and `asyncio` tasks
- **Rust**: `tokio::spawn` and `std::thread`

### Singleton Pattern
- **Python**: Class-based singletons with `@classmethod`
- **Rust**: `std::sync::Once` for thread-safe initialization

### State Management
- **Python**: Struct-like memory mapping
- **Rust**: Direct memory layout with `repr(C)` structs

## Performance Characteristics

- **State access time**: ~5-20 nanoseconds (direct memory read)
- **Task communication**: No serialization overhead
- **Memory overhead**: Minimal singleton instances
- **Startup time**: Reduced by eliminating subprocess spawn

## Related Documentation
- [Rust/RTOS Migration Guide](arch-rust-rtos-migration.md)
- [GPU Architecture](arch-gpu.md)
- [IPC Architecture](arch-ipc.md)
- [State Management](arch-state-management.md)
- [Process Architecture](archive/feagi_processes.md)
