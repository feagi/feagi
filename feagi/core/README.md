# FEAGI Core Module

*Rust/RTOS Compatible Architecture*

## Overview

The FEAGI Core module provides the fundamental infrastructure for the Framework for Evolutionary Artificial General Intelligence. It implements a **Rust/RTOS compatible architecture** designed for high performance, mission-critical reliability, and seamless future migration.

## Key Components

### 1. ProcessManager (`process_manager.py`)

**Purpose**: Coordinates all FEAGI tasks and services with priority-based resource allocation.

**Architecture**: Direct task spawning instead of subprocess boundaries

```python
# RUST/RTOS COMPATIBLE: Direct async task spawning
def run_api_service():
    loop = asyncio.new_event_loop()
    app = create_rest_app_direct(config)  # Direct dependency injection
    uvicorn.run(app, host=config['host'], port=config['port'])

# In Rust: tokio::spawn(async move { ... })
api_thread = threading.Thread(target=run_api_service, daemon=True)
api_thread.start()
```

**Features**:
- ✅ Singleton ConnectomeManager integration
- ✅ Direct task spawning (no subprocesses)
- ✅ Memory-mapped state synchronization
- ✅ Priority-based task management (Critical/Important/Background)

### 2. StateManager (`state_manager.py`)

**Purpose**: Memory-mapped state management for zero-copy data access across tasks.

**Architecture**: Shared memory files for instant synchronization

```python
# RUST/RTOS COMPATIBLE: Memory-mapped state (translates to memmap2 crate)
class FeagiStateManager:
    def __init__(self, path: Optional[str] = None):
        self._mmap = mmap.mmap(file.fileno(), self.TOTAL_SIZE)
        # Direct memory access - perfect for Rust migration
```

**Features**:
- ✅ Memory-mapped files for performance
- ✅ Atomic state updates
- ✅ Cross-task state synchronization
- ✅ No environment variable dependencies

### 3. Core API (`__init__.py`)

**Purpose**: Central gateway for all genome and connectome operations.

**Architecture**: Singleton pattern with direct dependency injection

```python
# Mission-critical singleton access
from feagi.bdu.connectome_manager import ConnectomeManager
connectome_manager = ConnectomeManager.instance()

def create_core_api(connectome_manager, config):
    # Direct dependency injection - no subprocess boundaries
    return CoreAPI(connectome_manager, config)
```

**Features**:
- ✅ Singleton ConnectomeManager integration
- ✅ Direct API access to critical components
- ✅ No subprocess overhead
- ✅ Thread-safe operations

## Task Priority Levels

### Priority 1 (Critical - Real-time)
Mission-critical components that must maintain real-time performance:
- **Burst Engine**: Neuron firing dynamics
- **ConnectomeManager**: Brain state management (singleton)
- **FCL Manager**: Fire Candidate List operations
- **Memory & Learning Manager**: Plasticity rules

### Priority 2 (Important - Near Real-time)  
Important but less time-critical operations:
- **FQSampler**: Fire Queue sampling
- **ZMQ Server**: Inter-process communication
- **Resource Manager**: Resource allocation monitoring

### Priority 3 (Background - Best Effort)
Background services as direct async tasks:
- **REST API Service**: HTTP endpoints (direct dependency injection)
- **Stem Cell Manager**: Neurogenesis operations
- **Sleep Manager**: Memory consolidation

## Rust/RTOS Compatibility

### Memory Management
- **Python**: `mmap` memory-mapped files
- **Rust**: Direct translation to `memmap2` crate
- **Performance**: Zero-copy data access

### Concurrency
- **Python**: `threading.Thread` and `asyncio` 
- **Rust**: `tokio::spawn` and `std::thread`
- **Safety**: Thread-safe singleton patterns

### State Management
- **Python**: Struct-like memory mapping
- **Rust**: `repr(C)` structs with direct memory layout
- **Synchronization**: Atomic operations for consistency

## Migration Benefits

### Performance Improvements
- **No subprocess overhead**: Direct task communication
- **Zero-copy data access**: Memory-mapped state
- **Singleton consistency**: Single source of truth
- **Reduced latency**: Eliminated IPC boundaries

### Memory Safety
- **Controlled shared state**: Memory-mapped files
- **No environment variables**: Direct injection
- **Predictable usage**: Task-based allocation
- **Thread safety**: Atomic operations

### Architecture Simplicity
- **Fewer moving parts**: No subprocess management
- **Direct dependencies**: Clear component relationships
- **Easier debugging**: Single process space
- **Better testability**: Direct component access

## Usage Examples

### Starting the Process Manager
```python
from feagi.process_manager import get_process_manager

# Get singleton instance
process_manager = get_process_manager()

# Start all tasks in priority order
config = {
    "api": {"host": "127.0.0.1", "port": 8000},
    "zmq": {"host": "127.0.0.1", "req_port": 5555}
}

success = process_manager.start(config)
if success:
    print("FEAGI started successfully")
```

### Accessing Core Components
```python
# Get references to core components
core_api = process_manager.get_core_api()
zmq_server = process_manager.get_zmq_server()

# Access singleton ConnectomeManager
from feagi.bdu.connectome_manager import ConnectomeManager
connectome = ConnectomeManager.instance()
```

### State Management
```python
from feagi.core.state_manager import FeagiStateManager, ServiceState

# Get singleton state manager
state_manager = FeagiStateManager.instance()

# Check system state
if state_manager.get_genome_state() == GenomeState.LOADED:
    print("Genome is ready")

# Update service state
state_manager.set_api_state(ServiceState.READY)
```

## Design Principles

### 1. Mission-Critical Reliability
- Singleton patterns for critical components
- Memory-mapped state for consistency
- Priority-based resource allocation
- Fault-tolerant task management

### 2. Performance Optimization
- Zero-copy data access
- Direct task communication
- Minimal memory overhead
- Lock-free operations where possible

### 3. Rust Migration Ready
- Compatible concurrency patterns
- Direct memory management
- Thread-safe singleton design
- No environment variable dependencies

### 4. RTOS Compatibility
- Predictable memory usage
- Deterministic task scheduling
- No subprocess overhead
- Real-time friendly operations

## Related Documentation

- [Architecture Overview](../../../docs/arch-system-overview.md)
- [Process Manager Architecture](../../../docs/archive/feagi_processes.md)
- [IPC Architecture](../../../docs/arch-ipc.md)
- [State Management](../../../docs/arch-state-manager.md)

## Testing

```bash
# Run core module tests
cd feagi_core
python -m pytest tests/core/ -v

# Run performance benchmarks
python -m pytest tests/core/.benchmarks/ -v
```

## Future Roadmap

### Phase 1: Python Optimization ✅
- Singleton ConnectomeManager
- Memory-mapped state management
- Direct task spawning
- Subprocess elimination

### Phase 2: Rust Core Components 🔄
- Core data structures in Rust
- FFI bindings for Python interop
- Memory-mapped state in Rust

### Phase 3: Full Rust Migration 📋
- Complete async runtime with tokio
- RTOS compatibility layer
- Performance optimizations 