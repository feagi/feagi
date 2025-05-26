# FEAGI Memory-Mapped State Management Specification

*Last Updated: January 15, 2025*

## Overview

The FEAGI memory-mapped state management system enables high-performance, zero-copy data synchronization within a unified process architecture. This specification defines the memory-mapped file formats, singleton patterns, and state synchronization mechanisms used by FEAGI 2.0.

**Architecture**: FEAGI uses **singleton state managers** with **memory-mapped backing stores** for persistent, cross-session state while maintaining all processing within a single unified process for optimal performance.

## 1. Singleton State Architecture

### Core Principles

- **Single Process**: All components (API, ZMQ, core engine) run in one unified process
- **Singleton Instances**: Single authoritative instance of each state manager
- **Memory-Mapped Persistence**: State persisted to memory-mapped files for crash recovery
- **Zero-Copy Access**: Direct memory access to state data structures
- **Rust/RTOS Compatible**: Design enables future migration to Rust with minimal changes

### State Manager Hierarchy

```
┌─────────────────────────────────────────────────────────────────────┐
│                    FEAGI Unified Process                           │
├─────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────┐  │
│  │ ConnectomeManager│  │ FeagiStateManager│  │  ProcessManager    │  │
│  │   (Singleton)   │  │   (Singleton)    │  │   (Singleton)      │  │
│  └─────────┬───────┘  └─────────┬───────┘  └─────────┬───────────┘  │
│            │                    │                    │              │
│            ▼                    ▼                    ▼              │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────┐  │
│  │ connectome.mmap │  │ feagi_state.mmap│  │ process_state.mmap │  │
│  │ (Brain State)   │  │ (System State)  │  │ (Service State)    │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

## 2. Memory-Mapped File Structure

### File Naming Convention

All memory-mapped state files follow the pattern: `feagi_{component}.mmap`

- `feagi_connectome.mmap` - Brain connectome and neural state
- `feagi_state.mmap` - System state and configuration
- `feagi_process.mmap` - Process and service state

### File Location

- **Default**: `/tmp/feagi/` (configurable via `FEAGI_STATE_DIR`)
- **Permissions**: 0600 (owner read/write only)
- **Cleanup**: Automatic cleanup on graceful shutdown

### Memory Layout

Each memory-mapped file uses a structured binary format:

```
┌─────────────────┬─────────────────┬────────────────┬─────────────────┐
│ Header (256B)   │ Index Table     │ Data Sections  │ Free Space      │
│ - Magic Number  │ - Section Count │ - JSON Blobs   │ - Available     │
│ - Version       │ - Offsets       │ - Binary Data  │ - For Growth    │
│ - State Hash    │ - Sizes         │ - Arrays       │                 │
└─────────────────┴─────────────────┴────────────────┴─────────────────┘
```

## 3. FeagiStateManager Specification

### Purpose

Central singleton for system-wide state management with memory-mapped persistence.

### State Categories

| State Type | Format | Persistence | Access Pattern |
|------------|--------|-------------|----------------|
| **GenomeState** | Enum (LOADING, LOADED, ERROR) | Memory-mapped | Read-heavy |
| **BrainState** | Enum (INITIALIZING, READY, RUNNING) | Memory-mapped | Read-heavy |
| **BurstEngineState** | Boolean | Memory-mapped | Read-heavy |
| **Configuration** | JSON | Memory-mapped | Read-moderate |
| **Metrics** | JSON | Memory-only | Read-heavy |

### Implementation Pattern

```python
class FeagiStateManager:
    _instance = None
    _mmap_file = None
    
    @classmethod
    def instance(cls, path=None):
        """Get singleton instance with memory-mapped backing"""
        if cls._instance is None:
            cls._instance = cls(path)
        return cls._instance
    
    def __init__(self, path=None):
        if path is None:
            path = os.path.join(get_state_dir(), "feagi_state.mmap")
        
        self.path = path
        self._initialize_mmap()
        self._load_state()
    
    def _initialize_mmap(self):
        """Initialize memory-mapped file with proper structure"""
        # Ensure file exists with proper size
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        
        if not os.path.exists(self.path):
            self._create_initial_file()
        
        # Memory-map the file
        self._mmap_file = mmap.mmap(
            os.open(self.path, os.O_RDWR),
            0,
            access=mmap.ACCESS_WRITE
        )
    
    def get_genome_state(self) -> GenomeState:
        """Get current genome state with zero-copy access"""
        # Direct memory access - no serialization overhead
        state_value = struct.unpack('I', self._mmap_file[64:68])[0]
        return GenomeState(state_value)
    
    def set_genome_state(self, state: GenomeState):
        """Set genome state with immediate persistence"""
        # Atomic write to memory-mapped region
        struct.pack_into('I', self._mmap_file, 64, state.value)
        self._mmap_file.flush()  # Ensure immediate persistence
```

### State Synchronization

#### Atomic Updates

All state changes are atomic and immediately visible to all readers:

```python
# Example: Atomic genome state transition
state_manager = FeagiStateManager.instance()

# Atomic state update - visible immediately
state_manager.set_genome_state(GenomeState.LOADING)

# All subsequent reads see new state immediately
current_state = state_manager.get_genome_state()  # Returns LOADING
```

#### Thread Safety

The memory-mapped approach provides natural thread safety for:
- **Read Operations**: Multiple threads can read simultaneously
- **Atomic Writes**: Single-value updates are atomic by design
- **Complex Updates**: Protected by instance-level locks when needed

## 4. ConnectomeManager Specification

### Purpose

Singleton manager for brain connectome with memory-mapped neural arrays and sparse connectivity data.

### Memory Layout

```
ConnectomeManager Memory Map (feagi_connectome.mmap):

┌─────────────────────────────────────────────────────────────┐
│ Header Section (1KB)                                        │
│ - Total Neurons: uint64                                     │
│ - Total Synapses: uint64                                    │
│ - Cortical Area Count: uint32                               │
│ - Array Offsets: uint64[N]                                  │
└─────────────────────────────────────────────────────────────┘
│ Neuron Properties (SoA Layout)                              │
│ - membrane_potentials: float32[max_neurons]                 │
│ - firing_states: uint8[max_neurons]                         │
│ - positions_x: uint32[max_neurons]                          │
│ - positions_y: uint32[max_neurons]                          │
│ - positions_z: uint32[max_neurons]                          │
└─────────────────────────────────────────────────────────────┘
│ Cortical Area Metadata (JSON)                               │
│ - Area definitions, types, parameters                       │
└─────────────────────────────────────────────────────────────┘
│ Sparse Connectivity Data                                    │
│ - Synaptic connections in compressed format                 │
└─────────────────────────────────────────────────────────────┘
```

### Zero-Copy Neural Data Access

```python
class ConnectomeManager:
    @classmethod
    def instance(cls, max_neurons=10_000_000, max_synapses=100_000_000):
        if cls._instance is None:
            cls._instance = cls(max_neurons, max_synapses)
        return cls._instance
    
    def get_firing_neurons(self, cortical_area_id: str) -> np.ndarray:
        """Get firing neurons with zero-copy access"""
        # Direct access to memory-mapped firing state array
        area_offset, area_size = self._get_area_bounds(cortical_area_id)
        
        # Zero-copy slice of memory-mapped array
        firing_slice = self.firing_states[area_offset:area_offset + area_size]
        
        # Return indices of firing neurons
        return np.where(firing_slice > 0)[0] + area_offset
    
    def get_membrane_potentials(self, neuron_indices: np.ndarray) -> np.ndarray:
        """Get membrane potentials with zero-copy access"""
        # Direct indexing into memory-mapped array
        return self.membrane_potentials[neuron_indices]
```

## 5. Cross-Language Compatibility

### Rust Migration Readiness

The memory-mapped format is designed for seamless Rust migration:

```rust
// Future Rust implementation
use memmap2::MmapMut;
use std::sync::{Arc, Mutex};

struct FeagiStateManager {
    mmap: Arc<Mutex<MmapMut>>,
}

impl FeagiStateManager {
    fn get_genome_state(&self) -> GenomeState {
        let mmap = self.mmap.lock().unwrap();
        let state_bytes = &mmap[64..68];
        let state_value = u32::from_ne_bytes(state_bytes.try_into().unwrap());
        GenomeState::from(state_value)
    }
}
```

### Memory Layout Compatibility

- **Byte Order**: Native endianness for performance
- **Alignment**: Struct alignment compatible between Python/Rust
- **Size Guarantees**: Fixed-size primitives (uint32, float32, etc.)

## 6. Performance Characteristics

### Memory Access Patterns

| Operation | Latency | Throughput | CPU Cache Impact |
|-----------|---------|------------|------------------|
| State Read | ~10ns | 10GB/s | Excellent |
| State Write | ~50ns | 2GB/s | Good |
| Array Access | ~5ns | 20GB/s | Excellent |
| JSON Config | ~1μs | Variable | Fair |

### Optimization Features

- **CPU Cache Friendly**: Data structures aligned for cache efficiency
- **SIMD Ready**: Arrays layout compatible with vectorized operations
- **Memory Locality**: Related data stored contiguously
- **Zero Allocation**: No garbage collection pressure during normal operation

## 7. Configuration and Setup

### Environment Variables

```bash
# State directory location
export FEAGI_STATE_DIR="/tmp/feagi"

# Memory-mapped file size limits
export FEAGI_CONNECTOME_SIZE="1GB"
export FEAGI_STATE_SIZE="100MB"

# Performance tuning
export FEAGI_MMAP_POPULATE="true"    # Pre-populate pages
export FEAGI_MMAP_LOCKED="true"     # Lock pages in memory
```

### Initialization Sequence

```python
def initialize_feagi_state():
    """Initialize FEAGI with memory-mapped state"""
    # 1. Initialize state manager singleton
    state_manager = FeagiStateManager.instance()
    
    # 2. Initialize connectome manager singleton  
    connectome = ConnectomeManager.instance()
    
    # 3. Verify memory-mapped files are healthy
    if not state_manager.verify_integrity():
        raise RuntimeError("State file corruption detected")
    
    # 4. Set initial states
    state_manager.set_brain_state(BrainState.INITIALIZING)
    
    return state_manager, connectome
```

## 8. Error Handling and Recovery

### File Corruption Detection

```python
def verify_mmap_integrity(mmap_file):
    """Verify memory-mapped file integrity"""
    # Check magic number
    magic = struct.unpack('Q', mmap_file[0:8])[0]
    if magic != FEAGI_MAGIC_NUMBER:
        return False
    
    # Verify checksum
    stored_checksum = struct.unpack('Q', mmap_file[8:16])[0]
    calculated_checksum = calculate_checksum(mmap_file[16:])
    
    return stored_checksum == calculated_checksum
```

### Recovery Strategies

1. **Graceful Degradation**: Fall back to in-memory state if mmap fails
2. **Backup Recovery**: Restore from last known good state
3. **Clean Initialization**: Start fresh if corruption is unrecoverable

## 9. Testing and Validation

### State Consistency Tests

```python
def test_singleton_consistency():
    """Ensure singleton instances provide consistent state"""
    state1 = FeagiStateManager.instance()
    state2 = FeagiStateManager.instance()
    
    # Same instance reference
    assert state1 is state2
    
    # Consistent state reads
    state1.set_genome_state(GenomeState.LOADED)
    assert state2.get_genome_state() == GenomeState.LOADED
```

### Performance Benchmarks

```python
def benchmark_state_access():
    """Benchmark memory-mapped state access performance"""
    state_manager = FeagiStateManager.instance()
    
    # Measure read latency
    start = time.perf_counter()
    for _ in range(1000000):
        state = state_manager.get_genome_state()
    read_time = time.perf_counter() - start
    
    print(f"Average read latency: {read_time / 1000000 * 1e9:.1f} ns")
```

## Related Documentation

- [System Architecture](arch-system-overview.md)
- [State Management](arch-state-management.md)
- [API Formats](spec-api-formats.md)
- [Usage Guide](guide-usage.md) 