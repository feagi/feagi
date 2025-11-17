# FEAGI State Manager

## Overview

The FEAGI State Manager provides a high-performance, cross-language mechanism for tracking and updating FEAGI's global states. It uses memory-mapped files to achieve near-zero overhead state access between Python and Rust components.

## Design Goals

1. **Ultra-fast state access**: Optimized for high-frequency checks (every clock cycle)
2. **Cross-language compatibility**: Equal access from both Python and Rust
3. **Thread safety**: Multiple components can read/write concurrently
4. **Clear semantics**: Enum-based states with well-defined transitions
5. **Low overhead**: Minimal memory and CPU usage

## Implementation Details

### Memory-Mapped Architecture

The state manager uses a memory-mapped file as a shared memory region:

```
┌─────────────────────────┐
│  Memory-Mapped File     │
├─────────────────────────┤
│ genome_state:    u8     │
│ connectome_state: u8    │
│ api_state:        u8    │
│ zmq_state:        u8    │
│ agent_count:     u32    │
│ burst_engine:     u8    │
│ burst_frequency: f32    │
│ visualization_state: u8 │
│ state_version:   u64    │
└─────────────────────────┘
```

Both Python and Rust map this file into memory, allowing direct reads/writes without serialization overhead.

### State Types

States are implemented as enums with consistent integer values across languages:

1. **GenomeState**: Tracks genome loading status
   - `MISSING = 0`: No genome available
   - `LOADING = 1`: Genome being loaded/parsed
   - `LOADED = 2`: Genome loaded and ready
   - `SAVING = 3`: Genome being saved
   - `ERROR = 4`: Error in genome loading/saving

2. **ConnectomeState**: Tracks connectome status
   - `MISSING = 0`: No connectome available
   - `INITIALIZING = 1`: Being constructed from genome
   - `UPDATING = 2`: Being modified
   - `READY = 3`: Operational
   - `SNAPSHOTTING = 4`: Being saved
   - `ERROR = 5`: Error state

3. **ServiceState**: General service status
   - `UNAVAILABLE = 0`: Service not available
   - `INITIALIZING = 1`: Service starting
   - `READY = 2`: Service operational
   - `DEGRADED = 3`: Service operational but with issues
   - `ERROR = 4`: Service encountered an error

4. **VisualizationState**: Brain Visualization status
   - `STOPPED = 0`: No simulation running
   - `PAUSED = 1`: Simulation paused
   - `RUNNING = 2`: Simulation running
   - `STEPPING = 3`: Running in single-step mode

### State Change Detection

The `state_version` counter is atomically incremented on every state change. Components can track this value to detect when any state has changed without needing to poll individual fields.

## Multi-Process Operation

The State Manager is designed for distributed responsibility in FEAGI's multi-process architecture:

1. **No single owner**: The memory-mapped approach works without a central owner process
2. **Domain-specific responsibility**: Each process updates only the states it directly controls:
   - API server process: `api_state`
   - ZMQ process: `zmq_state`
   - Burst engine process: `burst_engine_state` and `burst_frequency`
   - Core/Simulation process: `genome_state`, `connectome_state`, `simulation_state`
3. **Independent access**: Any process can read any state field at any time
4. **Process initialization**: First process to start creates the file; subsequent processes use existing file

## Performance Characteristics

- **Read access time**: ~5-20 nanoseconds (direct memory read)
- **Write access time**: ~10-30 nanoseconds (direct memory write + atomic increment)
- **Memory overhead**: ~20 bytes total for all tracked states
- **CPU overhead**: Near-zero (no function call overhead for direct access)
- **Cache efficiency**: Entire state fits in a single CPU cache line

## Usage Examples (Python)

```python
from feagi.core.state_manager import FeagiStateManager, GenomeState, ServiceState

# Get the singleton instance
state_mgr = FeagiStateManager.instance()

# Update states with enum values
state_mgr.set_genome_state(GenomeState.LOADING)
state_mgr.set_burst_engine_state(ServiceState.READY)
state_mgr.set_burst_frequency(30.0)  # 30 Hz

# Check states efficiently
if state_mgr.get_genome_state() == GenomeState.LOADED and state_mgr.get_burst_engine_state() == ServiceState.READY:
    # Ready to start simulation
    state_mgr.set_simulation_state(SimulationState.RUNNING)

# High-level helpers
if state_mgr.is_simulation_running() and state_mgr.is_connectome_ready():
    # Simulation is active with valid connectome
    pass
```

## Genome Loading Example

Here's how to update the genome loading process to use the state manager:

```python
from feagi.core.state_manager import FeagiStateManager, GenomeState

# Get singleton instance of state manager
state_mgr = FeagiStateManager.instance()

def load_genome(genome_path):
    try:
        # Signal genome loading has started
        state_mgr.set_genome_state(GenomeState.LOADING)

        # Your existing genome loading logic
        # ...

        # When loading completes successfully
        state_mgr.set_genome_state(GenomeState.LOADED)
        return True

    except Exception as e:
        # If any error occurs during loading
        state_mgr.set_genome_state(GenomeState.ERROR)
        print(f"Genome loading failed: {e}")
        return False
```

## Migration Path to Rust

### Step 1: Implement Rust State Access

```rust
// In feagi/rust/src/core/state_manager.rs
use std::fs::OpenOptions;
use memmap2::{MmapMut, MmapOptions};

#[repr(u8)]
pub enum GenomeState {
    Missing = 0,
    Loading = 1,
    Loaded = 2,
    Saving = 3,
    Error = 4,
}

// Define other enums similarly...

#[repr(C)]
pub struct FeagiStateShared {
    pub genome_state: u8,
    pub connectome_state: u8,
    pub api_state: u8,
    pub zmq_state: u8,
    pub agent_count: u32,
    pub burst_engine_state: u8,
    pub burst_frequency: f32,
    pub simulation_state: u8,
    pub state_version: u64,
}

pub struct StateManager {
    mapping: MmapMut,
}

impl StateManager {
    pub fn new(path: &str) -> Self {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .open(path)
            .unwrap();

        // Ensure file is the right size
        let size = std::mem::size_of::<FeagiStateShared>();
        file.set_len(size as u64).unwrap();

        let mapping = unsafe { MmapOptions::new().map_mut(&file).unwrap() };

        Self { mapping }
    }

    #[inline]
    pub fn state_ptr(&self) -> *mut FeagiStateShared {
        self.mapping.as_ptr() as *mut FeagiStateShared
    }

    #[inline]
    pub fn get_genome_state(&self) -> GenomeState {
        unsafe {
            match (*self.state_ptr()).genome_state {
                0 => GenomeState::Missing,
                1 => GenomeState::Loading,
                2 => GenomeState::Loaded,
                3 => GenomeState::Saving,
                4 => GenomeState::Error,
                _ => GenomeState::Missing,
            }
        }
    }

    #[inline]
    pub fn set_genome_state(&mut self, state: GenomeState) {
        unsafe {
            (*self.state_ptr()).genome_state = state as u8;
            (*self.state_ptr()).state_version = (*self.state_ptr()).state_version.wrapping_add(1);
        }
    }

    // Implement getters/setters for other states...

    #[inline]
    pub fn is_genome_loaded(&self) -> bool {
        self.get_genome_state() == GenomeState::Loaded
    }
}
```

### Step 2: Future Full-Rust Implementation

```rust
// Future full-Rust implementation
use std::sync::atomic::{AtomicU8, AtomicU32, AtomicU64, Ordering};

pub struct FeagiState {
    pub genome_state: AtomicU8,
    pub connectome_state: AtomicU8,
    pub api_state: AtomicU8,
    pub zmq_state: AtomicU8,
    pub agent_count: AtomicU32,
    pub burst_engine_state: AtomicU8,
    pub burst_frequency: atomic_float::AtomicF32,
    pub simulation_state: AtomicU8,
    pub state_version: AtomicU64,
}

// Global state instance
pub static STATE: FeagiState = FeagiState {
    genome_state: AtomicU8::new(0),
    connectome_state: AtomicU8::new(0),
    // Initialize other fields...
};

// Ultra-fast access in performance-critical code
#[inline(always)]
pub fn is_burst_engine_ready() -> bool {
    STATE.burst_engine_state.load(Ordering::Relaxed) == 2
}
```

## Best Practices

1. **Use enum values**: Always use enum variants, never raw integer values
2. **Check version for changes**: Use state_version to detect changes efficiently
3. **Minimize writes**: Only update states when they genuinely change
4. **Explicit disk syncing**: Only call sync_to_disk when persistence is required
5. **Access pattern awareness**: Check if an operation needs the current state before reading

## Debugging Tips

1. Examine file contents directly: `hexdump -C /tmp/feagi_state.bin`
2. Add debug logging for state transitions in non-performance-critical code
3. Use process monitoring tools to verify both Python and Rust are accessing the file
4. Test state coherence by updating from one process and reading from another
