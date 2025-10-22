# FEAGI Rust Burst Engine

**High-performance neural burst processing engine for FEAGI 2.0**

## 🎯 Project Goals

- **Performance**: 50-100x speedup over Python (165ms → <3ms for synaptic propagation)
- **Scalability**: Support 30Hz burst frequency with 1.2M neuron firings
- **Architecture**: Incremental migration path from Python to Rust
- **IP Protection**: Separate plasticity crate with proprietary license

## 📦 Workspace Structure

```
feagi-rust/
├── Cargo.toml                    # Workspace root
├── crates/
│   ├── feagi-types/              # Core types (NeuronId, Synapse, etc.)
│   ├── feagi-burst-engine/       # High-performance burst processing
│   ├── feagi-plasticity/         # Plasticity algorithms (separate IP)
│   └── feagi-python/             # PyO3 bindings for Python interop
└── target/
    └── release/
        └── feagi_rust.so         # Python extension module
```

## 🏗️ Architecture Decisions

### 1. **Workspace-based Crate Organization**
- **feagi-types**: Shared core types (no dependencies)
- **feagi-burst-engine**: Depends only on types (fast, pure Rust)
- **feagi-plasticity**: Separate crate for IP protection (PROPRIETARY license)
- **feagi-python**: PyO3 bindings (depends on all above)

### 2. **IP Separation**
The plasticity crate is intentionally separate:
- Different license (PROPRIETARY vs Apache-2.0)
- Can be compiled/distributed separately
- Trait-based plugin system for runtime integration

### 3. **Incremental Migration Strategy**
1. **Phase 1** (Current): Synaptic propagation (the bottleneck)
2. **Phase 2**: Full burst engine phases
3. **Phase 3**: Connectome management
4. **Phase 4**: Complete FEAGI core

## 🚀 Performance Target vs Achievement

### Python Baseline (from profiling)
```
Phase 1 (Injection):  163.84 ms ( 88.7%)
  └─ Synaptic Propagation: 161.07 ms (100% of Phase 1)
     └─ Numpy Processing:  164.67 ms ( 91.7%)
```

### Rust Implementation
- **Target**: <3ms (50-100x speedup)
- **Actual**: TBD (requires benchmarking with real data)

## 🔧 Building

```bash
# Build entire workspace (all crates)
cargo build --workspace --release

# Build just the Python extension
cargo build --release -p feagi-python

# Run tests
cargo test --workspace

# Create Python-compatible symlink (macOS)
cd target/release && ln -sf libfeagi_rust.dylib feagi_rust.so
```

## 🐍 Python Integration

### Example Usage

```python
import numpy as np
import feagi_rust

# Create engine
engine = feagi_rust.SynapticPropagationEngine()

# Build synapse index (once during initialization)
engine.build_index(
    source_neurons,   # np.array[uint32]
    target_neurons,   # np.array[uint32]
    weights,          # np.array[uint8, 0-255]
    conductances,     # np.array[uint8, 0-255]
    types,            # np.array[uint8, 0=excitatory, 1=inhibitory]
    valid_mask        # np.array[bool]
)

# Set neuron mapping
engine.set_neuron_mapping({
    neuron_id: cortical_area_id,
    # ...
})

# Compute propagation (HOT PATH - called every burst!)
fired_neurons = np.array([1, 2, 3], dtype=np.uint32)
result = engine.propagate(fired_neurons)
# Returns: {area_id: [(target_neuron, contribution), ...], ...}
```

### Integrating with burst_engine.py

Replace `_compute_synaptic_propagation()` in `feagi/npu/burst_engine.py`:

```python
# OLD (Python - 165ms):
def _compute_synaptic_propagation(self) -> Dict[int, List[tuple]]:
    # ... 180 lines of Python/NumPy code ...
    return propagation_data

# NEW (Rust - <3ms):
def _compute_synaptic_propagation(self) -> Dict[int, List[tuple]]:
    if not hasattr(self, '_rust_engine'):
        self._initialize_rust_engine()
    
    fired_neuron_ids = self.previous_fire_queue.get_all_neuron_ids()
    return self._rust_engine.propagate(np.array(fired_neuron_ids, dtype=np.uint32))
```

## 📊 Test Results

```bash
$ python test_integration.py
✅ Successfully imported feagi_rust module
   Version: 0.1.0

📊 Creating test synapse data...
   Created 6 synapses

🚀 Creating Rust synaptic propagation engine...
   Engine created successfully

🔨 Building synapse index...
   Index built successfully

🗺️  Setting neuron-to-cortical-area mapping...
   Mapped 5 neurons to cortical areas

⚡ Computing synaptic propagation...
   Fired neurons: [1, 2]
   Result: {2: [(13, 0.39)], 1: [(10, 1.0), (11, -0.50), (12, 0.62), (10, 1.0)]}

📈 Performance statistics:
   Total propagations: 1
   Total synapses processed: 5

✅ ALL TESTS PASSED!
```

## 🔬 Design Principles

### RTOS Compatibility
- No allocations in hot paths (pre-allocate during init)
- No locks/mutexes in critical sections
- Rayon for data parallelism (CPU-bound tasks)
- Conditional compilation for embedded targets

### Cache-Friendly Data Structures
- `#[repr(C)]` for predictable memory layout
- Array-of-Structures (AoS) for synapse data
- Sequential memory access patterns
- SIMD-friendly vectorized operations

### Type Safety
- Strong types instead of primitives (`NeuronId(u32)` not `u32`)
- Compile-time guarantees (no runtime type checks)
- Zero-cost abstractions

## 📝 Next Steps

1. **Benchmark with Real Data**
   - Use actual FEAGI synapse arrays (12K neurons, 13K synapses)
   - Measure end-to-end performance
   - Validate 50-100x speedup claim

2. **Integrate with burst_engine.py**
   - Add Rust engine initialization
   - Replace Python `_compute_synaptic_propagation()`
   - Add fallback for graceful degradation

3. **Expand Coverage**
   - Implement other burst engine phases in Rust
   - Add neuron dynamics (membrane potential updates)
   - Migrate connectome operations

4. **Production Deployment**
   - Build for multiple platforms (Linux, macOS, Windows)
   - Create Python wheel for easy installation
   - Add CI/CD pipeline for automated builds

## 📄 License

- **feagi-types**: Apache-2.0
- **feagi-burst-engine**: Apache-2.0
- **feagi-python**: Apache-2.0
- **feagi-plasticity**: PROPRIETARY (separate IP protection)

## 🤝 Contributing

Follow FEAGI's coding guidelines in `/docs/coding_guidelines.md`.

All Rust code must:
- Pass `cargo clippy` (zero warnings)
- Pass `cargo test` (100% passing)
- Include inline documentation
- Follow Rust API guidelines

---

**Built with ❤️ by the FEAGI team using Rust 🦀**




