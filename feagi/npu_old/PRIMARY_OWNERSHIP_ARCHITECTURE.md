# NPU Primary Ownership Architecture

*Version: 2.0 | Last Updated: January 2025*

## Overview

This document describes the **NPU Primary Ownership** architecture for FEAGI 2.0, designed specifically for **Rust migration** where NPU will be the first module rewritten in Rust. This architecture establishes NPU as the **primary owner** of neural data structures while providing **controlled access** to BDU during sleep periods.

## Key Design Principles

### 1. **NPU is Primary Owner**
- NPU **owns** the neuron and synapse Structure of Arrays (SoA)
- NPU **controls access** to these data structures
- NPU **manages memory allocation** and lifecycle
- NPU **provides interfaces** for BDU access

### 2. **BDU Gets Controlled Access**
- BDU can **restructure data** only during NPU sleep periods
- BDU uses **NPU-provided interfaces** to access data
- BDU **cannot directly access** NPU-owned arrays
- BDU operations are **coordinated by Sleep Manager**

### 3. **Rust Migration Ready**
- NPU data structures are **FFI-compatible**
- Clear **ownership boundaries** for Rust/Python interop
- **No shared ownership** complexity
- **Deterministic access patterns**

### 4. **Memory Neuron Separation**
- Memory neurons remain **BDU-owned** (CPU-based)
- Regular neurons are **NPU-owned** (GPU/SIMD optimized)
- **Separate processing pipelines** maintained
- **No architectural conflicts**

---

## Architecture Components

### NPU Components (Primary Owners)

#### 1. **NeuralProcessor**
```python
class NeuralProcessor:
    """PRIMARY OWNER of neural data structures"""
    
    def __init__(self, max_neurons: int, max_synapses: int, backend: str):
        # NPU OWNS these data structures
        self.neurons = NPUNeuronArray(max_neurons, backend)
        self.synapses = NPUSynapseArray(max_synapses, backend)
        
        # BDU access control
        self._bdu_access_enabled = True
        self._npu_processing_active = False
```

#### 2. **NPUNeuronArray** (NPU-Owned)
```python
class NPUNeuronArray:
    """NPU-owned neuron storage using Structure of Arrays (SoA)"""
    
    def __init__(self, max_neurons: int, backend: str):
        # Structure of Arrays - 64-byte aligned for SIMD
        self.membrane_potentials = np.zeros(max_neurons, dtype=np.float32)
        self.thresholds = np.ones(max_neurons, dtype=np.float32)
        # ... other arrays
        
        # NPU owns the mappings
        self.neuron_id_to_index = {}
        self.index_to_neuron_id = {}
```

#### 3. **NPUSynapseArray** (NPU-Owned)
```python
class NPUSynapseArray:
    """NPU-owned synapse storage using Structure of Arrays (SoA)"""
    
    def __init__(self, max_synapses: int, backend: str):
        # Structure of Arrays - optimized for scatter-gather
        self.source_neurons = np.zeros(max_synapses, dtype=np.uint32)
        self.target_neurons = np.zeros(max_synapses, dtype=np.uint32)
        self.weights = np.zeros(max_synapses, dtype=np.float32)
        
        # NPU owns the indexing
        self.source_neuron_index = {}
```

### BDU Interface Components (Controlled Access)

#### 1. **BDUNeuronInterface**
```python
class BDUNeuronInterface:
    """BDU interface to NPU-owned neuron data"""
    
    def __init__(self, npu_neuron_array):
        self.npu_neurons = npu_neuron_array  # Reference to NPU data
    
    def add_neuron(self, neuron_id: int, properties: Dict) -> bool:
        """Add neuron to NPU-owned arrays (during sleep only)"""
    
    def update_neuron_property(self, neuron_id: int, prop: str, value) -> bool:
        """Update neuron in NPU-owned arrays (during sleep only)"""
```

#### 2. **BDUSynapseInterface**
```python
class BDUSynapseInterface:
    """BDU interface to NPU-owned synapse data"""
    
    def __init__(self, npu_synapse_array):
        self.npu_synapses = npu_synapse_array  # Reference to NPU data
    
    def add_synapse(self, src: int, tgt: int, weight: float) -> bool:
        """Add synapse to NPU-owned arrays (during sleep only)"""
```

---

## Sleep Manager Coordination

### NPU Sleep Cycle
```python
# NPU goes to sleep
npu_processor.enable_bdu_access()  # Allow BDU restructuring

# BDU gets interfaces to NPU data
neuron_interface = npu_processor.get_bdu_neuron_interface()
synapse_interface = npu_processor.get_bdu_synapse_interface()

# BDU performs restructuring operations
neuron_interface.add_neuron(new_id, properties)
synapse_interface.add_synapse(src, tgt, weight)

# NPU wakes up
npu_processor.disable_bdu_access()  # Reclaim exclusive access
```

### Access Control States
```python
# NPU Processing (BDU blocked)
_npu_processing_active = True
_bdu_access_enabled = False

# NPU Sleep (BDU allowed)
_npu_processing_active = False
_bdu_access_enabled = True
```

---

## Data Flow Architecture

### 1. **Brain Development Phase** (BDU → NPU Transfer)
```
BDU ConnectomeManager (Development)
    ↓ (One-time transfer)
NPU Neural Processor (Runtime Primary Owner)
    ↓ (Controlled interfaces)
BDU Restructuring Operations (Sleep periods only)
```

### 2. **Runtime Processing Phase** (NPU Primary)
```
NPU Neural Processor (Primary Owner)
    ├── Regular Neurons (GPU/SIMD optimized)
    ├── Synaptic Propagation (NPU-owned arrays)
    └── Memory Neurons (BDU-owned, CPU-based)
```

### 3. **Sleep Period Restructuring** (BDU Controlled Access)
```
Sleep Manager
    ↓ (Coordinates access)
NPU enables BDU access
    ↓ (Provides interfaces)
BDU Neurogenesis/Synaptogenesis
    ↓ (Via NPU interfaces)
NPU-owned data structures updated
    ↓ (Sleep ends)
NPU reclaims exclusive access
```

---

## Rust Migration Benefits

### 1. **Clear Ownership Boundaries**
```rust
// Rust NPU (Primary Owner)
pub struct NeuralProcessor {
    neurons: NPUNeuronArray,     // Rust-owned
    synapses: NPUSynapseArray,   // Rust-owned
    bdu_access_enabled: bool,    // Access control
}

// Python BDU (Controlled Access)
// Gets FFI interfaces to Rust data
```

### 2. **FFI-Compatible Data Structures**
```rust
#[repr(C)]
pub struct NPUNeuronArray {
    membrane_potentials: *mut f32,  // C-compatible arrays
    thresholds: *mut f32,
    neuron_count: u32,
    max_neurons: u32,
}
```

### 3. **No Shared Ownership Complexity**
- **Rust owns** the data structures
- **Python gets** controlled access through FFI
- **No Arc/Mutex** complexity
- **Deterministic access** patterns

---

## Implementation Phases

### Phase 1: **Python NPU Primary Owner** (Current)
- NPU owns neuron/synapse arrays in Python
- BDU gets controlled access through interfaces
- Sleep Manager coordinates access
- **Validates architecture before Rust migration**

### Phase 2: **Rust NPU Migration**
- Rewrite NPU components in Rust
- Maintain same ownership model
- Python BDU uses FFI interfaces
- **Seamless transition due to established architecture**

### Phase 3: **Full Rust NPU**
- Complete NPU in Rust
- Optimized SIMD/GPU operations
- Python BDU integration via FFI
- **Maximum performance with clean architecture**

---

## Benefits Over Wrapper Approach

### ❌ **Wrapper Approach Problems**
- Shared ownership complexity
- Memory duplication risks
- Rust FFI boundary issues
- Synchronization overhead

### ✅ **Primary Ownership Benefits**
- **Clear ownership** - NPU owns, BDU accesses
- **No duplication** - Single source of truth
- **Rust-ready** - Clean FFI boundaries
- **Performance** - Direct NPU control
- **Deterministic** - Controlled access patterns

---

## Memory Neuron Handling

### **Separation Maintained**
```python
# Regular neurons: NPU-owned (GPU/SIMD)
npu_processor.neurons = NPUNeuronArray(...)

# Memory neurons: BDU-owned (CPU-based)
npu_processor.memory_neurons = bdu_connectome.memory_neuron_array

# Processing separation
def process_neural_burst(self, timestep: int):
    # Phase 1: Regular neurons (NPU-owned, GPU optimized)
    fired_neurons = self.neurons.neural_update_simd(timestep)
    
    # Phase 2: Memory neurons (BDU-owned, CPU-based)
    memory_fired = self.connectome_manager.process_memory_neurons(timestep)
    
    return fired_neurons + memory_fired
```

---

## Conclusion

The **NPU Primary Ownership** architecture provides:

1. **🎯 Rust Migration Ready** - Clear ownership boundaries for FFI
2. **🚀 Performance Optimized** - NPU controls data layout and access
3. **🔒 Data Integrity** - Single source of truth, controlled access
4. **🧠 Memory Neuron Respect** - Maintains CPU/GPU separation
5. **🔄 BDU Compatibility** - Controlled restructuring during sleep

This architecture **solves the synapse array reset bug** by eliminating data handoffs while **preparing for Rust migration** with clean ownership boundaries and **respecting the existing memory neuron architecture**.

The NPU becomes the **primary owner** ready for Rust migration, while BDU gets **controlled access** for restructuring operations during coordinated sleep periods managed by the Sleep Manager.
