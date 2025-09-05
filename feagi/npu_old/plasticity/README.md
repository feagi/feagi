# NPU Plasticity System - Modular Architecture

*Version: 2.0 | Last Updated: January 2025*

## Overview

This directory contains a highly modular, RTOS-friendly synaptic plasticity system designed for Rust migration and high-performance neural processing. The system provides comprehensive plasticity operations while maintaining strict modularity and performance requirements.

## Architecture Principles

### 🎯 **Modular Design**
- **Small modules** (< 200 lines each)
- **Single responsibility** per module
- **Clear interfaces** between components
- **Zero dependencies** between plasticity modules

### 🚀 **RTOS-Friendly**
- **Zero memory allocation** during runtime
- **Deterministic execution** paths
- **Pre-allocated buffers** for all operations
- **No blocking operations**
- **Configurable update intervals**

### 🦀 **Rust Migration Ready**
- **FFI-compatible** data structures
- **Primitive types** only in interfaces
- **Vectorized operations** for SIMD
- **No Python objects** in hot paths
- **Clear ownership** boundaries

## Module Structure

```
feagi/npu/plasticity/
├── __init__.py          # Public API exports
├── core.py              # Core plasticity operations (STP, LTP, LTD)
├── pruning.py           # Synaptic pruning and maintenance
├── homeostatic.py       # Homeostatic plasticity for stability
├── manager.py           # High-level coordination and scheduling
└── README.md            # This documentation
```

## Core Modules

### 1. **core.py** - Fundamental Plasticity Operations
**Size**: 185 lines | **Purpose**: Core plasticity algorithms

**Key Functions**:
- `stp_weight_update()` - Short-term plasticity (multiplicative)
- `ltp_weight_update()` - Long-term potentiation (additive)
- `ltd_weight_update()` - Long-term depression (subtractive)
- `batch_plasticity_update()` - Vectorized batch processing

**Design Features**:
- Pure functions (no side effects)
- Vectorized NumPy operations
- Pre-allocated output arrays
- SIMD-friendly memory layout

### 2. **pruning.py** - Synaptic Maintenance
**Size**: 142 lines | **Purpose**: Synapse pruning and cleanup

**Key Functions**:
- `prune_by_weight()` - Remove weak synapses
- `prune_by_activity()` - Remove inactive synapses
- `prune_by_age()` - Remove old unused synapses
- `combined_pruning()` - Multi-criteria pruning
- `compact_synapse_arrays()` - Array compaction

**Design Features**:
- In-place operations where possible
- Vectorized mask operations
- Activity tracking integration
- Configurable thresholds

### 3. **homeostatic.py** - Network Stability
**Size**: 178 lines | **Purpose**: Homeostatic plasticity mechanisms

**Key Functions**:
- `synaptic_scaling()` - Activity-dependent scaling
- `weight_normalization()` - Total strength normalization
- `intrinsic_excitability_update()` - Threshold adaptation
- `homeostatic_update_batch()` - Coordinated homeostatic updates

**Design Features**:
- Maintains network stability
- Prevents runaway excitation
- Local computation only
- Configurable target activity

### 4. **manager.py** - Coordination and Scheduling
**Size**: 267 lines | **Purpose**: High-level plasticity coordination

**Key Features**:
- **Scheduling**: Configurable update intervals
- **Coordination**: Manages all plasticity types
- **Statistics**: Performance tracking
- **Configuration**: Runtime parameter control

**Design Features**:
- Lightweight coordinator
- Non-blocking operation
- Pre-allocated buffers
- Configurable schedules

## Integration with NeuralProcessor

### **Primary Ownership Model**
```python
class NeuralProcessor:
    def __init__(self, max_neurons, max_synapses, backend):
        # NPU OWNS plasticity operations
        self.plasticity_manager = PlasticityManager(max_synapses, config)
        
        # NPU OWNS synapse data structures with plasticity fields
        self.synapses = NPUSynapseArray(max_synapses, backend)
        # - plasticity_types: PlasticityType enum
        # - plasticity_coeffs: Plasticity coefficients
        # - decay_rates: Decay rates for plasticity
        # - scaling_exponents: Scaling exponents
```

### **Runtime Integration**
```python
def process_neural_burst(self, timestep: int) -> List[int]:
    # PHASE 1: Neural updates and firing
    fired_neurons = self.neurons.neural_update_simd(timestep)
    
    # PHASE 2: Synaptic propagation
    self.synapses.propagate_simd(fired_neurons, self.neurons.membrane_potentials)
    
    # PHASE 3: Plasticity updates (NPU-owned)
    if fired_neurons:
        # Update activity tracking (every timestep)
        self.plasticity_manager.update_activity_tracking(
            fired_neurons, self.synapses.source_neurons, 
            self.synapses.target_neurons, timestep
        )
        
        # Scheduled plasticity updates
        plasticity_updated = self.plasticity_manager.update_plasticity(
            timestep, self.synapses.weights, self.synapses.plasticity_types,
            self.synapses.plasticity_coeffs, activity_factors, 
            self.synapses.decay_rates, self.synapses.scaling_exponents, dt
        )
```

## Configuration

### **PlasticityConfig**
```python
@dataclass
class PlasticityConfig:
    # Update schedules (in timesteps)
    plasticity_update_interval: int = 1      # Every timestep
    pruning_update_interval: int = 100       # Every 100 timesteps  
    homeostatic_update_interval: int = 1000  # Every 1000 timesteps
    
    # Plasticity parameters
    stp_enabled: bool = True
    ltp_enabled: bool = True
    ltd_enabled: bool = True
    homeostatic_enabled: bool = True
    
    # Pruning thresholds
    weight_prune_threshold: float = 0.01
    activity_prune_threshold: float = 0.001
    age_prune_threshold: int = 10000
    
    # Homeostatic parameters
    target_activity: float = 0.1
    scaling_rate: float = 0.01
    threshold_adaptation_rate: float = 0.001
```

## Performance Characteristics

### **Memory Usage**
- **Zero allocation** during runtime
- **Pre-allocated buffers** for all operations
- **Structure of Arrays** for cache efficiency
- **Vectorized operations** for SIMD

### **Computational Complexity**
- **Plasticity updates**: O(N) where N = active synapses
- **Pruning operations**: O(N) where N = total synapses
- **Homeostatic updates**: O(N + M) where N = synapses, M = neurons
- **Activity tracking**: O(F) where F = firing neurons

### **Scheduling Overhead**
- **Minimal**: Simple interval-based scheduling
- **Configurable**: Adjust update frequencies
- **Non-blocking**: Never blocks neural processing

## Rust Migration Path

### **Phase 1: FFI Preparation** (Current)
- ✅ Primitive types in interfaces
- ✅ Vectorized operations
- ✅ Pre-allocated arrays
- ✅ No Python objects in hot paths

### **Phase 2: Rust Implementation**
```rust
// Core plasticity operations
pub fn stp_weight_update(
    current_weights: &[f32],
    plasticity_coeffs: &[f32],
    activity_factors: &[f32],
    decay_rates: &[f32],
    scaling_exponents: &[f32],
    dt: f32,
    output_weights: &mut [f32]
) -> usize;

// Plasticity manager
pub struct PlasticityManager {
    config: PlasticityConfig,
    temp_buffers: PlasticityBuffers,
    statistics: PlasticityStats,
}
```

### **Phase 3: Python FFI Bindings**
```python
# Python wrapper for Rust implementation
from feagi.rust.plasticity import (
    rust_stp_update, rust_ltp_update, rust_ltd_update,
    RustPlasticityManager
)
```

## Usage Examples

### **Basic Usage**
```python
from feagi.npu.plasticity import PlasticityManager, PlasticityConfig

# Initialize plasticity system
config = PlasticityConfig(
    plasticity_update_interval=1,
    pruning_update_interval=100,
    homeostatic_update_interval=1000
)
plasticity = PlasticityManager(max_synapses=1_000_000, config=config)

# Update plasticity during neural processing
updated_count = plasticity.update_plasticity(
    timestep=current_timestep,
    weights=synapse_weights,
    plasticity_types=plasticity_types,
    plasticity_coeffs=coefficients,
    activity_factors=activity_levels,
    decay_rates=decay_rates,
    scaling_exponents=exponents,
    dt=0.001
)
```

### **Advanced Configuration**
```python
# Custom plasticity configuration
config = PlasticityConfig(
    # High-frequency plasticity updates
    plasticity_update_interval=1,
    
    # Conservative pruning
    pruning_update_interval=1000,
    weight_prune_threshold=0.001,
    
    # Active homeostatic regulation
    homeostatic_update_interval=100,
    target_activity=0.15,
    scaling_rate=0.02
)
```

## Benefits

### **🎯 Modularity**
- **Easy to test** - each module is independent
- **Easy to maintain** - clear separation of concerns
- **Easy to extend** - add new plasticity types easily
- **Easy to debug** - isolated functionality

### **🚀 Performance**
- **Zero allocation** - no runtime memory allocation
- **Vectorized** - SIMD-optimized operations
- **Cache-friendly** - Structure of Arrays layout
- **Configurable** - adjust update frequencies

### **🦀 Rust Ready**
- **FFI compatible** - primitive types only
- **Deterministic** - predictable execution paths
- **RTOS friendly** - no blocking operations
- **Clear ownership** - NPU owns all plasticity data

### **🧠 Biological Accuracy**
- **STP/LTP/LTD** - standard plasticity mechanisms
- **Homeostatic** - maintains network stability
- **Activity-dependent** - realistic activity tracking
- **Configurable** - match biological parameters

## Conclusion

This modular plasticity system provides a complete, high-performance solution for synaptic plasticity in FEAGI 2.0. The design prioritizes modularity, performance, and Rust migration readiness while maintaining biological accuracy and configurability.

The system successfully moves all plasticity operations from BDU to NPU, establishing NPU as the primary owner of all runtime neural processing operations, including plasticity, while maintaining clean interfaces and high performance.
