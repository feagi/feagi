# Rust NPU Integration Plan

**Status**: 🔄 **AWAITING APPROVAL**  
**Goal**: Integrate the complete Rust NPU into `burst_engine.py` for 50-100x performance improvement

---

## ✅ **What's Ready**

1. ✅ **Complete Rust NPU** - All burst phases implemented and tested
2. ✅ **Python Bindings** - PyO3 bindings working perfectly
3. ✅ **Integration Test** - Simple test confirms bindings work
4. ⏳ **Production Integration** - Needs approval to proceed

---

## 🎯 **Integration Strategy**

### **Option A: Opt-In Rust NPU (RECOMMENDED)**

**Approach**: Add Rust NPU as an **optional high-performance backend**, keeping Python as default.

#### **Pros**:
- ✅ Zero risk to existing functionality
- ✅ Easy rollback if issues arise
- ✅ Can compare performance side-by-side
- ✅ Gradual migration path
- ✅ Follows FEAGI design principles

#### **Cons**:
- ⚠️ Maintains two code paths temporarily
- ⚠️ Requires testing both paths

#### **Implementation**:
```python
class BurstEngine:
    def __init__(self, ...):
        # Add Rust NPU flag
        self.use_rust_npu = False  # Default: Python
        self._rust_npu = None
        self._rust_npu_initialized = False
        
        # Try to enable Rust NPU if available
        if RUST_AVAILABLE and os.getenv("FEAGI_USE_RUST_NPU", "0") == "1":
            self.use_rust_npu = True
            logger.info("🦀 [RUST-NPU] Rust NPU enabled via environment variable")
    
    def _initialize_rust_npu(self) -> bool:
        """Initialize the complete Rust NPU with connectome data."""
        if not RUST_AVAILABLE:
            return False
        
        # Get neuron/synapse counts
        neuron_count = self.connectome_manager.get_neuron_count()
        synapse_count = self.connectome_manager.get_synapse_count()
        
        # Create Rust NPU
        self._rust_npu = feagi_rust.RustNPU(
            neuron_capacity=neuron_count * 2,  # Some headroom
            synapse_capacity=synapse_count * 2,
            fire_ledger_window=20
        )
        
        # Load all neurons
        for neuron_id, neuron in self.connectome_manager.get_all_neurons():
            self._rust_npu.add_neuron(
                threshold=neuron.threshold,
                leak_rate=neuron.leak_rate,
                refractory_period=neuron.refractory_period,
                excitability=neuron.excitability,
                cortical_area=neuron.cortical_area_id,
                x=neuron.coordinates[0],
                y=neuron.coordinates[1],
                z=neuron.coordinates[2]
            )
        
        # Load all synapses
        for synapse in self.connectome_manager.get_all_synapses():
            self._rust_npu.add_synapse(
                source=synapse.source_neuron_id,
                target=synapse.target_neuron_id,
                weight=synapse.weight,
                conductance=synapse.conductance,
                synapse_type=0 if synapse.type == "excitatory" else 1
            )
        
        # Rebuild indexes
        self._rust_npu.rebuild_indexes()
        
        # Set neuron mapping
        neuron_mapping = {}
        for neuron_id, neuron in self.connectome_manager.get_all_neurons():
            neuron_mapping[neuron_id] = neuron.cortical_area_id
        self._rust_npu.set_neuron_mapping(neuron_mapping)
        
        self._rust_npu_initialized = True
        logger.info("🦀 [RUST-NPU] Initialized with %d neurons and %d synapses",
                    self._rust_npu.get_neuron_count(),
                    self._rust_npu.get_synapse_count())
        
        return True
    
    def process_burst(self) -> List[int]:
        """Execute burst processing - use Rust NPU if enabled, else Python."""
        if self.use_rust_npu:
            # Initialize Rust NPU on first use
            if not self._rust_npu_initialized:
                if not self._initialize_rust_npu():
                    logger.warning("🦀 [RUST-NPU] Initialization failed - falling back to Python")
                    self.use_rust_npu = False
                    return self._process_burst_python()
            
            # Process with Rust NPU
            return self._process_burst_rust()
        else:
            # Process with Python (existing code)
            return self._process_burst_python()
    
    def _process_burst_rust(self) -> List[int]:
        """Process burst using Rust NPU (ALL IN RUST!)."""
        # Get power neurons
        power_neurons = self._get_power_neurons()
        
        # Call Rust NPU (THIS IS THE FAST PATH!)
        result = self._rust_npu.process_burst(power_neurons=power_neurons)
        
        # Update internal state
        self.burst_count = result.burst
        self.previous_fire_queue = result.fired_neurons
        
        # Publish to streams (if needed)
        if self.fq_sampler:
            self._publish_to_sampler(result.fired_neurons)
        
        # Performance logging
        if result.burst % 100 == 0:
            logger.info("🦀 [RUST-NPU] Burst #%d: %d neurons fired (power: %d, synaptic: %d)",
                        result.burst, result.neuron_count,
                        result.power_injections, result.synaptic_injections)
        
        return result.fired_neurons
    
    def _process_burst_python(self) -> List[int]:
        """Process burst using Python (existing implementation)."""
        return self._process_burst()  # Existing Python code
```

---

### **Option B: Replace Python NPU Entirely**

**Approach**: Directly replace Python burst processing with Rust NPU.

#### **Pros**:
- ✅ Maximum performance immediately
- ✅ Clean codebase (no dual paths)
- ✅ Aligns with "move entire FEAGI to Rust" goal

#### **Cons**:
- ⚠️ Higher risk (no easy rollback)
- ⚠️ Need to ensure all edge cases work
- ⚠️ May break existing integrations

---

## 📊 **Comparison**

| Aspect | Option A (Opt-In) | Option B (Replace) |
|--------|-------------------|---------------------|
| **Risk** | ✅ Low (fallback available) | ⚠️ Medium (no fallback) |
| **Performance** | ✅ 50-100x when enabled | ✅ 50-100x always |
| **Migration** | ✅ Gradual | ⚠️ Immediate |
| **Testing** | ⏳ Both paths | ✅ Single path |
| **Rollback** | ✅ Easy (disable flag) | ⚠️ Hard (revert code) |
| **Code Complexity** | ⚠️ Two paths temporarily | ✅ Single path |

---

## 🎯 **Recommended Approach**

**Option A: Opt-In Rust NPU** with environment variable `FEAGI_USE_RUST_NPU=1`

### **Why?**
1. **Safety First**: Zero risk to production systems
2. **Validation**: Can compare Python vs Rust results side-by-side
3. **Debugging**: Easy to switch back if issues arise
4. **FEAGI Philosophy**: Incremental, validated changes

### **Migration Path**:
```
Phase 1: Opt-in (1-2 weeks validation)
  ├─ Add Rust NPU as optional backend
  ├─ Test with real genomes
  ├─ Benchmark performance
  └─ Identify and fix any edge cases

Phase 2: Default enabled (1 week validation)
  ├─ Make Rust NPU default (but keep Python fallback)
  ├─ Monitor production usage
  └─ Address any issues

Phase 3: Python removal (future)
  ├─ Remove Python burst processing
  ├─ Clean up codebase
  └─ Rust-only NPU
```

---

## 📋 **Integration Checklist**

### **Code Changes**
- ⏳ Add `use_rust_npu` flag to `BurstEngine.__init__()`
- ⏳ Implement `_initialize_rust_npu()` method
- ⏳ Implement `_process_burst_rust()` method
- ⏳ Update `process_burst()` to route to Rust or Python
- ⏳ Add environment variable check (`FEAGI_USE_RUST_NPU`)

### **Testing**
- ⏳ Load `essential_genome.json` with Rust NPU
- ⏳ Process 1000+ bursts and verify correctness
- ⏳ Compare Python vs Rust firing patterns
- ⏳ Benchmark performance (expect 50-100x speedup)
- ⏳ Test dynamic synapse modifications

### **Documentation**
- ⏳ Update `burst_engine.py` docstrings
- ⏳ Add Rust NPU usage guide
- ⏳ Document environment variables
- ⏳ Add performance benchmarks

---

## 🚀 **Expected Performance**

### **Current** (Python)
```
Burst #100:
  - Processing time: 47ms
  - Frequency: 7.67 Hz (51% of 15Hz target)
  - Synaptic propagation: 165ms
  - Neural dynamics: 12ms
```

### **Expected** (Rust NPU)
```
Burst #100:
  - Processing time: <5ms
  - Frequency: 30-60 Hz (200-400% of target!)
  - Synaptic propagation: <0.5ms
  - Neural dynamics: <2ms
```

**Improvement**: **50-100x faster** 🚀

---

## ⚠️ **Potential Issues & Mitigations**

### **Issue 1**: Neuron/Synapse Data Format Mismatch
**Mitigation**: Comprehensive data validation during initialization, with detailed error messages.

### **Issue 2**: Missing Features in Rust NPU
**Mitigation**: Keep Python fallback available during validation phase.

### **Issue 3**: Performance Bottleneck Outside NPU
**Mitigation**: Profile entire burst loop, not just NPU processing.

### **Issue 4**: Memory Overhead
**Mitigation**: Monitor memory usage during testing. Rust NPU should use LESS memory due to Structure-of-Arrays layout.

---

## 🔍 **Success Criteria**

1. ✅ **Compiles**: Code builds without errors
2. ⏳ **Functional**: Processes bursts correctly with real genome
3. ⏳ **Performance**: 50-100x faster than Python
4. ⏳ **Correctness**: Firing patterns match Python reference
5. ⏳ **Stable**: No crashes or memory leaks over 10,000+ bursts
6. ⏳ **Dynamic**: Synapse add/remove/update works correctly

---

## 🎯 **Next Steps** (Pending Approval)

1. **Implement Option A** (Opt-in Rust NPU)
2. **Test with essential_genome.json**
3. **Benchmark performance**
4. **Validate correctness**
5. **Document results**

---

## ❓ **Questions for Review**

1. **Do you approve Option A (Opt-in with fallback)?**
   - Or would you prefer Option B (direct replacement)?

2. **Any specific edge cases or features to test?**
   - Special neuron types?
   - Plasticity integration?
   - Real-time constraints?

3. **What validation criteria are most important?**
   - Performance?
   - Correctness?
   - Stability?

---

**Awaiting your approval to proceed with implementation!** 🚀
