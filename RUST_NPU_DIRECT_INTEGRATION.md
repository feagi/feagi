# Rust NPU Direct Integration (Option B)

**Status**: 🚧 **IN PROGRESS**  
**Approach**: Direct replacement - No fallbacks, production-ready

---

## 🎯 **Integration Philosophy**

> "We need a commercial-ready product that is robust and predictable. We cannot afford unpredictable fallbacks."

**Design Principles**:
- ✅ **Fail Fast**: If Rust NPU fails to initialize, raise clear error
- ✅ **No Fallbacks**: Single code path - Rust only
- ✅ **Predictable**: Deterministic behavior
- ✅ **Production-Ready**: Proper error handling and logging
- ✅ **Clean**: Remove Python burst processing code

---

## 📋 **Implementation Checklist**

### **Phase 1: Core Integration**
- ⏳ Update `__init__` to initialize Rust NPU immediately
- ⏳ Replace `_initialize_rust_engine` with `_initialize_rust_npu`
- ⏳ Replace `process_burst()` to call Rust NPU directly
- ⏳ Remove Python burst processing phases (keep as documentation)

### **Phase 2: Data Loading**
- ⏳ Implement `_load_connectome_into_rust_npu()`
- ⏳ Load all neurons from connectome_manager
- ⏳ Load all synapses from connectome_manager
- ⏳ Build neuron→cortical_area mapping
- ⏳ Rebuild indexes

### **Phase 3: Integration Points**
- ⏳ Keep FQ Sampler integration (publish firing to visualization)
- ⏳ Keep State Manager integration (burst state tracking)
- ⏳ Keep Injection Service integration (power neurons)
- ⏳ Keep Performance monitoring (burst frequency tracking)

### **Phase 4: Genome Integration**
- ⏳ Update `update_with_genome()` to reload Rust NPU
- ⏳ Handle dynamic synapse modifications
- ⏳ Test with `essential_genome.json`

---

## 🏗️ **Code Structure**

### **Key Changes in `burst_engine.py`**

```python
class BurstEngine:
    def __init__(self, connectome_manager=None, ...):
        # ... (existing initialization)
        
        # RUST NPU INITIALIZATION (CRITICAL PATH!)
        self._rust_npu = None
        self._rust_npu_initialized = False
        
        if not RUST_AVAILABLE:
            raise RuntimeError(
                "🦀 [RUST-NPU] CRITICAL: Rust NPU not available! "
                "FEAGI requires the Rust NPU for production use. "
                "Please build the Rust components: cd feagi-rust && cargo build --release"
            )
        
        logger.info("🦀 [RUST-NPU] Rust NPU available - will initialize after genome load")
    
    def _initialize_rust_npu(self) -> None:
        """Initialize the complete Rust NPU with connectome data.
        
        CRITICAL: This must succeed or we fail fast with a clear error.
        NO FALLBACKS - production-ready or nothing.
        """
        if self._rust_npu_initialized:
            return
        
        if not self.connectome_manager:
            raise RuntimeError(
                "🦀 [RUST-NPU] CRITICAL: Cannot initialize without connectome_manager"
            )
        
        logger.info("🦀 [RUST-NPU] Initializing with connectome data...")
        
        # Get neuron and synapse counts with headroom for dynamic additions
        neuron_count = self._get_neuron_count()
        synapse_count = self._get_synapse_count()
        
        neuron_capacity = max(neuron_count * 2, 10000)  # 2x headroom
        synapse_capacity = max(synapse_count * 2, 100000)  # 2x headroom
        
        logger.info("🦀 [RUST-NPU] Creating NPU: %d neurons (capacity: %d), %d synapses (capacity: %d)",
                    neuron_count, neuron_capacity, synapse_count, synapse_capacity)
        
        # Create Rust NPU
        self._rust_npu = feagi_rust.RustNPU(
            neuron_capacity=neuron_capacity,
            synapse_capacity=synapse_capacity,
            fire_ledger_window=self.fire_ledger.window_size
        )
        
        # Load connectome into Rust NPU
        self._load_connectome_into_rust_npu()
        
        self._rust_npu_initialized = True
        
        logger.info("🦀 [RUST-NPU] ✅ Initialization complete: %d neurons, %d synapses loaded",
                    self._rust_npu.get_neuron_count(),
                    self._rust_npu.get_synapse_count())
    
    def _load_connectome_into_rust_npu(self) -> None:
        """Load all neurons and synapses from connectome_manager into Rust NPU."""
        import time
        load_start = time.perf_counter()
        
        # Load neurons
        neurons_loaded = 0
        for neuron_id, neuron in self._iterate_all_neurons():
            self._rust_npu.add_neuron(
                threshold=neuron.get('threshold', 1.0),
                leak_rate=neuron.get('leak_rate', 0.0),
                refractory_period=neuron.get('refractory_period', 0),
                excitability=neuron.get('excitability', 1.0),
                cortical_area=neuron.get('cortical_area_id', 0),
                x=neuron.get('coordinates', [0, 0, 0])[0],
                y=neuron.get('coordinates', [0, 0, 0])[1],
                z=neuron.get('coordinates', [0, 0, 0])[2]
            )
            neurons_loaded += 1
        
        logger.info("🦀 [RUST-NPU] Loaded %d neurons", neurons_loaded)
        
        # Load synapses
        synapses_loaded = 0
        for synapse in self._iterate_all_synapses():
            self._rust_npu.add_synapse(
                source=synapse.get('source_neuron_id'),
                target=synapse.get('target_neuron_id'),
                weight=synapse.get('weight', 128),
                conductance=synapse.get('conductance', 255),
                synapse_type=0 if synapse.get('type') == 'excitatory' else 1
            )
            synapses_loaded += 1
        
        logger.info("🦀 [RUST-NPU] Loaded %d synapses", synapses_loaded)
        
        # Rebuild indexes (CRITICAL for performance!)
        self._rust_npu.rebuild_indexes()
        logger.info("🦀 [RUST-NPU] Indexes rebuilt")
        
        # Set neuron→cortical_area mapping
        neuron_mapping = {}
        for neuron_id, neuron in self._iterate_all_neurons():
            neuron_mapping[neuron_id] = neuron.get('cortical_area_id', 0)
        
        self._rust_npu.set_neuron_mapping(neuron_mapping)
        logger.info("🦀 [RUST-NPU] Neuron mapping set (%d neurons mapped)", len(neuron_mapping))
        
        load_time = (time.perf_counter() - load_start) * 1000
        logger.info("🦀 [RUST-NPU] Connectome load complete in %.2f ms", load_time)
    
    def process_burst(self) -> List[int]:
        """Process a single burst using Rust NPU.
        
        PRODUCTION-READY: All burst processing happens in Rust for maximum performance.
        
        Returns:
            List[int]: Neuron IDs that fired in this burst
        """
        # Initialize Rust NPU on first use
        if not self._rust_npu_initialized:
            self._initialize_rust_npu()
        
        # Get power neurons from injection service
        power_neurons = self._get_power_neurons()
        
        # Call Rust NPU (THIS IS THE FAST PATH - ALL IN RUST!)
        import time
        burst_start = time.perf_counter()
        
        result = self._rust_npu.process_burst(power_neurons=power_neurons)
        
        burst_time = (time.perf_counter() - burst_start) * 1000
        
        # Update internal state
        self.burst_count = result.burst
        self.current_timestep += 1
        
        # Track burst timing for performance monitoring
        self._track_burst_performance(burst_start, burst_time, result)
        
        # Publish to FQ samplers (for visualization)
        if result.neuron_count > 0:
            self._publish_to_samplers(result.fired_neurons)
        
        # Performance logging every 100 bursts
        if result.burst % 100 == 0:
            self._log_performance_metrics(result, burst_time)
        
        return result.fired_neurons
    
    def _track_burst_performance(self, burst_start_time, burst_time_ms, result):
        """Track burst performance metrics."""
        # ... (existing performance tracking code)
        pass
    
    def _log_performance_metrics(self, result, burst_time_ms):
        """Log performance metrics."""
        if len(self._burst_times) > 0:
            avg_interval = sum(self._burst_times) / len(self._burst_times)
            actual_hz = 1.0 / avg_interval if avg_interval > 0 else 0.0
            
            if actual_hz < self.desired_frequency * 0.9:
                logger.warning(
                    "⚠️ [RUST-NPU] PERFORMANCE: Burst #%d | "
                    "Desired: %.2f Hz | Actual: %.2f Hz (%.1f%%) | "
                    "Processing: %.2f ms | Neurons fired: %d",
                    result.burst, self.desired_frequency, actual_hz,
                    (actual_hz / self.desired_frequency * 100.0),
                    burst_time_ms, result.neuron_count
                )
            else:
                logger.info(
                    "🦀 [RUST-NPU] Burst #%d: %.2f Hz | %.2f ms | %d neurons fired | "
                    "power: %d, synaptic: %d, refractory: %d",
                    result.burst, actual_hz, burst_time_ms, result.neuron_count,
                    result.power_injections, result.synaptic_injections,
                    result.neurons_in_refractory
                )
```

---

## 🔧 **Helper Methods**

```python
def _get_neuron_count(self) -> int:
    """Get neuron count from connectome_manager."""
    # Implementation depends on connectome_manager structure
    pass

def _get_synapse_count(self) -> int:
    """Get synapse count from connectome_manager."""
    # Implementation depends on connectome_manager structure
    pass

def _iterate_all_neurons(self):
    """Iterate over all neurons in connectome_manager."""
    # Implementation depends on connectome_manager structure
    pass

def _iterate_all_synapses(self):
    """Iterate over all synapses in connectome_manager."""
    # Implementation depends on connectome_manager structure
    pass

def _get_power_neurons(self) -> List[int]:
    """Get power neurons from injection service."""
    if self.injection_service:
        return self.injection_service.get_power_neurons()
    return []

def _publish_to_samplers(self, fired_neurons: List[int]):
    """Publish fired neurons to FQ samplers for visualization."""
    if self.fq_sampler:
        self.fq_sampler.sample_fire_queue(fired_neurons)
```

---

## 📊 **Error Handling Strategy**

### **Initialization Errors** (Fail Fast)
```python
if not RUST_AVAILABLE:
    raise RuntimeError("Rust NPU required but not available")

if not self.connectome_manager:
    raise RuntimeError("Cannot initialize Rust NPU without connectome_manager")
```

### **Runtime Errors** (Log and Re-raise)
```python
try:
    result = self._rust_npu.process_burst(power_neurons=power_neurons)
except Exception as e:
    logger.error("🦀 [RUST-NPU] CRITICAL: Burst processing failed: %s", str(e))
    raise  # Re-raise for caller to handle
```

### **No Silent Failures**
- Every error is logged with 🦀 [RUST-NPU] prefix
- Every error is re-raised (no swallowing)
- Clear, actionable error messages

---

## ✅ **Success Criteria**

1. ✅ **Compiles**: Code runs without syntax errors
2. ⏳ **Initializes**: Rust NPU loads connectome successfully
3. ⏳ **Processes**: Bursts process without errors
4. ⏳ **Performance**: 50-100x faster than Python
5. ⏳ **Correctness**: Firing patterns are valid
6. ⏳ **Stable**: No crashes over 1000+ bursts

---

**Next**: Implement the changes in `burst_engine.py` 🚀
