# Rust NPU Synapse Reload Architecture

## Problem Statement

The Rust NPU migration introduced a **regression** in dynamic synapse addition behavior compared to the previous Python NPU implementation.

## Architectural Difference

### Old Python NPU (commit 80f6960d)
```python
def _compute_synaptic_propagation(self):
    # Get NPU interface for synapse data access
    npu_interface = getattr(self.connectome_manager, '_npu_interface', None)
    synapse_array = getattr(npu_interface, 'synapse_array', None)
    # ... accesses synapse_array DIRECTLY during each burst
```

**Behavior:**
- ✅ Accessed `synapse_array` **LIVE during every burst**
- ✅ New synapses added via UI were **immediately active** in the next burst
- ✅ No restart required

### New Rust NPU (current)
```rust
// Loads synapses once at initialization
fn initialize(&mut self) {
    self._load_synapses();
    self._rust_npu.rebuild_indexes();
}
```

**Behavior:**
- ❌ Loads synapses **once at initialization** into Rust data structures
- ❌ Uses internal synapse index for fast lookup
- ❌ Does NOT see new synapses added to `synapse_array` after initialization
- ❌ Required restart to pick up new synapses

## Root Cause

The Rust NPU optimizes for **performance** by:
1. Loading synapses once
2. Building a fast lookup index (AHashMap) in Rust
3. Avoiding repeated Python→Rust boundary crossings during bursts

**Trade-off:** This breaks the "live" behavior of the Python NPU where `synapse_array` changes were immediately visible.

## Solution

### Automatic Synapse Index Reload

**Implementation:** Auto-reload triggers in THREE places:

#### 1. Batch Synapse Creation API
**Location:** `core_api_service.py` - `batch_create_synapses()`

After synapses are successfully created via batch API:
1. Check if Rust NPU is initialized
2. Call `burst_engine.reinitialize_rust_npu()`
3. Rust NPU reloads all synapses from `synapse_array`
4. Rebuilds the synapse index
5. **Safety:** If reinitialization fails, restore the old state

**Code Location:** `feagi_core/feagi/api/core/services/core_api_service.py:3948-3972`

#### 2. Morphology Mapping Changes
**Location:** `genome_service.py` - `update_cortical_mapping_properties()`

After morphology mappings are updated (which creates/deletes synapses):
1. Check if Rust NPU is initialized
2. Call `burst_engine.reinitialize_rust_npu()`
3. Rust NPU reloads all synapses from `synapse_array`
4. Rebuilds the synapse index
5. **Safety:** If reinitialization fails, restore the old state

**Code Location:** `feagi_core/feagi/api/core/services/genome/genome_service.py:3643-3663`

#### 3. Neuroembryogenesis Synapse Creation
**Location:** `neuroembryogenesis.py` - `update_cortical_mapping()`

After morphology-based synaptogenesis creates synapses:
1. Check if Rust NPU is initialized
2. Call `burst_engine.reinitialize_rust_npu()`
3. Rust NPU reloads all synapses from `synapse_array`
4. Rebuilds the synapse index
5. **Safety:** If reinitialization fails, restore the old state

**Code Location:** `feagi_core/feagi/bdu/embryogenesis/neuroembryogenesis.py:2918-2938`

**Why Three Paths?** The Brain Visualizer UI can create synapses via:
- Direct batch API calls (programmatic)
- Morphology/mapping system via genome service (UI connection editor high-level)
- Neuroembryogenesis direct morphology application (UI connection editor low-level)

### Safety Mechanisms

1. **Check if initialized:** Only reinitialize if `_rust_npu_integration is not None`
2. **Fail-safe:** `reinitialize_rust_npu()` keeps old integration and restores it if reload fails
3. **Non-blocking:** Synapse creation succeeds even if Rust NPU reload fails
4. **Logging:** Clear diagnostic messages about reload status

## Testing

### Before Fix
1. Start FEAGI + load genome
2. Power neuron fires
3. Add synapse: `_power` → `iic000` via UI
4. **Result:** Power neuron fires, but NO synaptic propagation to `iic000`
5. **Logs:**
   ```
   🦀 [RUST-NPU] Loaded 1552 synapses (0 from power neuron)
   ```

### After Fix
1. Start FEAGI + load genome
2. Power neuron fires
3. Add synapse: `_power` → `iic000` via UI
4. **Expected:**
   ```
   🦀 [RUST-NPU] Synapse(s) added - reloading synapse index from synapse_array...
   🦀 [RUST-NPU] Loading 500000 synapses from array...
   🦀 [RUST-NPU] Power synapse #1: source=2 → target=16439, weight=128, conductance=255, type=0
   🦀 [RUST-NPU] Loaded 1553 synapses (1 from power neuron)
   🦀 [RUST-NPU] ✅ Synapse index reloaded successfully
   ```
5. **Result:** Power neuron fires → synaptic propagation → `iic000` neuron 16439 fires!

## Future Optimization

**Option 1: Incremental Update (Rust)**
- Instead of full reload, add individual synapses to Rust NPU
- Requires new Rust API: `RustNPU.add_synapse()`
- More complex but more efficient

**Option 2: Live Synapse Array Access (Rust)**
- Rust NPU accesses Python `synapse_array` directly during bursts
- Requires careful PyO3 object lifetime management
- May introduce Python→Rust boundary overhead

**Current Approach:** Full reload is acceptable for now as synapse additions are relatively rare operations compared to burst frequency.

## Related Files
- `feagi_core/feagi/api/core/services/core_api_service.py` - Auto-reload trigger
- `feagi_core/feagi/npu/burst_engine.py` - Reinitialization logic
- `feagi_core/feagi/npu/rust_npu_integration.py` - Synapse loading
- `feagi_core/feagi-rust/crates/feagi-python/src/lib.rs` - Rust NPU bindings
