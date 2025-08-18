# GPU Performance Findings and High-Impact Fixes

Last Updated: ${DATE}

## Summary

Observed GPU performance equal to or below CPU. Root causes are predominantly data movement, per-burst setup overhead, gating that prevents GPU usage, and partial offload. Below we list the main issues and the top three actions applied to gain immediate speedups.

## Key Technical Findings

1. Excessive CPU↔GPU transfers each burst
   - Conversion to NumPy (`to_numpy`) before GPU usage and synchronous readbacks after kernels keep data resident on CPU, defeating residency benefits.
   - Example: `feagi/bdu/webgpu_integration.py` converts neuron arrays to NumPy; `array_backend` writes buffers and maps back every burst.

2. Per-burst pipeline/bind-group/uniform creation
   - Compute pipelines and bind group layouts are recreated each burst; pipeline creation is expensive and should be cached.

3. Hybrid gating disables GPU frequently
   - Default `gpu_threshold = 1_000_000` synapses forces CPU path for typical per-burst workloads.

4. CPU-heavy fanout assembly before GPU
   - Python loops/dicts used to gather `all_target_neurons` and weights per burst dominate CPU time.

5. Partial offload
   - GPU only performs synaptic scatter-add; thresholding/refractory updates run on CPU, limiting end-to-end gain.

6. Synchronous readbacks stall
   - `map_sync` and per-burst readback enforce barrier synchronization; no overlap between compute and transfers.

7. Fixed-point conversions for atomics on CPU side
   - Per-burst float↔int32 conversions add memory/compute overhead.

8. High logging overhead in hot paths
   - Info-level logs in per-burst paths reduce throughput.

## Top 3 Immediate Improvements Implemented

1. Prefer WGPU when backend is `auto` in `GlobalSynapseArray`
   - Ensures the optimized WGPU path is selected where available instead of defaulting to CPU or non-optimized GPU paths.

2. Cache WGPU pipeline and bind group layout in `ArrayBackend`
   - Avoids recreating pipeline/bind-group layout every burst; reduces significant setup overhead.

3. Reduce logging level in hot GPU paths
   - Convert noisy info-level logs to debug to minimize per-burst overhead.

## Next Steps (High ROI)

- Lower `gpu_threshold` via config to enable GPU for realistic workloads.
- Maintain device residency and eliminate per-burst readbacks; read back only when streams require it.
- Move fanout preparation to GPU-friendly CSR/COO and keep on device.
- Consider fusing threshold/refractory updates into GPU kernels to increase offloaded fraction.

## References

- `feagi/bdu/models/array_backend.py`
- `feagi/bdu/synapse_array.py`
- `feagi/bdu/webgpu_integration.py`


