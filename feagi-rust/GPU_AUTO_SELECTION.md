# GPU Backend Auto-Selection

## Overview
FEAGI Rust NPU now supports automatic backend selection between CPU and GPU based on genome size and hardware availability.

## Auto-Selection (Default)
```python
# Python - auto-selects optimal backend
rust_npu = RustNPUIntegration(connectome_manager)  # backend="auto" (default)
```

System automatically:
1. Checks genome size (neurons, synapses)
2. Checks GPU availability
3. Estimates speedup
4. Chooses GPU only if >1.5x faster

## Thresholds
- **GPU considered**: >500K neurons OR >50M synapses
- **GPU used**: Only if estimated speedup >1.5x
- **Fallback**: Always falls back to CPU if GPU unavailable

## Configuration
```rust
BackendConfig {
    gpu_neuron_threshold: 500_000,
    gpu_synapse_threshold: 50_000_000,
    force_cpu: false,  // Override to force CPU
    force_gpu: false,  // Override to force GPU (testing)
}
```

## Expected Performance
| Neurons | Backend | Speedup | Decision |
|---------|---------|---------|----------|
| <100K | CPU | 1.0x | Too small |
| 500K | GPU | 2-3x | Crossover |
| 1M | GPU | 5-10x | Beneficial |
| 5M+ | GPU | 20-50x | Ideal |

## Validation
```bash
# Run benchmark suite
cd feagi_core/feagi-rust
cargo bench --bench backend_comparison

# Run tests
cargo test --test backend_selection_test --features gpu -- --nocapture
```

## Current Status
✅ Auto-selection logic
✅ Threshold configuration
✅ CPU backend (functional)
✅ GPU backend (skeleton)
✅ Benchmark suite
✅ Test suite

⏳ WGSL shaders (in progress)
⏳ GPU buffer management
⏳ Full GPU pipeline (3-4 weeks)

