# FEAGI Optimized Data Structures

This directory contains SIMD and WebGPU-optimized implementations of the core FEAGI data structures.

## Overview

The Rust implementation provides:

1. **Significant Performance Improvements** - Using SIMD vectorization for CPU operations and WebGPU compute shaders for GPU processing.
2. **Memory-Efficient Storage** - Optimized data layouts for both CPU and GPU processing.
3. **Python Compatibility** - Python bindings allow seamless use from the existing FEAGI codebase.

## Key Components

- **Global Neuron Array (GNA)** - SoA (Structure of Arrays) layout optimized for SIMD operations and GPU memory coalescing
- **Fire Candidate List (FCL)** - Roaring bitmap implementation with SIMD optimizations and flat array GPU representation
- **Connectome** - CSR (Compressed Sparse Row) format optimized for sparse matrix operations on both CPU and GPU
- **WebGPU Compute Shaders** - Efficient GPU implementation of core neural processing operations

## Building

To build the optimized extensions, run:

```bash
python scripts/build_rust_extensions.py
```

This will:
1. Detect your CPU features (AVX2, NEON)
2. Build the Rust code with appropriate optimizations
3. Install the Python extension module

## Usage

In Python code, you can use the optimized structures as follows:

```python
# Enhanced NeuronArray with integrated embedded optimizations
from feagi.bdu.models.neuron import NeuronArray
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.npu.optimized_structures import (
    FireCandidateList,
    Connectome,
    OptimizedFeagiCore,
)

# Create enhanced neuron array with embedded optimizations (automatic)
neuron_array = NeuronArray(max_neurons=100_000)

# Create connectome manager (uses embedded optimizations by default)
connectome_manager = ConnectomeManager(config_or_max_neurons=100_000)

# Get membrane potentials (using integrated optimizations)
potentials = neuron_array.membrane_potentials

# Performance summary shows embedded optimization features
perf_summary = neuron_array.get_performance_summary()
print(f"SIMD enabled: {perf_summary.get('simd_enabled')}")
print(f"Cache alignment: {perf_summary.get('alignment')}B")
```

The `optimized_integration.py` module provides helper functions for integrating these structures into existing code.

## Performance

Benchmarks comparing the optimized implementations to standard Python implementations can be run using:

```bash
python tests/performance/bdu/benchmark_optimized_structures.py
```

Typical performance improvements:
- GNA operations: 10-50x faster
- FCL operations: 5-20x faster
- Connectome propagation: 20-100x faster
- Overall simulation: 15-70x faster

## Implementation Details

### SIMD Optimization

CPU optimization uses explicit SIMD intrinsics via Rust's std::arch module:
- AVX2 on x86_64 platforms
- NEON on ARM64 platforms
- Scalar fallback on other platforms

### WebGPU Integration

GPU processing uses the wgpu crate for WebGPU compute shaders:
- Neuron operations (membrane potential updates, fire candidate detection)
- Connectome propagation (synaptic signal transmission)
- Zero-copy data sharing between CPU and GPU where possible

### Memory Layout

- Data is aligned to cache line boundaries (64 bytes) for optimal SIMD performance
- Structure of Arrays (SoA) layout used for better memory access patterns on both CPU and GPU
- Memory is pre-allocated and reused when possible to minimize allocations
- Sparse formats (CSR for Connectome, Roaring bitmap for FCL) minimize memory usage

## Future Work

- Implement more neural models and plasticity rules
- Add WebGPU texture support for sensorimotor processing
- Improve zero-copy memory sharing between CPU and GPU
- Optimize for heterogeneous computing (using both CPU and GPU simultaneously)
