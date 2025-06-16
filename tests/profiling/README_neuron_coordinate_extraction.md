# Neuron Coordinate Extraction Performance Profiling

This directory contains comprehensive performance profiling tests for FEAGI's neuron coordinate extraction functionality, focusing on the SIMD-optimized methods implemented in `CoreAPIService`.

## Overview

The neuron coordinate extraction profiling suite tests the high-performance coordinate retrieval methods that are critical for:
- Real-time brain visualization
- Spatial analysis of neural activity
- Performance optimization of FEAGI's core operations
- SIMD and GPU acceleration validation

## Performance Results Summary

### Latest Benchmark Results
- **Setup Performance**: ~460,000 neurons/sec connectome creation
- **Dictionary API**: ~22 million neurons/sec coordinate extraction
- **NumPy API**: ~44 million neurons/sec coordinate extraction (2x faster)
- **Optimal Batch Size**: 10,000-25,000 neurons for peak performance
- **Memory Efficiency**: <100 bytes per neuron

### SIMD Optimization
- Tests SIMD-optimized vectorized operations
- Validates backend selection (PyTorch, NumPy, CUDA)
- Measures SIMD utilization efficiency
- Provides scaling analysis across batch sizes

## Files

### Core Test File
- **`test_neuron_coordinate_extraction.py`** - Main pytest suite with comprehensive profiling tests

### Supporting Files
- **`run_neuron_profiling.py`** - Convenient script to run all tests and display summary
- **`logs/`** - Directory containing detailed profiling results in JSON format

## Test Categories

### 1. Setup and Validation Tests
- `test_coordinate_extraction_setup` - Validates test connectome creation
- `test_coordinate_extraction_accuracy` - Verifies coordinate consistency between methods

### 2. Performance Tests by Batch Size
- `test_small_batch_coordinate_extraction` - Tests 100-1,000 neuron batches
- `test_large_batch_coordinate_extraction` - Tests 5,000-50,000 neuron batches

### 3. SIMD and Optimization Tests
- `test_simd_optimization_effectiveness` - Validates SIMD configuration and utilization
- `test_performance_scaling_analysis` - Analyzes scaling characteristics across batch sizes

### 4. Comprehensive Benchmark
- `test_coordinate_extraction_benchmark_suite` - Full performance analysis with detailed reporting

## Running the Tests

### Quick Start
```bash
# Run the convenient profiling script
python tests/profiling/run_neuron_profiling.py
```

### Individual Test Commands
```bash
# Run all profiling tests
python -m pytest tests/profiling/test_neuron_coordinate_extraction.py -v

# Run comprehensive benchmark (saves detailed results)
python -m pytest tests/profiling/test_neuron_coordinate_extraction.py::test_coordinate_extraction_benchmark_suite -v

# Run small batch performance test
python -m pytest tests/profiling/test_neuron_coordinate_extraction.py::test_small_batch_coordinate_extraction -v

# Run with verbose output and timing
python -m pytest tests/profiling/test_neuron_coordinate_extraction.py -v -s
```

### Benchmark-Specific Tests
```bash
# Run only benchmark-tagged tests
python -m pytest tests/profiling/test_neuron_coordinate_extraction.py -m benchmark -v
```

## Understanding the Results

### Performance Metrics
- **neurons_per_second**: Primary performance metric for coordinate extraction throughput
- **scaling_efficiency**: How well performance scales with batch size (0.0-1.0)
- **simd_utilization_percent**: SIMD hardware utilization efficiency
- **memory_mb**: Memory usage for coordinate storage

### Scaling Trends
- **strong_positive_scaling**: Performance improves significantly with larger batches
- **moderate_positive_scaling**: Performance improves moderately with batch size
- **flat_scaling**: Performance relatively constant across batch sizes
- **negative_scaling**: Performance degrades with larger batches (indicates bottlenecks)

### Method Comparison
- **Dictionary API**: `get_neuron_coordinates()` - Returns structured dictionary with metadata
- **NumPy API**: `get_neuron_coordinates_numpy()` - Returns raw NumPy array for maximum performance
- **Benchmark API**: `benchmark_neuron_coordinate_extraction()` - Built-in performance analysis

## Optimization Recommendations

### Based on Profiling Results:
1. **Use NumPy API** for maximum performance (2x faster than Dictionary API)
2. **Optimal batch size**: 10,000-25,000 neurons for best throughput
3. **SIMD benefits**: Significant speedup on supported hardware (2-4x)
4. **Memory efficiency**: NumPy API uses ~50% less memory overhead

### Performance Thresholds:
- **Minimum**: 10,000 neurons/sec (basic functionality)
- **Target**: 50,000 neurons/sec (good performance)
- **Optimal**: 100,000+ neurons/sec (excellent performance)

## Detailed Output Files

### JSON Profiling Reports
The comprehensive benchmark saves detailed results to `logs/neuron_coordinate_extraction_profile_YYYYMMDD_HHMMSS.json`:

```json
{
  "setup_results": {
    "neuron_count": 64000,
    "setup_time_ms": 139.2,
    "neurons_per_second_setup": 459830
  },
  "scaling_analysis": {
    "numpy_api": {
      "optimal_batch_size": 10000,
      "optimal_performance": 44559901,
      "scaling_trend": "moderate_positive_scaling",
      "scaling_efficiency": 0.55
    }
  },
  "results_by_batch": {
    "100": { "methods": {...} },
    "500": { "methods": {...} },
    "1000": { "methods": {...} },
    ...
  }
}
```

## Integration with FEAGI

### Architecture Compliance
- **No fallbacks**: Tests validate that coordinate extraction never generates fake data
- **Rust compatibility**: SIMD operations designed for future Rust migration
- **SIMD optimization**: Leverages vectorized operations for maximum performance
- **Platform agnostic**: Tests work across different hardware configurations

### CoreAPIService Integration
The profiling tests validate the coordinate extraction methods added to `CoreAPIService`:
- `get_neuron_coordinates()` - Dictionary-based API with validation
- `get_neuron_coordinates_numpy()` - High-performance NumPy array API
- `benchmark_neuron_coordinate_extraction()` - Built-in benchmarking

### Performance Impact
These coordinate extraction methods are critical for:
- **Brain visualization**: Real-time display of neuron positions
- **Spatial analysis**: Computing distances and spatial relationships
- **Activity tracking**: Mapping neural activity to 3D coordinates
- **Memory optimization**: Efficient coordinate storage and retrieval

## Development Notes

### Test Architecture
- Uses `NeuronCoordinateExtractionProfiler` class for modular testing
- Creates realistic test connectomes with structured neuron placement
- Validates coordinate consistency between different extraction methods
- Measures both performance and accuracy

### SIMD Configuration
- Tests automatically detect available SIMD backends
- Validates SIMD configuration safety (prevents zero-width vectors)
- Measures actual vs theoretical SIMD speedup
- Provides optimization recommendations based on hardware

### Memory Profiling
- Tracks memory usage patterns for different batch sizes
- Validates memory efficiency improvements
- Compares memory overhead between API methods
- Ensures optimal memory layout for SIMD operations

## Future Enhancements

### Planned Improvements
1. **GPU acceleration testing** - CUDA/Metal coordinate extraction
2. **Distributed processing** - Multi-core coordinate extraction
3. **Cache optimization** - Memory access pattern analysis
4. **Real-time profiling** - Integration with live FEAGI instances

### Hardware-Specific Testing
- AVX/AVX2 instruction set validation
- ARM NEON SIMD testing
- Apple Silicon M-series optimization
- CUDA GPU acceleration benchmarks

---

For questions or issues with the profiling tests, see the main FEAGI documentation or create an issue in the FEAGI repository.
