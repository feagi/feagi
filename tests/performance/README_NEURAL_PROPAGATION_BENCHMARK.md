# FEAGI Neural Propagation Performance Benchmark

## Overview

The Neural Propagation Benchmark is a comprehensive performance testing suite that measures FEAGI's **realistic neural computation** capabilities. Unlike previous tests that only measured FCL (Fire Candidate List) data structure operations, this benchmark tests **actual synaptic propagation** through interconnected cortical areas.

## 🧠 Test Architecture

### Neural Network Design
```
┌─────────┐    ┌─────────┐    ┌─────────┐
│ Area A  │───▶│ Area B  │───▶│ Area C  │
│ M×M×1   │    │ M×M×1   │    │ M×M×1   │
└─────────┘    └─────────┘    └─────────┘
     ▲                              │
     └──────────────────────────────┘
```

- **3 Cortical Areas**: A, B, C (each M×M×1 dimensions)
- **Circular Connectivity**: A→B→C→A using `block_to_block` morphology
- **Total Neurons**: 3 × M² neurons
- **Synaptic Connections**: ~10% connectivity between areas

### Test Flow
1. **Initial Stimulation**: Fire all M² neurons in Area A
2. **Propagation**: Neural activity flows A→B→C→A automatically
3. **Duration Control**: `consecutive_fire_count` determines how long neurons stay active
4. **Measurement**: Track performance across multiple burst cycles

## 📊 Test Parameters

### Primary Parameters
- **M (Dimension)**: Cortical area size (M×M neurons per area)
- **Consecutive Fire Count**: How many bursts each neuron can fire (controls test duration)
- **Simulation Timestep**: Burst frequency in milliseconds

### Parameter Ranges
```python
M values: [10, 25, 50, 100, 200, 500, 1000, 1500, 2000]
Consecutive fires: [1, 3, 5, 10, 20, 50, 100, 200]
Timesteps: [100.0, 50.0, 25.0, 10.0, 5.0, 2.0, 1.0] ms
```

## 🎯 Test Scenarios

### 1. Systematic Parameter Sweeps
**One parameter varies, others held constant**

#### M Dimension Sweep
- Tests neural network scaling from 300 to 12M neurons
- Baseline: consecutive_fires=10, timestep=50ms
- Scenarios: `M_sweep_10`, `M_sweep_25`, ..., `M_sweep_2000`

#### Consecutive Fires Sweep  
- Tests duration scaling from 1 to 200 bursts
- Baseline: M=50, timestep=50ms
- Scenarios: `fires_sweep_1`, `fires_sweep_3`, ..., `fires_sweep_200`

#### Timestep Sweep
- Tests frequency scaling from 10Hz to 1000Hz
- Baseline: M=50, consecutive_fires=10
- Scenarios: `timestep_sweep_100.0ms`, ..., `timestep_sweep_1.0ms`

### 2. Extreme Stress Scenarios
**Push FEAGI to its limits**

#### Ultra-High Neuron Count
```python
{"M": 2000, "consecutive_fires": 10, "timestep_ms": 50.0}  # 12M neurons
{"M": 2000, "consecutive_fires": 20, "timestep_ms": 5.0}   # 12M neurons, 200Hz
```

#### Ultra-High Frequency
```python
{"M": 100, "consecutive_fires": 10, "timestep_ms": 5.0}    # 200Hz
{"M": 500, "consecutive_fires": 5, "timestep_ms": 5.0}     # 200Hz, 750K neurons
```

#### Long Duration Tests
```python
{"M": 100, "consecutive_fires": 500, "timestep_ms": 10.0}  # 500 bursts
{"M": 200, "consecutive_fires": 1000, "timestep_ms": 20.0} # 1000 bursts
```

#### Ultimate Stress
```python
{"M": 2000, "consecutive_fires": 25, "timestep_ms": 10.0}  # 12M neurons, 100Hz, 25 bursts
```

### 3. Frequency Stability Tests
**Validate burst frequency robustness under load**

Tests frequency stability at 200Hz (5ms timestep) with increasing neuron counts:
- 50×50 (7.5K neurons)
- 100×100 (30K neurons)  
- 200×200 (120K neurons)
- 500×500 (750K neurons)
- 1000×1000 (3M neurons)

## 📈 Performance Metrics

### Execution Metrics
- **Average Burst Time**: Mean time per neural computation burst
- **Max/Min Burst Time**: Performance variability
- **Total Test Time**: End-to-end benchmark duration
- **Setup Time**: Network initialization overhead

### Throughput Metrics
- **Neurons Processed/Sec**: Neural computation throughput
- **Synapses Processed/Sec**: Synaptic operation throughput  
- **Bursts/Sec**: Simulation frequency achieved

### Resource Metrics
- **Peak Memory Usage**: Maximum RAM consumption
- **Average/Max CPU Usage**: Processor utilization
- **Memory Efficiency**: Memory per neuron

### Neural Activity Metrics
- **Total Bursts**: Number of simulation cycles
- **Total Neurons Fired**: Cumulative neural activity
- **Propagation Cycles**: A→B→C→A cycles completed
- **Activity Decay Rate**: How quickly activity diminishes

### Frequency Stability Metrics
- **Target vs Actual Frequency**: Frequency accuracy
- **Frequency Deviation %**: Performance under load
- **Frequency Stability Score**: Consistency measure

## 🚀 Usage

### Run Complete Benchmark Suite
```bash
python tests/performance/run_fcl_benchmarks.py --neural-propagation
```

### Run Specific Test Categories
```bash
# Quick validation
python tests/performance/npu/test_neural_propagation_benchmark.py

# With custom logging
PYTHONPATH=. python tests/performance/npu/test_neural_propagation_benchmark.py
```

### Integration with Main Benchmark Runner
```bash
# Include in full benchmark suite
python tests/performance/run_fcl_benchmarks.py --full-suite

# Compare with other benchmarks
python tests/performance/run_fcl_benchmarks.py --quick
python tests/performance/run_fcl_benchmarks.py --neural-propagation
```

## 📊 Expected Results

### Performance Baselines
Based on test design, expected performance characteristics:

#### Small Networks (M ≤ 100)
- **Burst Time**: < 10ms
- **Throughput**: > 100K neurons/sec
- **Frequency Deviation**: < 5%
- **Memory**: < 100MB

#### Medium Networks (100 < M ≤ 500)  
- **Burst Time**: 10-50ms
- **Throughput**: 50K-100K neurons/sec
- **Frequency Deviation**: 5-15%
- **Memory**: 100MB-1GB

#### Large Networks (M > 500)
- **Burst Time**: 50-200ms
- **Throughput**: 10K-50K neurons/sec  
- **Frequency Deviation**: 15-50%
- **Memory**: > 1GB

### Scaling Characteristics
- **Linear scaling** with neuron count (M²)
- **Frequency degradation** above 100Hz for large networks
- **Memory scaling** approximately 1MB per 10K neurons

## 🔍 What This Benchmark Reveals

### 1. Real Neural Computation Performance
Unlike FCL-only tests, this measures:
- Synaptic weight calculations
- Membrane potential updates
- Inter-area communication overhead
- Realistic neural network dynamics

### 2. Scaling Bottlenecks
Identifies performance limits:
- Maximum practical network size
- Frequency limits under load
- Memory constraints
- CPU utilization patterns

### 3. Architecture Validation
Validates FEAGI's design:
- BDU/NPU separation effectiveness
- Async FCL processing benefits
- Rust optimization potential
- Cross-platform performance

### 4. Production Readiness
Determines operational limits:
- Recommended configurations
- Safe operating parameters
- Resource requirements
- Performance predictability

## 🎯 Key Differences from Previous Tests

| Aspect | Previous FCL Tests | Neural Propagation Tests |
|--------|-------------------|-------------------------|
| **Scope** | Data structure operations | Full neural computation |
| **Connectivity** | No synapses | Real synaptic connections |
| **Propagation** | Manual neuron lists | Automatic neural flow |
| **Realism** | Artificial patterns | Biologically-inspired |
| **Performance** | I/O bound | Compute bound |
| **Bottlenecks** | Memory access | Neural computation |

## 📁 Output Files

### Results Directory
```
tests/performance/logs/
├── neural_propagation_benchmark_YYYYMMDD_HHMMSS.json
├── neural_propagation_summary_YYYYMMDD_HHMMSS.txt
└── neural_propagation_analysis_YYYYMMDD_HHMMSS.csv
```

### JSON Results Format
```json
{
  "benchmark_type": "neural_propagation",
  "timestamp": 1642678800.0,
  "total_scenarios": 45,
  "successful_scenarios": 42,
  "failed_scenarios": 3,
  "results": [
    {
      "scenario_name": "M_sweep_100",
      "M": 100,
      "total_neurons": 30000,
      "avg_burst_time_ms": 12.5,
      "neurons_processed_per_sec": 85000,
      "frequency_deviation_percent": 3.2,
      "peak_memory_mb": 245.8,
      ...
    }
  ]
}
```

## 🔧 Configuration

### Environment Variables
```bash
export FEAGI_LOG_LEVEL=INFO          # Control logging verbosity
export FEAGI_DEBUG_NPU=1             # Enable NPU debugging
export FEAGI_DEBUG_BDU=1             # Enable BDU debugging
```

### Test Customization
Modify test scenarios in `create_test_scenarios()`:
```python
# Add custom scenario
scenarios.append({
    "M": 300,
    "consecutive_fires": 15,
    "timestep_ms": 8.0,
    "name": "custom_scenario"
})
```

## 🎉 Benefits

### For Developers
- **Realistic Performance Data**: Actual neural computation metrics
- **Bottleneck Identification**: Clear performance limits
- **Optimization Targets**: Specific areas for improvement
- **Regression Detection**: Performance change tracking

### For Users
- **Configuration Guidance**: Optimal parameter selection
- **Resource Planning**: Memory and CPU requirements
- **Performance Expectations**: Realistic throughput estimates
- **Scaling Predictions**: Growth planning support

### For Research
- **Benchmark Comparisons**: Standard performance metrics
- **Algorithm Validation**: Neural computation accuracy
- **Architecture Analysis**: Design effectiveness
- **Future Development**: Optimization roadmap

---

**This benchmark represents a major advancement in FEAGI performance testing, moving from artificial data structure tests to realistic neural computation validation.**
