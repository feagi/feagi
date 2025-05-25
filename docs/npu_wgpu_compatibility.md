# NPU WGPU Compatibility Monitoring

## Overview

The NPU WGPU compatibility monitoring system (`test_wgpu_compatibility.py`) is a comprehensive static analysis tool that evaluates all NPU Python modules for compatibility with **WGPU** - the native cross-platform GPU library supporting **Metal on macOS, DirectX 12 on Windows, and Vulkan on Linux**.

## Purpose

WGPU enables FEAGI to achieve high-performance GPU acceleration across all major platforms with a single codebase. This test ensures the NPU remains WGPU-compatible by:

- **Detecting WGPU-ready patterns**: Structure of Arrays (SoA), contiguous memory, GPU-friendly dtypes
- **Identifying incompatible patterns**: Environment dependencies, file I/O, non-contiguous memory
- **Preventing regressions**: Failing CI builds when WGPU compatibility is violated
- **Guiding implementation**: Providing specific recommendations for WGPU readiness

## Usage

### Running the Test

```bash
# Run the full WGPU compatibility analysis
cd feagi_core
python -m pytest tests/npu/test_wgpu_compatibility.py::test_npu_wgpu_compatibility -v -s

# The test will generate a detailed report at:
# tmp/npu_wgpu_compatibility_report.txt
```

### Test Results

The test analyzes each NPU module and assigns a **compatibility score (0-100)** and **compatibility level**:

- **🟢 EXCELLENT (90-100)**: Perfectly WGPU-ready, can use GPU acceleration immediately
- **🟢 GOOD (75-89)**: WGPU-compatible with minor considerations  
- **🟡 MEDIUM (50-74)**: Requires adaptation for optimal WGPU performance
- **🟠 POOR (25-49)**: Significant compatibility issues need addressing
- **🔴 CRITICAL (0-24)**: Major blockers prevent WGPU usage

### Current NPU Status

Based on the latest analysis:

#### ✅ **WGPU-Ready Modules (100% Compatible)**
- **`gpu_fcl_adapter.py`**: Already designed for GPU backends with backend abstraction
- **`optimized_structures.py`**: Perfect SoA memory layout and GPU-friendly dtypes

#### 🟡 **Requires Minor Adaptation**
- **`optimized_integration.py`**: Good patterns, needs environment variable cleanup
- **`special_area_handler.py`**: Minor dynamic allocation issues

#### 🔴 **Critical Issues to Fix**
- **`burst_engine.py`**: Heavy `os.environ` usage and print statements
- **`fcl_manager.py`**: Dynamic allocation patterns and stdout dependencies

## WGPU Compatibility Patterns

### ✅ **EXCELLENT - WGPU-Ready Patterns**

#### Structure of Arrays (SoA) Memory Layout
```python
# PERFECT for WGPU compute shaders:
self.membrane_potentials = np.zeros(capacity, dtype=np.float32, order='C')
self.coordinates_x = np.zeros(capacity, dtype=np.uint32, order='C') 
self.coordinates_y = np.zeros(capacity, dtype=np.uint32, order='C')
self.coordinates_z = np.zeros(capacity, dtype=np.uint32, order='C')
```
**Why good**: WGPU compute shaders expect separate arrays for each data component.

#### Backend Abstraction
```python
# EXCELLENT - Ready for WGPU backend:
from feagi.core.backend import get_backend, BackendType
backend = get_backend()
if backend.backend_type == BackendType.WGPU:
    # Use WGPU acceleration
```
**Why good**: Allows seamless integration of WGPU backend without code changes.

#### GPU-Friendly Data Types
```python
# PERFECT alignment with WGPU shaders:
dtype=np.float32  # Maps to WGSL f32
dtype=np.uint32   # Maps to WGSL u32  
dtype=np.int32    # Maps to WGSL i32
order='C'         # Contiguous memory for buffer uploads
```
**Why good**: Direct correspondence with WGPU shader data types.

#### Pre-allocated Buffers
```python
# WGPU-compatible fixed allocation:
self.fcl_history = [BitMap() for _ in range(window_size)]
membrane_potentials = np.zeros(capacity, dtype=np.float32)
```
**Why good**: WGPU works best with pre-allocated, fixed-size buffers.

### 🔴 **CRITICAL - WGPU Incompatible Patterns**

#### Environment Variable Dependencies
```python
# INCOMPATIBLE with WGPU:
if os.environ.get('FEAGI_DEBUG_NPU') == '1':
    print("Debug info...")

# WGPU-COMPATIBLE alternative:
if self.config.get('debug_npu', False):
    logger.debug("Debug info...")
```
**Why bad**: WGPU environments don't have access to `os.environ`.

#### Print Statements (stdout/stderr)
```python
# INCOMPATIBLE with WGPU:
print(f"🔥 BURST ENGINE: Status update")

# WGPU-COMPATIBLE alternative:
logger.info(f"🔥 BURST ENGINE: Status update")
```
**Why bad**: WGPU/GPU contexts don't have traditional stdout/stderr.

#### File I/O Operations
```python
# INCOMPATIBLE with WGPU:
with open('data.txt', 'r') as f:
    data = f.read()

# WGPU-COMPATIBLE alternative:
# Pass data through configuration or memory buffers
```
**Why bad**: WGPU contexts have no file system access.

#### Dynamic Memory Allocation in Hot Paths
```python
# PROBLEMATIC for WGPU:
def process_burst(self):
    temp_list = []  # Dynamic allocation
    for neuron in firing_neurons:
        temp_list.append(process(neuron))

# WGPU-FRIENDLY alternative:
def process_burst(self):
    # Use pre-allocated buffers
    self.temp_buffer[:len(firing_neurons)] = process_neurons(firing_neurons)
```
**Why bad**: WGPU performance degrades with frequent memory allocation.

## Integration with CI/CD

### Failure Conditions

The test **FAILS** CI builds when:

1. **File I/O operations found** (absolute WGPU blocker)
2. **Excessive environment dependencies** (>5 per module)
3. **Average compatibility score < 70/100**
4. **More than 1 module with critical issues**

### Warning Conditions

The test **warns** (but doesn't fail) when:
- Average compatibility score < 80/100
- Individual modules have poor scores

### Continuous Monitoring

```bash
# Add to your CI pipeline:
- name: Check NPU WGPU Compatibility
  run: |
    cd feagi_core
    python -m pytest tests/npu/test_wgpu_compatibility.py::test_npu_wgpu_compatibility -v
```

## WGPU Implementation Roadmap

### Phase 1: Fix Critical Issues (Priority: HIGH)
1. **Replace `os.environ` with config parameters** in `burst_engine.py`
2. **Replace print statements with logger calls** across all modules
3. **Eliminate dynamic allocation in hot paths** in `fcl_manager.py`

### Phase 2: Implement WGPU Backend (Priority: MEDIUM)
1. **Create `WGPUBackend` class** implementing `BackendInterface`
2. **Add WGPU buffer management** for NumPy array transfers
3. **Implement basic compute operations** (bitmap operations, neuron updates)

### Phase 3: Optimize Performance (Priority: LOW)
1. **Add WGPU compute shaders** for burst processing
2. **Optimize memory transfers** between CPU and GPU
3. **Implement advanced GPU algorithms** for large-scale simulations

## Benefits of WGPU Compatibility

### Cross-Platform GPU Acceleration
- **macOS**: Native Metal backend for Apple Silicon and Intel Macs
- **Windows**: DirectX 12 backend for modern Windows systems
- **Linux**: Vulkan backend for high-performance Linux systems

### Performance Improvements
- **Parallel Processing**: Thousands of neurons processed simultaneously
- **Memory Bandwidth**: Optimal memory usage patterns for GPU architectures
- **Compute Efficiency**: GPU-optimized algorithms for neural simulation

### Future-Proofing
- **WebGPU Compatibility**: Can easily support browser environments
- **Unified Codebase**: Single implementation across all platforms
- **Modern GPU Features**: Access to latest GPU compute capabilities

## Troubleshooting

### Common Issues

**"Heavy environment dependencies" error**:
- Replace `os.environ.get()` calls with config parameters
- Pass configuration through constructor arguments
- Use dependency injection for runtime configuration

**"Print statement" warnings**:
- Replace `print()` with `logger.debug()`, `logger.info()`, etc.
- Ensure logging is configured properly for the target environment

**"Dynamic allocation in hot path" warnings**:
- Pre-allocate buffers during initialization
- Reuse existing data structures instead of creating new ones
- Use fixed-size arrays instead of dynamic lists/sets

### Detailed Analysis

For comprehensive debugging:
```bash
# Run with verbose output
python -m pytest tests/npu/test_wgpu_compatibility.py::test_npu_wgpu_compatibility -v -s

# Check the detailed report
cat tmp/npu_wgpu_compatibility_report.txt
```

The report shows:
- Exact line numbers with compatibility issues
- Specific recommendations for each module
- WGPU-ready patterns already implemented
- Priority order for fixing issues

## Maintaining WGPU Compatibility

### Development Guidelines

1. **Always use backend abstraction** instead of direct GPU library calls
2. **Prefer SoA memory layouts** over Array of Structures (AoS)
3. **Use contiguous NumPy arrays** with GPU-friendly dtypes
4. **Avoid environment variables** in favor of explicit configuration
5. **Use logging instead of print statements** for all output
6. **Pre-allocate buffers** rather than dynamic memory allocation

### Code Review Checklist

- [ ] No `os.environ` usage
- [ ] No `print()` statements in production code
- [ ] No file I/O in hot paths
- [ ] NumPy arrays use `order='C'` and GPU-friendly dtypes
- [ ] SoA patterns used for multi-component data
- [ ] Backend abstraction used for GPU operations
- [ ] Fixed-size buffers used instead of dynamic allocation

This monitoring system ensures that FEAGI's NPU remains ready for high-performance, cross-platform GPU acceleration through WGPU. 