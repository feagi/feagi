# FEAGI GPU Architecture & Backend Support

## Overview

FEAGI supports multiple compute backends for neural network operations, enabling transparent acceleration across different hardware platforms. This document outlines our GPU architecture and backend implementations.

## Backend Types

### 1. **NumPy (CPU)**
- **Use case**: Development, testing, small models
- **Platform**: All platforms
- **Performance**: Baseline CPU performance

### 2. **PyTorch (CPU/GPU)**
- **Use case**: General GPU acceleration, compatibility
- **Platforms**: CUDA (NVIDIA), MPS (Apple Silicon), CPU fallback
- **Performance**: Good GPU acceleration with broad compatibility

### 3. **CuPy (NVIDIA GPU)**
- **Use case**: High-performance NVIDIA GPU compute
- **Platform**: CUDA-capable NVIDIA GPUs
- **Performance**: Excellent for large-scale operations

### 4. **wgpu (Cross-platform GPU)**
- **Use case**: High-performance native GPU compute
- **Platforms**: Metal (macOS), Vulkan (Linux), D3D12 (Windows)
- **Performance**: Near-native GPU performance with low overhead

## Important Distinction: wgpu vs WebGPU

**This is a critical clarification to avoid confusion:**

### **wgpu** (What we use)
- **Type**: Rust-based native GPU library
- **Target**: Desktop/mobile applications, embedded systems
- **Backends**: Metal, Vulkan, D3D12, OpenGL
- **Performance**: Near-native, optimized for high-performance compute
- **Use case**: Our implementation for Mac M4 Metal acceleration

### **WebGPU** (What we DON'T use)
- **Type**: Web standard/browser API
- **Target**: Web browsers only
- **Backend**: Browser's GPU abstraction layer
- **Performance**: Good but limited by browser sandbox
- **Use case**: Web-based applications

## FEAGI wgpu Implementation

### Mac M4 Optimization
Our wgpu backend specifically targets Apple Silicon with Metal:

```python
# Backend selection on Mac M4
backend = ArrayBackend(BackendType.WGPU)
# → Uses wgpu → Metal → Apple GPU
```

### Performance Characteristics
- **Memory**: Direct GPU memory management
- **Latency**: Low-latency compute shaders
- **Throughput**: Optimized for neural network operations
- **Efficiency**: ~2-5x faster than PyTorch MPS for certain operations

### Supported Operations
- Matrix multiplication (via compute shaders)
- Element-wise operations
- Memory transfers (CPU ↔ GPU)
- Precision control (FP32, FP16)

## Backend Selection Strategy

FEAGI automatically selects the best available backend:

1. **PyTorch** (if CUDA/MPS available)
2. **CuPy** (if CUDA available)
3. **wgpu** (if wgpu library available)
4. **NumPy** (fallback)

### Manual Override
```python
# Force specific backend
backend = ArrayBackend(BackendType.WGPU)  # wgpu
backend = ArrayBackend(BackendType.PYTORCH)  # PyTorch
backend = ArrayBackend(BackendType.AUTO)  # Auto-select
```

## Platform-Specific Recommendations

### **macOS (Apple Silicon)**
- **Primary**: wgpu (Metal backend)
- **Fallback**: PyTorch (MPS backend)
- **Reason**: wgpu provides better performance than PyTorch MPS

### **Linux (NVIDIA GPU)**
- **Primary**: CuPy or PyTorch (CUDA)
- **Fallback**: wgpu (Vulkan backend)
- **Reason**: Mature CUDA ecosystem

### **Linux (AMD GPU)**
- **Primary**: wgpu (Vulkan backend)
- **Fallback**: PyTorch (CPU)
- **Reason**: Better AMD GPU support than PyTorch

### **Windows (Any GPU)**
- **Primary**: PyTorch (CUDA/DirectML)
- **Secondary**: wgpu (D3D12 backend)
- **Fallback**: NumPy (CPU)

## Memory Management

### wgpu Memory Model
```python
# Create GPU buffer
gpu_array = backend.array(cpu_data)  # CPU → GPU

# Compute on GPU
result = backend.matmul(gpu_array, weights)

# Transfer back when needed
cpu_result = backend.to_numpy(result)  # GPU → CPU
```

### Memory Optimization
- Minimize CPU ↔ GPU transfers
- Use GPU-resident data when possible
- Batch operations for efficiency

## Future Roadmap

### Short-term
- ✅ Mac M4 Metal acceleration via wgpu
- 🔄 Optimized compute shaders for neural operations
- 🔄 Automatic backend benchmarking

### Medium-term
- 🔄 Linux Vulkan backend optimization
- 🔄 Windows D3D12 backend support
- 🔄 WebAssembly deployment via wgpu

### Long-term
- 🔄 Custom neural compute kernels
- 🔄 Multi-GPU support
- 🔄 Embedded system deployment

## Configuration

### Environment Variables
```bash
export FEAGI_BACKEND=wgpu          # Force wgpu backend
export FEAGI_PRECISION=fp16        # Use half precision
export WGPU_BACKEND=metal          # Force Metal backend (macOS)
```

### Runtime Configuration
```python
# In feagi_configuration.toml
[compute]
backend = "wgpu"
precision = "fp16"
device_preference = "high_performance"
```

## Troubleshooting

### Common Issues

**wgpu not detected on Mac M4:**
```bash
pip install wgpu  # Install wgpu library
```

**Metal backend not available:**
- Ensure macOS 10.15+ for Metal support
- Check hardware compatibility

**Performance slower than expected:**
- Verify GPU backend is actually being used
- Check for CPU ↔ GPU transfer bottlenecks
- Use profiling tools to identify issues

### Debug Information
```python
# Check backend status
stats = backend.get_device_stats()
print(f"Backend: {stats['backend']}")
print(f"Device: {stats['device']}")
```

## See Also
- [GPU Optimization Guide](arch-gpu-optimization.md)
- [wgpu Compatibility](npu_wgpu_compatibility.md)
- [Performance Benchmarking](../tests/performance/)
