"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
Tests for WebGPU-specific backend functionality.

These tests verify that the WebGPU backend correctly implements
operations that might behave differently than on CPU or other backends.
"""

import os
import pytest
import numpy as np
from feagi.core.backend import get_backend, BackendType


@pytest.fixture
def wgpu_backend():
    """Get the WebGPU backend, skip if not available."""
    # Force WebGPU backend
    os.environ["FEAGI_BACKEND"] = "wgpu"
    
    # Initialize and get backend
    try:
        backend = get_backend()
        if backend.get_type() != BackendType.WGPU:
            pytest.skip("WebGPU backend not available")
    except (ImportError, AttributeError):
        pytest.skip("WebGPU backend not installed")
    
    return backend


@pytest.mark.wgpu
def test_tensor_creation(wgpu_backend):
    """Test creating tensors on the WebGPU backend."""
    # Create a tensor
    shape = (100, 100)
    tensor = wgpu_backend.create_tensor(shape, dtype=np.float32)
    
    # Verify properties
    assert tensor.shape == shape
    assert hasattr(tensor, "device")  # Should be on WebGPU device


@pytest.mark.wgpu
def test_tensor_operations(wgpu_backend):
    """Test basic tensor operations on WebGPU."""
    # Create tensors
    a = wgpu_backend.create_tensor((10, 10), dtype=np.float32)
    wgpu_backend.fill_tensor(a, 2.0)
    
    b = wgpu_backend.create_tensor((10, 10), dtype=np.float32)
    wgpu_backend.fill_tensor(b, 3.0)
    
    # Test operations
    c = wgpu_backend.add(a, b)
    
    # Convert to CPU for assertion
    c_cpu = wgpu_backend.to_cpu(c)
    assert np.allclose(c_cpu, 5.0)


@pytest.mark.wgpu
def test_compute_shader(wgpu_backend):
    """Test running a compute shader on WebGPU."""
    # Skip if compute shader functionality not available
    if not hasattr(wgpu_backend, "run_compute_shader"):
        pytest.skip("Compute shader functionality not available")
    
    # Create input tensor
    input_tensor = wgpu_backend.create_tensor((256,), dtype=np.float32)
    wgpu_backend.fill_tensor(input_tensor, 1.0)
    
    # Create output tensor
    output_tensor = wgpu_backend.create_tensor((256,), dtype=np.float32)
    
    # Define a simple compute shader that multiplies values by 2
    shader_code = """
    @group(0) @binding(0) var<storage, read> input: array<f32>;
    @group(0) @binding(1) var<storage, read_write> output: array<f32>;
    
    @compute @workgroup_size(64)
    fn main(@builtin(global_invocation_id) global_id: vec3<u32>) {
        let idx = global_id.x;
        if (idx >= arrayLength(&output)) {
            return;
        }
        output[idx] = input[idx] * 2.0;
    }
    """
    
    # Run the compute shader
    wgpu_backend.run_compute_shader(
        shader_code=shader_code,
        bindings=[input_tensor, output_tensor],
        dispatch_size=(4, 1, 1)  # 4 workgroups of 64 threads = 256 total
    )
    
    # Verify results
    result = wgpu_backend.to_cpu(output_tensor)
    assert np.allclose(result, 2.0)


@pytest.mark.wgpu
def test_browser_compatibility(wgpu_backend):
    """Test browser compatibility features of WebGPU backend."""
    # Skip if browser compatibility check not available
    if not hasattr(wgpu_backend, "is_browser_compatible"):
        pytest.skip("Browser compatibility check not available")
    
    # Check if the backend reports browser compatibility
    is_compatible = wgpu_backend.is_browser_compatible()
    
    # This is just a check that the function exists and returns something
    assert isinstance(is_compatible, bool)


@pytest.mark.wgpu
def test_cross_backend_data_transfer(wgpu_backend):
    """Test transferring data between WebGPU and other backends."""
    # Create a WebGPU tensor
    wgpu_tensor = wgpu_backend.create_tensor((10, 10), dtype=np.float32)
    wgpu_backend.fill_tensor(wgpu_tensor, 3.14)
    
    # Convert to CPU
    cpu_data = wgpu_backend.to_cpu(wgpu_tensor)
    assert isinstance(cpu_data, np.ndarray)
    assert np.allclose(cpu_data, 3.14)
    
    # If CUDA is available, test transfer to CUDA
    try:
        os.environ["FEAGI_BACKEND"] = "cuda"
        cuda_backend = get_backend()
        if cuda_backend.get_type() == BackendType.CUDA:
            # Transfer data to CUDA
            cuda_tensor = cuda_backend.from_cpu(cpu_data)
            assert cuda_tensor.shape == (10, 10)
            
            # Verify data is preserved
            cuda_cpu = cuda_backend.to_cpu(cuda_tensor)
            assert np.allclose(cuda_cpu, 3.14)
    except (ImportError, AttributeError):
        # CUDA not available, skip this part
        pass
    
    # Restore WebGPU backend for cleanup
    os.environ["FEAGI_BACKEND"] = "wgpu" 