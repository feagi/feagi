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
Tests for CUDA-specific backend functionality.

These tests verify that the CUDA backend correctly implements
operations that might behave differently than on CPU.
"""

import os

import numpy as np
import pytest

from feagi.core.backend import BackendType, get_backend


@pytest.fixture
def cuda_backend(skip_if_no_gpu):
    """Get the CUDA backend, skip if not available."""
    # Force CUDA backend
    os.environ["FEAGI_BACKEND"] = "cuda"

    # Initialize and get backend
    backend = get_backend()
    if backend.get_type() != BackendType.CUDA:
        pytest.skip("CUDA backend not available")

    return backend


@pytest.mark.backend("cuda")
def test_tensor_creation(cuda_backend):
    """Test creating tensors on the CUDA backend."""
    # Create a tensor
    shape = (100, 100)
    tensor = cuda_backend.create_tensor(shape, dtype=np.float32)

    # Verify properties
    assert tensor.shape == shape
    assert tensor.device.type == "cuda"  # Should be on CUDA device


@pytest.mark.backend("cuda")
def test_tensor_operations(cuda_backend):
    """Test basic tensor operations on CUDA."""
    # Create tensors
    a = cuda_backend.create_tensor((10, 10), dtype=np.float32)
    a.fill_(2.0)

    b = cuda_backend.create_tensor((10, 10), dtype=np.float32)
    b.fill_(3.0)

    # Test operations
    c = cuda_backend.add(a, b)

    # Convert to CPU for assertion
    c_cpu = cuda_backend.to_cpu(c)
    assert np.allclose(c_cpu, 5.0)


@pytest.mark.backend("cuda")
def test_matrix_multiply(cuda_backend):
    """Test matrix multiplication on CUDA."""
    # Create tensors
    a = cuda_backend.create_tensor((10, 20), dtype=np.float32)
    a.fill_(1.0)

    b = cuda_backend.create_tensor((20, 30), dtype=np.float32)
    b.fill_(2.0)

    # Matrix multiply
    c = cuda_backend.matmul(a, b)

    # Verify shape and values (each element should be 1.0 * 2.0 * 20)
    assert c.shape == (10, 30)
    c_cpu = cuda_backend.to_cpu(c)
    assert np.allclose(c_cpu, 40.0)  # 20 * 2.0 = 40.0


@pytest.mark.backend("cuda")
def test_sparse_operations(cuda_backend):
    """Test sparse tensor operations on CUDA."""
    # Create sparse matrix indices and values
    indices = cuda_backend.create_tensor((2, 10), dtype=np.int64)
    # Set indices for a sparse 100x100 matrix with 10 non-zero elements
    for i in range(10):
        indices[0, i] = i  # Row indices
        indices[1, i] = i  # Col indices

    values = cuda_backend.create_tensor((10,), dtype=np.float32)
    values.fill_(1.0)

    # Create sparse tensor
    sparse_tensor = cuda_backend.create_sparse_tensor(indices, values, (100, 100))

    # Verify sparse tensor
    assert sparse_tensor.shape == (100, 100)
    assert cuda_backend.get_sparse_nnz(sparse_tensor) == 10


@pytest.mark.backend("cuda")
def test_cuda_memory_usage(cuda_backend):
    """Test querying CUDA memory usage."""
    # Get initial memory usage
    initial_used = cuda_backend.get_gpu_memory_used()

    # Allocate a large tensor
    large_tensor = cuda_backend.create_tensor((1000, 1000), dtype=np.float32)

    # Check memory usage increased
    new_used = cuda_backend.get_gpu_memory_used()
    assert new_used > initial_used

    # Free tensor and check memory usage decreases
    del large_tensor
    cuda_backend.cuda_synchronize()  # Ensure CUDA operations complete
    final_used = cuda_backend.get_gpu_memory_used()
    assert final_used < new_used
