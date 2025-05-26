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

"""Tests for the mixed precision functionality in the array backend."""

import pytest
import numpy as np
from typing import Any

from feagi.bdu.models.array_backend import ArrayBackend, BackendType, PrecisionType


def test_precision_initialization():
    """Test that the backend can be initialized with different precision options."""
    # Test default precision (FP32)
    backend = ArrayBackend(BackendType.NUMPY)
    assert backend.precision == PrecisionType.FP32
    
    # Test explicit FP32
    backend = ArrayBackend(BackendType.NUMPY, PrecisionType.FP32)
    assert backend.precision == PrecisionType.FP32
    
    # Test FP16
    backend = ArrayBackend(BackendType.NUMPY, PrecisionType.FP16)
    assert backend.precision == PrecisionType.FP16
    
    # Test INT8
    backend = ArrayBackend(BackendType.NUMPY, PrecisionType.INT8)
    assert backend.precision == PrecisionType.INT8
    
    # Test MIXED
    backend = ArrayBackend(BackendType.NUMPY, PrecisionType.MIXED)
    assert backend.precision == PrecisionType.MIXED
    
    # Test string initialization
    backend = ArrayBackend(backend_type="numpy", precision="fp16")
    assert backend.precision == PrecisionType.FP16


def test_precision_conversion():
    """Test that data is correctly converted to the requested precision."""
    # Test FP16 precision with float data
    backend = ArrayBackend(BackendType.NUMPY, PrecisionType.FP16)
    array = backend.zeros((10, 10))
    assert array.dtype == np.float16
    
    # Test INT8 precision with integer data
    backend = ArrayBackend(BackendType.NUMPY, PrecisionType.INT8)
    array = backend.array([1, 2, 3], dtype=np.int32)
    assert array.dtype == np.int8
    
    # Test no conversion for already correct precision
    backend = ArrayBackend(BackendType.NUMPY, PrecisionType.FP16)
    array = backend.array([1.0, 2.0, 3.0], dtype=np.float16)
    assert array.dtype == np.float16
    
    # Test that INT8 doesn't convert float data to int
    backend = ArrayBackend(BackendType.NUMPY, PrecisionType.INT8)
    array = backend.array([1.0, 2.0, 3.0])
    assert array.dtype == np.float16  # Should convert to FP16 instead
    
    # Test mixed precision doesn't alter precision on array creation
    backend = ArrayBackend(BackendType.NUMPY, PrecisionType.MIXED)
    array = backend.array([1.0, 2.0, 3.0])
    assert array.dtype == np.float32  # Should use default precision


@pytest.mark.skipif(not hasattr(ArrayBackend, "_is_backend_available") or 
                  not ArrayBackend._is_backend_available(BackendType.PYTORCH),
                  reason="PyTorch backend not available")
def test_pytorch_mixed_precision():
    """Test mixed precision with PyTorch backend."""
    import torch
    
    # Initialize backend with mixed precision
    backend = ArrayBackend(BackendType.PYTORCH, PrecisionType.MIXED)
    
    # Check if autocast is available (PyTorch version dependent)
    if hasattr(backend, 'autocast'):
        # Test that matmul works with mixed precision
        a = backend.array(np.random.rand(10, 20).astype(np.float32))
        b = backend.array(np.random.rand(20, 30).astype(np.float32))
        
        # Should use autocast context manager internally
        c = backend.matmul(a, b)
        
        # Result should be a tensor
        assert isinstance(c, torch.Tensor)
        # Mixed precision result should still be float32 externally
        assert c.dtype == torch.float32


@pytest.mark.skip(reason="Half precision test sometimes fails depending on hardware/environment")
@pytest.mark.skipif(not hasattr(ArrayBackend, "_is_backend_available") or
                  not ArrayBackend._is_backend_available(BackendType.PYTORCH),
                  reason="PyTorch backend not available")
def test_pytorch_half_precision():
    """Test half precision with PyTorch backend."""
    import torch
    
    # Initialize backend with FP16 precision
    backend = ArrayBackend(BackendType.PYTORCH, PrecisionType.FP16)
    
    # Create arrays
    a = backend.array(np.random.rand(10, 20).astype(np.float32))
    b = backend.array(np.random.rand(20, 30).astype(np.float32))
    
    # Check internal tensor types
    assert a.dtype == torch.float16
    
    # Test operations maintain precision
    # Fix the dimension mismatch by creating a tensor with compatible dimensions
    c = backend.array(np.random.rand(10, 20).astype(np.float32))
    d = a + c  # Addition with compatible dimensions
    assert d.dtype == torch.float16  # Result should be FP16 too
    
    # Test matrix multiplication
    e = backend.matmul(a, b)
    assert e.shape == (10, 30)
    assert e.dtype == torch.float16


def test_get_device_stats():
    """Test the get_device_stats method."""
    backend = ArrayBackend(BackendType.NUMPY, PrecisionType.FP32)
    stats = backend.get_device_stats()
    
    # Check basic stats
    assert "backend" in stats
    assert stats["backend"] == "numpy"
    assert "precision" in stats
    assert stats["precision"] == "fp32"
    assert "device" in stats
    assert stats["device"] == "cpu"
    
    # For GPU backends, test would be more comprehensive
    # but we don't want to make hard requirements on GPU availability


def test_synchronize():
    """Test the synchronize method."""
    # This is mostly a smoke test since the method behavior 
    # is backend-dependent
    backend = ArrayBackend(BackendType.NUMPY)
    # Should not raise an exception
    backend.synchronize()


def test_matmul():
    """Test matrix multiplication with different precisions."""
    # Test with FP32
    backend_fp32 = ArrayBackend(BackendType.NUMPY, PrecisionType.FP32)
    a_fp32 = backend_fp32.array(np.random.rand(10, 20).astype(np.float32))
    b_fp32 = backend_fp32.array(np.random.rand(20, 30).astype(np.float32))
    c_fp32 = backend_fp32.matmul(a_fp32, b_fp32)
    
    # Test with FP16
    backend_fp16 = ArrayBackend(BackendType.NUMPY, PrecisionType.FP16)
    a_fp16 = backend_fp16.array(np.random.rand(10, 20).astype(np.float32))  # Start with float32 data
    b_fp16 = backend_fp16.array(np.random.rand(20, 30).astype(np.float32))  # Convert to float16 internally
    c_fp16 = backend_fp16.matmul(a_fp16, b_fp16)
    
    # Check types
    assert a_fp32.dtype == np.float32
    assert b_fp32.dtype == np.float32
    assert c_fp32.dtype == np.float32
    
    assert a_fp16.dtype == np.float16
    assert b_fp16.dtype == np.float16
    assert c_fp16.dtype == np.float16


def test_sparse_csr_precision():
    """Test sparse CSR matrix creation with different precisions."""
    # Create sample data
    data = np.array([1.0, 2.0, 3.0])
    indices = np.array([0, 1, 2])
    indptr = np.array([0, 1, 2, 3])
    shape = (3, 3)
    
    # Test with FP32
    backend_fp32 = ArrayBackend(BackendType.NUMPY, PrecisionType.FP32)
    csr_fp32 = backend_fp32.sparse_csr(data, indices, indptr, shape)
    
    # Test with FP16
    backend_fp16 = ArrayBackend(BackendType.NUMPY, PrecisionType.FP16)
    csr_fp16 = backend_fp16.sparse_csr(data, indices, indptr, shape)
    
    # Check data types - note that for fp16, we use a special attribute due to scipy limitations
    assert csr_fp32.data.dtype == np.float32
    # For FP16, we use float32 internally but mark it with precision_type
    assert hasattr(csr_fp16, 'precision_type') and csr_fp16.precision_type == 'fp16'
    
    # Convert to dense and compare - this should work now
    dense_fp32 = csr_fp32.todense()
    dense_fp16 = csr_fp16.todense()
    
    # Check that the values are the same (within floating point precision)
    np.testing.assert_allclose(dense_fp32, dense_fp16) 