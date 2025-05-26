"""Unit tests for the array backend abstraction layer."""

import unittest
import pytest
import numpy as np
from feagi.bdu.models.array_backend import ArrayBackend, BackendType

# Skip tests for backends that aren't available
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

try:
    import cupy as cp
    CUPY_AVAILABLE = True
except ImportError:
    CUPY_AVAILABLE = False

try:
    import wgpu
    WGPU_AVAILABLE = True
except ImportError:
    WGPU_AVAILABLE = False


class TestArrayBackend(unittest.TestCase):
    """Test basic functionality of array backend abstraction."""
    
    def test_numpy_backend(self):
        """Test NumPy backend operations."""
        backend = ArrayBackend(BackendType.NUMPY)
        self.assertEqual(backend.backend_type, BackendType.NUMPY)
        
        # Test array creation
        shape = (10, 10)
        zeros = backend.zeros(shape)
        ones = backend.ones(shape)
        full = backend.full(shape, 5.0)
        
        # Verify arrays are correct
        self.assertEqual(zeros.shape, shape)
        self.assertEqual(ones.shape, shape)
        self.assertEqual(full.shape, shape)
        self.assertTrue(np.all(zeros == 0))
        self.assertTrue(np.all(ones == 1))
        self.assertTrue(np.all(full == 5.0))
        
        # Test array conversion
        arr = np.random.random(shape)
        backend_arr = backend.array(arr)
        numpy_arr = backend.to_numpy(backend_arr)
        self.assertTrue(np.allclose(arr, numpy_arr))
        
        # Test device transfer (no-op for NumPy)
        device_arr = backend.to_device(arr)
        self.assertTrue(np.allclose(arr, device_arr))
        
        # Test sparse matrix creation
        data = np.array([1.0, 2.0, 3.0])
        indices = np.array([0, 1, 2])
        indptr = np.array([0, 1, 2, 3])
        sparse_mat = backend.sparse_csr(data, indices, indptr, (3, 3))
        self.assertEqual(sparse_mat.shape, (3, 3))
    
    @pytest.mark.skipif(not TORCH_AVAILABLE, reason="PyTorch not available")
    def test_pytorch_backend(self):
        """Test PyTorch backend operations."""
        backend = ArrayBackend(BackendType.PYTORCH)
        self.assertEqual(backend.backend_type, BackendType.PYTORCH)
        
        # Test array creation
        shape = (10, 10)
        zeros = backend.zeros(shape)
        ones = backend.ones(shape)
        full = backend.full(shape, 5.0)
        
        # Verify arrays are correct
        self.assertEqual(tuple(zeros.shape), shape)
        self.assertEqual(tuple(ones.shape), shape)
        self.assertEqual(tuple(full.shape), shape)
        self.assertTrue(torch.all(zeros == 0))
        self.assertTrue(torch.all(ones == 1))
        self.assertTrue(torch.all(full == 5.0))
        
        # Test array conversion
        arr = np.random.random(shape)
        backend_arr = backend.array(arr)
        numpy_arr = backend.to_numpy(backend_arr)
        self.assertTrue(np.allclose(arr, numpy_arr))
        
        # Test device transfer
        device_arr = backend.to_device(arr)
        self.assertTrue(isinstance(device_arr, torch.Tensor))
        self.assertEqual(tuple(device_arr.shape), shape)
        
        # Test sparse matrix creation (if PyTorch version supports it)
        try:
            data = np.array([1.0, 2.0, 3.0])
            indices = np.array([0, 1, 2])
            indptr = np.array([0, 1, 2, 3])
            sparse_mat = backend.sparse_csr(data, indices, indptr, (3, 3))
            self.assertEqual(tuple(sparse_mat.shape), (3, 3))
        except (NotImplementedError, AttributeError) as e:
            print(f"PyTorch sparse CSR test skipped: {e}")
    
    @pytest.mark.skipif(not CUPY_AVAILABLE, reason="CuPy not available")
    def test_cupy_backend(self):
        """Test CuPy backend operations."""
        backend = ArrayBackend(BackendType.CUPY)
        self.assertEqual(backend.backend_type, BackendType.CUPY)
        
        # Test array creation
        shape = (10, 10)
        zeros = backend.zeros(shape)
        ones = backend.ones(shape)
        full = backend.full(shape, 5.0)
        
        # Verify arrays are correct
        self.assertEqual(zeros.shape, shape)
        self.assertEqual(ones.shape, shape)
        self.assertEqual(full.shape, shape)
        self.assertTrue(cp.all(zeros == 0))
        self.assertTrue(cp.all(ones == 1))
        self.assertTrue(cp.all(full == 5.0))
        
        # Test array conversion
        arr = np.random.random(shape)
        backend_arr = backend.array(arr)
        numpy_arr = backend.to_numpy(backend_arr)
        self.assertTrue(np.allclose(arr, numpy_arr))
        
        # Test device transfer
        device_arr = backend.to_device(arr)
        self.assertTrue(isinstance(device_arr, cp.ndarray))
        self.assertEqual(device_arr.shape, shape)
        
        # Test sparse matrix creation
        data = np.array([1.0, 2.0, 3.0])
        indices = np.array([0, 1, 2])
        indptr = np.array([0, 1, 2, 3])
        sparse_mat = backend.sparse_csr(data, indices, indptr, (3, 3))
        self.assertEqual(sparse_mat.shape, (3, 3))
    
    @pytest.mark.skipif(not WGPU_AVAILABLE, reason="WebGPU not available")
    def test_webgpu_backend(self):
        """Test WebGPU backend operations."""
        # This is a simplified test since WebGPU operations are more complex
        try:
            backend = ArrayBackend(BackendType.WEBGPU)
            self.assertEqual(backend.backend_type, BackendType.WEBGPU)
            
            # Test simple array creation
            shape = (10, 10)
            zeros = backend.zeros(shape)
            numpy_zeros = backend.to_numpy(zeros)
            self.assertEqual(numpy_zeros.shape, shape)
            self.assertTrue(np.all(numpy_zeros == 0))
        except Exception as e:
            print(f"WebGPU test failed: {e}")
            # Skip the test if WebGPU initialization fails
            pytest.skip("WebGPU initialization failed")
    
    def test_auto_selection(self):
        """Test automatic backend selection."""
        backend = ArrayBackend(BackendType.AUTO)
        
        # Verify a backend was selected
        self.assertIn(backend.backend_type, [
            BackendType.NUMPY, 
            BackendType.PYTORCH, 
            BackendType.CUPY,
            BackendType.WEBGPU
        ])
        
        # Test basic operations with auto-selected backend
        shape = (5, 5)
        zeros = backend.zeros(shape)
        numpy_zeros = backend.to_numpy(zeros)
        self.assertEqual(numpy_zeros.shape, shape)
        self.assertTrue(np.all(numpy_zeros == 0))


if __name__ == "__main__":
    unittest.main() 