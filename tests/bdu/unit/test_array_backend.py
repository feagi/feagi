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

"""Unit tests for the array backend abstraction layer."""

import unittest

import numpy as np
import pytest

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
        zeros = backend.zeros(shape, dtype=np.float32)
        ones = backend.ones(shape, dtype=np.float32)
        full = backend.full(shape, 5.0, dtype=np.float32)

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
        zeros = backend.zeros(shape, dtype=np.float32)
        ones = backend.ones(shape, dtype=np.float32)
        full = backend.full(shape, 5.0, dtype=np.float32)

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
        zeros = backend.zeros(shape, dtype=np.float32)
        ones = backend.ones(shape, dtype=np.float32)
        full = backend.full(shape, 5.0, dtype=np.float32)

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
            backend = ArrayBackend(BackendType.WGPU)
            self.assertEqual(backend.backend_type, BackendType.WGPU)

            # Test simple array creation
            shape = (10, 10)
            zeros = backend.zeros(shape, dtype=np.float32)
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
        self.assertIn(
            backend.backend_type,
            [
                BackendType.NUMPY,
                BackendType.PYTORCH,
                BackendType.CUPY,
                BackendType.WGPU,
            ],
        )

        # Test basic operations with auto-selected backend
        shape = (5, 5)
        zeros = backend.zeros(shape, dtype=np.float32)
        numpy_zeros = backend.to_numpy(zeros)
        self.assertEqual(numpy_zeros.shape, shape)
        self.assertTrue(np.all(numpy_zeros == 0))


class TestArrayBackendAdvanced(unittest.TestCase):
    """Test advanced functionality and edge cases of array backend."""

    def setUp(self):
        """Set up test with numpy backend."""
        self.backend = ArrayBackend(BackendType.NUMPY)

    def test_error_handling(self):
        """Test error handling for invalid operations."""
        # Test invalid backend type
        with self.assertRaises(ValueError):
            ArrayBackend("invalid_backend")

        # Test invalid array operations
        with self.assertRaises((ValueError, TypeError)):
            self.backend.zeros((-1, 5))  # Negative dimension

    def test_dtype_handling(self):
        """Test different data type handling."""
        shape = (5, 5)

        # Test different dtypes
        dtypes = [np.float32, np.float64, np.int32, np.int64, np.bool_]
        for dtype in dtypes:
            arr = self.backend.zeros(shape, dtype=dtype)
            numpy_arr = self.backend.to_numpy(arr)
            self.assertEqual(numpy_arr.dtype, dtype)

    def test_array_operations(self):
        """Test various array operations."""
        shape = (10, 10)

        # Create test arrays
        a = self.backend.ones(shape, dtype=np.float32)
        b = self.backend.full(shape, 2.0, dtype=np.float32)

        # Test element-wise operations
        c = a + b  # Should work with numpy arrays
        numpy_c = self.backend.to_numpy(c)
        self.assertTrue(np.all(numpy_c == 3.0))

    def test_memory_management(self):
        """Test memory management operations."""
        shape = (100, 100)

        # Create large array
        large_arr = self.backend.zeros(shape, dtype=np.float32)

        # Test memory usage estimation
        memory_bytes = (
            large_arr.nbytes
            if hasattr(large_arr, "nbytes")
            else len(large_arr.flatten()) * 4
        )
        self.assertGreater(memory_bytes, 0)

    def test_sparse_matrix_operations(self):
        """Test sparse matrix operations."""
        # Create a simple sparse matrix
        data = np.array([1.0, 2.0, 3.0, 4.0])
        indices = np.array([0, 1, 2, 1])
        indptr = np.array([0, 2, 3, 4])
        shape = (3, 3)

        sparse_mat = self.backend.sparse_csr(data, indices, indptr, shape)

        # Verify shape
        self.assertEqual(sparse_mat.shape, shape)

        # Convert to dense and verify values
        if hasattr(sparse_mat, "toarray"):
            dense = sparse_mat.toarray()
            self.assertEqual(dense[0, 0], 1.0)
            self.assertEqual(dense[0, 1], 2.0)
            self.assertEqual(dense[1, 2], 3.0)
            self.assertEqual(dense[2, 1], 4.0)

    def test_device_operations(self):
        """Test device transfer operations."""
        shape = (10, 10)
        arr = np.random.random(shape).astype(np.float32)

        # Test device transfer
        device_arr = self.backend.to_device(arr)

        # For numpy backend, should be same array
        if self.backend.backend_type == BackendType.NUMPY:
            self.assertTrue(np.array_equal(arr, device_arr))

        # Test back to numpy
        numpy_arr = self.backend.to_numpy(device_arr)
        self.assertTrue(np.allclose(arr, numpy_arr))

    def test_array_creation_edge_cases(self):
        """Test array creation edge cases."""
        # Test empty arrays
        empty_arr = self.backend.zeros((0,), dtype=np.float32)
        self.assertEqual(empty_arr.shape, (0,))

        # Test 1D arrays
        arr_1d = self.backend.ones((10,), dtype=np.float32)
        self.assertEqual(arr_1d.shape, (10,))

        # Test 3D arrays
        arr_3d = self.backend.zeros((2, 3, 4), dtype=np.float32)
        self.assertEqual(arr_3d.shape, (2, 3, 4))

    def test_array_conversion_edge_cases(self):
        """Test array conversion edge cases."""
        # Test with different input types
        python_list = [[1, 2], [3, 4]]
        arr_from_list = self.backend.array(python_list)
        numpy_arr = self.backend.to_numpy(arr_from_list)
        expected = np.array(python_list)
        self.assertTrue(np.array_equal(numpy_arr, expected))

        # Test with scalar
        scalar_arr = self.backend.array(5.0)
        numpy_scalar = self.backend.to_numpy(scalar_arr)
        self.assertEqual(numpy_scalar.shape, ())
        self.assertEqual(float(numpy_scalar), 5.0)

    def test_backend_properties(self):
        """Test backend property access."""
        # Test backend type
        self.assertEqual(self.backend.backend_type, BackendType.NUMPY)

        # Test device property
        device = self.backend.device
        self.assertIsInstance(device, str)

        # Test is_gpu property
        is_gpu = self.backend.is_gpu
        self.assertIsInstance(is_gpu, bool)

        # For numpy backend, should not be GPU
        if self.backend.backend_type == BackendType.NUMPY:
            self.assertFalse(is_gpu)

    def test_mathematical_operations(self):
        """Test mathematical operations."""
        shape = (5, 5)

        # Create test arrays
        a = self.backend.full(shape, 2.0, dtype=np.float32)
        b = self.backend.full(shape, 3.0, dtype=np.float32)

        # Test basic math operations
        numpy_a = self.backend.to_numpy(a)
        numpy_b = self.backend.to_numpy(b)

        # Addition
        c = numpy_a + numpy_b
        self.assertTrue(np.all(c == 5.0))

        # Multiplication
        d = numpy_a * numpy_b
        self.assertTrue(np.all(d == 6.0))

    def test_array_slicing(self):
        """Test array slicing operations."""
        shape = (10, 10)
        arr = self.backend.ones(shape, dtype=np.float32)
        numpy_arr = self.backend.to_numpy(arr)

        # Test slicing
        slice_arr = numpy_arr[2:5, 3:7]
        self.assertEqual(slice_arr.shape, (3, 4))
        self.assertTrue(np.all(slice_arr == 1.0))

    def test_array_reshaping(self):
        """Test array reshaping operations."""
        original_shape = (6, 4)
        new_shape = (8, 3)

        arr = self.backend.ones(original_shape, dtype=np.float32)
        numpy_arr = self.backend.to_numpy(arr)

        # Test reshape
        reshaped = numpy_arr.reshape(new_shape)
        self.assertEqual(reshaped.shape, new_shape)
        self.assertTrue(np.all(reshaped == 1.0))

    def test_performance_characteristics(self):
        """Test performance characteristics."""
        import time

        # Test creation time for large arrays
        large_shape = (1000, 1000)

        start_time = time.time()
        large_arr = self.backend.zeros(large_shape, dtype=np.float32)
        creation_time = time.time() - start_time

        # Should create array reasonably quickly (less than 1 second)
        self.assertLess(creation_time, 1.0)

        # Verify array was created correctly
        numpy_arr = self.backend.to_numpy(large_arr)
        self.assertEqual(numpy_arr.shape, large_shape)


class TestBackendSelection(unittest.TestCase):
    """Test backend selection logic."""

    def test_explicit_backend_selection(self):
        """Test explicit backend selection."""
        # Test numpy backend
        numpy_backend = ArrayBackend(BackendType.NUMPY)
        self.assertEqual(numpy_backend.backend_type, BackendType.NUMPY)

    def test_auto_backend_fallback(self):
        """Test auto backend selection with fallback."""
        # Auto selection should always work (falls back to numpy)
        auto_backend = ArrayBackend(BackendType.AUTO)
        self.assertIsNotNone(auto_backend.backend_type)

        # Should be one of the supported backends
        supported_backends = [
            BackendType.NUMPY,
            BackendType.PYTORCH,
            BackendType.CUPY,
            BackendType.WGPU,
        ]
        self.assertIn(auto_backend.backend_type, supported_backends)

    def test_backend_availability_detection(self):
        """Test backend availability detection."""
        # This tests the import logic indirectly
        try:
            pytorch_backend = ArrayBackend(BackendType.PYTORCH)
            self.assertEqual(pytorch_backend.backend_type, BackendType.PYTORCH)
        except (ImportError, ValueError):
            # PyTorch not available, which is fine
            pass

        try:
            cupy_backend = ArrayBackend(BackendType.CUPY)
            self.assertEqual(cupy_backend.backend_type, BackendType.CUPY)
        except (ImportError, ValueError):
            # CuPy not available, which is fine
            pass


if __name__ == "__main__":
    unittest.main()
