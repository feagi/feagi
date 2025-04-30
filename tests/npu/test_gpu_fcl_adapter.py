"""
Tests for the GPU FCL adapter.
"""

import unittest
import pytest
import numpy as np
from unittest.mock import patch, MagicMock
import random

from feagi.npu.fcl_manager import BitMap
from feagi.npu.gpu_fcl_adapter import GPUBitMap, create_gpu_accelerated_fcl, GPUAcceleratedFCL


# Mock WebGPUTensor for testing
class MockWebGPUTensor:
    def __init__(self, shape, data=None):
        self.shape = shape
        self.size = np.prod(shape)
        self.data = data or np.zeros(shape, dtype=np.uint32)
        
    def __repr__(self):
        return f"MockWebGPUTensor(shape={self.shape})"


# Mock WebGPU backend for testing
class MockWebGPUBackend:
    def __init__(self):
        self.name = "mock_webgpu"
        self.initialized = True
        
    def initialize(self):
        return True
        
    def create_tensor(self, shape, dtype=np.float32):
        return MockWebGPUTensor(shape)
        
    def from_numpy(self, array):
        return MockWebGPUTensor(array.shape, array)
        
    def to_numpy(self, tensor):
        return tensor.data.copy()
        
    def bitmap_or(self, bitmap1, bitmap2):
        result = bitmap1.data | bitmap2.data
        return MockWebGPUTensor(result.shape, result)
        
    def bitmap_and(self, bitmap1, bitmap2):
        result = bitmap1.data & bitmap2.data
        return MockWebGPUTensor(result.shape, result)
        
    def bitmap_xor(self, bitmap1, bitmap2):
        result = bitmap1.data ^ bitmap2.data
        return MockWebGPUTensor(result.shape, result)
        
    def bitmap_subtract(self, bitmap1, bitmap2):
        result = bitmap1.data & ~bitmap2.data
        return MockWebGPUTensor(result.shape, result)
        
    def synchronize(self):
        pass
        
    def supports_capability(self, capability):
        return capability == "bitmap_operations"


class TestGPUBitMap(unittest.TestCase):
    
    @patch('feagi.npu.gpu_fcl_adapter.get_backend')
    def setUp(self, mock_get_backend):
        # Create mock backend
        self.mock_backend = MockWebGPUBackend()
        mock_get_backend.return_value = self.mock_backend
        
    def test_initialization_empty(self):
        """Test initialization of empty GPU bitmap."""
        bitmap = GPUBitMap()
        self.assertEqual(len(bitmap), 0)
        self.assertTrue(bitmap.is_empty())
        
    def test_initialization_with_elements(self):
        """Test initialization with elements."""
        elements = {1, 5, 10, 100}
        bitmap = GPUBitMap(elements)
        self.assertEqual(len(bitmap), len(elements))
        for element in elements:
            self.assertIn(element, bitmap)
            
    def test_add_element(self):
        """Test adding an element to the bitmap."""
        bitmap = GPUBitMap()
        bitmap.add(42)
        self.assertEqual(len(bitmap), 1)
        self.assertIn(42, bitmap)
        
    def test_clear(self):
        """Test clearing the bitmap."""
        bitmap = GPUBitMap({1, 2, 3})
        self.assertEqual(len(bitmap), 3)
        bitmap.clear()
        self.assertEqual(len(bitmap), 0)
        self.assertTrue(bitmap.is_empty())
        
    def test_copy(self):
        """Test copying the bitmap."""
        bitmap = GPUBitMap({1, 2, 3})
        copy = bitmap.copy()
        self.assertEqual(len(copy), len(bitmap))
        for element in bitmap:
            self.assertIn(element, copy)
            
    def test_or_operation(self):
        """Test OR operation between bitmaps."""
        bitmap1 = GPUBitMap({1, 2, 3})
        bitmap2 = GPUBitMap({3, 4, 5})
        result = bitmap1 | bitmap2
        self.assertEqual(len(result), 5)
        for element in {1, 2, 3, 4, 5}:
            self.assertIn(element, result)
            
    def test_and_operation(self):
        """Test AND operation between bitmaps."""
        bitmap1 = GPUBitMap({1, 2, 3})
        bitmap2 = GPUBitMap({3, 4, 5})
        result = bitmap1 & bitmap2
        self.assertEqual(len(result), 1)
        self.assertIn(3, result)
        
    def test_sub_operation(self):
        """Test subtraction operation between bitmaps."""
        bitmap1 = GPUBitMap({1, 2, 3})
        bitmap2 = GPUBitMap({3, 4, 5})
        result = bitmap1 - bitmap2
        self.assertEqual(len(result), 2)
        for element in {1, 2}:
            self.assertIn(element, result)
        self.assertNotIn(3, result)
        
    def test_xor_operation(self):
        """Test XOR operation between bitmaps."""
        bitmap1 = GPUBitMap({1, 2, 3})
        bitmap2 = GPUBitMap({3, 4, 5})
        result = bitmap1 ^ bitmap2
        self.assertEqual(len(result), 4)
        for element in {1, 2, 4, 5}:
            self.assertIn(element, result)
        self.assertNotIn(3, result)
        
    def test_conversion_to_cpu_bitmap(self):
        """Test conversion to CPU bitmap."""
        elements = {1, 5, 10, 100}
        gpu_bitmap = GPUBitMap(elements)
        cpu_bitmap = gpu_bitmap.to_cpu_bitmap()
        
        self.assertEqual(len(cpu_bitmap), len(elements))
        for element in elements:
            self.assertIn(element, cpu_bitmap)


class TestGPUAcceleratedFCL(unittest.TestCase):
    
    @patch('feagi.npu.gpu_fcl_adapter.get_backend')
    def setUp(self, mock_get_backend):
        # Create mock backend
        self.mock_backend = MockWebGPUBackend()
        mock_get_backend.return_value = self.mock_backend
        
        # Create GPU-accelerated FCL
        self.fcl = GPUAcceleratedFCL(default_window_size=10, backend=self.mock_backend)
        
        # Add some test data
        neurons_by_area = {
            1: BitMap({1, 2, 3}),
            2: BitMap({10, 11, 12})
        }
        self.fcl.update_fcl(0, neurons_by_area)
        
        neurons_by_area = {
            1: BitMap({2, 3, 4}),
            2: BitMap({11, 12, 13})
        }
        self.fcl.update_fcl(1, neurons_by_area)
        
    def test_get_fcl_delta(self):
        """Test computing FCL delta with GPU acceleration."""
        # Get delta between timestep 0 and 1
        delta = self.fcl.get_fcl_delta(0, 1)
        
        # Expected: neurons that fired at t=1 but not at t=0
        expected = {4, 13}
        self.assertEqual(len(delta), len(expected))
        for neuron in expected:
            self.assertIn(neuron, delta)
        
    def test_get_consistently_active_neurons(self):
        """Test getting consistently active neurons with GPU acceleration."""
        # Get neurons that fired in both timesteps
        consistent = self.fcl.get_consistently_active_neurons(2)
        
        # Expected: neurons that fired at both t=0 and t=1
        expected = {2, 3, 11, 12}
        self.assertEqual(len(consistent), len(expected))
        for neuron in expected:
            self.assertIn(neuron, consistent)
        
    def test_get_neurons_fired_in_last_n_steps(self):
        """Test getting neurons that fired in any of the last n steps."""
        # Get neurons that fired in either timestep
        fired = self.fcl.get_neurons_fired_in_last_n_steps(2)
        
        # Expected: neurons that fired at either t=0 or t=1
        expected = {1, 2, 3, 4, 10, 11, 12, 13}
        self.assertEqual(len(fired), len(expected))
        for neuron in expected:
            self.assertIn(neuron, fired)


@patch('feagi.npu.gpu_fcl_adapter.get_backend')
class TestCreateGPUAcceleratedFCL(unittest.TestCase):
    
    def test_create_with_gpu_backend(self, mock_get_backend):
        """Test creation of GPU-accelerated FCL with compatible backend."""
        # Setup mock backend with bitmap operations
        mock_backend = MockWebGPUBackend()
        mock_get_backend.return_value = mock_backend
        
        # Create FCL
        fcl = create_gpu_accelerated_fcl()
        
        # Should be GPU-accelerated
        self.assertIsInstance(fcl, GPUAcceleratedFCL)
        
    def test_create_with_no_backend(self, mock_get_backend):
        """Test fallback to CPU when no backend is available."""
        # No backend available
        mock_get_backend.return_value = None
        
        # Create FCL
        fcl = create_gpu_accelerated_fcl()
        
        # Should fall back to CPU implementation
        from feagi.npu.fcl_manager import EnhancedHierarchicalFCL
        self.assertIsInstance(fcl, EnhancedHierarchicalFCL)
        
    def test_create_with_incompatible_backend(self, mock_get_backend):
        """Test fallback to CPU when backend doesn't support bitmap operations."""
        # Create backend without bitmap operations
        mock_backend = MagicMock()
        mock_backend.supports_capability.return_value = False
        mock_get_backend.return_value = mock_backend
        
        # Create FCL
        fcl = create_gpu_accelerated_fcl()
        
        # Should fall back to CPU implementation
        from feagi.npu.fcl_manager import EnhancedHierarchicalFCL
        self.assertIsInstance(fcl, EnhancedHierarchicalFCL)


if __name__ == '__main__':
    unittest.main() 