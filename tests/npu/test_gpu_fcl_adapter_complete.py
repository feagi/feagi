"""
Comprehensive tests for the GPU FCL Adapter.

This module provides test coverage for the GPUBitMap class in the GPU FCL Adapter module,
which performs optimized bitmap operations for fire candidate lists.
"""

import pytest
import numpy as np
from unittest.mock import patch, MagicMock, Mock

from feagi.npu.gpu_fcl_adapter import GPUBitMap, GPUAcceleratedFCL, create_gpu_accelerated_fcl
from feagi.core.backend import BackendType, get_backend


class TestGPUBitMapOperations:
    """Tests for GPU bitmap operations."""
    
    @pytest.fixture
    def mock_backend(self):
        """Mock backend for testing."""
        mock = Mock()
        mock.from_numpy = Mock(return_value=Mock())
        mock.to_numpy = Mock(return_value=np.zeros(125, dtype=np.uint32))
        mock.create_tensor = Mock(return_value=Mock())
        return mock
    
    @pytest.fixture
    def gpu_bitmap(self, mock_backend):
        """Create a GPU bitmap for testing."""
        with patch('feagi.npu.gpu_fcl_adapter.get_backend', return_value=mock_backend):
            bitmap = GPUBitMap()
            return bitmap
    
    def test_gpu_bitmap_init_empty(self, mock_backend):
        """Test empty GPUBitMap initialization."""
        with patch('feagi.npu.gpu_fcl_adapter.get_backend', return_value=mock_backend):
            bitmap = GPUBitMap()
            
            # Verify create_tensor was called
            mock_backend.create_tensor.assert_called_once()
            assert bitmap._elements_cache == set()
            assert bitmap._cache_valid is True
    
    def test_gpu_bitmap_init_with_elements(self, mock_backend):
        """Test GPUBitMap initialization with elements."""
        with patch('feagi.npu.gpu_fcl_adapter.get_backend', return_value=mock_backend):
            elements = [1, 2, 3]
            bitmap = GPUBitMap(elements)
            
            # Verify from_numpy was called
            assert mock_backend.from_numpy.called
            assert bitmap._elements_cache == set(elements)
            assert bitmap._cache_valid is True
    
    def test_gpu_bitmap_add(self, gpu_bitmap, mock_backend):
        """Test adding element to GPUBitMap."""
        # Setup mock to return a modifiable array
        mock_array = np.zeros(5, dtype=np.uint32)
        mock_backend.to_numpy.return_value = mock_array
        
        # Add element
        gpu_bitmap.add(42)
        
        # Check cache updated
        assert 42 in gpu_bitmap._elements_cache
        
        # Verify backend methods called
        assert mock_backend.to_numpy.called
        assert mock_backend.from_numpy.called
    
    def test_gpu_bitmap_clear(self, gpu_bitmap, mock_backend):
        """Test clearing GPUBitMap."""
        # Setup initial state
        gpu_bitmap._elements_cache = {1, 2, 3}
        
        # Clear bitmap
        gpu_bitmap.clear()
        
        # Check cache cleared
        assert len(gpu_bitmap._elements_cache) == 0
        assert gpu_bitmap._cache_valid is True
        
        # Verify new empty bitmap created
        assert mock_backend.to_numpy.called
        assert mock_backend.from_numpy.called
    
    def test_gpu_bitmap_copy(self, gpu_bitmap, mock_backend):
        """Test copying GPUBitMap."""
        # Setup initial state
        gpu_bitmap._elements_cache = {1, 2, 3}
        
        # Copy bitmap
        copy = gpu_bitmap.copy()
        
        # Check cache copied
        assert copy._elements_cache == {1, 2, 3}
        assert copy._cache_valid is True
        
        # Verify backend methods called
        assert mock_backend.to_numpy.called
        assert mock_backend.from_numpy.called
    
    def test_gpu_bitmap_or(self, mock_backend):
        """Test OR operation between GPUBitMaps."""
        with patch('feagi.npu.gpu_fcl_adapter.get_backend', return_value=mock_backend):
            # Create bitmaps with different elements
            bitmap1 = GPUBitMap([1, 2])
            bitmap2 = GPUBitMap([2, 3])
            
            # Mock to_numpy to return appropriate arrays
            array1 = np.zeros(5, dtype=np.uint32)
            array1[0] = 6  # bits for 1,2
            array2 = np.zeros(5, dtype=np.uint32)
            array2[0] = 12  # bits for 2,3
            
            mock_backend.to_numpy.side_effect = [array1, array2]
            
            # Perform OR operation
            result = bitmap1 | bitmap2
            
            # Check cache
            assert result._elements_cache == {1, 2, 3}
            assert result._cache_valid is True
    
    def test_gpu_bitmap_and(self, mock_backend):
        """Test AND operation between GPUBitMaps."""
        with patch('feagi.npu.gpu_fcl_adapter.get_backend', return_value=mock_backend):
            # Create bitmaps with overlapping elements
            bitmap1 = GPUBitMap([1, 2, 3])
            bitmap2 = GPUBitMap([2, 3, 4])
            
            # Mock to_numpy to return appropriate arrays
            array1 = np.zeros(5, dtype=np.uint32)
            array1[0] = 14  # bits for 1,2,3
            array2 = np.zeros(5, dtype=np.uint32)
            array2[0] = 28  # bits for 2,3,4
            
            mock_backend.to_numpy.side_effect = [array1, array2]
            
            # Perform AND operation
            result = bitmap1 & bitmap2
            
            # Check cache
            assert result._elements_cache == {2, 3}
            assert result._cache_valid is True
    
    def test_gpu_bitmap_sub(self, mock_backend):
        """Test subtraction operation between GPUBitMaps."""
        with patch('feagi.npu.gpu_fcl_adapter.get_backend', return_value=mock_backend):
            # Create bitmaps
            bitmap1 = GPUBitMap([1, 2, 3])
            bitmap2 = GPUBitMap([2, 4])
            
            # Mock to_numpy to return appropriate arrays
            array1 = np.zeros(5, dtype=np.uint32)
            array1[0] = 14  # bits for 1,2,3
            array2 = np.zeros(5, dtype=np.uint32)
            array2[0] = 20  # bits for 2,4
            
            mock_backend.to_numpy.side_effect = [array1, array2]
            
            # Perform subtraction operation
            result = bitmap1 - bitmap2
            
            # Check cache
            assert result._elements_cache == {1, 3}
            assert result._cache_valid is True
    
    def test_gpu_bitmap_contains(self, gpu_bitmap):
        """Test checking if GPUBitMap contains an element."""
        # Setup initial state with cache
        gpu_bitmap._elements_cache = {1, 2, 3}
        gpu_bitmap._cache_valid = True
        
        # Check contains
        assert 2 in gpu_bitmap
        assert 4 not in gpu_bitmap
    
    def test_gpu_bitmap_len(self, gpu_bitmap):
        """Test getting length of GPUBitMap."""
        # Setup initial state with cache
        gpu_bitmap._elements_cache = {1, 2, 3}
        gpu_bitmap._cache_valid = True
        
        # Check length
        assert len(gpu_bitmap) == 3
    
    def test_gpu_bitmap_is_empty(self, gpu_bitmap):
        """Test checking if GPUBitMap is empty."""
        # Setup empty bitmap
        gpu_bitmap._elements_cache = set()
        gpu_bitmap._cache_valid = True
        
        # Check is_empty
        assert gpu_bitmap.is_empty()
        
        # Add element
        gpu_bitmap._elements_cache = {1}
        
        # Check not empty
        assert not gpu_bitmap.is_empty()


class TestGPUAcceleratedFCL:
    """Tests for GPU Accelerated FCL implementation."""
    
    @pytest.fixture
    def mock_backend(self):
        """Create a mock backend for testing."""
        mock = Mock()
        mock.name = "TestBackend"
        mock.supports_capability = Mock(return_value=True)
        mock.bitmap_or = Mock()
        return mock
    
    def test_create_gpu_accelerated_fcl_with_backend(self, mock_backend):
        """Test creating GPU FCL when backend is available."""
        with patch('feagi.npu.gpu_fcl_adapter.get_backend', return_value=mock_backend):
            with patch('feagi.npu.gpu_fcl_adapter.EnhancedFCLManager'):
                fcl = create_gpu_accelerated_fcl()
                assert isinstance(fcl, GPUAcceleratedFCL)
    
    def test_create_gpu_accelerated_fcl_no_backend(self):
        """Test creating GPU FCL when no backend is available."""
        with patch('feagi.npu.gpu_fcl_adapter.get_backend', return_value=None):
            fcl = create_gpu_accelerated_fcl()
            assert not isinstance(fcl, GPUAcceleratedFCL)
    
    def test_create_gpu_accelerated_fcl_unsupported_backend(self):
        """Test creating GPU FCL when backend doesn't support bitmap operations."""
        mock_backend = Mock()
        mock_backend.supports_capability = Mock(return_value=False)
        
        with patch('feagi.npu.gpu_fcl_adapter.get_backend', return_value=mock_backend):
            fcl = create_gpu_accelerated_fcl()
            assert not isinstance(fcl, GPUAcceleratedFCL)
    
    def test_gpu_accelerated_fcl_methods_delegation(self, mock_backend):
        """Test method delegation to CPU FCL manager."""
        # Create a mock object with a test_method attribute
        mock_fcl_instance = MagicMock()
        mock_fcl_instance.test_method.return_value = "delegated"
        
        # We need to patch the actual instance that gets created
        with patch('feagi.npu.fcl_manager.EnhancedFCLManager', return_value=mock_fcl_instance):
            # Create GPU FCL
            fcl = GPUAcceleratedFCL(mock_backend)
            
            # Call delegated method
            result = fcl.test_method()
            
            # Verify delegation happened
            assert result == "delegated"
            mock_fcl_instance.test_method.assert_called_once()


if __name__ == "__main__":
    pytest.main(["-v", __file__]) 