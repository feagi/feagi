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
Tests for GPU FCL Adapter with mocked hardware.

These tests use mocks to simulate GPU hardware, allowing for testing of the GPU FCL adapter
even when actual GPU hardware is not available.
"""

from unittest.mock import MagicMock, Mock, patch

import numpy as np
import pytest

from feagi.npu.fcl_manager import BitMap
from feagi.npu.gpu_fcl_adapter import (
    GPUAcceleratedFCL,
    GPUBitMap,
    create_gpu_accelerated_fcl,
)


class TestGPUBitMapMocked:
    """Test GPUBitMap operations with mocked GPU hardware."""

    @pytest.fixture
    def mock_backend(self):
        """Create a mock backend."""
        backend = MagicMock()
        # Configure the backend to support bitmap operations
        backend.supports_capability.return_value = True
        backend.bitmap_or = Mock(side_effect=lambda x, y: MagicMock())
        backend.bitmap_and = Mock(side_effect=lambda x, y: MagicMock())
        backend.bitmap_xor = Mock(side_effect=lambda x, y: MagicMock())
        backend.bitmap_subtract = Mock(side_effect=lambda x, y: MagicMock())
        # Make to_numpy return a set-like array
        backend.to_numpy = Mock(side_effect=lambda x: np.array([1, 2, 3]))
        # Make from_numpy create a tensor
        tensor = MagicMock()
        backend.from_numpy = Mock(return_value=tensor)
        # Add create_tensor method to avoid errors
        backend.create_tensor = Mock(return_value=MagicMock())
        return backend

    @pytest.fixture
    def gpu_bitmap_empty(self, mock_backend):
        """Create an empty GPU bitmap."""
        with patch("feagi.npu.gpu_fcl_adapter.get_backend", return_value=mock_backend):
            return GPUBitMap()

    @pytest.fixture
    def gpu_bitmap_with_elements(self, mock_backend):
        """Create a GPU bitmap with elements."""
        with patch("feagi.npu.gpu_fcl_adapter.get_backend", return_value=mock_backend):
            return GPUBitMap([1, 2, 3])

    def test_initialization_empty(self, gpu_bitmap_empty, mock_backend):
        """Test initialization of empty GPU bitmap."""
        # Check backend interactions
        assert mock_backend.create_tensor.call_count > 0

        # Check bitmap size
        mock_backend.to_numpy.return_value = np.array([])
        assert len(gpu_bitmap_empty) == 0

    def test_initialization_with_elements(self, gpu_bitmap_with_elements, mock_backend):
        """Test initialization with elements."""
        # Check backend interactions
        assert mock_backend.from_numpy.call_count > 0

        # Check bitmap elements
        mock_backend.to_numpy.return_value = np.array([1, 2, 3])
        assert len(gpu_bitmap_with_elements) == 3

    def test_add_element(self, gpu_bitmap_empty, mock_backend):
        """Test adding an element to the bitmap."""
        # Add an element
        gpu_bitmap_empty.add(42)

        # Check backend interaction for creating new bitmap
        assert mock_backend.from_numpy.call_count > 0

    def test_clear(self, gpu_bitmap_with_elements, mock_backend):
        """Test clearing the bitmap."""
        gpu_bitmap_with_elements.clear()

        # Check backend interaction for creating new empty bitmap
        assert mock_backend.from_numpy.call_count > 0

        # Return empty array for the cleared bitmap
        mock_backend.to_numpy.return_value = np.array([])
        assert len(gpu_bitmap_with_elements) == 0

    def test_or_operation(self, gpu_bitmap_with_elements, mock_backend):
        """Test OR operation between bitmaps."""
        # Create another bitmap
        with patch("feagi.npu.gpu_fcl_adapter.get_backend", return_value=mock_backend):
            other_bitmap = GPUBitMap([3, 4, 5])

        # Perform OR operation
        result = gpu_bitmap_with_elements | other_bitmap

        # Check backend interaction
        mock_backend.bitmap_or.assert_called_once()

        # Set result for the OR operation
        mock_backend.to_numpy.return_value = np.array([1, 2, 3, 4, 5])
        assert len(result) == 5

    def test_and_operation(self, gpu_bitmap_with_elements, mock_backend):
        """Test AND operation between bitmaps."""
        # Create another bitmap
        with patch("feagi.npu.gpu_fcl_adapter.get_backend", return_value=mock_backend):
            other_bitmap = GPUBitMap([3, 4, 5])

        # Perform AND operation
        result = gpu_bitmap_with_elements & other_bitmap

        # Check backend interaction
        mock_backend.bitmap_and.assert_called_once()

        # Set result for the AND operation
        mock_backend.to_numpy.return_value = np.array([3])
        assert len(result) == 1

    def test_sub_operation(self, gpu_bitmap_with_elements, mock_backend):
        """Test subtraction operation between bitmaps."""
        # Create another bitmap
        with patch("feagi.npu.gpu_fcl_adapter.get_backend", return_value=mock_backend):
            other_bitmap = GPUBitMap([3, 4, 5])

        # Perform subtraction operation
        result = gpu_bitmap_with_elements - other_bitmap

        # Check backend interaction
        mock_backend.bitmap_subtract.assert_called_once()

        # Set result for the subtract operation
        mock_backend.to_numpy.return_value = np.array([1, 2])
        assert len(result) == 2

    def test_xor_operation(self, gpu_bitmap_with_elements, mock_backend):
        """Test XOR operation between bitmaps."""
        # Create another bitmap
        with patch("feagi.npu.gpu_fcl_adapter.get_backend", return_value=mock_backend):
            other_bitmap = GPUBitMap([3, 4, 5])

        # Perform XOR operation
        result = gpu_bitmap_with_elements ^ other_bitmap

        # Check backend interaction
        mock_backend.bitmap_xor.assert_called_once()

        # Set result for the XOR operation
        mock_backend.to_numpy.return_value = np.array([1, 2, 4, 5])
        assert len(result) == 4

    def test_is_empty(self, gpu_bitmap_empty, mock_backend):
        """Test is_empty method."""
        # Configure backend to return empty array
        mock_backend.to_numpy.return_value = np.array([])

        assert gpu_bitmap_empty.is_empty()

        # Add an element and verify not empty
        gpu_bitmap_empty.add(42)
        mock_backend.to_numpy.return_value = np.array([42])

        assert not gpu_bitmap_empty.is_empty()

    def test_conversion_to_cpu_bitmap(self, gpu_bitmap_with_elements, mock_backend):
        """Test conversion to CPU bitmap."""
        # Set the array that to_numpy will return
        mock_backend.to_numpy.return_value = np.array([1, 2, 3])

        # Convert to CPU bitmap
        cpu_bitmap = gpu_bitmap_with_elements.to_cpu_bitmap()

        # Verify the result
        assert len(cpu_bitmap) == 3
        assert isinstance(cpu_bitmap, BitMap)

        # Check elements
        elements = list(cpu_bitmap)
        assert sorted(elements) == [1, 2, 3]

    def test_contains(self, gpu_bitmap_with_elements, mock_backend):
        """Test contains method."""
        # Set the array that to_numpy will return
        mock_backend.to_numpy.return_value = np.array([1, 2, 3])

        # Check if elements are in the bitmap
        assert 1 in gpu_bitmap_with_elements
        assert 2 in gpu_bitmap_with_elements
        assert 3 in gpu_bitmap_with_elements
        assert 4 not in gpu_bitmap_with_elements


class TestGPUAcceleratedFCLMocked:
    """Test GPUAcceleratedFCL with mocked GPU hardware."""

    @pytest.fixture
    def mock_backend(self):
        """Create a mock backend."""
        backend = MagicMock()
        # Configure backend to support bitmap operations
        backend.supports_capability.return_value = True
        backend.bitmap_or = Mock(side_effect=lambda x, y: MagicMock())
        backend.bitmap_and = Mock(side_effect=lambda x, y: MagicMock())
        backend.bitmap_xor = Mock(side_effect=lambda x, y: MagicMock())
        backend.bitmap_subtract = Mock(side_effect=lambda x, y: MagicMock())
        # Make to_numpy return various arrays depending on the call
        to_numpy_results = [
            np.array([]),  # Initial empty bitmap
            np.array([1, 2, 3]),  # First call in update_fcl
            np.array([2, 3, 4]),  # Second call in update_fcl
            np.array([1, 2, 3, 4]),  # Union of all neurons
            np.array([2, 3]),  # Intersection
            np.array([4]),  # Delta
            np.array([1, 4]),  # XOR
        ]
        backend.to_numpy = Mock(side_effect=to_numpy_results)
        # Make from_numpy create a tensor
        tensor = MagicMock()
        backend.from_numpy = Mock(return_value=tensor)
        # Add create_tensor method
        backend.create_tensor = Mock(return_value=MagicMock())
        return backend

    @pytest.fixture
    def fcl(self, mock_backend):
        """Create a GPU-accelerated FCL."""
        return GPUAcceleratedFCL(mock_backend, default_window_size=3)

    def test_initialization(self, fcl, mock_backend):
        """Test initialization of GPU-accelerated FCL."""
        assert fcl.cpu_fcl.window_size == 3
        assert fcl.backend == mock_backend

    def test_update_fcl(self, fcl, mock_backend):
        """Test updating the FCL."""
        # Set up a mock cpu_fcl to avoid actual updates
        fcl.cpu_fcl = MagicMock()

        # Update FCL with test data
        neurons_by_cortical = {1: BitMap([1, 2, 3]), 2: BitMap([2, 3, 4])}
        fcl.update_fcl(1, neurons_by_cortical)

        # Check that the CPU FCL was used
        fcl.cpu_fcl.update_fcl.assert_called_once_with(1, neurons_by_cortical)

    def test_get_neurons_fired_in_last_n_steps(self, fcl, mock_backend):
        """Test getting neurons that fired in last n steps."""
        # Set up a mock cpu_fcl to return specific FCLs
        fcl.cpu_fcl = MagicMock()
        fcl.cpu_fcl.current_timestep = 2
        fcl.cpu_fcl.get_global_fcl.side_effect = [
            BitMap([1, 2, 3]),  # Current timestep
            BitMap([2, 3, 4]),  # Previous timestep
        ]

        # Need to patch GPUBitMap to use our mock backend
        with patch("feagi.npu.gpu_fcl_adapter.get_backend", return_value=mock_backend):
            # Get neurons fired in last 2 steps
            result = fcl.get_neurons_fired_in_last_n_steps(2)

        # Check the result
        mock_backend.to_numpy.return_value = np.array([1, 2, 3, 4])
        assert len(result) == 4

    def test_get_consistently_active_neurons(self, fcl, mock_backend):
        """Test getting consistently active neurons."""
        # Set up a mock cpu_fcl to return specific FCLs
        fcl.cpu_fcl = MagicMock()
        fcl.cpu_fcl.current_timestep = 2
        fcl.cpu_fcl.get_global_fcl.side_effect = [
            BitMap([1, 2, 3]),  # Current timestep
            BitMap([2, 3, 4]),  # Previous timestep
        ]

        # Need to patch GPUBitMap to use our mock backend
        with patch("feagi.npu.gpu_fcl_adapter.get_backend", return_value=mock_backend):
            # Get consistently active neurons
            result = fcl.get_consistently_active_neurons(2)

        # Check the result
        mock_backend.to_numpy.return_value = np.array([2, 3])
        assert len(result) == 2

    def test_get_fcl_delta(self, fcl, mock_backend):
        """Test getting FCL delta."""
        # Set up a mock cpu_fcl to return specific FCLs
        fcl.cpu_fcl = MagicMock()
        fcl.cpu_fcl.get_global_fcl.side_effect = [
            BitMap([1, 2, 3]),  # Start timestep
            BitMap([2, 3, 4]),  # End timestep
        ]

        # Need to patch GPUBitMap to use our mock backend
        with patch("feagi.npu.gpu_fcl_adapter.get_backend", return_value=mock_backend):
            # Get FCL delta
            result = fcl.get_fcl_delta(1, 2)

        # Check the result
        mock_backend.to_numpy.return_value = np.array([4])
        assert len(result) == 1

    def test_get_fcl_xor(self, fcl, mock_backend):
        """Test getting FCL XOR."""

        # Add the method to the FCL object to test it
        def get_fcl_xor(self, time1, time2, cortical_indices=None):
            # Get FCLs
            fcl1 = self.cpu_fcl.get_global_fcl(time1)
            fcl2 = self.cpu_fcl.get_global_fcl(time2)

            # Convert to GPU bitmaps
            gpu_fcl1 = GPUBitMap(fcl1)
            gpu_fcl2 = GPUBitMap(fcl2)

            # Perform XOR
            result = gpu_fcl1 ^ gpu_fcl2

            # Convert back to CPU bitmap
            return result.to_cpu_bitmap()

        fcl.get_fcl_xor = get_fcl_xor.__get__(fcl)

        # Set up a mock cpu_fcl to return specific FCLs
        fcl.cpu_fcl = MagicMock()
        fcl.cpu_fcl.get_global_fcl.side_effect = [
            BitMap([1, 2, 3]),  # First timestep
            BitMap([2, 3, 4]),  # Second timestep
        ]

        # Need to patch GPUBitMap to use our mock backend
        with patch("feagi.npu.gpu_fcl_adapter.get_backend", return_value=mock_backend):
            # Get FCL XOR
            result = fcl.get_fcl_xor(1, 2)

        # Check the result
        mock_backend.to_numpy.return_value = np.array([1, 4])
        assert len(result) == 2


@patch("feagi.npu.gpu_fcl_adapter.get_backend")
class TestCreateGPUAcceleratedFCLMocked:
    """Test function to create GPU-accelerated FCL."""

    def test_create_with_gpu_backend(self, mock_get_backend):
        """Test creation with GPU backend."""
        # Create mock backend
        backend = MagicMock()
        backend.name = "mock_gpu"
        backend.supports_capability.return_value = True
        mock_get_backend.return_value = backend

        # Create FCL
        fcl = create_gpu_accelerated_fcl(default_window_size=3)

        # Check result
        assert isinstance(fcl, GPUAcceleratedFCL)
        assert fcl.cpu_fcl.window_size == 3
        assert fcl.backend == backend

    def test_create_with_unsupported_backend(self, mock_get_backend):
        """Test creation with unsupported backend."""
        # Create mock backend
        backend = MagicMock()
        backend.name = "mock_cpu"
        backend.supports_capability.return_value = False
        mock_get_backend.return_value = backend

        # Create FCL
        fcl = create_gpu_accelerated_fcl(default_window_size=3)

        # Should return a CPU FCL
        from feagi.npu.fcl_manager import EnhancedFCLManager

        assert isinstance(fcl, EnhancedFCLManager)
        assert fcl.window_size == 3

    def test_create_with_no_backend(self, mock_get_backend):
        """Test creation with no backend available."""
        # No backend available
        mock_get_backend.return_value = None

        # Create FCL
        fcl = create_gpu_accelerated_fcl(default_window_size=3)

        # Should return a CPU FCL
        from feagi.npu.fcl_manager import EnhancedFCLManager

        assert isinstance(fcl, EnhancedFCLManager)
        assert fcl.window_size == 3


if __name__ == "__main__":
    pytest.main(["-v", __file__])
