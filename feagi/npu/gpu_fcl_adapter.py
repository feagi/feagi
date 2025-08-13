"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
GPU Adapter for Fire Candidate List (FCL) operations.

This module bridges the FCL Manager with GPU-accelerated bitmap operations,
providing high-performance implementations of FCL operations when a GPU backend
is available.
"""

from feagi.utils.logger import setup_logger

logger = setup_logger()
from typing import Dict, List, Optional, Set, Union

import numpy as np

from feagi.core.backend import get_backend
from feagi.core.backend.interface import BackendInterface
from feagi.npu.fcl_manager import BitMap, CorticalIdx, FCLManager


class GPUBitMap:
    """GPU-accelerated implementation of bitmap operations for FCL.

    This class provides a compatible interface with the BitMapProtocol, but
    delegates operations to the GPU backend when available.
    """

    def __init__(self, elements=None):
        self.backend = get_backend()

        # Store elements in GPU memory if available
        if elements is not None:
            # Convert to numpy array first
            if not isinstance(elements, np.ndarray):
                elements_array = np.array(list(elements), dtype=np.uint32)
            else:
                elements_array = elements

            # Convert to bitmap representation (each bit represents a neuron)
            max_neuron_id = (
                elements_array.max() if elements_array.size > 0 else 0
            )
            chunk_count = (max_neuron_id // 32) + 1
            bitmap_array = np.zeros(chunk_count, dtype=np.uint32)

            # Set bits for each neuron
            for neuron_id in elements_array:
                chunk_index = neuron_id // 32
                bit_index = neuron_id % 32
                bitmap_array[chunk_index] |= 1 << bit_index

            # Upload to GPU
            self.gpu_bitmap = self.backend.from_numpy(bitmap_array)
        else:
            # Create empty bitmap
            self.gpu_bitmap = self.backend.create_tensor((1,), dtype=np.uint32)

        # Keep a cache of elements for operations that can't be done on GPU
        self._elements_cache = set(elements) if elements else set()
        self._cache_valid = True

    def add(self, element: int) -> None:
        """Add an element to the bitmap."""
        # Update cache
        if self._cache_valid:
            self._elements_cache.add(element)

        # Get current bitmap as numpy array
        bitmap_array = self.backend.to_numpy(self.gpu_bitmap)

        # Check if we need to resize the bitmap
        chunk_index = element // 32
        if chunk_index >= bitmap_array.shape[0]:
            new_size = chunk_index + 1
            new_bitmap = np.zeros(new_size, dtype=np.uint32)
            new_bitmap[: bitmap_array.shape[0]] = bitmap_array
            bitmap_array = new_bitmap

        # Set the bit
        bit_index = element % 32
        bitmap_array[chunk_index] |= 1 << bit_index

        # Upload back to GPU
        self.gpu_bitmap = self.backend.from_numpy(bitmap_array)

    def clear(self) -> None:
        """Clear the bitmap."""
        # Clear cache
        self._elements_cache.clear()
        self._cache_valid = True

        # Create new empty bitmap
        bitmap_array = np.zeros_like(self.backend.to_numpy(self.gpu_bitmap))
        self.gpu_bitmap = self.backend.from_numpy(bitmap_array)

    def copy(self) -> "GPUBitMap":
        """Create a copy of the bitmap."""
        result = GPUBitMap()
        # Copy GPU data
        result.gpu_bitmap = self.backend.from_numpy(
            self.backend.to_numpy(self.gpu_bitmap)
        )
        # Copy cache if valid
        if self._cache_valid:
            result._elements_cache = self._elements_cache.copy()
            result._cache_valid = True
        else:
            result._cache_valid = False
        return result

    def __or__(self, other: "GPUBitMap") -> "GPUBitMap":
        """Perform bitwise OR operation."""
        if not isinstance(other, GPUBitMap):
            raise TypeError(f"Unsupported operand type: {type(other)}")

        # Make sure both bitmaps have the same size
        left_bitmap = self.backend.to_numpy(self.gpu_bitmap)
        right_bitmap = self.backend.to_numpy(other.gpu_bitmap)

        # Resize if necessary
        if left_bitmap.shape[0] != right_bitmap.shape[0]:
            new_size = max(left_bitmap.shape[0], right_bitmap.shape[0])
            if left_bitmap.shape[0] < new_size:
                new_left = np.zeros(new_size, dtype=np.uint32)
                new_left[: left_bitmap.shape[0]] = left_bitmap
                left_bitmap = new_left
            if right_bitmap.shape[0] < new_size:
                new_right = np.zeros(new_size, dtype=np.uint32)
                new_right[: right_bitmap.shape[0]] = right_bitmap
                right_bitmap = new_right

        # Upload to GPU
        gpu_left = self.backend.from_numpy(left_bitmap)
        gpu_right = self.backend.from_numpy(right_bitmap)

        # Perform OR operation on GPU
        result = GPUBitMap()
        if hasattr(self.backend, "bitmap_or"):
            result.gpu_bitmap = self.backend.bitmap_or(gpu_left, gpu_right)
        else:
            # Fallback to CPU if operation not available
            result_bitmap = left_bitmap | right_bitmap
            result.gpu_bitmap = self.backend.from_numpy(result_bitmap)

        # Update cache if both caches are valid
        if self._cache_valid and other._cache_valid:
            result._elements_cache = (
                self._elements_cache | other._elements_cache
            )
            result._cache_valid = True
        else:
            result._cache_valid = False

        return result

    def __and__(self, other: "GPUBitMap") -> "GPUBitMap":
        """Perform bitwise AND operation."""
        if not isinstance(other, GPUBitMap):
            raise TypeError(f"Unsupported operand type: {type(other)}")

        # Make sure both bitmaps have the same size
        left_bitmap = self.backend.to_numpy(self.gpu_bitmap)
        right_bitmap = self.backend.to_numpy(other.gpu_bitmap)

        # Resize if necessary
        if left_bitmap.shape[0] != right_bitmap.shape[0]:
            new_size = max(left_bitmap.shape[0], right_bitmap.shape[0])
            if left_bitmap.shape[0] < new_size:
                new_left = np.zeros(new_size, dtype=np.uint32)
                new_left[: left_bitmap.shape[0]] = left_bitmap
                left_bitmap = new_left
            if right_bitmap.shape[0] < new_size:
                new_right = np.zeros(new_size, dtype=np.uint32)
                new_right[: right_bitmap.shape[0]] = right_bitmap
                right_bitmap = new_right

        # Upload to GPU
        gpu_left = self.backend.from_numpy(left_bitmap)
        gpu_right = self.backend.from_numpy(right_bitmap)

        # Perform AND operation on GPU
        result = GPUBitMap()
        if hasattr(self.backend, "bitmap_and"):
            result.gpu_bitmap = self.backend.bitmap_and(gpu_left, gpu_right)
        else:
            # Fallback to CPU if operation not available
            result_bitmap = left_bitmap & right_bitmap
            result.gpu_bitmap = self.backend.from_numpy(result_bitmap)

        # Update cache if both caches are valid
        if self._cache_valid and other._cache_valid:
            result._elements_cache = (
                self._elements_cache & other._elements_cache
            )
            result._cache_valid = True
        else:
            result._cache_valid = False

        return result

    def __sub__(self, other: "GPUBitMap") -> "GPUBitMap":
        """Perform bitmap subtraction (remove elements in other from self)."""
        if not isinstance(other, GPUBitMap):
            raise TypeError(f"Unsupported operand type: {type(other)}")

        # Make sure both bitmaps have the same size
        left_bitmap = self.backend.to_numpy(self.gpu_bitmap)
        right_bitmap = self.backend.to_numpy(other.gpu_bitmap)

        # Resize if necessary
        if left_bitmap.shape[0] != right_bitmap.shape[0]:
            new_size = max(left_bitmap.shape[0], right_bitmap.shape[0])
            if left_bitmap.shape[0] < new_size:
                new_left = np.zeros(new_size, dtype=np.uint32)
                new_left[: left_bitmap.shape[0]] = left_bitmap
                left_bitmap = new_left
            if right_bitmap.shape[0] < new_size:
                new_right = np.zeros(new_size, dtype=np.uint32)
                new_right[: right_bitmap.shape[0]] = right_bitmap
                right_bitmap = new_right

        # Upload to GPU
        gpu_left = self.backend.from_numpy(left_bitmap)
        gpu_right = self.backend.from_numpy(right_bitmap)

        # Perform subtraction operation on GPU
        result = GPUBitMap()
        if hasattr(self.backend, "bitmap_subtract"):
            result.gpu_bitmap = self.backend.bitmap_subtract(
                gpu_left, gpu_right
            )
        else:
            # Fallback to CPU if operation not available
            result_bitmap = left_bitmap & ~right_bitmap
            result.gpu_bitmap = self.backend.from_numpy(result_bitmap)

        # Update cache if both caches are valid
        if self._cache_valid and other._cache_valid:
            result._elements_cache = (
                self._elements_cache - other._elements_cache
            )
            result._cache_valid = True
        else:
            result._cache_valid = False

        return result

    def __xor__(self, other: "GPUBitMap") -> "GPUBitMap":
        """Perform bitwise XOR operation."""
        if not isinstance(other, GPUBitMap):
            raise TypeError(f"Unsupported operand type: {type(other)}")

        # Make sure both bitmaps have the same size
        left_bitmap = self.backend.to_numpy(self.gpu_bitmap)
        right_bitmap = self.backend.to_numpy(other.gpu_bitmap)

        # Resize if necessary
        if left_bitmap.shape[0] != right_bitmap.shape[0]:
            new_size = max(left_bitmap.shape[0], right_bitmap.shape[0])
            if left_bitmap.shape[0] < new_size:
                new_left = np.zeros(new_size, dtype=np.uint32)
                new_left[: left_bitmap.shape[0]] = left_bitmap
                left_bitmap = new_left
            if right_bitmap.shape[0] < new_size:
                new_right = np.zeros(new_size, dtype=np.uint32)
                new_right[: right_bitmap.shape[0]] = right_bitmap
                right_bitmap = new_right

        # Upload to GPU
        gpu_left = self.backend.from_numpy(left_bitmap)
        gpu_right = self.backend.from_numpy(right_bitmap)

        # Perform XOR operation on GPU
        result = GPUBitMap()
        if hasattr(self.backend, "bitmap_xor"):
            result.gpu_bitmap = self.backend.bitmap_xor(gpu_left, gpu_right)
        else:
            # Fallback to CPU if operation not available
            result_bitmap = left_bitmap ^ right_bitmap
            result.gpu_bitmap = self.backend.from_numpy(result_bitmap)

        # Update cache if both caches are valid
        if self._cache_valid and other._cache_valid:
            result._elements_cache = (
                self._elements_cache ^ other._elements_cache
            )
            result._cache_valid = True
        else:
            result._cache_valid = False

        return result

    def __len__(self) -> int:
        """Get the number of elements in the bitmap."""
        # If cache is valid, use it
        if self._cache_valid:
            return len(self._elements_cache)

        # Otherwise, compute it from the bitmap
        self._update_cache()
        return len(self._elements_cache)

    def _update_cache(self) -> None:
        """Update the element cache from the GPU bitmap."""
        bitmap_array = self.backend.to_numpy(self.gpu_bitmap)

        # Convert bitmap to set of elements
        elements = set()
        for chunk_index, chunk in enumerate(bitmap_array):
            if chunk == 0:
                continue

            # Extract set bits
            for bit_index in range(32):
                if chunk & (1 << bit_index):
                    neuron_id = chunk_index * 32 + bit_index
                    elements.add(neuron_id)

        self._elements_cache = elements
        self._cache_valid = True

    def __iter__(self):
        """Iterate over elements in the bitmap."""
        if not self._cache_valid:
            self._update_cache()
        return iter(self._elements_cache)

    def __contains__(self, item: int) -> bool:
        """Check if an element is in the bitmap."""
        if self._cache_valid:
            return item in self._elements_cache

        # Calculate position in bitmap
        chunk_index = item // 32
        bit_index = item % 32

        # Get bitmap array
        bitmap_array = self.backend.to_numpy(self.gpu_bitmap)

        # Check if chunk index is valid
        if chunk_index >= bitmap_array.shape[0]:
            return False

        # Check if bit is set
        return bool(bitmap_array[chunk_index] & (1 << bit_index))

    def is_empty(self) -> bool:
        """Check if the bitmap is empty."""
        if self._cache_valid:
            return len(self._elements_cache) == 0

        # Check if any chunk is non-zero
        bitmap_array = self.backend.to_numpy(self.gpu_bitmap)
        return np.all(bitmap_array == 0)

    def to_cpu_bitmap(self) -> BitMap:
        """Convert GPU bitmap to CPU bitmap."""
        if not self._cache_valid:
            self._update_cache()
        return BitMap(self._elements_cache)


def create_gpu_accelerated_fcl(window_size: int = 20):
    """Create a GPU-accelerated FCL manager if a compatible backend is
    available.

    Args:
        window_size: Size of the FCL sliding window

    Returns:
        An FCL manager instance that uses GPU acceleration if available,
        otherwise returns a standard FCL manager.
    """

    # Check if GPU backend is available
    backend = get_backend()
    if backend is None:
        logger.info("No backend available, falling back to CPU FCL manager")
        return FCLManager(default_window_size=window_size)

    # Check if backend supports bitmap operations
    if not hasattr(backend, "bitmap_or") or not backend.supports_capability(
        "bitmap_operations"
    ):
        logger.info(
            f"Backend {getattr(backend, 'name', type(backend).__name__)} does not support bitmap operations, using CPU FCL manager"
        )
        return FCLManager(default_window_size=window_size)

    # Create GPU-accelerated FCL manager
    logger.info(
        f"Using GPU-accelerated FCL manager with {getattr(backend, 'name', type(backend).__name__)} backend"
    )
    return GPUAcceleratedFCL(backend, window_size)


class GPUAcceleratedFCL:
    """GPU-accelerated implementation of Fire Candidate List Manager.

    This class provides the same interface as FCLManager, but uses GPU
    operations for performance-critical bitmap operations.
    """

    def __init__(
        self,
        backend: Optional[BackendInterface],
        default_window_size: int = 20,
    ):
        """Initialize the GPU-accelerated FCL manager.

        Args:
            backend: GPU backend to use for operations
            default_window_size: Default window size for FCL history
        """

        # Get backend if not provided
        self.backend = backend or get_backend()
        if self.backend is None:
            raise RuntimeError("No GPU backend available")

        # Ensure backend supports required bitmap operations
        if not hasattr(
            self.backend, "bitmap_or"
        ) or not self.backend.supports_capability("bitmap_operations"):
            raise TypeError(
                "Backend does not support required bitmap operations for GPUAcceleratedFCL"
            )

        # Create a CPU FCL manager as a delegate for operations that can't be accelerated
        self.cpu_fcl = FCLManager(default_window_size=default_window_size)

        logger.info(
            f"Initialized GPU-accelerated FCL manager with {getattr(self.backend, 'name', type(self.backend).__name__)} backend"
        )

    def __getattr__(self, name):
        """Delegate all non-overridden attributes to the CPU FCL manager.

        This allows us to implement only the methods that benefit from GPU
        acceleration and rely on the CPU implementation for the rest.
        """
        return getattr(self.cpu_fcl, name)

    def update_fcl(
        self,
        current_timestep: int,
        neurons_by_cortical: Dict[
            CorticalIdx, Union[BitMap, List[int], Set[int]]
        ],
    ) -> None:
        """Update the FCL with new firing neurons, accelerated with GPU.

        Args:
            current_timestep: Current simulation timestep
            neurons_by_cortical: Dictionary mapping cortical indices to collections of firing neurons
        """
        # For now, delegate to CPU implementation
        # A fully GPU-accelerated version would require more complex data structures
        # that match the FCL implementation
        self.cpu_fcl.update_fcl(current_timestep, neurons_by_cortical)

    def get_fcl_delta(
        self,
        start_time: int,
        end_time: int,
        cortical_indices: Optional[List[CorticalIdx]] = None,
    ) -> BitMap:
        """Compute the delta (difference) between FCLs at two timesteps,
        accelerated with GPU.

        Args:
            start_time: Starting timestep
            end_time: Ending timestep
            cortical_indices: Optional list of cortical indices to restrict the delta computation

        Returns:
            BitMap containing neurons that fired in end_time but not start_time
        """
        # Get FCLs from CPU implementation
        fcl1 = self.cpu_fcl.get_global_fcl(start_time)
        fcl2 = self.cpu_fcl.get_global_fcl(end_time)

        # Convert to numpy arrays
        if cortical_indices:
            # Filter by cortical areas
            fcl1 = self.cpu_fcl.get_neurons_by_corticals(
                cortical_indices, start_time
            )
            fcl2 = self.cpu_fcl.get_neurons_by_corticals(
                cortical_indices, end_time
            )

        # Convert to GPU bitmaps
        gpu_fcl1 = GPUBitMap(fcl1)
        gpu_fcl2 = GPUBitMap(fcl2)

        # Perform delta computation on GPU
        result = gpu_fcl2 - gpu_fcl1

        # Convert back to CPU bitmap
        return result.to_cpu_bitmap()

    def get_consistently_active_neurons(
        self,
        n_steps: int,
        cortical_indices: Optional[List[CorticalIdx]] = None,
    ) -> BitMap:
        """Get neurons that have been consistently active over the last n
        steps, accelerated with GPU.

        Args:
            n_steps: Number of timesteps to check
            cortical_indices: Optional list of cortical indices to restrict the check

        Returns:
            BitMap containing neurons that were active in all n_steps
        """
        # Get current timestep
        current_timestep = self.cpu_fcl.current_timestep

        # Initialize with the first FCL
        first_fcl = self.cpu_fcl.get_global_fcl(current_timestep)
        if cortical_indices:
            first_fcl = self.cpu_fcl.get_neurons_by_corticals(
                cortical_indices, current_timestep
            )

        # Convert to GPU bitmap
        result = GPUBitMap(first_fcl)

        # Intersect with FCLs from previous timesteps
        for offset in range(1, n_steps):
            timestep = current_timestep - offset

            # Skip if timestep is out of range
            if timestep < 0:
                continue

            # Get FCL for this timestep
            fcl = self.cpu_fcl.get_global_fcl(timestep)
            if cortical_indices:
                fcl = self.cpu_fcl.get_neurons_by_corticals(
                    cortical_indices, timestep
                )

            # Convert to GPU bitmap and intersect
            gpu_fcl = GPUBitMap(fcl)
            result = result & gpu_fcl

            # If result is empty, we can stop early
            if result.is_empty():
                break

        # Convert back to CPU bitmap
        return result.to_cpu_bitmap()

    def get_neurons_fired_in_last_n_steps(
        self,
        n_steps: int,
        cortical_indices: Optional[List[CorticalIdx]] = None,
    ) -> BitMap:
        """Get neurons that fired in any of the last n steps, accelerated with
        GPU.

        Args:
            n_steps: Number of timesteps to check
            cortical_indices: Optional list of cortical indices to restrict the check

        Returns:
            BitMap containing neurons that fired in any of the last n_steps
        """
        # Get current timestep
        current_timestep = self.cpu_fcl.current_timestep

        # Initialize with the first FCL
        first_fcl = self.cpu_fcl.get_global_fcl(current_timestep)
        if cortical_indices:
            first_fcl = self.cpu_fcl.get_neurons_by_corticals(
                cortical_indices, current_timestep
            )

        # Convert to GPU bitmap
        result = GPUBitMap(first_fcl)

        # Union with FCLs from previous timesteps
        for offset in range(1, n_steps):
            timestep = current_timestep - offset

            # Skip if timestep is out of range
            if timestep < 0:
                continue

            # Get FCL for this timestep
            fcl = self.cpu_fcl.get_global_fcl(timestep)
            if cortical_indices:
                fcl = self.cpu_fcl.get_neurons_by_corticals(
                    cortical_indices, timestep
                )

            # Convert to GPU bitmap and union
            gpu_fcl = GPUBitMap(fcl)
            result = result | gpu_fcl

        # Convert back to CPU bitmap
        return result.to_cpu_bitmap()
