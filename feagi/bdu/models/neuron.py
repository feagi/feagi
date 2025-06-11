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

"""Neuron model implementation optimized for embedded single-core and GPU processing.

This module provides a high-performance implementation of neuron storage using 
Structure of Arrays (SoA) format optimized for:
- Embedded single-core operation (10M neurons at 15Hz)
- GPU acceleration (CUDA, Metal, WebGPU)
- SIMD vectorization (AVX-512, AVX2, NEON)
- Memory efficiency and cache locality
- Zero-allocation operation paths
"""

import numpy as np
from typing import Dict, List, Tuple, Any, Optional, Union, Set
import torch
import logging
import threading
import os
import ctypes
from feagi.bdu.models.array_backend import ArrayBackend, BackendType

# Try to import optimized libraries
try:
    import numba
    from numba import njit, prange
    NUMBA_AVAILABLE = True
except ImportError:
    NUMBA_AVAILABLE = False
    def njit(*args, **kwargs):
        def decorator(func):
            return func
        return decorator
    def prange(x):
        return range(x)

try:
    from feagi_rust import create_gna
    RUST_AVAILABLE = True
except (ImportError, AttributeError):
    RUST_AVAILABLE = False

logger = logging.getLogger(__name__)

# Cache-friendly configuration
CACHE_LINE_SIZE = 64  # Standard cache line size
MEMORY_ALIGNMENT = 64  # 64-byte alignment for AVX-512
BLOCK_SIZE = 64       # Block size for block-sparse operations

# SIMD configuration detection
try:
    from feagi.utils.simd_detection import get_simd_detector
    simd_detector = get_simd_detector()
    MEMORY_ALIGNMENT = simd_detector.get_memory_alignment()
    VECTOR_WIDTH = simd_detector.capabilities.vector_width
    HAS_AVX512 = simd_detector.capabilities.avx512f
    HAS_AVX2 = simd_detector.capabilities.avx2
    HAS_NEON = simd_detector.capabilities.neon
except ImportError:
    VECTOR_WIDTH = 16  # Default for AVX-512
    HAS_AVX512 = False
    HAS_AVX2 = False
    HAS_NEON = False

class CacheAlignedArray:
    """Cache-aligned array for optimal SIMD performance."""
    
    def __init__(self, size: int, dtype: np.dtype, alignment: int = MEMORY_ALIGNMENT):
        """Create cache-aligned array.
        
        Args:
            size: Number of elements
            dtype: Data type
            alignment: Memory alignment in bytes
        """
        self.size = size
        self.dtype = np.dtype(dtype)  # Ensure it's a proper numpy dtype
        self.alignment = alignment
        
        # Calculate aligned size with safety checks
        element_size = self.dtype.itemsize
        if element_size <= 0:
            raise ValueError(f"Invalid element size: {element_size}")
        if size <= 0:
            raise ValueError(f"Invalid array size: {size}")
        if alignment <= 0 or (alignment & (alignment - 1)) != 0:
            raise ValueError(f"Alignment must be a power of 2, got: {alignment}")
            
        total_bytes = size * element_size
        aligned_bytes = (total_bytes + alignment - 1) & ~(alignment - 1)
        
        # Allocate oversized array with extra safety margin
        safety_margin = alignment * 2  # Extra safety margin
        oversized_elements = (aligned_bytes + element_size - 1) // element_size + alignment + safety_margin
        self._oversized = np.zeros(oversized_elements, dtype=dtype)
        
        # Find aligned start position
        data_ptr = self._oversized.ctypes.data
        offset_bytes = alignment - (data_ptr % alignment)
        if offset_bytes == alignment:
            offset_bytes = 0
        
        offset_elements = offset_bytes // element_size
        end_element = offset_elements + size
        
        # Ensure we don't go out of bounds
        if end_element > len(self._oversized):
            raise RuntimeError(f"Array alignment calculation error: end_element={end_element}, oversized_length={len(self._oversized)}")
            
        # CRITICAL FIX: Keep a strong reference to the underlying array to prevent garbage collection
        # This prevents use-after-free memory corruption when the _oversized array is GC'd
        self._underlying_array = self._oversized  # Strong reference to prevent GC
        self.array = self._oversized[offset_elements:end_element]
        
        # Verify alignment
        assert self.array.ctypes.data % alignment == 0, f"Failed to achieve {alignment}-byte alignment"
    
    def __getitem__(self, key):
        return self.array[key]
    
    def __setitem__(self, key, value):
        self.array[key] = value
    
    def __del__(self):
        """Explicit cleanup to prevent memory leaks."""
        # MEMORY SAFETY: Explicit cleanup to prevent resource leaks
        if hasattr(self, '_underlying_array'):
            del self._underlying_array
        if hasattr(self, '_oversized'):
            del self._oversized

class BlockSparseMatrix:
    """Block-sparse matrix for cache-friendly connectivity storage."""
    
    def __init__(self, shape: Tuple[int, int], block_size: int = BLOCK_SIZE):
        """Initialize block-sparse matrix.
        
        Args:
            shape: Matrix dimensions (rows, cols)
            block_size: Size of each block (must be power of 2)
        """
        self.shape = shape
        self.block_size = block_size
        self.rows, self.cols = shape
        
        # Calculate block grid dimensions
        self.block_rows = (self.rows + block_size - 1) // block_size
        self.block_cols = (self.cols + block_size - 1) // block_size
        
        # Active block tracking
        self.active_blocks: Dict[Tuple[int, int], np.ndarray] = {}
        self.block_map = np.zeros((self.block_rows, self.block_cols), dtype=np.bool_)
    
    def get_block_coords(self, row: int, col: int) -> Tuple[int, int, int, int]:
        """Get block coordinates for a matrix position."""
        block_row = row // self.block_size
        block_col = col // self.block_size
        in_block_row = row % self.block_size
        in_block_col = col % self.block_size
        return block_row, block_col, in_block_row, in_block_col
    
    def __setitem__(self, key: Tuple[int, int], value: float):
        """Set matrix element."""
        row, col = key
        block_row, block_col, in_block_row, in_block_col = self.get_block_coords(row, col)
        
        # Create block if it doesn't exist and value is non-zero
        if value != 0.0:
            if (block_row, block_col) not in self.active_blocks:
                self.active_blocks[(block_row, block_col)] = np.zeros(
                    (self.block_size, self.block_size), dtype=np.float32
                )
                self.block_map[block_row, block_col] = True
            
            self.active_blocks[(block_row, block_col)][in_block_row, in_block_col] = value
        else:
            # Remove zero values
            if (block_row, block_col) in self.active_blocks:
                self.active_blocks[(block_row, block_col)][in_block_row, in_block_col] = 0.0
                
                # Remove block if it becomes all zeros
                if not np.any(self.active_blocks[(block_row, block_col)]):
                    del self.active_blocks[(block_row, block_col)]
                    self.block_map[block_row, block_col] = False
    
    def __getitem__(self, key: Tuple[int, int]) -> float:
        """Get matrix element."""
        row, col = key
        block_row, block_col, in_block_row, in_block_col = self.get_block_coords(row, col)
        
        if (block_row, block_col) in self.active_blocks:
            return float(self.active_blocks[(block_row, block_col)][in_block_row, in_block_col])
        return 0.0
    
    def get_active_connections(self, source_indices: np.ndarray) -> List[Tuple[np.ndarray, np.ndarray]]:
        """Get all active connections from source indices efficiently."""
        connections = []
        
        for source_idx in source_indices:
            block_row, _, _, _ = self.get_block_coords(source_idx, 0)
            
            # Process all active blocks in this row
            for (br, bc), block in self.active_blocks.items():
                if br == block_row:
                    # Find non-zero connections in this block
                    in_block_row = source_idx % self.block_size
                    nonzero_cols = np.nonzero(block[in_block_row, :])[0]
                    
                    if len(nonzero_cols) > 0:
                        # Convert to global indices with bounds checking
                        global_targets = bc * self.block_size + nonzero_cols
                        # Filter targets that are within matrix bounds
                        valid_targets_mask = global_targets < self.cols
                        if np.any(valid_targets_mask):
                            valid_targets = global_targets[valid_targets_mask]
                            valid_weights = block[in_block_row, nonzero_cols][valid_targets_mask]
                            connections.append((valid_targets, valid_weights))
        
        return connections

# Optimized SIMD functions
if NUMBA_AVAILABLE:
    @njit(parallel=True, fastmath=True)
    def simd_membrane_decay(potentials: np.ndarray, decay_rates: np.ndarray, 
                           valid_mask: np.ndarray) -> None:
        # CRITICAL FIX: Ensure all arrays have compatible sizes before broadcast operations
        min_size = min(len(potentials), len(decay_rates), len(valid_mask))
        if min_size == 0:
            return
        
        # CRITICAL FIX: Create bounds-safe mask instead of slicing arrays (which creates unstable views)
        # Only apply to neurons within the safe bounds of all arrays
        safe_bounds_mask = np.arange(len(valid_mask)) < min_size
        combined_mask = valid_mask & safe_bounds_mask
        
        # Apply decay only to valid neurons within safe bounds - no array slicing
        potentials[combined_mask] *= decay_rates[combined_mask]
    
    @njit(parallel=True, fastmath=True)
    def simd_refractory_update(refractory_counters: np.ndarray, 
                              valid_mask: np.ndarray) -> None:
        """SIMD-optimized refractory period updates."""
        n = refractory_counters.shape[0]
        for i in prange(n):
            if valid_mask[i] and refractory_counters[i] > 0:
                refractory_counters[i] -= 1
    
    @njit(parallel=True, fastmath=True)
    def simd_threshold_check(potentials: np.ndarray, thresholds: np.ndarray, 
                            refractory_counters: np.ndarray, valid_mask: np.ndarray,
                            fired_mask: np.ndarray) -> None:
        """SIMD-optimized threshold checking and firing."""
        n = potentials.shape[0]
        for i in prange(n):
            if valid_mask[i] and refractory_counters[i] == 0:
                fired_mask[i] = potentials[i] >= thresholds[i]
            else:
                fired_mask[i] = False
    
    @njit(parallel=True, fastmath=True)
    def simd_fire_neurons(potentials: np.ndarray, resting_potentials: np.ndarray,
                         refractory_counters: np.ndarray, refractory_periods: np.ndarray,
                         fired_mask: np.ndarray) -> None:
        """SIMD-optimized neuron firing logic."""
        n = potentials.shape[0]
        for i in prange(n):
            if fired_mask[i]:
                potentials[i] = resting_potentials[i]
                refractory_counters[i] = refractory_periods[i]
    
    @njit(parallel=True, fastmath=True)
    def simd_synaptic_integration(targets: np.ndarray, weights: np.ndarray, 
                                 potentials: np.ndarray) -> None:
        """SIMD-optimized synaptic integration with bounds checking."""
        n = targets.shape[0]
        max_target = potentials.shape[0]
        for i in prange(n):
            target_idx = targets[i]
            if 0 <= target_idx < max_target:  # Critical bounds check
                potentials[target_idx] += weights[i]

else:
    # Fallback implementations
    def simd_membrane_decay(potentials: np.ndarray, decay_rates: np.ndarray, 
                           valid_mask: np.ndarray) -> None:
        # CRITICAL FIX: Ensure all arrays have compatible sizes before broadcast operations
        min_size = min(len(potentials), len(decay_rates), len(valid_mask))
        if min_size == 0:
            return
        
        # CRITICAL FIX: Create bounds-safe mask instead of slicing arrays (which creates unstable views)
        # Only apply to neurons within the safe bounds of all arrays
        safe_bounds_mask = np.arange(len(valid_mask)) < min_size
        combined_mask = valid_mask & safe_bounds_mask
        
        # Apply decay only to valid neurons within safe bounds - no array slicing
        potentials[combined_mask] *= decay_rates[combined_mask]
    
    def simd_refractory_update(refractory_counters: np.ndarray, 
                              valid_mask: np.ndarray) -> None:
        mask = valid_mask & (refractory_counters > 0)
        refractory_counters[mask] -= 1
    
    def simd_threshold_check(potentials: np.ndarray, thresholds: np.ndarray, 
                            refractory_counters: np.ndarray, valid_mask: np.ndarray,
                            fired_mask: np.ndarray) -> None:
        # CRITICAL FIX: Ensure all arrays have compatible sizes
        min_size = min(len(potentials), len(thresholds), len(refractory_counters), len(valid_mask), len(fired_mask))
        if min_size == 0:
            fired_mask[:] = False
            return
        
        # CRITICAL FIX: Use bounds-safe mask instead of array slicing
        safe_bounds_mask = np.arange(len(valid_mask)) < min_size
        safe_valid_mask = valid_mask & safe_bounds_mask
        can_fire = safe_valid_mask & (refractory_counters[:min_size] == 0)
        
        fired_mask[:] = False
        fired_mask[can_fire] = potentials[can_fire] >= thresholds[can_fire]
    
    def simd_fire_neurons(potentials: np.ndarray, resting_potentials: np.ndarray,
                         refractory_counters: np.ndarray, refractory_periods: np.ndarray,
                         fired_mask: np.ndarray) -> None:
        # CRITICAL FIX: Ensure all arrays have compatible sizes
        min_size = min(len(potentials), len(resting_potentials), len(refractory_counters), 
                      len(refractory_periods), len(fired_mask))
        if min_size == 0:
            return
        
        # CRITICAL FIX: Use bounds-safe mask instead of array slicing
        safe_bounds_mask = np.arange(len(fired_mask)) < min_size
        safe_fired_mask = fired_mask & safe_bounds_mask
        
        potentials[safe_fired_mask] = resting_potentials[safe_fired_mask]
        refractory_counters[safe_fired_mask] = refractory_periods[safe_fired_mask]
    
    def simd_synaptic_integration(targets: np.ndarray, weights: np.ndarray, 
                                 potentials: np.ndarray) -> None:
        # Filter valid targets to prevent buffer overflow
        valid_mask = (targets >= 0) & (targets < len(potentials))
        valid_targets = targets[valid_mask]
        valid_weights = weights[valid_mask]
        np.add.at(potentials, valid_targets, valid_weights)

class NeuronArray:
    """Ultra-high-performance neuron storage optimized for embedded and GPU processing.
    
    This implementation uses Structure of Arrays (SoA) with:
    - 64-byte cache-aligned memory for SIMD optimization
    - Block-sparse connectivity matrices for cache locality
    - SIMD-vectorized neural operations (Numba JIT compiled)
    - Memory pools for zero-allocation operation
    - Both embedded single-core and GPU optimization
    
    Performance target: 10M neuron operations at 15Hz on single-core embedded systems.
    """
    
    def __init__(self, max_neurons: int = 10_000_000, backend: Optional[str] = None):
        """Initialize ultra-high-performance NeuronArray.
        
        Args:
            max_neurons: Maximum number of neurons to support
            backend: Backend type to use (numpy, pytorch, cupy, webgpu, rust, or auto)
        """
        self.max_neurons = max_neurons
        
        # Align capacity to SIMD vector boundaries
        self.aligned_capacity = (max_neurons + VECTOR_WIDTH - 1) & ~(VECTOR_WIDTH - 1)
        
        # Initialize Rust backend if available and requested
        if backend == "rust" or (backend is None and RUST_AVAILABLE):
            try:
                self._rust_backend = create_gna(self.aligned_capacity)
                self._use_rust = True
                self.backend_type = "rust"
                logger.info(f"Initialized NeuronArray with Rust backend, capacity: {self.aligned_capacity}")
            except Exception as e:
                logger.warning(f"Failed to initialize Rust backend: {e}, falling back to NumPy")
                self._use_rust = False
                self._init_optimized_backend(backend)
        else:
            self._use_rust = False
            self._init_optimized_backend(backend)
        
        # Common tracking regardless of backend
        self.id_to_index_map: Dict[int, int] = {}
        self.index_to_id_map: Dict[int, int] = {}
        self.cortical_id_to_indices: Dict[int, List[int]] = {}
        self.next_index = 0
        self.free_indices: Set[int] = set()
        self.neuron_count = 0
        
        # Unique neuron ID generator - separate from array indices to prevent corruption
        self._next_neuron_id = 1  # Start from 1 (0 reserved for invalid)
        
        # Performance tracking
        self.operation_count = 0
        self.total_operation_time = 0.0
    
    def _init_optimized_backend(self, backend: Optional[str]):
        """Initialize optimized backend with cache-aligned arrays."""
        # Set backend
        self.backend = ArrayBackend(backend)
        self.backend_type = self.backend.backend_type
        
        # CRITICAL FIX: Create cache-aligned arrays and keep strong references to prevent GC
        # Store both the CacheAlignedArray objects AND their .array views to prevent premature GC
        self._aligned_membrane_potentials = CacheAlignedArray(self.aligned_capacity, np.float32)
        self.membrane_potentials = self._aligned_membrane_potentials.array
        
        self._aligned_resting_potentials = CacheAlignedArray(self.aligned_capacity, np.float32)
        self.resting_potentials = self._aligned_resting_potentials.array
        
        self._aligned_thresholds = CacheAlignedArray(self.aligned_capacity, np.float32)
        self.thresholds = self._aligned_thresholds.array
        
        self._aligned_decay_rates = CacheAlignedArray(self.aligned_capacity, np.float32)
        self.decay_rates = self._aligned_decay_rates.array
        
        self._aligned_refractory_periods = CacheAlignedArray(self.aligned_capacity, np.int32)
        self.refractory_periods = self._aligned_refractory_periods.array
        
        self._aligned_refractory_counters = CacheAlignedArray(self.aligned_capacity, np.int32)
        self.refractory_counters = self._aligned_refractory_counters.array
        
        # Coordinate arrays (cache-aligned SoA)
        self._aligned_coordinates_x = CacheAlignedArray(self.aligned_capacity, np.uint32)
        self.coordinates_x = self._aligned_coordinates_x.array
        
        self._aligned_coordinates_y = CacheAlignedArray(self.aligned_capacity, np.uint32)
        self.coordinates_y = self._aligned_coordinates_y.array
        
        self._aligned_coordinates_z = CacheAlignedArray(self.aligned_capacity, np.uint32)
        self.coordinates_z = self._aligned_coordinates_z.array
        
        # Area mapping and activation
        self._aligned_cortical_idxs = CacheAlignedArray(self.aligned_capacity, np.int32)
        self.cortical_idxs = self._aligned_cortical_idxs.array
        
        self._aligned_is_active = CacheAlignedArray(self.aligned_capacity, np.bool_)
        self.is_active = self._aligned_is_active.array
        
        self._aligned_valid_mask = CacheAlignedArray(self.aligned_capacity, np.bool_)
        self.valid_mask = self._aligned_valid_mask.array
        
        # Additional optimization arrays
        self._aligned_last_fired = CacheAlignedArray(self.aligned_capacity, np.int32)
        self.last_fired = self._aligned_last_fired.array
        
        self._aligned_neuron_types = CacheAlignedArray(self.aligned_capacity, np.int32)
        self.neuron_types = self._aligned_neuron_types.array
        
        self._aligned_enabled_flags = CacheAlignedArray(self.aligned_capacity, np.int32)
        self.enabled_flags = self._aligned_enabled_flags.array
        
        # Working arrays for SIMD operations (pre-allocated)
        self._aligned_temp_fired_mask = CacheAlignedArray(self.aligned_capacity, np.bool_)
        self._temp_fired_mask = self._aligned_temp_fired_mask.array
        
        self._aligned_temp_targets = CacheAlignedArray(self.aligned_capacity * 10, np.int32)
        self._temp_targets = self._aligned_temp_targets.array  # Pool for targets
        
        self._aligned_temp_weights = CacheAlignedArray(self.aligned_capacity * 10, np.float32)
        self._temp_weights = self._aligned_temp_weights.array  # Pool for weights
        
        # Initialize default values
        self.thresholds.fill(1.0)
        self.decay_rates.fill(0.95)
        self.refractory_periods.fill(1)
        self.enabled_flags.fill(1)
        
        # Device tracking for GPU support
        self.device = getattr(self.backend, 'device', 'cpu')
        
        logger.info(f"Initialized optimized NeuronArray: {self.backend_type} backend, "
                   f"capacity: {self.aligned_capacity}, alignment: {MEMORY_ALIGNMENT}B, "
                   f"SIMD: {VECTOR_WIDTH}-wide, Numba: {NUMBA_AVAILABLE}")

    def get_coordinates(self, neuron_id: int) -> Tuple[int, int, int]:
        """Get 3D coordinates for a neuron.
        
        Args:
            neuron_id: ID of the neuron
            
        Returns:
            Tuple of (x, y, z) coordinates
        """
        if self._use_rust:
            return self._rust_backend.get_coordinates(neuron_id)
        else:
            if neuron_id not in self.id_to_index_map:
                raise ValueError(f"Neuron ID {neuron_id} not found")
            
            index = self.id_to_index_map[neuron_id]
            return (
                int(self.backend.to_numpy(self.coordinates_x)[index]),
                int(self.backend.to_numpy(self.coordinates_y)[index]),
                int(self.backend.to_numpy(self.coordinates_z)[index])
            )
    
    def set_coordinates(self, neuron_id: int, x: int, y: int, z: int) -> None:
        """Set 3D coordinates for a neuron.
        
        Args:
            neuron_id: ID of the neuron
            x: X coordinate (will be converted to uint32)
            y: Y coordinate (will be converted to uint32) 
            z: Z coordinate (will be converted to uint32)
        """
        if self._use_rust:
            self._rust_backend.set_coordinates(neuron_id, x, y, z)
        else:
            if neuron_id not in self.id_to_index_map:
                raise ValueError(f"Neuron ID {neuron_id} not found")
            
            index = self.id_to_index_map[neuron_id]
            
            # Convert to numpy for indexing, ensuring uint32 type
            coords_x = self.backend.to_numpy(self.coordinates_x)
            coords_y = self.backend.to_numpy(self.coordinates_y)
            coords_z = self.backend.to_numpy(self.coordinates_z)
            
            # Ensure non-negative and convert to uint32
            coords_x[index] = np.uint32(max(0, x))
            coords_y[index] = np.uint32(max(0, y))
            coords_z[index] = np.uint32(max(0, z))
            
            # Convert back to backend arrays, preserving uint32 type
            self.coordinates_x = self.backend.array(coords_x.astype(np.uint32))
            self.coordinates_y = self.backend.array(coords_y.astype(np.uint32))
            self.coordinates_z = self.backend.array(coords_z.astype(np.uint32))

    def batch_update_coordinates(self, neuron_ids: List[int], 
                               coordinates: List[Tuple[int, int, int]]) -> None:
        """Batch update coordinates for multiple neurons - SIMD optimized.
        
        Args:
            neuron_ids: List of neuron IDs
            coordinates: List of (x, y, z) coordinate tuples
        """
        if len(neuron_ids) != len(coordinates):
            raise ValueError("neuron_ids and coordinates must have same length")
        
        if self._use_rust:
            for neuron_id, (x, y, z) in zip(neuron_ids, coordinates):
                self._rust_backend.set_coordinates(neuron_id, x, y, z)
        else:
            # Convert to indices
            indices = [self.id_to_index_map[nid] for nid in neuron_ids if nid in self.id_to_index_map]
            if not indices:
                return
            
            # Vectorized batch update
            coords_x = self.backend.to_numpy(self.coordinates_x)
            coords_y = self.backend.to_numpy(self.coordinates_y)
            coords_z = self.backend.to_numpy(self.coordinates_z)
            
            # Extract coordinates into separate arrays for vectorized assignment
            x_vals = np.array([max(0, coord[0]) for coord in coordinates], dtype=np.uint32)
            y_vals = np.array([max(0, coord[1]) for coord in coordinates], dtype=np.uint32)
            z_vals = np.array([max(0, coord[2]) for coord in coordinates], dtype=np.uint32)
            
            # Vectorized assignment - SIMD friendly, preserving uint32 type
            coords_x[indices] = x_vals
            coords_y[indices] = y_vals
            coords_z[indices] = z_vals
            
            # Update backend arrays, ensuring uint32 type is preserved
            self.coordinates_x = self.backend.array(coords_x.astype(np.uint32))
            self.coordinates_y = self.backend.array(coords_y.astype(np.uint32))
            self.coordinates_z = self.backend.array(coords_z.astype(np.uint32))

    def simd_optimized_update_membrane_potentials(self, decay_factor: float) -> None:
        """SIMD-optimized membrane potential update with vectorization.
        
        Args:
            decay_factor: Global decay factor to apply
        """
        if self._use_rust:
            self._rust_backend.decay_membrane_potentials(decay_factor)
        else:
            # Use SIMD-friendly vectorized operations
            # Process in chunks for optimal cache utilization
            chunk_size = max(VECTOR_WIDTH * 8, 64)  # Process 8 SIMD vectors at a time
            
            potentials = self.backend.to_numpy(self.membrane_potentials)
            
            for i in range(0, self.aligned_capacity, chunk_size):
                end_idx = min(i + chunk_size, self.aligned_capacity)
                # Vectorized decay operation on chunk - SIMD optimized
                potentials[i:end_idx] *= decay_factor
            
            self.membrane_potentials = self.backend.array(potentials)

    def simd_optimized_find_fire_candidates(self, timestep: int) -> List[int]:
        """SIMD-optimized fire candidate detection with vectorization.
        
        Args:
            timestep: Current simulation timestep
            
        Returns:
            List of neuron IDs ready to fire
        """
        if self._use_rust:
            return self._rust_backend.find_fire_candidates(timestep)
        else:
            # Vectorized comparison operations - SIMD friendly
            potentials = self.backend.to_numpy(self.membrane_potentials)
            thresholds = self.backend.to_numpy(self.thresholds)
            refractory = self.backend.to_numpy(self.refractory_counters)
            valid = self.backend.to_numpy(self.valid_mask)
            enabled = self.backend.to_numpy(self.enabled_flags)
            
            # SIMD-optimized boolean mask operations
            fire_mask = (potentials >= thresholds) & (refractory == 0) & valid & (enabled > 0)
            
            # Get indices of firing neurons
            fire_indices = np.where(fire_mask)[0]
            
            # Convert indices to neuron IDs
            return [self.index_to_id_map[idx] for idx in fire_indices if idx in self.index_to_id_map]

    def embedded_optimized_neural_update(self, timestep: int, connectivity_matrix=None) -> List[int]:
        """Ultra-optimized neural update for embedded single-core operation.
        
        Processes complete neural pipeline: decay, refractory, threshold, firing.
        Optimized for 10M neurons at 15Hz on single-core embedded systems.
        
        Args:
            timestep: Current simulation timestep
            connectivity_matrix: Optional connectivity matrix for synaptic integration
            
        Returns:
            List of neuron IDs that fired
        """
        if self._use_rust:
            return self._rust_backend.embedded_neural_update(timestep, connectivity_matrix)
        
        import time
        start_time = time.perf_counter()
        
        # PHASE 1: Membrane potential decay (SIMD-optimized)
        simd_membrane_decay(self.membrane_potentials, self.decay_rates, self.valid_mask)
        
        # PHASE 2: Refractory period updates (SIMD-optimized)
        simd_refractory_update(self.refractory_counters, self.valid_mask)
        
        # PHASE 3: Threshold checking and firing decision (SIMD-optimized)
        simd_threshold_check(
            self.membrane_potentials, self.thresholds,
            self.refractory_counters, self.valid_mask,
            self._temp_fired_mask
        )
        
        # PHASE 4: Fire neurons and reset states (SIMD-optimized)
        simd_fire_neurons(
            self.membrane_potentials, self.resting_potentials,
            self.refractory_counters, self.refractory_periods,
            self._temp_fired_mask
        )
        
        # PHASE 5: Extract fired neuron IDs (minimal allocation)
        fired_indices = np.where(self._temp_fired_mask)[0]
        fired_neurons = []
        for idx in fired_indices:
            if idx in self.index_to_id_map:
                fired_neurons.append(self.index_to_id_map[idx])
        
        # Update performance tracking
        self.operation_count += 1
        operation_time = time.perf_counter() - start_time
        self.total_operation_time += operation_time
        
        return fired_neurons
    
    def embedded_synaptic_integration(self, fired_neurons: List[int], 
                                    connectivity_matrix) -> None:
        """Ultra-optimized synaptic signal integration.
        
        Args:
            fired_neurons: List of neuron IDs that fired
            connectivity_matrix: Sparse connectivity matrix or BlockSparseMatrix
        """
        if not fired_neurons:
            return
        
        # Convert neuron IDs to indices
        fired_indices = np.array([self.id_to_index_map[nid] for nid in fired_neurons 
                                 if nid in self.id_to_index_map], dtype=np.int32)
        
        if len(fired_indices) == 0:
            return
        
        # Handle different connectivity matrix types
        if isinstance(connectivity_matrix, BlockSparseMatrix):
            # Optimized block-sparse processing
            connections = connectivity_matrix.get_active_connections(fired_indices)
            
            target_count = 0
            for targets, weights in connections:
                # Batch synaptic integration using pre-allocated arrays
                batch_size = len(targets)
                if target_count + batch_size <= len(self._temp_targets):
                    self._temp_targets[target_count:target_count + batch_size] = targets
                    self._temp_weights[target_count:target_count + batch_size] = weights
                    target_count += batch_size
            
            if target_count > 0:
                simd_synaptic_integration(
                    self._temp_targets[:target_count],
                    self._temp_weights[:target_count],
                    self.membrane_potentials
                )
        else:
            # Standard sparse matrix processing
            for fired_idx in fired_indices:
                if hasattr(connectivity_matrix, 'getrow'):
                    # CSR matrix
                    row = connectivity_matrix.getrow(fired_idx)
                    targets = row.indices
                    weights = row.data
                elif hasattr(connectivity_matrix, '__getitem__'):
                    # Dense or other indexable matrix
                    row_data = connectivity_matrix[fired_idx, :]
                    if hasattr(row_data, 'nonzero'):
                        targets = row_data.nonzero()[1]
                        weights = row_data[0, targets].A1
                    else:
                        continue
                else:
                    continue
                
                if len(targets) > 0:
                    # Direct integration for smaller batches
                    self.membrane_potentials[targets] += weights
    
    def get_performance_summary(self) -> Dict[str, Any]:
        """Get performance summary for embedded optimization."""
        if self.operation_count == 0:
            return {"avg_operation_time_ms": 0.0, "total_operations": 0}
        
        avg_time_ms = (self.total_operation_time / self.operation_count) * 1000
        
        return {
            "avg_operation_time_ms": avg_time_ms,
            "total_operations": self.operation_count,
            "total_time_s": self.total_operation_time,
            "backend": self.backend_type,
            "simd_enabled": NUMBA_AVAILABLE,
            "alignment": MEMORY_ALIGNMENT,
            "vector_width": VECTOR_WIDTH,
            "capacity": self.aligned_capacity,
            "neuron_count": self.neuron_count
        }

    def to_gpu(self):
        """Transfer neuron arrays to GPU for accelerated computation."""
        if self._use_rust:
            logger.info("Rust backend handles GPU operations internally")
            return True
            
        if self.device == "gpu":
            logger.info("Arrays are already on GPU")
            return True
            
        logger.info("Transferring neuron arrays to GPU...")
        
        try:
            # Move all arrays to GPU using the backend's to_device method
            self.membrane_potentials = self.backend.to_device(self.membrane_potentials)
            self.resting_potentials = self.backend.to_device(self.resting_potentials)
            self.thresholds = self.backend.to_device(self.thresholds)
            self.decay_rates = self.backend.to_device(self.decay_rates)
            self.refractory_periods = self.backend.to_device(self.refractory_periods)
            self.refractory_counters = self.backend.to_device(self.refractory_counters)
            self.is_active = self.backend.to_device(self.is_active)
            self.valid_mask = self.backend.to_device(self.valid_mask)
            self.cortical_idxs = self.backend.to_device(self.cortical_idxs)
            self.coordinates_x = self.backend.to_device(self.coordinates_x)
            self.coordinates_y = self.backend.to_device(self.coordinates_y)
            self.coordinates_z = self.backend.to_device(self.coordinates_z)
            self.last_fired = self.backend.to_device(self.last_fired)
            self.neuron_types = self.backend.to_device(self.neuron_types)
            self.enabled_flags = self.backend.to_device(self.enabled_flags)
            
            self.device = "gpu"
            logger.info("Neuron arrays successfully transferred to GPU")
            return True
        except Exception as e:
            logger.error(f"Failed to transfer arrays to GPU: {e}")
            return False

    def to_cpu(self):
        """Transfer neuron arrays back to CPU."""
        if self.device == "cpu":
            logger.info("Arrays are already on CPU")
            return True
            
        logger.info("Transferring neuron arrays to CPU...")
        
        try:
            # Move all arrays to CPU using the backend's to_cpu method
            self.membrane_potentials = self.backend.to_cpu(self.membrane_potentials)
            self.resting_potentials = self.backend.to_cpu(self.resting_potentials)
            self.thresholds = self.backend.to_cpu(self.thresholds)
            self.decay_rates = self.backend.to_cpu(self.decay_rates)
            self.refractory_periods = self.backend.to_cpu(self.refractory_periods)
            self.refractory_counters = self.backend.to_cpu(self.refractory_counters)
            self.is_active = self.backend.to_cpu(self.is_active)
            self.valid_mask = self.backend.to_cpu(self.valid_mask)
            self.cortical_idxs = self.backend.to_cpu(self.cortical_idxs)
            self.coordinates_x = self.backend.to_cpu(self.coordinates_x)
            self.coordinates_y = self.backend.to_cpu(self.coordinates_y)
            self.coordinates_z = self.backend.to_cpu(self.coordinates_z)
            self.last_fired = self.backend.to_cpu(self.last_fired)
            self.neuron_types = self.backend.to_cpu(self.neuron_types)
            self.enabled_flags = self.backend.to_cpu(self.enabled_flags)
            
            self.device = "cpu"
            logger.info("Neuron arrays successfully transferred to CPU")
            return True
        except Exception as e:
            logger.error(f"Failed to transfer arrays to CPU: {e}")
            return False

    def allocate_neuron(self, neuron_id: int) -> int:
        """Allocate space for a neuron and return its array index.
        
        Args:
            neuron_id: Unique ID for the neuron
            
        Returns:
            Index in the arrays where the neuron is stored
        """
        if neuron_id in self.id_to_index_map:
            logger.warning(f"Neuron ID {neuron_id} already exists")
            return self.id_to_index_map[neuron_id]
            
        # Reuse a free index if available, otherwise use next_index
        if self.free_indices:
            index = self.free_indices.pop()
        else:
            if self.next_index >= self.aligned_capacity:
                raise ValueError(f"Maximum neuron capacity ({self.aligned_capacity}) reached")
            index = self.next_index
            self.next_index += 1
            
        if not self._use_rust:
            # Mark this index as valid and store the ID mapping
            valid_mask_np = self.backend.to_numpy(self.valid_mask)
            valid_mask_np[index] = True
            self.valid_mask = self.backend.array(valid_mask_np)
        
        # Store bidirectional mapping
        self.id_to_index_map[neuron_id] = index
        self.index_to_id_map[index] = neuron_id
        self.neuron_count += 1
        
        # Invalidate lookup array cache
        self._invalidate_index_to_id_lookup_array()
        
        return index

    def delete_neuron(self, neuron_id: int) -> bool:
        """Delete a neuron by marking its index as available for reuse.
        
        Args:
            neuron_id: ID of the neuron to delete
            
        Returns:
            True if deleted, False if it didn't exist
        """
        if neuron_id not in self.id_to_index_map:
            return False
            
        index = self.id_to_index_map[neuron_id]
        
        if not self._use_rust:
            # Mark as invalid and reset values
            valid_mask_np = self.backend.to_numpy(self.valid_mask)
            valid_mask_np[index] = False
            self.valid_mask = self.backend.array(valid_mask_np)
            
            # Reset neuron properties to defaults
            potentials = self.backend.to_numpy(self.membrane_potentials)
            potentials[index] = 0.0
            self.membrane_potentials = self.backend.array(potentials)
        
        # Add to free indices for reuse
        self.free_indices.add(index)
        
        # Remove from bidirectional mapping
        del self.id_to_index_map[neuron_id]
        if index in self.index_to_id_map:
            del self.index_to_id_map[index]
        
        # Invalidate lookup array cache
        self._invalidate_index_to_id_lookup_array()
        
        self.neuron_count -= 1
        return True

    def create_neuron(
        self,
        cortical_idx: Optional[int] = None,
        position: Tuple[int, int, int] = (0, 0, 0),
        threshold: float = 1.0,
        membrane_potential: float = 0.0,
        resting_potential: float = 0.0,
        decay_rate: float = 0.5,
        refractory_period: int = 1,
        neuron_type: int = 0,
        is_active: bool = True,
        properties: Optional[Dict[str, Any]] = None
    ) -> int:
        """
        Create a new neuron and add it to the array.

        Args:
            cortical_idx: Integer index of the cortical area this neuron belongs to
            position: 3D coordinates (x, y, z)
            threshold: Firing threshold
            membrane_potential: Initial membrane potential
            resting_potential: Resting potential
            decay_rate: Membrane potential decay rate (0-1)
            refractory_period: Refractory period in timesteps
            neuron_type: Type of neuron (0=excitatory, 1=inhibitory, etc.)
            is_active: Whether the neuron is initially active
            properties: Additional properties for the neuron

        Returns:
            ID of the created neuron
        """
        # Default to cortical_idx 0 if not provided
        if cortical_idx is None:
            cortical_idx = 0
        
        # Make sure we have capacity
        if self.next_index >= self.max_neurons:
            raise ValueError(f"Maximum number of neurons ({self.max_neurons}) exceeded")

        # Generate unique neuron ID - CRITICAL FIX: Keep IDs separate from indices
        if not hasattr(self, '_next_neuron_id'):
            # Initialize the neuron ID counter if it doesn't exist yet
            self._next_neuron_id = max(self.id_to_index_map.keys(), default=0) + 1
        
        neuron_id = self._next_neuron_id
        self._next_neuron_id += 1
        
        # Allocate space for the neuron in the array
        idx = self.allocate_neuron(neuron_id)

        # Set valid flag
        self.valid_mask[idx] = True
        
        # Initialize the neuron's properties
        self.membrane_potentials[idx] = membrane_potential
        self.resting_potentials[idx] = resting_potential
        self.thresholds[idx] = threshold
        self.decay_rates[idx] = decay_rate
        self.refractory_periods[idx] = refractory_period
        self.refractory_counters[idx] = 0  # Start with no refractory state
        
        # Initialize coordinates as uint32
        self.coordinates_x[idx] = np.uint32(max(0, position[0]))
        self.coordinates_y[idx] = np.uint32(max(0, position[1]))
        self.coordinates_z[idx] = np.uint32(max(0, position[2]))
        
        self.cortical_idxs[idx] = cortical_idx
        self.is_active[idx] = is_active
        
        # Store any additional properties
        if properties:
            self.properties[neuron_id] = properties
            
        return neuron_id

    def get_neuron_property(self, neuron_id: int, property_name: str) -> Any:
        """Get a specific property of a neuron.
        
        Args:
            neuron_id: ID of the neuron
            property_name: Name of the property to get
            
        Returns:
            Value of the requested property
            
        Raises:
            KeyError: If the neuron_id doesn't exist
            KeyError: If the property doesn't exist
        """
        if neuron_id not in self.id_to_index_map:
            raise KeyError(f"Neuron {neuron_id} does not exist")
        
        index = self.id_to_index_map[neuron_id]
        
        if property_name == "membrane_potential":
            return float(self.membrane_potentials[index])
        elif property_name == "resting_potential":
            return float(self.resting_potentials[index])
        elif property_name == "threshold":
            return float(self.thresholds[index])
        elif property_name == "decay_rate":
            return float(self.decay_rates[index])
        elif property_name == "refractory_period":
            return int(self.refractory_periods[index])
        elif property_name == "refractory_counter":
            return int(self.refractory_counters[index])
        elif property_name == "cortical_idx":
            return int(self.cortical_idxs[index])
        elif property_name == "position":
            return (int(np.uint32(self.coordinates_x[index])),
                    int(np.uint32(self.coordinates_y[index])),
                    int(np.uint32(self.coordinates_z[index])))
        elif property_name == "is_active":
            return bool(self.is_active[index])
        else:
            raise KeyError(f"Property {property_name} not found")

    def set_neuron_property(self, neuron_id: int, property_name: str, value: Any) -> None:
        """Set a specific property of a neuron.
        
        Args:
            neuron_id: ID of the neuron
            property_name: Name of the property to set
            value: New value for the property
            
        Raises:
            KeyError: If the neuron_id doesn't exist
            KeyError: If the property doesn't exist
        """
        if neuron_id not in self.id_to_index_map:
            raise KeyError(f"Neuron {neuron_id} does not exist")
        
        index = self.id_to_index_map[neuron_id]
        
        if property_name == "membrane_potential":
            self.membrane_potentials[index] = float(value)
        elif property_name == "resting_potential":
            self.resting_potentials[index] = float(value)
        elif property_name == "threshold":
            self.thresholds[index] = float(value)
        elif property_name == "decay_rate":
            self.decay_rates[index] = float(value)
        elif property_name == "refractory_period":
            self.refractory_periods[index] = int(value)
        elif property_name == "refractory_counter":
            self.refractory_counters[index] = int(value)
        elif property_name == "cortical_idx":
            self.cortical_idxs[index] = int(value)
        elif property_name == "position":
            if not isinstance(value, tuple) or len(value) != 3:
                raise ValueError("Position must be a tuple of (x, y, z)")
            self.coordinates_x[index] = np.uint32(max(0, int(value[0])))
            self.coordinates_y[index] = np.uint32(max(0, int(value[1])))
            self.coordinates_z[index] = np.uint32(max(0, int(value[2])))
        elif property_name == "is_active":
            self.is_active[index] = bool(value)
        else:
            raise KeyError(f"Property {property_name} not found")

    def get_neurons_by_cortical_area(self, cortical_id: int) -> List[int]:
        """
        Get all neurons in a specific cortical area.
        
        Args:
            cortical_id: ID of the cortical area
            
        Returns:
            List of neuron IDs in the area
        """
        # Find neurons with matching cortical_id and valid mask
        indices = self.cortical_id_to_indices.get(cortical_id, [])
        
        # Convert indices to neuron IDs
        neuron_ids = []
        for idx in indices:
            if self.valid_mask[idx]:
                for neuron_id, index in self.id_to_index_map.items():
                    if index == idx:
                        neuron_ids.append(neuron_id)
                        break
        
        return neuron_ids

    def get_neurons_by_area(self, area_id: int) -> List[int]:
        """
        Get all neurons in a specific cortical area.
        
        WARNING: This method is deprecated and will be removed. 
        Use get_neurons_by_cortical_area() instead.
        
        Args:
            area_id: ID of the cortical area
            
        Returns:
            List of neuron IDs in the area
        """
        import warnings
        warnings.warn(
            "get_neurons_by_area is deprecated and will be removed. "
            "Use get_neurons_by_cortical_area instead.",
            DeprecationWarning,
            stacklevel=2
        )
        return self.get_neurons_by_cortical_area(area_id)

    def get_neuron_count(self) -> int:
        """Get the total number of neurons.
        
        Returns:
            Total number of neurons
        """
        if isinstance(self.valid_mask, torch.Tensor):
            return int(torch.sum(self.valid_mask).item())
        else:
            return int(np.sum(self.valid_mask))

    def update_membrane_potentials(self, synapse_indices=None, synapse_data=None, 
                                  timestep: Optional[int] = None, decay_factor: Optional[float] = None):
        """High-performance membrane potential update with embedded optimization.
        
        This method provides a unified interface that automatically uses the most optimized
        path available (Rust backend > SIMD > fallback).
        
        Args:
            synapse_indices: Indices of neurons that fired (for backward compatibility)
            synapse_data: Synaptic connectivity data (sparse matrix or BlockSparseMatrix)
            timestep: Current simulation timestep
            decay_factor: Optional global decay factor (for backward compatibility)
            
        Returns:
            List of neuron IDs that fired this timestep
        """
        # If called with old-style parameters, provide backward compatibility
        if decay_factor is not None and synapse_indices is None and synapse_data is None:
            # Legacy mode: just apply decay and return empty list
            simd_membrane_decay(self.membrane_potentials, 
                              np.full_like(self.decay_rates, decay_factor), 
                              self.valid_mask)
            return []
        
        # Use embedded optimization for full neural update
        if timestep is None:
            timestep = getattr(self, '_current_timestep', 0)
        
        # Perform complete optimized neural update
        fired_neurons = self.embedded_optimized_neural_update(timestep, synapse_data)
        
        # If we have synaptic data, integrate synaptic signals
        if synapse_data is not None and fired_neurons:
            self.embedded_synaptic_integration(fired_neurons, synapse_data)
        elif synapse_indices is not None and synapse_data is not None:
            # Legacy synaptic integration mode
            self._legacy_synaptic_integration(synapse_indices, synapse_data)
        
        return fired_neurons
    
    def _legacy_synaptic_integration(self, synapse_indices, synapse_data):
        """Legacy synaptic integration for backward compatibility."""
        # Update refractory counters for valid neurons
        valid = self.valid_mask
        is_torch = isinstance(self.membrane_potentials, torch.Tensor)
        
        if is_torch:
            can_update_mask = valid & (self.refractory_counters == 0)
        else:
            can_update_mask = valid & (self.refractory_counters == 0)
        
        # Update membrane potentials for valid neurons with CRITICAL SHAPE VALIDATION
        if isinstance(synapse_data, torch.Tensor):
            # GPU implementation using PyTorch with shape safety
            for idx in synapse_indices:
                if idx >= len(self.membrane_potentials):
                    continue
                
                try:
                    # Get all post-synaptic targets with safe tensor operations
                    targets = torch.nonzero(synapse_data[idx]).squeeze()
                    if targets.dim() == 0 and targets.numel() > 0:
                        targets = targets.unsqueeze(0)
                    elif targets.dim() > 1:  # CRITICAL: Handle multi-dimensional nonzero results
                        targets = targets.squeeze(-1) if targets.shape[-1] == 1 else targets[:, 0]
                    
                    if targets.numel() == 0:
                        continue
                    
                    # CRITICAL BOUNDS VALIDATION: Ensure targets are within array bounds
                    if targets.max() >= self.membrane_potentials.shape[0]:
                        valid_bounds_mask = targets < self.membrane_potentials.shape[0]
                        targets = targets[valid_bounds_mask]
                        if targets.numel() == 0:
                            continue
                    
                    # Get weights with shape-safe indexing
                    weights = synapse_data[idx, targets]
                    
                    # CRITICAL SHAPE VALIDATION: Ensure weights and targets match exactly
                    if weights.shape[0] != targets.shape[0]:
                        min_size = min(weights.shape[0], targets.shape[0])
                        weights = weights[:min_size]
                        targets = targets[:min_size]
                        if min_size == 0:
                            continue
                    
                    # Apply refractory mask with bounds checking
                    update_mask = can_update_mask[targets]
                    
                    # CRITICAL: Final shape validation before membrane potential update
                    if update_mask.shape[0] != targets.shape[0] or update_mask.shape[0] != weights.shape[0]:
                        logger.warning(f"BURST ENGINE: Tensor shape mismatch avoided - update_mask: {update_mask.shape}, targets: {targets.shape}, weights: {weights.shape}")
                        continue
                    
                    if update_mask.any():
                        valid_targets = targets[update_mask]
                        valid_weights = weights[update_mask]
                        
                        # Final safety check before tensor operation
                        if valid_targets.shape[0] == valid_weights.shape[0] and valid_targets.shape[0] > 0:
                            self.membrane_potentials[valid_targets] += valid_weights
                        
                except Exception as e:
                    # CRITICAL: Catch and log broadcasting errors instead of crashing
                    logger.warning(f"BURST ENGINE: Synaptic integration error caught for neuron {idx}: {e}")
                    continue
        else:
            # CPU implementation using sparse matrices
            for idx in synapse_indices:
                if idx >= synapse_data.shape[0]:
                    continue
                
                # Get the row for this neuron
                row = synapse_data.getrow(idx)
                targets = row.indices
                weights = row.data
                
                if len(targets) == 0:
                    continue
                
                # Only update neurons not in refractory period
                update_mask = can_update_mask[targets]
                if np.any(update_mask):
                    self.membrane_potentials[targets[update_mask]] += weights[update_mask]
        
        # Check which neurons exceed threshold and should fire
        fired = can_update_mask & (self.membrane_potentials >= self.thresholds)
        
        # Reset membrane potentials and set refractory period for fired neurons
        if is_torch:
            self.membrane_potentials[fired] = self.resting_potentials[fired]
            self.refractory_counters[fired] = self.refractory_periods[fired]
            self.is_active[fired] = True
        else:
            np.copyto(self.membrane_potentials, self.resting_potentials, where=fired)
            np.copyto(self.refractory_counters, self.refractory_periods, where=fired)
            self.is_active[fired] = True
        
        # Decay membrane potentials for all valid neurons
        valid_not_fired = valid & ~fired
        if is_torch:
            self.membrane_potentials[valid_not_fired] *= self.decay_rates[valid_not_fired]
        else:
            self.membrane_potentials[valid_not_fired] *= self.decay_rates[valid_not_fired]

    def decay_and_check_firing(self):
        """Decay membrane potentials and check for neurons that now exceed threshold.
        
        This method is used when no neurons in the FCL are firing, to handle passive
        decay and check if any neurons cross their threshold due to decay towards
        resting potential.
        
        Returns:
            Array of indices that exceed their threshold and fire
        """
        # Get the mask of valid neurons
        valid = self.valid_mask
        
        # Only consider neurons not in refractory period
        can_update_mask = valid & (self.refractory_counters <= 0)
        
        # For neurons in refractory period, decrement counters
        in_refractory = valid & (self.refractory_counters > 0)
        
        if isinstance(self.refractory_counters, torch.Tensor):
            self.refractory_counters[in_refractory] -= 1
        else:
            np.subtract(self.refractory_counters, 1, out=self.refractory_counters, where=in_refractory)
        
        # Decay membrane potentials for all valid neurons that can update
        if isinstance(self.membrane_potentials, torch.Tensor):
            self.membrane_potentials[can_update_mask] = (
                self.membrane_potentials[can_update_mask] * (1.0 - self.decay_rates[can_update_mask]) + 
                self.resting_potentials[can_update_mask] * self.decay_rates[can_update_mask]
            )
        else:
            decay_effect = self.membrane_potentials[can_update_mask] * (1.0 - self.decay_rates[can_update_mask])
            rest_effect = self.resting_potentials[can_update_mask] * self.decay_rates[can_update_mask]
            self.membrane_potentials[can_update_mask] = decay_effect + rest_effect
        
        # Check which neurons exceed threshold and should fire
        fired = can_update_mask & (self.membrane_potentials >= self.thresholds)
        
        # Reset membrane potentials and set refractory period for fired neurons
        if isinstance(self.membrane_potentials, torch.Tensor):
            self.membrane_potentials[fired] = self.resting_potentials[fired]
            self.refractory_counters[fired] = self.refractory_periods[fired]
            self.is_active[fired] = True
        else:
            np.copyto(self.membrane_potentials, self.resting_potentials, where=fired)
            np.copyto(self.refractory_counters, self.refractory_periods, where=fired)
            self.is_active[fired] = True
        
        # Return indices of fired neurons
        return np.where(fired)[0]

    def to_dict(self, neuron_id: int) -> Dict[str, Any]:
        """Convert a neuron to a dictionary representation (for API compatibility).
        
        Args:
            neuron_id: ID of the neuron
            
        Returns:
            Dictionary representation of the neuron
            
        Raises:
            KeyError: If the neuron_id doesn't exist
        """
        if neuron_id not in self.id_to_index_map:
            raise KeyError(f"Neuron {neuron_id} does not exist")
        
        index = self.id_to_index_map[neuron_id]
        
        return {
            "id": neuron_id,
            "cortical_idx": int(self.cortical_idxs[index]),
            "position": (int(self.coordinates_x[index]), 
                         int(self.coordinates_y[index]), 
                         int(self.coordinates_z[index])),
            "threshold": float(self.thresholds[index]),
            "membrane_potential": float(self.membrane_potentials[index]),
            "resting_potential": float(self.resting_potentials[index]),
            "decay_rate": float(self.decay_rates[index]),
            "refractory_period": int(self.refractory_periods[index]),
            "refractory_counter": int(self.refractory_counters[index]),
            "is_active": bool(self.is_active[index])
        }

    def batch_create_neurons(self, cortical_idx: Optional[int], positions: List[Tuple[int, int, int]],
                         thresholds: Union[float, List[float]] = 1.0,
                         membrane_potentials: Union[float, List[float]] = 0.0,
                         resting_potentials: Union[float, List[float]] = 0.0,
                         decay_rates: Union[float, List[float]] = 0.5,
                         refractory_periods: Union[int, List[int]] = 1) -> List[int]:
        """Create multiple neurons with the same or different properties in batch.
        
        This method is optimized for creating large numbers of neurons at once by
        using vectorized operations instead of loops.
        
        Args:
            cortical_idx: Integer index of the cortical area
            positions: List of 3D coordinates for each neuron
            thresholds: Either a single value for all neurons or a list of values
            membrane_potentials: Either a single value for all neurons or a list of values
            resting_potentials: Either a single value for all neurons or a list of values
            decay_rates: Either a single value for all neurons or a list of values
            refractory_periods: Either a single value for all neurons or a list of values
            
        Returns:
            List of neuron IDs created
            
        Raises:
            ValueError: If any list parameter doesn't match the length of positions
        """
        # Default cortical_idx to 0 if None
        if cortical_idx is None:
            cortical_idx = 0
            
        num_neurons = len(positions)
        
        # CRITICAL SAFETY LIMIT: Prevent segmentation faults with huge batches
        # The segfault occurs around 12,288 neurons due to multiprocessing issues
        MAX_SAFE_BATCH_SIZE = 8192  # Power of 2, well below the crash threshold
        
        if num_neurons > MAX_SAFE_BATCH_SIZE:
            # Split large batches into smaller safe chunks to prevent segfaults
            all_neuron_ids = []
            for i in range(0, num_neurons, MAX_SAFE_BATCH_SIZE):
                end_idx = min(i + MAX_SAFE_BATCH_SIZE, num_neurons)
                batch_positions = positions[i:end_idx]
                batch_size = len(batch_positions)
                
                # Handle parameter slicing for each batch
                def slice_param(param, start, end):
                    if isinstance(param, list):
                        return param[start:end]
                    else:
                        return param  # Single value used for entire batch
                
                batch_neuron_ids = self.batch_create_neurons(
                    cortical_idx=cortical_idx,
                    positions=batch_positions,
                    thresholds=slice_param(thresholds, i, end_idx),
                    membrane_potentials=slice_param(membrane_potentials, i, end_idx),
                    resting_potentials=slice_param(resting_potentials, i, end_idx),
                    decay_rates=slice_param(decay_rates, i, end_idx),
                    refractory_periods=slice_param(refractory_periods, i, end_idx)
                )
                all_neuron_ids.extend(batch_neuron_ids)
                
                # CRITICAL: Force garbage collection after each batch to prevent resource buildup
                import gc
                gc.collect()
            
            return all_neuron_ids
        
        # Validate lengths of parameters if they are lists
        for param_name, param_value in [
            ("thresholds", thresholds),
            ("membrane_potentials", membrane_potentials),
            ("resting_potentials", resting_potentials),
            ("decay_rates", decay_rates),
            ("refractory_periods", refractory_periods)
        ]:
            if isinstance(param_value, list) and len(param_value) != num_neurons:
                raise ValueError(f"Length of {param_name} ({len(param_value)}) does not match length of positions ({num_neurons})")
        
        # Find indices for all neurons
        if len(self.free_indices) >= num_neurons:
            # We have enough free indices
            indices = np.array(list(self.free_indices)[:num_neurons])
            self.free_indices = self.free_indices - set(indices)
        else:
            # Use free indices + new indices
            num_new_indices = num_neurons - len(self.free_indices)
            
            if self.next_index + num_new_indices > self.max_neurons:
                raise ValueError(f"Maximum number of neurons ({self.max_neurons}) exceeded")
            
            free_indices = np.array(list(self.free_indices)) if self.free_indices else np.array([], dtype=np.int32)
            new_indices = np.arange(self.next_index, self.next_index + num_new_indices, dtype=np.int32)
            indices = np.concatenate([free_indices, new_indices])
            
            self.free_indices = set()
            self.next_index += num_new_indices
        
        # CRITICAL FIX: Generate unique neuron IDs separately from array indices
        # Array indices are for internal storage, neuron IDs are external identifiers
        # Reusing indices as IDs causes memory corruption and tensor shape mismatches
        if not hasattr(self, '_next_neuron_id'):
            self._next_neuron_id = 1  # Start neuron IDs from 1 (0 reserved for invalid)
        
        neuron_ids = list(range(self._next_neuron_id, self._next_neuron_id + num_neurons))
        self._next_neuron_id += num_neurons
        
        # Prepare property arrays (convert to arrays if they are single values)
        # Use backend-specific arrays
        # Convert inputs to arrays with proper shapes and types - MEMORY SAFE
        # Ensure all arrays have exactly num_neurons elements to prevent broadcasting errors
        if not isinstance(thresholds, list) and not isinstance(thresholds, np.ndarray):
            thresholds_list = [float(thresholds)] * num_neurons
        else:
            thresholds_list = list(thresholds)
            if len(thresholds_list) != num_neurons:
                raise ValueError(f"thresholds length ({len(thresholds_list)}) must match num_neurons ({num_neurons})")
        thresholds = self.backend.array(thresholds_list)
        
        if not isinstance(membrane_potentials, list) and not isinstance(membrane_potentials, np.ndarray):
            membrane_potentials_list = [float(membrane_potentials)] * num_neurons
        else:
            membrane_potentials_list = list(membrane_potentials)
            if len(membrane_potentials_list) != num_neurons:
                raise ValueError(f"membrane_potentials length ({len(membrane_potentials_list)}) must match num_neurons ({num_neurons})")
        membrane_potentials = self.backend.array(membrane_potentials_list)
        
        if not isinstance(resting_potentials, list) and not isinstance(resting_potentials, np.ndarray):
            resting_potentials_list = [float(resting_potentials)] * num_neurons
        else:
            resting_potentials_list = list(resting_potentials)
            if len(resting_potentials_list) != num_neurons:
                raise ValueError(f"resting_potentials length ({len(resting_potentials_list)}) must match num_neurons ({num_neurons})")
        resting_potentials = self.backend.array(resting_potentials_list)
        
        if not isinstance(decay_rates, list) and not isinstance(decay_rates, np.ndarray):
            decay_rates_list = [float(decay_rates)] * num_neurons
        else:
            decay_rates_list = list(decay_rates)
            if len(decay_rates_list) != num_neurons:
                raise ValueError(f"decay_rates length ({len(decay_rates_list)}) must match num_neurons ({num_neurons})")
        decay_rates = self.backend.array(decay_rates_list)
        
        if not isinstance(refractory_periods, list) and not isinstance(refractory_periods, np.ndarray):
            refractory_periods_list = [int(refractory_periods)] * num_neurons
        else:
            refractory_periods_list = list(refractory_periods)
            if len(refractory_periods_list) != num_neurons:
                raise ValueError(f"refractory_periods length ({len(refractory_periods_list)}) must match num_neurons ({num_neurons})")
        refractory_periods = self.backend.array(refractory_periods_list)
        
        # Extract position components as uint32
        coordinates_x = self.backend.array([np.uint32(max(0, pos[0])) for pos in positions])
        coordinates_y = self.backend.array([np.uint32(max(0, pos[1])) for pos in positions])
        coordinates_z = self.backend.array([np.uint32(max(0, pos[2])) for pos in positions])
        
        # Get numpy array of indices for indexing
        idx_array = indices
        
        # Convert to tensors if using PyTorch
        if isinstance(self.membrane_potentials, torch.Tensor):
            # Validate tensor shapes before operations to prevent memory corruption
            idx_tensor = torch.tensor(idx_array, dtype=torch.long)
            
            # CRITICAL MEMORY SAFETY: Validate all indices are within bounds
            max_idx = torch.max(idx_tensor).item() if len(idx_tensor) > 0 else 0
            if max_idx >= self.membrane_potentials.shape[0]:
                raise ValueError(f"Index out of bounds: max_idx={max_idx}, tensor_size={self.membrane_potentials.shape[0]}")
            
            # CRITICAL: Check for negative indices which cause undefined behavior
            min_idx = torch.min(idx_tensor).item() if len(idx_tensor) > 0 else 0
            if min_idx < 0:
                raise ValueError(f"Negative index detected: min_idx={min_idx}, this causes memory corruption")
            
            # Verify tensor compatibility before index_copy operations
            expected_size = len(idx_array)
            if (membrane_potentials.shape[0] != expected_size or 
                resting_potentials.shape[0] != expected_size or
                thresholds.shape[0] != expected_size or
                decay_rates.shape[0] != expected_size or
                refractory_periods.shape[0] != expected_size):
                raise ValueError(f"Tensor size mismatch: expected {expected_size}, got membrane_potentials={membrane_potentials.shape[0]}, resting_potentials={resting_potentials.shape[0]}, thresholds={thresholds.shape[0]}, decay_rates={decay_rates.shape[0]}, refractory_periods={refractory_periods.shape[0]}")
            
            # Determine the dtype of the target tensors
            target_dtype = self.membrane_potentials.dtype
            
            try:
                # MEMORY SAFE: Use in-place operations with explicit bounds checking
                # Create a safe slice to avoid out-of-bounds writes
                safe_indices = idx_tensor[idx_tensor < self.membrane_potentials.shape[0]]
                safe_count = len(safe_indices)
                
                if safe_count != len(idx_tensor):
                    raise ValueError(f"Unsafe indices detected: {len(idx_tensor) - safe_count} indices out of bounds")
                
                # CRITICAL FIX: Ensure tensors are on the same device before operations
                device = self.membrane_potentials.device
                safe_indices = safe_indices.to(device)
                membrane_potentials = membrane_potentials.to(device)
                resting_potentials = resting_potentials.to(device)
                thresholds = thresholds.to(device)
                decay_rates = decay_rates.to(device)
                refractory_periods = refractory_periods.to(device)
                coordinates_x = coordinates_x.to(device)
                coordinates_y = coordinates_y.to(device)
                coordinates_z = coordinates_z.to(device)
                
                # Use PyTorch indexing for tensors - ensure matching dtypes and safe operations
                self.membrane_potentials[safe_indices] = membrane_potentials[:safe_count].to(dtype=target_dtype)
                self.resting_potentials[safe_indices] = resting_potentials[:safe_count].to(dtype=target_dtype)
                self.thresholds[safe_indices] = thresholds[:safe_count].to(dtype=target_dtype)
                self.decay_rates[safe_indices] = decay_rates[:safe_count].to(dtype=target_dtype)
                self.refractory_periods[safe_indices] = refractory_periods[:safe_count].to(dtype=self.refractory_periods.dtype)
                self.coordinates_x[safe_indices] = coordinates_x[:safe_count].to(dtype=self.coordinates_x.dtype)
                self.coordinates_y[safe_indices] = coordinates_y[:safe_count].to(dtype=self.coordinates_y.dtype)
                self.coordinates_z[safe_indices] = coordinates_z[:safe_count].to(dtype=self.coordinates_z.dtype)
                self.cortical_idxs[safe_indices] = cortical_idx
                
                # Set valid mask - use safe numpy operations to avoid tensor issues
                valid_mask = self.backend.to_numpy(self.valid_mask)
                # MEMORY SAFETY: Bounds check for valid_mask updates
                numpy_indices = idx_array[:safe_count]
                
                # CRITICAL FIX: Validate valid_mask array integrity before update
                if not isinstance(valid_mask, np.ndarray):
                    raise RuntimeError("valid_mask is not a numpy array, memory corruption detected")
                
                if len(valid_mask) != self.aligned_capacity:
                    raise RuntimeError(f"valid_mask size mismatch: expected {self.aligned_capacity}, got {len(valid_mask)}")
                
                # CRITICAL FIX: Only update indices that are within bounds
                if len(numpy_indices) > 0:
                    max_idx = np.max(numpy_indices)
                    if max_idx >= len(valid_mask):
                        raise ValueError(f"Valid mask update would be out of bounds: max_idx={max_idx}, mask_size={len(valid_mask)}")
                    
                    # Ensure valid_mask is writable and not a read-only view
                    if not valid_mask.flags.writeable:
                        valid_mask = valid_mask.copy()
                    
                    # Safe update with bounds-checked indices
                    valid_mask[numpy_indices] = True
                    self.valid_mask = self.backend.array(valid_mask)
                else:
                    # No indices to update, but ensure valid_mask remains consistent
                    self.valid_mask = self.backend.array(valid_mask)
                
            except Exception as e:
                raise RuntimeError(f"PyTorch tensor operation failed during neuron creation: {e}. This may indicate memory corruption or incompatible tensor shapes.")
        else:
            # For NumPy arrays, use standard indexing with CRITICAL BOUNDS CHECKING
            # MEMORY SAFETY: Validate all array operations to prevent corruption
            if len(idx_array) > 0:
                max_idx = np.max(idx_array)
                if max_idx >= len(self.membrane_potentials):
                    raise ValueError(f"Index out of bounds in NumPy path: max_idx={max_idx}, array_size={len(self.membrane_potentials)}")
                
                min_idx = np.min(idx_array)
                if min_idx < 0:
                    raise ValueError(f"Negative index in NumPy path: min_idx={min_idx}")
            
            # CRITICAL FIX: Validate array sizes before assignment
            expected_size = len(idx_array)
            if (len(membrane_potentials) != expected_size or 
                len(resting_potentials) != expected_size or
                len(thresholds) != expected_size or
                len(decay_rates) != expected_size or
                len(refractory_periods) != expected_size or
                len(coordinates_x) != expected_size or
                len(coordinates_y) != expected_size or
                len(coordinates_z) != expected_size):
                raise ValueError(f"Array size mismatch in NumPy path: expected {expected_size}")
            
            # Safe array updates with bounds checking
            self.membrane_potentials[idx_array] = membrane_potentials
            self.resting_potentials[idx_array] = resting_potentials
            self.thresholds[idx_array] = thresholds
            self.decay_rates[idx_array] = decay_rates
            self.refractory_periods[idx_array] = refractory_periods
            self.coordinates_x[idx_array] = coordinates_x
            self.coordinates_y[idx_array] = coordinates_y
            self.coordinates_z[idx_array] = coordinates_z
            self.cortical_idxs[idx_array] = cortical_idx
            
            # CRITICAL FIX: Safe valid_mask update with bounds checking
            if len(idx_array) > 0:
                max_idx = np.max(idx_array)
                if max_idx >= len(self.valid_mask):
                    raise ValueError(f"Valid mask index out of bounds: max_idx={max_idx}, mask_size={len(self.valid_mask)}")
                
                # Ensure valid_mask is writable
                if not self.valid_mask.flags.writeable:
                    raise RuntimeError("valid_mask is not writable, memory corruption detected")
                
                self.valid_mask[idx_array] = True
        
        # Update mappings
        for i, neuron_id in enumerate(neuron_ids):
            idx = idx_array[i]
            self.id_to_index_map[neuron_id] = idx
            self.index_to_id_map[idx] = neuron_id
            
            # Add to cortical area mapping
            if cortical_idx not in self.cortical_id_to_indices:
                self.cortical_id_to_indices[cortical_idx] = []
            self.cortical_id_to_indices[cortical_idx].append(idx)
        
        # Update neuron count
        self.neuron_count += len(neuron_ids)
        
        # Invalidate lookup array cache after batch operations
        self._invalidate_index_to_id_lookup_array()
        
        return neuron_ids

    def batch_update_membrane_potentials(self, neuron_ids: List[int], values: List[float]) -> None:
        """Update membrane potentials for multiple neurons at once.
        
        Args:
            neuron_ids: List of neuron IDs
            values: List of new membrane potential values
            
        Raises:
            ValueError: If lengths of neuron_ids and values don't match
        """
        if len(neuron_ids) != len(values):
            raise ValueError(f"Length of neuron_ids ({len(neuron_ids)}) does not match length of values ({len(values)})")
        
        # Get indices for the neuron IDs
        indices = np.array([self.id_to_index_map[nid] for nid in neuron_ids], dtype=np.int32)
        
        # Update membrane potentials in a vectorized way
        self.membrane_potentials[indices] = np.array(values, dtype=np.float32)

    # ============================================================================
    # VECTORIZED INDEX-TO-ID LOOKUP METHODS - GPU/SIMD COMPATIBLE
    # ============================================================================
    
    def batch_indices_to_neuron_ids(self, indices: np.ndarray) -> np.ndarray:
        """Convert array indices to neuron IDs in vectorized operation.
        
        This replaces inefficient dictionary comprehensions like:
        [self.index_to_neuron_id[idx] for idx in indices if idx in self.index_to_neuron_id]
        
        Args:
            indices: NumPy array of indices to convert
            
        Returns:
            NumPy array of neuron IDs (same length as indices, -1 for invalid indices)
        """
        # Vectorized lookup using NumPy - GPU/SIMD friendly
        neuron_ids = np.full(len(indices), -1, dtype=np.int64)
        valid_mask = np.zeros(len(indices), dtype=bool)
        
        # Find valid indices efficiently
        for i, idx in enumerate(indices):
            if idx in self.index_to_id_map:
                neuron_ids[i] = self.index_to_id_map[idx]
                valid_mask[i] = True
        
        return neuron_ids, valid_mask
    
    def vectorized_indices_to_neuron_ids(self, indices: np.ndarray, filter_invalid: bool = True) -> np.ndarray:
        """Ultra-fast vectorized index-to-ID conversion using pre-built lookup array.
        
        This is the highest performance method for index→ID conversion.
        Uses a pre-allocated lookup array for O(1) access instead of dictionary lookups.
        
        Args:
            indices: NumPy array of indices to convert
            filter_invalid: If True, return only valid neuron IDs. If False, include -1 for invalid.
            
        Returns:
            NumPy array of neuron IDs
        """
        # Create a lookup array on first use for O(1) vectorized access
        if not hasattr(self, '_index_to_id_lookup_array'):
            self._build_index_to_id_lookup_array()
        
        # Vectorized lookup - single memory access per element, SIMD-friendly
        if filter_invalid:
            # Filter out invalid indices before lookup
            valid_mask = (indices >= 0) & (indices < len(self._index_to_id_lookup_array))
            valid_indices = indices[valid_mask]
            if len(valid_indices) == 0:
                return np.array([], dtype=np.int64)
            neuron_ids = self._index_to_id_lookup_array[valid_indices]
            # Filter out -1 values (unmapped indices)
            return neuron_ids[neuron_ids != -1]
        else:
            # Include invalid indices as -1
            safe_indices = np.clip(indices, 0, len(self._index_to_id_lookup_array) - 1)
            neuron_ids = self._index_to_id_lookup_array[safe_indices]
            # Mark out-of-bounds indices as -1
            neuron_ids[indices >= len(self._index_to_id_lookup_array)] = -1
            neuron_ids[indices < 0] = -1
            return neuron_ids
    
    def _build_index_to_id_lookup_array(self):
        """Build a fast lookup array for index→ID conversion.
        
        This creates a pre-allocated array where lookup_array[index] = neuron_id.
        Much faster than dictionary lookups for vectorized operations.
        """
        # MEMORY SAFETY: Guard against double-allocation and ensure proper cleanup
        if hasattr(self, '_index_to_id_lookup_array'):
            # Clean up existing array to prevent memory leaks
            del self._index_to_id_lookup_array
        
        # Create lookup array initialized to -1 (invalid) with bounds checking
        try:
            self._index_to_id_lookup_array = np.full(self.aligned_capacity, -1, dtype=np.int64)
        except MemoryError:
            # Fallback to smaller allocation if memory is tight
            self._index_to_id_lookup_array = np.full(self.max_neurons, -1, dtype=np.int64)
        
        # MEMORY SAFETY: Populate with valid mappings using bounds checking
        for neuron_id, index in self.id_to_index_map.items():
            if 0 <= index < len(self._index_to_id_lookup_array):
                self._index_to_id_lookup_array[index] = neuron_id
            else:
                # Log warning about out-of-bounds index but don't crash
                logger.warning(f"Index {index} for neuron {neuron_id} is out of bounds (max: {len(self._index_to_id_lookup_array)-1})")
    
    def _invalidate_index_to_id_lookup_array(self):
        """Invalidate the lookup array when mappings change - with memory safety."""
        # MEMORY SAFETY: Guard against double-free and use-after-free
        if hasattr(self, '_index_to_id_lookup_array'):
            try:
                # Explicitly delete the array to free memory immediately
                del self._index_to_id_lookup_array
            except Exception as e:
                # Log but don't crash on cleanup errors
                logger.warning(f"Error cleaning up lookup array: {e}")
            finally:
                # Ensure the attribute is removed even if deletion fails
                if hasattr(self, '_index_to_id_lookup_array'):
                    try:
                        delattr(self, '_index_to_id_lookup_array')
                    except AttributeError:
                        pass  # Already cleaned up
    
    def get_fired_neuron_ids_vectorized(self, fired_indices: np.ndarray) -> np.ndarray:
        """Get neuron IDs for fired indices using vectorized operations.
        
        This replaces the expensive list comprehension:
        [self.index_to_neuron_id[idx] for idx in fired_indices if idx in self.index_to_neuron_id]
        
        Args:
            fired_indices: NumPy array of indices that fired
            
        Returns:
            NumPy array of neuron IDs that fired
        """
        return self.vectorized_indices_to_neuron_ids(fired_indices, filter_invalid=True)

    def process_incoming_signals(self, signal_matrix, batch_size=10000):
        """Process incoming signals and update membrane potentials using GPU acceleration.
        
        This method takes a sparse matrix of pre-synaptic signals and updates all neuron
        membrane potentials in a vectorized operation that can be executed on GPU.
        
        Args:
            signal_matrix: Sparse matrix (CSR format) where rows are source neurons,
                          columns are target neurons, and values are signal strengths
            batch_size: Size of batches to process at once to avoid memory overflow
                        
        Returns:
            ndarray: Boolean mask of neurons that fired
        """
        if self.device == "cuda" and isinstance(self.membrane_potentials, torch.Tensor):
            # Use PyTorch for GPU acceleration if available
            # Process in batches to avoid memory overflow
            rows, cols = signal_matrix.nonzero()
            data = signal_matrix.data
            
            for i in range(0, len(rows), batch_size):
                batch_rows = rows[i:i+batch_size]
                batch_cols = cols[i:i+batch_size]
                batch_data = data[i:i+batch_size]
                
                if isinstance(batch_data, np.ndarray):
                    batch_data = torch.from_numpy(batch_data).to(self.device)
                
                # Convert to PyTorch tensors if they're numpy arrays
                if isinstance(batch_rows, np.ndarray):
                    batch_rows = torch.from_numpy(batch_rows).to(self.device)
                if isinstance(batch_cols, np.ndarray):
                    batch_cols = torch.from_numpy(batch_cols).to(self.device)
                
                # Use PyTorch scatter_add_ for efficient updates
                self.membrane_potentials.scatter_add_(0, batch_cols, batch_data)
                
            # Check for firing neurons
            fired_mask = self.membrane_potentials >= self.thresholds
            
            # Reset membrane potentials for neurons that fired
            self.membrane_potentials[fired_mask] = self.resting_potentials[fired_mask]
            
            # Set refractory counters for neurons that fired
            self.refractory_counters[fired_mask] = self.refractory_periods[fired_mask]
            
            # Update active neurons mask
            self.is_active = fired_mask
            
            return fired_mask.cpu().numpy() if isinstance(fired_mask, torch.Tensor) else fired_mask
            
        else:
            # NumPy implementation for CPU
            # Update all membrane potentials at once
            for i in range(0, signal_matrix.shape[0], batch_size):
                batch_end = min(i + batch_size, signal_matrix.shape[0])
                # Extract the batch of signals
                batch_signals = signal_matrix[i:batch_end, :]
                
                # Identify target neurons and signal values
                targets, values = batch_signals.nonzero()[1], batch_signals.data
                
                # Add signals to membrane potentials
                for target, value in zip(targets, values):
                    self.membrane_potentials[target] += value
            
            # Check for firing neurons - vectorized operation
            fired_mask = self.membrane_potentials >= self.thresholds
            
            # Reset membrane potentials for neurons that fired
            self.membrane_potentials[fired_mask] = self.resting_potentials[fired_mask]
            
            # Set refractory counters for neurons that fired
            self.refractory_counters[fired_mask] = self.refractory_periods[fired_mask]
            
            # Update active neurons mask
            self.is_active = fired_mask
            
            return fired_mask

    def use_best_available_device(self):
        """Automatically use the best available device (GPU if available, CPU otherwise).
        
        This method attempts to move the neuron arrays to GPU memory if a CUDA-capable
        GPU is available and has sufficient memory. If not, it falls back to CPU with
        a warning.
        
        Returns:
            str: The device being used ('cuda' or 'cpu')
        """
        try:
            if not torch.cuda.is_available():
                logger.info("CUDA is not available. Using CPU.")
                return "cpu"
            
            # Calculate approximate memory requirements
            # Each float32 value takes 4 bytes, each int32 value takes 4 bytes, each bool takes 1 byte
            float_arrays_count = 4  # membrane_potentials, resting_potentials, thresholds, decay_rates
            int_arrays_count = 4    # refractory_periods, refractory_counters, cortical_idxs
            bool_arrays_count = 2   # is_active, valid_mask
            positions_count = 3     # coordinates_x, coordinates_y, coordinates_z
            
            bytes_per_neuron = (float_arrays_count * 4) + (int_arrays_count * 4) + \
                             (bool_arrays_count * 1) + (positions_count * 4)
            
            total_bytes_needed = bytes_per_neuron * self.max_neurons
            
            # Add buffer for temporary arrays during computation (50% extra)
            total_bytes_needed = int(total_bytes_needed * 1.5)
            
            # Check available GPU memory
            free_memory, total_memory = torch.cuda.mem_get_info()
            
            if free_memory < total_bytes_needed:
                logger.warning(
                    f"Insufficient GPU memory. Need {total_bytes_needed/1e9:.2f} GB, "
                    f"but only {free_memory/1e9:.2f} GB free out of {total_memory/1e9:.2f} GB total."
                )
                return "cpu"
            
            # We have enough memory, move to GPU
            logger.info(
                f"Using GPU with {free_memory/1e9:.2f} GB free memory. "
                f"Estimated requirement: {total_bytes_needed/1e9:.2f} GB."
            )
            
            self.to_gpu()
            return "cuda"
            
        except Exception as e:
            logger.exception(f"Error switching to GPU: {e}")
            # Ensure we're using CPU if there was an error
            if self.device != "cpu":
                self.to_cpu()
            return "cpu"

    def get_device_stats(self):
        """Get statistics about the current device and memory usage.
        
        Returns:
            dict: Dictionary containing device information and memory usage
        """
        stats = {
            "device": self.device,
            "max_neurons": self.max_neurons,
            "used_neurons": np.sum(self.valid_mask),
            "arrays_size_mb": 0
        }
        
        # Calculate approximate memory usage
        # Each float32 array
        float_arrays = [
            self.membrane_potentials,
            self.resting_potentials,
            self.thresholds,
            self.decay_rates
        ]
        
        # Each int32 array
        int_arrays = [
            self.refractory_periods,
            self.refractory_counters,
            self.cortical_idxs,
            self.coordinates_x,
            self.coordinates_y,
            self.coordinates_z
        ]
        
        # Each bool array
        bool_arrays = [
            self.is_active,
            self.valid_mask
        ]
        
        # Calculate sizes
        float_size = sum(4 * self.max_neurons for _ in float_arrays)
        int_size = sum(4 * self.max_neurons for _ in int_arrays)
        bool_size = sum(1 * self.max_neurons for _ in bool_arrays)
        
        total_bytes = float_size + int_size + bool_size
        stats["arrays_size_mb"] = total_bytes / (1024 * 1024)
        
        # Add GPU-specific stats if on GPU
        if self.device == "cuda" and torch.cuda.is_available():
            stats["cuda_device_name"] = torch.cuda.get_device_name(0)
            free_memory, total_memory = torch.cuda.mem_get_info()
            stats["cuda_total_memory_gb"] = total_memory / (1024**3)
            stats["cuda_free_memory_gb"] = free_memory / (1024**3)
            stats["cuda_used_memory_gb"] = (total_memory - free_memory) / (1024**3)
        
        return stats


# For backwards compatibility
class Neuron:
    """
    Wrapper class for individual neurons.
    
    This class provides an object-oriented interface to individual neurons for API compatibility.
    It delegates most operations to the underlying NeuronArray for efficiency.
    """
    
    def __init__(
        self,
        neuron_id: int,
        cortical_id: Optional[str] = None,
        position: Optional[Tuple[int, int, int]] = None,
        threshold: float = 1.0,
        membrane_potential: float = 0.0,
        resting_potential: float = 0.0,
        decay_rate: float = 0.5,
        refractory_period: int = 1
    ):
        """
        Initialize a Neuron object.
        
        Args:
            neuron_id: Unique neuron ID
            cortical_id: ID of the cortical area
            position: 3D coordinates (x, y, z)
            threshold: Firing threshold
            membrane_potential: Initial membrane potential
            resting_potential: Resting potential
            decay_rate: Membrane potential decay rate
            refractory_period: Refractory period in timesteps
        """
        self.id = neuron_id
        self.cortical_id = cortical_id or "unknown"
        self.position = position or (0, 0, 0)
        self.threshold = threshold
        self.membrane_potential = membrane_potential
        self.resting_potential = resting_potential
        self.decay_rate = decay_rate
        self.refractory_period = refractory_period

    def to_dict(self) -> Dict[str, Any]:
        """Convert neuron to dictionary representation.
        
        Returns:
            Dictionary with neuron properties
        """
        return {
            "id": self.id,
            "cortical_id": self.cortical_id,
            "position": self.position,
            "threshold": self.threshold,
            "membrane_potential": self.membrane_potential,
            "resting_potential": self.resting_potential,
            "decay_rate": self.decay_rate,
            "refractory_period": self.refractory_period,
            # Add defaults for optional attributes
            "refractory_counter": getattr(self, "refractory_counter", 0),
            "is_active": getattr(self, "is_active", False),
            "properties": getattr(self, "properties", {})
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Neuron':
        """Create neuron from dictionary representation.
        
        Args:
            data: Dictionary with neuron properties
            
        Returns:
            Neuron instance
        """
        neuron = cls(
            neuron_id=data["id"],
            cortical_id=data["cortical_id"],
            position=data["position"],
            threshold=data["threshold"],
            membrane_potential=data["membrane_potential"],
            resting_potential=data["resting_potential"],
            decay_rate=data["decay_rate"],
            refractory_period=data["refractory_period"]
        )
        neuron.refractory_counter = data.get("refractory_counter", 0)
        neuron.is_active = data.get("is_active", False)
        
        return neuron 