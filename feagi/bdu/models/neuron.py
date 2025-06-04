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

"""Neuron model implementation optimized for GPU processing, SIMD, and Rust.

This module provides a high-performance implementation of neuron storage 
using Structure of Arrays (SoA) format compatible with SIMD, GPU acceleration,
and Rust backend processing. This is the unified neuron array implementation
that replaces GlobalNeuronArray for maximum performance.
"""

import numpy as np
from typing import Dict, List, Tuple, Any, Optional, Union, Set
import torch
import logging
from feagi.bdu.models.array_backend import ArrayBackend, BackendType

# Try to import Rust backend
try:
    from feagi_rust import create_gna
    RUST_AVAILABLE = True
except (ImportError, AttributeError):
    RUST_AVAILABLE = False

logger = logging.getLogger(__name__)

# Ensure 64-byte alignment for AVX-512 compatibility (512 bits = 64 bytes)
MEMORY_ALIGNMENT = 64

class NeuronArray:
    """GPU-optimized neuron storage using contiguous memory arrays with SIMD and Rust support.
    
    This implementation uses an array backend (NumPy, PyTorch, CuPy, WebGPU, or Rust)
    for neuron property storage, which can be efficiently processed with SIMD 
    or transferred to GPU memory for parallel computation.
    
    Instead of storing individual Neuron objects, we store properties in columnar
    format for vectorized operations. This unified implementation replaces
    GlobalNeuronArray for maximum performance.
    """
    
    def __init__(self, max_neurons: int = 10_000_000, backend: Optional[str] = None):
        """Initialize the NeuronArray with arrays for neuron properties.
        
        Args:
            max_neurons: Maximum number of neurons to support
            backend: Backend type to use (numpy, pytorch, cupy, webgpu, rust, or auto)
        """
        self.max_neurons = max_neurons
        
        # Import SIMD detection for alignment
        try:
            from feagi.utils.simd_detection import get_simd_detector
            simd_detector = get_simd_detector()
            self.alignment = simd_detector.get_memory_alignment()
            self.vector_width = simd_detector.capabilities.vector_width
            
            # Align capacity to SIMD vector boundaries for optimal performance
            self.aligned_capacity = simd_detector.get_aligned_size(max_neurons)
        except ImportError:
            self.alignment = 64  # Default to 64-byte alignment for AVX-512
            self.vector_width = 16  # 16 float32 values in AVX-512
            self.aligned_capacity = (max_neurons + 15) & ~15  # 16-element alignment
        
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
                self._init_numpy_backend(backend)
        else:
            self._use_rust = False
            self._init_numpy_backend(backend)
        
        # Common tracking regardless of backend
        self.id_to_index_map: Dict[int, int] = {}
        self.index_to_id_map: Dict[int, int] = {}
        self.cortical_id_to_indices: Dict[int, List[int]] = {}
        self.next_index = 0
        self.free_indices: Set[int] = set()
        self.neuron_count = 0
        
    def _init_numpy_backend(self, backend: Optional[str]):
        """Initialize NumPy/GPU backend with SIMD optimization."""
        # Set backend
        self.backend = ArrayBackend(backend)
        self.backend_type = self.backend.backend_type
        
        # Initialize storage arrays with SIMD-aligned memory
        self.membrane_potentials = self._create_aligned_array(self.aligned_capacity, "float32")
        self.resting_potentials = self._create_aligned_array(self.aligned_capacity, "float32")
        self.thresholds = self.backend.full((self.aligned_capacity,), 1.0, dtype=np.float32)
        self.decay_rates = self.backend.full((self.aligned_capacity,), 0.95, dtype=np.float32)
        self.refractory_periods = self.backend.full((self.aligned_capacity,), 1, dtype=np.int32)
        self.refractory_counters = self._create_aligned_array(self.aligned_capacity, "int32")
        
        # Coordinate arrays - SIMD-optimized SoA layout (explicitly uint32)
        self.coordinates_x = self.backend.zeros((self.aligned_capacity,), dtype=np.uint32)
        self.coordinates_y = self.backend.zeros((self.aligned_capacity,), dtype=np.uint32)
        self.coordinates_z = self.backend.zeros((self.aligned_capacity,), dtype=np.uint32)
        
        # Area mapping and activation
        self.cortical_idxs = self._create_aligned_array(self.aligned_capacity, "int32")
        self.is_active = self._create_aligned_array(self.aligned_capacity, "bool")
        
        # Valid mask (indicates which elements correspond to active neurons)
        self.valid_mask = self._create_aligned_array(self.aligned_capacity, "bool")
        
        # Additional optimization arrays
        self.last_fired = self._create_aligned_array(self.aligned_capacity, "int32")
        self.neuron_types = self._create_aligned_array(self.aligned_capacity, "int32")
        self.enabled_flags = self.backend.full((self.aligned_capacity,), 1, dtype=np.int32)
        
        # Working arrays for SIMD vectorized operations
        self._temp_potentials = self._create_aligned_array(self.aligned_capacity, "float32")
        self._temp_mask = self._create_aligned_array(self.aligned_capacity, "bool")
        
        # Device tracking for GPU support
        self.device = getattr(self.backend, 'device', 'cpu')
        
        logger.info(f"Initialized NeuronArray with {self.backend_type} backend, "
                   f"capacity: {self.aligned_capacity}, alignment: {self.alignment}")
    
    def _create_aligned_array(self, size: int, dtype: str) -> Any:
        """Create a SIMD-aligned array for optimal performance.
        
        Args:
            size: Number of elements
            dtype: Data type string
            
        Returns:
            Aligned array using the current backend
        """
        # Create array using backend
        array = self.backend.zeros(size, dtype=dtype)
        
        # For NumPy backend, try to ensure SIMD alignment
        if self.backend_type == "numpy":
            np_array = self.backend.to_numpy(array)
            
            # Check if we have proper alignment
            element_size = np_array.dtype.itemsize
            total_bytes = size * element_size
            aligned_bytes = (total_bytes + self.alignment - 1) & ~(self.alignment - 1)
            
            # If alignment is off, create a new aligned array
            if np_array.ctypes.data % self.alignment != 0:
                # Create oversized array and slice to get alignment
                oversized = np.zeros(aligned_bytes // element_size + self.alignment, dtype=np_array.dtype)
                offset = self.alignment - (oversized.ctypes.data % self.alignment)
                if offset == self.alignment:
                    offset = 0
                
                aligned_start = offset // element_size
                aligned_array = oversized[aligned_start:aligned_start + size]
                
                # Verify alignment achieved
                if aligned_array.ctypes.data % self.alignment == 0:
                    array = self.backend.array(aligned_array)
                # If still not aligned, keep original (still functional, just not optimized)
        
        return array

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
            chunk_size = max(self.vector_width * 8, 64)  # Process 8 SIMD vectors at a time
            
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

        # Assign a unique ID
        neuron_id = self.next_index
        self.next_index += 1

        # Get the next available index
        idx = self.next_index - 1

        # Record the mapping
        self.id_to_index_map[neuron_id] = idx
        if cortical_idx not in self.cortical_id_to_indices:
            self.cortical_id_to_indices[cortical_idx] = []
        self.cortical_id_to_indices[cortical_idx].append(idx)

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

    def update_membrane_potentials(self, synapse_indices, synapse_data):
        """Update membrane potentials based on firing neurons and incoming connections.
        
        Args:
            synapse_indices: Indices of firing neurons (FCL)
            synapse_data: Sparse matrix of incoming connections
            
        Returns:
            Array of indices that exceed their threshold and fire
        """
        # We need to ensure we're working with the right data structures
        if isinstance(synapse_data, torch.Tensor):
            device = synapse_data.device
            is_torch = True
        else:
            is_torch = False
            device = "cpu" if not hasattr(self, 'device') else self.device

        # Get the mask of valid neurons
        valid = self.valid_mask
        
        # Only consider neurons not in refractory period
        can_update_mask = valid & (self.refractory_counters <= 0)
        
        # For neurons in refractory period, decrement counters
        in_refractory = valid & (self.refractory_counters > 0)
        if is_torch:
            self.refractory_counters[in_refractory] -= 1
        else:
            np.subtract(self.refractory_counters, 1, out=self.refractory_counters, where=in_refractory)
        
        # Update membrane potentials for valid neurons
        if isinstance(synapse_data, torch.Tensor):
            # GPU implementation using PyTorch
            # This would need a specialized CUDA kernel or specialized PyTorch ops
            # For now, we just simulate it without true vectorization
            for idx in synapse_indices:
                if idx >= len(self.membrane_potentials):
                    continue
                
                # Get all post-synaptic targets and weights
                targets = torch.nonzero(synapse_data[idx]).squeeze()
                if targets.dim() == 0 and targets.numel() > 0:
                    # Handle single target case
                    targets = targets.unsqueeze(0)
                
                if targets.numel() == 0:
                    continue
                
                weights = synapse_data[idx, targets]
                
                # Add weights to membrane potentials of targets
                # Only update neurons not in refractory period
                update_mask = can_update_mask[targets]
                if update_mask.any():
                    self.membrane_potentials[targets[update_mask]] += weights[update_mask]
        else:
            # CPU implementation using sparse matrices
            # Get all post-synaptic connections from firing neurons
            for idx in synapse_indices:
                if idx >= synapse_data.shape[0]:
                    continue
                
                # Get the row for this neuron
                row = synapse_data.getrow(idx)
                
                # Get the targets and weights
                targets = row.indices
                weights = row.data
                
                if len(targets) == 0:
                    continue
                
                # Only update neurons not in refractory period
                update_mask = can_update_mask[targets]
                if np.any(update_mask):
                    self.membrane_potentials[targets[update_mask]] += weights[update_mask]
        
        # Check which neurons exceed threshold and should fire
        # Only consider neurons not already in refractory period
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
            self.membrane_potentials[valid_not_fired] = (
                self.membrane_potentials[valid_not_fired] * (1.0 - self.decay_rates[valid_not_fired]) + 
                self.resting_potentials[valid_not_fired] * self.decay_rates[valid_not_fired]
            )
        else:
            decay_effect = self.membrane_potentials[valid_not_fired] * (1.0 - self.decay_rates[valid_not_fired])
            rest_effect = self.resting_potentials[valid_not_fired] * self.decay_rates[valid_not_fired]
            self.membrane_potentials[valid_not_fired] = decay_effect + rest_effect
        
        # Return indices of fired neurons
        return np.where(fired)[0]

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
        
        # Generate neuron IDs (in this case, same as indices for simplicity)
        neuron_ids = list(indices.copy())
        
        # Prepare property arrays (convert to arrays if they are single values)
        # Use backend-specific arrays
        if not isinstance(thresholds, list) and not isinstance(thresholds, np.ndarray):
            thresholds = self.backend.array([float(thresholds)] * num_neurons)
        else:
            thresholds = self.backend.array(thresholds)
        
        if not isinstance(membrane_potentials, list) and not isinstance(membrane_potentials, np.ndarray):
            membrane_potentials = self.backend.array([float(membrane_potentials)] * num_neurons)
        else:
            membrane_potentials = self.backend.array(membrane_potentials)
        
        if not isinstance(resting_potentials, list) and not isinstance(resting_potentials, np.ndarray):
            resting_potentials = self.backend.array([float(resting_potentials)] * num_neurons)
        else:
            resting_potentials = self.backend.array(resting_potentials)
        
        if not isinstance(decay_rates, list) and not isinstance(decay_rates, np.ndarray):
            decay_rates = self.backend.array([float(decay_rates)] * num_neurons)
        else:
            decay_rates = self.backend.array(decay_rates)
        
        if not isinstance(refractory_periods, list) and not isinstance(refractory_periods, np.ndarray):
            refractory_periods = self.backend.array([int(refractory_periods)] * num_neurons)
        else:
            refractory_periods = self.backend.array(refractory_periods)
        
        # Extract position components as uint32
        coordinates_x = self.backend.array([np.uint32(max(0, pos[0])) for pos in positions])
        coordinates_y = self.backend.array([np.uint32(max(0, pos[1])) for pos in positions])
        coordinates_z = self.backend.array([np.uint32(max(0, pos[2])) for pos in positions])
        
        # Get numpy array of indices for indexing
        idx_array = indices
        
        # Convert to tensors if using PyTorch
        if isinstance(self.membrane_potentials, torch.Tensor):
            idx_tensor = torch.tensor(idx_array, dtype=torch.long)
            
            # Determine the dtype of the target tensors
            target_dtype = self.membrane_potentials.dtype
            
            # Use PyTorch indexing for tensors - ensure matching dtypes
            self.membrane_potentials.index_copy_(0, idx_tensor, 
                                               membrane_potentials.to(dtype=target_dtype))
            self.resting_potentials.index_copy_(0, idx_tensor, 
                                              resting_potentials.to(dtype=target_dtype))
            self.thresholds.index_copy_(0, idx_tensor, 
                                      thresholds.to(dtype=target_dtype))
            self.decay_rates.index_copy_(0, idx_tensor, 
                                       decay_rates.to(dtype=target_dtype))
            # Convert integer tensors to the appropriate type
            self.refractory_periods.index_copy_(0, idx_tensor, 
                                              refractory_periods.to(dtype=self.refractory_periods.dtype))
            self.coordinates_x.index_copy_(0, idx_tensor, 
                                       coordinates_x.to(dtype=self.coordinates_x.dtype))
            self.coordinates_y.index_copy_(0, idx_tensor, 
                                       coordinates_y.to(dtype=self.coordinates_y.dtype))
            self.coordinates_z.index_copy_(0, idx_tensor, 
                                       coordinates_z.to(dtype=self.coordinates_z.dtype))
            self.cortical_idxs.index_copy_(0, idx_tensor, 
                                        torch.full_like(idx_tensor, cortical_idx, 
                                                     dtype=self.cortical_idxs.dtype))
            
            # Set valid mask
            valid_mask = self.backend.to_numpy(self.valid_mask)
            valid_mask[idx_array] = True
            self.valid_mask = self.backend.array(valid_mask)
        else:
            # For NumPy arrays, use standard indexing
            self.membrane_potentials[idx_array] = membrane_potentials
            self.resting_potentials[idx_array] = resting_potentials
            self.thresholds[idx_array] = thresholds
            self.decay_rates[idx_array] = decay_rates
            self.refractory_periods[idx_array] = refractory_periods
            self.coordinates_x[idx_array] = coordinates_x
            self.coordinates_y[idx_array] = coordinates_y
            self.coordinates_z[idx_array] = coordinates_z
            self.cortical_idxs[idx_array] = cortical_idx
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