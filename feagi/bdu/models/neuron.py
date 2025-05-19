"""Neuron model implementation optimized for GPU processing.

This module provides a high-performance implementation of neuron storage 
using Structure of Arrays (SoA) format compatible with SIMD and GPU acceleration.
"""

import numpy as np
from typing import Dict, List, Tuple, Any, Optional, Union
import torch
import logging
from feagi.bdu.models.array_backend import ArrayBackend, BackendType

logger = logging.getLogger(__name__)

# Ensure 64-byte alignment for AVX-512 compatibility (512 bits = 64 bytes)
MEMORY_ALIGNMENT = 64

class NeuronArray:
    """GPU-optimized neuron storage using contiguous memory arrays.
    
    This implementation uses an array backend (NumPy, PyTorch, CuPy, or WebGPU)
    for neuron property storage, which can be efficiently processed with SIMD 
    or transferred to GPU memory for parallel computation.
    
    Instead of storing individual Neuron objects, we store properties in columnar
    format for vectorized operations.
    """
    
    def __init__(self, max_neurons: int = 10_000_000, backend: Union[str, BackendType] = "auto"):
        """Initialize neuron storage using contiguous arrays.
        
        Args:
            max_neurons: Maximum number of neurons to allocate memory for
            backend: Array backend to use (numpy, pytorch, cupy, webgpu, or auto)
        """
        self.max_neurons = max_neurons
        
        # Ensure neuron count is aligned to MEMORY_ALIGNMENT-byte boundary for SIMD
        # For float32, which is 4 bytes, we need to ensure the array size is divisible by 16
        # (MEMORY_ALIGNMENT / 4) to maintain 64-byte alignment
        aligned_size = ((max_neurons + (MEMORY_ALIGNMENT // 4) - 1) // (MEMORY_ALIGNMENT // 4)) * (MEMORY_ALIGNMENT // 4)
        self.aligned_size = aligned_size
        
        # Initialize array backend
        self.backend = ArrayBackend(backend)
        logger.info(f"Using {self.backend.backend_type.value} backend for NeuronArray")
        
        # Core neuron properties as contiguous arrays
        # Using float32 for most values as it's optimal for GPU computation
        self.membrane_potentials = self.backend.zeros((aligned_size,), dtype=np.float32)
        self.resting_potentials = self.backend.zeros((aligned_size,), dtype=np.float32)
        self.thresholds = self.backend.ones((aligned_size,), dtype=np.float32)
        self.decay_rates = self.backend.full((aligned_size,), 0.5, dtype=np.float32)
        self.refractory_periods = self.backend.ones((aligned_size,), dtype=np.int32)
        self.refractory_counters = self.backend.zeros((aligned_size,), dtype=np.int32)
        
        # Boolean values should use native boolean type for the backend
        self.is_active = self.backend.zeros((aligned_size,), dtype=np.bool_)
        
        # Area membership
        self.area_ids = self.backend.zeros((aligned_size,), dtype=np.int32)
        
        # Position tracking (3D coordinates)
        # Stored as separate arrays for better memory access patterns in GPU
        self.positions_x = self.backend.zeros((aligned_size,), dtype=np.int32)
        self.positions_y = self.backend.zeros((aligned_size,), dtype=np.int32)
        self.positions_z = self.backend.zeros((aligned_size,), dtype=np.int32)
        
        # Neuron validity mask (1 if neuron exists at this index, 0 if not)
        self.valid_mask = self.backend.zeros((aligned_size,), dtype=np.bool_)
        
        # Keep track of allocated neurons
        self.next_index = 0
        self.free_indices = []  # Indices that can be reused
        
        # Mapping from neuron_id to index in the arrays
        self.id_to_index_map = {}
        
        # Device where arrays are stored (depends on backend)
        self.device = "cpu"  # Will be updated if arrays are moved to GPU

    def to_gpu(self):
        """Transfer neuron arrays to GPU for accelerated computation."""
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
            self.area_ids = self.backend.to_device(self.area_ids)
            self.positions_x = self.backend.to_device(self.positions_x)
            self.positions_y = self.backend.to_device(self.positions_y)
            self.positions_z = self.backend.to_device(self.positions_z)
            
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
            self.area_ids = self.backend.to_cpu(self.area_ids)
            self.positions_x = self.backend.to_cpu(self.positions_x)
            self.positions_y = self.backend.to_cpu(self.positions_y)
            self.positions_z = self.backend.to_cpu(self.positions_z)
            
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
            if self.next_index >= self.max_neurons:
                raise ValueError(f"Maximum neuron capacity ({self.max_neurons}) reached")
            index = self.next_index
            self.next_index += 1
            
        # Mark this index as valid and store the ID mapping
        # Convert to numpy arrays first if they're backend-specific types
        # This ensures we can use boolean indexing
        valid_mask_np = self.backend.to_numpy(self.valid_mask)
        valid_mask_np[index] = True
        self.valid_mask = self.backend.array(valid_mask_np)
        
        self.id_to_index_map[neuron_id] = index
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
        
        # Mark as invalid and reset values
        valid_mask_np = self.backend.to_numpy(self.valid_mask)
        valid_mask_np[index] = False
        self.valid_mask = self.backend.array(valid_mask_np)
        
        # Add to free indices for reuse
        self.free_indices.append(index)
        
        # Remove from ID mapping
        del self.id_to_index_map[neuron_id]
        
        return True

    def create_neuron(self, area_id: int, position: Tuple[int, int, int],
                     threshold: float = 1.0, membrane_potential: float = 0.0,
                     resting_potential: float = 0.0, decay_rate: float = 0.5,
                     refractory_period: int = 1) -> int:
        """Create a new neuron with the given properties.
        
        Args:
            area_id: ID of the cortical area
            position: 3D coordinates within the cortical area (x, y, z)
            threshold: Firing threshold potential
            membrane_potential: Initial membrane potential
            resting_potential: Base membrane potential
            decay_rate: Rate at which potential decays each timestep
            refractory_period: Number of timesteps after firing during which the neuron cannot fire
            
        Returns:
            Unique ID of the created neuron
        """
        # Find the next available index
        if self.free_indices:
            index = self.free_indices.pop()
        else:
            index = self.next_index
            self.next_index += 1
            
            if index >= self.max_neurons:
                raise ValueError(f"Maximum number of neurons ({self.max_neurons}) exceeded")
        
        # Generate a unique ID
        neuron_id = index  # In this simple implementation, we use the index as the ID
        
        # Store neuron properties
        self.membrane_potentials[index] = membrane_potential
        self.resting_potentials[index] = resting_potential
        self.thresholds[index] = threshold
        self.decay_rates[index] = decay_rate
        self.refractory_periods[index] = refractory_period
        self.refractory_counters[index] = 0
        self.is_active[index] = False
        
        # Store area and position
        self.area_ids[index] = area_id
        self.positions_x[index] = position[0]
        self.positions_y[index] = position[1]
        self.positions_z[index] = position[2]
        
        # Mark as valid
        self.valid_mask[index] = True
        
        # Map ID to index
        self.id_to_index_map[neuron_id] = index
        
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
        elif property_name == "area_id":
            return int(self.area_ids[index])
        elif property_name == "position":
            return (int(self.positions_x[index]), 
                    int(self.positions_y[index]), 
                    int(self.positions_z[index]))
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
        elif property_name == "area_id":
            self.area_ids[index] = int(value)
        elif property_name == "position":
            if not isinstance(value, tuple) or len(value) != 3:
                raise ValueError("Position must be a tuple of (x, y, z)")
            self.positions_x[index] = int(value[0])
            self.positions_y[index] = int(value[1])
            self.positions_z[index] = int(value[2])
        elif property_name == "is_active":
            self.is_active[index] = bool(value)
        else:
            raise KeyError(f"Property {property_name} not found")

    def get_neurons_by_area(self, area_id: int) -> List[int]:
        """Get all neurons in a specific cortical area.
        
        Args:
            area_id: ID of the cortical area
            
        Returns:
            List of neuron IDs in the area
        """
        # Find neurons with matching area_id and valid mask
        indices = np.where((self.area_ids == area_id) & self.valid_mask)[0]
        
        # Convert indices to neuron IDs
        neuron_ids = []
        for index in indices:
            for neuron_id, idx in self.id_to_index_map.items():
                if idx == index:
                    neuron_ids.append(neuron_id)
                    break
        
        return neuron_ids

    def get_neuron_count(self) -> int:
        """Get the total number of neurons.
        
        Returns:
            Total number of neurons
        """
        return np.sum(self.valid_mask)

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
            "area_id": int(self.area_ids[index]),
            "position": (int(self.positions_x[index]), 
                         int(self.positions_y[index]), 
                         int(self.positions_z[index])),
            "threshold": float(self.thresholds[index]),
            "membrane_potential": float(self.membrane_potentials[index]),
            "resting_potential": float(self.resting_potentials[index]),
            "decay_rate": float(self.decay_rates[index]),
            "refractory_period": int(self.refractory_periods[index]),
            "refractory_counter": int(self.refractory_counters[index]),
            "is_active": bool(self.is_active[index])
        }

    def batch_create_neurons(self, area_id: int, positions: List[Tuple[int, int, int]],
                         thresholds: Union[float, List[float]] = 1.0,
                         membrane_potentials: Union[float, List[float]] = 0.0,
                         resting_potentials: Union[float, List[float]] = 0.0,
                         decay_rates: Union[float, List[float]] = 0.5,
                         refractory_periods: Union[int, List[int]] = 1) -> List[int]:
        """Create multiple neurons with the same or different properties in batch.
        
        This method is optimized for creating large numbers of neurons at once by
        using vectorized operations instead of loops.
        
        Args:
            area_id: ID of the cortical area (int)
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
            indices = np.array(self.free_indices[:num_neurons])
            self.free_indices = self.free_indices[num_neurons:]
        else:
            # Use free indices + new indices
            num_new_indices = num_neurons - len(self.free_indices)
            
            if self.next_index + num_new_indices > self.max_neurons:
                raise ValueError(f"Maximum number of neurons ({self.max_neurons}) exceeded")
            
            free_indices = np.array(self.free_indices) if self.free_indices else np.array([], dtype=np.int32)
            new_indices = np.arange(self.next_index, self.next_index + num_new_indices, dtype=np.int32)
            indices = np.concatenate([free_indices, new_indices])
            
            self.free_indices = []
            self.next_index += num_new_indices
        
        # Generate neuron IDs (in this case, same as indices for simplicity)
        neuron_ids = indices.copy()
        
        # Prepare property arrays (convert to arrays if they are single values)
        if not isinstance(thresholds, list):
            thresholds = np.full(num_neurons, thresholds, dtype=np.float32)
        else:
            thresholds = np.array(thresholds, dtype=np.float32)
        
        if not isinstance(membrane_potentials, list):
            membrane_potentials = np.full(num_neurons, membrane_potentials, dtype=np.float32)
        else:
            membrane_potentials = np.array(membrane_potentials, dtype=np.float32)
        
        if not isinstance(resting_potentials, list):
            resting_potentials = np.full(num_neurons, resting_potentials, dtype=np.float32)
        else:
            resting_potentials = np.array(resting_potentials, dtype=np.float32)
        
        if not isinstance(decay_rates, list):
            decay_rates = np.full(num_neurons, decay_rates, dtype=np.float32)
        else:
            decay_rates = np.array(decay_rates, dtype=np.float32)
        
        if not isinstance(refractory_periods, list):
            refractory_periods = np.full(num_neurons, refractory_periods, dtype=np.int32)
        else:
            refractory_periods = np.array(refractory_periods, dtype=np.int32)
        
        # Extract position components
        positions_x = np.array([pos[0] for pos in positions], dtype=np.int32)
        positions_y = np.array([pos[1] for pos in positions], dtype=np.int32)
        positions_z = np.array([pos[2] for pos in positions], dtype=np.int32)
        
        # Set neuron properties using vectorized operations
        self.membrane_potentials[indices] = membrane_potentials
        self.resting_potentials[indices] = resting_potentials
        self.thresholds[indices] = thresholds
        self.decay_rates[indices] = decay_rates
        self.refractory_periods[indices] = refractory_periods
        self.refractory_counters[indices] = 0
        self.is_active[indices] = False
        
        # Set area and positions
        self.area_ids[indices] = area_id
        self.positions_x[indices] = positions_x
        self.positions_y[indices] = positions_y
        self.positions_z[indices] = positions_z
        
        # Mark as valid
        self.valid_mask[indices] = True
        
        # Update ID to index mapping
        for neuron_id, index in zip(neuron_ids, indices):
            self.id_to_index_map[int(neuron_id)] = int(index)
        
        return neuron_ids.tolist()

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
            int_arrays_count = 4    # refractory_periods, refractory_counters, area_ids
            bool_arrays_count = 2   # is_active, valid_mask
            positions_count = 3     # positions_x, positions_y, positions_z
            
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
            self.area_ids,
            self.positions_x,
            self.positions_y,
            self.positions_z
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
    """Legacy Neuron class for API compatibility.
    
    This class provides a thin wrapper around neuron properties stored in the
    NeuronArray class. It's meant for API compatibility only and should not be
    used for internal storage of neurons.
    """
    
    def __init__(self, 
                neuron_id: int,
                area_id: str,
                position: Tuple[int, int, int],
                threshold: float = 1.0,
                membrane_potential: float = 0.0,
                resting_potential: float = 0.0,
                decay_rate: float = 0.5,
                refractory_period: int = 1,
                properties: Optional[Dict[str, Any]] = None):
        """Initialize a neuron object.
        
        This is used primarily for API serialization and should not be used
        for internal storage of neurons.
        
        Args:
            neuron_id: Unique identifier for this neuron
            area_id: ID of the cortical area
            position: 3D coordinates within the cortical area (x, y, z)
            threshold: Firing threshold potential
            membrane_potential: Initial membrane potential
            resting_potential: Base membrane potential
            decay_rate: Rate at which potential decays each timestep
            refractory_period: Number of timesteps after firing during which the neuron cannot fire
            properties: Additional properties for the neuron (optional)
        """
        self.id = neuron_id
        self.area_id = area_id
        self.position = position
        self.threshold = threshold
        self.membrane_potential = membrane_potential
        self.resting_potential = resting_potential
        self.decay_rate = decay_rate
        self.refractory_period = refractory_period
        self.refractory_counter = 0
        self.is_active = False
        self.properties = properties or {}

    def to_dict(self) -> Dict[str, Any]:
        """Convert neuron to dictionary representation.
        
        Returns:
            Dictionary with neuron properties
        """
        return {
            "id": self.id,
            "area_id": self.area_id,
            "position": self.position,
            "threshold": self.threshold,
            "membrane_potential": self.membrane_potential,
            "resting_potential": self.resting_potential,
            "decay_rate": self.decay_rate,
            "refractory_period": self.refractory_period,
            "refractory_counter": self.refractory_counter,
            "is_active": self.is_active,
            "properties": self.properties
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
            area_id=data["area_id"],
            position=data["position"],
            threshold=data["threshold"],
            membrane_potential=data["membrane_potential"],
            resting_potential=data["resting_potential"],
            decay_rate=data["decay_rate"],
            refractory_period=data["refractory_period"],
            properties=data.get("properties", {})
        )
        neuron.refractory_counter = data.get("refractory_counter", 0)
        neuron.is_active = data.get("is_active", False)
        
        return neuron 