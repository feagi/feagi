"""Neuron model implementation optimized for GPU processing.

This module provides a high-performance implementation of neuron storage 
using CSR (Compressed Sparse Row) format compatible with GPU acceleration.
"""

import numpy as np
from typing import Dict, List, Tuple, Any, Optional, Union
import torch

class NeuronArray:
    """GPU-optimized neuron storage using contiguous memory arrays.
    
    This implementation uses NumPy/PyTorch arrays for neuron property storage,
    which can be efficiently transferred to GPU memory for parallel computation.
    
    Instead of storing individual Neuron objects, we store properties in columnar
    format for vectorized operations.
    """
    
    def __init__(self, max_neurons: int = 10_000_000):
        """Initialize neuron storage using contiguous arrays.
        
        Args:
            max_neurons: Maximum number of neurons to allocate memory for
        """
        self.max_neurons = max_neurons
        
        # Core neuron properties as contiguous arrays
        # Using float32 for most values as it's optimal for GPU computation
        self.membrane_potentials = np.zeros(max_neurons, dtype=np.float32)
        self.resting_potentials = np.zeros(max_neurons, dtype=np.float32)
        self.thresholds = np.ones(max_neurons, dtype=np.float32)
        self.decay_rates = np.full(max_neurons, 0.5, dtype=np.float32)
        self.refractory_periods = np.ones(max_neurons, dtype=np.int32)
        self.refractory_counters = np.zeros(max_neurons, dtype=np.int32)
        self.is_active = np.zeros(max_neurons, dtype=np.bool_)
        
        # Area membership
        self.area_ids = np.zeros(max_neurons, dtype=np.int32)
        
        # Position tracking (3D coordinates)
        self.positions_x = np.zeros(max_neurons, dtype=np.int32)
        self.positions_y = np.zeros(max_neurons, dtype=np.int32)
        self.positions_z = np.zeros(max_neurons, dtype=np.int32)
        
        # Neuron validity mask (1 if neuron exists at this index, 0 if not)
        self.valid_mask = np.zeros(max_neurons, dtype=np.bool_)
        
        # Keep track of allocated neurons
        self.next_index = 0
        self.free_indices = []  # Indices that can be reused
        
        # Mapping from neuron_id to index in the arrays
        self.id_to_index_map = {}
        
        # Device where arrays are stored (CPU by default)
        self.device = "cpu"

    def to_gpu(self):
        """Transfer neuron arrays to GPU for accelerated computation."""
        if torch.cuda.is_available():
            self.device = "cuda"
            
            # Convert numpy arrays to torch tensors on GPU
            self.membrane_potentials = torch.from_numpy(self.membrane_potentials).to(self.device)
            self.resting_potentials = torch.from_numpy(self.resting_potentials).to(self.device)
            self.thresholds = torch.from_numpy(self.thresholds).to(self.device)
            self.decay_rates = torch.from_numpy(self.decay_rates).to(self.device)
            self.refractory_periods = torch.from_numpy(self.refractory_periods).to(self.device)
            self.refractory_counters = torch.from_numpy(self.refractory_counters).to(self.device)
            self.is_active = torch.from_numpy(self.is_active).to(self.device)
            self.valid_mask = torch.from_numpy(self.valid_mask).to(self.device)
            
            return True
        else:
            return False

    def to_cpu(self):
        """Transfer neuron arrays back to CPU."""
        if self.device != "cpu":
            # Convert torch tensors back to numpy arrays
            self.membrane_potentials = self.membrane_potentials.cpu().numpy()
            self.resting_potentials = self.resting_potentials.cpu().numpy()
            self.thresholds = self.thresholds.cpu().numpy()
            self.decay_rates = self.decay_rates.cpu().numpy()
            self.refractory_periods = self.refractory_periods.cpu().numpy()
            self.refractory_counters = self.refractory_counters.cpu().numpy()
            self.is_active = self.is_active.cpu().numpy()
            self.valid_mask = self.valid_mask.cpu().numpy()
            
            self.device = "cpu"
            return True
        return False

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

    def delete_neuron(self, neuron_id: int) -> bool:
        """Delete a neuron.
        
        Args:
            neuron_id: ID of the neuron to delete
            
        Returns:
            True if the neuron was deleted, False if it didn't exist
        """
        if neuron_id not in self.id_to_index_map:
            return False
        
        index = self.id_to_index_map[neuron_id]
        
        # Mark as invalid
        self.valid_mask[index] = False
        
        # Reset properties to default values
        self.membrane_potentials[index] = 0.0
        self.is_active[index] = False
        
        # Remove from ID map
        del self.id_to_index_map[neuron_id]
        
        # Add to free indices list for reuse
        self.free_indices.append(index)
        
        return True

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