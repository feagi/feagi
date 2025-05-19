"""ConnectomeManager for the BDU optimized for GPU processing.

This module provides a GPU-optimized implementation of the connectome manager
using NumPy arrays and sparse matrices for efficient data processing and
transfer to GPU memory.
"""

import logging
import numpy as np
from enum import Enum
from typing import Dict, Any, List, Tuple, Optional, Set, Union
import uuid
from scipy import sparse
import os
import torch

# Import models
from feagi.bdu.models.neuron import NeuronArray
from feagi.bdu.models.cortical_area import CorticalArea
# Import utility functions
from feagi.bdu.utils.position import (
    linearize_position,
    delinearize_position,
    validate_position
)

logger = logging.getLogger(__name__)


class NeuronPropertyType(Enum):
    """Types of neuron properties that can be accessed/modified."""
    
    MEMBRANE_POTENTIAL = "membrane_potential"
    RESTING_POTENTIAL = "resting_potential"
    THRESHOLD = "threshold"
    REFRACTORY_PERIOD = "refractory_period"
    DECAY_RATE = "decay_rate"
    AREA_ID = "area_id"
    POSITION = "position"
    FIRING = "firing"


class ConnectomeManagerGPU:
    """GPU-optimized manager for creating and manipulating the neural connectome.
    
    This implementation uses NumPy arrays and sparse matrices for efficient 
    data processing and transfer to GPU memory.
    """
    
    def __init__(self, config_or_max_neurons=10_000_000, max_synapses=100_000_000):
        """Initialize the ConnectomeManager with GPU-optimized data structures.
        
        Args:
            config_or_max_neurons: Either a FeagiConfig object or the maximum number of neurons
            max_synapses: Maximum number of synapses (only used if first parameter is an integer)
        """
        # Handle either a config object or direct integers
        if hasattr(config_or_max_neurons, 'get'):
            # This is a FeagiConfig object
            self.max_neurons = config_or_max_neurons.get('connectome.max_neurons', 10_000_000)
            self.max_synapses = config_or_max_neurons.get('connectome.max_synapses_per_neuron', 10) * self.max_neurons
            fcl_window_size = config_or_max_neurons.get('connectome.fcl_window_size', 20)
        else:
            # This is a direct integer
            self.max_neurons = config_or_max_neurons
            self.max_synapses = max_synapses
            fcl_window_size = 20
        
        # Initialize neuron storage using NeuronArray for SIMD/GPU optimization
        self.neuron_array = NeuronArray(max_neurons=self.max_neurons)
        
        # Cortical area storage (still dictionary-based as areas are fewer in number)
        self.cortical_areas: Dict[str, CorticalArea] = {}
        
        # Mapping from area_id to neuron indices - use NumPy arrays for efficiency
        # This is a mapping of area_id (string) to a boolean mask of length max_neurons
        self.area_neuron_masks: Dict[str, np.ndarray] = {}
        
        # Brain region storage
        self.brain_regions: Dict[str, Dict[str, Any]] = {}
        
        # Region to area mapping - use sets for small collections
        self.region_area_map: Dict[str, Set[str]] = {}
        
        # Connectivity rules storage
        self.connectivity_rules: Dict[str, Dict[str, Any]] = {}
        
        # Cortical connections storage
        self.cortical_connections: Dict[str, Dict[str, Any]] = {}
        
        # Synapse storage using sparse matrices
        self._init_synapse_storage()
        
        # Track neuron ID assignment
        self.next_neuron_id = 0
        
        # Mapping from neuron_id (int) to index in the NeuronArray
        # This is needed for backward compatibility with existing API
        self.neuron_id_to_index: Dict[int, int] = {}
        self.index_to_neuron_id: Dict[int, int] = {}
        
        # For tracking active neurons
        self.active_neurons = np.zeros(self.max_neurons, dtype=np.bool_)
        
        # Simulation state
        self.current_timestep = 0
        
        # Initialize FCL manager
        # Import here to avoid circular imports
        from feagi.npu.fcl_manager import FCLManager
        self.fcl_manager = FCLManager(window_size=fcl_window_size)
        
        # For test compatibility
        self._neuron_to_position = {}
        self.is_initialized = True
        
        logger.info(f"Initialized GPU-optimized ConnectomeManager with capacity for {self.max_neurons} neurons "
                   f"and {self.max_synapses} synapses")
    
    #----------------------------------------------------------------------
    # Synapse Storage Methods
    #----------------------------------------------------------------------
    
    def _init_synapse_storage(self):
        """Initialize the sparse matrix storage for synapses."""
        # Always use LIL for construction phase for consistent behavior
        self.outgoing_matrix = sparse.lil_matrix((self.max_neurons, self.max_neurons), dtype=np.float32)
        self.incoming_matrix = sparse.lil_matrix((self.max_neurons, self.max_neurons), dtype=np.float32)
    
    def _ensure_csr_format_outgoing(self):
        """Ensure outgoing matrix is in CSR format for efficient row access."""
        if not isinstance(self.outgoing_matrix, sparse.csr_matrix):
            self.outgoing_matrix = self.outgoing_matrix.tocsr()
    
    def _ensure_csc_format_incoming(self):
        """Ensure incoming matrix is in CSC format for efficient column access."""
        if not isinstance(self.incoming_matrix, sparse.csc_matrix):
            self.incoming_matrix = self.incoming_matrix.tocsc()
            
    def _convert_to_lil_if_needed(self):
        """Convert matrices to LIL format if needed for modifications."""
        if not isinstance(self.outgoing_matrix, sparse.lil_matrix):
            self.outgoing_matrix = self.outgoing_matrix.tolil()
        if not isinstance(self.incoming_matrix, sparse.lil_matrix):
            self.incoming_matrix = self.incoming_matrix.tolil()
    
    #----------------------------------------------------------------------
    # Neuron CRUD Operations
    #----------------------------------------------------------------------
    
    def create_neuron(self, area_id: str, position: Tuple[int, int, int], 
                     threshold: float = 1.0, membrane_potential: float = 0.0,
                     resting_potential: float = 0.0, decay_rate: float = 0.5,
                     refractory_period: int = 1, properties: Optional[Dict[str, Any]] = None) -> int:
        """Create a new neuron in the specified cortical area.
        
        Args:
            area_id: ID of the cortical area
            position: 3D coordinates within the cortical area (x, y, z)
            threshold: Firing threshold potential
            membrane_potential: Initial membrane potential
            resting_potential: Base membrane potential
            decay_rate: Rate at which potential decays each timestep
            refractory_period: Number of timesteps after firing during which the neuron cannot fire
            properties: Additional properties for the neuron (optional)
            
        Returns:
            Unique ID of the created neuron
            
        Raises:
            ValueError: If the area_id doesn't exist
            ValueError: If the position is outside the area's boundaries
        """
        # Validate area exists
        if area_id not in self.cortical_areas:
            raise ValueError(f"Cortical area {area_id} does not exist")
        
        area = self.cortical_areas[area_id]
        
        # Validate position
        if not area.contains_position(position):
            raise ValueError(f"Position {position} is outside the bounds of area {area.name}")
        
        # Get next available neuron ID
        neuron_id = self.next_neuron_id
        self.next_neuron_id += 1
        
        # Create the neuron in the NeuronArray
        # Convert string area_id to integer for the array using a hash function
        # that will fit within int32 range
        area_id_int = hash(area_id) & 0x7FFFFFFF  # Ensure positive and within int32 range
        
        # Get next available index in the NeuronArray
        index = self.neuron_array.create_neuron(
            area_id=area_id_int,
            position=position,
            threshold=threshold,
            membrane_potential=membrane_potential,
            resting_potential=resting_potential,
            decay_rate=decay_rate,
            refractory_period=refractory_period
        )
        
        # Map neuron_id to index
        self.neuron_id_to_index[neuron_id] = index
        self.index_to_neuron_id[index] = neuron_id
        
        # Update area tracking
        if area_id not in self.area_neuron_masks:
            self.area_neuron_masks[area_id] = np.zeros(self.max_neurons, dtype=np.bool_)
        self.area_neuron_masks[area_id][index] = True
        
        # Update _neuron_to_position for test compatibility 
        # Format matches test expectation: (area_id, x, y, z, neuron_index)
        self._neuron_to_position[neuron_id] = (area_id, *position, index)
        
        # Add to cortical area
        area.add_neuron(neuron_id, position)
        
        logger.debug(f"Created neuron {neuron_id} in area {area.name} at position {position}")
        return neuron_id
    
    def get_neuron(self, neuron_id: int) -> Dict[str, Any]:
        """Get information about a specific neuron.
        
        Args:
            neuron_id: ID of the neuron
            
        Returns:
            Dictionary with neuron properties
            
        Raises:
            KeyError: If the neuron_id doesn't exist
        """
        if neuron_id not in self.neuron_id_to_index:
            raise KeyError(f"Neuron {neuron_id} does not exist")
        
        index = self.neuron_id_to_index[neuron_id]
        
        # Convert neuron array data to dictionary
        position = (
            int(self.neuron_array.positions_x[index]),
            int(self.neuron_array.positions_y[index]),
            int(self.neuron_array.positions_z[index])
        )
        
        # Find area_id from the area_id_int
        area_id_int = int(self.neuron_array.area_ids[index])
        # We need to map this back to the string area_id
        # For now, we'll have to search through the area masks
        area_id = None
        for area_str, mask in self.area_neuron_masks.items():
            if mask[index]:
                area_id = area_str
                break
        
        result = {
            "area_id": area_id,
            "position": position,
            "threshold": float(self.neuron_array.thresholds[index]),
            "membrane_potential": float(self.neuron_array.membrane_potentials[index]),
            "resting_potential": float(self.neuron_array.resting_potentials[index]),
            "decay_rate": float(self.neuron_array.decay_rates[index]),
            "refractory_period": int(self.neuron_array.refractory_periods[index]),
            "refractory_counter": int(self.neuron_array.refractory_counters[index]),
            "properties": {}  # We don't store additional properties in the optimized version
        }
        
        return result
    
    def get_neuron_property(self, neuron_id: int, property_name: Union[str, NeuronPropertyType]) -> Any:
        """Get a specific property of a neuron.
        
        Args:
            neuron_id: ID of the neuron
            property_name: Name or enum of the property to get
            
        Returns:
            Value of the property
            
        Raises:
            KeyError: If the neuron_id doesn't exist
            KeyError: If the property doesn't exist
        """
        if neuron_id not in self.neuron_id_to_index:
            raise KeyError(f"Neuron {neuron_id} does not exist")
        
        index = self.neuron_id_to_index[neuron_id]
        
        # Handle property name as either string or enum
        if isinstance(property_name, NeuronPropertyType):
            property_name = property_name.value
        
        return self.neuron_array.get_neuron_property(neuron_id, property_name)
    
    def set_neuron_property(self, neuron_id: int, property_name: Union[str, NeuronPropertyType], value: Any) -> None:
        """Set a specific property of a neuron.
        
        Args:
            neuron_id: ID of the neuron
            property_name: Name or enum of the property to set
            value: New value for the property
            
        Raises:
            KeyError: If the neuron_id doesn't exist
            KeyError: If the property doesn't exist
        """
        if neuron_id not in self.neuron_id_to_index:
            raise KeyError(f"Neuron {neuron_id} does not exist")
        
        index = self.neuron_id_to_index[neuron_id]
        
        # Handle property name as either string or enum
        if isinstance(property_name, NeuronPropertyType):
            property_name = property_name.value
        
        self.neuron_array.set_neuron_property(neuron_id, property_name, value)
    
    def get_neurons_by_area(self, area_id: str) -> List[int]:
        """Get all neurons in a specific cortical area.
        
        Args:
            area_id: ID of the cortical area
            
        Returns:
            List of neuron IDs in the area
            
        Raises:
            KeyError: If the area_id doesn't exist
        """
        if area_id not in self.cortical_areas:
            raise KeyError(f"Cortical area {area_id} does not exist")
        
        if area_id not in self.area_neuron_masks:
            return []
        
        # Get the mask for this area
        mask = self.area_neuron_masks[area_id]
        
        # Get the indices where the mask is True
        indices = np.where(mask)[0]
        
        # Convert indices to neuron_ids
        neuron_ids = [self.index_to_neuron_id[idx] for idx in indices]
        
        return neuron_ids
    
    def get_area_for_neuron(self, neuron_id: int) -> str:
        """Get the ID of the cortical area containing a neuron.
        
        Args:
            neuron_id: ID of the neuron
            
        Returns:
            ID of the cortical area
            
        Raises:
            KeyError: If the neuron_id doesn't exist
        """
        if neuron_id not in self.neuron_id_to_index:
            raise KeyError(f"Neuron {neuron_id} does not exist")
        
        index = self.neuron_id_to_index[neuron_id]
        
        # Find the area_id by checking the masks
        for area_id, mask in self.area_neuron_masks.items():
            if mask[index]:
                return area_id
        
        # This should not happen if the neuron exists
        raise RuntimeError(f"Neuron {neuron_id} exists but is not assigned to any area")
    
    def get_neuron_count(self) -> int:
        """Get the total number of neurons in the connectome.
        
        Returns:
            Number of neurons
        """
        return self.neuron_array.get_neuron_count()
    
    def delete_neuron(self, neuron_id: int) -> None:
        """Delete a neuron from the connectome.
        
        Args:
            neuron_id: ID of the neuron to delete
            
        Raises:
            KeyError: If the neuron_id doesn't exist
        """
        if neuron_id not in self.neuron_id_to_index:
            raise KeyError(f"Neuron {neuron_id} does not exist")
        
        index = self.neuron_id_to_index[neuron_id]
        
        # Get the area_id before deleting the neuron
        area_id = self.get_area_for_neuron(neuron_id)
        
        # Delete the neuron from the area
        if area_id in self.cortical_areas:
            self.cortical_areas[area_id].remove_neuron(neuron_id)
        
        # Delete the neuron from the mask
        if area_id in self.area_neuron_masks:
            self.area_neuron_masks[area_id][index] = False
        
        # Remove from neuron array
        self.neuron_array.delete_neuron(neuron_id)
        
        # Delete from mappings
        del self.index_to_neuron_id[index]
        del self.neuron_id_to_index[neuron_id]
        
        # Remove from _neuron_to_position for test compatibility
        if neuron_id in self._neuron_to_position:
            del self._neuron_to_position[neuron_id]
        
        # Delete all synapses connected to this neuron
        self._convert_to_lil_if_needed()
        self.outgoing_matrix[index, :] = 0
        self.incoming_matrix[:, index] = 0
    
    def get_neuron_position(self, neuron_id: int) -> Tuple[int, int, int]:
        """Get the position of a neuron.
        
        Args:
            neuron_id: ID of the neuron
            
        Returns:
            3D coordinates of the neuron
            
        Raises:
            KeyError: If the neuron_id doesn't exist
        """
        if neuron_id not in self.neuron_id_to_index:
            raise KeyError(f"Neuron {neuron_id} does not exist")
        
        index = self.neuron_id_to_index[neuron_id]
        
        # Get position from neuron array
        return (
            int(self.neuron_array.positions_x[index]),
            int(self.neuron_array.positions_y[index]),
            int(self.neuron_array.positions_z[index])
        )
    
    #----------------------------------------------------------------------
    # Synapse Management Methods
    #----------------------------------------------------------------------
    
    def create_synapse(self, pre_neuron_id: int, post_neuron_id: int, weight: float, 
                      is_plastic: bool = False, plasticity_coeff: float = 0.0, 
                      plasticity_decay: float = 0.0, **kwargs) -> bool:
        """Create a synapse between two neurons.
        
        Args:
            pre_neuron_id: ID of the presynaptic neuron
            post_neuron_id: ID of the postsynaptic neuron
            weight: Synaptic weight
            is_plastic: Whether the synapse is plastic
            plasticity_coeff: Coefficient for plasticity
            plasticity_decay: Decay rate for plasticity
            **kwargs: Additional synapse properties
            
        Returns:
            True if the synapse was created, False if it already existed
            
        Raises:
            KeyError: If either neuron doesn't exist
        """
        # Check both neurons exist
        if pre_neuron_id not in self.neuron_id_to_index:
            raise KeyError(f"Pre-synaptic neuron {pre_neuron_id} does not exist")
        if post_neuron_id not in self.neuron_id_to_index:
            raise KeyError(f"Post-synaptic neuron {post_neuron_id} does not exist")
        
        # Get indices
        pre_idx = self.neuron_id_to_index[pre_neuron_id]
        post_idx = self.neuron_id_to_index[post_neuron_id]
        
        # Check if synapse already exists
        self._ensure_csr_format_outgoing()
        if self.outgoing_matrix[pre_idx, post_idx] != 0:
            return False
        
        # Convert matrices to LIL for modification
        self._convert_to_lil_if_needed()
        
        # Create synapse by setting weight
        self.outgoing_matrix[pre_idx, post_idx] = weight
        self.incoming_matrix[post_idx, pre_idx] = weight
        
        # Note: We don't store plasticity information in the basic implementation
        # In a more advanced implementation, we could use additional sparse matrices
        # for plasticity_coeff and plasticity_decay
        
        return True
    
    def batch_create_synapses(self, synapse_specs: List[Tuple[int, int, float]]) -> int:
        """Create multiple synapses in a batch operation.
        
        Args:
            synapse_specs: List of tuples (pre_neuron_id, post_neuron_id, weight)
            
        Returns:
            Number of synapses successfully created
        """
        # Convert matrices to LIL for efficient batch modification
        self._convert_to_lil_if_needed()
        
        created_count = 0
        
        # Process each synapse specification
        for pre_id, post_id, weight in synapse_specs:
            try:
                # Check both neurons exist
                if pre_id not in self.neuron_id_to_index or post_id not in self.neuron_id_to_index:
                    continue
                
                # Get indices
                pre_idx = self.neuron_id_to_index[pre_id]
                post_idx = self.neuron_id_to_index[post_id]
                
                # Skip if synapse already exists
                if self.outgoing_matrix[pre_idx, post_idx] != 0:
                    continue
                
                # Create synapse
                self.outgoing_matrix[pre_idx, post_idx] = weight
                self.incoming_matrix[post_idx, pre_idx] = weight
                
                created_count += 1
            except Exception as e:
                logger.error(f"Error creating synapse {pre_id} -> {post_id}: {e}")
        
        return created_count
    
    def remove_synapse(self, pre_neuron_id: int, post_neuron_id: int) -> bool:
        """Remove a synapse between two neurons.
        
        Args:
            pre_neuron_id: ID of the presynaptic neuron
            post_neuron_id: ID of the postsynaptic neuron
            
        Returns:
            True if the synapse was removed, False if it didn't exist
            
        Raises:
            KeyError: If either neuron doesn't exist
        """
        # Check both neurons exist
        if pre_neuron_id not in self.neuron_id_to_index:
            raise KeyError(f"Pre-synaptic neuron {pre_neuron_id} does not exist")
        if post_neuron_id not in self.neuron_id_to_index:
            raise KeyError(f"Post-synaptic neuron {post_neuron_id} does not exist")
        
        # Get indices
        pre_idx = self.neuron_id_to_index[pre_neuron_id]
        post_idx = self.neuron_id_to_index[post_neuron_id]
        
        # Check if synapse exists
        self._ensure_csr_format_outgoing()
        if self.outgoing_matrix[pre_idx, post_idx] == 0:
            return False
        
        # Convert matrices to LIL for modification
        self._convert_to_lil_if_needed()
        
        # Remove synapse by setting weight to zero
        self.outgoing_matrix[pre_idx, post_idx] = 0
        self.incoming_matrix[post_idx, pre_idx] = 0
        
        return True
    
    def get_synapse_weight(self, pre_neuron_id: int, post_neuron_id: int) -> float:
        """Get the weight of a synapse between two neurons.
        
        Args:
            pre_neuron_id: ID of the presynaptic neuron
            post_neuron_id: ID of the postsynaptic neuron
            
        Returns:
            Weight of the synapse, or 0.0 if it doesn't exist
            
        Raises:
            KeyError: If either neuron doesn't exist
        """
        # Check both neurons exist
        if pre_neuron_id not in self.neuron_id_to_index:
            raise KeyError(f"Pre-synaptic neuron {pre_neuron_id} does not exist")
        if post_neuron_id not in self.neuron_id_to_index:
            raise KeyError(f"Post-synaptic neuron {post_neuron_id} does not exist")
        
        # Get indices
        pre_idx = self.neuron_id_to_index[pre_neuron_id]
        post_idx = self.neuron_id_to_index[post_neuron_id]
        
        # Ensure CSR format for efficient row access
        self._ensure_csr_format_outgoing()
        
        # Get synapse weight
        return float(self.outgoing_matrix[pre_idx, post_idx])
    
    def update_synapse_weight(self, pre_neuron_id: int, post_neuron_id: int, new_weight: float) -> bool:
        """Update the weight of a synapse between two neurons.
        
        Args:
            pre_neuron_id: ID of the presynaptic neuron
            post_neuron_id: ID of the postsynaptic neuron
            new_weight: New weight for the synapse
            
        Returns:
            True if the synapse was updated, False if it didn't exist
            
        Raises:
            KeyError: If either neuron doesn't exist
        """
        # Check both neurons exist
        if pre_neuron_id not in self.neuron_id_to_index:
            raise KeyError(f"Pre-synaptic neuron {pre_neuron_id} does not exist")
        if post_neuron_id not in self.neuron_id_to_index:
            raise KeyError(f"Post-synaptic neuron {post_neuron_id} does not exist")
        
        # Get indices
        pre_idx = self.neuron_id_to_index[pre_neuron_id]
        post_idx = self.neuron_id_to_index[post_neuron_id]
        
        # Check if synapse exists
        self._ensure_csr_format_outgoing()
        if self.outgoing_matrix[pre_idx, post_idx] == 0:
            return False
        
        # Convert matrices to LIL for modification
        self._convert_to_lil_if_needed()
        
        # Update synapse weight
        self.outgoing_matrix[pre_idx, post_idx] = new_weight
        self.incoming_matrix[post_idx, pre_idx] = new_weight
        
        return True
    
    def get_outgoing_connections(self, neuron_id: int) -> List[Tuple[int, float]]:
        """Get all outgoing connections from a neuron.
        
        Args:
            neuron_id: ID of the neuron
            
        Returns:
            List of (target_neuron_id, weight) tuples
            
        Raises:
            KeyError: If the neuron doesn't exist
        """
        if neuron_id not in self.neuron_id_to_index:
            raise KeyError(f"Neuron {neuron_id} does not exist")
        
        # Get neuron index
        index = self.neuron_id_to_index[neuron_id]
        
        # Ensure CSR format for efficient row access
        self._ensure_csr_format_outgoing()
        
        # Get the row for this neuron
        row = self.outgoing_matrix.getrow(index)
        
        # Get the non-zero column indices and data
        col_indices, data = row.indices, row.data
        
        # Map column indices back to neuron IDs and build result
        result = []
        for col_idx, weight in zip(col_indices, data):
            if col_idx in self.index_to_neuron_id:
                target_id = self.index_to_neuron_id[col_idx]
                result.append((target_id, float(weight)))
        
        return result
    
    def get_incoming_connections(self, neuron_id: int) -> List[Tuple[int, float]]:
        """Get all incoming connections to a neuron.
        
        Args:
            neuron_id: ID of the neuron
            
        Returns:
            List of (source_neuron_id, weight) tuples
            
        Raises:
            KeyError: If the neuron doesn't exist
        """
        if neuron_id not in self.neuron_id_to_index:
            raise KeyError(f"Neuron {neuron_id} does not exist")
        
        # Get neuron index
        index = self.neuron_id_to_index[neuron_id]
        
        # Ensure CSC format for efficient column access
        self._ensure_csc_format_incoming()
        
        # Get the column for this neuron
        col = self.incoming_matrix.getcol(index)
        
        # Get the non-zero row indices and data
        row_indices, data = col.indices, col.data
        
        # Map row indices back to neuron IDs and build result
        result = []
        for row_idx, weight in zip(row_indices, data):
            if row_idx in self.index_to_neuron_id:
                source_id = self.index_to_neuron_id[row_idx]
                result.append((source_id, float(weight)))
        
        return result
    
    def get_synapse_count(self) -> int:
        """Get the total number of synapses in the connectome.
        
        Returns:
            Number of synapses
        """
        # Ensure matrix is in a format that provides efficient nnz count
        self._ensure_csr_format_outgoing()
        return self.outgoing_matrix.nnz
    
    def update_membrane_potentials(self, current_timestep=None) -> List[int]:
        """Update membrane potentials based on incoming signals.
        
        This GPU-optimized implementation uses sparse matrix operations and
        vectorized computations for maximum performance.
        
        Args:
            current_timestep: Current simulation timestep (optional)
            
        Returns:
            List of neuron IDs that fired
        """
        if current_timestep is not None:
            self.current_timestep = current_timestep
        
        # Ensure outgoing matrix is in CSR format for efficient row access
        self._ensure_csr_format_outgoing()
        
        # Get subset of outgoing matrix for active neurons
        # Convert active_neurons to boolean mask if it's a set
        if isinstance(self.active_neurons, set):
            active_mask = np.zeros(self.max_neurons, dtype=np.bool_)
            for nid in self.active_neurons:
                if nid in self.neuron_id_to_index:
                    idx = self.neuron_id_to_index[nid]
                    active_mask[idx] = True
            self.active_neurons = active_mask
        
        # If we have no active neurons, decay potentials and check for firing
        if not np.any(self.active_neurons):
            return self._update_without_firing()
        
        # Create a signal propagation matrix from active neurons
        # This uses the active_neurons mask to efficiently extract only relevant rows
        active_indices = np.where(self.active_neurons)[0]
        
        if len(active_indices) == 0:
            return self._update_without_firing()
        
        # Extract rows from outgoing matrix for active neurons
        # This creates a submatrix of only the connections from active neurons
        signal_matrix = self.outgoing_matrix[active_indices]
        
        # Reset active neurons for next timestep
        self.active_neurons.fill(False)
        
        # Process incoming signals using the GPU-optimized NeuronArray
        # This updates membrane potentials and returns a mask of neurons that fired
        fired_mask = self.neuron_array.process_incoming_signals(signal_matrix)
        
        # Decay membrane potentials for non-firing neurons
        non_fired_mask = ~fired_mask
        valid_neurons = self.neuron_array.valid_mask
        
        # Apply decay to valid neurons that didn't fire
        neurons_to_decay = non_fired_mask & valid_neurons
        
        if isinstance(self.neuron_array.membrane_potentials, torch.Tensor) and self.neuron_array.device == "cuda":
            # Use PyTorch operations if on GPU
            self.neuron_array.membrane_potentials[neurons_to_decay] = (
                self.neuron_array.membrane_potentials[neurons_to_decay] * 
                (1 - self.neuron_array.decay_rates[neurons_to_decay])
            )
        else:
            # Use NumPy operations if on CPU
            self.neuron_array.membrane_potentials[neurons_to_decay] *= (
                1 - self.neuron_array.decay_rates[neurons_to_decay]
            )
        
        # Decrement refractory counters
        refractory_mask = self.neuron_array.refractory_counters > 0
        self.neuron_array.refractory_counters[refractory_mask] -= 1
        
        # Update active_neurons for next timestep (neurons that fired)
        self.active_neurons = fired_mask & valid_neurons
        
        # Update FCL manager
        fired_indices = np.where(fired_mask)[0]
        fired_neuron_ids = [
            self.index_to_neuron_id.get(idx, idx) for idx in fired_indices
        ]
        
        self.fcl_manager.register_event(self.current_timestep, fired_neuron_ids)
        
        # Increment timestep
        self.current_timestep += 1
        
        return fired_neuron_ids
    
    def _update_without_firing(self) -> List[int]:
        """Update membrane potentials when no neurons are firing.
        
        This method:
        1. Decays membrane potentials
        2. Decrements refractory counters
        3. Checks for neurons that now exceed their threshold
        
        Returns:
            List of neuron IDs that fired
        """
        # Move to the next FCL window
        self.fcl_manager.advance_window(self.current_timestep)
        
        # Let the neuron array handle the update
        fired_indices = self.neuron_array.decay_and_check_firing()
        
        # Add fired neurons to the next FCL
        if len(fired_indices):
            self.fcl_manager.add_to_fcl(fired_indices, self.current_timestep + 1)
        
        # Convert fired indices to neuron IDs
        fired_neuron_ids = [self.index_to_neuron_id[idx] for idx in fired_indices if idx in self.index_to_neuron_id]
        
        # Increment timestep
        self.current_timestep += 1
        
        return fired_neuron_ids
    
    # More methods would follow for cortical areas, brain regions, etc.
    # For this initial implementation, we're focusing on neuron and synapse management

    #----------------------------------------------------------------------
    # Cortical Area Management Methods
    #----------------------------------------------------------------------
    
    def add_cortical_area(self, name: str, dimensions: Tuple[int, int, int],
                         position: Tuple[int, int, int], area_type: str = "custom",
                         properties: Optional[Dict[str, Any]] = None,
                         area_id: Optional[str] = None) -> str:
        """Add a new cortical area to the connectome.
        
        Args:
            name: Human-readable name for this area
            dimensions: 3D dimensions of the area (width, height, depth)
            position: 3D coordinates of the area's origin in the brain space
            area_type: Type of cortical area (e.g., "sensory", "motor", "custom")
            properties: Additional properties for the area (optional)
            area_id: Unique identifier for this area (optional, generated if not provided)
            
        Returns:
            Unique ID of the created cortical area
            
        Raises:
            ValueError: If an area with the same name already exists
        """
        # Check if an area with this name already exists
        for area in self.cortical_areas.values():
            if area.name == name:
                raise ValueError(f"Cortical area with name '{name}' already exists")
        
        # Create the cortical area
        area = CorticalArea(
            name=name,
            dimensions=dimensions,
            position=position,
            area_type=area_type,
            properties=properties or {},
            area_id=area_id
        )
        
        # Add to cortical areas dict
        self.cortical_areas[area.id] = area
        
        # Initialize area neuron mask
        self.area_neuron_masks[area.id] = np.zeros(self.max_neurons, dtype=np.bool_)
        
        logger.info(f"Added cortical area '{name}' with ID {area.id}")
        return area.id
        
    def get_cortical_area(self, area_id: str) -> CorticalArea:
        """Get a cortical area by its ID.
        
        Args:
            area_id: ID of the cortical area
            
        Returns:
            The cortical area object
            
        Raises:
            KeyError: If the area_id doesn't exist
        """
        if area_id not in self.cortical_areas:
            raise KeyError(f"Cortical area {area_id} does not exist")
        
        return self.cortical_areas[area_id]
    
    def get_cortical_area_by_name(self, name: str) -> Optional[CorticalArea]:
        """Get a cortical area by its name.
        
        Args:
            name: Name of the cortical area
            
        Returns:
            The cortical area object, or None if not found
        """
        for area in self.cortical_areas.values():
            if area.name == name:
                return area
        
        return None
    
    def get_cortical_area_names(self) -> List[str]:
        """Get the names of all cortical areas.
        
        Returns:
            List of area names
        """
        return [area.name for area in self.cortical_areas.values()]

    # Property to maintain backward compatibility with the existing API
    @property
    def _neuron_id_to_index(self):
        return self.neuron_id_to_index
        
    # Implement additional methods as needed for a full replacement of ConnectomeManager 

    def update_neuron_position(self, neuron_id: int, new_position: Tuple[int, int, int]) -> bool:
        """Update the position of a neuron within its cortical area.
        
        Args:
            neuron_id: ID of the neuron
            new_position: New 3D coordinates within the cortical area (x, y, z)
            
        Returns:
            True if the position was updated, False if the neuron doesn't exist
            
        Raises:
            ValueError: If the new position is outside the area's boundaries
        """
        # Check if neuron exists
        if neuron_id not in self.neuron_id_to_index:
            return False
        
        index = self.neuron_id_to_index[neuron_id]
        
        # Get the area ID using the hash 
        area_id_int = self.neuron_array.area_ids[index]
        
        # Find the original string area_id by matching with the hash
        area_id = None
        for a_id in self.cortical_areas:
            if hash(a_id) & 0x7FFFFFFF == area_id_int:
                area_id = a_id
                break
        
        if area_id is None:
            logger.error(f"Could not find area for neuron {neuron_id}")
            return False
        
        # Validate new position
        area = self.cortical_areas[area_id]
        if not area.contains_position(new_position):
            raise ValueError(f"Position {new_position} is outside the bounds of area {area.name}")
        
        # Update position in neuron array
        self.neuron_array.positions_x[index] = new_position[0]
        self.neuron_array.positions_y[index] = new_position[1]
        self.neuron_array.positions_z[index] = new_position[2]
        
        # Update position tracking
        if hasattr(self, '_neuron_to_position'):
            self._neuron_to_position[neuron_id] = new_position
        
        return True 

    def batch_create_neurons(self, area_id: str, positions: List[Tuple[int, int, int]],
                           threshold: float = 1.0, membrane_potential: float = 0.0,
                           resting_potential: float = 0.0, decay_rate: float = 0.5,
                           refractory_period: int = 1, properties: Optional[Dict[str, Any]] = None) -> List[int]:
        """Create multiple neurons in the specified cortical area in a batch operation.
        
        This method is optimized for creating large numbers of neurons at once using
        vectorized operations instead of loops.
        
        Args:
            area_id: ID of the cortical area
            positions: List of 3D coordinates within the cortical area
            threshold: Firing threshold potential (can be a single value or a list)
            membrane_potential: Initial membrane potential (can be a single value or a list)
            resting_potential: Base membrane potential (can be a single value or a list)
            decay_rate: Rate at which potential decays each timestep (can be a single value or a list)
            refractory_period: Refractory period (can be a single value or a list)
            properties: Additional properties for the neurons (optional)
            
        Returns:
            List of unique IDs of the created neurons
            
        Raises:
            ValueError: If the area_id doesn't exist
            ValueError: If any position is outside the area's boundaries
        """
        # Validate area exists
        if area_id not in self.cortical_areas:
            raise ValueError(f"Cortical area {area_id} does not exist")
        
        area = self.cortical_areas[area_id]
        
        # Validate positions
        for pos in positions:
            if not area.contains_position(pos):
                raise ValueError(f"Position {pos} is outside the bounds of area {area.name}")
        
        # Convert area_id to integer for storage in NeuronArray
        area_id_int = hash(area_id) & 0x7FFFFFFF  # Ensure positive and within int32 range
        
        # Create neurons in batch using NeuronArray's batch method
        neuron_ids = self.neuron_array.batch_create_neurons(
            area_id=area_id_int,
            positions=positions,
            thresholds=threshold,
            membrane_potentials=membrane_potential,
            resting_potentials=resting_potential,
            decay_rates=decay_rate,
            refractory_periods=refractory_period
        )
        
        # Update ID mapping for all created neurons
        for neuron_id in neuron_ids:
            self.neuron_id_to_index[neuron_id] = neuron_id  # In this implementation, ID = index
            self.index_to_neuron_id[neuron_id] = neuron_id
        
        # Update area tracking using boolean mask
        if area_id not in self.area_neuron_masks:
            self.area_neuron_masks[area_id] = np.zeros(self.max_neurons, dtype=np.bool_)
        
        # Set mask bits for new neurons
        for neuron_id in neuron_ids:
            self.area_neuron_masks[area_id][neuron_id] = True
        
        # Store additional properties if provided
        if properties:
            # Currently, additional properties are not stored in the GPU-optimized implementation
            # This would require additional arrays in NeuronArray for each property
            pass
        
        return neuron_ids 