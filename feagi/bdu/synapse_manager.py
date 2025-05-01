"""
Synapse Manager for FEAGI.

This module provides a high-performance, memory-efficient data structure
designed to store and manage synapses in a large-scale spiking neural network.
It supports sparse synaptic connectivity, plasticity modeling, and
efficient operations for CPU and GPU execution.
"""

import logging
import numpy as np
import threading
import scipy.sparse as sp
from typing import Dict, List, Tuple, Optional, Set, Union, Any

logger = logging.getLogger(__name__)


class SynapseManager:
    """
    Manages synapse storage and operations using compressed sparse representations.
    
    This class uses compressed sparse row/column (CSR/CSC) format to efficiently
    store synaptic connections between neurons, optimizing for both memory usage
    and computational efficiency.
    
    Features:
    - Sparse Synapse Representation: Uses efficient storage for large networks
    - Plastic & Non-Plastic Synapses: Distinguishes between synapse types
    - Memory Efficient: Stores plasticity attributes only for plastic synapses
    - Thread-safe: Provides locking mechanisms for concurrent access
    """
    
    def __init__(self, max_neurons: int, max_synapses_per_neuron: int = 1000):
        """
        Initialize the SynapseManager.
        
        Args:
            max_neurons: Maximum number of neurons that can have synapses
            max_synapses_per_neuron: Maximum number of synapses per neuron
        """
        self.max_neurons = max_neurons
        self.max_synapses_per_neuron = max_synapses_per_neuron
        
        # Lock for thread-safe operations
        self._synapse_lock = threading.RLock()
        
        # Initialize sparse matrices for synapse storage
        self._init_synapse_storage()
        
        # Counters for statistics
        self.total_synapses = 0
        self.plastic_synapses = 0
    
    def _init_synapse_storage(self):
        """Initialize storage for synapses using sparse matrices."""
        # Weights matrix (sparse) - stores connection strengths
        self.weights = sp.lil_matrix((self.max_neurons, self.max_neurons), dtype=np.float32)
        
        # Plasticity-related matrices (only allocated for plastic synapses)
        self.is_plastic = sp.lil_matrix((self.max_neurons, self.max_neurons), dtype=bool)
        self.plasticity_coeffs = sp.lil_matrix((self.max_neurons, self.max_neurons), dtype=np.float32)
        self.plasticity_decay = sp.lil_matrix((self.max_neurons, self.max_neurons), dtype=np.float32)
        
        # Additional plasticity parameters (new)
        self.plasticity_type = sp.lil_matrix((self.max_neurons, self.max_neurons), dtype=np.uint8)
        self.activity_factor = sp.lil_matrix((self.max_neurons, self.max_neurons), dtype=np.float32)
        self.scaling_exponent = sp.lil_matrix((self.max_neurons, self.max_neurons), dtype=np.float32)
        
        # Adjacency lists for quick lookups
        self.outgoing_synapses = [[] for _ in range(self.max_neurons)]  # Pre -> Post
        self.incoming_synapses = [[] for _ in range(self.max_neurons)]  # Post <- Pre
        
        # Counter for synapses per neuron
        self.synapse_count = np.zeros(self.max_neurons, dtype=np.int32)
        
        logger.info(f"Initialized synapse storage for up to {self.max_neurons} neurons " 
                   f"with max {self.max_synapses_per_neuron} synapses per neuron")
    
    def add_synapse(self, pre_neuron: int, post_neuron: int, weight: float, 
                   is_plastic: bool = False, plasticity_coeff: float = 0.0,
                   plasticity_decay: float = 0.0, plasticity_type: int = 0,
                   activity_factor: float = 1.0, scaling_exponent: float = 1.0) -> bool:
        """
        Add a synapse between two neurons.
        
        Args:
            pre_neuron: ID of the presynaptic neuron
            post_neuron: ID of the postsynaptic neuron
            weight: Synaptic weight
            is_plastic: Whether the synapse exhibits plasticity
            plasticity_coeff: Coefficient for plasticity updates
            plasticity_decay: Decay rate for plasticity effects
            plasticity_type: Type of plasticity (0: None, 1: STP, 2: LTP/LTD)
            activity_factor: Multiplier for synaptic activity
            scaling_exponent: Nonlinear scaling factor for plasticity
            
        Returns:
            True if the synapse was added successfully, False otherwise
        """
        if pre_neuron >= self.max_neurons or post_neuron >= self.max_neurons:
            logger.error(f"Cannot create synapse: neuron ID exceeds max neurons ({self.max_neurons})")
            return False
        
        if self.synapse_count[pre_neuron] >= self.max_synapses_per_neuron:
            logger.warning(f"Neuron {pre_neuron} has reached maximum synapse count")
            return False
        
        with self._synapse_lock:
            # Check if synapse already exists
            if post_neuron in self.outgoing_synapses[pre_neuron]:
                # Update existing synapse
                self.weights[pre_neuron, post_neuron] = weight
                
                # Handle plasticity changes
                was_plastic = bool(self.is_plastic[pre_neuron, post_neuron])
                
                if is_plastic:
                    self.is_plastic[pre_neuron, post_neuron] = True
                    self.plasticity_coeffs[pre_neuron, post_neuron] = plasticity_coeff
                    self.plasticity_decay[pre_neuron, post_neuron] = plasticity_decay
                    self.plasticity_type[pre_neuron, post_neuron] = plasticity_type
                    self.activity_factor[pre_neuron, post_neuron] = activity_factor
                    self.scaling_exponent[pre_neuron, post_neuron] = scaling_exponent
                    
                    if not was_plastic:
                        self.plastic_synapses += 1
                elif was_plastic:
                    self.is_plastic[pre_neuron, post_neuron] = False
                    self.plastic_synapses -= 1
                
                logger.debug(f"Updated synapse: {pre_neuron} -> {post_neuron}, weight={weight}")
                return True
            
            # Add new synapse
            self.weights[pre_neuron, post_neuron] = weight
            self.outgoing_synapses[pre_neuron].append(post_neuron)
            self.incoming_synapses[post_neuron].append(pre_neuron)
            self.synapse_count[pre_neuron] += 1
            self.total_synapses += 1
            
            # Handle plastic synapse attributes
            if is_plastic:
                self.is_plastic[pre_neuron, post_neuron] = True
                self.plasticity_coeffs[pre_neuron, post_neuron] = plasticity_coeff
                self.plasticity_decay[pre_neuron, post_neuron] = plasticity_decay
                self.plasticity_type[pre_neuron, post_neuron] = plasticity_type
                self.activity_factor[pre_neuron, post_neuron] = activity_factor
                self.scaling_exponent[pre_neuron, post_neuron] = scaling_exponent
                self.plastic_synapses += 1
            
            logger.debug(f"Added synapse: {pre_neuron} -> {post_neuron}, weight={weight}")
            return True
    
    def remove_synapse(self, pre_neuron: int, post_neuron: int) -> bool:
        """
        Remove a synapse between two neurons.
        
        Args:
            pre_neuron: ID of the presynaptic neuron
            post_neuron: ID of the postsynaptic neuron
            
        Returns:
            True if the synapse was removed, False if it didn't exist
        """
        if pre_neuron >= self.max_neurons or post_neuron >= self.max_neurons:
            return False
        
        with self._synapse_lock:
            # Check if synapse exists
            if post_neuron not in self.outgoing_synapses[pre_neuron]:
                return False
            
            # Remove synapse
            self.weights[pre_neuron, post_neuron] = 0
            self.outgoing_synapses[pre_neuron].remove(post_neuron)
            self.incoming_synapses[post_neuron].remove(pre_neuron)
            self.synapse_count[pre_neuron] -= 1
            self.total_synapses -= 1
            
            # Handle plastic synapse cleanup
            if self.is_plastic[pre_neuron, post_neuron]:
                self.is_plastic[pre_neuron, post_neuron] = False
                self.plasticity_coeffs[pre_neuron, post_neuron] = 0
                self.plasticity_decay[pre_neuron, post_neuron] = 0
                self.plasticity_type[pre_neuron, post_neuron] = 0
                self.activity_factor[pre_neuron, post_neuron] = 0
                self.scaling_exponent[pre_neuron, post_neuron] = 0
                self.plastic_synapses -= 1
            
            logger.debug(f"Removed synapse: {pre_neuron} -> {post_neuron}")
            return True
    
    def get_synapse_weight(self, pre_neuron: int, post_neuron: int) -> float:
        """
        Get the weight of a synapse between two neurons.
        
        Args:
            pre_neuron: ID of the presynaptic neuron
            post_neuron: ID of the postsynaptic neuron
            
        Returns:
            Synaptic weight, or 0 if the synapse doesn't exist
        """
        if pre_neuron >= self.max_neurons or post_neuron >= self.max_neurons:
            return 0.0
        
        return self.weights[pre_neuron, post_neuron]
    
    def get_outgoing_synapses(self, neuron_id: int) -> List[Tuple[int, float]]:
        """
        Get all outgoing synapses for a neuron.
        
        Args:
            neuron_id: ID of the neuron
            
        Returns:
            List of (target_neuron_id, weight) tuples
        """
        if neuron_id >= self.max_neurons:
            return []
        
        return [(post_id, self.weights[neuron_id, post_id]) 
                for post_id in self.outgoing_synapses[neuron_id]]
    
    def get_incoming_synapses(self, neuron_id: int) -> List[Tuple[int, float]]:
        """
        Get all incoming synapses for a neuron.
        
        Args:
            neuron_id: ID of the neuron
            
        Returns:
            List of (source_neuron_id, weight) tuples
        """
        if neuron_id >= self.max_neurons:
            return []
        
        return [(pre_id, self.weights[pre_id, neuron_id]) 
                for pre_id in self.incoming_synapses[neuron_id]]
    
    def update_synapse_weight(self, pre_neuron: int, post_neuron: int, new_weight: float) -> bool:
        """
        Update the weight of an existing synapse.
        
        Args:
            pre_neuron: ID of the presynaptic neuron
            post_neuron: ID of the postsynaptic neuron
            new_weight: New synaptic weight
            
        Returns:
            True if the synapse was updated, False if it doesn't exist
        """
        if pre_neuron >= self.max_neurons or post_neuron >= self.max_neurons:
            return False
        
        with self._synapse_lock:
            if post_neuron not in self.outgoing_synapses[pre_neuron]:
                return False
            
            self.weights[pre_neuron, post_neuron] = new_weight
            return True
    
    def get_synapse_count(self) -> int:
        """Get the total number of synapses."""
        return self.total_synapses
    
    def get_plastic_synapse_count(self) -> int:
        """Get the number of plastic synapses."""
        return self.plastic_synapses
    
    def compute_synaptic_input(self, firing_neurons: List[int]) -> np.ndarray:
        """
        Compute synaptic input for all neurons based on firing neurons.
        
        Args:
            firing_neurons: List of neuron IDs that are firing
            
        Returns:
            Array of synaptic inputs for all neurons
        """
        if not firing_neurons:
            return np.zeros(self.max_neurons, dtype=np.float32)
        
        inputs = np.zeros(self.max_neurons, dtype=np.float32)
        
        # For each firing neuron, add its outgoing weights to the inputs
        for pre_id in firing_neurons:
            if pre_id >= self.max_neurons:
                continue
                
            for post_id in self.outgoing_synapses[pre_id]:
                inputs[post_id] += self.weights[pre_id, post_id]
        
        return inputs
    
    def compute_synaptic_input_matrix(self, firing_neurons: np.ndarray) -> np.ndarray:
        """
        Matrix-based computation of synaptic input for all neurons.
        
        This method uses sparse matrix operations for efficiency.
        
        Args:
            firing_neurons: Boolean array indicating which neurons are firing
            
        Returns:
            Array of synaptic inputs for all neurons
        """
        # Convert weights to CSR format for efficient row slicing
        weights_csr = self.weights.tocsr()
        
        # Sum the weights from all firing neurons
        firing_mask = np.zeros(self.max_neurons, dtype=np.float32)
        firing_mask[firing_neurons] = 1.0
        
        # Multiply firing_mask by weights matrix to get input currents
        inputs = weights_csr.T.dot(firing_mask)
        
        return inputs
    
    def optimize_storage(self):
        """
        Optimize storage format for operations.
        
        This can be called after adding many synapses to improve performance.
        """
        # Convert to CSR for efficient matrix operations
        self.weights = self.weights.tocsr()
        self.is_plastic = self.is_plastic.tocsr()
        self.plasticity_coeffs = self.plasticity_coeffs.tocsr()
        self.plasticity_decay = self.plasticity_decay.tocsr()
        self.plasticity_type = self.plasticity_type.tocsr()
        self.activity_factor = self.activity_factor.tocsr()
        self.scaling_exponent = self.scaling_exponent.tocsr()
        
        logger.info("Optimized synapse storage format")

    # New methods below this point
    
    def prune_weak_synapses(self, threshold: float = 0.1) -> int:
        """
        Remove synapses with weights below the threshold.
        
        Args:
            threshold: Minimum weight to keep a synapse
            
        Returns:
            Number of synapses pruned
        """
        pruned_count = 0
        
        with self._synapse_lock:
            # Convert to COO format for iteration
            weights_coo = self.weights.tocoo()
            
            for i, j, w in zip(weights_coo.row, weights_coo.col, weights_coo.data):
                if w < threshold:
                    if j in self.outgoing_synapses[i]:
                        self.remove_synapse(i, j)
                        pruned_count += 1
        
        logger.info(f"Pruned {pruned_count} weak synapses below threshold {threshold}")
        return pruned_count
    
    def update_plasticity(self, dt: float = 1.0) -> None:
        """
        Update all plastic synapses based on their plasticity rules.
        
        Args:
            dt: Time step for the update
        """
        # Get plastic synapse data in COO format for iteration
        plastic_coo = self.is_plastic.tocoo()
        
        if plastic_coo.nnz == 0:  # No plastic synapses
            return
            
        with self._synapse_lock:
            # Process updates for all plastic synapses
            for i, j in zip(plastic_coo.row, plastic_coo.col):
                if not self.is_plastic[i, j]:
                    continue  # Skip if not plastic
                    
                # Get plasticity parameters
                p_type = self.plasticity_type[i, j]
                coeff = self.plasticity_coeffs[i, j]
                decay = self.plasticity_decay[i, j]
                activity = self.activity_factor[i, j]
                exponent = self.scaling_exponent[i, j]
                current_weight = self.weights[i, j]
                
                # Apply plasticity rule based on type
                if p_type == 1:  # STP (Short-Term Plasticity)
                    # Multiplicative update
                    new_weight = current_weight * (coeff ** exponent) * activity * (decay ** dt)
                elif p_type == 2:  # LTP/LTD (Long-Term Potentiation/Depression)
                    # Additive update
                    new_weight = current_weight + (coeff ** exponent) * activity * (decay ** dt) * current_weight
                else:
                    continue  # Skip unknown plasticity type
                
                # Apply bounds
                new_weight = max(0.0, min(255.0, new_weight))
                
                # Update weight
                self.weights[i, j] = new_weight
        
        logger.debug(f"Updated {plastic_coo.nnz} plastic synapses")
    
    def finalize_synapses(self) -> None:
        """
        Finalize synapse structures after adding all synapses.
        
        This optimizes the data structures for efficient access during simulation.
        """
        self.optimize_storage()
        logger.info(f"Finalized {self.total_synapses} synapses ({self.plastic_synapses} plastic)")
    
    def get_synapse_info(self, pre_neuron: int, post_neuron: int) -> Dict[str, Any]:
        """
        Get detailed information about a specific synapse.
        
        Args:
            pre_neuron: ID of the presynaptic neuron
            post_neuron: ID of the postsynaptic neuron
            
        Returns:
            Dictionary with synapse properties, or empty dict if synapse doesn't exist
        """
        if pre_neuron >= self.max_neurons or post_neuron >= self.max_neurons:
            return {}
            
        if post_neuron not in self.outgoing_synapses[pre_neuron]:
            return {}
            
        info = {
            "weight": float(self.weights[pre_neuron, post_neuron]),
            "is_plastic": bool(self.is_plastic[pre_neuron, post_neuron])
        }
        
        if info["is_plastic"]:
            # Use exact float values by converting to Python float
            info.update({
                "plasticity_type": int(self.plasticity_type[pre_neuron, post_neuron]),
                "plasticity_coeff": float(self.plasticity_coeffs[pre_neuron, post_neuron]),
                "plasticity_decay": float(self.plasticity_decay[pre_neuron, post_neuron]),
                "activity_factor": float(self.activity_factor[pre_neuron, post_neuron]),
                "scaling_exponent": float(self.scaling_exponent[pre_neuron, post_neuron])
            })
            
        return info
    
    def resize(self, new_max_neurons: int) -> bool:
        """
        Resize the synapse manager to accommodate more neurons.
        
        Args:
            new_max_neurons: New maximum number of neurons
            
        Returns:
            True if resizing was successful, False otherwise
        """
        if new_max_neurons <= self.max_neurons:
            logger.warning(f"New size {new_max_neurons} is not larger than current size {self.max_neurons}")
            return False
            
        with self._synapse_lock:
            # Create new matrices with larger size
            new_weights = sp.lil_matrix((new_max_neurons, new_max_neurons), dtype=np.float32)
            new_is_plastic = sp.lil_matrix((new_max_neurons, new_max_neurons), dtype=bool)
            new_plasticity_coeffs = sp.lil_matrix((new_max_neurons, new_max_neurons), dtype=np.float32)
            new_plasticity_decay = sp.lil_matrix((new_max_neurons, new_max_neurons), dtype=np.float32)
            new_plasticity_type = sp.lil_matrix((new_max_neurons, new_max_neurons), dtype=np.uint8)
            new_activity_factor = sp.lil_matrix((new_max_neurons, new_max_neurons), dtype=np.float32)
            new_scaling_exponent = sp.lil_matrix((new_max_neurons, new_max_neurons), dtype=np.float32)
            
            # Copy existing data
            new_weights[:self.max_neurons, :self.max_neurons] = self.weights
            new_is_plastic[:self.max_neurons, :self.max_neurons] = self.is_plastic
            new_plasticity_coeffs[:self.max_neurons, :self.max_neurons] = self.plasticity_coeffs
            new_plasticity_decay[:self.max_neurons, :self.max_neurons] = self.plasticity_decay
            new_plasticity_type[:self.max_neurons, :self.max_neurons] = self.plasticity_type
            new_activity_factor[:self.max_neurons, :self.max_neurons] = self.activity_factor
            new_scaling_exponent[:self.max_neurons, :self.max_neurons] = self.scaling_exponent
            
            # Extend adjacency lists
            new_outgoing_synapses = list(self.outgoing_synapses)
            new_outgoing_synapses.extend([[] for _ in range(new_max_neurons - self.max_neurons)])
            
            new_incoming_synapses = list(self.incoming_synapses)
            new_incoming_synapses.extend([[] for _ in range(new_max_neurons - self.max_neurons)])
            
            # Extend synapse count array
            new_synapse_count = np.zeros(new_max_neurons, dtype=np.int32)
            new_synapse_count[:self.max_neurons] = self.synapse_count
            
            # Update instance variables
            self.weights = new_weights
            self.is_plastic = new_is_plastic
            self.plasticity_coeffs = new_plasticity_coeffs
            self.plasticity_decay = new_plasticity_decay
            self.plasticity_type = new_plasticity_type
            self.activity_factor = new_activity_factor
            self.scaling_exponent = new_scaling_exponent
            self.outgoing_synapses = new_outgoing_synapses
            self.incoming_synapses = new_incoming_synapses
            self.synapse_count = new_synapse_count
            self.max_neurons = new_max_neurons
            
            logger.info(f"Resized synapse storage from {self.max_neurons} to {new_max_neurons} neurons")
            return True 