"""
SIMD-Optimized Membrane Potential Operations for FEAGI NPU.

This module provides highly optimized membrane potential update algorithms
that leverage SIMD vectorization, efficient memory layouts, and cache-friendly
data access patterns for maximum performance on CPU and GPU.

@cursor:simd-optimized
@cursor:critical-path
@cursor:gpu-compatible
"""

import numpy as np
from typing import Tuple, Optional, Union, List
import warnings
from ..utils.simd_detection import get_simd_detector, get_backend_selector
from ..utils.simd_profiler import profile_simd_operation

# Suppress NumPy warnings for performance-critical code
warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)

class SIMDMembraneProcessor:
    """
    SIMD-optimized membrane potential processor.
    
    Uses vectorized operations, optimal memory layouts, and cache-friendly
    algorithms for maximum throughput on neural membrane potential updates.
    """
    
    def __init__(self, capacity: int, use_profiling: bool = False):
        """
        Initialize SIMD membrane processor.
        
        Args:
            capacity: Maximum number of neurons to support
            use_profiling: Enable performance profiling
        """
        self.capacity = capacity
        self.use_profiling = use_profiling
        
        # Get SIMD configuration
        self.simd_detector = get_simd_detector()
        self.backend_selector = get_backend_selector()
        self.simd_config = self.backend_selector.backend_config
        
        # Align capacity to SIMD boundaries
        self.aligned_capacity = self.simd_detector.get_aligned_size(capacity)
        self.vector_width = self.simd_config["vector_width"]
        self.alignment = self.simd_config["alignment"]
        
        # Pre-allocate aligned arrays
        self._allocate_arrays()
        
        # Performance statistics
        self.update_count = 0
        self.total_neurons_processed = 0
    
    def _allocate_arrays(self):
        """Allocate SIMD-aligned arrays for optimal performance."""
        
        # Use aligned allocation for better SIMD performance
        # Note: NumPy doesn't guarantee alignment, but we can try
        self.membrane_potentials = np.zeros(self.aligned_capacity, dtype=np.float32, order='C')
        self.thresholds = np.ones(self.aligned_capacity, dtype=np.float32, order='C')
        self.decay_rates = np.full(self.aligned_capacity, 0.95, dtype=np.float32, order='C')
        self.resting_potentials = np.zeros(self.aligned_capacity, dtype=np.float32, order='C')
        
        # Refractory and state tracking
        self.refractory_counters = np.zeros(self.aligned_capacity, dtype=np.int32, order='C')
        self.refractory_periods = np.ones(self.aligned_capacity, dtype=np.int32, order='C')
        self.fired_mask = np.zeros(self.aligned_capacity, dtype=np.bool_, order='C')
        self.active_mask = np.ones(self.aligned_capacity, dtype=np.bool_, order='C')
        
        # Working arrays for intermediate calculations
        self._temp_potentials = np.zeros(self.aligned_capacity, dtype=np.float32, order='C')
        self._temp_mask = np.zeros(self.aligned_capacity, dtype=np.bool_, order='C')
    
    def vectorized_membrane_update(self, 
                                 neuron_indices: np.ndarray,
                                 input_currents: np.ndarray) -> np.ndarray:
        """
        Perform vectorized membrane potential update with SIMD optimization.
        
        Args:
            neuron_indices: Indices of neurons to update
            input_currents: Input currents for each neuron
            
        Returns:
            Array of neuron indices that fired
        """
        
        if self.use_profiling:
            with profile_simd_operation("membrane_update", len(neuron_indices)):
                return self._vectorized_update_impl(neuron_indices, input_currents)
        else:
            return self._vectorized_update_impl(neuron_indices, input_currents)
    
    def _vectorized_update_impl(self, 
                              neuron_indices: np.ndarray, 
                              input_currents: np.ndarray) -> np.ndarray:
        """Core vectorized membrane update implementation."""
        
        # Ensure inputs are properly aligned
        if len(neuron_indices) != len(input_currents):
            raise ValueError("Neuron indices and currents must have same length")
        
        # Reset fired mask
        self.fired_mask.fill(False)
        
        # Step 1: Vectorized decay for all active neurons
        active_neurons = self.active_mask[:self.capacity]
        can_update = active_neurons & (self.refractory_counters[:self.capacity] <= 0)
        
        if np.any(can_update):
            # Vectorized decay operation
            self.membrane_potentials[:self.capacity][can_update] *= self.decay_rates[:self.capacity][can_update]
            
            # Vectorized drift towards resting potential
            potential_diff = (self.resting_potentials[:self.capacity][can_update] - 
                            self.membrane_potentials[:self.capacity][can_update])
            self.membrane_potentials[:self.capacity][can_update] += (
                potential_diff * (1.0 - self.decay_rates[:self.capacity][can_update])
            )
        
        # Step 2: Update refractory counters (vectorized)
        in_refractory = self.refractory_counters[:self.capacity] > 0
        if np.any(in_refractory):
            self.refractory_counters[:self.capacity][in_refractory] -= 1
        
        # Step 3: Apply input currents (vectorized scatter operation)
        if len(neuron_indices) > 0:
            # Clamp indices to valid range
            valid_indices = neuron_indices[neuron_indices < self.capacity]
            valid_currents = input_currents[:len(valid_indices)]
            
            # Vectorized current application
            can_receive = can_update[valid_indices]
            if np.any(can_receive):
                receiving_indices = valid_indices[can_receive]
                receiving_currents = valid_currents[can_receive]
                
                # Use numpy's advanced indexing for efficient scatter
                self.membrane_potentials[receiving_indices] += receiving_currents
        
        # Step 4: Vectorized threshold detection and firing
        threshold_exceeded = (
            (self.membrane_potentials[:self.capacity] >= self.thresholds[:self.capacity]) &
            can_update
        )
        
        if np.any(threshold_exceeded):
            fired_indices = np.where(threshold_exceeded)[0]
            
            # Vectorized reset of fired neurons
            self.membrane_potentials[fired_indices] = self.resting_potentials[fired_indices]
            self.refractory_counters[fired_indices] = self.refractory_periods[fired_indices]
            self.fired_mask[fired_indices] = True
            
            # Update statistics
            self.update_count += 1
            self.total_neurons_processed += len(fired_indices)
            
            return fired_indices
        
        return np.array([], dtype=np.int32)
    
    def batch_membrane_update(self, 
                            batch_indices: List[np.ndarray],
                            batch_currents: List[np.ndarray]) -> List[np.ndarray]:
        """
        Process multiple batches of membrane updates efficiently.
        
        Args:
            batch_indices: List of neuron index arrays
            batch_currents: List of current arrays
            
        Returns:
            List of fired neuron arrays for each batch
        """
        
        if self.use_profiling:
            total_elements = sum(len(indices) for indices in batch_indices)
            with profile_simd_operation("batch_membrane_update", total_elements):
                return self._batch_update_impl(batch_indices, batch_currents)
        else:
            return self._batch_update_impl(batch_indices, batch_currents)
    
    def _batch_update_impl(self, 
                         batch_indices: List[np.ndarray], 
                         batch_currents: List[np.ndarray]) -> List[np.ndarray]:
        """Core batch update implementation."""
        
        results = []
        
        # Process batches in chunks sized for optimal SIMD utilization
        chunk_size = self.backend_selector.get_chunk_size(len(batch_indices))
        
        for i in range(0, len(batch_indices), chunk_size):
            chunk_end = min(i + chunk_size, len(batch_indices))
            
            # Process chunk
            for j in range(i, chunk_end):
                fired = self.vectorized_membrane_update(batch_indices[j], batch_currents[j])
                results.append(fired)
        
        return results
    
    def sparse_membrane_update(self, 
                             sparse_input_matrix: np.ndarray,
                             active_sources: np.ndarray) -> np.ndarray:
        """
        Update membrane potentials from sparse input matrix.
        
        Args:
            sparse_input_matrix: Sparse matrix of synaptic weights
            active_sources: Array of active source neuron indices
            
        Returns:
            Array of fired neuron indices
        """
        
        if self.use_profiling:
            with profile_simd_operation("sparse_membrane_update", len(active_sources)):
                return self._sparse_update_impl(sparse_input_matrix, active_sources)
        else:
            return self._sparse_update_impl(sparse_input_matrix, active_sources)
    
    def _sparse_update_impl(self, 
                          sparse_input_matrix: np.ndarray, 
                          active_sources: np.ndarray) -> np.ndarray:
        """Core sparse update implementation."""
        
        # Use sparse matrix operations for efficiency
        try:
            from scipy import sparse
            
            if not sparse.issparse(sparse_input_matrix):
                # Convert to sparse if not already
                sparse_input_matrix = sparse.csr_matrix(sparse_input_matrix)
            
            # Extract rows for active sources (vectorized)
            if len(active_sources) > 0:
                # Clamp source indices
                valid_sources = active_sources[active_sources < sparse_input_matrix.shape[0]]
                
                if len(valid_sources) > 0:
                    # Get contribution from active sources
                    source_contributions = sparse_input_matrix[valid_sources]
                    
                    # Sum contributions (vectorized sparse operation)
                    total_inputs = np.array(source_contributions.sum(axis=0)).flatten()
                    
                    # Apply to all neurons
                    target_indices = np.arange(len(total_inputs))
                    return self.vectorized_membrane_update(target_indices, total_inputs)
            
        except ImportError:
            # Fallback to dense operations if scipy not available
            if len(active_sources) > 0:
                # Dense matrix approach
                valid_sources = active_sources[active_sources < sparse_input_matrix.shape[0]]
                
                if len(valid_sources) > 0:
                    # Sum over active source rows
                    total_inputs = sparse_input_matrix[valid_sources].sum(axis=0)
                    if hasattr(total_inputs, 'A1'):  # Handle matrix objects
                        total_inputs = total_inputs.A1
                    
                    target_indices = np.arange(len(total_inputs))
                    return self.vectorized_membrane_update(target_indices, total_inputs)
        
        return np.array([], dtype=np.int32)
    
    def optimized_decay_only_update(self) -> np.ndarray:
        """
        Perform decay-only update for all neurons (no input).
        
        This is optimized for the common case where no neurons are firing
        but membrane potentials still need to decay.
        
        Returns:
            Array of neuron indices that fired due to decay
        """
        
        if self.use_profiling:
            with profile_simd_operation("decay_only_update", self.capacity):
                return self._decay_only_impl()
        else:
            return self._decay_only_impl()
    
    def _decay_only_impl(self) -> np.ndarray:
        """Core decay-only implementation."""
        
        # Reset fired mask
        self.fired_mask.fill(False)
        
        # Update refractory counters (vectorized)
        in_refractory = self.refractory_counters[:self.capacity] > 0
        if np.any(in_refractory):
            self.refractory_counters[:self.capacity][in_refractory] -= 1
        
        # Only update neurons not in refractory period
        can_update = self.active_mask[:self.capacity] & (self.refractory_counters[:self.capacity] <= 0)
        
        if np.any(can_update):
            # Vectorized decay operation
            self.membrane_potentials[:self.capacity][can_update] *= self.decay_rates[:self.capacity][can_update]
            
            # Vectorized drift towards resting potential
            potential_diff = (self.resting_potentials[:self.capacity][can_update] - 
                            self.membrane_potentials[:self.capacity][can_update])
            self.membrane_potentials[:self.capacity][can_update] += (
                potential_diff * (1.0 - self.decay_rates[:self.capacity][can_update])
            )
            
            # Check for spontaneous firing due to decay dynamics
            threshold_exceeded = (
                (self.membrane_potentials[:self.capacity] >= self.thresholds[:self.capacity]) &
                can_update
            )
            
            if np.any(threshold_exceeded):
                fired_indices = np.where(threshold_exceeded)[0]
                
                # Vectorized reset
                self.membrane_potentials[fired_indices] = self.resting_potentials[fired_indices]
                self.refractory_counters[fired_indices] = self.refractory_periods[fired_indices]
                self.fired_mask[fired_indices] = True
                
                return fired_indices
        
        return np.array([], dtype=np.int32)
    
    def get_performance_stats(self) -> dict:
        """Get performance statistics."""
        return {
            "update_count": self.update_count,
            "total_neurons_processed": self.total_neurons_processed,
            "capacity": self.capacity,
            "aligned_capacity": self.aligned_capacity,
            "vector_width": self.vector_width,
            "simd_backend": self.simd_config["backend"].value,
            "avg_neurons_per_update": (
                self.total_neurons_processed / self.update_count 
                if self.update_count > 0 else 0
            )
        }
    
    def reset_stats(self):
        """Reset performance statistics."""
        self.update_count = 0
        self.total_neurons_processed = 0

# Convenience functions for direct use

def vectorized_membrane_update(membrane_potentials: np.ndarray,
                             thresholds: np.ndarray,
                             decay_rates: np.ndarray,
                             input_currents: np.ndarray,
                             neuron_indices: np.ndarray,
                             refractory_counters: np.ndarray,
                             refractory_periods: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Standalone vectorized membrane potential update.
    
    Args:
        membrane_potentials: Current membrane potentials
        thresholds: Firing thresholds
        decay_rates: Decay rates per neuron
        input_currents: Input currents to apply
        neuron_indices: Indices where to apply currents
        refractory_counters: Current refractory counters
        refractory_periods: Refractory periods per neuron
        
    Returns:
        Tuple of (updated_membrane_potentials, fired_neuron_indices)
    """
    
    # Make a copy to avoid modifying input
    potentials = membrane_potentials.copy()
    refrac_counters = refractory_counters.copy()
    
    # Vectorized decay
    can_update = refrac_counters <= 0
    potentials[can_update] *= decay_rates[can_update]
    
    # Vectorized refractory counter update
    in_refractory = refrac_counters > 0
    refrac_counters[in_refractory] -= 1
    
    # Apply input currents
    if len(neuron_indices) > 0 and len(input_currents) > 0:
        # Use advanced indexing for scatter operation
        receiving_neurons = can_update[neuron_indices]
        potentials[neuron_indices[receiving_neurons]] += input_currents[receiving_neurons]
    
    # Vectorized threshold check
    fired_mask = (potentials >= thresholds) & can_update
    fired_indices = np.where(fired_mask)[0]
    
    # Reset fired neurons
    if len(fired_indices) > 0:
        potentials[fired_indices] = 0.0
        refrac_counters[fired_indices] = refractory_periods[fired_indices]
    
    return potentials, fired_indices

def batch_vectorized_update(batch_data: List[Tuple[np.ndarray, np.ndarray]],
                          membrane_processor: SIMDMembraneProcessor) -> List[np.ndarray]:
    """
    Process multiple membrane update batches efficiently.
    
    Args:
        batch_data: List of (neuron_indices, input_currents) tuples
        membrane_processor: Processor instance to use
        
    Returns:
        List of fired neuron arrays
    """
    
    batch_indices = [data[0] for data in batch_data]
    batch_currents = [data[1] for data in batch_data]
    
    return membrane_processor.batch_membrane_update(batch_indices, batch_currents) 