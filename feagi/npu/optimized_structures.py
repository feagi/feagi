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
Optimized data structures for FEAGI using SIMD and GPU-friendly implementations.

This module provides high-performance SIMD and WebGPU accelerated data structures
for key FEAGI components like the Global Neuron Array, Fire Candidate List, and Connectome.

All data structures use Structure of Arrays (SoA) pattern with NumPy arrays
for SIMD/GPU compatibility. Memory alignment and data types are chosen
for optimal performance on both CPU and GPU targets.
"""

import os
import sys
import logging
import numpy as np
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple, Union, Any

# Try to import the Rust extension
try:
    # Try to import the Rust extension
    try:
        from feagi_rust import (
            create_gna, 
            create_fcl, 
            create_connectome, 
            create_feagi_core,
        )
        RUST_AVAILABLE = True
    except (ImportError, AttributeError) as e:
        # Log the specific error to help with debugging
        logging.warning(f"Rust optimized structures not available (Error: {str(e)}). Using NumPy-based SIMD fallback.")
        RUST_AVAILABLE = False
except Exception as e:
    # Catch any other errors during import
    logging.warning(f"Error importing Rust components: {str(e)}. Using NumPy-based SIMD fallback.")
    RUST_AVAILABLE = False

class GlobalNeuronArray:
    """
    Global Neuron Array (GNA) optimized for SIMD and WebGPU.
    
    This class manages the state of all neurons in the network with optimized memory layouts
    for both CPU SIMD operations and GPU processing.
    """
    
    def __init__(self, capacity: int):
        """
        Initialize a GNA with specified capacity.
        
        Args:
            capacity: Maximum number of neurons to support
        """
        self.capacity = capacity
        
        # Import SIMD detection for alignment
        try:
            from ..utils.simd_detection import get_simd_detector
            simd_detector = get_simd_detector()
            self.alignment = simd_detector.get_memory_alignment()
            self.vector_width = simd_detector.capabilities.vector_width
            
            # Align capacity to SIMD vector boundaries for optimal performance
            self.aligned_capacity = simd_detector.get_aligned_size(capacity)
        except ImportError:
            self.alignment = 32  # Default to 32-byte alignment
            self.vector_width = 8
            self.aligned_capacity = (capacity + 7) & ~7  # 8-element alignment
        
        if RUST_AVAILABLE:
            self._rust_gna = create_gna(self.aligned_capacity)
            self._use_rust = True
        else:
            # Fallback to NumPy implementation - SoA pattern with SIMD alignment
            self._use_rust = False
            
            # Use contiguous C-order arrays with proper dtype alignment for SIMD/GPU
            # Float32 for all floating point values - optimal for GPU and SIMD
            self.membrane_potentials = self._create_aligned_array(self.aligned_capacity, np.float32)
            self.thresholds = self._create_aligned_array(self.aligned_capacity, np.float32)
            
            # Initialize thresholds to 1.0
            self.thresholds.fill(1.0)
            
            # Int32 for integer values - optimal for GPU
            self.refractory_periods = self._create_aligned_array(self.aligned_capacity, np.int32)
            self.refractory_counters = self._create_aligned_array(self.aligned_capacity, np.int32)
            self.last_fired = self._create_aligned_array(self.aligned_capacity, np.int32)
            self.neuron_types = self._create_aligned_array(self.aligned_capacity, np.int32)  # 0=excitatory, 1=inhibitory, etc.
            self.enabled_flags = self._create_aligned_array(self.aligned_capacity, np.int32)  # 1=enabled, 0=disabled
            self.cortical_idxs = self._create_aligned_array(self.aligned_capacity, np.int32)
            
            # Initialize enabled flags to 1
            self.enabled_flags.fill(1)
            
            # Use separate arrays for coordinates - optimal SoA layout for SIMD/GPU processing
            # Each coordinate component gets its own contiguous memory array
            self.coordinates_x = self._create_aligned_array(self.aligned_capacity, np.uint32)
            self.coordinates_y = self._create_aligned_array(self.aligned_capacity, np.uint32)
            self.coordinates_z = self._create_aligned_array(self.aligned_capacity, np.uint32)
            
            # Additional arrays for SIMD-optimized operations
            self.decay_rates = self._create_aligned_array(self.aligned_capacity, np.float32)
            self.resting_potentials = self._create_aligned_array(self.aligned_capacity, np.float32)
            
            # Initialize decay rates to default value
            self.decay_rates.fill(0.95)
            
            # Working arrays for vectorized operations
            self._temp_potentials = self._create_aligned_array(self.aligned_capacity, np.float32)
            self._temp_mask = self._create_aligned_array(self.aligned_capacity, np.bool_)
    
    def _create_aligned_array(self, size: int, dtype: np.dtype) -> np.ndarray:
        """
        Create a SIMD-aligned NumPy array for optimal performance.
        
        Args:
            size: Number of elements
            dtype: NumPy data type
            
        Returns:
            Aligned NumPy array
        """
        # Calculate total bytes needed
        element_size = np.dtype(dtype).itemsize
        total_bytes = size * element_size
        
        # Pad to alignment boundary
        aligned_bytes = (total_bytes + self.alignment - 1) & ~(self.alignment - 1)
        aligned_size = aligned_bytes // element_size
        
        # Create array with extra padding and ensure C-order
        array = np.zeros(aligned_size, dtype=dtype, order='C')
        
        # Verify alignment (best effort - NumPy doesn't guarantee alignment)
        if array.ctypes.data % self.alignment == 0:
            # Array is properly aligned
            pass
        else:
            # NumPy couldn't provide aligned memory, but array is still usable
            # Performance may be slightly reduced
            pass
        
        return array[:size]  # Return only the requested size
    
    def get_membrane_potential(self, neuron_id: int) -> float:
        """Get membrane potential for a neuron."""
        if self._use_rust:
            return self._rust_gna.get_membrane_potential(neuron_id)
        else:
            return float(self.membrane_potentials[neuron_id])
    
    def set_membrane_potential(self, neuron_id: int, value: float) -> None:
        """Set membrane potential for a neuron."""
        if self._use_rust:
            self._rust_gna.set_membrane_potential(neuron_id, value)
        else:
            self.membrane_potentials[neuron_id] = float(value)
    
    def get_coordinates(self, neuron_id: int) -> Tuple[int, int, int]:
        """Get 3D coordinates for a neuron.
        
        Args:
            neuron_id: ID of the neuron
            
        Returns:
            Tuple of (x, y, z) coordinates
        """
        if self._use_rust:
            return self._rust_gna.get_coordinates(neuron_id)
        else:
            return (
                int(self.coordinates_x[neuron_id]),
                int(self.coordinates_y[neuron_id]),
                int(self.coordinates_z[neuron_id])
            )
    
    def set_coordinates(self, neuron_id: int, x: int, y: int, z: int) -> None:
        """Set 3D coordinates for a neuron.
        
        Args:
            neuron_id: ID of the neuron
            x: X coordinate
            y: Y coordinate
            z: Z coordinate
        """
        if self._use_rust:
            self._rust_gna.set_coordinates(neuron_id, x, y, z)
        else:
            # Convert to uint32, ensuring non-negative values
            self.coordinates_x[neuron_id] = np.uint32(max(0, x))
            self.coordinates_y[neuron_id] = np.uint32(max(0, y))
            self.coordinates_z[neuron_id] = np.uint32(max(0, z))
    
    def simd_optimized_update_membrane_potentials(self, decay_factor: float) -> None:
        """
        SIMD-optimized membrane potential update with vectorization.
        
        Args:
            decay_factor: Global decay factor to apply
        """
        if self._use_rust:
            self._rust_gna.decay_membrane_potentials(decay_factor)
        else:
            # Use SIMD-friendly vectorized operations
            # Process in chunks for optimal cache utilization
            chunk_size = max(self.vector_width * 8, 64)  # Process 8 SIMD vectors at a time
            
            for i in range(0, self.capacity, chunk_size):
                end_idx = min(i + chunk_size, self.capacity)
                
                # Vectorized decay operation on chunk
                self.membrane_potentials[i:end_idx] *= decay_factor
    
    def update_refractory_counters(self) -> None:
        """Update refractory counters for all neurons."""
        if self._use_rust:
            self._rust_gna.update_refractory_counters()
        else:
            # Vectorized operation using NumPy
            mask = self.refractory_counters > 0
            self.refractory_counters[mask] -= 1
    
    def simd_optimized_find_fire_candidates(self, timestep: int) -> List[int]:
        """
        SIMD-optimized fire candidate detection with vectorization.
        
        Args:
            timestep: Current simulation timestep
            
        Returns:
            List of neuron IDs ready to fire
        """
        if self._use_rust:
            return self._rust_gna.get_fire_candidates(timestep)
        else:
            # Use vectorized operations for candidate detection
            # Process in SIMD-friendly chunks
            candidates = []
            chunk_size = max(self.vector_width * 4, 32)
            
            for i in range(0, self.capacity, chunk_size):
                end_idx = min(i + chunk_size, self.capacity)
                
                # Vectorized conditions check
                not_refractory = self.refractory_counters[i:end_idx] == 0
                enabled = self.enabled_flags[i:end_idx] > 0
                above_threshold = self.membrane_potentials[i:end_idx] >= self.thresholds[i:end_idx]
                
                # Combined condition using vectorized AND
                fire_mask = not_refractory & enabled & above_threshold
                
                # Get indices of firing neurons in this chunk
                if np.any(fire_mask):
                    chunk_candidates = np.where(fire_mask)[0] + i
                    candidates.extend(chunk_candidates.tolist())
            
            return candidates
    
    def simd_optimized_process_fired_neurons(self, fired_list: List[int], timestep: int) -> None:
        """
        SIMD-optimized processing of fired neurons with vectorization.
        
        Args:
            fired_list: List of neuron IDs that fired
            timestep: Current simulation timestep
        """
        if self._use_rust:
            self._rust_gna.process_fired_neurons(fired_list, timestep)
        else:
            if not fired_list:
                return
                
            # Convert to NumPy array for vectorized operations
            fired_indices = np.asarray(fired_list, dtype=np.int32)
            
            # Vectorized reset operations
            self.membrane_potentials[fired_indices] = self.resting_potentials[fired_indices]
            self.refractory_counters[fired_indices] = self.refractory_periods[fired_indices]
            self.last_fired[fired_indices] = timestep
    
    def batch_update_coordinates(self, neuron_ids: List[int], 
                               coordinates: List[Tuple[int, int, int]]) -> None:
        """
        Batch update coordinates using vectorized operations.
        
        Args:
            neuron_ids: List of neuron IDs to update
            coordinates: List of (x, y, z) coordinate tuples
        """
        if len(neuron_ids) != len(coordinates):
            raise ValueError("neuron_ids and coordinates must have same length")
        
        if not neuron_ids:
            return
        
        # Convert to arrays for vectorized assignment
        indices = np.asarray(neuron_ids, dtype=np.int32)
        coords_array = np.asarray(coordinates, dtype=np.uint32)
        
        # Vectorized coordinate updates
        self.coordinates_x[indices] = coords_array[:, 0]
        self.coordinates_y[indices] = coords_array[:, 1]
        self.coordinates_z[indices] = coords_array[:, 2]
    
    def vectorized_membrane_potential_update(self, 
                                           input_currents: np.ndarray, 
                                           target_indices: np.ndarray) -> np.ndarray:
        """
        Vectorized membrane potential update with input currents.
        
        Args:
            input_currents: Array of input currents
            target_indices: Array of target neuron indices
            
        Returns:
            Array of neurons that fired
        """
        if len(input_currents) != len(target_indices):
            raise ValueError("input_currents and target_indices must have same length")
        
        # Apply input currents using vectorized scatter operation
        # Only update neurons not in refractory period
        can_update = (self.refractory_counters[target_indices] == 0) & (self.enabled_flags[target_indices] > 0)
        
        if np.any(can_update):
            valid_targets = target_indices[can_update]
            valid_currents = input_currents[can_update]
            
            # Vectorized current application
            self.membrane_potentials[valid_targets] += valid_currents
            
            # Check for firing (vectorized)
            fired_mask = self.membrane_potentials[valid_targets] >= self.thresholds[valid_targets]
            
            if np.any(fired_mask):
                fired_neurons = valid_targets[fired_mask]
                
                # Reset fired neurons (vectorized)
                self.membrane_potentials[fired_neurons] = self.resting_potentials[fired_neurons]
                self.refractory_counters[fired_neurons] = self.refractory_periods[fired_neurons]
                
                return fired_neurons
        
        return np.array([], dtype=np.int32)
    
    def get_all_membrane_potentials(self) -> List[float]:
        """Get all membrane potentials."""
        if self._use_rust:
            return self._rust_gna.get_all_membrane_potentials()
        else:
            return self.membrane_potentials.tolist()
            


class FireCandidateList:
    """
    Fire Candidate List (FCL) optimized for SIMD and WebGPU.
    
    Manages the list of neurons that are firing in the current timestep,
    with optimizations for both CPU (SIMD) and GPU processing.
    
    Uses bitmap-like structures for efficient SIMD/GPU operations.
    """
    
    def __init__(self, neuron_ids: Optional[List[int]] = None, capacity: int = 1000000):
        """
        Initialize an FCL, optionally with a list of neuron IDs.
        
        Args:
            neuron_ids: Initial list of firing neurons
            capacity: Maximum expected neuron ID (for sparse bitmap optimization)
        """
        self.capacity = capacity
        
        if RUST_AVAILABLE:
            self._use_rust = True
            if neuron_ids is not None:
                self._rust_fcl = create_fcl(neuron_ids)
            else:
                self._rust_fcl = create_fcl([])
        else:
            # Fallback to NumPy implementation
            self._use_rust = False
            
            # For smaller neuron counts, use a dense boolean mask (fastest for most operations)
            if capacity <= 1000000:  # 1M neurons threshold
                self._mask = np.zeros(capacity, dtype=np.bool_)
                if neuron_ids is not None:
                    self._mask[neuron_ids] = True
                self._use_dense = True
            else:
                # For large neuron counts, use a sparse representation
                self._active_ids = np.array(neuron_ids or [], dtype=np.int32)
                self._use_dense = False
    
    def add(self, neuron_id: int) -> None:
        """Add a neuron to the FCL."""
        if self._use_rust:
            self._rust_fcl.add(neuron_id)
        else:
            if self._use_dense:
                self._mask[neuron_id] = True
            else:
                if neuron_id not in self._active_ids:
                    self._active_ids = np.append(self._active_ids, neuron_id)
    
    def add_multiple(self, neuron_ids: List[int]) -> None:
        """Add multiple neurons to the FCL."""
        if self._use_rust:
            self._rust_fcl.add_multiple(neuron_ids)
        else:
            if self._use_dense:
                # Convert to NumPy array if not already
                ids = np.asarray(neuron_ids, dtype=np.int32)
                self._mask[ids] = True
            else:
                # Use numpy's setdiff1d for efficiency
                to_add = np.setdiff1d(
                    np.asarray(neuron_ids, dtype=np.int32), 
                    self._active_ids,
                    assume_unique=True
                )
                if len(to_add) > 0:
                    self._active_ids = np.concatenate([self._active_ids, to_add])
    
    def remove(self, neuron_id: int) -> None:
        """Remove a neuron from the FCL."""
        if self._use_rust:
            self._rust_fcl.remove(neuron_id)
        else:
            if self._use_dense:
                self._mask[neuron_id] = False
            else:
                # Find and remove the neuron ID
                mask = self._active_ids != neuron_id
                self._active_ids = self._active_ids[mask]
    
    def clear(self) -> None:
        """Clear the FCL."""
        if self._use_rust:
            self._rust_fcl.clear()
        else:
            if self._use_dense:
                self._mask.fill(False)
            else:
                self._active_ids = np.array([], dtype=np.int32)
    
    def contains(self, neuron_id: int) -> bool:
        """Check if a neuron is in the FCL."""
        if self._use_rust:
            return self._rust_fcl.contains(neuron_id)
        else:
            if self._use_dense:
                return bool(self._mask[neuron_id])
            else:
                return neuron_id in self._active_ids
    
    def __len__(self) -> int:
        """Get the number of neurons in the FCL."""
        if self._use_rust:
            return self._rust_fcl.len()
        else:
            if self._use_dense:
                return int(np.sum(self._mask))
            else:
                return len(self._active_ids)
    
    def is_empty(self) -> bool:
        """Check if the FCL is empty."""
        if self._use_rust:
            return self._rust_fcl.is_empty()
        else:
            if self._use_dense:
                return not np.any(self._mask)
            else:
                return len(self._active_ids) == 0
    
    def to_list(self) -> List[int]:
        """Get a list of all neurons in the FCL."""
        if self._use_rust:
            return self._rust_fcl.to_list()
        else:
            if self._use_dense:
                return np.where(self._mask)[0].tolist()
            else:
                return self._active_ids.tolist()
    
    def __iter__(self):
        """Iterate over neurons in the FCL."""
        if self._use_rust:
            return iter(self._rust_fcl.to_list())
        else:
            if self._use_dense:
                return iter(np.where(self._mask)[0])
            else:
                return iter(self._active_ids)

class Connectome:
    """
    Connectome (synaptic connectivity) optimized for SIMD and WebGPU.
    
    Manages synaptic connections between neurons with optimizations for
    sparse matrix operations in both CPU (SIMD) and GPU contexts.
    
    Uses CSR-like format for efficient sparse matrix operations.
    """
    
    def __init__(self, neuron_count: int, estimated_connections: int = 1000000):
        """
        Initialize a Connectome with specified capacity.
        
        Args:
            neuron_count: Number of neurons in the network
            estimated_connections: Estimated number of connections (for memory allocation)
        """
        self.neuron_count = neuron_count
        self.initial_capacity = estimated_connections
        
        if RUST_AVAILABLE:
            self._rust_connectome = create_connectome(neuron_count, estimated_connections)
            self._use_rust = True
        else:
            # Fallback to NumPy implementation using CSR-like format
            self._use_rust = False
            
            # Initialize CSR-like arrays
            # source_offsets[i] gives the starting index in target_indices for source i's connections
            # source_offsets[i+1] - source_offsets[i] gives the number of connections from source i
            self.source_offsets = np.zeros(neuron_count + 1, dtype=np.int32)
            
            # Allocate with expected capacity
            # These arrays will be resized as needed
            self.target_indices = np.zeros(estimated_connections, dtype=np.int32)
            self.weights = np.zeros(estimated_connections, dtype=np.float32)
            self.delays = np.zeros(estimated_connections, dtype=np.int32)
            self.connection_types = np.zeros(estimated_connections, dtype=np.int32)
            self.source_cortical_idxs = np.zeros(estimated_connections, dtype=np.int32)
            self.target_cortical_idxs = np.zeros(estimated_connections, dtype=np.int32)
            
            # Track actual used size
            self._connection_count = 0
    
    def add_connection(
        self,
        source_id: int,
        target_id: int,
        weight: float,
        delay: int = 0,
        connection_type: int = 0,
        source_cortical_idx: int = 0,
        target_cortical_idx: int = 0,
    ) -> None:
        """Add a connection between neurons.
        
        Args:
            source_id: ID of the source neuron
            target_id: ID of the target neuron
            weight: Connection weight
            delay: Connection delay in timesteps
            connection_type: Type of connection (0=excitatory, 1=inhibitory, etc.)
            source_cortical_idx: Index of the source cortical area
            target_cortical_idx: Index of the target cortical area
        """
        if self._use_rust:
            # Use Rust implementation if available
            self._rust_connectome.add_connection(
                source_id, target_id, weight, delay, connection_type, source_cortical_idx, target_cortical_idx
            )
            return
        else:
            # Check if we need to resize the arrays
            if self._connection_count >= len(self.target_indices):
                # Double capacity (typical amortized growth strategy)
                new_capacity = max(self._connection_count * 2, self.initial_capacity)
                self._resize_arrays(new_capacity)
            
            # Find position to insert: after existing connections from source_id
            insert_pos = self.source_offsets[source_id]
            
            # Shift existing connections from later sources
            # We need to:
            # 1. Shift all connections after insert_pos
            # 2. Update all source_offsets after source_id
            
            # Make space for new connection
            if self._connection_count > insert_pos:
                # Move existing connections one position forward
                self.target_indices[insert_pos+1:self._connection_count+1] = self.target_indices[insert_pos:self._connection_count]
                self.weights[insert_pos+1:self._connection_count+1] = self.weights[insert_pos:self._connection_count]
                self.delays[insert_pos+1:self._connection_count+1] = self.delays[insert_pos:self._connection_count]
                self.connection_types[insert_pos+1:self._connection_count+1] = self.connection_types[insert_pos:self._connection_count]
                self.source_cortical_idxs[insert_pos+1:self._connection_count+1] = self.source_cortical_idxs[insert_pos:self._connection_count]
                self.target_cortical_idxs[insert_pos+1:self._connection_count+1] = self.target_cortical_idxs[insert_pos:self._connection_count]
            
            # Insert new connection
            self.target_indices[insert_pos] = target_id
            self.weights[insert_pos] = weight
            self.delays[insert_pos] = delay
            self.connection_types[insert_pos] = connection_type
            self.source_cortical_idxs[insert_pos] = source_cortical_idx
            self.target_cortical_idxs[insert_pos] = target_cortical_idx
            
            # Update offsets for all sources after this one
            self.source_offsets[source_id+1:] += 1
            
            # Increment connection count
            self._connection_count += 1
    
    def _resize_arrays(self, new_capacity: int) -> None:
        """
        Resize internal arrays to new capacity.
        
        Args:
            new_capacity: New capacity for arrays
        """
        self.target_indices = np.resize(self.target_indices, new_capacity)
        self.weights = np.resize(self.weights, new_capacity)
        self.delays = np.resize(self.delays, new_capacity)
        self.connection_types = np.resize(self.connection_types, new_capacity)
        self.source_cortical_idxs = np.resize(self.source_cortical_idxs, new_capacity)
        self.target_cortical_idxs = np.resize(self.target_cortical_idxs, new_capacity)
    
    def get_connections_for_neuron(self, neuron_id: int) -> List[Dict[str, Any]]:
        """
        Get all outgoing connections for a neuron.
        
        Args:
            neuron_id: Source neuron ID
            
        Returns:
            List of dictionaries with connection details
        """
        if self._use_rust:
            return self._rust_connectome.get_connections_for_neuron(neuron_id)
        else:
            # Get range of connections for this neuron
            start_idx = self.source_offsets[neuron_id]
            end_idx = self.source_offsets[neuron_id + 1]
            
            # Create list of dictionaries
            connections = []
            for i in range(start_idx, end_idx):
                connections.append({
                    "source_id": neuron_id,
                    "target_id": int(self.target_indices[i]),
                    "weight": float(self.weights[i]),
                    "delay": int(self.delays[i]),
                    "connection_type": int(self.connection_types[i]),
                    "source_cortical_id": int(self.source_cortical_idxs[i]),
                    "target_cortical_id": int(self.target_cortical_idxs[i])
                })
            
            return connections
    
    def connection_count(self) -> int:
        """Get the total number of connections."""
        if self._use_rust:
            return self._rust_connectome.connection_count()
        else:
            return self._connection_count
    

    
    def propagate_activations(
        self, source_activations: List[float], target_buffer: List[float]
    ) -> List[float]:
        """
        Propagate activations from source neurons to target neurons.
        
        Args:
            source_activations: Activation values for all source neurons
            target_buffer: Buffer to store target neuron activations
            
        Returns:
            Updated target buffer
        """
        if self._use_rust:
            return self._rust_connectome.propagate_activations(source_activations, target_buffer)
        else:
            # Convert to NumPy arrays for vectorized operations
            src_act = np.asarray(source_activations, dtype=np.float32)
            tgt_buff = np.asarray(target_buffer, dtype=np.float32)
            
            # For each source with nonzero activation
            nonzero_sources = np.nonzero(src_act)[0]
            for src_id in nonzero_sources:
                # Get connections for this source
                start_idx = self.source_offsets[src_id]
                end_idx = self.source_offsets[src_id + 1]
                
                if start_idx == end_idx:
                    continue  # No connections
                
                # Get target neurons and weights
                targets = self.target_indices[start_idx:end_idx]
                conn_weights = self.weights[start_idx:end_idx]
                
                # Calculate PSP contribution (activation * weight)
                psp = src_act[src_id] * conn_weights
                
                # Add to target buffer (using np.add.at for safe accumulation)
                np.add.at(tgt_buff, targets, psp)
            
            return tgt_buff.tolist()

class OptimizedFeagiCore:
    """
    Optimized FEAGI Core integrating GNA, FCL, and Connectome.
    
    This class provides a unified interface for the core FEAGI components,
    optimized for SIMD and WebGPU operations.
    """
    
    def __init__(self, neuron_capacity: int, estimated_connections: int = 1000000):
        """
        Initialize an optimized FEAGI core.
        
        Args:
            neuron_capacity: Maximum number of neurons to support
            estimated_connections: Estimated number of synaptic connections
        """
        if RUST_AVAILABLE:
            self._rust_core = create_feagi_core(neuron_capacity, estimated_connections)
            self._use_rust = True
            
            # These are just Python wrappers around Rust objects
            self.gna = GlobalNeuronArray(neuron_capacity)
            self.fcl = FireCandidateList()
            self.connectome = Connectome(neuron_capacity, estimated_connections)
            self._current_timestep = 0
        else:
            # Use our SIMD/GPU-friendly NumPy implementations
            self._use_rust = False
            self.gna = GlobalNeuronArray(neuron_capacity)
            self.fcl = FireCandidateList(capacity=neuron_capacity)
            self.connectome = Connectome(neuron_capacity, estimated_connections)
            self._current_timestep = 0
    
    def step(self):
        """Step the simulation forward by one timestep."""
        if self._use_rust:
            self._rust_core.step()
            self._current_timestep += 1
        else:
            # Use our vectorized implementations
            # 1. Decay membrane potentials
            self.gna.simd_optimized_update_membrane_potentials(0.95)  # Example decay factor
            
            # 2. Update refractory counters
            self.gna.update_refractory_counters()
            
            # 3. Find neurons ready to fire
            fire_candidates = self.gna.simd_optimized_find_fire_candidates(self._current_timestep)
            
            # 4. Update FCL
            self.fcl.clear()
            self.fcl.add_multiple(fire_candidates)
            
            # 5. Process fired neurons
            if fire_candidates:  # Only process if there are any candidates
                self.gna.simd_optimized_process_fired_neurons(fire_candidates, self._current_timestep)
            
            # Increment timestep
            self._current_timestep += 1
    
    def propagate_activations(self) -> List[float]:
        """
        Propagate activations from all firing neurons to their targets.
        
        Returns:
            New membrane potentials after propagation
        """
        if self._use_rust:
            return self._rust_core.propagate_activations()
        else:
            # Get firing neurons
            firing_neurons = self.fcl.to_list()
            
            # Create source activations (1.0 for firing, 0.0 for others)
            source_activations = np.zeros(self.gna.capacity, dtype=np.float32)
            if firing_neurons:  # Only set if there are firing neurons
                source_activations[firing_neurons] = 1.0
            
            # Create target buffer with current membrane potentials
            target_buffer = self.gna.membrane_potentials.copy()
            
            # Propagate through the connectome
            updated_buffer = self.connectome.propagate_activations(
                source_activations.tolist(), target_buffer.tolist()
            )
            
            # Update membrane potentials (could be optimized further)
            for i, potential in enumerate(updated_buffer):
                self.gna.set_membrane_potential(i, potential)
            
            return updated_buffer
    
    @property
    def current_timestep(self) -> int:
        """Get the current timestep."""
        if self._use_rust:
            return self._rust_core.get_current_timestep()
        else:
            return self._current_timestep
    
    @current_timestep.setter
    def current_timestep(self, value: int):
        """Set the current timestep."""
        if self._use_rust:
            self._rust_core.set_current_timestep(value)
        self._current_timestep = value 