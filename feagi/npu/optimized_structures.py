"""
Optimized data structures for FEAGI using Rust implementations.

This module provides high-performance SIMD and WebGPU accelerated data structures
for key FEAGI components like the Global Neuron Array, Fire Candidate List, and Connectome.
"""

import os
import sys
import logging
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple, Union, Any

# Try to import the Rust extension
try:
    from feagi_rust import (
        create_gna, 
        create_fcl, 
        create_connectome, 
        create_feagi_core,
    )
    RUST_AVAILABLE = True
except ImportError:
    RUST_AVAILABLE = False
    logging.warning("Rust optimized structures not available. Using Python fallback.")

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
        if RUST_AVAILABLE:
            self._rust_gna = create_gna(capacity)
            self._use_rust = True
        else:
            # Fallback to Python implementation
            self._use_rust = False
            self.membrane_potentials = [0.0] * capacity
            self.thresholds = [1.0] * capacity
            self.refractory_periods = [0] * capacity
            self.refractory_counters = [0] * capacity
            self.last_fired = [0] * capacity
            self.neuron_types = [0] * capacity  # 0=excitatory, 1=inhibitory, etc.
            self.enabled_flags = [1] * capacity  # 1=enabled, 0=disabled
            self.cortical_area_ids = [0] * capacity
            self.coordinates = [(0, 0, 0)] * capacity
    
    def get_membrane_potential(self, neuron_id: int) -> float:
        """Get membrane potential for a neuron."""
        if self._use_rust:
            return self._rust_gna.get_membrane_potential(neuron_id)
        else:
            return self.membrane_potentials[neuron_id]
    
    def set_membrane_potential(self, neuron_id: int, value: float) -> None:
        """Set membrane potential for a neuron."""
        if self._use_rust:
            self._rust_gna.set_membrane_potential(neuron_id, value)
        else:
            self.membrane_potentials[neuron_id] = value
    
    def update_membrane_potentials(self, decay_factor: float) -> None:
        """Update all membrane potentials with decay."""
        if self._use_rust:
            self._rust_gna.decay_membrane_potentials(decay_factor)
        else:
            # Slow Python implementation
            for i in range(self.capacity):
                self.membrane_potentials[i] *= decay_factor
    
    def update_refractory_counters(self) -> None:
        """Update refractory counters for all neurons."""
        if self._use_rust:
            self._rust_gna.update_refractory_counters()
        else:
            # Slow Python implementation
            for i in range(self.capacity):
                if self.refractory_counters[i] > 0:
                    self.refractory_counters[i] -= 1
    
    def find_fire_candidates(self, timestep: int) -> List[int]:
        """Find neurons that are ready to fire."""
        if self._use_rust:
            return self._rust_gna.get_fire_candidates(timestep)
        else:
            # Slow Python implementation
            candidates = []
            for i in range(self.capacity):
                # Skip neurons in refractory period
                if self.refractory_counters[i] > 0:
                    continue
                
                # Skip disabled neurons
                if self.enabled_flags[i] == 0:
                    continue
                
                # Check if above threshold
                if self.membrane_potentials[i] >= self.thresholds[i]:
                    candidates.append(i)
            
            return candidates
    
    def process_fired_neurons(self, fired_list: List[int], timestep: int) -> None:
        """Process neurons that have fired."""
        if self._use_rust:
            self._rust_gna.process_fired_neurons(fired_list, timestep)
        else:
            # Slow Python implementation
            for neuron_id in fired_list:
                # Reset membrane potential
                self.membrane_potentials[neuron_id] = 0.0
                
                # Set refractory counter
                self.refractory_counters[neuron_id] = self.refractory_periods[neuron_id]
                
                # Update last fired timestamp
                self.last_fired[neuron_id] = timestep
    
    def get_all_membrane_potentials(self) -> List[float]:
        """Get all membrane potentials."""
        if self._use_rust:
            return self._rust_gna.get_all_membrane_potentials()
        else:
            return self.membrane_potentials

class FireCandidateList:
    """
    Fire Candidate List (FCL) optimized for Rust with SIMD operations.
    
    Manages the list of neurons that are firing in the current timestep,
    with optimizations for both CPU (SIMD) and GPU processing.
    """
    
    def __init__(self, neuron_ids: Optional[List[int]] = None):
        """
        Initialize an FCL, optionally with a list of neuron IDs.
        
        Args:
            neuron_ids: Initial list of firing neurons
        """
        if RUST_AVAILABLE:
            self._use_rust = True
            if neuron_ids is not None:
                self._rust_fcl = create_fcl(neuron_ids)
            else:
                self._rust_fcl = create_fcl([])
        else:
            # Fallback to Python implementation
            self._use_rust = False
            self._neurons = set(neuron_ids or [])
    
    def add(self, neuron_id: int) -> None:
        """Add a neuron to the FCL."""
        if self._use_rust:
            self._rust_fcl.add(neuron_id)
        else:
            self._neurons.add(neuron_id)
    
    def add_multiple(self, neuron_ids: List[int]) -> None:
        """Add multiple neurons to the FCL."""
        if self._use_rust:
            self._rust_fcl.add_multiple(neuron_ids)
        else:
            self._neurons.update(neuron_ids)
    
    def remove(self, neuron_id: int) -> None:
        """Remove a neuron from the FCL."""
        if self._use_rust:
            self._rust_fcl.remove(neuron_id)
        else:
            self._neurons.discard(neuron_id)
    
    def clear(self) -> None:
        """Clear the FCL."""
        if self._use_rust:
            self._rust_fcl.clear()
        else:
            self._neurons.clear()
    
    def contains(self, neuron_id: int) -> bool:
        """Check if a neuron is in the FCL."""
        if self._use_rust:
            return self._rust_fcl.contains(neuron_id)
        else:
            return neuron_id in self._neurons
    
    def __len__(self) -> int:
        """Get the number of neurons in the FCL."""
        if self._use_rust:
            return self._rust_fcl.len()
        else:
            return len(self._neurons)
    
    def is_empty(self) -> bool:
        """Check if the FCL is empty."""
        if self._use_rust:
            return self._rust_fcl.is_empty()
        else:
            return len(self._neurons) == 0
    
    def to_list(self) -> List[int]:
        """Get a list of all neurons in the FCL."""
        if self._use_rust:
            return self._rust_fcl.to_list()
        else:
            return list(self._neurons)
    
    def __iter__(self):
        """Iterate over neurons in the FCL."""
        if self._use_rust:
            return iter(self._rust_fcl.to_list())
        else:
            return iter(self._neurons)

class Connectome:
    """
    Connectome (synaptic connectivity) optimized for SIMD and WebGPU.
    
    Manages synaptic connections between neurons with optimizations for
    sparse matrix operations in both CPU (SIMD) and GPU contexts.
    """
    
    def __init__(self, neuron_count: int, estimated_connections: int = 1000000):
        """
        Initialize a Connectome with specified capacity.
        
        Args:
            neuron_count: Number of neurons in the network
            estimated_connections: Estimated number of connections (for memory allocation)
        """
        self.neuron_count = neuron_count
        if RUST_AVAILABLE:
            self._rust_connectome = create_connectome(neuron_count, estimated_connections)
            self._use_rust = True
        else:
            # Fallback to Python implementation
            self._use_rust = False
            self._connections = {}  # source_id -> [(target_id, weight, delay, type, src_area, tgt_area)]
    
    def add_connection(
        self,
        source_id: int,
        target_id: int,
        weight: float,
        delay: int = 0,
        connection_type: int = 0,
        source_area_id: int = 0,
        target_area_id: int = 0,
    ) -> None:
        """Add a connection to the connectome."""
        if self._use_rust:
            self._rust_connectome.add_connection(
                source_id, target_id, weight, delay, connection_type, source_area_id, target_area_id
            )
        else:
            # Fallback Python implementation
            if source_id not in self._connections:
                self._connections[source_id] = []
            
            self._connections[source_id].append(
                (target_id, weight, delay, connection_type, source_area_id, target_area_id)
            )
    
    def get_connections_for_neuron(self, neuron_id: int) -> List[Dict[str, Any]]:
        """Get all connections for a specific neuron."""
        if self._use_rust:
            return self._rust_connectome.get_connections_for_neuron(neuron_id)
        else:
            if neuron_id not in self._connections:
                return []
            
            return [
                {
                    "target_id": conn[0],
                    "weight": conn[1],
                    "delay": conn[2],
                    "type": conn[3],
                    "source_area_id": conn[4],
                    "target_area_id": conn[5],
                }
                for conn in self._connections[neuron_id]
            ]
    
    def connection_count(self) -> int:
        """Get the total number of connections in the connectome."""
        if self._use_rust:
            return self._rust_connectome.connection_count()
        else:
            return sum(len(conns) for conns in self._connections.values())
    
    def propagate_activations(
        self, source_activations: List[float], target_buffer: List[float]
    ) -> List[float]:
        """Propagate activations through the connectome."""
        if self._use_rust:
            return self._rust_connectome.propagate_activations(source_activations, target_buffer)
        else:
            # Slow Python implementation
            for source_id, activation in enumerate(source_activations):
                if activation <= 0.0:
                    continue
                
                if source_id not in self._connections:
                    continue
                
                for target_id, weight, _, _, _, _ in self._connections[source_id]:
                    target_buffer[target_id] += activation * weight
            
            return target_buffer

class OptimizedFeagiCore:
    """
    Optimized FEAGI Core with SIMD and WebGPU acceleration.
    
    Combines GNA, FCL, and Connectome in an optimized processing system.
    """
    
    def __init__(self, neuron_capacity: int, estimated_connections: int = 1000000):
        """
        Initialize the FEAGI core.
        
        Args:
            neuron_capacity: Maximum number of neurons to support
            estimated_connections: Estimated number of connections
        """
        if RUST_AVAILABLE:
            self._use_rust = True
            self._rust_core = create_feagi_core(neuron_capacity, estimated_connections)
        else:
            self._use_rust = False
            self.gna = GlobalNeuronArray(neuron_capacity)
            self.fcl = FireCandidateList()
            self.connectome = Connectome(neuron_capacity, estimated_connections)
            self.current_timestep = 0
    
    def step(self):
        """Perform a single simulation timestep."""
        if self._use_rust:
            self._rust_core.step()
        else:
            # 1. Decay membrane potentials
            self.gna.update_membrane_potentials(0.95)  # Example decay factor
            
            # 2. Update refractory counters
            self.gna.update_refractory_counters()
            
            # 3. Find neurons ready to fire
            fire_candidates = self.gna.find_fire_candidates(self.current_timestep)
            
            # 4. Update the FCL
            self.fcl.clear()
            self.fcl.add_multiple(fire_candidates)
            
            # 5. Process fired neurons
            self.gna.process_fired_neurons(fire_candidates, self.current_timestep)
            
            # Increment timestep
            self.current_timestep += 1
    
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
        if not self._use_rust:
            self._current_timestep = value
    
    def propagate_activations(self) -> List[float]:
        """Propagate activations from fired neurons through the connectome."""
        if self._use_rust:
            return self._rust_core.propagate_activations()
        else:
            # Create a temporary buffer of activations (1.0 for fired neurons, 0.0 otherwise)
            activations = [0.0] * self.gna.capacity
            
            # Set activations for fired neurons
            for neuron_id in self.fcl:
                activations[neuron_id] = 1.0
            
            # Create target buffer
            target_buffer = [0.0] * self.gna.capacity
            
            # Propagate through the connectome
            return self.connectome.propagate_activations(activations, target_buffer) 