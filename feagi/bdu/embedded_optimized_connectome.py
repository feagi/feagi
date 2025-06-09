"""
Embedded-Optimized Connectome Manager

This module provides a connectome manager specifically optimized for embedded
single-core operation targeting 10M neuron operations at 15Hz. It integrates
the embedded-optimized neuron array with cache-friendly sparse connectivity
management.

Key Optimizations:
1. Block-sparse connectivity matrix for cache-friendly access
2. Memory pool allocation for zero-allocation operation
3. SIMD-optimized batch processing
4. Embedded mode configuration
5. GPU compatibility maintained
"""

import numpy as np
import time
from typing import Dict, List, Tuple, Any, Optional, Union, Set
from dataclasses import dataclass
import logging

from feagi.bdu.models.embedded_optimized_neuron import (
    EmbeddedOptimizedNeuronArray, 
    EmbeddedConfig,
    BlockSparseMatrix,
    CacheAlignedArray
)
from feagi.bdu.models.cortical_area import CorticalArea
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)

@dataclass 
class ConnectomePerformanceMetrics:
    """Performance metrics for connectome operations."""
    burst_time_ms: float = 0.0
    neurons_processed: int = 0
    neurons_fired: int = 0
    connections_processed: int = 0
    cache_hits: int = 0
    cache_misses: int = 0
    simd_efficiency: float = 0.0
    memory_bandwidth_utilization: float = 0.0

class EmbeddedOptimizedConnectomeManager:
    """
    Ultra-high-performance connectome manager for embedded operation.
    
    Designed to achieve 10M neuron operations at 15Hz by:
    - Using embedded-optimized neuron array with SIMD vectorization
    - Block-sparse connectivity matrix for cache efficiency
    - Memory pool allocation for zero-allocation paths
    - Single-threaded optimizations for embedded systems
    """
    
    def __init__(self, config: Optional[EmbeddedConfig] = None):
        """Initialize embedded-optimized connectome manager."""
        self.config = config or EmbeddedConfig()
        
        # Initialize the embedded-optimized neuron array
        self.neuron_array = EmbeddedOptimizedNeuronArray(self.config)
        
        # Initialize sparse connectivity matrix
        self.connectivity_matrix = BlockSparseMatrix(
            max_neurons=self.config.max_neurons,
            block_size=self.config.sparse_block_size
        )
        
        # Neuron ID management (optimized for embedded operation)
        self.next_neuron_id = 0
        self.neuron_id_to_index: Dict[int, int] = {}
        self.index_to_neuron_id: Dict[int, int] = {}
        
        # Cortical area management
        self.cortical_areas: Dict[str, CorticalArea] = {}
        self.next_cortical_idx = 2  # Reserve 0 for "_death", 1 for "___pwr"
        
        # Reserved areas
        self.reserved_cortical_areas = {
            "_death": 0,
            "___pwr": 1
        }
        
        # Performance tracking
        self.performance_metrics = ConnectomePerformanceMetrics()
        self.burst_count = 0
        self.total_processing_time = 0.0
        
        # Memory pools for zero-allocation operation
        if self.config.use_memory_pools:
            self._init_memory_pools()
        
        # FCL manager for compatibility
        self.current_timestep = 0
        self.active_neurons = set()
        
        logger.info(f"EmbeddedOptimizedConnectomeManager initialized: "
                   f"{self.config.max_neurons} max neurons, "
                   f"block size: {self.config.sparse_block_size}")
    
    def _init_memory_pools(self):
        """Initialize memory pools for zero-allocation operation."""
        # Pre-allocate arrays for fired neuron processing
        self.temp_fired_neurons = CacheAlignedArray(1000, np.int32)  # Reasonable upper bound
        self.temp_target_neurons = CacheAlignedArray(10000, np.int32)  # For connectivity
        self.temp_weights = CacheAlignedArray(10000, np.float32)
        
        # Pre-allocate arrays for batch operations
        self.temp_membrane_updates = CacheAlignedArray(self.config.max_neurons, np.float32)
        self.temp_neuron_mask = CacheAlignedArray(self.config.max_neurons, np.bool_)
    
    def create_cortical_area(self, name: str, dimensions: Tuple[int, int, int],
                           area_type: str = "cortical") -> CorticalArea:
        """Create a new cortical area."""
        if name in self.cortical_areas:
            raise ValueError(f"Cortical area '{name}' already exists")
        
        # Get cortical index
        if name in self.reserved_cortical_areas:
            cortical_idx = self.reserved_cortical_areas[name]
        else:
            cortical_idx = self.next_cortical_idx
            self.next_cortical_idx += 1
        
        # Create cortical area
        area = CorticalArea(
            name=name,
            dimensions=dimensions,
            cortical_idx=cortical_idx,
            area_type=area_type
        )
        
        self.cortical_areas[name] = area
        logger.debug(f"Created cortical area '{name}' with index {cortical_idx}")
        
        return area
    
    def create_neuron(self, cortical_id: str, position: Tuple[int, int, int],
                     threshold: float = 1.0, membrane_potential: float = 0.0,
                     resting_potential: float = 0.0, decay_rate: float = 0.05,
                     refractory_period: int = 1) -> int:
        """Create a new neuron in the specified cortical area."""
        # Get or create cortical area
        if cortical_id not in self.cortical_areas:
            # Infer dimensions from position for auto-creation
            dimensions = (position[0] + 1, position[1] + 1, position[2] + 1)
            self.create_cortical_area(cortical_id, dimensions)
        
        area = self.cortical_areas[cortical_id]
        
        # Create neuron in the embedded-optimized array
        index = self.neuron_array.create_neuron(
            cortical_idx=area.cortical_idx,
            position=position,
            threshold=threshold,
            membrane_potential=membrane_potential,
            resting_potential=resting_potential,
            decay_rate=decay_rate,
            refractory_period=refractory_period
        )
        
        # Generate neuron ID and create mappings
        neuron_id = self.next_neuron_id
        self.next_neuron_id += 1
        
        self.neuron_id_to_index[neuron_id] = index
        self.index_to_neuron_id[index] = neuron_id
        
        # Add to cortical area
        area.add_neuron(neuron_id, position)
        
        logger.debug(f"Created neuron {neuron_id} in area {cortical_id} at position {position}")
        return neuron_id
    
    def add_connection(self, source_id: int, target_id: int, weight: float):
        """Add synaptic connection between neurons."""
        # Get indices
        if source_id not in self.neuron_id_to_index or target_id not in self.neuron_id_to_index:
            raise ValueError("Source or target neuron does not exist")
        
        source_index = self.neuron_id_to_index[source_id]
        target_index = self.neuron_id_to_index[target_id]
        
        # Add to block-sparse connectivity matrix
        self.connectivity_matrix.add_connection(source_index, target_index, weight)
        
        # Also add to neuron array's connectivity
        self.neuron_array.add_connection(source_index, target_index, weight)
    
    def update_membrane_potentials(self, current_timestep: Optional[int] = None) -> List[int]:
        """
        Process membrane potential updates using embedded-optimized algorithm.
        
        This is the critical path that must execute in <66.7ms for 10M neurons at 15Hz.
        
        Returns:
            List of neuron IDs that fired
        """
        start_time = time.perf_counter()
        
        # Process burst using embedded-optimized neuron array
        fired_indices, metrics = self.neuron_array.process_burst_embedded()
        
        # Convert indices back to neuron IDs
        fired_neuron_ids = [
            self.index_to_neuron_id.get(idx, idx) for idx in fired_indices
        ]
        
        # Update active neurons for next timestep
        self.active_neurons = set(fired_neuron_ids)
        
        # Update timestep
        self.current_timestep += 1
        self.burst_count += 1
        
        # Update performance metrics
        processing_time = time.perf_counter() - start_time
        self.total_processing_time += processing_time
        
        self.performance_metrics.burst_time_ms = processing_time * 1000
        self.performance_metrics.neurons_processed = self.neuron_array.neuron_count
        self.performance_metrics.neurons_fired = len(fired_neuron_ids)
        self.performance_metrics.simd_efficiency = metrics.get('simd_efficiency', 0.0)
        
        # Log performance if debugging
        if self.burst_count % 100 == 0:  # Log every 100 bursts
            avg_time = self.total_processing_time / self.burst_count * 1000
            logger.debug(f"Burst {self.burst_count}: {avg_time:.2f}ms avg, "
                        f"{len(fired_neuron_ids)} fired, "
                        f"SIMD efficiency: {self.performance_metrics.simd_efficiency:.2f}")
        
        return fired_neuron_ids
    
    def get_neuron(self, neuron_id: int) -> Dict[str, Any]:
        """Get information about a specific neuron."""
        if neuron_id not in self.neuron_id_to_index:
            raise KeyError(f"Neuron {neuron_id} does not exist")
        
        index = self.neuron_id_to_index[neuron_id]
        
        # Get neuron properties from embedded array
        position = (
            int(self.neuron_array.coordinates_x[index]),
            int(self.neuron_array.coordinates_y[index]),
            int(self.neuron_array.coordinates_z[index])
        )
        
        cortical_idx = int(self.neuron_array.cortical_idxs[index])
        
        # Find cortical ID
        cortical_id = None
        for area_id, area in self.cortical_areas.items():
            if area.cortical_idx == cortical_idx:
                cortical_id = area_id
                break
        
        return {
            "cortical_id": cortical_id,
            "cortical_idx": cortical_idx,
            "position": position,
            "threshold": float(self.neuron_array.thresholds[index]),
            "membrane_potential": float(self.neuron_array.membrane_potentials[index]),
            "resting_potential": float(self.neuron_array.resting_potentials[index]),
            "decay_rate": float(self.neuron_array.decay_rates[index]),
            "refractory_period": int(self.neuron_array.refractory_periods[index]),
            "refractory_counter": int(self.neuron_array.refractory_counters[index])
        }
    
    def get_cortical_area(self, area_id: str) -> Optional[CorticalArea]:
        """Get cortical area by ID."""
        return self.cortical_areas.get(area_id)
    
    def get_neurons_in_area(self, area_id: str) -> List[int]:
        """Get all neuron IDs in a cortical area."""
        if area_id not in self.cortical_areas:
            return []
        
        area = self.cortical_areas[area_id]
        return list(area.neuron_positions.keys())
    
    def get_neuron_count(self) -> int:
        """Get total number of neurons."""
        return self.neuron_array.neuron_count
    
    def get_performance_summary(self) -> Dict[str, Any]:
        """Get comprehensive performance summary."""
        neuron_perf = self.neuron_array.get_performance_summary()
        
        avg_burst_time = self.total_processing_time / self.burst_count if self.burst_count > 0 else 0
        
        summary = {
            # Overall metrics
            'total_neurons': self.neuron_array.neuron_count,
            'total_areas': len(self.cortical_areas),
            'burst_count': self.burst_count,
            'avg_burst_time_ms': avg_burst_time * 1000,
            'target_frequency_hz': self.config.target_frequency_hz,
            'target_achieved': avg_burst_time < (self.config.max_burst_time_ms / 1000),
            
            # Embedded optimizations
            'embedded_mode': self.config.single_threaded,
            'simd_enabled': self.config.simd_vectorization,
            'memory_pools_enabled': self.config.use_memory_pools,
            'block_sparse_size': self.config.sparse_block_size,
            
            # Performance details
            'simd_type': neuron_perf.get('simd_type', 'Unknown'),
            'simd_width': neuron_perf.get('simd_width', 0),
            'operations_per_second': neuron_perf.get('operations_per_second', 0),
            'megaops_per_second': neuron_perf.get('megaops_per_second', 0),
            
            # Latest burst metrics
            'latest_burst_time_ms': self.performance_metrics.burst_time_ms,
            'latest_neurons_fired': self.performance_metrics.neurons_fired,
            'latest_simd_efficiency': self.performance_metrics.simd_efficiency
        }
        
        return summary
    
    def enable_embedded_mode(self):
        """Enable embedded mode optimizations."""
        self.config.single_threaded = True
        self.config.use_memory_pools = True
        self.config.cache_friendly_layout = True
        self.config.simd_vectorization = True
        
        logger.info("Embedded mode optimizations enabled")
    
    def batch_create_neurons(self, cortical_id: str, positions: List[Tuple[int, int, int]],
                           thresholds: Union[float, List[float]] = 1.0,
                           membrane_potentials: Union[float, List[float]] = 0.0,
                           resting_potentials: Union[float, List[float]] = 0.0,
                           decay_rates: Union[float, List[float]] = 0.05,
                           refractory_periods: Union[int, List[int]] = 1) -> List[int]:
        """Create multiple neurons efficiently in batch."""
        # Ensure cortical area exists
        if cortical_id not in self.cortical_areas:
            max_pos = [0, 0, 0]
            for pos in positions:
                max_pos[0] = max(max_pos[0], pos[0])
                max_pos[1] = max(max_pos[1], pos[1])
                max_pos[2] = max(max_pos[2], pos[2])
            dimensions = (max_pos[0] + 1, max_pos[1] + 1, max_pos[2] + 1)
            self.create_cortical_area(cortical_id, dimensions)
        
        area = self.cortical_areas[cortical_id]
        neuron_ids = []
        
        # Convert scalar parameters to lists if needed
        n_neurons = len(positions)
        if isinstance(thresholds, (int, float)):
            thresholds = [thresholds] * n_neurons
        if isinstance(membrane_potentials, (int, float)):
            membrane_potentials = [membrane_potentials] * n_neurons
        if isinstance(resting_potentials, (int, float)):
            resting_potentials = [resting_potentials] * n_neurons
        if isinstance(decay_rates, (int, float)):
            decay_rates = [decay_rates] * n_neurons
        if isinstance(refractory_periods, int):
            refractory_periods = [refractory_periods] * n_neurons
        
        # Create neurons in batch
        for i, position in enumerate(positions):
            neuron_id = self.create_neuron(
                cortical_id=cortical_id,
                position=position,
                threshold=thresholds[i],
                membrane_potential=membrane_potentials[i],
                resting_potential=resting_potentials[i],
                decay_rate=decay_rates[i],
                refractory_period=refractory_periods[i]
            )
            neuron_ids.append(neuron_id)
        
        logger.info(f"Created {len(neuron_ids)} neurons in area '{cortical_id}' via batch operation")
        return neuron_ids
    
    def clear_brain_data(self):
        """Clear all brain data for new genome loading."""
        # Reset neuron array
        self.neuron_array = EmbeddedOptimizedNeuronArray(self.config)
        
        # Reset connectivity
        self.connectivity_matrix = BlockSparseMatrix(
            max_neurons=self.config.max_neurons,
            block_size=self.config.sparse_block_size
        )
        
        # Reset mappings
        self.neuron_id_to_index.clear()
        self.index_to_neuron_id.clear()
        self.cortical_areas.clear()
        
        # Reset counters
        self.next_neuron_id = 0
        self.next_cortical_idx = 2
        self.current_timestep = 0
        self.active_neurons.clear()
        
        # Reset performance tracking
        self.burst_count = 0
        self.total_processing_time = 0.0
        self.performance_metrics = ConnectomePerformanceMetrics()
        
        logger.info("Brain data cleared for new genome")
    
    # Compatibility methods for existing FEAGI interfaces
    
    @property
    def neuron_count(self) -> int:
        """Compatibility property for neuron count."""
        return self.neuron_array.neuron_count
    
    def get_neuron_property(self, neuron_id: int, property_name: str) -> Any:
        """Get a specific property of a neuron."""
        neuron_info = self.get_neuron(neuron_id)
        return neuron_info.get(property_name)
    
    def set_neuron_property(self, neuron_id: int, property_name: str, value: Any) -> None:
        """Set a specific property of a neuron."""
        if neuron_id not in self.neuron_id_to_index:
            raise KeyError(f"Neuron {neuron_id} does not exist")
        
        index = self.neuron_id_to_index[neuron_id]
        
        # Map property names to neuron array attributes
        if property_name == "membrane_potential":
            self.neuron_array.membrane_potentials[index] = float(value)
        elif property_name == "threshold":
            self.neuron_array.thresholds[index] = float(value)
        elif property_name == "resting_potential":
            self.neuron_array.resting_potentials[index] = float(value)
        elif property_name == "decay_rate":
            self.neuron_array.decay_rates[index] = float(value)
        elif property_name == "refractory_period":
            self.neuron_array.refractory_periods[index] = int(value)
        elif property_name == "refractory_counter":
            self.neuron_array.refractory_counters[index] = int(value)
        else:
            raise ValueError(f"Unknown property: {property_name}")