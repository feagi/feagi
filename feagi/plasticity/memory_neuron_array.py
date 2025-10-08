"""
Memory Neuron Array - Structure of Arrays for memory neurons with lifecycle management.

This module implements a high-performance SoA (Structure of Arrays) for memory neurons
with full lifecycle management including aging, reactivation, and long-term conversion.

Key features:
- SIMD-optimized vectorized operations using NumPy
- Rust/RTOS compatible data layout
- Thread-safe operations with minimal locking
- Efficient memory management with index reuse
- Comprehensive lifecycle tracking
"""

import numpy as np
import threading
from typing import Dict, List, Optional, Set, Tuple, Any
from dataclasses import dataclass
from collections import defaultdict
import time

from .neuron_id_manager import get_neuron_id_manager, NeuronIdManager

@dataclass
class MemoryNeuronLifecycleConfig:
    """Configuration for memory neuron lifecycle management."""
    initial_lifespan: int = 20  # Initial lifespan in bursts
    lifespan_growth_rate: float = 3.0  # Additive increment on reactivation
    longterm_threshold: int = 100  # Lifespan threshold for long-term conversion
    max_reactivations: int = 1000  # Maximum reactivations before forced LTM

@dataclass
class MemoryNeuronStats:
    """Statistics for memory neuron array."""
    total_capacity: int = 0
    active_neurons: int = 0
    longterm_neurons: int = 0
    dead_neurons: int = 0
    reusable_indices: int = 0
    memory_usage_bytes: int = 0
    avg_lifespan: float = 0.0
    avg_activation_count: float = 0.0

class MemoryNeuronArray:
    """
    High-performance Structure of Arrays for memory neurons.
    
    Optimized for:
    - Vectorized operations using NumPy SIMD
    - Rust/RTOS compatibility with fixed-size arrays
    - Thread-safe operations
    - Memory efficiency with index reuse
    - GPU-friendly data layout (when needed)
    """
    
    def __init__(self, capacity: int = 50000):
        self.capacity = capacity
        self._lock = threading.RLock()
        
        # Core neuron properties (SoA layout)
        self.neuron_ids = np.zeros(capacity, dtype=np.uint32)  # Global unique IDs
        self.cortical_area_ids = np.zeros(capacity, dtype=np.uint32)  # Which memory area
        self.is_active = np.zeros(capacity, dtype=np.bool_)  # Active/inactive state
        
        # Lifecycle management
        self.lifespan_current = np.zeros(capacity, dtype=np.uint32)  # Current remaining lifespan
        self.lifespan_initial = np.zeros(capacity, dtype=np.uint32)  # Initial lifespan when created
        self.lifespan_growth_rate = np.zeros(capacity, dtype=np.float32)  # Growth rate per reactivation
        self.is_longterm_memory = np.zeros(capacity, dtype=np.bool_)  # Long-term memory flag
        
        # Temporal tracking
        self.creation_burst = np.zeros(capacity, dtype=np.uint64)  # When neuron was created
        self.last_activation_burst = np.zeros(capacity, dtype=np.uint64)  # Last reactivation
        self.activation_count = np.zeros(capacity, dtype=np.uint32)  # Total activations
        
        # Pattern association (for lookup)
        self.pattern_hash_to_index: Dict[bytes, int] = {}  # Pattern hash -> neuron index
        self.index_to_pattern_hash: Dict[int, bytes] = {}  # Neuron index -> pattern hash
        
        # Index management
        self.next_available_index = 0
        self.reusable_indices: Set[int] = set()  # Indices of dead neurons for reuse
        
        # Area-specific tracking
        self.area_neuron_indices: Dict[int, Set[int]] = defaultdict(set)  # area_id -> neuron indices
        
        # Statistics
        self._stats_cache: Optional[MemoryNeuronStats] = None
        self._stats_cache_timestamp = 0.0
        self._stats_cache_ttl = 1.0  # Cache stats for 1 second
        
        # Get neuron ID manager
        self.id_manager = get_neuron_id_manager()
    
    def create_memory_neuron(
        self,
        pattern_hash: bytes,
        cortical_area_id: int,
        current_burst: int,
        config: MemoryNeuronLifecycleConfig
    ) -> Optional[int]:
        """
        Create a new memory neuron.
        
        Args:
            pattern_hash: Unique pattern identifier (32-byte SHA-256)
            cortical_area_id: Which memory area this neuron belongs to
            current_burst: Current simulation timestep
            config: Lifecycle configuration
            
        Returns:
            Neuron index if successful, None if capacity exhausted
        """
        with self._lock:
            # Check if pattern already exists
            if pattern_hash in self.pattern_hash_to_index:
                existing_idx = self.pattern_hash_to_index[pattern_hash]
                if self.is_active[existing_idx]:
                    # Reactivate existing neuron instead
                    self.reactivate_memory_neuron(existing_idx, current_burst)
                    return existing_idx
            
            # Get neuron index (reuse or allocate new)
            neuron_idx = self._get_available_index()
            if neuron_idx is None:
                return None
            
            # Allocate global neuron ID
            neuron_id = self.id_manager.allocate_memory_neuron_id()
            if neuron_id is None:
                return None
            
            # Initialize neuron properties
            self.neuron_ids[neuron_idx] = neuron_id
            self.cortical_area_ids[neuron_idx] = cortical_area_id
            self.is_active[neuron_idx] = True
            
            # Initialize lifecycle
            self.lifespan_current[neuron_idx] = config.initial_lifespan
            self.lifespan_initial[neuron_idx] = config.initial_lifespan
            self.lifespan_growth_rate[neuron_idx] = config.lifespan_growth_rate
            self.is_longterm_memory[neuron_idx] = False
            
            # Initialize temporal tracking
            self.creation_burst[neuron_idx] = current_burst
            self.last_activation_burst[neuron_idx] = current_burst
            self.activation_count[neuron_idx] = 1
            
            # Register pattern association
            self.pattern_hash_to_index[pattern_hash] = neuron_idx
            self.index_to_pattern_hash[neuron_idx] = pattern_hash
            
            # Add to area tracking
            self.area_neuron_indices[cortical_area_id].add(neuron_idx)
            
            # Invalidate stats cache
            self._stats_cache = None
            
            return neuron_idx
    
    def reactivate_memory_neuron(self, neuron_idx: int, current_burst: int) -> bool:
        """
        Reactivate an existing memory neuron.
        
        Args:
            neuron_idx: Index of neuron to reactivate
            current_burst: Current simulation timestep
            
        Returns:
            True if successfully reactivated
        """
        if not self._is_valid_index(neuron_idx) or not self.is_active[neuron_idx]:
            return False
        
        with self._lock:
            # Update activation tracking
            self.last_activation_burst[neuron_idx] = current_burst
            self.activation_count[neuron_idx] += 1
            
            # Grow lifespan if not long-term memory
            if not self.is_longterm_memory[neuron_idx]:
                current_lifespan = int(self.lifespan_current[neuron_idx])
                growth = int(self.lifespan_growth_rate[neuron_idx])
                new_lifespan = current_lifespan + growth
                self.lifespan_current[neuron_idx] = np.uint32(new_lifespan)
            
            # Invalidate stats cache
            self._stats_cache = None
            
            return True
    
    def age_memory_neurons(self, current_burst: int) -> List[int]:
        """
        Age all active memory neurons (vectorized operation).
        
        Args:
            current_burst: Current simulation timestep
            
        Returns:
            List of neuron indices that died during aging
        """
        with self._lock:
            n = self.next_available_index
            if n == 0:
                return []
            
            # Create masks for eligible neurons (vectorized)
            active_mask = self.is_active[:n]
            not_longterm_mask = ~self.is_longterm_memory[:n]
            eligible_mask = active_mask & not_longterm_mask
            
            if not np.any(eligible_mask):
                return []
            
            # Age eligible neurons (vectorized operation)
            lifespans = self.lifespan_current[:n]
            positive_lifespan_mask = eligible_mask & (lifespans > 0)
            
            # Decrement lifespans (SIMD-optimized)
            lifespans[positive_lifespan_mask] -= 1
            
            # Find neurons that died
            died_mask = eligible_mask & (lifespans == 0)
            died_indices = np.flatnonzero(died_mask).astype(int).tolist()
            
            # Mark dead neurons as inactive
            if died_indices:
                self.is_active[died_indices] = False
                
                # Add to reusable indices and clean up associations
                for idx in died_indices:
                    self._cleanup_dead_neuron(idx)
                
                # Invalidate stats cache
                self._stats_cache = None
            
            return died_indices
    
    def check_longterm_conversion(self, longterm_threshold: int = 100) -> List[int]:
        """
        Check for neurons ready for long-term memory conversion.
        
        Args:
            longterm_threshold: Lifespan threshold for conversion
            
        Returns:
            List of neuron indices converted to long-term memory
        """
        with self._lock:
            n = self.next_available_index
            if n == 0:
                return []
            
            # Find eligible neurons (vectorized)
            active_mask = self.is_active[:n]
            not_longterm_mask = ~self.is_longterm_memory[:n]
            threshold_mask = self.lifespan_current[:n] >= longterm_threshold
            eligible_mask = active_mask & not_longterm_mask & threshold_mask
            
            converted_indices = np.flatnonzero(eligible_mask).astype(int).tolist()
            
            if converted_indices:
                # Convert to long-term memory (vectorized)
                self.is_longterm_memory[converted_indices] = True
                
                # Invalidate stats cache
                self._stats_cache = None
            
            return converted_indices
    
    def get_active_neurons_by_area(self, cortical_area_id: int) -> List[int]:
        """
        Get all active neuron IDs for a cortical area.
        
        Args:
            cortical_area_id: Cortical area identifier
            
        Returns:
            List of active neuron IDs (global IDs, not indices)
        """
        with self._lock:
            if cortical_area_id not in self.area_neuron_indices:
                return []
            
            active_neuron_ids = []
            for neuron_idx in self.area_neuron_indices[cortical_area_id]:
                if self._is_valid_index(neuron_idx) and self.is_active[neuron_idx]:
                    active_neuron_ids.append(int(self.neuron_ids[neuron_idx]))
            
            return active_neuron_ids
    
    def find_neuron_by_pattern(self, pattern_hash: bytes) -> Optional[int]:
        """
        Find neuron index by pattern hash.
        
        Args:
            pattern_hash: Pattern identifier
            
        Returns:
            Neuron index if found and active, None otherwise
        """
        with self._lock:
            neuron_idx = self.pattern_hash_to_index.get(pattern_hash)
            if neuron_idx is not None and self._is_valid_index(neuron_idx) and self.is_active[neuron_idx]:
                return neuron_idx
            return None
    
    def get_neuron_info(self, neuron_idx: int) -> Optional[Dict[str, Any]]:
        """
        Get detailed information about a neuron.
        
        Args:
            neuron_idx: Neuron index
            
        Returns:
            Dictionary with neuron information, None if invalid
        """
        if not self._is_valid_index(neuron_idx):
            return None
        
        with self._lock:
            pattern_hash = self.index_to_pattern_hash.get(neuron_idx)
            
            return {
                'neuron_id': int(self.neuron_ids[neuron_idx]),
                'neuron_idx': neuron_idx,
                'cortical_area_id': int(self.cortical_area_ids[neuron_idx]),
                'is_active': bool(self.is_active[neuron_idx]),
                'lifespan_current': int(self.lifespan_current[neuron_idx]),
                'lifespan_initial': int(self.lifespan_initial[neuron_idx]),
                'lifespan_growth_rate': float(self.lifespan_growth_rate[neuron_idx]),
                'is_longterm_memory': bool(self.is_longterm_memory[neuron_idx]),
                'creation_burst': int(self.creation_burst[neuron_idx]),
                'last_activation_burst': int(self.last_activation_burst[neuron_idx]),
                'activation_count': int(self.activation_count[neuron_idx]),
                'pattern_hash': pattern_hash.hex() if pattern_hash else None,
            }
    
    def get_stats(self) -> MemoryNeuronStats:
        """
        Get comprehensive statistics (cached for performance).
        
        Returns:
            MemoryNeuronStats with current statistics
        """
        current_time = time.time()
        
        # Return cached stats if still valid
        if (self._stats_cache is not None and 
            current_time - self._stats_cache_timestamp < self._stats_cache_ttl):
            return self._stats_cache
        
        with self._lock:
            n = self.next_available_index
            
            if n == 0:
                stats = MemoryNeuronStats(total_capacity=self.capacity)
            else:
                active_mask = self.is_active[:n]
                longterm_mask = self.is_longterm_memory[:n]
                
                active_count = int(np.sum(active_mask))
                longterm_count = int(np.sum(active_mask & longterm_mask))
                dead_count = n - active_count
                
                # Calculate averages for active neurons
                if active_count > 0:
                    active_lifespans = self.lifespan_current[:n][active_mask]
                    active_activations = self.activation_count[:n][active_mask]
                    avg_lifespan = float(np.mean(active_lifespans))
                    avg_activation_count = float(np.mean(active_activations))
                else:
                    avg_lifespan = 0.0
                    avg_activation_count = 0.0
                
                # Estimate memory usage
                memory_usage = (
                    self.capacity * (
                        4 * 4 +  # uint32 arrays
                        4 * 1 +  # float32 arrays  
                        8 * 2 +  # uint64 arrays
                        1 * 2    # bool arrays
                    ) +
                    len(self.pattern_hash_to_index) * (32 + 8) +  # Pattern hash mappings
                    len(self.area_neuron_indices) * 64  # Area tracking overhead
                )
                
                stats = MemoryNeuronStats(
                    total_capacity=self.capacity,
                    active_neurons=active_count,
                    longterm_neurons=longterm_count,
                    dead_neurons=dead_count,
                    reusable_indices=len(self.reusable_indices),
                    memory_usage_bytes=memory_usage,
                    avg_lifespan=avg_lifespan,
                    avg_activation_count=avg_activation_count,
                )
            
            # Cache the stats
            self._stats_cache = stats
            self._stats_cache_timestamp = current_time
            
            return stats
    
    def _get_available_index(self) -> Optional[int]:
        """Get next available neuron index (reuse or allocate new)."""
        # Try to reuse a dead neuron index first
        if self.reusable_indices:
            return self.reusable_indices.pop()
        
        # Allocate new index if capacity allows
        if self.next_available_index < self.capacity:
            idx = self.next_available_index
            self.next_available_index += 1
            return idx
        
        return None  # Capacity exhausted
    
    def _cleanup_dead_neuron(self, neuron_idx: int):
        """Clean up associations for a dead neuron."""
        # Deallocate global neuron ID
        neuron_id = int(self.neuron_ids[neuron_idx])
        self.id_manager.deallocate_memory_neuron_id(neuron_id)
        
        # Remove pattern association
        pattern_hash = self.index_to_pattern_hash.pop(neuron_idx, None)
        if pattern_hash is not None:
            self.pattern_hash_to_index.pop(pattern_hash, None)
        
        # Remove from area tracking
        area_id = int(self.cortical_area_ids[neuron_idx])
        self.area_neuron_indices[area_id].discard(neuron_idx)
        
        # Add to reusable indices
        self.reusable_indices.add(neuron_idx)
    
    def _is_valid_index(self, neuron_idx: int) -> bool:
        """Check if neuron index is valid."""
        return 0 <= neuron_idx < self.next_available_index
    
    def reset(self):
        """Reset array state (for testing)."""
        with self._lock:
            # Reset arrays
            self.neuron_ids.fill(0)
            self.cortical_area_ids.fill(0)
            self.is_active.fill(False)
            self.lifespan_current.fill(0)
            self.lifespan_initial.fill(0)
            self.lifespan_growth_rate.fill(0.0)
            self.is_longterm_memory.fill(False)
            self.creation_burst.fill(0)
            self.last_activation_burst.fill(0)
            self.activation_count.fill(0)
            
            # Reset mappings
            self.pattern_hash_to_index.clear()
            self.index_to_pattern_hash.clear()
            self.area_neuron_indices.clear()
            
            # Reset index management
            self.next_available_index = 0
            self.reusable_indices.clear()
            
            # Reset cache
            self._stats_cache = None
