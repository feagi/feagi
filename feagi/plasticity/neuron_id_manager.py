"""
Neuron ID allocation system for memory and regular neurons.

This module provides globally unique neuron ID allocation with range partitioning
to ensure memory neurons and regular neurons never have ID collisions while
maintaining GPU performance.

Architecture:
- Regular neurons: 0 to 49,999,999
- Memory neurons: 50,000,000 to 99,999,999
- Reserved range: 100,000,000+ for future use

This design ensures:
- No bitwise operations in GPU hot paths
- Simple integer comparisons for type checking
- RTOS-friendly with no dynamic allocation
- Rust-compatible with simple integer types
"""

from typing import Optional, Set
import threading
from dataclasses import dataclass

# ID Range Constants - GPU and RTOS friendly
REGULAR_NEURON_ID_START = 0
REGULAR_NEURON_ID_MAX = 49_999_999
MEMORY_NEURON_ID_START = 50_000_000
MEMORY_NEURON_ID_MAX = 99_999_999
RESERVED_ID_START = 100_000_000

@dataclass
class NeuronIdRanges:
    """Neuron ID range configuration for different neuron types."""
    regular_start: int = REGULAR_NEURON_ID_START
    regular_max: int = REGULAR_NEURON_ID_MAX
    memory_start: int = MEMORY_NEURON_ID_START
    memory_max: int = MEMORY_NEURON_ID_MAX
    reserved_start: int = RESERVED_ID_START

class NeuronIdManager:
    """
    Thread-safe neuron ID allocation manager.
    
    Provides globally unique IDs with range partitioning for performance.
    Designed for Rust migration and RTOS compatibility.
    """
    
    def __init__(self):
        self._lock = threading.Lock()
        self._next_regular_id = REGULAR_NEURON_ID_START
        self._next_memory_id = MEMORY_NEURON_ID_START
        
        # Track allocated IDs for validation (optional in production)
        self._allocated_regular_ids: Set[int] = set()
        self._allocated_memory_ids: Set[int] = set()
        
        # Statistics
        self._regular_allocated_count = 0
        self._memory_allocated_count = 0
    
    def allocate_regular_neuron_id(self) -> Optional[int]:
        """
        Allocate a new regular neuron ID.
        
        Returns:
            New unique regular neuron ID, or None if range exhausted
        """
        with self._lock:
            if self._next_regular_id > REGULAR_NEURON_ID_MAX:
                return None
            
            neuron_id = self._next_regular_id
            self._next_regular_id += 1
            self._allocated_regular_ids.add(neuron_id)
            self._regular_allocated_count += 1
            
            return neuron_id
    
    def allocate_memory_neuron_id(self) -> Optional[int]:
        """
        Allocate a new memory neuron ID.
        
        Returns:
            New unique memory neuron ID, or None if range exhausted
        """
        with self._lock:
            if self._next_memory_id > MEMORY_NEURON_ID_MAX:
                return None
            
            neuron_id = self._next_memory_id
            self._next_memory_id += 1
            self._allocated_memory_ids.add(neuron_id)
            self._memory_allocated_count += 1
            
            return neuron_id
    
    def deallocate_regular_neuron_id(self, neuron_id: int) -> bool:
        """
        Deallocate a regular neuron ID for reuse.
        
        Args:
            neuron_id: ID to deallocate
            
        Returns:
            True if successfully deallocated
        """
        if not self.is_regular_neuron_id(neuron_id):
            return False
        
        with self._lock:
            if neuron_id in self._allocated_regular_ids:
                self._allocated_regular_ids.remove(neuron_id)
                return True
            return False
    
    def deallocate_memory_neuron_id(self, neuron_id: int) -> bool:
        """
        Deallocate a memory neuron ID for reuse.
        
        Args:
            neuron_id: ID to deallocate
            
        Returns:
            True if successfully deallocated
        """
        if not self.is_memory_neuron_id(neuron_id):
            return False
        
        with self._lock:
            if neuron_id in self._allocated_memory_ids:
                self._allocated_memory_ids.remove(neuron_id)
                return True
            return False
    
    @staticmethod
    def is_regular_neuron_id(neuron_id: int) -> bool:
        """
        Check if neuron ID belongs to regular neuron range.
        
        GPU-optimized: Simple integer comparison, no bitwise operations.
        """
        return REGULAR_NEURON_ID_START <= neuron_id <= REGULAR_NEURON_ID_MAX
    
    @staticmethod
    def is_memory_neuron_id(neuron_id: int) -> bool:
        """
        Check if neuron ID belongs to memory neuron range.
        
        GPU-optimized: Simple integer comparison, no bitwise operations.
        """
        return MEMORY_NEURON_ID_START <= neuron_id <= MEMORY_NEURON_ID_MAX
    
    @staticmethod
    def get_neuron_type(neuron_id: int) -> str:
        """
        Get neuron type from ID.
        
        Returns:
            'regular', 'memory', 'reserved', or 'invalid'
        """
        if NeuronIdManager.is_regular_neuron_id(neuron_id):
            return 'regular'
        elif NeuronIdManager.is_memory_neuron_id(neuron_id):
            return 'memory'
        elif neuron_id >= RESERVED_ID_START:
            return 'reserved'
        else:
            return 'invalid'
    
    def get_allocation_stats(self) -> dict:
        """Get allocation statistics."""
        with self._lock:
            return {
                'regular_allocated': self._regular_allocated_count,
                'memory_allocated': self._memory_allocated_count,
                'regular_capacity': REGULAR_NEURON_ID_MAX - REGULAR_NEURON_ID_START + 1,
                'memory_capacity': MEMORY_NEURON_ID_MAX - MEMORY_NEURON_ID_START + 1,
                'regular_utilization': self._regular_allocated_count / (REGULAR_NEURON_ID_MAX - REGULAR_NEURON_ID_START + 1),
                'memory_utilization': self._memory_allocated_count / (MEMORY_NEURON_ID_MAX - MEMORY_NEURON_ID_START + 1),
            }
    
    def reset(self):
        """Reset allocation state (for testing)."""
        with self._lock:
            self._next_regular_id = REGULAR_NEURON_ID_START
            self._next_memory_id = MEMORY_NEURON_ID_START
            self._allocated_regular_ids.clear()
            self._allocated_memory_ids.clear()
            self._regular_allocated_count = 0
            self._memory_allocated_count = 0

# Global singleton instance
_neuron_id_manager: Optional[NeuronIdManager] = None
_manager_lock = threading.Lock()

def get_neuron_id_manager() -> NeuronIdManager:
    """Get global neuron ID manager singleton."""
    global _neuron_id_manager
    if _neuron_id_manager is None:
        with _manager_lock:
            if _neuron_id_manager is None:
                _neuron_id_manager = NeuronIdManager()
    return _neuron_id_manager

def reset_neuron_id_manager():
    """Reset global neuron ID manager (for testing)."""
    global _neuron_id_manager
    with _manager_lock:
        _neuron_id_manager = None
