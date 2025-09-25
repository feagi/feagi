"""
Fire Ledger Interface for FEAGI NPU

Python interface to historical firing data management. This will be implemented in Rust
for maximum performance, with this Python wrapper providing the interface.

Responsibilities:
- Historical firing pattern storage (roaring bitmaps)
- STDP (Spike-Time Dependent Plasticity) data access
- Short-term and long-term memory formation support  
- Per-cortical-area window size configuration
- High-performance temporal pattern queries

Future: This will call into a Rust crate `fire_ledger_rs` for actual implementation.
"""

from typing import Dict, List, Optional, Set, Tuple
import numpy as np
from collections import deque
from feagi.utils.logger import setup_logger
from .fire_queue import FiringNeuron

logger = setup_logger(__name__)

# Placeholder for future Rust implementation
try:
    # This will work once we create the Rust crate
    # import fire_ledger_rs
    fire_ledger_rs = None
except ImportError:
    fire_ledger_rs = None


class RoaringBitmap:
    """Placeholder for roaring bitmap - will use Rust implementation."""
    
    def __init__(self, data=None):
        self._data = set(data) if data else set()
    
    def add(self, value: int):
        self._data.add(value)
    
    def union(self, other: 'RoaringBitmap') -> 'RoaringBitmap':
        result = RoaringBitmap()
        result._data = self._data.union(other._data)
        return result
    
    def intersection(self, other: 'RoaringBitmap') -> 'RoaringBitmap':
        result = RoaringBitmap()
        result._data = self._data.intersection(other._data)
        return result
        
    def is_empty(self) -> bool:
        return len(self._data) == 0
    
    def __len__(self) -> int:
        return len(self._data)
    
    def __iter__(self):
        return iter(self._data)
    
    def copy(self) -> 'RoaringBitmap':
        result = RoaringBitmap()
        result._data = self._data.copy()
        return result


class CorticalHistory:
    """Historical firing data for a single cortical area."""
    
    def __init__(self, window_size: int):
        self.window_size = window_size
        self.firing_history: deque = deque(maxlen=window_size)
        self.current_timestep = 0
        
        # Pre-populate with empty bitmaps
        for _ in range(window_size):
            self.firing_history.append(RoaringBitmap())
    
    def add_timestep(self, timestep: int, firing_neurons: List[int]):
        """Add firing data for a timestep."""
        bitmap = RoaringBitmap(firing_neurons)
        self.firing_history.append(bitmap)
        self.current_timestep = timestep
    
    def get_pattern(self, lookback_steps: int) -> RoaringBitmap:
        """Get union of neurons that fired in last N steps."""
        if lookback_steps <= 0:
            return RoaringBitmap()
            
        pattern = RoaringBitmap()
        steps_to_check = min(lookback_steps, len(self.firing_history))
        
        for i in range(steps_to_check):
            pattern = pattern.union(self.firing_history[-(i+1)])
            
        return pattern
    
    def get_timestep_pattern(self, timestep_offset: int = 0) -> RoaringBitmap:
        """Get firing pattern for specific timestep offset."""
        if timestep_offset >= 0 or abs(timestep_offset) > len(self.firing_history):
            return RoaringBitmap()
        return self.firing_history[timestep_offset].copy()


class MemoryArea:
    """Extended cortical area with memory-specific functionality."""
    
    def __init__(self, cortical_idx: int, window_size: int, upstream_areas: List[int]):
        self.cortical_idx = cortical_idx
        self.window_size = window_size
        self.upstream_areas = upstream_areas
        self.history = CorticalHistory(window_size)
        
    def add_firing_data(self, timestep: int, firing_neurons: List[int]):
        """Add firing data for memory area."""
        self.history.add_timestep(timestep, firing_neurons)
    
    def get_memory_pattern(self, lookback_steps: int) -> RoaringBitmap:
        """Get memory pattern for STDP processing."""
        return self.history.get_pattern(lookback_steps)


class FireLedgerInterface:
    """Python interface to Fire Ledger (future Rust implementation)."""
    
    def __init__(self, default_window_size: int = 20):
        """Initialize Fire Ledger interface.
        
        Args:
            default_window_size: Default number of timesteps to maintain
        """
        self.default_window_size = default_window_size
        self.window_size = default_window_size  # Compatibility alias
        
        # Historical storage (will be replaced by Rust implementation)
        self.cortical_histories: Dict[int, CorticalHistory] = {}
        self.memory_areas: Dict[int, MemoryArea] = {}
        self.current_timestep = 0
        
        # Performance tracking
        self._archive_operations = 0
        self._query_operations = 0
        
        # Try to initialize Rust backend
        if fire_ledger_rs:
            self._rust_ledger = fire_ledger_rs.FireLedger(default_window_size)
            logger.info("Fire Ledger initialized with Rust backend")
        else:
            self._rust_ledger = None
            logger.warning("Rust Fire Ledger not available - using Python fallback")
    
    def archive_timestep(self, 
                        timestep: int,
                        neurons_by_area: Dict[int, List[FiringNeuron]]):
        """Archive firing data from Fire Queue for historical access.
        
        This is called after each burst to preserve firing data for STDP and memory.
        
        Args:
            timestep: Current timestep
            neurons_by_area: Firing neurons organized by cortical area
        """
        self.current_timestep = timestep
        self._archive_operations += 1
        
        if self._rust_ledger:
            # Use Rust implementation when available
            rust_data = {area_idx: [n.neuron_id for n in neurons] 
                        for area_idx, neurons in neurons_by_area.items()}
            self._rust_ledger.archive_firing_data(timestep, rust_data)
        else:
            # Python fallback implementation
            for area_idx, neurons in neurons_by_area.items():
                neuron_ids = [n.neuron_id for n in neurons]
                self._archive_area_timestep(area_idx, timestep, neuron_ids)
        
        # Timestep archived to ledger
    
    def _archive_area_timestep(self, area_idx: int, timestep: int, neuron_ids: List[int]):
        """Archive firing data for a specific cortical area (Python implementation)."""
        if area_idx not in self.cortical_histories:
            window_size = self._get_area_window_size(area_idx)
            self.cortical_histories[area_idx] = CorticalHistory(window_size)
            
        self.cortical_histories[area_idx].add_timestep(timestep, neuron_ids)
    
    def get_firing_history(self, 
                          cortical_idx: int, 
                          lookback_steps: int) -> RoaringBitmap:
        """Get historical firing pattern for STDP processing.
        
        Args:
            cortical_idx: Cortical area index
            lookback_steps: Number of timesteps to look back
            
        Returns:
            RoaringBitmap of neurons that fired in the lookback period
        """
        self._query_operations += 1
        
        if self._rust_ledger:
            # Use Rust implementation when available
            return self._rust_ledger.get_stdp_pattern(cortical_idx, lookback_steps)
        else:
            # Python fallback
            if cortical_idx not in self.cortical_histories:
                return RoaringBitmap()
            return self.cortical_histories[cortical_idx].get_pattern(lookback_steps)
    
    def get_timestep_pattern(self, 
                           cortical_idx: int, 
                           timestep_offset: int = -1) -> RoaringBitmap:
        """Get firing pattern for specific timestep offset.
        
        Args:
            cortical_idx: Cortical area index  
            timestep_offset: Offset from current timestep (negative for past)
            
        Returns:
            RoaringBitmap of neurons that fired at that timestep
        """
        self._query_operations += 1
        
        if cortical_idx not in self.cortical_histories:
            return RoaringBitmap()
        return self.cortical_histories[cortical_idx].get_timestep_pattern(timestep_offset)
    
    def configure_area_window(self, cortical_idx: int, window_size: int):
        """Configure custom window size for cortical area.
        
        Different areas may need different history lengths based on:
        - STDP requirements
        - Memory formation characteristics  
        - Computational constraints
        """
        if self._rust_ledger:
            self._rust_ledger.configure_area_window(cortical_idx, window_size)
        else:
            # Recreate history with new window size
            old_history = self.cortical_histories.get(cortical_idx)
            new_history = CorticalHistory(window_size)
            
            if old_history:
                # Preserve recent data
                steps_to_preserve = min(window_size, len(old_history.firing_history))
                for i in range(steps_to_preserve):
                    new_history.firing_history.append(old_history.firing_history[-(i+1)])
                new_history.current_timestep = old_history.current_timestep
                    
            self.cortical_histories[cortical_idx] = new_history
            
        # Window size configured for cortical area
    
    def configure_memory_area(self,
                             cortical_idx: int,
                             window_size: int,
                             upstream_areas: List[int]):
        """Configure a memory area with enhanced temporal processing.
        
        Memory areas have longer history windows and track upstream area patterns.
        """
        if self._rust_ledger:
            self._rust_ledger.configure_memory_area(cortical_idx, window_size, upstream_areas)
        else:
            memory_area = MemoryArea(cortical_idx, window_size, upstream_areas)
            self.memory_areas[cortical_idx] = memory_area
            
            # Also configure regular history
            self.configure_area_window(cortical_idx, window_size)
            
        # Memory area configured with upstream connections
    
    def get_memory_pattern(self, 
                          memory_area_idx: int, 
                          upstream_area_idx: int,
                          lookback_steps: int) -> Tuple[RoaringBitmap, RoaringBitmap]:
        """Get correlated patterns between memory area and upstream area.
        
        Returns:
            Tuple of (memory_pattern, upstream_pattern) for STDP correlation analysis
        """
        memory_pattern = self.get_firing_history(memory_area_idx, lookback_steps)
        upstream_pattern = self.get_firing_history(upstream_area_idx, lookback_steps)
        return memory_pattern, upstream_pattern
    
    def cleanup_old_data(self, current_timestep: int):
        """Clean up expired historical data (Rust-optimized when available)."""
        if self._rust_ledger:
            self._rust_ledger.cleanup_old_data(current_timestep)
        else:
            # Python cleanup is handled automatically by deque maxlen
            pass
    
    def get_statistics(self) -> Dict:
        """Get fire ledger statistics for monitoring."""
        area_count = len(self.cortical_histories)
        memory_area_count = len(self.memory_areas)
        
        total_neurons_tracked = 0
        for history in self.cortical_histories.values():
            for bitmap in history.firing_history:
                total_neurons_tracked += len(bitmap)
        
        return {
            'current_timestep': self.current_timestep,
            'tracked_areas': area_count,
            'memory_areas': memory_area_count,
            'archive_operations': self._archive_operations,
            'query_operations': self._query_operations,
            'total_neurons_tracked': total_neurons_tracked,
            'using_rust_backend': self._rust_ledger is not None
        }
    
    def _get_area_window_size(self, area_idx: int) -> int:
        """Get window size for cortical area (with memory area support)."""
        if area_idx in self.memory_areas:
            return self.memory_areas[area_idx].window_size
        return self.default_window_size
    
    def get_area_activity(self, area_idx: int, timestep: int) -> Optional['RoaringBitmap']:
        """
        Get activity bitmap for any area type at specific timestep.
        
        Supports both regular and memory areas with unified ID space.
        Used by memory pattern detection system.
        
        Args:
            area_idx: Cortical area index (regular or memory)
            timestep: Absolute timestep to query
            
        Returns:
            RoaringBitmap of active neuron IDs, or None if no data
        """
        from .plasticity.neuron_id_manager import NeuronIdManager
        
        # Calculate timestep offset from current
        if timestep > self.current_timestep:
            return RoaringBitmap()  # Future timestep
        
        timestep_offset = timestep - self.current_timestep
        
        # Handle memory areas (may have memory neurons with different ID space)
        if NeuronIdManager.is_memory_neuron_id(area_idx):
            # This is a memory area - get memory neuron activity
            # Memory neurons are managed separately and have global unique IDs
            return self._get_memory_area_activity(area_idx, timestep_offset)
        else:
            # Regular cortical area
            return self.get_timestep_pattern(area_idx, timestep_offset)
    
    def _get_memory_area_activity(self, memory_area_idx: int, timestep_offset: int) -> 'RoaringBitmap':
        """
        Get memory neuron activity for memory area.
        
        Memory neurons are stored separately and need special handling.
        """
        # For now, return empty - this will be populated when memory neurons
        # are created and their activity is tracked
        return RoaringBitmap()
    
    def get_combined_upstream_activity(
        self, 
        upstream_areas: List[int], 
        timestep: int
    ) -> 'RoaringBitmap':
        """
        Get combined activity from multiple upstream areas at timestep.
        
        Optimized for memory pattern detection with SIMD bitmap operations.
        
        Args:
            upstream_areas: List of upstream cortical area indices
            timestep: Absolute timestep to query
            
        Returns:
            Combined RoaringBitmap with union of all upstream activity
        """
        combined_bitmap = RoaringBitmap()
        
        for area_idx in upstream_areas:
            area_activity = self.get_area_activity(area_idx, timestep)
            if area_activity and not area_activity.is_empty():
                combined_bitmap = combined_bitmap.union(area_activity)
        
        return combined_bitmap
    
    def get_temporal_pattern_sequence(
        self,
        upstream_areas: List[int],
        current_timestep: int,
        temporal_depth: int
    ) -> List['RoaringBitmap']:
        """
        Get sequence of combined upstream activity for temporal pattern detection.
        
        Optimized for memory formation pattern extraction.
        
        Args:
            upstream_areas: List of upstream area indices
            current_timestep: Current simulation timestep
            temporal_depth: Number of timesteps to look back
            
        Returns:
            List of RoaringBitmaps in temporal order [t, t-1, t-2, ...]
        """
        pattern_sequence = []
        
        for offset in range(temporal_depth):
            timestep = current_timestep - offset
            combined_activity = self.get_combined_upstream_activity(upstream_areas, timestep)
            pattern_sequence.append(combined_activity)
        
        return pattern_sequence
    
    def get_active_areas(self) -> List[int]:
        """Get list of cortical areas with historical data."""
        return list(self.cortical_histories.keys())
    
    def is_memory_area(self, area_idx: int) -> bool:
        """Check if cortical area is configured as memory area."""
        return area_idx in self.memory_areas
    
    def get_memory_area_config(self, area_idx: int) -> Optional[Dict]:
        """
        Get memory area configuration.
        
        Returns:
            Dict with window_size and upstream_areas, or None if not memory area
        """
        if area_idx not in self.memory_areas:
            return None
        
        memory_area = self.memory_areas[area_idx]
        return {
            'window_size': memory_area.window_size,
            'upstream_areas': memory_area.upstream_areas.copy(),
            'cortical_idx': memory_area.cortical_idx
        }
