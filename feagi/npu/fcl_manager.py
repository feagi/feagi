"""
Fire Candidate List (FCL) Manager Implementation

This module implements the hierarchical Fire Candidate List (FCL) management system,
which tracks neuron activations while preserving their cortical area associations.
The implementation uses Roaring Bitmaps for efficient storage and operations.
"""

from typing import Dict, List, Set, Optional, Union, Tuple, Any, Protocol, TypeVar, Generic, cast, Iterator
from enum import Enum
import logging
from collections import defaultdict
from dataclasses import dataclass
try:
    import pyroaring
    PYROARING_AVAILABLE = True
except ImportError:
    logging.warning("PyRoaring not found. FCL will use slower fallback implementation.")
    PYROARING_AVAILABLE = False
    pyroaring = None

# Type definitions to make migration to Rust more straightforward
NeuronId = int
AreaId = int  # Alias for backward compatibility
CorticalIdx = int  # Changed from AreaId to CorticalIdx for consistency
Timestep = int


class FCLError(Exception):
    """Base error class for FCL operations."""
    pass


class TimestepOutOfRangeError(FCLError):
    """Error raised when a requested timestep is outside the available window."""
    pass


@dataclass
class MembraneUpdate:
    """Represents a pending update to a neuron's membrane potential."""
    neuron_idx: int
    delta_potential: float
    source_neuron_idx: Optional[int] = None  # Source of the update (for tracing)


# Define a Protocol for bitmap-like objects (similar to Rust traits)
class BitMapProtocol(Protocol):
    """Protocol defining the interface for bitmap-like objects."""
    
    def __init__(self, elements=None): ...
    
    def add(self, element: int) -> None: ...
    
    def clear(self) -> None: ...
    
    def copy(self) -> 'BitMapProtocol': ...
    
    def __or__(self, other: 'BitMapProtocol') -> 'BitMapProtocol': ...
    
    def __and__(self, other: 'BitMapProtocol') -> 'BitMapProtocol': ...
    
    def __sub__(self, other: 'BitMapProtocol') -> 'BitMapProtocol': ...
    
    def __xor__(self, other: 'BitMapProtocol') -> 'BitMapProtocol': ...
    
    def __len__(self) -> int: ...
    
    def __iter__(self) -> Iterator[int]: ...
    
    def __contains__(self, item: int) -> bool: ...
    
    def is_empty(self) -> bool: ...


# Fallback implementation for environments without pyroaring
class FallbackBitMap:
    """Simple set-based fallback for environments without pyroaring."""
    def __init__(self, elements=None):
        self.elements: Set[int] = set(elements) if elements else set()
        
    def add(self, element: int) -> None:
        self.elements.add(element)
        
    def clear(self) -> None:
        self.elements.clear()
        
    def copy(self) -> 'FallbackBitMap':
        result = FallbackBitMap()
        result.elements = self.elements.copy()
        return result
        
    def __or__(self, other: 'FallbackBitMap') -> 'FallbackBitMap':
        result = self.copy()
        result.elements |= other.elements
        return result
    
    def __and__(self, other: 'FallbackBitMap') -> 'FallbackBitMap':
        result = self.copy()
        result.elements &= other.elements
        return result
    
    def __sub__(self, other: 'FallbackBitMap') -> 'FallbackBitMap':
        result = self.copy()
        result.elements -= other.elements
        return result
    
    def __xor__(self, other: 'FallbackBitMap') -> 'FallbackBitMap':
        result = self.copy()
        result.elements ^= other.elements
        return result
    
    def __len__(self) -> int:
        return len(self.elements)
    
    def __iter__(self) -> Iterator[int]:
        return iter(self.elements)
    
    def __contains__(self, item: int) -> bool:
        return item in self.elements
    
    def is_empty(self) -> bool:
        return len(self.elements) == 0
    
    def __repr__(self) -> str:
        return repr(self.elements)


# Choose bitmap implementation based on availability
# This makes it easier to create a Rust-specific implementation later
if PYROARING_AVAILABLE:
    # Wrapper to ensure PyRoaring matches our Protocol
    class RoaringBitmap:
        """Wrapper for PyRoaring bitmap with consistent interface."""
        
        def __init__(self, elements=None):
            self._bitmap = pyroaring.BitMap(elements) if elements is not None else pyroaring.BitMap()
            
        def add(self, element: int) -> None:
            self._bitmap.add(element)
            
        def clear(self) -> None:
            self._bitmap.clear()
            
        def copy(self) -> 'RoaringBitmap':
            result = RoaringBitmap()
            result._bitmap = self._bitmap.copy()
            return result
            
        def __or__(self, other: 'RoaringBitmap') -> 'RoaringBitmap':
            result = RoaringBitmap()
            result._bitmap = self._bitmap | other._bitmap
            return result
        
        def __and__(self, other: 'RoaringBitmap') -> 'RoaringBitmap':
            result = RoaringBitmap()
            result._bitmap = self._bitmap & other._bitmap
            return result
        
        def __sub__(self, other: 'RoaringBitmap') -> 'RoaringBitmap':
            result = RoaringBitmap()
            result._bitmap = self._bitmap - other._bitmap
            return result
        
        def __xor__(self, other: 'RoaringBitmap') -> 'RoaringBitmap':
            result = RoaringBitmap()
            result._bitmap = self._bitmap ^ other._bitmap
            return result
        
        def __len__(self) -> int:
            return len(self._bitmap)
        
        def __iter__(self) -> Iterator[int]:
            return iter(self._bitmap)
        
        def __contains__(self, item: int) -> bool:
            return item in self._bitmap
        
        def is_empty(self) -> bool:
            return len(self._bitmap) == 0
        
        def __repr__(self) -> str:
            return repr(self._bitmap)
            
    BitMap = RoaringBitmap
else:
    BitMap = FallbackBitMap


# Enum for neuron collections to enable static type checking
class NeuronCollectionType(Enum):
    """Types of neuron collection inputs that can be converted to bitmaps."""
    BITMAP = "bitmap"
    LIST = "list"
    SET = "set"


# Wrapper for neuron collections to make type handling explicit
@dataclass
class NeuronCollection:
    """Container for different types of neuron collections with explicit type."""
    collection_type: NeuronCollectionType
    data: Union[BitMap, List[int], Set[int]]
    
    @classmethod
    def from_any(cls, data: Union[BitMap, List[int], Set[int]]) -> 'NeuronCollection':
        """Create a NeuronCollection from any supported data type."""
        if isinstance(data, BitMap):
            return cls(NeuronCollectionType.BITMAP, data)
        elif isinstance(data, list):
            return cls(NeuronCollectionType.LIST, data)
        elif isinstance(data, set):
            return cls(NeuronCollectionType.SET, data)
        else:
            raise TypeError(f"Unsupported neuron collection type: {type(data)}")
    
    def to_bitmap(self) -> BitMap:
        """Convert the collection to a bitmap."""
        if self.collection_type == NeuronCollectionType.BITMAP:
            return cast(BitMap, self.data)
        else:
            return BitMap(self.data)


class HierarchicalFCL:
    """
    Hierarchical Fire Candidate List Manager
    
    This class maintains a record of firing neurons while preserving their
    cortical area associations, enabling efficient area-based queries.
    It uses Roaring Bitmaps for memory-efficient storage and fast operations.
    """
    
    def __init__(self, window_size: int = 20):
        """
        Initialize a hierarchical FCL that tracks both neurons and their cortical areas.
        
        Args:
            window_size: Number of timesteps to maintain in history
        """
        self.window_size: int = window_size
        # Main FCL history - stores all neurons regardless of area
        self.global_fcl_history: List[BitMap] = [BitMap() for _ in range(window_size)]
        # Area-specific FCL history - mapping from cortical_idx to list of bitmaps
        self.area_fcl_history: Dict[CorticalIdx, List[BitMap]] = {}
        self.current_window_index: int = 0
        self.current_timestep: int = 0
        
        # Stats
        self.total_neurons_fired: int = 0
        self.neurons_per_area: Dict[CorticalIdx, int] = {}
        
        # Logger
        self.logger = logging.getLogger("feagi.npu.fcl_manager")
        
    def _ensure_area_initialized(self, cortical_idx: CorticalIdx) -> None:
        """
        Ensure an area is initialized in the FCL history.
        
        Args:
            cortical_idx: ID of the cortical area to initialize
        """
        if cortical_idx not in self.area_fcl_history:
            self.area_fcl_history[cortical_idx] = [BitMap() for _ in range(self.window_size)]
    
    def _get_index_for_timestep(self, timestep: Optional[int] = None) -> int:
        """
        Get the correct index in the FCL history for a given timestep.
        
        Args:
            timestep: Timestep to get index for, or None for current timestep
            
        Returns:
            Index in the FCL history arrays
            
        Raises:
            TimestepOutOfRangeError: If timestep is outside valid range
        """
        if timestep is None:
            return self.current_window_index
            
        # Check if timestep is within valid range
        current_time = self.current_timestep
        oldest_time = current_time - self.window_size + 1
        if timestep < oldest_time or timestep > current_time:
            raise TimestepOutOfRangeError(
                f"Timestep {timestep} is outside valid range [{oldest_time}, {current_time}]"
            )
            
        return timestep % self.window_size
        
    def update_fcl(self, current_timestep: int, neurons_by_area: Dict[CorticalIdx, Union[BitMap, List[int], Set[int]]]) -> None:
        """
        Update the FCL with neurons firing in the current timestep, preserving area information.
        
        Args:
            current_timestep: Current simulation timestep
            neurons_by_area: Dictionary mapping cortical_idx -> list/set/bitmap of neuron_ids
        """
        self.current_timestep = current_timestep
        current_index = current_timestep % self.window_size
        
        # Clear the oldest bitmaps for reuse
        self.global_fcl_history[current_index].clear()
        
        # Track firing statistics
        burst_total = 0
        
        # Update area-specific FCLs
        for cortical_idx, neuron_ids in neurons_by_area.items():
            # Create area bitmap if not exists
            self._ensure_area_initialized(cortical_idx)
            
            # Clear the oldest bitmap for this area
            self.area_fcl_history[cortical_idx][current_index].clear()
            
            # Convert to bitmap using the wrapper
            neuron_collection = NeuronCollection.from_any(neuron_ids)
            area_bitmap = neuron_collection.to_bitmap()
                
            # Count neurons in this area
            area_neuron_count = len(area_bitmap)
            burst_total += area_neuron_count
            self.neurons_per_area[cortical_idx] = area_neuron_count
                
            # Update area-specific bitmap
            self.area_fcl_history[cortical_idx][current_index] = area_bitmap
            
            # Update global bitmap (union of all areas)
            self.global_fcl_history[current_index] = self.global_fcl_history[current_index] | area_bitmap
        
        # Update total count and window index
        self.total_neurons_fired = burst_total
        self.current_window_index = current_index
        
        self.logger.debug(f"FCL updated for timestep {current_timestep}: {burst_total} neurons fired across {len(neurons_by_area)} areas")
    
    def get_global_fcl(self, timestep: Optional[int] = None) -> BitMap:
        """
        Get the complete FCL for a specific timestep.
        
        Args:
            timestep: Specific timestep to query (defaults to current)
            
        Returns:
            BitMap containing all firing neuron IDs
            
        Raises:
            TimestepOutOfRangeError: If timestep is outside valid range
        """
        index = self._get_index_for_timestep(timestep)
        return self.global_fcl_history[index].copy()
    
    def get_area_fcl(self, cortical_idx: CorticalIdx, timestep: Optional[int] = None) -> BitMap:
        """
        Get FCL for a specific cortical area at a specific timestep.
        
        Args:
            cortical_idx: ID of the cortical area to query
            timestep: Optional timestep to query (defaults to current timestep)
            
        Returns:
            BitMap of neurons firing in the specified area at the specified timestep
        """
        index = self._get_index_for_timestep(timestep)
        
        # Return empty bitmap if area not initialized
        if cortical_idx not in self.area_fcl_history:
            return BitMap()
            
        # Return copy to prevent modification of internal state
        return self.area_fcl_history[cortical_idx][index].copy()
    
    def get_fcl_by_area(self, timestep: Optional[int] = None) -> Dict[CorticalIdx, BitMap]:
        """
        Return a dictionary mapping each cortical area to its firing neurons.
        
        Args:
            timestep: Optional timestep to query (defaults to current timestep)
            
        Returns:
            Dict[cortical_idx, BitMap] mapping areas to their active neurons
        """
        index = self._get_index_for_timestep(timestep)
        
        result: Dict[CorticalIdx, BitMap] = {}
        for cortical_idx, fcl_history in self.area_fcl_history.items():
            if not fcl_history[index].is_empty():
                result[cortical_idx] = fcl_history[index].copy()
                
        return result
    
    def get_active_areas(self, timestep: Optional[int] = None) -> Set[CorticalIdx]:
        """
        Return a set of cortical indices that have any firing neurons.
        
        Args:
            timestep: Optional timestep to query (defaults to current timestep)
            
        Returns:
            Set of cortical indices with active neurons
        """
        index = self._get_index_for_timestep(timestep)
            
        result: Set[CorticalIdx] = set()
        for cortical_idx, fcl_history in self.area_fcl_history.items():
            if not fcl_history[index].is_empty():
                result.add(cortical_idx)
                
        return result
    
    def get_neurons_by_areas(self, area_ids: List[CorticalIdx], timestep: Optional[int] = None) -> BitMap:
        """
        Get neurons firing in any of the specified areas.
        
        Args:
            area_ids: List of cortical area IDs to query
            timestep: Optional timestep (defaults to current)
            
        Returns:
            BitMap of neuron IDs active in any of the specified areas
            
        Raises:
            TimestepOutOfRangeError: If timestep is outside valid range
        """
        index = self._get_index_for_timestep(timestep)
            
        result = BitMap()
        for area_id in area_ids:
            if area_id in self.area_fcl_history:
                result = result | self.area_fcl_history[area_id][index]
                
        return result
    
    def get_neurons_fired_in_last_n_steps(self, n_steps: int, area_ids: Optional[List[CorticalIdx]] = None) -> BitMap:
        """
        Get neurons that fired in any of the last n timesteps.
        Optionally filter by specific cortical areas.
        
        Args:
            n_steps: Number of timesteps to look back
            area_ids: Optional list of area IDs to filter by
            
        Returns:
            BitMap containing neuron IDs that fired in the specified timespan
            
        Raises:
            ValueError: If n_steps is negative or exceeds window size
        """
        if n_steps <= 0:
            raise ValueError(f"n_steps must be positive, got {n_steps}")
            
        if n_steps > self.window_size:
            n_steps = self.window_size
        
        if area_ids is None:
            # Use global FCL history if no area filtering
            result = BitMap()
            for i in range(n_steps):
                step_index = (self.current_window_index - i) % self.window_size
                result = result | self.global_fcl_history[step_index]
        else:
            # Filter by specified areas
            result = BitMap()
            for area_id in area_ids:
                if area_id in self.area_fcl_history:
                    for i in range(n_steps):
                        step_index = (self.current_window_index - i) % self.window_size
                        result = result | self.area_fcl_history[area_id][step_index]
        
        return result
    
    def get_consistently_active_neurons(self, n_steps: int, area_ids: Optional[List[CorticalIdx]] = None) -> BitMap:
        """
        Get neurons that fired in ALL of the last n timesteps.
        
        Args:
            n_steps: Number of timesteps to look back
            area_ids: Optional list of area IDs to filter by
            
        Returns:
            BitMap of neuron IDs that fired consistently across the timespan
            
        Raises:
            ValueError: If n_steps is negative or exceeds window size
        """
        if n_steps <= 0:
            raise ValueError(f"n_steps must be positive, got {n_steps}")
            
        if n_steps > self.window_size:
            n_steps = self.window_size
        
        # Initialize result with neurons from the first relevant timestep
        first_step_index = (self.current_window_index) % self.window_size
        
        if area_ids is None:
            # Start with all neurons from first timestep
            result = self.global_fcl_history[first_step_index].copy()
            
            # Intersect with each subsequent timestep
            for i in range(1, n_steps):
                step_index = (self.current_window_index - i) % self.window_size
                result = result & self.global_fcl_history[step_index]
        else:
            # Filter by specified areas
            first_step_neurons = self.get_neurons_by_areas(area_ids, timestep=self.current_timestep - n_steps + 1)
            result = first_step_neurons
            
            # Intersect with each subsequent timestep
            for i in range(1, n_steps):
                current_step = self.current_timestep - n_steps + 1 + i
                filtered_step = self.get_neurons_by_areas(area_ids, timestep=current_step)
                result = result & filtered_step
        
        return result
    
    def get_fcl_delta(self, start_time: int, end_time: int, area_ids: Optional[List[CorticalIdx]] = None) -> BitMap:
        """
        Get neurons that became active between start and end times.
        
        Args:
            start_time: Starting timestep
            end_time: Ending timestep
            area_ids: Optional list of cortical area IDs to filter by
            
        Returns:
            BitMap of neurons that became active between the two timesteps
            
        Raises:
            TimestepOutOfRangeError: If timesteps are outside valid range
            ValueError: If start_time is greater than end_time
        """
        if start_time > end_time:
            raise ValueError(f"start_time ({start_time}) must be <= end_time ({end_time})")
        
        time_diff = abs(end_time - start_time)
        if time_diff > self.window_size:
            raise TimestepOutOfRangeError(
                f"Time difference ({time_diff}) exceeds history window size ({self.window_size})"
            )
        
        start_index = self._get_index_for_timestep(start_time)
        end_index = self._get_index_for_timestep(end_time)
        
        if area_ids is None:
            # Neurons active at end but not at start (newly activated)
            return self.global_fcl_history[end_index] - self.global_fcl_history[start_index]
        else:
            # Filter by specified areas
            start_neurons = self.get_neurons_by_areas(area_ids, timestep=start_time)
            end_neurons = self.get_neurons_by_areas(area_ids, timestep=end_time)
            
            # Neurons active at end but not at start (newly activated)
            return end_neurons - start_neurons
    
    def get_fcl_xor(self, time1: int, time2: int, area_ids: Optional[List[CorticalIdx]] = None) -> BitMap:
        """
        Get neurons that fired at either time1 or time2, but not both.
        Useful for detecting changes in firing patterns.
        
        Args:
            time1: First timestep
            time2: Second timestep
            area_ids: Optional list of cortical area IDs to filter by
            
        Returns:
            BitMap of neurons that changed activation state between timesteps
            
        Raises:
            TimestepOutOfRangeError: If timesteps are outside valid range
        """
        idx1 = self._get_index_for_timestep(time1)
        idx2 = self._get_index_for_timestep(time2)
        
        if area_ids is None:
            return self.global_fcl_history[idx1] ^ self.global_fcl_history[idx2]
        else:
            neurons1 = self.get_neurons_by_areas(area_ids, timestep=time1)
            neurons2 = self.get_neurons_by_areas(area_ids, timestep=time2)
            
            return neurons1 ^ neurons2
    
    def count_firing_neurons(self, timestep: Optional[int] = None, area_id: Optional[CorticalIdx] = None) -> int:
        """
        Efficiently count the number of firing neurons.
        
        Args:
            timestep: Optional timestep (defaults to current)
            area_id: Optional area ID to count neurons for a specific area
            
        Returns:
            Count of firing neurons
            
        Raises:
            TimestepOutOfRangeError: If timestep is outside valid range
        """
        if timestep is None and area_id is None:
            # Use cached total for current timestep
            return self.total_neurons_fired
        
        if timestep is None:
            timestep = self.current_timestep
            
        index = self._get_index_for_timestep(timestep)
            
        if area_id is None:
            # Count all neurons at the specified timestep
            return len(self.global_fcl_history[index])
        else:
            # Count neurons in the specified area
            if area_id in self.area_fcl_history:
                return len(self.area_fcl_history[area_id][index])
            return 0
    
    def get_firing_statistics(self) -> Dict[str, Any]:
        """
        Get statistics about the current FCL state.
        
        Returns:
            Dictionary with statistics about firing neurons and active areas
        """
        active_areas = self.get_active_areas()
        
        return {
            "timestep": self.current_timestep,
            "total_neurons_fired": self.total_neurons_fired,
            "active_areas_count": len(active_areas),
            "active_areas": list(active_areas),
            "neurons_per_area": dict(self.neurons_per_area),
            "window_size": self.window_size
        }

    # Methods for two-phase membrane potential update process
    
    def advance_timestep(self) -> None:
        """
        Advance to the next timestep, shifting FCL history.
        """
        self.current_timestep += 1
        self.current_window_index = (self.current_window_index + 1) % self.window_size
        
        # Clear the current FCL slot (both global and per-area)
        self.global_fcl_history[self.current_window_index].clear()
        
        for area_id in self.area_fcl_history:
            self.area_fcl_history[area_id][self.current_window_index].clear()
            
        # Reset update queue for new timestep
        self._reset_update_queue()
    
    def _reset_update_queue(self) -> None:
        """Reset the membrane potential update queue for a new timestep."""
        if not hasattr(self, 'mp_update_queue'):
            self.mp_update_queue: List[MembraneUpdate] = []
            self.updates_processed_count = 0
        else:
            self.mp_update_queue = []
    
    def queue_membrane_update(self, 
                             neuron_idx: int, 
                             delta_potential: float,
                             source_neuron_idx: Optional[int] = None) -> None:
        """
        Queue an update to a neuron's membrane potential.
        
        Args:
            neuron_idx: Index of the neuron to update
            delta_potential: Change in membrane potential
            source_neuron_idx: Source of the update (if from a specific neuron)
        """
        if not hasattr(self, 'mp_update_queue'):
            self._reset_update_queue()
            
        self.mp_update_queue.append(
            MembraneUpdate(
                neuron_idx=neuron_idx,
                delta_potential=delta_potential,
                source_neuron_idx=source_neuron_idx
            )
        )
        self.updates_processed_count += 1
    
    def process_update_queue(self) -> List[Tuple[int, float]]:
        """
        Process the membrane potential update queue, aggregating updates per neuron.
        
        Returns:
            List of (neuron_idx, total_delta) tuples with the final updates
        """
        if not hasattr(self, 'mp_update_queue') or not self.mp_update_queue:
            return []
        
        # Aggregate updates by neuron
        neuron_updates: Dict[int, float] = {}
        
        for update in self.mp_update_queue:
            neuron_updates[update.neuron_idx] = (
                neuron_updates.get(update.neuron_idx, 0.0) + update.delta_potential
            )
        
        # Convert to list of tuples
        aggregated_updates = [(idx, delta) for idx, delta in neuron_updates.items()]
        
        # Clear the queue after processing
        self.mp_update_queue = []
        
        return aggregated_updates
    
    def get_firing_neurons(self, offset: int = -1) -> List[int]:
        """
        Get list of neuron indices that are in the FCL at the specified offset.
        
        Args:
            offset: Timestep offset (-1 for previous timestep, which is the firing phase)
            
        Returns:
            List of neuron indices
        """
        fcl = self.get_global_fcl(self.current_timestep + offset if offset else None)
        return list(fcl)
    
    def add_to_current_fcl(self, neuron_indices: Union[List[int], Set[int], BitMap]) -> None:
        """
        Add neurons to the current timestep's FCL.
        
        Args:
            neuron_indices: Indices of neurons to add to current FCL
        """
        # Convert to BitMap if needed
        if not isinstance(neuron_indices, BitMap):
            neuron_bitmap = BitMap(neuron_indices)
        else:
            neuron_bitmap = neuron_indices
            
        # Add to global FCL
        current_fcl = self.global_fcl_history[self.current_window_index]
        # Use BitMap.__or__ to update in place
        updated_fcl = current_fcl | neuron_bitmap
        self.global_fcl_history[self.current_window_index] = updated_fcl
        
        # Update statistics
        self.total_neurons_fired += len(neuron_bitmap)
    
    def get_fcl(self, offset: int = 0) -> BitMap:
        """
        Get the FCL for a specific timestep relative to current.
        
        Args:
            offset: Timestep offset (0 for current, -1 for previous, etc.)
            
        Returns:
            BitMap of neuron indices in the FCL
        """
        return self.get_global_fcl(self.current_timestep + offset if offset else None)


class EnhancedHierarchicalFCL:
    """
    Enhanced Hierarchical Fire Candidate List Manager with support for memory cortical areas
    
    This class extends the HierarchicalFCL functionality to support custom window sizes
    for memory-type cortical areas, enabling longer temporal pattern analysis for specific
    brain regions while maintaining memory efficiency.
    """
    
    def __init__(self, default_window_size: int = 20):
        """
        Initialize an enhanced hierarchical FCL with support for area-specific window sizes.
        
        Args:
            default_window_size: Default number of timesteps to maintain for standard areas
        """
        self.default_window_size: int = default_window_size
        self.global_fcl_history: List[BitMap] = [BitMap() for _ in range(default_window_size)]
        
        # Main FCL history - stores all neurons regardless of area
        self.current_window_index: int = 0
        self.current_timestep: int = 0
        
        # Standard area storage using default window size
        self.area_fcl_history: Dict[CorticalIdx, List[BitMap]] = {}
        
        # Storage for areas with custom window sizes
        # Maps area_id -> (window_size, [BitMaps], start_timestep)
        self.custom_area_history: Dict[CorticalIdx, Tuple[int, List[BitMap], int]] = {}
        
        # Track area types for quick lookup
        self.memory_area_ids: Set[CorticalIdx] = set()
        
        # Stats
        self.total_neurons_fired: int = 0
        self.neurons_per_area: Dict[CorticalIdx, int] = {}
        
        # Logger
        self.logger = logging.getLogger("feagi.npu.fcl_manager.enhanced")
        
    def register_memory_area(self, cortical_idx: CorticalIdx, window_size: int) -> None:
        """
        Register a memory-type cortical area with a custom window size.
        
        Args:
            cortical_idx: ID of the memory-type cortical area
            window_size: Custom history window size for this area (must be >= default_window_size)
            
        Raises:
            ValueError: If window_size is less than default_window_size
        """
        if window_size < self.default_window_size:
            raise ValueError(f"Custom window size ({window_size}) must be >= default window size ({self.default_window_size})")
            
        # Initialize with empty bitmaps
        history_array = [BitMap() for _ in range(window_size)]
        
        # Store area with custom settings
        # The third element is the start timestep, initialized to current timestep
        self.custom_area_history[cortical_idx] = (window_size, history_array, self.current_timestep)
        self.memory_area_ids.add(cortical_idx)
        
        self.logger.info(f"Registered memory area {cortical_idx} with window size {window_size}")
        
    def is_memory_area(self, cortical_idx: CorticalIdx) -> bool:
        """
        Check if an area is registered as a memory-type area with custom window size.
        
        Args:
            cortical_idx: Area ID to check
            
        Returns:
            True if this is a memory area with custom window size, False otherwise
        """
        return cortical_idx in self.memory_area_ids
        
    def get_area_window_size(self, cortical_idx: CorticalIdx) -> int:
        """
        Get the window size for a specific area.
        
        Args:
            cortical_idx: ID of the cortical area
            
        Returns:
            Window size for the specified area (custom size for memory areas, default size for others)
        """
        if cortical_idx in self.custom_area_history:
            return self.custom_area_history[cortical_idx][0]
        return self.default_window_size
        
    def _get_custom_area_index(self, cortical_idx: CorticalIdx, timestep: int) -> int:
        """
        Calculate the correct index in the custom window size history for a given area and timestep.
        
        Args:
            cortical_idx: ID of the memory-type cortical area
            timestep: Specific timestep to get index for
            
        Returns:
            Index in the area's custom-sized history array
            
        Raises:
            ValueError: If cortical_idx is not a registered memory area
            TimestepOutOfRangeError: If timestep is outside valid range for this area
        """
        if cortical_idx not in self.custom_area_history:
            raise ValueError(f"Area {cortical_idx} is not registered as a memory area")
            
        window_size, history_array, start_timestep = self.custom_area_history[cortical_idx]
        
        # Check if timestep is within valid range for this area
        oldest_time = max(start_timestep, self.current_timestep - window_size + 1)
        if timestep < oldest_time or timestep > self.current_timestep:
            raise TimestepOutOfRangeError(
                f"Timestep {timestep} is outside valid range [{oldest_time}, {self.current_timestep}] for area {cortical_idx}"
            )
            
        # Convert relative to start_timestep, then modulo window_size
        relative_timestep = timestep - start_timestep
        return relative_timestep % window_size
    
    def _get_index_for_timestep(self, timestep: Optional[int] = None) -> int:
        """
        Get the correct index in the standard FCL history for a given timestep.
        
        Args:
            timestep: Timestep to get index for, or None for current timestep
            
        Returns:
            Index in the standard FCL history arrays
            
        Raises:
            TimestepOutOfRangeError: If timestep is outside valid range
        """
        if timestep is None:
            return self.current_window_index
            
        # Check if timestep is within valid range
        current_time = self.current_timestep
        oldest_time = current_time - self.default_window_size + 1
        if timestep < oldest_time or timestep > current_time:
            raise TimestepOutOfRangeError(
                f"Timestep {timestep} is outside valid range [{oldest_time}, {current_time}]"
            )
            
        return timestep % self.default_window_size
     
    def update_fcl(self, current_timestep: int, neurons_by_area: Dict[CorticalIdx, Union[BitMap, List[int], Set[int]]]) -> None:
        """
        Update the FCL with neurons firing in the current timestep, with support for memory areas.
        
        Args:
            current_timestep: Current simulation timestep
            neurons_by_area: Dictionary mapping cortical_idx -> list/set/bitmap of neuron_ids
        """
        self.current_timestep = current_timestep
        standard_index = current_timestep % self.default_window_size
        
        # Clear the oldest global bitmap for reuse
        self.global_fcl_history[standard_index].clear()
        
        # Track firing statistics
        burst_total = 0
        
        # Process each area
        for cortical_idx, neuron_ids in neurons_by_area.items():
            # Convert to bitmap if needed
            neuron_collection = NeuronCollection.from_any(neuron_ids)
            area_bitmap = neuron_collection.to_bitmap()
            
            # Count neurons in this area
            area_neuron_count = len(area_bitmap)
            burst_total += area_neuron_count
            self.neurons_per_area[cortical_idx] = area_neuron_count
            
            # Check if this is a memory area with custom window size
            if self.is_memory_area(cortical_idx):
                window_size, history_array, start_timestep = self.custom_area_history[cortical_idx]
                custom_index = self._get_custom_area_index(cortical_idx, current_timestep)
                
                # Clear the bitmap at the current position
                history_array[custom_index].clear()
                # Update with new neurons
                history_array[custom_index] = area_bitmap
            else:
                # Standard area processing
                if cortical_idx not in self.area_fcl_history:
                    self.area_fcl_history[cortical_idx] = [BitMap() for _ in range(self.default_window_size)]
                    
                # Clear and update the standard area bitmap
                self.area_fcl_history[cortical_idx][standard_index].clear()
                self.area_fcl_history[cortical_idx][standard_index] = area_bitmap
            
            # Always update the global FCL (for all areas)
            self.global_fcl_history[standard_index] = self.global_fcl_history[standard_index] | area_bitmap
        
        # Update total count and window index
        self.total_neurons_fired = burst_total
        self.current_window_index = standard_index
        
        self.logger.debug(f"FCL updated for timestep {current_timestep}: {burst_total} neurons fired across {len(neurons_by_area)} areas")
    
    def get_global_fcl(self, timestep: Optional[int] = None) -> BitMap:
        """
        Get the complete FCL for a specific timestep.
        
        Args:
            timestep: Specific timestep to query (defaults to current)
            
        Returns:
            BitMap containing all firing neuron IDs
            
        Raises:
            TimestepOutOfRangeError: If timestep is outside valid range
        """
        index = self._get_index_for_timestep(timestep)
        return self.global_fcl_history[index].copy()
    
    def get_area_fcl(self, cortical_idx: CorticalIdx, timestep: Optional[int] = None) -> BitMap:
        """
        Get FCL for a specific cortical area, handling both standard and memory areas.
        
        Args:
            cortical_idx: ID of the cortical area to query
            timestep: Specific timestep (defaults to current)
            
        Returns:
            BitMap containing firing neuron IDs in the specified area
            
        Raises:
            TimestepOutOfRangeError: If timestep is outside valid range
        """
        if timestep is None:
            timestep = self.current_timestep
            
        # Check if this is a memory area with custom window size
        if self.is_memory_area(cortical_idx):
            custom_index = self._get_custom_area_index(cortical_idx, timestep)
            window_size, history_array, start_timestep = self.custom_area_history[cortical_idx]
            return history_array[custom_index].copy()
        else:
            # Use standard area processing
            standard_index = self._get_index_for_timestep(timestep)
            
            if cortical_idx not in self.area_fcl_history:
                return BitMap()
                
            return self.area_fcl_history[cortical_idx][standard_index].copy()
    
    def get_fcl_by_area(self, timestep: Optional[int] = None) -> Dict[CorticalIdx, BitMap]:
        """
        Return a dictionary mapping each cortical area to its firing neurons.
        
        Args:
            timestep: Optional timestep (defaults to current)
            
        Returns:
            Dict[cortical_idx, BitMap] mapping areas to their active neurons
            
        Raises:
            TimestepOutOfRangeError: If timestep is outside valid range
        """
        if timestep is None:
            timestep = self.current_timestep
        
        standard_index = self._get_index_for_timestep(timestep)
        
        result: Dict[CorticalIdx, BitMap] = {}
        
        # Process standard areas
        for cortical_idx, fcl_history in self.area_fcl_history.items():
            if not fcl_history[standard_index].is_empty():
                result[cortical_idx] = fcl_history[standard_index].copy()
        
        # Process memory areas with custom window sizes
        for cortical_idx in self.memory_area_ids:
            try:
                custom_index = self._get_custom_area_index(cortical_idx, timestep)
                window_size, history_array, start_timestep = self.custom_area_history[cortical_idx]
                bitmap = history_array[custom_index]
                if not bitmap.is_empty():
                    result[cortical_idx] = bitmap.copy()
            except TimestepOutOfRangeError:
                # Skip this area if the timestep is out of range for it
                pass
                
        return result
    
    def get_active_areas(self, timestep: Optional[int] = None) -> Set[CorticalIdx]:
        """
        Return a set of cortical indices that have any firing neurons.
        
        Args:
            timestep: Optional timestep (defaults to current)
            
        Returns:
            Set of cortical indices with active neurons
        """
        if timestep is None:
            timestep = self.current_timestep
        
        fcl_by_area = self.get_fcl_by_area(timestep=timestep)
        return set(fcl_by_area.keys())
    
    def get_neurons_by_areas(self, area_ids: List[CorticalIdx], timestep: Optional[int] = None) -> BitMap:
        """
        Get neurons firing in any of the specified areas.
        
        Args:
            area_ids: List of cortical area IDs to query
            timestep: Optional timestep (defaults to current)
            
        Returns:
            BitMap of neuron IDs active in any of the specified areas
            
        Raises:
            TimestepOutOfRangeError: If timestep is outside valid range
        """
        result = BitMap()
        
        for area_id in area_ids:
            try:
                area_neurons = self.get_area_fcl(area_id, timestep)
                result = result | area_neurons
            except TimestepOutOfRangeError:
                # Skip this area if the timestep is out of range for it
                pass
                
        return result
    
    def get_area_temporal_pattern(self, cortical_idx: CorticalIdx, n_steps: int) -> BitMap:
        """
        Get neurons in a memory area that fired in the last n timesteps.
        Optimized for memory areas with longer history windows.
        
        Args:
            cortical_idx: ID of the memory-type cortical area 
            n_steps: Number of timesteps to look back
            
        Returns:
            BitMap of neuron IDs that fired in the specified timespan
            
        Raises:
            ValueError: If n_steps is negative or cortical_idx is not a memory area
        """
        if not self.is_memory_area(cortical_idx):
            raise ValueError(f"Area {cortical_idx} is not registered as a memory area")
            
        window_size, history_array, start_timestep = self.custom_area_history[cortical_idx]
        
        if n_steps <= 0:
            raise ValueError(f"n_steps must be positive, got {n_steps}")
            
        if n_steps > window_size:
            n_steps = window_size
            self.logger.warning(f"n_steps {n_steps} exceeds window size {window_size}, limiting to window size")
        
        result = BitMap()
        for i in range(n_steps):
            if self.current_timestep - i < start_timestep:
                break  # Don't go before the start timestep
                
            timestep = self.current_timestep - i
            custom_index = self._get_custom_area_index(cortical_idx, timestep)
            result = result | history_array[custom_index]
            
        return result
    
    def get_memory_area_consistency(self, cortical_idx: CorticalIdx, pattern_duration: int, window_duration: int) -> float:
        """
        Calculate how consistently a pattern has been maintained in a memory area.
        
        Args:
            cortical_idx: ID of the memory-type cortical area
            pattern_duration: Duration of the pattern to analyze (in timesteps)
            window_duration: Total window to evaluate (must be >= pattern_duration)
            
        Returns:
            Consistency score between 0.0 (no consistency) and 1.0 (perfect consistency)
            
        Raises:
            ValueError: If cortical_idx is not a memory area or if window parameters are invalid
        """
        if not self.is_memory_area(cortical_idx):
            raise ValueError(f"Area {cortical_idx} is not registered as a memory area")
            
        if window_duration < pattern_duration:
            raise ValueError(f"Window duration ({window_duration}) must be >= pattern duration ({pattern_duration})")
            
        window_size, history_array, start_timestep = self.custom_area_history[cortical_idx]
        
        if pattern_duration > window_size:
            pattern_duration = window_size
            self.logger.warning(f"Pattern duration {pattern_duration} exceeds window size {window_size}, limiting to window size")
            
        if window_duration > window_size:
            window_duration = window_size
            self.logger.warning(f"Window duration {window_duration} exceeds window size {window_size}, limiting to window size")
        
        # Get the pattern to check for consistency
        pattern = self.get_area_temporal_pattern(cortical_idx, pattern_duration)
        
        if len(pattern) == 0:
            return 0.0  # No pattern to check
        
        # Check how consistently this pattern appears in the window
        match_count = 0
        
        for offset in range(window_duration - pattern_duration + 1):
            # For each possible pattern position in the window
            match = True
            
            for i in range(pattern_duration):
                timestep = self.current_timestep - offset - i
                if timestep < start_timestep:
                    match = False
                    break
                    
                idx = self._get_custom_area_index(cortical_idx, timestep)
                step_neurons = history_array[idx]
                
                # Check if this timestep's neurons match the pattern
                if step_neurons != pattern:
                    match = False
                    break
            
            if match:
                match_count += 1
        
        # Return proportion of positions where pattern was found
        possible_positions = window_duration - pattern_duration + 1
        return match_count / possible_positions if possible_positions > 0 else 0.0
    
    def get_consistent_neurons_in_memory_area(self, cortical_idx: CorticalIdx, n_steps: int) -> BitMap:
        """
        Get neurons in a memory area that fired consistently across all specified timesteps.
        
        Args:
            cortical_idx: ID of the memory-type cortical area
            n_steps: Number of timesteps to look back
            
        Returns:
            BitMap of neurons that fired in ALL of the last n_steps
            
        Raises:
            ValueError: If n_steps is negative or cortical_idx is not a memory area
        """
        if not self.is_memory_area(cortical_idx):
            raise ValueError(f"Area {cortical_idx} is not registered as a memory area")
        
        if n_steps <= 0:
            raise ValueError(f"n_steps must be positive, got {n_steps}")
            
        window_size, history_array, start_timestep = self.custom_area_history[cortical_idx]
        
        if n_steps > window_size:
            n_steps = window_size
            self.logger.warning(f"n_steps {n_steps} exceeds window size {window_size}, limiting to window size")
        
        # Initialize result with neurons from the first relevant timestep
        timestep = self.current_timestep
        custom_index = self._get_custom_area_index(cortical_idx, timestep)
        result = history_array[custom_index].copy()
        
        # Intersect with each subsequent timestep
        for i in range(1, n_steps):
            if self.current_timestep - i < start_timestep:
                # If we can't go back far enough, return empty set
                return BitMap()
                
            timestep = self.current_timestep - i
            custom_index = self._get_custom_area_index(cortical_idx, timestep)
            result = result & history_array[custom_index]
            
            if result.is_empty():
                # Short-circuit if intersection becomes empty
                break
                
        return result
    
    def get_neurons_fired_in_last_n_steps(self, n_steps: int, area_ids: Optional[List[CorticalIdx]] = None) -> BitMap:
        """
        Get neurons that fired in any of the last n timesteps.
        Optionally filter by specific cortical areas.
        
        Args:
            n_steps: Number of timesteps to look back
            area_ids: Optional list of area IDs to filter by
            
        Returns:
            BitMap containing neuron IDs that fired in the specified timespan
            
        Raises:
            ValueError: If n_steps is negative
        """
        if n_steps <= 0:
            raise ValueError(f"n_steps must be positive, got {n_steps}")
        
        # For memory areas, we can use the specialized function if requested
        if area_ids is not None and len(area_ids) == 1 and self.is_memory_area(area_ids[0]):
            return self.get_area_temporal_pattern(area_ids[0], n_steps)
        
        # Standard implementation for multiple areas or non-memory areas
        if n_steps > self.default_window_size:
            n_steps = self.default_window_size
            self.logger.warning(f"For standard areas, n_steps {n_steps} exceeds default window size {self.default_window_size}, limiting to default window size")
        
        result = BitMap()
        
        # No specific areas requested, use global FCL
        if area_ids is None:
            for i in range(n_steps):
                step_index = (self.current_window_index - i) % self.default_window_size
                result = result | self.global_fcl_history[step_index]
        else:
            # Process both standard and memory areas
            for area_id in area_ids:
                if self.is_memory_area(area_id):
                    # For memory areas, use their specialized temporal pattern function
                    area_result = self.get_area_temporal_pattern(area_id, n_steps)
                    result = result | area_result
                else:
                    # For standard areas, use the default processing
                    if area_id in self.area_fcl_history:
                        for i in range(min(n_steps, self.default_window_size)):
                            step_index = (self.current_window_index - i) % self.default_window_size
                            result = result | self.area_fcl_history[area_id][step_index]
        
        return result
    
    def count_firing_neurons(self, timestep: Optional[int] = None, area_id: Optional[CorticalIdx] = None) -> int:
        """
        Efficiently count the number of firing neurons.
        
        Args:
            timestep: Optional timestep (defaults to current)
            area_id: Optional area ID to count neurons for a specific area
            
        Returns:
            Count of firing neurons
            
        Raises:
            TimestepOutOfRangeError: If timestep is outside valid range
        """
        if timestep is None and area_id is None:
            # Use cached total for current timestep
            return self.total_neurons_fired
        
        if timestep is None:
            timestep = self.current_timestep
        
        if area_id is None:
            # Count all neurons at the specified timestep
            index = self._get_index_for_timestep(timestep)
            return len(self.global_fcl_history[index])
        else:
            # Count neurons in the specified area
            try:
                area_fcl = self.get_area_fcl(area_id, timestep)
                return len(area_fcl)
            except (ValueError, TimestepOutOfRangeError):
                return 0
    
    def get_firing_statistics(self) -> Dict[str, Any]:
        """
        Get statistics about firing patterns across the FCL history.
        
        Returns:
            Dictionary with firing statistics
        """
        stats = {
            "total_neurons_fired": self.total_neurons_fired,
            "neurons_per_area": self.neurons_per_area,
            "active_areas": len(self.area_fcl_history),
            "memory_areas": len(self.memory_area_ids),
            "current_window_index": self.current_window_index,
            "current_timestep": self.current_timestep
        }
        return stats
    
    # Methods for two-phase membrane potential update process
    
    def advance_timestep(self) -> None:
        """
        Advance to the next timestep, shifting FCL history.
        """
        self.current_timestep += 1
        self.current_window_index = (self.current_window_index + 1) % self.default_window_size
        
        # Clear the current FCL slot (both global and per-area)
        self.global_fcl_history[self.current_window_index].clear()
        
        for area_id in self.area_fcl_history:
            window_size = self.get_area_window_size(area_id)
            idx = self._get_custom_area_index(area_id, self.current_timestep)
            self.area_fcl_history[area_id][idx].clear()
            
        # Reset update queue for new timestep
        self._reset_update_queue()
    
    def _reset_update_queue(self) -> None:
        """Reset the membrane potential update queue for a new timestep."""
        if not hasattr(self, 'mp_update_queue'):
            self.mp_update_queue: List[MembraneUpdate] = []
            self.updates_processed_count = 0
        else:
            self.mp_update_queue = []
    
    def queue_membrane_update(self, 
                             neuron_idx: int, 
                             delta_potential: float,
                             source_neuron_idx: Optional[int] = None) -> None:
        """
        Queue an update to a neuron's membrane potential.
        
        Args:
            neuron_idx: Index of the neuron to update
            delta_potential: Change in membrane potential
            source_neuron_idx: Source of the update (if from a specific neuron)
        """
        if not hasattr(self, 'mp_update_queue'):
            self._reset_update_queue()
            
        self.mp_update_queue.append(
            MembraneUpdate(
                neuron_idx=neuron_idx,
                delta_potential=delta_potential,
                source_neuron_idx=source_neuron_idx
            )
        )
        self.updates_processed_count += 1
    
    def process_update_queue(self) -> List[Tuple[int, float]]:
        """
        Process the membrane potential update queue, aggregating updates per neuron.
        
        Returns:
            List of (neuron_idx, total_delta) tuples with the final updates
        """
        if not hasattr(self, 'mp_update_queue') or not self.mp_update_queue:
            return []
        
        # Aggregate updates by neuron
        neuron_updates: Dict[int, float] = {}
        
        for update in self.mp_update_queue:
            neuron_updates[update.neuron_idx] = (
                neuron_updates.get(update.neuron_idx, 0.0) + update.delta_potential
            )
        
        # Convert to list of tuples
        aggregated_updates = [(idx, delta) for idx, delta in neuron_updates.items()]
        
        # Clear the queue after processing
        self.mp_update_queue = []
        
        return aggregated_updates
    
    def get_firing_neurons(self, offset: int = -1) -> List[int]:
        """
        Get list of neuron indices that are in the FCL at the specified offset.
        
        Args:
            offset: Timestep offset (-1 for previous timestep, which is the firing phase)
            
        Returns:
            List of neuron indices
        """
        fcl = self.get_global_fcl(self.current_timestep + offset if offset else None)
        return list(fcl)
    
    def add_to_current_fcl(self, neuron_indices: Union[List[int], Set[int], BitMap]) -> None:
        """
        Add neurons to the current timestep's FCL.
        
        Args:
            neuron_indices: Indices of neurons to add to current FCL
        """
        # Convert to BitMap if needed
        if not isinstance(neuron_indices, BitMap):
            neuron_bitmap = BitMap(neuron_indices)
        else:
            neuron_bitmap = neuron_indices
            
        # Add to global FCL
        current_fcl = self.global_fcl_history[self.current_window_index]
        # Use BitMap.__or__ to update in place
        updated_fcl = current_fcl | neuron_bitmap
        self.global_fcl_history[self.current_window_index] = updated_fcl
        
        # Update statistics
        self.total_neurons_fired += len(neuron_bitmap)
    
    def get_fcl(self, offset: int = 0) -> BitMap:
        """
        Get the FCL for a specific timestep relative to current.
        
        Args:
            offset: Timestep offset (0 for current, -1 for previous, etc.)
            
        Returns:
            BitMap of neuron indices in the FCL
        """
        return self.get_global_fcl(self.current_timestep + offset if offset else None)


# Example usage
def example_fcl_usage() -> None:
    """Example usage of the FCL manager."""
    
    # Create FCL manager with window size of 3
    fcl_manager = HierarchicalFCL(window_size=3)
    
    # Example data for first timestep: cortical_idx -> neurons firing
    firing_neurons_t1: Dict[CorticalIdx, BitMap] = {
        100: BitMap([1001, 1002, 1005, 1008]),
        200: BitMap([2001, 2010, 2015]),
        300: BitMap([3001, 3002, 3003, 3004, 3005])
    }
    
    # Update FCL with timestep 1 data
    fcl_manager.update_fcl(current_timestep=1, neurons_by_area=firing_neurons_t1)
    
    # Example data for second timestep
    firing_neurons_t2: Dict[CorticalIdx, Union[BitMap, List[int]]] = {
        100: BitMap([1002, 1003, 1009]),
        200: BitMap([2001, 2005]),
        300: [3002, 3005, 3010]  # Mix of collection types to show conversion
    }
    
    # Update FCL with timestep 2 data
    fcl_manager.update_fcl(current_timestep=2, neurons_by_area=firing_neurons_t2)
    
    # Example data for third timestep
    firing_neurons_t3: Dict[CorticalIdx, Union[BitMap, Set[int]]] = {
        100: BitMap([1003, 1004, 1005]),
        200: set([2001, 2002, 2003]),  # Set instead of bitmap
        300: BitMap([3001, 3005, 3008])
    }
    
    # Update FCL with timestep 3 data
    fcl_manager.update_fcl(current_timestep=3, neurons_by_area=firing_neurons_t3)
    
    # Get global FCL for current timestep
    global_fcl = fcl_manager.get_global_fcl()
    print(f"All firing neurons at timestep 3: {global_fcl}")
    
    # Get FCL for a specific cortical area
    area_fcl = fcl_manager.get_area_fcl(cortical_idx=100)
    print(f"Firing neurons in cortical area 100: {area_fcl}")
    
    # Get FCL for timestep 2
    prev_fcl = fcl_manager.get_global_fcl(timestep=2)
    print(f"All firing neurons at timestep 2: {prev_fcl}")
    
    # Get all firing neurons across areas 100 and 300
    selected_areas_fcl = fcl_manager.get_neurons_by_areas([100, 300])
    print(f"Firing neurons in areas 100 and 300: {selected_areas_fcl}")
    
    # Get neurons that fired in any of the last 2 timesteps (3 and 2)
    recent_fcl = fcl_manager.get_neurons_fired_in_last_n_steps(2)
    print(f"Neurons firing in recent 2 timesteps: {recent_fcl}")
    
    # Get active cortical areas
    active_areas = fcl_manager.get_active_areas()
    print(f"Active cortical areas: {active_areas}")


def example_enhanced_fcl_usage() -> None:
    """Example usage of the enhanced FCL manager with memory areas."""
    
    # Create FCL manager with default window size of 5
    fcl_manager = EnhancedHierarchicalFCL(default_window_size=5)
    
    # Register some memory areas with custom window sizes
    fcl_manager.register_memory_area(cortical_idx=400, window_size=20)  # Memory area 1 with 20-step window
    fcl_manager.register_memory_area(cortical_idx=500, window_size=10)  # Memory area 2 with 10-step window
    
    # Create some example neuron firing data
    # Regular areas and memory areas can be mixed freely
    for timestep in range(1, 30):
        firing_neurons = {}
        
        # Regular areas with default window
        firing_neurons[100] = BitMap([1001, 1002, 1005])  # Visual cortex always has same neurons
        
        # Variable neurons in motor cortex
        if timestep % 2 == 0:
            firing_neurons[200] = BitMap([2001, 2003, 2005])
        else:
            firing_neurons[200] = BitMap([2002, 2004, 2006])
            
        # First memory area (400) with stable pattern that changes every 5 steps
        pattern_set = (timestep // 5) % 4
        if pattern_set == 0:
            firing_neurons[400] = BitMap([4001, 4002, 4003])
        elif pattern_set == 1:
            firing_neurons[400] = BitMap([4010, 4011, 4012])
        elif pattern_set == 2:
            firing_neurons[400] = BitMap([4020, 4021, 4022])
        else:
            firing_neurons[400] = BitMap([4030, 4031, 4032])
            
        # Second memory area (500) with accumulating neuron activations
        memory_neurons = list(range(5001, 5001 + timestep % 10))
        firing_neurons[500] = BitMap(memory_neurons)
        
        # Update the FCL with this timestep's data
        fcl_manager.update_fcl(timestep, firing_neurons)
        
        print(f"\nTimestep {timestep}:")
        print(f"Active areas: {fcl_manager.get_active_areas()}")
        
        # Perform some analysis at specific timesteps
        if timestep == 15:
            # At timestep 15, examine memory area patterns
            memory_fcl = fcl_manager.get_area_fcl(cortical_idx=400)
            print(f"Memory area 400 current neurons: {memory_fcl}")
            
            # Look back to a much earlier timestep for memory area (only possible in memory areas)
            old_memory_fcl = fcl_manager.get_area_fcl(cortical_idx=400, timestep=5)
            print(f"Memory area 400 at timestep 5: {old_memory_fcl}")
            
            # Looking back at regular areas is limited to the default window size
            try:
                old_regular_fcl = fcl_manager.get_area_fcl(cortical_idx=100, timestep=5)
                print(f"Regular area 100 at timestep 5: {old_regular_fcl}")
            except TimestepOutOfRangeError:
                print("Cannot look back that far for regular areas")
    
    # Check memory consistency for the first memory area
    pattern_consistency = fcl_manager.get_memory_area_consistency(
        cortical_idx=400,  # The area to check
        pattern_duration=3,  # How long a pattern typically lasts
        window_duration=15  # How far back to look
    )
    print(f"\nPattern consistency in memory area 400: {pattern_consistency:.2f}")
    
    # Look for consistently active neurons in the past 6 timesteps
    consistent_neurons = fcl_manager.get_consistent_neurons_in_memory_area(
        cortical_idx=500,  # The area to check
        n_steps=5  # Number of consecutive steps neurons must be active
    )
    print(f"\nConsistently active neurons in memory area 500: {consistent_neurons}")
    
    # Get neurons that fired in any memory area in the past 10 timesteps
    memory_areas = [400, 500]
    recent_memory_fcl = fcl_manager.get_neurons_fired_in_last_n_steps(10, memory_areas)
    print(f"\nRecent memory neurons: {recent_memory_fcl}")

def inject_neurons_into_fcl(fcl_manager, cortical_idx, neuron_ids, timestep=None):
    """
    Helper function to inject neurons into an FCL for a specific cortical area.
    
    Args:
        fcl_manager: FCL manager instance
        cortical_idx: Cortical area ID
        neuron_ids: List of neuron IDs to inject
        timestep: Optional timestep, defaults to current
    """
    neurons_by_area = {cortical_idx: neuron_ids}
    if timestep is None:
        timestep = fcl_manager.current_timestep
    fcl_manager.update_fcl(timestep, neurons_by_area)


if __name__ == "__main__":
    # Set up logging
    logging.basicConfig(level=logging.DEBUG)
    
    # Run examples
    example_fcl_usage()
    example_enhanced_fcl_usage()

    # Initialize the enhanced FCL
    fcl = EnhancedHierarchicalFCL(default_window_size=5)

    # Register a memory area
    fcl.register_memory_area(cortical_idx=500, window_size=100)

    # Inject specific neurons into the hippocampus memory area
    hippocampus_neurons = [501, 502, 503, 504]
    inject_neurons_into_fcl(fcl, cortical_idx=500, neuron_ids=hippocampus_neurons)

    # Check the injected neurons
    result = fcl.get_area_fcl(cortical_idx=500)
    print(f"Neurons in hippocampus: {result}") 