"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
Fire Candidate List (FCL) Manager for FEAGI.

The FCL Manager maintains the temporal queue of neurons that are scheduled to fire.
It provides efficient access to current and historical firing patterns.

Key features:
- Maintains multi-timestep FCL history
- Thread-safe operations
- Optimized for fast lookups and modifications
- Cortical-specific sampling capabilities
- Dependency-injected design
"""

from enum import Enum
from typing import (
    Any,
    Dict,
    Iterator,
    List,
    Optional,
    Protocol,
    Set,
    Tuple,
    Union,
    cast,
)

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)
import logging
from dataclasses import dataclass

import numpy as np

import pyroaring

# Type definitions to make migration to Rust more straightforward
NeuronId = int
CorticalIdx = int  # Integer identifier for cortical areas internally
Timestep = int


class FCLError(Exception):
    """Base error class for FCL operations."""

    pass


class TimestepOutOfRangeError(FCLError):
    """Error raised when a requested timestep is outside the available
    window."""

    pass


@dataclass
class MembraneUpdate:
    """Represents a pending update to a neuron's membrane potential."""

    neuron_idx: int
    delta_potential: float
    source_neuron_idx: Optional[int] = (
        None  # Source of the update (for tracing)
    )


# Define a Protocol for bitmap-like objects (similar to Rust traits)
class BitMapProtocol(Protocol):
    """Protocol defining the interface for bitmap-like objects."""

    def __init__(self, elements=None): ...

    def add(self, element: int) -> None: ...

    def clear(self) -> None: ...

    def copy(self) -> "BitMapProtocol": ...

    def __or__(self, other: "BitMapProtocol") -> "BitMapProtocol": ...

    def __and__(self, other: "BitMapProtocol") -> "BitMapProtocol": ...

    def __sub__(self, other: "BitMapProtocol") -> "BitMapProtocol": ...

    def __xor__(self, other: "BitMapProtocol") -> "BitMapProtocol": ...

    def __len__(self) -> int: ...

    def __iter__(self) -> Iterator[int]: ...

    def __contains__(self, item: int) -> bool: ...

    def is_empty(self) -> bool: ...


# PyRoaring bitmap wrapper with consistent interface
class RoaringBitmap:
    """Wrapper for PyRoaring bitmap with consistent interface."""

    def __init__(self, elements=None):
        # WGPU-COMPATIBLE: Optimize initialization
        if elements is not None:
            self._bitmap = pyroaring.BitMap(elements)
        else:
            self._bitmap = pyroaring.BitMap()

    def add(self, element: int) -> None:
        self._bitmap.add(element)

    def clear(self) -> None:
        self._bitmap.clear()

    def copy(self) -> "RoaringBitmap":
        # WGPU-COMPATIBLE: Optimize copy operation
        result = RoaringBitmap.__new__(RoaringBitmap)
        result._bitmap = self._bitmap.copy()
        return result

    def __or__(self, other: "RoaringBitmap") -> "RoaringBitmap":
        # WGPU-COMPATIBLE: Optimize union operation
        result = RoaringBitmap.__new__(RoaringBitmap)
        result._bitmap = self._bitmap | other._bitmap
        return result

    def __and__(self, other: "RoaringBitmap") -> "RoaringBitmap":
        # WGPU-COMPATIBLE: Optimize intersection operation
        result = RoaringBitmap.__new__(RoaringBitmap)
        result._bitmap = self._bitmap & other._bitmap
        return result

    def __sub__(self, other: "RoaringBitmap") -> "RoaringBitmap":
        # WGPU-COMPATIBLE: Optimize difference operation
        result = RoaringBitmap.__new__(RoaringBitmap)
        result._bitmap = self._bitmap - other._bitmap
        return result

    def __xor__(self, other: "RoaringBitmap") -> "RoaringBitmap":
        # WGPU-COMPATIBLE: Optimize symmetric difference operation
        result = RoaringBitmap.__new__(RoaringBitmap)
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

    def serialize(self) -> bytes:
        """Serialize the bitmap to bytes for pattern storage."""
        return self._bitmap.serialize()

    def __repr__(self) -> str:
        return repr(self._bitmap)

BitMap = RoaringBitmap


# Enum for neuron collections to enable static type checking
class NeuronCollectionType(Enum):
    """Types of neuron collection inputs that can be converted to bitmaps."""

    BITMAP = "bitmap"
    LIST = "list"
    SET = "set"


# Wrapper for neuron collections to make type handling explicit
@dataclass
class NeuronCollection:
    """Container for different types of neuron collections with explicit
    type."""

    collection_type: NeuronCollectionType
    data: Union[BitMap, List[int], Set[int]]

    @classmethod
    def from_any(
        cls, data: Union[BitMap, List[int], Set[int]]
    ) -> "NeuronCollection":
        """Create a NeuronCollection from any supported data type."""
        if isinstance(data, BitMap):
            return cls(NeuronCollectionType.BITMAP, data)
        elif isinstance(data, list):
            return cls(NeuronCollectionType.LIST, data)
        elif isinstance(data, set):
            return cls(NeuronCollectionType.SET, data)
        else:
            raise TypeError(
                f"Unsupported neuron collection type: {type(data)}"
            )

    def to_bitmap(self) -> BitMap:
        """Convert the collection to a bitmap."""
        if self.collection_type == NeuronCollectionType.BITMAP:
            return cast(BitMap, self.data)
        else:
            return BitMap(self.data)


class FCLManager:
    """Manager for the Fire Candidate List (FCL) queue with support for memory
    corticals.

    The FCLManager maintains data structures for tracking which neurons are
    firing at the current timestep and which have fired in previous timesteps.
    It provides methods for adding neurons to the FCL, advancing the temporal
    window, and querying firing history.

    This implementation supports both standard cortical areas and memory
    cortical areas with custom window sizes for enhanced temporal pattern
    recognition.

    This implementation is designed to be thread-safe and efficient for large-
    scale neural simulations.
    """

    def __init__(self, window_size: int = 20):
        """Initialize a hierarchical FCL that tracks both neurons and their
        cortical areas.

        Args:
            window_size: Default number of timesteps to maintain in history (also called default_window_size)
        """
        self.window_size: int = window_size
        self.default_window_size: int = (
            window_size  # For backward compatibility
        )

        # Dynamic window sizing integration with StateManager
        self._state_manager = None
        self._dynamic_sizing_enabled = False
        self._cortical_window_sizes: Dict[CorticalIdx, int] = (
            {}
        )  # Cache for dynamic window sizes

        # WGPU-COMPATIBLE: Pre-allocate BitMaps more efficiently
        # Main FCL history - stores all neurons regardless of cortical
        self.global_fcl_history: List[BitMap] = []
        for _ in range(window_size):
            self.global_fcl_history.append(BitMap())

        #  Cortical-specific FCL history - mapping from cortical_idx to list of
        #  bitmaps
        self.cortical_fcl_history: Dict[CorticalIdx, List[BitMap]] = {}

        # Memory cortical areas with custom window sizes
        # Format: cortical_idx -> (window_size, history_array, start_timestep)
        self.custom_cortical_history: Dict[
            CorticalIdx, Tuple[int, List[BitMap], int]
        ] = {}
        self.memory_cortical_indices: Set[CorticalIdx] = set()

        self.current_window_index: int = 0
        self.current_timestep: int = 0

        # Membrane potential update queue
        self.membrane_update_queue: List[MembraneUpdate] = []

        # Stats
        self.total_neurons_fired: int = 0
        self.neurons_per_cortical: Dict[CorticalIdx, int] = {}

        # Logger
        self.logger = setup_logger("feagi.npu.fcl_manager")

        # Initialize StateManager integration
        self._initialize_state_manager_integration()

    def _initialize_state_manager_integration(self) -> None:
        """Initialize integration with StateManager for dynamic window sizing."""
        from feagi.core.state_manager import get_state_manager

        self._state_manager = get_state_manager()
        self._dynamic_sizing_enabled = True
        self.logger.info(
            "FCL Manager: Dynamic window sizing enabled via StateManager"
        )

    def enable_dynamic_window_sizing(self, enabled: bool = True) -> None:
        """Enable or disable dynamic window sizing based on memory areas."""
        self._dynamic_sizing_enabled = enabled
        self.logger.info(
            f"FCL Manager: Dynamic window sizing {'enabled' if self._dynamic_sizing_enabled else 'disabled'}"
        )

    def update_cortical_window_size(
        self, cortical_idx: CorticalIdx, new_window_size: int
    ) -> bool:
        """Update window size for a cortical area and resize its history if
        needed.

        Args:
            cortical_idx: Cortical area index
            new_window_size: New window size

        Returns:
            True if successful
        """
        if cortical_idx not in self.cortical_fcl_history:
            # Initialize new cortical history with new window size
            self.cortical_fcl_history[cortical_idx] = []
            for _ in range(new_window_size):
                self.cortical_fcl_history[cortical_idx].append(BitMap())
        else:
            # Resize existing history
            current_history = self.cortical_fcl_history[cortical_idx]
            current_size = len(current_history)

            if new_window_size > current_size:
                # Expand history
                for _ in range(new_window_size - current_size):
                    current_history.append(BitMap())
            elif new_window_size < current_size:
                # Shrink history (keep most recent data)
                self.cortical_fcl_history[cortical_idx] = current_history[
                    -new_window_size:
                ]

        # Update cached window size
        self._cortical_window_sizes[cortical_idx] = new_window_size

        self.logger.debug(
            f"Updated window size for cortical_idx {cortical_idx} to {new_window_size}"
        )
        return True

    def get_cortical_window_size(self, cortical_idx: CorticalIdx) -> int:
        """Get the window size for a specific cortical area with dynamic sizing
        support.

        Args:
            cortical_idx: ID of the cortical area

        Returns:
            Window size for the specified cortical area
        """
        # Check if dynamic sizing is enabled
        if self._dynamic_sizing_enabled:
            # First check cached value
            if cortical_idx in self._cortical_window_sizes:
                return self._cortical_window_sizes[cortical_idx]

            # Get cortical ID from index mapping
            cortical_id = self._get_cortical_id_from_index(cortical_idx)
            if cortical_id:
                # Query StateManager for dynamic window size
                dynamic_size = self._state_manager.get_fcl_window_size(
                    cortical_id
                )
                # Cache the result
                self._cortical_window_sizes[cortical_idx] = dynamic_size
                return dynamic_size

        # Use custom cortical history or default window size
        if cortical_idx in self.custom_cortical_history:
            return self.custom_cortical_history[cortical_idx][0]
        return self.default_window_size

    def _get_cortical_id_from_index(
        self, cortical_idx: CorticalIdx
    ) -> Optional[str]:
        """Convert cortical index to cortical ID using ConnectomeManager mapping."""
        from feagi.bdu.connectome_manager import ConnectomeManager

        connectome_manager = ConnectomeManager.instance()
        return connectome_manager.get_cortical_id_for_idx(cortical_idx)

    def invalidate_cortical_window_cache(
        self, cortical_idx: CorticalIdx
    ) -> None:
        """Invalidate cached window size for a cortical area."""
        self._cortical_window_sizes.pop(cortical_idx, None)
        self.logger.debug(
            f"Invalidated window size cache for cortical_idx {cortical_idx}"
        )

    def clear_all_window_caches(self) -> None:
        """Clear all cached window sizes to force recomputation."""
        self._cortical_window_sizes.clear()
        self.logger.info("Cleared all FCL window size caches")

    def clear_all_fcl_history(self) -> None:
        """Clear all FCL history data to prevent stale cortical indices.
        
        This method clears:
        - Global FCL history
        - Cortical-specific FCL history  
        - Custom cortical history (memory areas)
        - Window size caches
        
        Used during genome reload to prevent stale cortical index references.
        """
        try:
            # Clear global FCL history
            for bitmap in self.global_fcl_history:
                bitmap.clear()
            
            # Clear cortical-specific FCL history
            self.cortical_fcl_history.clear()
            
            # Clear custom cortical history (memory areas)
            self.custom_cortical_history.clear()
            self.memory_cortical_indices.clear()
            
            # Clear window size caches
            self._cortical_window_sizes.clear()
            
            # Reset counters
            self.total_neurons_fired = 0
            self.current_window_index = 0
            
            self.logger.info("Cleared all FCL history data and caches")
            
        except Exception as e:
            self.logger.error(f"Error clearing FCL history: {e}")
            raise

    def register_memory_cortical(
        self, cortical_idx: CorticalIdx, window_size: int
    ) -> None:
        """Register a memory-type cortical area with a custom window size.

        Args:
            cortical_idx: ID of the memory-type cortical area
            window_size: Custom history window size for this area (must be >= default_window_size)

        Raises:
            ValueError: If window_size is smaller than default_window_size
        """
        if window_size < self.default_window_size:
            raise ValueError(
                f"Memory area window size ({window_size}) must be >= default window size ({self.default_window_size})"
            )

        # Create array of empty bitmaps for this area's history
        history_array = [BitMap() for _ in range(window_size)]

        # Store area with custom settings
        #  The third element is the start timestep, initialized to current
        #  timestep
        self.custom_cortical_history[cortical_idx] = (
            window_size,
            history_array,
            self.current_timestep,
        )
        self.memory_cortical_indices.add(cortical_idx)

        self.logger.info(
            f"Registered memory cortical area {cortical_idx} with window size {window_size}"
        )

    def is_memory_cortical(self, cortical_idx: CorticalIdx) -> bool:
        """Check if a cortical area is registered as a memory-type area with
        custom window size.

        Args:
            cortical_idx: Cortical ID to check

        Returns:
            True if this is a memory cortical area with custom window size, False otherwise
        """
        return cortical_idx in self.memory_cortical_indices

    def _get_custom_cortical_index(
        self, cortical_idx: CorticalIdx, timestep: int
    ) -> int:
        """Calculate the correct index in the custom window size history for a
        given cortical area and timestep.

        Args:
            cortical_idx: ID of the memory-type cortical area
            timestep: Timestep to calculate index for

        Returns:
            Index in the cortical area's custom-sized history array

        Raises:
            ValueError: If cortical_idx is not a registered memory cortical area
            TimestepOutOfRangeError: If timestep is outside valid range for this cortical area
        """
        if cortical_idx not in self.custom_cortical_history:
            raise ValueError(
                f"Cortical area {cortical_idx} is not registered as a memory area"
            )

        window_size, history_array, start_timestep = (
            self.custom_cortical_history[cortical_idx]
        )

        # Check if timestep is too old for this area
        if timestep < start_timestep:
            raise TimestepOutOfRangeError(
                f"Timestep {timestep} is before the start timestep {start_timestep} for cortical area {cortical_idx}"
            )

        # Calculate the index in the circular buffer
        return (timestep - start_timestep) % window_size

    def _ensure_cortical_initialized(self, cortical_idx: CorticalIdx) -> None:
        """Ensure a cortical area is initialized in the FCL history.

        Args:
            cortical_idx: ID of the cortical area to initialize
        """
        if cortical_idx not in self.cortical_fcl_history:
            # WGPU-COMPATIBLE: Pre-allocate BitMaps more efficiently
            cortical_history: List[BitMap] = []
            for _ in range(self.window_size):
                cortical_history.append(BitMap())
            self.cortical_fcl_history[cortical_idx] = cortical_history

    def _get_index_for_timestep(self, timestep: Optional[int] = None) -> int:
        """Get the correct index in the standard FCL history for a given
        timestep.

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

    def update_fcl(
        self,
        current_timestep: int,
        neurons_by_cortical: Dict[
            CorticalIdx, Union[BitMap, List[int], Set[int]]
        ],
    ) -> None:
        """Update the FCL with neurons firing in the current timestep, with
        support for memory corticals.

        Args:
            current_timestep: Current simulation timestep
            neurons_by_cortical: Dictionary mapping cortical_idx -> list/set/bitmap of neuron_ids
        """
        # Optional debug tracing
        try:
            from feagi.core.state_manager import FeagiStateManager
            if FeagiStateManager.instance().is_debug_npu_enabled():
                logger.info(
                    f"[FCL-DEBUG] update_fcl called at t={current_timestep} with corticals={list(neurons_by_cortical.keys())}"
                )
        except Exception:
            pass

        # Track per-timestep clearing to avoid wiping earlier injection updates
        if not hasattr(self, "_last_update_timestep"):
            self._last_update_timestep = None
            self._cleared_corticals_for_timestep = set()

        if self._last_update_timestep != current_timestep:
            self._last_update_timestep = current_timestep
            just_cleared_this_timestep = True
            # Reset cleared set for new timestep
            self._cleared_corticals_for_timestep.clear()
        else:
            just_cleared_this_timestep = False

        self.current_timestep = current_timestep
        standard_index = current_timestep % self.default_window_size

        # Clear the oldest global bitmap for reuse
        if just_cleared_this_timestep:
            self.global_fcl_history[standard_index].clear()

        # Track firing statistics
        burst_total = 0

        # Process cortical areas using vectorized operations
        cortical_indices = list(neurons_by_cortical.keys())
        neuron_collections = [
            neurons_by_cortical[idx] for idx in cortical_indices
        ]

        # Vectorized bitmap conversion
        cortical_bitmaps = [
            NeuronCollection.from_any(neurons).to_bitmap()
            for neurons in neuron_collections
        ]
        cortical_neuron_counts = np.array(
            [len(bitmap) for bitmap in cortical_bitmaps]
        )

        # Bulk update neuron counts
        for idx, cortical_idx in enumerate(cortical_indices):
            self.neurons_per_cortical[cortical_idx] = cortical_neuron_counts[
                idx
            ]

        burst_total = int(np.sum(cortical_neuron_counts))

        # Optional debug tracing
        try:
            from feagi.core.state_manager import FeagiStateManager
            if FeagiStateManager.instance().is_debug_npu_enabled():
                logger.info(
                    f"[FCL-DEBUG] Pre-process: total candidates={burst_total}, per-cortical={self.neurons_per_cortical}"
                )
        except Exception:
            pass

        # Process each cortical area efficiently
        for idx, cortical_idx in enumerate(cortical_indices):
            cortical_bitmap = cortical_bitmaps[idx]

            # Check if this is a memory cortical with custom window size
            if self.is_memory_cortical(cortical_idx):
                window_size, history_array, _ = self.custom_cortical_history[
                    cortical_idx
                ]
                custom_index = self._get_custom_cortical_index(
                    cortical_idx, current_timestep
                )

                # Clear once per timestep for this cortical, then union
                if cortical_idx not in self._cleared_corticals_for_timestep:
                    history_array[custom_index].clear()
                    self._cleared_corticals_for_timestep.add(cortical_idx)
                # Union to preserve earlier injections in the same timestep
                history_array[custom_index] = (
                    history_array[custom_index] | cortical_bitmap
                )
            else:
                # Standard cortical processing
                self._ensure_cortical_initialized(cortical_idx)

                # Clear once per timestep for this cortical, then union
                if cortical_idx not in self._cleared_corticals_for_timestep:
                    self.cortical_fcl_history[cortical_idx][standard_index].clear()
                    self._cleared_corticals_for_timestep.add(cortical_idx)
                self.cortical_fcl_history[cortical_idx][standard_index] = (
                    self.cortical_fcl_history[cortical_idx][standard_index] | cortical_bitmap
                )

            # Always update the global FCL (for all corticals)
            self.global_fcl_history[standard_index] = (
                self.global_fcl_history[standard_index] | cortical_bitmap
            )

        # Update total count and window index
        self.total_neurons_fired = burst_total
        self.current_window_index = standard_index

        # Update cumulative activity counters in StateManager for sleep trigger
        from feagi.core.state_manager import FeagiStateManager

        FeagiStateManager.instance().increment_cumulative_activity(
            burst_total
        )


    def get_global_fcl(self, timestep: Optional[int] = None) -> BitMap:
        """Get the complete FCL for a specific timestep.

        Args:
            timestep: Specific timestep to query (defaults to current)

        Returns:
            BitMap containing all firing neuron IDs

        Raises:
            TimestepOutOfRangeError: If timestep is outside valid range
        """
        index = self._get_index_for_timestep(timestep)
        return self.global_fcl_history[index].copy()

    def get_cortical_fcl(
        self, cortical_idx: CorticalIdx, timestep: Optional[int] = None
    ) -> BitMap:
        """Get FCL for a specific cortical area at a specific timestep. Handles
        both standard and memory cortical areas.

        Args:
            cortical_idx: ID of the cortical area to query (must be integer)
            timestep: Optional timestep to query (defaults to current timestep)

        Returns:
            BitMap of neurons firing in the specified area at the specified timestep

        Raises:
            TypeError: If cortical_idx is not an integer
        """
        # CRITICAL FIX: Validate cortical_idx type to prevent silent failures
        if not isinstance(cortical_idx, int):
            raise TypeError(
                f"cortical_idx must be an integer (cortical_idx), got {type(cortical_idx).__name__}: {cortical_idx}"
            )

        if timestep is None:
            timestep = self.current_timestep

        # Handle memory cortical areas with custom window sizes
        if self.is_memory_cortical(cortical_idx):
            if cortical_idx not in self.custom_cortical_history:
                return BitMap()

            _, history_array, _ = self.custom_cortical_history[cortical_idx]
            custom_index = self._get_custom_cortical_index(
                cortical_idx, timestep
            )
            return history_array[custom_index].copy()
        else:
            # Handle standard cortical areas
            index = self._get_index_for_timestep(timestep)

            # Return empty bitmap if area not initialized
            if cortical_idx not in self.cortical_fcl_history:
                return BitMap()

            # Return copy to prevent modification of internal state
            return self.cortical_fcl_history[cortical_idx][index].copy()

    def get_fcl_by_cortical(
        self, timestep: Optional[int] = None
    ) -> Dict[CorticalIdx, BitMap]:
        """Return a dictionary mapping each cortical area to its firing
        neurons. Handles both standard and memory cortical areas.

        Args:
            timestep: Optional timestep to query (defaults to current timestep)

        Returns:
            Dict[cortical_idx, BitMap] mapping areas to their active neurons
        """
        if timestep is None:
            timestep = self.current_timestep

        result: Dict[CorticalIdx, BitMap] = {}

        # Handle standard cortical areas
        index = self._get_index_for_timestep(timestep)
        for cortical_idx, fcl_history in self.cortical_fcl_history.items():
            if not fcl_history[index].is_empty():
                result[cortical_idx] = fcl_history[index].copy()

        # Handle memory cortical areas
        for cortical_idx in self.memory_cortical_indices:
            cortical_fcl = self.get_cortical_fcl(cortical_idx, timestep)
            if not cortical_fcl.is_empty():
                result[cortical_idx] = cortical_fcl

        return result

    def get_active_corticals(
        self, timestep: Optional[int] = None
    ) -> Set[CorticalIdx]:
        """Get cortical areas that have firing neurons at the specified
        timestep. Handles both standard and memory cortical areas.

        Args:
            timestep: Optional timestep (defaults to current)

        Returns:
            Set of cortical area IDs with active neurons

        Raises:
            TimestepOutOfRangeError: If timestep is outside valid range
        """
        if timestep is None:
            timestep = self.current_timestep

        active_corticals: Set[CorticalIdx] = set()

        # Check standard cortical areas
        index = self._get_index_for_timestep(timestep)
        for cortical_idx, fcl_history in self.cortical_fcl_history.items():
            if not fcl_history[index].is_empty():
                active_corticals.add(cortical_idx)

        # Check memory cortical areas
        for cortical_idx in self.memory_cortical_indices:
            cortical_fcl = self.get_cortical_fcl(cortical_idx, timestep)
            if not cortical_fcl.is_empty():
                active_corticals.add(cortical_idx)

        return active_corticals

    def get_cortical_temporal_pattern(
        self, cortical_idx: CorticalIdx, n_steps: int
    ) -> BitMap:
        """Get combined firing pattern for a memory-type cortical area over
        multiple timesteps. Similar to get_neurons_fired_in_last_n_steps but
        optimized for memory areas.

        Args:
            cortical_idx: ID of the memory-type cortical area
            n_steps: Number of timesteps to include

        Returns:
            BitMap of all neurons that fired in the area over the timespan

        Raises:
            ValueError: If n_steps is negative or cortical_idx is not a memory area
        """
        if not self.is_memory_cortical(cortical_idx):
            raise ValueError(
                f"Area {cortical_idx} is not registered as a memory area"
            )

        if n_steps <= 0:
            raise ValueError(f"n_steps must be positive, got {n_steps}")

        window_size, history_array, _ = self.custom_cortical_history[
            cortical_idx
        ]

        result = BitMap()
        for i in range(n_steps):
            if i >= window_size:
                break

            step_index = (
                self._get_custom_cortical_index(
                    cortical_idx, self.current_timestep
                )
                - i
            ) % window_size
            result = result | history_array[step_index]

        return result

    def get_memory_cortical_consistency(
        self,
        cortical_idx: CorticalIdx,
        pattern_duration: int,
        window_duration: int,
    ) -> float:
        """Calculate how consistently a pattern has been maintained in a memory
        cortical.

        Args:
            cortical_idx: ID of the memory-type cortical area
            pattern_duration: Duration of the pattern to analyze (in timesteps)
            window_duration: Total window to evaluate (must be >= pattern_duration)

        Returns:
            Consistency score between 0.0 (no consistency) and 1.0 (perfect consistency)

        Raises:
            ValueError: If cortical_idx is not a memory cortical or if window parameters are invalid
        """
        if not self.is_memory_cortical(cortical_idx):
            raise ValueError(
                f"Cortical {cortical_idx} is not registered as a memory cortical"
            )

        if window_duration < pattern_duration:
            raise ValueError(
                f"Window duration ({window_duration}) must be >= pattern duration ({pattern_duration})"
            )

        window_size, history_array, start_timestep = (
            self.custom_cortical_history[cortical_idx]
        )

        if pattern_duration > window_size:
            pattern_duration = window_size
            self.logger.warning(
                f"Pattern duration {pattern_duration} exceeds window size {window_size}, limiting to window size"
            )

        if window_duration > window_size:
            window_duration = window_size
            self.logger.warning(
                f"Window duration {window_duration} exceeds window size {window_size}, limiting to window size"
            )

        # Get the pattern to check for consistency
        pattern = self.get_cortical_temporal_pattern(
            cortical_idx, pattern_duration
        )

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

                idx = self._get_custom_cortical_index(cortical_idx, timestep)
                step_neurons = history_array[idx]

                # Check if this timestep's neurons match the pattern
                if step_neurons != pattern:
                    match = False
                    break

            if match:
                match_count += 1

        # Return proportion of positions where pattern was found
        possible_positions = window_duration - pattern_duration + 1
        return (
            match_count / possible_positions if possible_positions > 0 else 0.0
        )

    def get_consistent_neurons_in_memory_cortical(
        self, cortical_idx: CorticalIdx, n_steps: int
    ) -> BitMap:
        """Get neurons in a memory cortical area that fired consistently across
        all specified timesteps.

        Args:
            cortical_idx: ID of the memory-type cortical area
            n_steps: Number of timesteps to look back

        Returns:
            BitMap of neurons that fired in ALL of the last n_steps

        Raises:
            ValueError: If n_steps is negative or cortical_idx is not a memory area
        """
        if not self.is_memory_cortical(cortical_idx):
            raise ValueError(
                f"Area {cortical_idx} is not registered as a memory area"
            )

        if n_steps <= 0:
            raise ValueError(f"n_steps must be positive, got {n_steps}")

        window_size, history_array, start_timestep = (
            self.custom_cortical_history[cortical_idx]
        )

        if n_steps > window_size:
            n_steps = window_size
            self.logger.warning(
                f"n_steps {n_steps} exceeds window size {window_size}, limiting to window size"
            )

        # Initialize result with neurons from the first relevant timestep
        timestep = self.current_timestep
        custom_index = self._get_custom_cortical_index(cortical_idx, timestep)
        result = history_array[custom_index].copy()

        # Intersect with each subsequent timestep
        for i in range(1, n_steps):
            if self.current_timestep - i < start_timestep:
                # If we can't go back far enough, return empty set
                return BitMap()

            timestep = self.current_timestep - i
            custom_index = self._get_custom_cortical_index(
                cortical_idx, timestep
            )
            result = result & history_array[custom_index]

            if result.is_empty():
                # Short-circuit if intersection becomes empty
                break

        return result

    def get_neurons_by_corticals(
        self,
        cortical_indices: List[CorticalIdx],
        timestep: Optional[int] = None,
    ) -> BitMap:
        """Get neurons firing in any of the specified cortical areas. Handles
        both standard and memory cortical areas.

        Args:
            cortical_indices: List of cortical area IDs to query
            timestep: Optional timestep (defaults to current)

        Returns:
            BitMap of neuron IDs active in any of the specified areas

        Raises:
            TimestepOutOfRangeError: If timestep is outside valid range
        """
        if timestep is None:
            timestep = self.current_timestep

        result = BitMap()
        for cortical_idx in cortical_indices:
            cortical_fcl = self.get_cortical_fcl(cortical_idx, timestep)
            result = result | cortical_fcl

        return result

    def get_neurons_fired_in_last_n_steps(
        self,
        n_steps: int,
        cortical_indices: Optional[List[CorticalIdx]] = None,
    ) -> BitMap:
        """Get neurons that fired in any of the last n timesteps. Optionally
        filter by specific cortical areas. Handles both standard and memory
        cortical areas.

        Args:
            n_steps: Number of timesteps to look back
            cortical_indices: Optional list of cortical area IDs to filter by

        Returns:
            BitMap containing neuron IDs that fired in the specified timespan

        Raises:
            ValueError: If n_steps is negative or exceeds window size
        """
        if n_steps <= 0:
            raise ValueError(f"n_steps must be positive, got {n_steps}")

        if n_steps > self.default_window_size:
            n_steps = self.default_window_size
            self.logger.warning(
                f"For standard areas, n_steps {n_steps} exceeds default window size {self.default_window_size}, limiting to default window size"
            )

        result = BitMap()

        # No specific areas requested, use global FCL
        if cortical_indices is None:
            for i in range(n_steps):
                step_index = (
                    self.current_window_index - i
                ) % self.default_window_size
                result = result | self.global_fcl_history[step_index]
        else:
            # Process both standard and memory areas
            for cortical_idx in cortical_indices:
                if self.is_memory_cortical(cortical_idx):
                    #  For memory areas, use their specialized temporal pattern
                    #  function
                    area_result = self.get_cortical_temporal_pattern(
                        cortical_idx, n_steps
                    )
                    result = result | area_result
                else:
                    # For standard areas, use the default processing
                    if cortical_idx in self.cortical_fcl_history:
                        for i in range(min(n_steps, self.default_window_size)):
                            step_index = (
                                self.current_window_index - i
                            ) % self.default_window_size
                            result = (
                                result
                                | self.cortical_fcl_history[cortical_idx][
                                    step_index
                                ]
                            )

        return result

    def get_consistently_active_neurons(
        self,
        n_steps: int,
        cortical_indices: Optional[List[CorticalIdx]] = None,
    ) -> BitMap:
        """Get neurons that fired in ALL of the last n timesteps.

        Args:
            n_steps: Number of timesteps to look back
            cortical_indices: Optional list of cortical area IDs to filter by

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

        if cortical_indices is None:
            # Start with all neurons from first timestep
            result = self.global_fcl_history[first_step_index].copy()

            # Intersect with each subsequent timestep
            for i in range(1, n_steps):
                step_index = (self.current_window_index - i) % self.window_size
                result = result & self.global_fcl_history[step_index]
        else:
            # Filter by specified areas
            first_step_neurons = self.get_neurons_by_corticals(
                cortical_indices, timestep=self.current_timestep - n_steps + 1
            )
            result = first_step_neurons

            # Intersect with each subsequent timestep
            for i in range(1, n_steps):
                current_step = self.current_timestep - n_steps + 1 + i
                filtered_step = self.get_neurons_by_corticals(
                    cortical_indices, timestep=current_step
                )
                result = result & filtered_step

        return result

    def get_fcl_delta(
        self,
        start_time: int,
        end_time: int,
        cortical_indices: Optional[List[CorticalIdx]] = None,
    ) -> BitMap:
        """Get neurons that became active between start and end times.

        Args:
            start_time: Starting timestep
            end_time: Ending timestep
            cortical_indices: Optional list of cortical area IDs to filter by

        Returns:
            BitMap of neurons that became active between the two timesteps

        Raises:
            TimestepOutOfRangeError: If timesteps are outside valid range
            ValueError: If start_time is greater than end_time
        """
        if start_time > end_time:
            raise ValueError(
                f"start_time ({start_time}) must be <= end_time ({end_time})"
            )

        time_diff = abs(end_time - start_time)
        if time_diff > self.window_size:
            raise TimestepOutOfRangeError(
                f"Time difference ({time_diff}) exceeds history window size ({self.window_size})"
            )

        start_index = self._get_index_for_timestep(start_time)
        end_index = self._get_index_for_timestep(end_time)

        if cortical_indices is None:
            # Neurons active at end but not at start (newly activated)
            return (
                self.global_fcl_history[end_index]
                - self.global_fcl_history[start_index]
            )
        else:
            # Filter by specified areas
            start_neurons = self.get_neurons_by_corticals(
                cortical_indices, timestep=start_time
            )
            end_neurons = self.get_neurons_by_corticals(
                cortical_indices, timestep=end_time
            )

            # Neurons active at end but not at start (newly activated)
            return end_neurons - start_neurons

    def get_fcl_xor(
        self,
        time1: int,
        time2: int,
        cortical_indices: Optional[List[CorticalIdx]] = None,
    ) -> BitMap:
        """Get neurons that fired at either time1 or time2, but not both.
        Useful for detecting changes in firing patterns.

        Args:
            time1: First timestep
            time2: Second timestep
            cortical_indices: Optional list of cortical area IDs to filter by

        Returns:
            BitMap of neurons that changed activation state between timesteps

        Raises:
            TimestepOutOfRangeError: If timesteps are outside valid range
        """
        idx1 = self._get_index_for_timestep(time1)
        idx2 = self._get_index_for_timestep(time2)

        if cortical_indices is None:
            return (
                self.global_fcl_history[idx1] ^ self.global_fcl_history[idx2]
            )
        else:
            neurons1 = self.get_neurons_by_corticals(
                cortical_indices, timestep=time1
            )
            neurons2 = self.get_neurons_by_corticals(
                cortical_indices, timestep=time2
            )

            return neurons1 ^ neurons2

    def count_firing_neurons(
        self,
        timestep: Optional[int] = None,
        cortical_idx: Optional[CorticalIdx] = None,
    ) -> int:
        """Efficiently count the number of firing neurons. Handles both
        standard and memory cortical areas.

        Args:
            timestep: Optional timestep (defaults to current)
            cortical_idx: Optional cortical area ID to count neurons for a specific area

        Returns:
            Count of firing neurons

        Raises:
            TimestepOutOfRangeError: If timestep is outside valid range
        """
        if timestep is None and cortical_idx is None:
            # Use cached total for current timestep
            return self.total_neurons_fired

        if timestep is None:
            timestep = self.current_timestep

        if cortical_idx is None:
            # Count all neurons at the specified timestep
            index = self._get_index_for_timestep(timestep)
            return len(self.global_fcl_history[index])
        else:
            #  Count neurons in the specified area (handles both standard and
            #  memory corticals)
            cortical_fcl = self.get_cortical_fcl(cortical_idx, timestep)
            return len(cortical_fcl)

    def get_firing_statistics(self) -> Dict[str, Any]:
        """Get statistics about firing patterns across the FCL history.

        Returns:
            Dictionary with statistics about firing neurons and active corticals
        """
        active_corticals = self.get_active_corticals()

        stats = {
            "total_neurons_fired": self.total_neurons_fired,
            "neurons_per_cortical": self.neurons_per_cortical,
            "active_corticals_count": len(active_corticals),
            "active_corticals": list(active_corticals),
            "memory_corticals_count": len(self.memory_cortical_indices),
            "memory_corticals": list(self.memory_cortical_indices),
            "current_window_index": self.current_window_index,
            "current_timestep": self.current_timestep,
        }

        return stats

    # Methods for two-phase membrane potential update process

    def advance_timestep(self) -> None:
        """Advance to the next timestep, shifting FCL history."""
        self.current_timestep += 1
        self.current_window_index = (
            self.current_window_index + 1
        ) % self.default_window_size

        # Clear the current FCL slot (both global and per-area)
        self.global_fcl_history[self.current_window_index].clear()

        for cortical_idx in self.cortical_fcl_history:
            # Only use custom index calculation for memory corticals
            if self.is_memory_cortical(cortical_idx):
                idx = self._get_custom_cortical_index(
                    cortical_idx, self.current_timestep
                )
            else:
                # For standard corticals, use the current window index
                idx = self.current_window_index
            self.cortical_fcl_history[cortical_idx][idx].clear()

        # Reset update queue for new timestep
        self._reset_update_queue()

    def _reset_update_queue(self) -> None:
        """Reset the membrane potential update queue for a new timestep."""
        if not hasattr(self, "membrane_update_queue"):
            self.membrane_update_queue: List[MembraneUpdate] = []
        else:
            self.membrane_update_queue = []

    def queue_membrane_update(
        self,
        neuron_idx: int,
        delta_potential: float,
        source_neuron_idx: Optional[int] = None,
    ) -> None:
        """Queue an update to a neuron's membrane potential.

        Args:
            neuron_idx: Index of the neuron to update
            delta_potential: Change in membrane potential
            source_neuron_idx: Source of the update (if from a specific neuron)
        """
        if not hasattr(self, "membrane_update_queue"):
            self._reset_update_queue()

        self.membrane_update_queue.append(
            MembraneUpdate(
                neuron_idx=neuron_idx,
                delta_potential=delta_potential,
                source_neuron_idx=source_neuron_idx,
            )
        )

    def process_update_queue(self) -> List[Tuple[int, float]]:
        """Process the membrane potential update queue, aggregating updates per
        neuron.

        Returns:
            List of (neuron_idx, total_delta) tuples with the final updates
        """
        if (
            not hasattr(self, "membrane_update_queue")
            or not self.membrane_update_queue
        ):
            return []

        # Aggregate updates by neuron
        neuron_updates: Dict[int, float] = {}

        for update in self.membrane_update_queue:
            neuron_updates[update.neuron_idx] = (
                neuron_updates.get(update.neuron_idx, 0.0)
                + update.delta_potential
            )

        # Convert to list of tuples
        aggregated_updates = [
            (idx, delta) for idx, delta in neuron_updates.items()
        ]

        # Clear the queue after processing
        self.membrane_update_queue = []

        return aggregated_updates

    def get_firing_neurons(self, offset: int = -1) -> List[int]:
        """Get list of neuron indices that are in the FCL at the specified
        offset.

        Args:
            offset: Timestep offset (-1 for previous timestep, which is the firing phase)

        Returns:
            List of neuron indices
        """
        fcl = self.get_global_fcl(
            self.current_timestep + offset if offset else None
        )
        return list(fcl)

    def add_to_current_fcl(
        self, neuron_indices: Union[List[int], Set[int], BitMap]
    ) -> None:
        """Add neurons to the current timestep's FCL.

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
        """Get the FCL for a specific timestep relative to current.

        Args:
            offset: Timestep offset (0 for current, -1 for previous, etc.)

        Returns:
            BitMap of neuron indices in the FCL
        """
        return self.get_global_fcl(
            self.current_timestep + offset if offset else None
        )


# Example usage
def example_fcl_usage() -> None:
    """Example usage of the FCL manager."""

    # Create FCL manager with window size of 3
    fcl_manager = FCLManager(window_size=3)

    # Example data for first timestep: cortical_idx -> neurons firing
    firing_neurons_t1: Dict[CorticalIdx, BitMap] = {
        100: BitMap([1001, 1002, 1005, 1008]),
        200: BitMap([2001, 2010, 2015]),
        300: BitMap([3001, 3002, 3003, 3004, 3005]),
    }

    # Update FCL with timestep 1 data
    fcl_manager.update_fcl(
        current_timestep=1, neurons_by_cortical=firing_neurons_t1
    )

    # Example data for second timestep
    firing_neurons_t2: Dict[CorticalIdx, Union[BitMap, List[int]]] = {
        100: BitMap([1002, 1003, 1009]),
        200: BitMap([2001, 2005]),
        300: [3002, 3005, 3010],  # Mix of collection types to show conversion
    }

    # Update FCL with timestep 2 data
    fcl_manager.update_fcl(
        current_timestep=2, neurons_by_cortical=firing_neurons_t2
    )

    # Example data for third timestep
    firing_neurons_t3: Dict[CorticalIdx, Union[BitMap, Set[int]]] = {
        100: BitMap([1003, 1004, 1005]),
        200: set([2001, 2002, 2003]),  # Set instead of bitmap
        300: BitMap([3001, 3005, 3008]),
    }

    # Update FCL with timestep 3 data
    fcl_manager.update_fcl(
        current_timestep=3, neurons_by_cortical=firing_neurons_t3
    )

    # Get global FCL for current timestep
    global_fcl = fcl_manager.get_global_fcl()
    print(f"All firing neurons at timestep 3: {global_fcl}")

    # Get FCL for a specific cortical area
    area_fcl = fcl_manager.get_cortical_fcl(cortical_idx=100)
    print(f"Firing neurons in cortical area 100: {area_fcl}")

    # Get FCL for timestep 2
    prev_fcl = fcl_manager.get_global_fcl(timestep=2)
    print(f"All firing neurons at timestep 2: {prev_fcl}")

    # Get all firing neurons across areas 100 and 300
    selected_areas_fcl = fcl_manager.get_neurons_by_corticals([100, 300])
    print(f"Firing neurons in areas 100 and 300: {selected_areas_fcl}")

    # Get neurons that fired in any of the last 2 timesteps (3 and 2)
    recent_fcl = fcl_manager.get_neurons_fired_in_last_n_steps(2)
    print(f"Neurons firing in recent 2 timesteps: {recent_fcl}")

    # Get active cortical areas
    active_areas = fcl_manager.get_active_corticals()
    print(f"Active cortical areas: {active_areas}")


def example_enhanced_fcl_usage() -> None:
    """Example showing the usage of the EnhancedFCLManager with memory
    corticals."""
    # Create manager
    fcl_manager = FCLManager(window_size=5)

    # Register memory corticals
    fcl_manager.register_memory_cortical(
        400, window_size=10
    )  # 10 timesteps of history
    fcl_manager.register_memory_cortical(
        500, window_size=20
    )  # 20 timesteps of history

    # Simulate some activity over multiple timesteps
    for t in range(30):
        # Create firing neurons for this timestep
        # In a real system, these would come from the neural processing unit
        neurons_by_cortical = {
            100: BitMap([1, 2, 3]),  # Standard cortical
            200: BitMap([4, 5, 6]),  # Standard cortical
            300: BitMap([7, 8, 9]),  # Standard cortical
            400: BitMap([10, 11, 12]),  # Memory cortical with 10-step history
            500: BitMap([13, 14, 15]),  # Memory cortical with 20-step history
        }

        # Every 5 steps, change the pattern slightly
        if t % 5 == 0:
            # Change pattern for memory area 400
            neurons_by_cortical[400] = BitMap([10, 11, 12, t])

        # Add activity for this timestep
        fcl_manager.update_fcl(t, neurons_by_cortical)

        # WGPU-COMPATIBLE: Use logger instead of print for output
        if t % 5 == 0:
            logger = setup_logger("feagi.npu.fcl_manager.enhanced_example")
            logger.info(
                f"Timestep {t} - Total firing neurons: {fcl_manager.total_neurons_fired}"
            )

    # Get neurons from specific corticals
    #  standard_cortical_fcl = fcl_manager.get_cortical_fcl(100) # Unused
    #  variable removed
    #  memory_cortical_fcl = fcl_manager.get_cortical_fcl(400) # Unused
    #  variable removed

    # Get combined activity from multiple corticals
    #  selected_corticals_fcl = fcl_manager.get_neurons_by_corticals([100,
    #  300]) # Unused variable removed

    # Check for temporal patterns in the memory cortical
    #  temporal_pattern = fcl_manager.get_cortical_temporal_pattern(400,
    #  n_steps=5) # Unused variable removed

    # For memory corticals, we can analyze consistency over time
    # pattern_consistency = fcl_manager.get_memory_cortical_consistency(
    #     400,  # Memory cortical ID
    #     pattern_duration=3,  # Length of the pattern to check
    #     window_duration=8,  # Total window to analyze
    # )  # Unused variable removed

    # Get the most consistently active neurons in the memory cortical
    #  consistent_neurons =
    #  fcl_manager.get_consistent_neurons_in_memory_cortical(
    #     400, n_steps=5
    # )  # Unused variable removed

    # We can access firing history from the last n timesteps
    # Even for a mix of standard and memory corticals
    memory_corticals = [400, 500]
    # recent_memory_fcl = fcl_manager.get_neurons_fired_in_last_n_steps(
    #     10, memory_corticals
    # )  # Unused variable removed

    # WGPU-COMPATIBLE: Use logger instead of print for final output
    logger = setup_logger("feagi.npu.fcl_manager.enhanced_example")
    logger.info(
        f"Completed simulation with {fcl_manager.total_neurons_fired} total firing neurons"
    )


def inject_neurons_into_fcl(
    fcl_manager, cortical_idx, neuron_ids, timestep=None
):
    """Helper function to inject neurons into an FCL for a specific cortical
    area.

    Args:
        fcl_manager: FCL manager instance
        cortical_idx: Cortical area ID
        neuron_ids: List of neuron IDs to inject
        timestep: Optional timestep, defaults to current
    """
    neurons_by_cortical = {cortical_idx: neuron_ids}
    if timestep is None:
        timestep = fcl_manager.current_timestep
    fcl_manager.update_fcl(timestep, neurons_by_cortical)


if __name__ == "__main__":
    # Set up logging
    logging.basicConfig(level=logging.DEBUG)

    # Run examples
    example_fcl_usage()
    example_enhanced_fcl_usage()

    # Initialize the enhanced FCL
    fcl = FCLManager(window_size=5)

    # Register a memory area
    fcl.register_memory_cortical(cortical_idx=500, window_size=100)

    # Inject specific neurons into the hippocampus memory area
    hippocampus_neurons = [501, 502, 503, 504]
    inject_neurons_into_fcl(
        fcl, cortical_idx=500, neuron_ids=hippocampus_neurons
    )

    # Check the injected neurons
    result = fcl.get_cortical_fcl(cortical_idx=500)
    logger = setup_logger("feagi.npu.fcl_manager.main_example")
    logger.info(f"Neurons in hippocampus: {result}")

# Compatibility aliases for backward compatibility
HierarchicalFCL = FCLManager
EnhancedHierarchicalFCL = FCLManager

# Legacy alias for backward compatibility
EnhancedFCLManager = FCLManager
