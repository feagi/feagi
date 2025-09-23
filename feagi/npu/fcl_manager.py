"""
FCL Manager - Compatibility layer for legacy tests

This module provides FCLManager and EnhancedFCLManager classes that wrap
the new FireCandidateList architecture to maintain compatibility with existing tests.
"""

from typing import Dict, List, Optional, Set, Tuple, Any
import time
import numpy as np
from feagi.utils.logger import setup_logger
from .fire_candidate_list import FireCandidateList, FCLCandidate
from .fire_ledger import RoaringBitmap

logger = setup_logger(__name__)


class BitMap:
    """Bitmap implementation compatible with legacy tests."""
    
    def __init__(self, elements: Optional[Set[int]] = None):
        """Initialize bitmap."""
        self._elements = set(elements) if elements else set()
    
    def add(self, element: int) -> None:
        """Add element to bitmap."""
        self._elements.add(element)
    
    def clear(self) -> None:
        """Clear all elements."""
        self._elements.clear()
    
    def copy(self) -> 'BitMap':
        """Create a copy."""
        return BitMap(self._elements.copy())
    
    def __len__(self) -> int:
        """Get number of elements."""
        return len(self._elements)
    
    def __iter__(self):
        """Iterate over elements."""
        return iter(sorted(self._elements))
    
    def __contains__(self, element: int) -> bool:
        """Check if element exists."""
        return element in self._elements
    
    def __bool__(self) -> bool:
        """Check if bitmap is non-empty."""
        return len(self._elements) > 0
    
    def __or__(self, other: 'BitMap') -> 'BitMap':
        """Union operation."""
        return BitMap(self._elements | other._elements)
    
    def __and__(self, other: 'BitMap') -> 'BitMap':
        """Intersection operation."""
        return BitMap(self._elements & other._elements)
    
    def __sub__(self, other: 'BitMap') -> 'BitMap':
        """Difference operation (elements in self but not in other)."""
        return BitMap(self._elements - other._elements)
    
    def __contains__(self, item: int) -> bool:
        """Check if item is in the bitmap."""
        return item in self._elements
    
    def __xor__(self, other: 'BitMap') -> 'BitMap':
        """XOR operation (symmetric difference - elements in either but not both)."""
        return BitMap(self._elements ^ other._elements)
    
    def to_bitmap(self) -> 'BitMap':
        """Convert to BitMap format."""
        return BitMap(self._elements)
    
    def is_empty(self) -> bool:
        """Check if bitmap is empty."""
        return len(self._elements) == 0
    
    def size(self) -> int:
        """Get number of elements in bitmap."""
        return len(self._elements)
    
    def clear(self) -> None:
        """Clear all elements from bitmap."""
        self._elements.clear()
    
    def add(self, item: int) -> None:
        """Add an element to the bitmap."""
        self._elements.add(item)
    
    def remove(self, item: int) -> None:
        """Remove an element from the bitmap."""
        self._elements.discard(item)  # Use discard to avoid KeyError


class FCLError(Exception):
    """General FCL error."""
    pass


class TimestepOutOfRangeError(FCLError):
    """Exception for timestep out of range errors."""
    pass


class MembraneUpdate:
    """Membrane update representation."""
    
    def __init__(self, neuron_id: int, delta: float, is_excitatory: bool = True):
        """Initialize membrane update."""
        self.neuron_id = neuron_id
        self.delta = delta
        self.is_excitatory = is_excitatory


class NeuronCollectionType:
    """Types of neuron collections."""
    BITMAP = "bitmap"
    LIST = "list"
    ARRAY = "array"
    SET = "set"  # Support for Python set collections


class NeuronCollection:
    """Collection of neurons with different backing types."""
    
    def __init__(self, neurons: Optional[Any] = None, collection_type: str = NeuronCollectionType.BITMAP):
        """Initialize neuron collection."""
        # Validate collection type
        valid_types = [NeuronCollectionType.BITMAP, NeuronCollectionType.LIST, 
                      NeuronCollectionType.ARRAY, NeuronCollectionType.SET]
        if collection_type not in valid_types:
            raise TypeError(f"Invalid collection_type '{collection_type}'. Must be one of: {valid_types}")
        
        self.collection_type = collection_type
        
        if collection_type == NeuronCollectionType.BITMAP:
            if isinstance(neurons, BitMap):
                self.data = neurons
            elif isinstance(neurons, set):
                self.data = BitMap(neurons)
            elif isinstance(neurons, list):
                self.data = BitMap(set(neurons))
            elif neurons is None:
                self.data = BitMap()
            else:
                # Try to convert unsupported types - raise TypeError if fails
                try:
                    self.data = BitMap(set(neurons))
                except (TypeError, ValueError):
                    raise TypeError(f"Cannot create BitMap from {type(neurons)}. Expected list, set, or BitMap.")
        elif collection_type == NeuronCollectionType.SET:
            if isinstance(neurons, set):
                self.data = neurons
            elif isinstance(neurons, list):
                self.data = set(neurons)
            elif neurons is None:
                self.data = set()
            else:
                # Try to convert unsupported types - raise TypeError if fails
                try:
                    self.data = set(neurons)
                except (TypeError, ValueError):
                    raise TypeError(f"Cannot create set from {type(neurons)}. Expected iterable.")
        else:
            # LIST or ARRAY types
            if neurons is None:
                self.data = []
            elif hasattr(neurons, '__iter__') and not isinstance(neurons, (str, bytes)):
                self.data = list(neurons)
            else:
                raise TypeError(f"Cannot create list from {type(neurons)}. Expected iterable.")
    
    def __len__(self) -> int:
        """Get collection size."""
        return len(self.data)
    
    def __iter__(self):
        """Iterate over collection."""
        return iter(self.data)
    
    def to_bitmap(self) -> 'BitMap':
        """Convert collection to BitMap format."""
        if self.collection_type == NeuronCollectionType.BITMAP:
            return self.data
        elif self.collection_type == NeuronCollectionType.SET:
            return BitMap(self.data)
        else:
            # LIST or ARRAY types
            return BitMap(set(self.data))
    
    def to_list(self) -> List[int]:
        """Convert collection to list format."""
        if self.collection_type == NeuronCollectionType.LIST:
            return self.data
        elif self.collection_type == NeuronCollectionType.SET:
            return list(self.data)
        elif self.collection_type == NeuronCollectionType.BITMAP:
            return list(self.data._elements)
        else:
            return list(self.data)
    
    def to_set(self) -> set:
        """Convert collection to set format."""
        if self.collection_type == NeuronCollectionType.SET:
            return self.data
        elif self.collection_type == NeuronCollectionType.LIST:
            return set(self.data)
        elif self.collection_type == NeuronCollectionType.BITMAP:
            return self.data._elements
        else:
            return set(self.data)
    
    @classmethod
    def from_any(cls, data: Any) -> 'NeuronCollection':
        """Create NeuronCollection from any input, detecting appropriate type."""
        if isinstance(data, list):
            return cls(data, NeuronCollectionType.LIST)
        elif isinstance(data, set):
            return cls(data, NeuronCollectionType.SET)  # Preserve set type
        elif isinstance(data, BitMap):
            return cls(data, NeuronCollectionType.BITMAP)
        elif isinstance(data, (str, bytes)):
            # Strings and bytes are iterable but shouldn't be treated as neuron collections
            raise TypeError(f"Cannot create NeuronCollection from {type(data).__name__}. Strings/bytes not supported.")
        elif hasattr(data, '__iter__'):
            # Other iterables - convert to list
            try:
                return cls(list(data), NeuronCollectionType.LIST)
            except (TypeError, ValueError) as e:
                raise TypeError(f"Cannot create NeuronCollection from {type(data).__name__}: {e}")
        else:
            # Non-iterable types
            raise TypeError(f"Cannot create NeuronCollection from {type(data).__name__}. Expected list, set, BitMap, or iterable.")




class FCLManager:
    """Fire Candidate List Manager - compatibility wrapper."""
    
    def __init__(self, window_size: int = 3):
        """Initialize FCL Manager."""
        self.window_size = window_size
        self._fcl = FireCandidateList()
        self._last_clear_time = time.time()
        self._injection_count = 0
        
        # Historical data for windowed operations
        self._history_window = []
        self._max_history = window_size
    
    def update_fcl(self, cortical_area: str, neuron_updates: Dict[int, Union[float, List[float]]]) -> None:
        """Update FCL with neuron candidates.
        
        Args:
            cortical_area: Cortical area identifier
            neuron_updates: Dictionary mapping neuron_id to delta(s). 
                          Values can be single float or list of floats for multiple synaptic inputs.
        """
        if not isinstance(neuron_updates, dict):
            raise FCLError(f"neuron_updates must be a dictionary, got {type(neuron_updates)}")
        
        candidates = []
        for neuron_id, deltas in neuron_updates.items():
            # Handle both single values and lists of values
            if isinstance(deltas, (list, tuple)):
                # Multiple synaptic inputs for same neuron
                for delta in deltas:
                    candidate = FCLCandidate(
                        neuron_id=int(neuron_id),
                        delta_potential=float(delta),
                        is_excitatory=delta >= 0
                    )
                    candidates.append(candidate)
            else:
                # Single synaptic input
                candidate = FCLCandidate(
                    neuron_id=int(neuron_id),
                    delta_potential=float(deltas),
                    is_excitatory=deltas >= 0
                )
                candidates.append(candidate)
        
        self._fcl.update_fcl(cortical_area, candidates)
        self._injection_count += len(candidates)
    
    def get_fcl_candidates(self, cortical_area: str) -> List[FCLCandidate]:
        """Get FCL candidates for a cortical area."""
        return self._fcl.candidates.get(cortical_area, [])
    
    def get_candidate_count(self, cortical_area: str) -> int:
        """Get number of candidates for an area."""
        return len(self._fcl.candidates.get(cortical_area, []))
    
    def clear_fcl(self) -> None:
        """Clear all FCL candidates."""
        # Store current state in history before clearing
        if len(self._history_window) >= self._max_history:
            self._history_window.pop(0)
        
        current_state = {}
        for area, candidates in self._fcl.candidates.items():
            neuron_ids = [c.neuron_id for c in candidates]
            current_state[area] = BitMap(set(neuron_ids))
        
        self._history_window.append(current_state)
        self._fcl.clear()
        self._last_clear_time = time.time()
    
    def get_fcl_delta(self, cortical_filter: Optional[List[str]] = None) -> Dict[str, BitMap]:
        """Get FCL delta as bitmap."""
        result = {}
        for cortical_area, candidates in self._fcl.candidates.items():
            if cortical_filter is None or cortical_area in cortical_filter:
                neuron_ids = [c.neuron_id for c in candidates]
                result[cortical_area] = BitMap(set(neuron_ids))
        return result
    
    def get_consistently_active_neurons(self, timesteps: int, cortical_filter: Optional[List[str]] = None) -> Dict[str, BitMap]:
        """Get consistently active neurons over N timesteps."""
        if timesteps <= 0:
            raise TimestepOutOfRangeError("timesteps must be positive")
        
        if timesteps > len(self._history_window):
            timesteps = len(self._history_window)
        
        result = {}
        
        # Get areas to check
        all_areas = set()
        for hist_state in self._history_window[-timesteps:]:
            all_areas.update(hist_state.keys())
        
        if cortical_filter:
            all_areas = all_areas.intersection(set(cortical_filter))
        
        for area in all_areas:
            # Find neurons active in all timesteps
            consistent_neurons = None
            
            for hist_state in self._history_window[-timesteps:]:
                area_bitmap = hist_state.get(area, BitMap())
                if consistent_neurons is None:
                    consistent_neurons = area_bitmap.copy()
                else:
                    consistent_neurons = consistent_neurons & area_bitmap
            
            result[area] = consistent_neurons if consistent_neurons else BitMap()
        
        return result
    
    def get_neurons_fired_in_last_n_steps(self, n: int, cortical_filter: Optional[List[str]] = None) -> Dict[str, BitMap]:
        """Get neurons that fired in last N steps."""
        if n <= 0:
            raise TimestepOutOfRangeError("n must be positive")
        
        if n > len(self._history_window):
            n = len(self._history_window)
        
        result = {}
        
        # Get areas to check
        all_areas = set()
        for hist_state in self._history_window[-n:]:
            all_areas.update(hist_state.keys())
        
        if cortical_filter:
            all_areas = all_areas.intersection(set(cortical_filter))
        
        for area in all_areas:
            # Union all neurons from last N steps
            union_neurons = BitMap()
            
            for hist_state in self._history_window[-n:]:
                area_bitmap = hist_state.get(area, BitMap())
                union_neurons = union_neurons | area_bitmap
            
            result[area] = union_neurons
        
        return result
    
    @property
    def injection_count(self) -> int:
        """Get total injection count."""
        return self._injection_count


class EnhancedFCLManager(FCLManager):
    """Enhanced FCL Manager with additional features."""
    
    def __init__(self, window_size: int = 3, enhanced_features: bool = True):
        """Initialize Enhanced FCL Manager."""
        super().__init__(window_size)
        self.enhanced_features = enhanced_features
        self._performance_metrics = {
            'total_updates': 0,
            'avg_update_time': 0.0,
            'peak_candidates': 0
        }
    
    def update_fcl(self, cortical_area: str, neuron_updates: Dict[int, float]) -> None:
        """Enhanced FCL update with performance tracking."""
        start_time = time.time()
        
        super().update_fcl(cortical_area, neuron_updates)
        
        # Update performance metrics
        self._performance_metrics['total_updates'] += 1
        update_time = time.time() - start_time
        
        # Running average of update times
        prev_avg = self._performance_metrics['avg_update_time']
        n = self._performance_metrics['total_updates']
        self._performance_metrics['avg_update_time'] = (prev_avg * (n-1) + update_time) / n
        
        # Track peak candidates
        total_candidates = sum(len(candidates) for candidates in self._fcl.candidates.values())
        self._performance_metrics['peak_candidates'] = max(
            self._performance_metrics['peak_candidates'], 
            total_candidates
        )
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """Get performance metrics."""
        return self._performance_metrics.copy()
    
    def get_enhanced_statistics(self) -> Dict[str, Any]:
        """Get enhanced FCL statistics."""
        stats = {
            'total_areas': len(self._fcl.candidates),
            'total_candidates': sum(len(candidates) for candidates in self._fcl.candidates.values()),
            'candidates_per_area': {},
            'excitatory_ratio': {},
            'window_size': self.window_size,
            'history_depth': len(self._history_window)
        }
        
        for area, candidates in self._fcl.candidates.items():
            stats['candidates_per_area'][area] = len(candidates)
            
            if candidates:
                excitatory_count = sum(1 for c in candidates if c.is_excitatory)
                stats['excitatory_ratio'][area] = excitatory_count / len(candidates)
            else:
                stats['excitatory_ratio'][area] = 0.0
        
        return stats


# Compatibility function for tests
def example_enhanced_fcl_usage():
    """Example usage of enhanced FCL manager."""
    manager = EnhancedFCLManager(window_size=5)
    
    # Example updates
    manager.update_fcl('V1', {1: 0.5, 2: -0.3, 3: 0.8})
    manager.update_fcl('V2', {10: 0.2, 11: 0.7})
    
    # Get statistics
    stats = manager.get_enhanced_statistics()
    metrics = manager.get_performance_metrics()
    
    return {
        'manager': manager,
        'stats': stats,
        'metrics': metrics
    }


def inject_neurons_into_fcl(fcl_manager: FCLManager, cortical_area: str, neuron_ids: List[int], delta: float = 1.0):
    """Inject neurons into FCL manager (compatibility function)."""
    if not isinstance(fcl_manager, FCLManager):
        raise FCLError("fcl_manager must be an FCLManager instance")
    
    neuron_updates = {neuron_id: delta for neuron_id in neuron_ids}
    fcl_manager.update_fcl(cortical_area, neuron_updates)


def example_fcl_usage():
    """Example usage of FCL manager."""
    manager = FCLManager(window_size=3)
    
    # Example updates
    manager.update_fcl('motor', {1: 1.0, 2: 0.5})
    manager.update_fcl('visual', {10: 0.8, 11: -0.2})
    
    # Get deltas
    deltas = manager.get_fcl_delta()
    
    # Clear FCL
    manager.clear_fcl()
    
    return {
        'manager': manager,
        'deltas': deltas
    }
