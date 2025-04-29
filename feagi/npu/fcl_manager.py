"""
Fire Candidate List (FCL) Manager Implementation

This module implements the hierarchical Fire Candidate List (FCL) management system,
which tracks neuron activations while preserving their cortical area associations.
The implementation uses Roaring Bitmaps for efficient storage and operations.
"""

from typing import Dict, List, Set, Optional, Union, Tuple, Any
import logging
from collections import defaultdict
try:
    import pyroaring
except ImportError:
    logging.warning("PyRoaring not found. FCL will use slower fallback implementation.")
    pyroaring = None

# Fallback implementation for environments without pyroaring
class FallbackBitMap:
    """Simple set-based fallback for environments without pyroaring."""
    def __init__(self, elements=None):
        self.elements = set(elements) if elements else set()
        
    def add(self, element):
        self.elements.add(element)
        
    def clear(self):
        self.elements.clear()
        
    def copy(self):
        result = FallbackBitMap()
        result.elements = self.elements.copy()
        return result
        
    def __or__(self, other):
        result = self.copy()
        result.elements |= other.elements
        return result
    
    def __and__(self, other):
        result = self.copy()
        result.elements &= other.elements
        return result
    
    def __sub__(self, other):
        result = self.copy()
        result.elements -= other.elements
        return result
    
    def __xor__(self, other):
        result = self.copy()
        result.elements ^= other.elements
        return result
    
    def __len__(self):
        return len(self.elements)
    
    def __iter__(self):
        return iter(self.elements)
    
    def __contains__(self, item):
        return item in self.elements
    
    def is_empty(self):
        return len(self.elements) == 0
    
    def __repr__(self):
        return repr(self.elements)

# Use pyroaring if available, otherwise use fallback
BitMap = pyroaring.BitMap if pyroaring else FallbackBitMap

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
        self.window_size = window_size
        # Main FCL history - stores all neurons regardless of area
        self.global_fcl_history = [BitMap() for _ in range(window_size)]
        # Area-specific FCL history - dictionary mapping area_id -> list of bitmaps
        self.area_fcl_history = defaultdict(lambda: [BitMap() for _ in range(window_size)])
        self.current_window_index = 0
        self.current_timestep = 0
        
        # Stats
        self.total_neurons_fired = 0
        self.neurons_per_area = defaultdict(int)
        
        # Logger
        self.logger = logging.getLogger("feagi.npu.fcl_manager")
        
    def update_fcl(self, current_timestep: int, neurons_by_area: Dict[int, Union[BitMap, List[int], Set[int]]]):
        """
        Update the FCL with neurons firing in the current timestep, preserving area information.
        
        Args:
            current_timestep: Current simulation timestep
            neurons_by_area: Dictionary mapping cortical_area_id -> list/set/bitmap of neuron_ids
        """
        self.current_timestep = current_timestep
        current_index = current_timestep % self.window_size
        
        # Clear the oldest bitmaps for reuse
        self.global_fcl_history[current_index].clear()
        
        # Track firing statistics
        burst_total = 0
        
        # Update area-specific FCLs
        for area_id, neuron_ids in neurons_by_area.items():
            # Create area bitmap if not exists
            if area_id not in self.area_fcl_history:
                self.area_fcl_history[area_id] = [BitMap() for _ in range(self.window_size)]
            
            # Clear the oldest bitmap for this area
            self.area_fcl_history[area_id][current_index].clear()
            
            # Convert to bitmap if not already
            if not isinstance(neuron_ids, BitMap):
                area_bitmap = BitMap(neuron_ids)
            else:
                area_bitmap = neuron_ids
                
            # Count neurons in this area
            area_neuron_count = len(area_bitmap)
            burst_total += area_neuron_count
            self.neurons_per_area[area_id] = area_neuron_count
                
            # Update area-specific bitmap
            self.area_fcl_history[area_id][current_index] |= area_bitmap
            
            # Update global bitmap (union of all areas)
            self.global_fcl_history[current_index] |= area_bitmap
        
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
        """
        if timestep is None:
            index = self.current_window_index
        else:
            index = timestep % self.window_size
        
        return self.global_fcl_history[index].copy()
    
    def get_area_fcl(self, area_id: int, timestep: Optional[int] = None) -> BitMap:
        """
        Get FCL for a specific cortical area at a specific timestep.
        
        Args:
            area_id: ID of the cortical area to query
            timestep: Specific timestep (defaults to current)
            
        Returns:
            BitMap containing firing neuron IDs in the specified area
        """
        if timestep is None:
            index = self.current_window_index
        else:
            index = timestep % self.window_size
            
        if area_id not in self.area_fcl_history:
            return BitMap()
            
        return self.area_fcl_history[area_id][index].copy()
    
    def get_fcl_by_area(self, timestep: Optional[int] = None) -> Dict[int, BitMap]:
        """
        Return a dictionary mapping each cortical area to its firing neurons.
        
        Args:
            timestep: Optional timestep (defaults to current)
            
        Returns:
            Dict[area_id, BitMap] mapping areas to their active neurons
        """
        if timestep is None:
            index = self.current_window_index
        else:
            index = timestep % self.window_size
        
        result = {}
        for area_id, fcl_history in self.area_fcl_history.items():
            if not fcl_history[index].is_empty():
                result[area_id] = fcl_history[index].copy()
                
        return result
    
    def get_active_areas(self, timestep: Optional[int] = None) -> Set[int]:
        """
        Return a set of area_ids that have any firing neurons.
        
        Args:
            timestep: Optional timestep (defaults to current)
            
        Returns:
            Set of active cortical area IDs
        """
        if timestep is None:
            index = self.current_window_index
        else:
            index = timestep % self.window_size
            
        return {area_id for area_id, fcl_history in self.area_fcl_history.items() 
                if not fcl_history[index].is_empty()}
    
    def get_neurons_by_areas(self, area_ids: List[int], timestep: Optional[int] = None) -> BitMap:
        """
        Get neurons firing in any of the specified areas.
        
        Args:
            area_ids: List of cortical area IDs to query
            timestep: Optional timestep (defaults to current)
            
        Returns:
            BitMap of neuron IDs active in any of the specified areas
        """
        if timestep is None:
            index = self.current_window_index
        else:
            index = timestep % self.window_size
            
        result = BitMap()
        for area_id in area_ids:
            if area_id in self.area_fcl_history:
                result |= self.area_fcl_history[area_id][index]
                
        return result
    
    def get_neurons_fired_in_last_n_steps(self, n_steps: int, area_ids: Optional[List[int]] = None) -> BitMap:
        """
        Get neurons that fired in any of the last n timesteps.
        Optionally filter by specific cortical areas.
        
        Args:
            n_steps: Number of timesteps to look back
            area_ids: Optional list of area IDs to filter by
            
        Returns:
            BitMap containing neuron IDs that fired in the specified timespan
        """
        if n_steps > self.window_size:
            n_steps = self.window_size
        
        if area_ids is None:
            # Use global FCL history if no area filtering
            result = BitMap()
            for i in range(n_steps):
                step_index = (self.current_window_index - i) % self.window_size
                result |= self.global_fcl_history[step_index]
        else:
            # Filter by specified areas
            result = BitMap()
            for area_id in area_ids:
                if area_id in self.area_fcl_history:
                    for i in range(n_steps):
                        step_index = (self.current_window_index - i) % self.window_size
                        result |= self.area_fcl_history[area_id][step_index]
        
        return result
    
    def get_consistently_active_neurons(self, n_steps: int, area_ids: Optional[List[int]] = None) -> BitMap:
        """
        Get neurons that fired in ALL of the last n timesteps.
        
        Args:
            n_steps: Number of timesteps to look back
            area_ids: Optional list of area IDs to filter by
            
        Returns:
            BitMap of neuron IDs that fired consistently across the timespan
        """
        if n_steps > self.window_size:
            n_steps = self.window_size
        
        if n_steps <= 0:
            return BitMap()
        
        # Initialize result with neurons from the first relevant timestep
        first_step_index = (self.current_window_index) % self.window_size
        
        if area_ids is None:
            # Start with all neurons from first timestep
            result = self.global_fcl_history[first_step_index].copy()
            
            # Intersect with each subsequent timestep
            for i in range(1, n_steps):
                step_index = (self.current_window_index - i) % self.window_size
                result &= self.global_fcl_history[step_index]
        else:
            # Filter by specified areas
            result = BitMap()
            filtered_neurons = self.get_neurons_by_areas(area_ids, timestep=first_step_index)
            result |= filtered_neurons
            
            # Intersect with each subsequent timestep
            for i in range(1, n_steps):
                step_index = (self.current_window_index - i) % self.window_size
                filtered_step = self.get_neurons_by_areas(area_ids, timestep=step_index)
                result &= filtered_step
        
        return result
    
    def get_fcl_delta(self, start_time: int, end_time: int, area_ids: Optional[List[int]] = None) -> BitMap:
        """
        Get neurons that became active between start and end times.
        
        Args:
            start_time: Starting timestep
            end_time: Ending timestep
            area_ids: Optional list of cortical area IDs to filter by
            
        Returns:
            BitMap of neurons that became active between the two timesteps
        """
        time_diff = abs(end_time - start_time)
        if time_diff > self.window_size:
            raise ValueError(f"Time difference ({time_diff}) exceeds history window size ({self.window_size})")
        
        start_index = start_time % self.window_size
        end_index = end_time % self.window_size
        
        if area_ids is None:
            # Neurons active at end but not at start (newly activated)
            return self.global_fcl_history[end_index] - self.global_fcl_history[start_index]
        else:
            # Filter by specified areas
            start_neurons = self.get_neurons_by_areas(area_ids, timestep=start_time)
            end_neurons = self.get_neurons_by_areas(area_ids, timestep=end_time)
            
            # Neurons active at end but not at start (newly activated)
            return end_neurons - start_neurons
    
    def get_fcl_xor(self, time1: int, time2: int, area_ids: Optional[List[int]] = None) -> BitMap:
        """
        Get neurons that fired at either time1 or time2, but not both.
        Useful for detecting changes in firing patterns.
        
        Args:
            time1: First timestep
            time2: Second timestep
            area_ids: Optional list of cortical area IDs to filter by
            
        Returns:
            BitMap of neurons that changed activation state between timesteps
        """
        idx1 = time1 % self.window_size
        idx2 = time2 % self.window_size
        
        if area_ids is None:
            return self.global_fcl_history[idx1] ^ self.global_fcl_history[idx2]
        else:
            neurons1 = self.get_neurons_by_areas(area_ids, timestep=time1)
            neurons2 = self.get_neurons_by_areas(area_ids, timestep=time2)
            
            return neurons1 ^ neurons2
    
    def count_firing_neurons(self, timestep: Optional[int] = None, area_id: Optional[int] = None) -> int:
        """
        Efficiently count the number of firing neurons.
        
        Args:
            timestep: Optional timestep (defaults to current)
            area_id: Optional area ID to count neurons for a specific area
            
        Returns:
            Count of firing neurons
        """
        if timestep is None and area_id is None:
            # Use cached total for current timestep
            return self.total_neurons_fired
        
        if timestep is None:
            timestep = self.current_timestep
            
        if area_id is None:
            # Count all neurons at the specified timestep
            index = timestep % self.window_size
            return len(self.global_fcl_history[index])
        else:
            # Count neurons in the specified area
            if area_id in self.area_fcl_history:
                index = timestep % self.window_size
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


# Example usage
def example_fcl_usage():
    """Example demonstrating the usage of the HierarchicalFCL class."""
    # Initialize FCL manager
    fcl_manager = HierarchicalFCL(window_size=10)
    
    # Example data for first timestep: area_id -> neurons firing
    firing_neurons_t1 = {
        100: BitMap([1001, 1002, 1005, 1008]),
        200: BitMap([2001, 2010, 2015]),
        300: BitMap([3004, 3007])
    }
    
    # Update FCL for timestep 1
    fcl_manager.update_fcl(1, firing_neurons_t1)
    
    # Example data for second timestep
    firing_neurons_t2 = {
        100: BitMap([1002, 1003, 1009]),
        200: BitMap([2001, 2005]),
        400: BitMap([4001, 4002])
    }
    
    # Update FCL for timestep 2
    fcl_manager.update_fcl(2, firing_neurons_t2)
    
    # Query examples
    
    # 1. Get all firing neurons in current timestep
    all_neurons = fcl_manager.get_global_fcl()
    print(f"All firing neurons: {all_neurons}")
    # Output: {1002, 1003, 1009, 2001, 2005, 4001, 4002}
    
    # 2. Get firing neurons by cortical area
    fcl_by_area = fcl_manager.get_fcl_by_area()
    print(f"FCL by area: {fcl_by_area}")
    # Output: {100: {1002, 1003, 1009}, 200: {2001, 2005}, 400: {4001, 4002}}
    
    # 3. Get all neurons that fired in visual cortex (area 100) in the last 2 steps
    visual_neurons = fcl_manager.get_neurons_fired_in_last_n_steps(2, [100])
    print(f"Visual cortex neurons active in last 2 steps: {visual_neurons}")
    # Output: {1001, 1002, 1003, 1005, 1008, 1009}
    
    # 4. Get active cortical areas in current timestep
    active_areas = fcl_manager.get_active_areas()
    print(f"Active areas: {active_areas}")
    # Output: {100, 200, 400}
    
    # 5. Get firing statistics
    stats = fcl_manager.get_firing_statistics()
    print(f"Firing statistics: {stats}")


if __name__ == "__main__":
    # Set up logging
    logging.basicConfig(level=logging.DEBUG)
    
    # Run example
    example_fcl_usage() 