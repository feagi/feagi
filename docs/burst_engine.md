# Burst Engine Implementation

The Burst Engine is a core component of the Neural Processing Unit (NPU) in FEAGI that handles all activities related to neuron firing, including updating membrane potentials, managing fire candidate lists, and handling cortical stimulation.

## Fire Candidate List (FCL) Queue

The Fire Candidate List contains the neurons that should fire simultaneously across the entire brain. The act of firing all neurons within the FCL is called a "burst" of neuron firing. The FCL Queue maintains a history of recent FCLs across multiple consecutive bursts, enabling temporal pattern analysis and visualization.

### Roaring Bitmaps Technology

For efficient FCL implementation, FEAGI uses **Roaring Bitmaps** as the primary data structure to represent neuron activation patterns. This technology offers several advantages:

1. **Compact Memory Representation:** Neural activations are typically very sparse (0.1-5% active), making traditional dense arrays inefficient. Roaring Bitmaps use a two-level indexing scheme that dynamically selects the optimal internal representation based on data density:
   - Sparse ranges use sorted arrays of 16-bit integers
   - Dense ranges use highly efficient bitmap containers
   - Very dense ranges use run-length encoding

2. **Operation Performance:** Set operations (union, intersection, difference) are computationally intensive but critical for analyzing activation patterns. Roaring Bitmaps implement these operations with container-aware algorithms that can be:
   - Up to 100x faster than traditional bit arrays
   - Up to 5x faster than specialized tree-based structures
   - SIMD-accelerated on supported hardware

3. **Time Window Analysis:** Maintaining a rolling window of activations requires efficient data rotation, clearing, and queries. Roaring Bitmaps support these operations with minimal overhead.

4. **GPU Compatibility:** While our primary implementation uses CPU-based PyRoaring for clarity, there are compatible GPU-based bitmap libraries (RAPIDS cuDF's GPU bitmaps) that follow similar principles for future integration.

### Basic FCL Implementation

A simple FCL implementation using Roaring Bitmaps looks like this:

```python
import numpy as np
import pyroaring  # For CPU implementation
# For GPU, a specialized implementation or wrapper would be needed

# Create a rolling window of fire candidate lists
ACTIVATION_WINDOW_SIZE = 20  # Number of timesteps to maintain in history
fcl_history = [pyroaring.BitMap() for _ in range(ACTIVATION_WINDOW_SIZE)]
current_window_index = 0

def update_fcl_history(current_timestep, fire_candidate_list):
    """Update the rolling window of FCLs with neurons that fired in the current timestep."""
    global current_window_index
    
    # Clear the oldest bitmap and reuse it for current timestep
    current_index = current_timestep % ACTIVATION_WINDOW_SIZE
    fcl_history[current_index].clear()
    
    # Add all neurons in the current FCL to the bitmap
    for neuron_idx in fire_candidate_list:
        fcl_history[current_index].add(neuron_idx)
    
    current_window_index = current_index

def get_neurons_fired_in_last_n_steps(n_steps):
    """
    Get neurons that fired in any of the last n timesteps.
    Returns a Roaring Bitmap containing their indices.
    """
    if n_steps > ACTIVATION_WINDOW_SIZE:
        n_steps = ACTIVATION_WINDOW_SIZE
    
    result = pyroaring.BitMap()
    current_index = current_window_index
    
    for i in range(n_steps):
        step_index = (current_index - i) % ACTIVATION_WINDOW_SIZE
        result |= fcl_history[step_index]  # Fast union operation
    
    return result

def get_consistently_active_neurons(n_steps):
    """
    Get neurons that fired in ALL of the last n timesteps.
    Returns a Roaring Bitmap containing their indices.
    """
    if n_steps > ACTIVATION_WINDOW_SIZE:
        n_steps = ACTIVATION_WINDOW_SIZE
    
    current_index = current_window_index
    step_index = current_index % ACTIVATION_WINDOW_SIZE
    result = fcl_history[step_index].copy()
    
    for i in range(1, n_steps):
        step_index = (current_index - i) % ACTIVATION_WINDOW_SIZE
        result &= fcl_history[step_index]  # Fast intersection operation
    
    return result

def get_fcl_delta(start_time, end_time):
    """
    Get neurons that became active between start and end times.
    Returns a Roaring Bitmap containing their indices.
    """
    if end_time - start_time > ACTIVATION_WINDOW_SIZE:
        raise ValueError("Time difference exceeds history window size")
    
    start_index = start_time % ACTIVATION_WINDOW_SIZE
    end_index = end_time % ACTIVATION_WINDOW_SIZE
    
    # Neurons active at end but not at start (newly activated)
    return fcl_history[end_index] - fcl_history[start_index]  # Fast difference

def get_fcl_xor(time1, time2):
    """
    Get neurons that fired at either time1 or time2, but not both.
    Useful for detecting changes in firing patterns.
    """
    idx1 = time1 % ACTIVATION_WINDOW_SIZE
    idx2 = time2 % ACTIVATION_WINDOW_SIZE
    
    return fcl_history[idx1] ^ fcl_history[idx2]  # Fast XOR operation

def count_firing_neurons(bitmap):
    """
    Efficiently count the number of firing neurons in a bitmap.
    Much faster than iterating through all neurons.
    """
    return len(bitmap)  # Optimized cardinality operation in Roaring
```

## Hierarchical Fire Candidate List Implementation

To efficiently track both neurons and their cortical area associations, FEAGI implements a hierarchical FCL data structure. This approach preserves the area information without requiring expensive lookups during queries.

The hierarchical FCL maintains:
1. A global bitmap of all firing neurons
2. Area-specific bitmaps organized in a dictionary (area_id → bitmap)
3. Temporal history for both global and area-specific activations

This design enables:
- Fast global queries across all neurons
- Efficient area-based filtering without table lookups
- Temporal pattern analysis within specific brain regions

```python
from collections import defaultdict

class HierarchicalFCL:
    def __init__(self, window_size=20):
        """Initialize a hierarchical FCL that tracks neurons with cortical areas."""
        self.window_size = window_size
        # Main FCL history - all neurons regardless of area
        self.global_fcl_history = [pyroaring.BitMap() for _ in range(window_size)]
        # Area-specific history - area_id → list of bitmaps
        self.area_fcl_history = defaultdict(lambda: [pyroaring.BitMap() for _ in range(window_size)])
        self.current_window_index = 0
        
    def update_fcl(self, current_timestep, neurons_by_area):
        """Update FCL with neurons firing in current timestep, preserving area info."""
        current_index = current_timestep % self.window_size
        
        # Clear the oldest bitmaps
        self.global_fcl_history[current_index].clear()
        
        # Update area-specific FCLs
        for area_id, neuron_ids in neurons_by_area.items():
            # Convert to bitmap if needed
            area_bitmap = pyroaring.BitMap(neuron_ids) if not isinstance(neuron_ids, pyroaring.BitMap) else neuron_ids
            
            # Update area-specific bitmap
            if area_id not in self.area_fcl_history:
                self.area_fcl_history[area_id] = [pyroaring.BitMap() for _ in range(self.window_size)]
            self.area_fcl_history[area_id][current_index].clear()
            self.area_fcl_history[area_id][current_index] |= area_bitmap
            
            # Update global bitmap
            self.global_fcl_history[current_index] |= area_bitmap
        
        self.current_window_index = current_index
    
    def get_global_fcl(self, timestep=None):
        """Get complete FCL for a specific timestep."""
        index = self.current_window_index if timestep is None else timestep % self.window_size
        return self.global_fcl_history[index].copy()
    
    def get_area_fcl(self, area_id, timestep=None):
        """Get FCL for a specific cortical area."""
        index = self.current_window_index if timestep is None else timestep % self.window_size
        if area_id not in self.area_fcl_history:
            return pyroaring.BitMap()
        return self.area_fcl_history[area_id][index].copy()
    
    def get_fcl_by_area(self, timestep=None):
        """Return dictionary mapping each cortical area to its firing neurons."""
        index = self.current_window_index if timestep is None else timestep % self.window_size
        result = {}
        for area_id, fcl_history in self.area_fcl_history.items():
            if not fcl_history[index].is_empty():
                result[area_id] = fcl_history[index].copy()
        return result
    
    def get_neurons_fired_in_last_n_steps(self, n_steps, area_ids=None):
        """Get neurons that fired in any of the last n timesteps, optionally filtered by area."""
        if n_steps > self.window_size:
            n_steps = self.window_size
        
        # Use global or area-filtered approach
        if area_ids is None:
            result = pyroaring.BitMap()
            for i in range(n_steps):
                step_index = (self.current_window_index - i) % self.window_size
                result |= self.global_fcl_history[step_index]
        else:
            result = pyroaring.BitMap()
            for area_id in area_ids:
                if area_id in self.area_fcl_history:
                    for i in range(n_steps):
                        step_index = (self.current_window_index - i) % self.window_size
                        result |= self.area_fcl_history[area_id][step_index]
        
        return result
        
    def get_consistently_active_neurons(self, n_steps, area_ids=None):
        """Get neurons that fired in ALL of the last n timesteps."""
        if n_steps > self.window_size:
            n_steps = self.window_size
        
        if n_steps <= 0:
            return pyroaring.BitMap()
        
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
            result = pyroaring.BitMap()
            filtered_neurons = self.get_neurons_by_areas(area_ids, timestep=first_step_index)
            result |= filtered_neurons
            
            # Intersect with each subsequent timestep
            for i in range(1, n_steps):
                step_index = (self.current_window_index - i) % self.window_size
                filtered_step = self.get_neurons_by_areas(area_ids, timestep=step_index)
                result &= filtered_step
        
        return result
```

### Benefits of the Hierarchical FCL

1. **Optimized Queries:** Both global and area-specific queries benefit from the bitmap's efficient operations:
   - Area-filtered temporal analyses (find neurons in area X that fired in last N steps)
   - Cross-area pattern detection (areas with synchronous firing)

2. **Efficient Memory Use:** Only stores bitmaps for active cortical areas, avoiding allocation for silent brain regions

3. **Cortical Analysis:** Enables investigation of information flow between brain regions by monitoring activation patterns across specific area sets

4. **Visualization Support:** Provides direct mapping of activities to brain regions for more meaningful visualizations

The original bitmap-based FCL implementation still shows its advantages for global operations, while this hierarchical extension adds area-based capabilities with minimal computational overhead.

## FCL Operational Details

The Fire Candidate List is a core mechanism within the Burst Engine that manages neuron firing activities. Understanding its operational aspects is essential for efficient neural simulation. This section details how neurons are added to, read from, and removed from the FCL, as well as how the rolling window mechanism works.

### FCL Rolling Window Mechanism

The FCL maintains a history of neuron firing patterns across multiple timesteps using a rolling window mechanism:

```python
# Initialize FCL with a specified window size
window_size = 20  # Keep track of 20 timesteps
fcl_manager = HierarchicalFCL(window_size=window_size)
```

The rolling window mechanism works as follows:

1. **Circular Buffer Structure**: The FCL uses a circular buffer (fixed-size array) to store firing data for each timestep
2. **Index Calculation**: Current timestep is mapped to array index: `index = timestep % window_size`
3. **Reuse of Memory**: When the buffer is full, the oldest timestep data is overwritten with new data
4. **Configurable History**: The window size can be adjusted to balance memory usage and temporal analysis needs

This approach efficiently maintains a fixed-size history that follows the simulation, allowing temporal pattern analysis without unbounded memory growth.

### Adding Neurons to FCL

Neurons are added to the FCL when they reach their firing threshold. The process is handled through the `update_fcl` method:

```python
# Example: Adding neurons to FCL at timestep 42
neurons_by_area = {
    100: [1001, 1002, 1005],  # 3 neurons firing in area 100
    200: [2001, 2010]         # 2 neurons firing in area 200
}
fcl_manager.update_fcl(current_timestep=42, neurons_by_area=neurons_by_area)
```

Key aspects of this operation:

1. **Automatic Clearing**: The oldest bitmap at the current timestep's position is automatically cleared before reuse
2. **Area Association Preservation**: Each neuron's cortical area affiliation is preserved in the hierarchical structure
3. **Efficient Updates**: Using bitmaps enables constant-time addition operations regardless of the number of neurons
4. **Batched Processing**: All neurons firing in the same timestep are added in a single batch operation
5. **Multiple Input Formats**: FCL accepts various input collection types (lists, sets, or existing bitmaps)

```python
# Different ways to specify firing neurons
fcl_manager.update_fcl(43, {
    100: BitMap([1001, 1003, 1009]),  # Using an existing BitMap
    200: [2001, 2005],                # Using a regular Python list
    300: {3001, 3002, 3005}           # Using a Python set
})
```

### Reading from FCL

The FCL provides multiple ways to read and query neuron firing data, optimized for different use cases:

#### Global FCL Access

To access all firing neurons at a specific timestep:

```python
# Get all firing neurons at the current timestep
all_current_neurons = fcl_manager.get_global_fcl()

# Get all firing neurons at a specific timestep
past_neurons = fcl_manager.get_global_fcl(timestep=40)
```

#### Area-Specific FCL Access

To access neurons firing in a specific cortical area:

```python
# Get neurons firing in visual cortex (area 100) at current timestep
visual_neurons = fcl_manager.get_area_fcl(area_id=100)

# Get neurons firing in motor cortex (area 200) at a specific timestep
motor_neurons = fcl_manager.get_area_fcl(area_id=200, timestep=41)
```

#### Temporal Pattern Queries

To analyze firing patterns across multiple timesteps:

```python
# Get all neurons that fired at any point in the last 5 timesteps
recently_active = fcl_manager.get_neurons_fired_in_last_n_steps(n_steps=5)

# Get neurons in area 300 that fired in the last 3 timesteps
area_recent = fcl_manager.get_neurons_fired_in_last_n_steps(n_steps=3, area_ids=[300])

# Get neurons that fired consistently across all of the last 4 timesteps
consistent_neurons = fcl_manager.get_consistently_active_neurons(n_steps=4)

# Get neurons that fired at timestep 40 but not at timestep 39 (newly activated)
new_neurons = fcl_manager.get_fcl_delta(start_time=39, end_time=40)

# Get neurons that changed state (activated or deactivated) between timesteps 40 and 41
changed_neurons = fcl_manager.get_fcl_xor(time1=40, time2=41)
```

#### Statistical Queries

To obtain statistics about firing activity:

```python
# Count all firing neurons at current timestep
neuron_count = fcl_manager.count_firing_neurons()

# Count neurons firing in area 100 at timestep 42
area_count = fcl_manager.count_firing_neurons(timestep=42, area_id=100)

# Get comprehensive firing statistics
stats = fcl_manager.get_firing_statistics()
print(f"Active areas: {stats['active_areas']}")
print(f"Total neurons fired: {stats['total_neurons_fired']}")
print(f"Neurons per area: {stats['neurons_per_area']}")
```

All read operations return copies of the internal bitmaps to ensure thread safety and prevent unintended modifications.

### Removing Neurons from FCL

Neurons are never explicitly removed from the FCL. Instead, the system relies on automatic clearing of the oldest bitmaps when they fall outside the rolling window:

1. **Implicit Removal**: As new timesteps are processed, neurons from the oldest timesteps automatically fall out of the window
2. **Bitmap Clearing**: When a bitmap position is reused, it's automatically cleared first: `bitmap.clear()`
3. **Memory Management**: This approach ensures constant memory usage regardless of simulation duration

Example of how the rolling window handles neuron removal:

```python
# Initialize FCL with window_size=3
fcl = HierarchicalFCL(window_size=3)

# Add neurons for timesteps 1, 2, 3
fcl.update_fcl(1, {100: [1, 2, 3]})     # Bitmap at index 1%3=1 contains [1,2,3]
fcl.update_fcl(2, {100: [4, 5]})        # Bitmap at index 2%3=2 contains [4,5]
fcl.update_fcl(3, {100: [6, 7, 8]})     # Bitmap at index 3%3=0 contains [6,7,8]

# FCL now contains data for timesteps 1, 2, 3

# Add neurons for timestep 4
fcl.update_fcl(4, {100: [9, 10]})       # Bitmap at index 4%3=1 is cleared and now contains [9,10]
                                        # Neurons [1,2,3] are implicitly removed

# FCL now contains data for timesteps 2, 3, 4 (timestep 1 data is gone)
```

### Error Handling and Edge Cases

The FCL implementation includes robust error handling for various edge cases:

1. **Out-of-Range Timesteps**: When requesting data for a timestep outside the valid window:
   ```python
   try:
       # Assume current timestep is 100 and window_size is 20
       # Valid range is [81-100]
       old_data = fcl_manager.get_global_fcl(timestep=70)  # Out of range
   except TimestepOutOfRangeError as e:
       print(f"Error: {e}")  # "Timestep 70 is outside valid range [81, 100]"
   ```

2. **Empty Areas**: Requesting data for inactive cortical areas returns empty bitmaps:
   ```python
   empty_area_fcl = fcl_manager.get_area_fcl(area_id=999)  # Returns empty bitmap
   ```

3. **Negative Steps**: Requesting temporal patterns with negative steps raises an error:
   ```python
   try:
       fcl_manager.get_neurons_fired_in_last_n_steps(n_steps=-1)
   except ValueError as e:
       print(f"Error: {e}")  # "n_steps must be positive, got -1"
   ```

### Performance Considerations

The FCL implementation is designed for high-performance neural simulation:

1. **Time Complexity**:
   - Adding neurons: O(1) per neuron with Roaring Bitmaps
   - Querying global FCL: O(1)
   - Area-specific queries: O(1)
   - Temporal pattern queries: O(n) where n is the number of timesteps

2. **Space Complexity**:
   - O(a × t × d) where:
     - a = number of active areas
     - t = window size
     - d = average density of firing (typically very sparse)

3. **Optimization Techniques**:
   - Lazy initialization of area bitmaps
   - Copy-on-read to ensure thread safety
   - Bitmap reuse to minimize memory allocation
   - Efficient set operations (union, intersection, difference)

The FCL's rolling window mechanism ensures that memory usage remains constant regardless of simulation duration, making it suitable for long-running brain simulations.

### Area-Specific History Windows

For memory-type cortical areas, maintaining custom history window sizes is often required to capture specialized temporal patterns or long-term memory traces. The FCL implementation can be extended to support area-specific window sizes:

```python
class EnhancedHierarchicalFCL:
    def __init__(self, default_window_size: int = 20):
        """
        Initialize an enhanced hierarchical FCL with support for area-specific window sizes.
        
        Args:
            default_window_size: Default number of timesteps to maintain for standard areas
        """
        self.default_window_size = default_window_size
        self.global_fcl_history = [BitMap() for _ in range(default_window_size)]
        
        # Main FCL history - stores all neurons regardless of area
        self.current_window_index = 0
        self.current_timestep = 0
        
        # Standard area storage using default window size
        self.area_fcl_history = {}
        
        # Storage for areas with custom window sizes
        # Maps area_id -> (window_size, [BitMaps], start_timestep)
        self.custom_area_history = {}
        
        # Track area types for quick lookup
        self.memory_area_ids = set()
        
        # Stats
        self.total_neurons_fired = 0
        self.neurons_per_area = {}
        
    def register_memory_area(self, area_id: int, window_size: int):
        """
        Register a memory-type cortical area with a custom window size.
        
        Args:
            area_id: ID of the memory-type cortical area
            window_size: Custom history window size for this area (must be >= default_window_size)
        """
        if window_size < self.default_window_size:
            raise ValueError(f"Custom window size ({window_size}) must be >= default window size ({self.default_window_size})")
            
        # Initialize with empty bitmaps
        history_array = [BitMap() for _ in range(window_size)]
        
        # Store area with custom settings
        # The third element is the start timestep, initialized to current timestep
        self.custom_area_history[area_id] = (window_size, history_array, self.current_timestep)
        self.memory_area_ids.add(area_id)
        
    def is_memory_area(self, area_id: int) -> bool:
        """Check if an area is registered as a memory-type area with custom window size."""
        return area_id in self.memory_area_ids
        
    def get_area_window_size(self, area_id: int) -> int:
        """Get the window size for a specific area."""
        if area_id in self.custom_area_history:
            return self.custom_area_history[area_id][0]
        return self.default_window_size
        
    def _get_custom_area_index(self, area_id: int, timestep: int) -> int:
        """
        Calculate the correct index in the custom window size history for a given area and timestep.
        
        Args:
            area_id: ID of the memory-type cortical area
            timestep: Specific timestep to get index for
            
        Returns:
            Index in the area's custom-sized history array
            
        Raises:
            TimestepOutOfRangeError: If timestep is outside valid range for this area
        """
        if area_id not in self.custom_area_history:
            raise ValueError(f"Area {area_id} is not registered as a memory area")
            
        window_size, history_array, start_timestep = self.custom_area_history[area_id]
        
        # Check if timestep is within valid range for this area
        oldest_time = max(start_timestep, self.current_timestep - window_size + 1)
        if timestep < oldest_time or timestep > self.current_timestep:
            raise TimestepOutOfRangeError(
                f"Timestep {timestep} is outside valid range [{oldest_time}, {self.current_timestep}] for area {area_id}"
            )
            
        # Convert relative to start_timestep, then modulo window_size
        relative_timestep = timestep - start_timestep
        return relative_timestep % window_size
    
    def update_fcl(self, current_timestep: int, neurons_by_area: Dict[int, Union[BitMap, List[int], Set[int]]]):
        """
        Update the FCL with neurons firing in the current timestep, with support for memory areas.
        
        Args:
            current_timestep: Current simulation timestep
            neurons_by_area: Dictionary mapping cortical_area_id -> list/set/bitmap of neuron_ids
        """
        self.current_timestep = current_timestep
        standard_index = current_timestep % self.default_window_size
        
        # Clear the oldest global bitmap for reuse
        self.global_fcl_history[standard_index].clear()
        
        # Track firing statistics
        burst_total = 0
        
        # Process each area
        for area_id, neuron_ids in neurons_by_area.items():
            # Convert to bitmap if needed
            neuron_collection = NeuronCollection.from_any(neuron_ids)
            area_bitmap = neuron_collection.to_bitmap()
            
            # Count neurons in this area
            area_neuron_count = len(area_bitmap)
            burst_total += area_neuron_count
            self.neurons_per_area[area_id] = area_neuron_count
            
            # Check if this is a memory area with custom window size
            if self.is_memory_area(area_id):
                window_size, history_array, start_timestep = self.custom_area_history[area_id]
                custom_index = self._get_custom_area_index(area_id, current_timestep)
                
                # Clear the bitmap at the current position
                history_array[custom_index].clear()
                # Update with new neurons
                history_array[custom_index] = area_bitmap
            else:
                # Standard area processing
                if area_id not in self.area_fcl_history:
                    self.area_fcl_history[area_id] = [BitMap() for _ in range(self.default_window_size)]
                    
                # Clear and update the standard area bitmap
                self.area_fcl_history[area_id][standard_index].clear()
                self.area_fcl_history[area_id][standard_index] = area_bitmap
            
            # Always update the global FCL (for all areas)
            self.global_fcl_history[standard_index] = self.global_fcl_history[standard_index] | area_bitmap
        
        # Update total count and window index
        self.total_neurons_fired = burst_total
        self.current_window_index = standard_index
    
    def get_area_fcl(self, area_id: int, timestep: Optional[int] = None) -> BitMap:
        """
        Get FCL for a specific cortical area, handling both standard and memory areas.
        
        Args:
            area_id: ID of the cortical area to query
            timestep: Specific timestep (defaults to current)
            
        Returns:
            BitMap containing firing neuron IDs in the specified area
        """
        if timestep is None:
            timestep = self.current_timestep
            
        # Check if this is a memory area with custom window size
        if self.is_memory_area(area_id):
            custom_index = self._get_custom_area_index(area_id, timestep)
            window_size, history_array, start_timestep = self.custom_area_history[area_id]
            return history_array[custom_index].copy()
        else:
            # Use standard area processing
            standard_index = timestep % self.default_window_size
            
            if area_id not in self.area_fcl_history:
                return BitMap()
                
            return self.area_fcl_history[area_id][standard_index].copy()
    
    def get_area_temporal_pattern(self, area_id: int, n_steps: int) -> BitMap:
        """
        Get neurons in a memory area that fired in the last n timesteps.
        Optimized for memory areas with longer history windows.
        
        Args:
            area_id: ID of the memory-type cortical area 
            n_steps: Number of timesteps to look back
            
        Returns:
            BitMap of neuron IDs that fired in the specified timespan
        """
        if not self.is_memory_area(area_id):
            raise ValueError(f"Area {area_id} is not registered as a memory area")
            
        window_size, history_array, start_timestep = self.custom_area_history[area_id]
        
        if n_steps <= 0:
            raise ValueError(f"n_steps must be positive, got {n_steps}")
            
        if n_steps > window_size:
            n_steps = window_size
        
        result = BitMap()
        for i in range(n_steps):
            if self.current_timestep - i < start_timestep:
                break  # Don't go before the start timestep
                
            timestep = self.current_timestep - i
            custom_index = self._get_custom_area_index(area_id, timestep)
            result = result | history_array[custom_index]
            
        return result
    
    def get_memory_area_consistency(self, area_id: int, pattern_duration: int, window_duration: int) -> float:
        """
        Calculate how consistently a pattern has been maintained in a memory area.
        
        Args:
            area_id: ID of the memory-type cortical area
            pattern_duration: Duration of the pattern to analyze (in timesteps)
            window_duration: Total window to evaluate (must be >= pattern_duration)
            
        Returns:
            Consistency score between 0.0 (no consistency) and 1.0 (perfect consistency)
        """
        if not self.is_memory_area(area_id):
            raise ValueError(f"Area {area_id} is not registered as a memory area")
            
        if window_duration < pattern_duration:
            raise ValueError(f"Window duration ({window_duration}) must be >= pattern duration ({pattern_duration})")
            
        window_size, history_array, start_timestep = self.custom_area_history[area_id]
        
        if pattern_duration > window_size:
            pattern_duration = window_size
            
        if window_duration > window_size:
            window_duration = window_size
        
        # Get the pattern to check for consistency
        pattern = self.get_area_temporal_pattern(area_id, pattern_duration)
        
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
                    
                idx = self._get_custom_area_index(area_id, timestep)
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
```

#### Implementation Details

This enhanced FCL implementation adds several key features for memory-type cortical areas:

1. **Area Registration**: Memory areas are explicitly registered with their custom window sizes
   ```python
   # Register hippocampus with a 1000-timestep history window
   fcl_manager.register_memory_area(area_id=500, window_size=1000)
   
   # Register working memory with a 100-timestep history
   fcl_manager.register_memory_area(area_id=501, window_size=100)
   ```

2. **Separate Storage Structures**:
   - Standard areas use the original circular buffer approach
   - Memory areas use dedicated storage with larger window sizes
   - Each memory area maintains its starting timestep for proper indexing

3. **Area-Specific Indexing**: Custom index calculation for memory areas considers:
   - Area-specific window size
   - Starting timestep (when the area was registered)
   - Current timestep

4. **Memory-Specific Analysis**: Special functions for temporal pattern analysis in memory areas:
   - `get_area_temporal_pattern`: Optimized for long-term pattern recognition
   - `get_memory_area_consistency`: Measures pattern stability over time

5. **Compatibility**: Still maintains the standard global FCL for all neurons, regardless of area type

#### Usage Example

```python
# Initialize with default 20-timestep window for regular areas
fcl_manager = EnhancedHierarchicalFCL(default_window_size=20)

# Register memory areas with custom window sizes
fcl_manager.register_memory_area(area_id=500, window_size=1000)  # Episodic memory
fcl_manager.register_memory_area(area_id=501, window_size=200)   # Working memory

# Normal update process works with both standard and memory areas
neurons_by_area = {
    100: [1001, 1002],  # Standard area (visual cortex)
    200: [2001, 2005],  # Standard area (motor cortex)
    500: [5001, 5002]   # Memory area with extended history
}
fcl_manager.update_fcl(current_timestep=42, neurons_by_area=neurons_by_area)

# Query standard areas normally
visual_neurons = fcl_manager.get_area_fcl(area_id=100)

# Query memory areas with extended history capabilities
memory_pattern = fcl_manager.get_area_temporal_pattern(area_id=500, n_steps=500)

# Check pattern consistency in memory area
consistency = fcl_manager.get_memory_area_consistency(
    area_id=500,
    pattern_duration=10,   # 10-timestep pattern
    window_duration=100    # Check across 100 timesteps
)
print(f"Memory pattern consistency: {consistency * 100:.1f}%")
```

#### Benefits for Memory Areas

This approach offers several advantages for memory-type cortical areas:

1. **Extended Temporal Memory**: Memory areas can maintain firing histories for thousands of timesteps
2. **Memory Efficiency**: Only allocates larger histories for areas that need them
3. **Pattern Recognition**: Specialized functions for analyzing temporal patterns
4. **Stability Metrics**: Tools to measure how consistently patterns are maintained
5. **Integration**: Works seamlessly with standard areas in the same simulation

This enhanced FCL implementation enables memory-type cortical areas to fulfill their specialized roles in maintaining and analyzing temporal patterns over extended periods while maintaining the efficiency of the standard FCL for other areas.

### Example: FCL Data Across Time Steps

To better understand how the enhanced FCL with area-specific window sizes works, let's walk through a concrete example showing the state of the FCL data across 3 time steps with custom window sizes for memory areas.

In this example, we'll have:
- **Standard areas**: Visual cortex (area 100) and Motor cortex (area 200)
- **Memory areas**: 
  - Hippocampus (area 500) with window_size=100
  - Working memory (area 501) with window_size=50

Let's set up our FCL manager:

```python
# Initialize with a default window size of 5 for standard areas
fcl = EnhancedHierarchicalFCL(default_window_size=5)

# Register our memory areas with custom window sizes
fcl.register_memory_area(area_id=500, window_size=100)  # Hippocampus
fcl.register_memory_area(area_id=501, window_size=50)   # Working memory
```

#### Time Step 1 (t=1)

Let's add some firing neurons for time step 1:

```python
neurons_t1 = {
    100: [101, 102, 103],        # Visual cortex neurons
    200: [201, 202],             # Motor cortex neurons
    500: [501, 502, 503],        # Hippocampus neurons
    501: [601, 602]              # Working memory neurons
}
fcl.update_fcl(current_timestep=1, neurons_by_area=neurons_t1)
```

After this update, the FCL data would look like:

```
Global FCL History (default_window_size=5):
┌───┬───┬───┬───┬───┐
│ 0 │ 1 │ 2 │ 3 │ 4 │
└───┴───┴───┴───┴───┘
      ▲
      │
      └── Contains: [101, 102, 103, 201, 202, 501, 502, 503, 601, 602]
          (All neurons firing at t=1)

Standard Areas (default_window_size=5):
Area 100 (Visual Cortex):
┌───┬───┬───┬───┬───┐
│ 0 │ 1 │ 2 │ 3 │ 4 │
└───┴───┴───┴───┴───┘
      ▲
      │
      └── Contains: [101, 102, 103]

Area 200 (Motor Cortex):
┌───┬───┬───┬───┬───┐
│ 0 │ 1 │ 2 │ 3 │ 4 │
└───┴───┴───┴───┴───┘
      ▲
      │
      └── Contains: [201, 202]

Memory Areas:
Area 500 (Hippocampus, window_size=100):
┌───┬───┬───┬───┬───┬─────┬─────┐
│ 0 │ 1 │ 2 │...│ 98│ 99  │     │
└───┴───┴───┴───┴───┴─────┴─────┘
      ▲
      │
      └── Contains: [501, 502, 503]

Area 501 (Working Memory, window_size=50):
┌───┬───┬───┬───┬───┬─────┬─────┐
│ 0 │ 1 │ 2 │...│ 48│ 49  │     │
└───┴───┴───┴───┴───┴─────┴─────┘
      ▲
      │
      └── Contains: [601, 602]
```

#### Time Step 2 (t=2)

Now let's update with firing data for time step 2:

```python
neurons_t2 = {
    100: [101, 104, 105],        # Visual cortex neurons
    200: [202, 203],             # Motor cortex neurons
    500: [501, 504, 505],        # Hippocampus neurons
    501: [601, 603]              # Working memory neurons
}
fcl.update_fcl(current_timestep=2, neurons_by_area=neurons_t2)
```

After this update, the FCL data would look like:

```
Global FCL History (default_window_size=5):
┌───┬───┬───┬───┬───┐
│ 0 │ 1 │ 2 │ 3 │ 4 │
└───┴───┴───┴───┴───┘
      │   ▲
      │   │
      │   └── Contains: [101, 104, 105, 202, 203, 501, 504, 505, 601, 603]
      │       (All neurons firing at t=2)
      │
      └── Still contains: [101, 102, 103, 201, 202, 501, 502, 503, 601, 602]
          (All neurons firing at t=1)

Standard Areas (default_window_size=5):
Area 100 (Visual Cortex):
┌───┬───┬───┬───┬───┐
│ 0 │ 1 │ 2 │ 3 │ 4 │
└───┴───┴───┴───┴───┘
      │   ▲
      │   │
      │   └── Contains: [101, 104, 105]
      │
      └── Still contains: [101, 102, 103]

Area 200 (Motor Cortex):
┌───┬───┬───┬───┬───┐
│ 0 │ 1 │ 2 │ 3 │ 4 │
└───┴───┴───┴───┴───┘
      │   ▲
      │   │
      │   └── Contains: [202, 203]
      │
      └── Still contains: [201, 202]

Memory Areas:
Area 500 (Hippocampus, window_size=100):
┌───┬───┬───┬───┬───┬─────┬─────┐
│ 0 │ 1 │ 2 │...│ 98│ 99  │     │
└───┴───┴───┴───┴───┴─────┴─────┘
      │   ▲
      │   │
      │   └── Contains: [501, 504, 505]
      │
      └── Still contains: [501, 502, 503]

Area 501 (Working Memory, window_size=50):
┌───┬───┬───┬───┬───┬─────┬─────┐
│ 0 │ 1 │ 2 │...│ 48│ 49  │     │
└───┴───┴───┴───┴───┴─────┴─────┘
      │   ▲
      │   │
      │   └── Contains: [601, 603]
      │
      └── Still contains: [601, 602]
```

#### Time Step 3 (t=3)

Let's update with firing data for time step 3:

```python
neurons_t3 = {
    100: [102, 105, 106],        # Visual cortex neurons
    200: [203, 204],             # Motor cortex neurons
    500: [501, 505, 506],        # Hippocampus neurons
    501: [602, 603]              # Working memory neurons
}
fcl.update_fcl(current_timestep=3, neurons_by_area=neurons_t3)
```

After this update, the FCL data would look like:

```
Global FCL History (default_window_size=5):
┌───┬───┬───┬───┬───┐
│ 0 │ 1 │ 2 │ 3 │ 4 │
└───┴───┴───┴───┴───┘
      │   │   ▲
      │   │   │
      │   │   └── Contains: [102, 105, 106, 203, 204, 501, 505, 506, 602, 603]
      │   │       (All neurons firing at t=3)
      │   │
      │   └── Still contains: [101, 104, 105, 202, 203, 501, 504, 505, 601, 603]
      │       (All neurons firing at t=2)
      │
      └── Still contains: [101, 102, 103, 201, 202, 501, 502, 503, 601, 602]
          (All neurons firing at t=1)

Standard Areas (default_window_size=5):
Area 100 (Visual Cortex):
┌───┬───┬───┬───┬───┐
│ 0 │ 1 │ 2 │ 3 │ 4 │
└───┴───┴───┴───┴───┘
      │   │   ▲
      │   │   │
      │   │   └── Contains: [102, 105, 106]
      │   │
      │   └── Still contains: [101, 104, 105]
      │
      └── Still contains: [101, 102, 103]

Area 200 (Motor Cortex):
┌───┬───┬───┬───┬───┐
│ 0 │ 1 │ 2 │ 3 │ 4 │
└───┴───┴───┴───┴───┘
      │   │   ▲
      │   │   │
      │   │   └── Contains: [203, 204]
      │   │
      │   └── Still contains: [202, 203]
      │
      └── Still contains: [201, 202]

Memory Areas:
Area 500 (Hippocampus, window_size=100):
┌───┬───┬───┬───┬───┬─────┬─────┐
│ 0 │ 1 │ 2 │ 3 │...│ 98  │ 99  │
└───┴───┴───┴───┴───┴─────┴─────┘
      │   │   ▲
      │   │   │
      │   │   └── Contains: [501, 505, 506]
      │   │
      │   └── Still contains: [501, 504, 505]
      │
      └── Still contains: [501, 502, 503]

Area 501 (Working Memory, window_size=50):
┌───┬───┬───┬───┬───┬─────┬─────┐
│ 0 │ 1 │ 2 │ 3 │...│ 48  │ 49  │
└───┴───┴───┴───┴───┴─────┴─────┘
      │   │   ▲
      │   │   │
      │   │   └── Contains: [602, 603]
      │   │
      │   └── Still contains: [601, 603]
      │
      └── Still contains: [601, 602]
```

#### Example Analysis

Now let's see how we can perform different analyses on this data:

1. **Get all neurons that fired in the visual cortex at time step 2**:
   ```python
   visual_t2 = fcl.get_area_fcl(area_id=100, timestep=2)
   # Result: [101, 104, 105]
   ```

2. **Get all neurons that fired in the hippocampus during the past 3 time steps**:
   ```python
   hippocampus_recent = fcl.get_area_temporal_pattern(area_id=500, n_steps=3)
   # Result: [501, 502, 503, 504, 505, 506]
   ```

3. **Find neurons that were consistently active in working memory for all 3 time steps**:
   ```python
   # First get all active in the last 3 steps
   all_active = fcl.get_area_temporal_pattern(area_id=501, n_steps=3)
   # Result: [601, 602, 603]
   
   # Then get those active at t=1
   active_t1 = fcl.get_area_fcl(area_id=501, timestep=1)
   # Result: [601, 602]
   
   # And those active at t=2
   active_t2 = fcl.get_area_fcl(area_id=501, timestep=2)
   # Result: [601, 603]
   
   # And those active at t=3
   active_t3 = fcl.get_area_fcl(area_id=501, timestep=3)
   # Result: [602, 603]
   
   # Neurons active in all 3 steps:
   # Intersection: active_t1 ∩ active_t2 ∩ active_t3
   # Result: None (no neuron was active in all 3)
   ```

4. **Find neurons in hippocampus that fired at least once in all time steps**:
   ```python
   # First get neurons active at t=1
   active_t1 = fcl.get_area_fcl(area_id=500, timestep=1)
   # Result: [501, 502, 503]
   
   # Then neurons active at t=2
   active_t2 = fcl.get_area_fcl(area_id=500, timestep=2)
   # Result: [501, 504, 505]
   
   # Then neurons active at t=3
   active_t3 = fcl.get_area_fcl(area_id=500, timestep=3)
   # Result: [501, 505, 506]
   
   # Common to all: active_t1 ∩ active_t2 ∩ active_t3
   # Result: [501] (only neuron 501 was active in all 3 time steps)
   ```

#### Key Observations

This example illustrates several important aspects of the enhanced FCL implementation:

1. **Memory Preservation**: The memory-type cortical areas (500, 501) can maintain much longer histories (50-100 timesteps) compared to standard areas (5 timesteps).

2. **Common Global FCL**: Despite having different window sizes, all areas contribute to the same global FCL for the current timestep.

3. **Area-Specific Patterns**: The implementation allows detection of patterns specific to certain brain areas, like finding consistent activation in hippocampus.

4. **Temporal Analysis**: We can perform both point-in-time queries and temporal pattern analysis across multiple time steps.

5. **Individual Neuron Tracing**: The system enables tracking individual neurons across time (e.g., neuron 501 in hippocampus remained active across all time steps).

As the simulation continues beyond time step 5, the standard areas would start overwriting their oldest data, while memory areas would preserve their histories for much longer periods, enabling analysis of extended temporal patterns crucial for memory function.

## FCL Manager

At the heart of the burst engine lies the FCL manager that is responsible for:
- Reading FCL content
- Initiating the firing of neurons
- Providing a copy of the current FCL content for transmission to the peripheral nervous system (PNS) and data visualization

The FCL Manager interfaces with the Hierarchical FCL implementation to manage the flow of neuron activations throughout the simulation.

## FCL Sampler

The FCL Sampler is responsible for:
- Invoking the FCL manager at a configurable frequency
- Generating motor commands based on neural activity
- Providing data for brain visualization
- Controlling sampling rate to balance performance and visual fidelity

### Visualization Sampling

To maintain visualization performance for high-frequency simulations, the FCL Sampler supports two sampling modes:

- **Ratio-based sampling** (0.0-1.0): Randomly selects a percentage of FCLs to visualize
  - Example: 0.1 means visualize ~10% of FCLs
  
- **Frequency-based sampling** (>1.0): Caps visualization at a maximum frequency (Hz)
  - Example: 30 means visualize at most 30 FCLs per second

## Neuron Firing Dynamics

### Neuron Models

FEAGI supports different neuron models that define the behavioral dynamics of neurons:

1. **Leaky Integrate-and-Fire (LIF)** - The primary model historically used in FEAGI, featuring:
   - Membrane potential that decays over time
   - Firing threshold
   - Refractory period
   - Simple but computationally efficient

2. **Custom Models** - FEAGI 2.0 is designed to enable supporting multiple neuron models, including:
   - Izhikevich model elements for more biologically realistic behavior
   - Customizable dynamics parameters
   - Model-specific properties 

## WebGPU Implementation

As part of the NPU's GPU acceleration strategy, the Burst Engine can leverage WebGPU for cross-platform GPU acceleration. Below is an example of a WebGPU compute shader for neuron dynamics:

```wgsl
@group(0) @binding(0) var<storage, read> area_is_active: array<u32>;
@group(0) @binding(1) var<storage, read> area_ids: array<u32>;
@group(0) @binding(2) var<storage, read_write> membrane_potentials: array<f32>;
@group(0) @binding(3) var<storage, read> resting_potentials: array<f32>;
@group(0) @binding(4) var<storage, read> membrane_time_constants: array<f32>;
@group(0) @binding(5) var<storage, read> firing_thresholds: array<f32>;
@group(0) @binding(6) var<storage, read_write> fcl_buffer: array<u32>;
@group(0) @binding(7) var<storage, read_write> refractory_timers: array<u32>;
@group(0) @binding(8) var<storage, read> refractory_periods: array<u32>;
@group(0) @binding(9) var<storage, read> active_neuron_indices: array<u32>;
@group(0) @binding(10) var<uniform> active_neuron_count: u32;

@compute @workgroup_size(256)
fn update_neurons(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let thread_id = global_id.x;
    if (thread_id >= active_neuron_count) {
        return;
    }
    
    let neuron_idx = active_neuron_indices[thread_id];
    
    // Check if parent area is active
    let area_id = area_ids[neuron_idx];
    if (area_is_active[area_id] == 0) {
        return;
    }
    
    // Refractory period check
    if (refractory_timers[neuron_idx] > 0) {
        refractory_timers[neuron_idx] -= 1;
        return;
    }
    
    // Update membrane potential
    let tau = membrane_time_constants[neuron_idx];
    let rest = resting_potentials[neuron_idx];
    membrane_potentials[neuron_idx] += (rest - membrane_potentials[neuron_idx]) / tau;
    
    // Check for spike
    let threshold = firing_thresholds[neuron_idx];
    if (membrane_potentials[neuron_idx] >= threshold) {
        fcl_buffer[neuron_idx] = 1;
        membrane_potentials[neuron_idx] = rest;
        refractory_timers[neuron_idx] = refractory_periods[neuron_idx];
    }
}
```

This shader demonstrates several important aspects of the NPU's design:

1. **Sparse Processing**: Only active neurons (those in `active_neuron_indices`) are processed
2. **Area-Based Filtering**: Early termination for neurons in inactive cortical areas
3. **Refractory Period Handling**: Neurons in refractory periods are skipped from further computation
4. **Memory Layout**: Uses Structure of Arrays (SoA) for optimal memory access patterns
5. **Workgroup Optimization**: Uses a workgroup size of 256 for efficient GPU utilization

By using WebGPU, FEAGI achieves high-performance neural simulation across multiple platforms without vendor lock-in, making advanced brain simulation more accessible and portable. 