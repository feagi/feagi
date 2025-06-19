# Brain Data Unit (BDU) Module

*Last Updated: January 19, 2025*

The Brain Data Unit (BDU) module contains FEAGI's core brain simulation data structures and algorithms. It manages neural connectivity, spatial indexing, and the fundamental operations required for artificial brain simulation.

## Overview

The BDU module provides:

- **Neural Connectivity Management**: Synaptic connections and neural networks
- **Spatial Indexing**: High-performance neuron location management
- **Memory Management**: Efficient neural data storage and retrieval
- **Thread Safety**: Concurrent access to brain data structures
- **Performance Optimization**: SIMD, GPU, and embedded-ready implementations

## Core Components

### Connectome Management
- **File**: `connectome_manager.py`
- **Purpose**: Central manager for neural connectivity and brain structure
- **Features**: 
  - Singleton pattern for global brain state
  - Thread-safe operations
  - Integration with spatial hash for location-based queries
  - Support for 10M+ neurons with 15Hz performance target

### Spatial Hash System
- **Files**: `morton_spatial_hash.py`, `spatial_hash_adapter.py`, `spatial_hash.py`
- **Purpose**: Efficient 3D spatial indexing for neuron locations
- **Technology**: Morton encoding + Roaring bitmaps
- **Performance**: 95%+ memory savings, O(log N) region queries
- **Compatibility**: 100% backward compatible through adapter pattern

### Neural Models
- **File**: `models/neuron.py`
- **Purpose**: Core neuron data structures and operations
- **Features**:
  - Cache-aligned arrays for SIMD optimization
  - Block-sparse matrices for connectivity
  - Embedded optimization methods
  - GPU-ready data layouts

## Spatial Hash Architecture

### Morton Encoding System

The BDU uses a sophisticated spatial hash system based on Morton encoding:

```python
from feagi.bdu.spatial_hash import get_spatial_hash

# Automatic Morton encoding backend
spatial_hash = get_spatial_hash()

# Add neurons to spatial index
spatial_hash.add_coordinate("cortical_area_1", x=10, y=20, z=5, neuron_id=12345)

# Fast spatial queries
neurons_in_region = spatial_hash.get_neurons_in_region(
    "cortical_area_1", 
    min_coords=(0, 0, 0), 
    max_coords=(50, 50, 10)
)
```

### Key Benefits

1. **Memory Efficiency**: 95-99% memory savings for sparse genomes
2. **Spatial Locality**: Morton encoding preserves 3D spatial relationships
3. **Performance**: Microsecond region queries and multi-area operations
4. **Scalability**: Handles 10M+ neurons efficiently
5. **Thread Safety**: Concurrent access with proper locking

### Implementation Details

The spatial hash system consists of three layers:

1. **Core Implementation** (`morton_spatial_hash.py`):
   - `MortonUtils` class for 3D coordinate encoding/decoding
   - `RoaringSpatialHash` class using pyroaring BitMap for sparse storage
   - Per-cortical-area organization with thread-safe operations

2. **Backward Compatibility** (`spatial_hash_adapter.py`):
   - `GlobalSpatialHashAdapter` maintaining exact legacy interface
   - State mapping between Morton and legacy systems
   - Complete method compatibility for existing code

3. **Main Interface** (`spatial_hash.py`):
   - Imports from adapter to maintain legacy import patterns
   - Zero breaking changes for existing integrations

## Integration Patterns

### ConnectomeManager Integration

```python
class ConnectomeManager:
    def __init__(self):
        # Automatic Morton spatial hash backend
        self.spatial_hash = get_spatial_hash()
    
    def add_neuron(self, cortical_area, x, y, z, neuron_id):
        # Uses Morton encoding automatically
        return self.spatial_hash.add_coordinate(cortical_area, x, y, z, neuron_id)
    
    def batch_coordinate_lookup(self, candidates, positions):
        # High-performance batch operations
        return self.spatial_hash.batch_coordinate_lookup(candidates, positions)
```

### API Integration

```python
# Neural stimulation with spatial queries
@app.post("/v2/neural/stimulate_region")
async def stimulate_region(request: RegionStimulationRequest):
    connectome = ConnectomeManager.instance()
    spatial_hash = connectome.spatial_hash
    
    # Fast region-based neuron lookup
    neurons = spatial_hash.get_neurons_in_region(
        request.cortical_area,
        request.min_coords,
        request.max_coords
    )
    
    # Stimulate found neurons
    results = connectome.stimulate_neurons(neurons, request.intensity)
    return success_response(data=results)
```

## Performance Characteristics

### Memory Usage Comparison

| Genome Type | Legacy Memory | Morton Memory | Savings |
|-------------|---------------|---------------|---------|
| Dense (1M neurons) | 134MB | 4MB | 97% |
| Sparse (10K neurons) | 134MB | 40KB | 99.97% |
| Very Sparse (1K neurons) | 134MB | 4KB | 99.997% |

### Operation Performance

| Operation | Legacy | Morton | Improvement |
|-----------|--------|---------|-------------|
| Add Neuron | O(1) | O(1) | Same |
| Point Query | O(1) | O(1) | Same |
| Region Query | O(N) | O(log N) | 10-100x faster |
| Multi-area Union | O(N×M) | O(N+M) | 100-1000x faster |

## Thread Safety

All BDU components provide comprehensive thread safety:

```python
# Thread-safe operations
with spatial_hash._lock:
    # Multiple operations guaranteed atomic
    spatial_hash.add_neuron(area, x, y, z, id1)
    spatial_hash.add_neuron(area, x+1, y, z, id2)
```

**Thread Safety Features:**
- Reentrant locks for nested operations
- Atomic bitmap operations  
- Safe concurrent reads
- Protected state transitions

## Cache System

Persistent caching for optimal performance:

```python
# Cache operations
spatial_hash.save_to_cache("genome_v1.2")
spatial_hash.load_from_cache("genome_v1.2")  # ~100x faster startup
```

**Cache Benefits:**
- 100x faster initialization for large genomes
- Persistent dimension analysis
- Performance metrics tracking
- Automatic cache invalidation

## Migration and Compatibility

### Zero-Breaking-Change Migration

The Morton spatial hash system provides **100% backward compatibility**:

```python
# Existing code works unchanged
from feagi.bdu.spatial_hash import get_spatial_hash

spatial_hash = get_spatial_hash()  # Now uses Morton backend automatically
spatial_hash.add_coordinate(area, x, y, z, neuron_id)  # Same interface
neuron_id = spatial_hash.get_neuron_id(area, x, y, z)  # Same interface
```

### Configuration Compatibility

Legacy configuration parameters are automatically handled:

```python
# Legacy config still supported
config = SpatialHashConfig(
    max_dimension=256,
    enable_simd=True,           # Ignored (Morton uses different approach)
    hash_prime=73856093,        # Ignored (Morton uses different approach)
    genome_based_sizing=True    # Converted to Morton equivalent
)
```

## Testing

Comprehensive test coverage with 21 tests:

- **Core Functionality**: Morton encoding/decoding, spatial operations
- **Integration Tests**: ConnectomeManager compatibility, FEAGI import chain
- **Performance Tests**: Benchmarking against legacy implementation
- **Thread Safety**: Concurrent access validation
- **Regression Tests**: Backward compatibility verification

```python
# Run Morton spatial hash tests
python -m pytest tests/cache/test_morton_spatial_hash.py -v
```

## Future Enhancements

### Rust Migration Path

Morton encoding provides excellent foundation for Rust migration:

```rust
pub struct MortonSpatialHash {
    cortical_areas: HashMap<String, RoaringBitmap>,
    neuron_map: HashMap<(String, u64), u32>,
}

impl MortonSpatialHash {
    pub fn morton_encode_3d(x: u32, y: u32, z: u32) -> u64 {
        // SIMD-optimized bit interleaving
    }
}
```

### GPU Acceleration

Morton codes are GPU-friendly for parallel operations:

```python
def gpu_region_query(morton_codes: torch.Tensor, 
                    min_morton: int, max_morton: int) -> torch.Tensor:
    """Parallel Morton range queries on GPU."""
    mask = (morton_codes >= min_morton) & (morton_codes <= max_morton)
    return torch.where(mask)[0]
```

## Related Documentation

- [Morton Spatial Hash Architecture](../docs/arch-morton-spatial-hash.md) - Detailed architecture documentation
- [Data Structures Architecture](../docs/arch-data-structures.md) - Overall FEAGI data structures
- [Embedded Performance Optimization](../docs/arch-embedded-performance-optimization.md) - Performance strategies
- [System Overview](../docs/arch-system-overview.md) - FEAGI system architecture

## API Reference

### Core Classes

- `ConnectomeManager` - Central brain connectivity manager
- `MortonUtils` - Morton encoding utilities
- `RoaringSpatialHash` - Core spatial hash implementation
- `GlobalSpatialHashAdapter` - Backward compatibility layer

### Key Methods

- `get_spatial_hash()` - Get global spatial hash instance
- `add_coordinate(area, x, y, z, neuron_id)` - Add neuron to spatial index
- `get_neuron_id(area, x, y, z)` - Query neuron at coordinates
- `get_neurons_in_region(area, min_coords, max_coords)` - Region-based queries
- `batch_coordinate_lookup(candidates, positions)` - High-performance batch operations

## Examples

### Basic Usage

```python
from feagi.bdu.spatial_hash import get_spatial_hash

# Get spatial hash instance
spatial_hash = get_spatial_hash()

# Add neurons
spatial_hash.add_coordinate("visual_cortex", 10, 20, 5, neuron_id=1001)
spatial_hash.add_coordinate("visual_cortex", 11, 20, 5, neuron_id=1002)

# Query specific location
neuron_id = spatial_hash.get_neuron_id("visual_cortex", 10, 20, 5)
print(f"Neuron at (10,20,5): {neuron_id}")  # Output: 1001

# Region query
neurons = spatial_hash.get_neurons_in_region(
    "visual_cortex",
    min_coords=(5, 15, 0),
    max_coords=(15, 25, 10)
)
print(f"Neurons in region: {neurons}")  # Output: [1001, 1002]
```

### Advanced Usage

```python
# Multi-area operations
areas = ["visual_cortex", "motor_cortex", "memory_area"]
union_results = spatial_hash.get_union_across_areas(areas)

# Batch operations for high performance
candidate_positions = {(10, 20, 5), (11, 20, 5), (12, 20, 5)}
neuron_positions = [(10, 20, 5), (11, 20, 5)]
matches = spatial_hash.batch_coordinate_lookup(candidate_positions, neuron_positions)

# Cache operations
spatial_hash.save_to_cache("my_genome_v1")
# Later...
spatial_hash.load_from_cache("my_genome_v1")  # Much faster startup
```
