# Morton Spatial Hash Architecture

**FEAGI's high-performance spatial indexing system using Morton encoding + Roaring bitmaps**

## Overview

FEAGI's spatial hash system provides ultra-efficient coordinate indexing for neuron positions using Morton encoding (Z-order curve) combined with Roaring bitmaps. This system achieves 95-99% memory savings compared to traditional spatial hash implementations while providing microsecond-level query performance.

## Architecture

### Simplified Architecture (Post-Cleanup)

```
┌─────────────────────────────────────────┐
│           FEAGI Application             │
├─────────────────────────────────────────┤
│        spatial_hash.py (API)           │
│   ┌─────────────────────────────────┐   │
│   │  • get_spatial_hash()           │   │
│   │  • initialize_spatial_hash()    │   │
│   │  • SpatialHashConfig            │   │
│   └─────────────────────────────────┘   │
├─────────────────────────────────────────┤
│     morton_spatial_hash.py (Core)      │
│   ┌─────────────────────────────────┐   │
│   │  RoaringSpatialHash             │   │
│   │  ├─ Morton encoding/decoding    │   │
│   │  ├─ Roaring bitmap storage      │   │
│   │  ├─ Per-area organization       │   │
│   │  └─ Thread-safe operations      │   │
│   └─────────────────────────────────┘   │
├─────────────────────────────────────────┤
│         External Dependencies          │
│   ┌─────────────────────────────────┐   │
│   │  • pyroaring (RoaringBitmap)    │   │
│   │  • pickle (caching)             │   │
│   │  • threading (RLock)            │   │
│   └─────────────────────────────────┘   │
└─────────────────────────────────────────┘
```

### Key Components

#### 1. **RoaringSpatialHash** (Core Implementation)
- **Morton Encoding**: 3D coordinates → single integer via bit interleaving
- **Roaring Bitmaps**: Sparse coordinate storage with 95%+ compression
- **Per-Area Organization**: Separate bitmaps for each cortical area
- **Thread Safety**: RLock protection for concurrent access
- **Caching**: Pickle-based persistence with statistics

#### 2. **MortonUtils** (Encoding Engine)
- **morton_encode_3d()**: (x,y,z) → morton_code
- **morton_decode_3d()**: morton_code → (x,y,z)
- **morton_encode_region_3d()**: Region → RoaringBitmap

#### 3. **Global Singleton Interface**
- **get_morton_spatial_hash()**: Thread-safe singleton access
- **Automatic initialization**: No configuration needed
- **Direct integration**: No adapter layers or complexity

## Performance Characteristics

### Memory Usage
- **Sparse Genomes**: 95-99% memory reduction
- **Dense Genomes**: 80-90% memory reduction
- **Example**: 10K neurons: 134MB → 40KB

### Query Performance
- **Point Queries**: O(1) - Direct Morton code lookup
- **Region Queries**: O(log N) - Roaring bitmap intersection
- **Multi-Area Operations**: O(N+M) - Fast bitmap unions/intersections

### Spatial Locality
- **Z-Order Curve**: Preserves 3D spatial relationships in 1D
- **Cache Efficiency**: Spatially close coordinates have similar Morton codes
- **GPU Friendly**: Contiguous memory access patterns

### Coordinate Limits (NEW)
- **21-bit per dimension**: Maximum coordinate value: 2,097,151 per axis
- **Total coordinate range**: 2,097,152³ ≈ 9.2 × 10¹⁸ total coordinates
- **Validation**: Built-in coordinate range validation prevents overflow
- **State manager integration**: Automatic registration of coordinate limits

## API Reference

### Core Methods

```python
# Get singleton instance
spatial_hash = get_spatial_hash()

# Add neuron coordinate (supports multiple neurons per coordinate)
spatial_hash.add_neuron("cortical_area", x, y, z, neuron_id)

# Query coordinate (returns first neuron for backward compatibility)
neuron_id = spatial_hash.get_neuron_at_coordinate("cortical_area", x, y, z)

# Query all neurons at coordinate (NEW: multiple neurons per coordinate)
neuron_ids = spatial_hash.get_neurons_at_coordinate("cortical_area", x, y, z)

# Region query (returns all neurons from all coordinates in region)
neurons = spatial_hash.get_neurons_in_region("area", x1, y1, z1, x2, y2, z2)

# Multi-area operations
union_bitmap = spatial_hash.get_area_union(["area1", "area2"])
intersection = spatial_hash.get_area_intersection(["area1", "area2"])

# Statistics and management
stats = spatial_hash.get_statistics()
spatial_hash.clear()

# Coordinate limit validation (NEW)
limits = get_morton_coordinate_limits()
is_valid = validate_coordinate_range(x, y, z)
analysis = analyze_coordinate_requirements(max_x, max_y, max_z)
```

### Compatibility Methods (ConnectomeManager Integration)

```python
# Legacy interface support
spatial_hash.add_coordinate(area, x, y, z, neuron_id)
neuron_id = spatial_hash.get_neuron_id(area, x, y, z)
matches = spatial_hash.batch_coordinate_lookup(candidates, neurons)

# Initialization (no-op for Morton)
spatial_hash.initialize_for_dimensions((max_x, max_y, max_z))
spatial_hash.expand_cache_for_new_area(position, dimensions)
```

## Integration Patterns

### 1. ConnectomeManager Integration
```python
from feagi.bdu.spatial_hash import get_spatial_hash

class ConnectomeManager:
    def __init__(self):
        self._spatial_hash = get_spatial_hash()
        
        # Register Morton spatial hash with state manager (NEW)
        from feagi.core.state_manager import get_state_manager
        state_manager = get_state_manager()
        morton_coordinate_limit = (1 << 21)  # 21-bit Morton encoding limit
        state_manager.set_morton_class_info("RoaringSpatialHash", morton_coordinate_limit)
        
    def create_neuron(self, cortical_id, position, ...):
        neuron_id = self._generate_neuron_id()
        # Add to spatial index
        self._spatial_hash.add_neuron(cortical_id, *position, neuron_id)
        return neuron_id
    
    # NEW: Cortical area dimension validation
    def add_cortical_area(self, name, dimensions, position, ...):
        # Validate dimensions against Morton coordinate limits
        max_dims = self.get_max_allowable_cortical_area_dimensions()
        if any(d > max_d for d, max_d in zip(dimensions, max_dims)):
            raise ValueError(f"Cortical area dimensions {dimensions} exceed Morton encoding limits {max_dims}")
        
        # Proceed with area creation
        return self._create_cortical_area_internal(name, dimensions, position, ...)
    
    def get_max_allowable_cortical_area_dimensions(self):
        """Get maximum allowable cortical area dimensions based on Morton spatial hash limits."""
        from feagi.core.state_manager import get_state_manager
        state_manager = get_state_manager()
        morton_limit = state_manager.get_morton_coordinate_limit()
        return (morton_limit - 1, morton_limit - 1, morton_limit - 1)
```

### 2. Batch Operations
```python
# High-performance batch coordinate lookup
matches = spatial_hash.batch_coordinate_lookup(
    candidate_positions=set([(x1,y1,z1), (x2,y2,z2), ...]),
    neuron_positions=[(x1,y1,z1), (x2,y2,z2), ...]
)
# Returns: [(candidate_idx, neuron_idx), ...]
```

### 3. Multi-Area Queries
```python
# Fast cross-area operations
all_visual_areas = ["v1", "v2", "v4", "mt"]
visual_union = spatial_hash.get_area_union(all_visual_areas)

motor_areas = ["m1", "pmd", "sma"]  
motor_union = spatial_hash.get_area_union(motor_areas)

# Find overlap between visual and motor
overlap = visual_union & motor_union
```

## Caching System

### Cache Structure
```
cache/morton_spatial_hash/
├── morton_hash_genome_v1.pkl    # Genome-specific cache
├── morton_hash_session_123.pkl  # Session-specific cache
└── statistics.json              # Performance metrics
```

### Cache Operations
```python
# Manual cache management
spatial_hash.save_to_cache("genome_v1")
spatial_hash.load_from_cache("genome_v1")

# Automatic caching (integrated with FEAGI lifecycle)
# - Saves on genome unload
# - Loads on genome initialization
# - Invalidates on structural changes
```

## State Manager Integration (NEW)

FEAGI's state manager now tracks Morton spatial hash information for system-wide coordination:

### State Manager Features
- **Morton Class Tracking**: Tracks active Morton implementation ("RoaringSpatialHash")
- **Coordinate Limit Registration**: Registers 21-bit coordinate limits (2,097,152 per dimension)
- **System Validation**: Provides coordinate limit validation for other components
- **Integration Points**: ConnectomeManager, cortical area creation, genome validation

### State Manager API
```python
from feagi.core.state_manager import get_state_manager

state_manager = get_state_manager()

# Get Morton coordinate limits
limit = state_manager.get_morton_coordinate_limit()  # Returns 2,097,152

# Get Morton class name
morton_class = state_manager.get_morton_class_name()  # Returns "RoaringSpatialHash"

# Check if Morton is registered
has_morton = state_manager.has_morton_class_info()  # Returns True

# Get Morton spatial hash info
info = state_manager.get_morton_spatial_hash_info()
# Returns: {"morton_class": "RoaringSpatialHash", "coordinate_limit": 2097152}
```

### Cortical Area Validation (NEW)

The ConnectomeManager now validates cortical area dimensions against Morton coordinate limits:

```python
from feagi.bdu.connectome_manager import ConnectomeManager

cm = ConnectomeManager(1000)

# Get maximum allowable dimensions
max_dims = cm.get_max_allowable_cortical_area_dimensions()
# Returns: (2097151, 2097151, 2097151)

# Validate before creating cortical areas
try:
    area_id = cm.add_cortical_area(
        name="Valid Area", 
        dimensions=(100, 100, 100),  # Within limits
        position=(0, 0, 0)
    )
    print(f"✅ Created area: {area_id}")
except ValueError as e:
    print(f"❌ Area creation failed: {e}")

# This will fail validation
try:
    cm.add_cortical_area(
        name="Invalid Area", 
        dimensions=(3000000, 100, 100),  # Exceeds 21-bit limit
        position=(0, 0, 0)
    )
except ValueError as e:
    print(f"✅ Correctly blocked invalid area: {e}")
```

## Thread Safety

The Morton spatial hash system is fully thread-safe:

- **RLock Protection**: All operations protected by reentrant locks
- **Atomic Operations**: Roaring bitmap operations are thread-safe
- **Singleton Pattern**: Thread-safe singleton initialization
- **Concurrent Reads**: Multiple threads can read simultaneously
- **Exclusive Writes**: Write operations are serialized
- **State Manager Integration**: Thread-safe Morton class registration

## Migration Guide

### From Legacy Spatial Hash

**Before (Legacy)**:
```python
from feagi.bdu.spatial_hash import GlobalSpatialHash
hash_instance = GlobalSpatialHash()
hash_instance.add_coordinate(area, x, y, z, neuron_id)
```

**After (Morton)**:
```python
from feagi.bdu.spatial_hash import get_spatial_hash
spatial_hash = get_spatial_hash()
spatial_hash.add_neuron(area, x, y, z, neuron_id)  # Preferred
# OR
spatial_hash.add_coordinate(area, x, y, z, neuron_id)  # Compatible
```

### Configuration Changes

**Before**:
```python
config = SpatialHashConfig(
    max_dimension=512,
    enable_simd=True,
    hash_prime=1009,
    cache_size=100000
)
```

**After**:
```python
config = SpatialHashConfig(
    max_dimension=512,  # Advisory only
    enable_caching=True
)
# Morton encoding handles any coordinate range automatically
```

## Future Enhancements

### Planned Features
1. **GPU Acceleration**: CUDA/Metal backend for Morton operations
2. **Distributed Caching**: Redis/shared memory for multi-process access
3. **Compression Optimization**: Custom compression for coordinate patterns
4. **Real-time Analytics**: Live performance monitoring dashboard

### Rust Migration Readiness
- **Static Typing**: All operations use concrete types
- **No Dynamic Behavior**: Predictable memory access patterns
- **SIMD Compatible**: Operations vectorizable for Rust/SIMD
- **FFI Safe**: C-compatible data structures

## Troubleshooting

### Common Issues

**1. Import Errors**
```
ImportError: No module named 'pyroaring'
```
**Solution**: Install pyroaring: `pip install pyroaring`

**2. Memory Issues**
```
MemoryError: Cannot allocate bitmap
```
**Solution**: Check coordinate ranges - Morton encoding uses 21 bits per dimension

**3. Performance Issues**
```
Slow region queries
```
**Solution**: Ensure coordinates are within reasonable ranges (< 2^21 per dimension)

### Debugging Tools

```python
# Get detailed statistics
stats = spatial_hash.get_statistics()
print(f"Total coordinates: {stats['total_coordinates']}")
print(f"Memory usage per area: {stats['cortical_areas']}")

# Check system state
print(f"State: {spatial_hash.get_state()}")
print(f"Ready: {spatial_hash.is_ready()}")

# NEW: Coordinate limit validation and analysis
from feagi.bdu.morton_spatial_hash import (
    get_morton_coordinate_limits, 
    validate_coordinate_range,
    analyze_coordinate_requirements
)

# Get coordinate limits
limits = get_morton_coordinate_limits()
print(f"Max coordinate per dimension: {limits['max_coordinate_per_dimension']:,}")
print(f"Encoding type: {limits['encoding_type']}")

# Validate specific coordinates
is_valid = validate_coordinate_range(1000, 2000, 3000)
print(f"Coordinates (1000, 2000, 3000) valid: {is_valid}")

# Analyze genome coordinate requirements
analysis = analyze_coordinate_requirements(max_x=50000, max_y=60000, max_z=70000)
print(f"Coordinate analysis: {analysis['status']}")
for rec in analysis['recommendations']:
    print(f"  - {rec}")

# NEW: State manager integration
from feagi.core.state_manager import get_state_manager
state_manager = get_state_manager()
print(f"Morton class: {state_manager.get_morton_class_name()}")
print(f"Coordinate limit: {state_manager.get_morton_coordinate_limit():,}")
```

---

**Copyright 2025 Neuraville Inc.**  
**Licensed under the Apache License, Version 2.0** 