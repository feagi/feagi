# Per-Area Temporal Depth Configuration Guide

## Overview

Each memory cortical area in FEAGI can have its own custom temporal depth, which determines how far back in time that specific memory area can "look" when forming temporal patterns. This allows for fine-grained control over memory formation characteristics across different brain regions.

## Key Concepts

### Temporal Depth
- **Definition**: The number of timesteps a memory area analyzes when detecting temporal patterns
- **Scope**: Per-memory-area parameter (not global)
- **Range**: Typically 1-50 timesteps, depending on the memory requirements
- **Impact**: Higher values = longer-term pattern detection, more memory usage

### Memory Area Types by Temporal Depth

#### Short-Term Memory Areas (temporal_depth: 1-5)
- **Use Case**: Immediate sensory buffering, working memory
- **Pattern Detection**: Very recent activity patterns
- **Examples**: Sensory echo, immediate motor planning
- **Performance**: Low memory usage, fast processing

#### Medium-Term Memory Areas (temporal_depth: 5-15)
- **Use Case**: Sequence learning, behavioral patterns
- **Pattern Detection**: Short sequences and transitions
- **Examples**: Motor sequences, simple associations
- **Performance**: Moderate memory usage, good balance

#### Long-Term Memory Areas (temporal_depth: 15-50+)
- **Use Case**: Complex pattern recognition, episodic memory
- **Pattern Detection**: Extended temporal relationships
- **Examples**: Complex behaviors, contextual memory
- **Performance**: Higher memory usage, comprehensive patterns

## Configuration Methods

### 1. Cortical Area Template (Default)

```python
# In feagi/evo/templates.py
cortical_template_memory = {
    # ... other properties ...
    "temporal_depth": 3,  # Default for new memory areas
}
```

### 2. Genome Definition (Per-Area Override)

```json
{
    "cortical_areas": {
        "mSTM1": {
            "area_type": "memory",
            "properties": {
                "temporal_depth": 5,  // Short-term memory
                "sub_group_id": "MEMORY"
            }
        },
        "mLTM1": {
            "area_type": "memory", 
            "properties": {
                "temporal_depth": 25,  // Long-term memory
                "sub_group_id": "MEMORY"
            }
        }
    }
}
```

### 3. Runtime Configuration (API)

```python
# Via PlasticityService
plasticity_service.register_memory_area(
    area_idx=42,
    temporal_depth=10,  # Custom temporal depth
    upstream_areas=[1, 2, 3]
)
```

## Implementation Details

### Pattern Detection Flow

1. **Area-Specific Configuration**: Each memory area registers its temporal depth
2. **Pattern Extraction**: Fire ledger extracts `temporal_depth` timesteps of activity
3. **Bitmap Operations**: RoaringBitmap operations on the specific time window
4. **Pattern Hashing**: SHA-256 hash includes temporal depth in pattern identity

### Fire Ledger Integration

```python
# Fire ledger maintains per-area window sizes
fire_ledger.configure_memory_area(
    cortical_idx=area_idx,
    window_size=temporal_depth,  # Custom window size
    upstream_areas=upstream_areas
)
```

### Pattern Detector Integration

```python
# Pattern detector uses area-specific temporal depth
pattern = detector.detect_pattern(
    fire_ledger=fire_ledger,
    memory_area_idx=area_idx,
    upstream_areas=upstream_areas,
    current_timestep=timestep,
    temporal_depth=area_temporal_depth  # Per-area value
)
```

## Performance Considerations

### Memory Usage
- **Linear scaling**: Memory usage scales linearly with temporal depth
- **Per-area impact**: Each area's memory usage is independent
- **Optimization**: Use appropriate temporal depth for each area's function

### Processing Performance
- **Pattern extraction**: O(temporal_depth × upstream_areas)
- **Bitmap operations**: SIMD-optimized, scales well
- **Caching**: Pattern cache shared across areas for efficiency

### GPU Integration
- **CPU-bound**: Pattern detection is CPU-optimized
- **Minimal GPU impact**: No GPU memory transfers for pattern detection
- **Batch processing**: Multiple areas processed efficiently

## Best Practices

### Temporal Depth Selection

1. **Start Small**: Begin with default temporal depth (3)
2. **Measure Impact**: Monitor memory usage and performance
3. **Incremental Increase**: Gradually increase if longer patterns needed
4. **Area-Specific Tuning**: Different areas may need different depths

### Memory Hierarchy Design

```
Sensory Buffer (temporal_depth=2)
    ↓
Working Memory (temporal_depth=5)
    ↓
Sequence Memory (temporal_depth=10)
    ↓
Episodic Memory (temporal_depth=30)
```

### Configuration Validation

- **Minimum**: temporal_depth ≥ 1
- **Maximum**: temporal_depth ≤ 100 (performance consideration)
- **Consistency**: Ensure upstream areas have sufficient history
- **Resource limits**: Consider total memory usage across all areas

## Troubleshooting

### Common Issues

1. **No Pattern Detection**
   - Check temporal_depth > 0
   - Verify upstream areas have activity
   - Ensure fire ledger has sufficient history

2. **High Memory Usage**
   - Reduce temporal_depth for less critical areas
   - Monitor per-area memory consumption
   - Consider pattern cache size limits

3. **Performance Issues**
   - Profile pattern detection timing
   - Reduce temporal_depth if not needed
   - Check bitmap operation efficiency

### Debugging Tools

```python
# Get pattern detector statistics
stats = plasticity_service.get_memory_stats()
print(f"Pattern cache hit ratio: {stats['pattern_detector_stats']}")

# Check area configuration
area_config = fire_ledger.get_memory_area_config(area_idx)
print(f"Temporal depth: {area_config['window_size']}")
```

## Future Enhancements

### Adaptive Temporal Depth
- **Dynamic adjustment**: Temporal depth based on pattern complexity
- **Learning-based**: Adjust based on pattern detection success
- **Resource-aware**: Automatic scaling based on available memory

### Hierarchical Patterns
- **Multi-scale**: Different temporal depths for different pattern scales
- **Cascade detection**: Patterns at multiple temporal resolutions
- **Compression**: Efficient storage of multi-scale patterns

### Rust Migration
- **Performance**: Native Rust implementation for pattern detection
- **Memory efficiency**: Zero-copy operations where possible
- **SIMD optimization**: Platform-specific optimizations
