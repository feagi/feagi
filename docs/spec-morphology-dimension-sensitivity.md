# Morphology Dimension Sensitivity & Intelligent Expansion

*Last Updated: {{DATE}}*

## Overview

The **Dimension Sensitivity** feature introduces intelligent synapse management during cortical area expansion. This feature allows morphologies to specify whether their synaptic patterns are dependent on cortical area dimensions, enabling more precise and efficient expansion behavior.

## Core Concepts

### Dimension Sensitivity

**Dimension Sensitive Morphologies** (`dimension_sensitive: true`):
- Synaptic patterns depend on cortical area dimensions
- Examples: Projectors, grid-based patterns, dimension-aware functions
- **Expansion Behavior**: All synapses using this morphology are deleted and reconstructed to fit new dimensions

**Dimension Agnostic Morphologies** (`dimension_sensitive: false`):
- Synaptic patterns independent of cortical area dimensions  
- Examples: Neighbor-to-neighbor connections, local patterns, vectors
- **Expansion Behavior**: Existing synapses are preserved, patterns extended to new neurons

### Intelligent Expansion Algorithm

When a cortical area undergoes expansion:

1. **Analysis Phase**: Identify all incoming/outgoing morphologies for the expanded area
2. **Classification Phase**: Group morphologies by dimension sensitivity
3. **Selective Reconstruction**: 
   - Dimension-sensitive morphologies: Delete all synapses, rebuild from scratch
   - Dimension-agnostic morphologies: Preserve existing synapses, extend patterns to new regions
4. **Validation Phase**: Ensure synaptic integrity and proper coverage

## API Integration

### Morphology Creation

#### Endpoint: `POST /v1/morphology/create`

```json
{
  "morphology_data": {
    "name": "lateral_neighbor",
    "type": "patterns",
    "parameters": { ... }
  },
  "dimension_sensitive": false
}
```

#### Endpoint: `POST /v1/morphology/morphology`

```json
{
  "morphology_name": "projector_morphology",
  "morphology_type": "functions", 
  "morphology_parameters": { ... },
  "dimension_sensitive": true
}
```

### Auto-Detection Rules

If `dimension_sensitive` is not specified, it's auto-detected based on morphology type:

| Morphology Type | Default Value | Rationale |
|-----------------|---------------|-----------|
| `patterns`      | `false`       | Local patterns typically dimension-agnostic |
| `vectors`       | `false`       | Vector-based connections independent of dimensions |
| `functions`     | `true`        | Functions often dimension-dependent (e.g., projectors) |
| `composite`     | `false`       | Conservative default for complex types |

## Backward Compatibility

### Genome Migration

When loading genomes without `dimension_sensitive` fields:

1. **Auto-Migration**: Missing fields are automatically added during genome validation
2. **Type-Based Defaults**: Values assigned based on morphology type using auto-detection rules
3. **Logging**: All migrations are logged for transparency
4. **No Breaking Changes**: Existing genomes continue to work without modification

### Migration Example

**Before Migration**:
```json
{
  "neuron_morphologies": {
    "block_to_block": {
      "type": "vectors",
      "parameters": { ... }
    }
  }
}
```

**After Migration**:
```json
{
  "neuron_morphologies": {
    "block_to_block": {
      "type": "vectors", 
      "parameters": { ... },
      "dimension_sensitive": false
    }
  }
}
```

## Implementation Architecture

### Genome Validation Integration

The feature integrates with existing `genome_validator.py` infrastructure:

- **`add_missing_dimension_sensitive_fields()`**: Adds missing fields during auto-recovery
- **`morphology_validator()`**: Extended to validate dimension_sensitive fields
- **`sanitize_invalid_morphologies()`**: Includes dimension_sensitive migration in recovery pipeline

### Expansion Service Architecture

**Planned Modular Implementation**:

```
feagi/api/core/services/expansion/
├── connection_analyzer.py    # Analyze area connectivity rules
├── synapse_manager.py       # Preserve/reconstruct synapses  
├── pattern_extender.py      # Extend patterns to new neurons
└── __init__.py
```

## Usage Examples

### Creating Dimension-Sensitive Morphology

```python
# Projector morphology - dimension-sensitive
morphology_request = {
    "morphology_name": "cortical_projector",
    "morphology_type": "functions",
    "morphology_parameters": {
        "projection_type": "grid_based",
        "coverage_ratio": 0.8
    },
    "dimension_sensitive": True
}
```

### Creating Dimension-Agnostic Morphology

```python
# Lateral connectivity - dimension-agnostic
morphology_request = {
    "morphology_name": "lateral_neighbors", 
    "morphology_type": "patterns",
    "morphology_parameters": {
        "neighbor_distance": 1,
        "connection_probability": 0.7
    },
    "dimension_sensitive": False
}
```

### Expansion Scenario

**Setup**: Area B (5x5x1) with:
- Area A → Area B: `projector_morphology` (`dimension_sensitive: true`)
- Area B → Area C: `lateral_neighbors` (`dimension_sensitive: false`)

**Expansion**: Area B expanded to (8x8x1)

**Result**:
- A → B synapses: **Deleted and reconstructed** to cover new 8x8x1 dimensions
- B → C synapses: **Preserved** and extended to new neurons in expanded regions

## Testing & Validation

### Test Coverage

- **Unit Tests**: Individual component functionality
- **Integration Tests**: End-to-end expansion scenarios  
- **Migration Tests**: Genome auto-migration validation
- **API Tests**: Parameter validation and auto-detection

### Validation Rules

1. **Field Presence**: All morphologies must have `dimension_sensitive` field
2. **Type Validation**: Field must be boolean (`true`/`false`)
3. **Migration Integrity**: Auto-migrated fields match expected defaults
4. **Expansion Consistency**: Synapse counts match expected patterns after expansion

## Error Handling

### Common Scenarios

- **Missing Field**: Auto-added during genome validation with appropriate logging
- **Invalid Type**: Validation error with clear remediation guidance
- **Expansion Conflicts**: Detailed logging of which morphologies are being reconstructed vs preserved

### Logging Examples

```
[INFO] AUTO-MIGRATION: Added dimension_sensitive=False to patterns morphology 'lateral_connectivity'
[INFO] AUTO-MIGRATION: Added dimension_sensitive=True to functions morphology 'grid_projector'
[INFO] EXPANSION: Preserving 1,247 synapses for dimension-agnostic morphology 'lateral_neighbors'
[INFO] EXPANSION: Reconstructing 3,891 synapses for dimension-sensitive morphology 'projector_grid'
```

## Future Enhancements

### Planned Features

1. **Pattern Templates**: Pre-defined expansion patterns for common morphology types
2. **Performance Optimization**: Parallel processing for large-scale expansions
3. **Advanced Heuristics**: Machine learning-based pattern extension strategies
4. **Visualization Integration**: Real-time expansion progress in Brain Visualizer

### Compatibility Roadmap

- **Phase 1**: Basic dimension sensitivity (Current)
- **Phase 2**: Advanced pattern detection and extension
- **Phase 3**: GPU-accelerated expansion for large cortical areas
- **Phase 4**: Rust/RTOS migration compatibility

---

## Related Documentation

- [Architecture: Data Structures](arch-data-structures.md)
- [Specification: API Formats](spec-api-formats.md) 
- [Guide: Cortical Area Management](guide-cortical-areas.md)
- [Architecture: Genome & Connectome](arch-genome-connectome.md) 