# FEAGI Naming Conventions Guide

*Last Updated: May 15, 2025*

## Overview

This guide establishes consistent naming conventions for FEAGI codebases and components. Following these conventions improves code clarity, maintainability, and collaboration among developers.

## 1. Cortical Area Identifiers

FEAGI uses two distinct types of identifiers for cortical areas:

### 1.1 `cortical_id`

- **Definition**: A 6-character unique identifier defined in the genome blueprint
- **Examples**: "iv00_C", "iv00BL", "iv00BR"
- **Usage**: Used in genome files and as the source of truth for cortical area identification
- **Format**: Limited to 6 characters by design
- **Persistence**: Remains consistent across runs and deployments, stored in genome files
- **Domain**: Primary identifier used in the genome and BDU (Brain Development Unit) processes
- **Operations**: Used for all operations where human-readable identifiers are needed or serialization is required

### 1.2 `cortical_idx`

- **Definition**: An auto-incremented integer used internally for efficient indexing and lookups
- **Examples**: 0, 1, 2, 3, etc.
- **Usage**: Used internally throughout the codebase for performance and memory efficiency
- **Format**: Simple integers starting from 0 or 1
- **Generation**: Auto-assigned during brain development based on order in the genome blueprint
- **Volatility**: May change between runs or deployments
- **Domain**: Used for high-performance operations like GPU processing and FCL (Firing Neuron List)
- **Operations**: Used in all compute-intensive operations where performance is critical

### 1.3 Domain Separation

Each identifier type has its specific domain of usage:

- NeuronArray and other GPU-optimized structures should only use `cortical_idx` for maximum performance
- ConnectomeManagerGPU and FCL manager are responsible for translating between `cortical_id` and `cortical_idx`
- APIs and user-facing components should primarily use `cortical_id` for consistency and readability
- Only high-performance computational components should directly use `cortical_idx`

### 1.4 Mapping Between Identifiers

The `Neuroembryogenesis` class maintains bidirectional mappings between these two identifier types:

```python
# Maps internal indices to genome IDs
cortical_id_map[cortical_idx] = cortical_id

# Maps genome IDs to internal indices
reverse_cortical_id_map[cortical_id] = cortical_idx
```

### 1.5 Implementation Details

The naming convention is consistently applied throughout the codebase:

- In `connectome_manager_gpu.py`, the property is called `cortical_idxs` in the NeuronArray class
- In `neuron.py`, the property is called `cortical_idxs` in the NeuronArray class
- In `Connectome` class, the properties `source_cortical_idxs` and `target_cortical_idxs` store integer indices
- All GPU-optimized structures consistently use the `_idxs` suffix for integer index arrays

This consistent naming makes it clear when we're dealing with string identifiers (`cortical_id`) versus integer indices (`cortical_idx`).

### 1.6 Deprecated Terminology

⚠️ **IMPORTANT**: The following terms are deprecated and should NOT be used in new code:

- `area_id` - Previously used to refer to `cortical_idx`
- `i` - Used as a loop variable that referred to `cortical_idx`

## 2. Type Definitions

For improved code readability and type checking, use these type aliases:

```python
CorticalId = str  # 6-character genome ID
CorticalIdx = int  # internal integer index
```

## 3. Coding Conventions

### 3.1 Function Parameters

```python
# CORRECT: Using proper naming convention
def process_cortical_area(cortical_idx: int, neuron_count: int) -> None:
    """
    Process neurons in a cortical area.

    Args:
        cortical_idx: The internal integer ID of the cortical area
        neuron_count: Number of neurons to process
    """
    # Implementation...
```

### 3.2 Accessing Cortical Areas

```python
# Get cortical_idx from cortical_id
cortical_idx = self.reverse_cortical_id_map[cortical_id]
cortical_area = self.cortical_areas[cortical_idx]

# Get cortical_id from cortical_idx
cortical_id = self.cortical_id_map[cortical_idx]
```

### 3.3 Backward Compatibility

For API endpoints that need to maintain backward compatibility:

```python
def get_cortical_area(self, area_id: str) -> Dict[str, Any]:
    """
    Get a cortical area by its index.

    Args:
        area_id: String representation of cortical_idx (kept for backward compatibility)

    Note:
        Despite the parameter name, this value represents the cortical_idx
    """
    try:
        cortical_idx = int(area_id)  # Convert to integer
        area = self._connectome_manager._areas.get(cortical_idx)
        # ...
    except ValueError:
        return None
```

## 4. API URL Structures

For API URL paths, both naming styles are supported for backward compatibility:

- Legacy style with underscores: `/v1/cortical_area/{area_id}`
- Modern style with hyphens: `/v1/cortical-area/{area_id}`

However, all new API endpoints should use the hyphenated kebab-case format.

## 5. Implementation Guidelines

### 5.1 For New Code

1. **Use `cortical_id`** for the 6-character genome identifier
2. **Use `cortical_idx`** for the integer-based internal ID
3. **Never use `area_id`** or other variations
4. **Use type annotations** with the proper type aliases
5. **Write clear docstrings** that explicitly state which ID type is being used

### 5.2 When Modifying Existing Code

1. **Keep parameter names unchanged** if needed for backward compatibility
2. **Add clear comments** explaining the meaning of each identifier
3. **Update internal variables** to use the proper convention
4. **Update docstrings** to clarify parameter meanings

## 6. Examples of Proper Implementation

### 6.1 Class Attributes

```python
class CorticalAreaManager:
    def __init__(self):
        # Mapping from cortical_idx to cortical areas
        self.areas_by_idx = {}

        # Mapping from cortical_id to cortical_idx
        self.idx_by_id = {}
```

### 6.2 Method Implementation

```python
def get_neurons(self, cortical_idx: CorticalIdx) -> List[Neuron]:
    """
    Get all neurons in the specified cortical area.

    Args:
        cortical_idx: The internal integer index of the cortical area

    Returns:
        List of neurons in the cortical area
    """
    if cortical_idx not in self.areas_by_idx:
        raise ValueError(f"Cortical area with idx {cortical_idx} not found")

    return self.areas_by_idx[cortical_idx].neurons
```

## Related Documentation

- [Coding Standards](guide-coding-standards.md)
- [Architecture Overview](arch-system-overview.md)
- [API Formats](spec-api-formats.md)
