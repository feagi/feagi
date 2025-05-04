# FEAGI Naming Conventions

## Cortical Area Identifiers

FEAGI uses two types of identifiers for cortical areas:

### 1. `cortical_id`

- **Definition**: A 6-character unique identifier defined in the genome blueprint
- **Examples**: "iv00_C", "iv00BL", "iv00BR"
- **Usage**: Used in the genome files and as the source of truth for cortical area identification
- **Format**: Limited to 6 characters by design
- **Persistence**: Remains consistent across runs and deployments, stored in genome files

### 2. `cortical_idx`

- **Definition**: An auto-incremented integer used internally for efficient indexing and lookups
- **Examples**: 0, 1, 2, 3, etc.
- **Usage**: Used internally throughout the codebase for performance and memory efficiency
- **Format**: Simple integers starting from 0 or 1
- **Generation**: Auto-assigned during brain development based on order in the genome blueprint
- **Volatility**: May change between runs or deployments

## Mapping Between Identifiers

The `Neuroembryogenesis` class maintains bidirectional mappings between these two identifier types:

1. `cortical_id_map[cortical_idx] = cortical_id` - Maps internal indices to genome IDs
2. `reverse_cortical_id_map[cortical_id] = cortical_idx` - Maps genome IDs to internal indices

## Legacy Naming (DEPRECATED)

The following terms should NO LONGER be used:

- `area_id` - Previously used in many places to refer to `cortical_idx`
- `i` - Used as a loop variable that happened to be the `cortical_idx`

⚠️ **IMPORTANT**: The term `area_id` is being phased out. All references to integer IDs for cortical areas should use `cortical_idx` instead.

## API Conventions

To maintain stable interfaces while implementing consistent naming:

- `cortical_id` for the 6-character string identifiers from the genome
- `cortical_idx` for the internal integer indices
- API methods continue to accept string representations of `cortical_idx` for backward compatibility, but internally convert these to integers

## Code Examples

### Using Cortical IDs Correctly

```python
# CORRECT: Using proper naming convention
def my_function(cortical_idx: int, neuron_count: int) -> None:
    """
    Process neurons in a cortical area.
    
    Args:
        cortical_idx: The internal integer ID of the cortical area
        neuron_count: Number of neurons to process
    """
    # Implementation...

# INCORRECT: Using legacy naming
def my_function(area_id: int, neuron_count: int) -> None:
    # This naming is deprecated
    # Implementation...
```

### Accessing a Cortical Area by ID

```python
# In neuroembryogenesis.py
# Get cortical_idx from cortical_id
cortical_idx = self.reverse_cortical_id_map[cortical_id]
cortical_area = self.cortical_areas[cortical_idx]

# Get cortical_id from cortical_idx
cortical_id = self.cortical_id_map[cortical_idx]

# In the API layer
# Convert string representation to integer
def get_cortical_area(self, area_id: str) -> Dict[str, Any]:
    try:
        cortical_idx = int(area_id)  # Convert to integer
        area = self._connectome_manager._areas.get(cortical_idx)
        # ...
    except ValueError:
        return None
```

## Type Definitions

For improved code readability and type checking, we've added type aliases:

```python
CorticalId = str  # 6-character genome ID 
CorticalIdx = int  # internal integer index
```

These should be used in type hints throughout the codebase where possible. 