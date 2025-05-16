# FEAGI Protocol Version Migration Guide

This document explains how to add new versions of byte structure formats to the FEAGI protocol system.

## Table of Contents
1. [Version Numbering](#version-numbering)
2. [Adding a New Structure Version](#adding-a-new-structure-version)
3. [Versioning Process](#versioning-process)
4. [Client-Server Compatibility](#client-server-compatibility)
5. [Migration Path for Clients](#migration-path-for-clients)
6. [Testing Version Compatibility](#testing-version-compatibility)

## Version Numbering

Each byte structure has its own version number, which is a simple integer starting from 1.
Version numbers should increase monotonically as new formats are introduced.

The version is stored in the second byte of the universal header that all byte structures use:

```
┌─────────────────┬─────────────────┬─────────────────────────┐
│ Structure Type  │ Version Number  │      Message Data      │
│    (1 byte)     │    (1 byte)     │    (variable size)     │
└─────────────────┴─────────────────┴─────────────────────────┘
```

## Adding a New Structure Version

When a byte structure format needs changes (for performance, new features, etc.), follow these steps:

### 1. Update the version registry

In `feagi/api/protocols/byte_structures/encoder.py`:

```python
SUPPORTED_VERSIONS = {
    ByteStructureID.JSON: [1],
    ByteStructureID.RAW_IMAGE: [1],
    ByteStructureID.MULTI_HOLDER: [1],
    ByteStructureID.NEURON_FLAT: [1, 2],  # Add the new version here
    ByteStructureID.NEURON_CATEGORIES: [1],
}
```

### 2. Add version-specific encoder method

Add a new private method for the new version:

```python
def _encode_neuron_flat_v2(self, 
                          cortical_ids: List[str],
                          x_coords: List[int],
                          y_coords: List[int], 
                          z_coords: List[int],
                          potentials: List[float],
                          # New parameters for version 2:
                          metadata: Optional[Dict] = None,
                          compression_type: int = 0) -> bytes:
    """
    Version 2 implementation of neuron flat format.
    
    Adds:
    - Metadata section (optional JSON)
    - Compression type byte
    """
    # Implementation with the new format...
```

### 3. Add version-specific decoder method

Add a corresponding decoder method in `feagi/api/protocols/byte_structures/decoder.py`:

```python
def _decode_neuron_flat_v2(self, data: bytes) -> Dict[str, Any]:
    """
    Decode version 2 of neuron flat format.
    
    New features:
    - Metadata section
    - Compression type support
    """
    # Implementation to decode the new format...
```

### 4. Update documentation

Document the new format in comments and update `feagi/api/protocols/README.md` to explain the new version.

## Versioning Process

1. **Design**: Define changes needed in the byte structure format
2. **Document**: Write detailed specs on what changes and why
3. **Implement**: Add encoder/decoder methods for the new version
4. **Test**: Create tests that verify both versions work correctly
5. **Release**: Include version changes in release notes

## Client-Server Compatibility

During handshake, clients and servers exchange supported structure versions:

1. Client sends its supported versions in the capabilities message
2. Server registers these capabilities
3. Translator selects the appropriate version when creating messages:
   ```python
   version = translator.get_supported_version(client_id, ByteStructureID.NEURON_FLAT)
   ```

This allows:
- Newer clients to work with older servers
- Older clients to work with newer servers
- Gradual rollout of new protocol versions

## Migration Path for Clients

For client libraries, we recommend:

1. Always implement and support all previous versions
2. When receiving an unknown version, fall back to the highest version supported
3. Log warnings when using compatibility modes
4. Recommend users to update to support newer versions

## Testing Version Compatibility

For each new version, implement tests that verify:

1. New version can be encoded/decoded correctly
2. Old clients can still work with new servers
3. New clients can still work with old servers
4. Version negotiation selects the correct version

Example test case:

```python
def test_version_negotiation():
    # Create translator with client capabilities
    translator = ByteStructureTranslator()
    
    # Register a client that only supports version 1
    translator.register_client_capabilities("old_client", {
        "structure_versions": {
            ByteStructureID.NEURON_FLAT: [1]
        }
    })
    
    # Register a client that supports versions 1 and 2
    translator.register_client_capabilities("new_client", {
        "structure_versions": {
            ByteStructureID.NEURON_FLAT: [1, 2]
        }
    })
    
    # Check version selection
    assert translator.get_supported_version("old_client", ByteStructureID.NEURON_FLAT) == 1
    assert translator.get_supported_version("new_client", ByteStructureID.NEURON_FLAT) == 2
``` 