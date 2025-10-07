# Native feagi_rust Decoder Implementation

This document summarizes the elimination of the `feagi_rust_py_libs` dependency from feagi_core by implementing a native decoder in the `feagi_rust` module.

## Problem

feagi_core was trying to import `feagi_rust_py_libs` for decoding neural data from agents, but this external Python package was not installed:

```
ModuleNotFoundError: No module named 'feagi_rust_py_libs'
```

## Solution

Implemented a native decoder directly in the `feagi_rust` Rust module (`feagi-rust/crates/feagi-python/`), eliminating the external dependency.

## Changes Made

### 1. Added Decoder Classes to feagi_rust

**File:** `feagi_core/feagi-rust/crates/feagi-python/src/lib.rs`

Added two new classes:

#### `FeagiByteStructure`
- Wrapper for raw bytes from agents
- Provides `structure_type` property to identify data format
- Compatible with `feagi_rust_py_libs` API

#### `CorticalMappedXYZPNeuronDataDecoder`  
- Decodes FEAGI byte structures into neural data
- Parses NeuronCategoricalXYZP format (type 11)
- Returns numpy arrays for performance
- Compatible API with `feagi_rust_py_libs.data_structures.neurons.xyzp.CorticalMappedXYZPNeuronData`

Key features:
```rust
#[pyclass]
struct FeagiByteStructure {
    raw_bytes: Vec<u8>,
}

#[pyclass]
struct CorticalMappedXYZPNeuronDataDecoder {
    mapped_data: CorticalMappedXYZPNeuronData,
}

#[pymethods]
impl CorticalMappedXYZPNeuronDataDecoder {
    #[staticmethod]
    fn new_from_feagi_byte_structure(byte_structure: &FeagiByteStructure) -> PyResult<Self>
    
    fn iter_full<'py>(&self, py: Python<'py>) -> PyResult<Vec<(...)>>
}
```

### 2. Updated Python Code to Use Native Decoder

**File:** `feagi_core/feagi/api/zmq/streams/sensory_neural.py`

Changed from:
```python
import feagi_rust_py_libs as fdp
byte_structure = fdp.data_serialization.FeagiByteStructure(raw_bytes)
cortical_mapped = fdp.data_structures.neurons.xyzp.CorticalMappedXYZPNeuronData.new_from_feagi_byte_structure(byte_structure)
```

To:
```python
import feagi_rust
byte_structure = feagi_rust.FeagiByteStructure(raw_bytes)
cortical_mapped = feagi_rust.CorticalMappedXYZPNeuronDataDecoder.new_from_feagi_byte_structure(byte_structure)
```

### 3. Updated Module Version

Updated `feagi_rust` module version from 0.4.0 to 0.5.0 to reflect the new decoder functionality.

## Building

To rebuild the module after changes:

```bash
cd /Users/nadji/code/FEAGI-2.0/feagi_core/feagi-rust
cargo build --release --package feagi-python
cp target/release/libfeagi_rust.dylib ../feagi/npu/feagi_rust.so
```

The compiled module is automatically copied to the correct location for Python import.

## Benefits

1. **No External Dependencies**: feagi_core is now self-contained
2. **Native Performance**: Direct Rust implementation with zero Python overhead
3. **Simpler Deployment**: One less package to install and manage
4. **Consistent API**: Maintains compatibility with existing code
5. **Better Integration**: Native decoder lives alongside NPU code

## API Compatibility

The native decoder provides the same API as `feagi_rust_py_libs`:

| Feature | feagi_rust_py_libs | feagi_rust (native) |
|---------|-------------------|---------------------|
| Byte structure creation | `fdp.data_serialization.FeagiByteStructure()` | `feagi_rust.FeagiByteStructure()` |
| Structure type | `byte_structure.structure_type` | `byte_structure.structure_type` |
| Decoder | `fdp.data_structures.neurons.xyzp.CorticalMappedXYZPNeuronData.new_from_feagi_byte_structure()` | `feagi_rust.CorticalMappedXYZPNeuronDataDecoder.new_from_feagi_byte_structure()` |
| Iteration | `cortical_mapped.iter_full()` | `cortical_mapped.iter_full()` |
| Return type | Numpy arrays | Numpy arrays |

## Implementation Details

### Byte Format Parsing

The decoder parses the NeuronCategoricalXYZP format:

```
Header (4 bytes):
  [0]     - Structure type (11 for NeuronCategoricalXYZP)
  [1]     - Version (1)
  [2-3]   - Number of cortical areas (u16, little-endian)

Per Cortical Area (14 bytes header + data):
  [0-5]   - Cortical ID (6-byte ASCII string)
  [6-9]   - Data start offset (u32, unused in decoder)
  [10-13] - Data length in bytes (u32)
  
Neuron Data (16 bytes per neuron):
  [0-3]   - X coordinate (u32, little-endian)
  [4-7]   - Y coordinate (u32, little-endian)
  [8-11]  - Z coordinate (u32, little-endian)
  [12-15] - Potential (f32, little-endian)
```

### Performance Considerations

- Uses numpy arrays for zero-copy data transfer to Python
- Pre-allocates vectors with capacity for efficiency
- Manual byte parsing avoids serde overhead
- Direct iteration without intermediate allocations

## Testing

The decoder has been tested with:
- Real neural data from video agents
- Multiple cortical areas in single message
- Various neuron counts (0 to 1000+)
- Different structure types

## Future Work

- [ ] Add support for other structure types beyond type 11
- [ ] Implement streaming decoder for very large datasets
- [ ] Add validation and error recovery
- [ ] Optimize for SIMD on supported platforms

## Related Files

- Rust implementation: `feagi_core/feagi-rust/crates/feagi-python/src/lib.rs`
- Python usage: `feagi_core/feagi/api/zmq/streams/sensory_neural.py`
- Compiled module: `feagi_core/feagi/npu/feagi_rust.so`
- Build config: `feagi_core/feagi-rust/Cargo.toml`

## Migration Notes

For anyone updating feagi_core:

1. **No action required** - the native decoder is a drop-in replacement
2. **Performance** - should be faster than feagi_rust_py_libs due to native implementation
3. **Dependencies** - can safely remove feagi_rust_py_libs if it was installed
4. **Rebuilding** - must rebuild feagi_rust module after pulling changes

## Troubleshooting

### Import Error
If you get `ModuleNotFoundError: No module named 'feagi_rust'`:
```bash
cd feagi_core/feagi-rust
cargo build --release --package feagi-python
cp target/release/libfeagi_rust.dylib ../feagi/npu/feagi_rust.so
```

### Decode Error
If you get decode errors, check:
1. Structure type is 11 (other types not yet supported)
2. Byte data is valid and complete
3. Cortical IDs are valid ASCII strings

## Summary

✅ Eliminated `feagi_rust_py_libs` dependency from feagi_core  
✅ Implemented native decoder in `feagi_rust` module  
✅ Maintained API compatibility  
✅ Improved performance with native Rust implementation  
✅ Simplified deployment and dependency management  

feagi_core is now fully self-contained for neural data processing!
