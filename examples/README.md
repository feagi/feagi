# FEAGI Connector Examples

This directory contains example scripts for using FEAGI Connector with explicit Python and Rust implementations.

## Example Scripts

### 1. Python Implementation Example

**File:** `neuron_processing_example.py`

This script demonstrates using the pure Python implementation of FEAGI byte processing utilities.
It's the most accessible example that works without any additional dependencies beyond the
base FEAGI Connector package.

```bash
python neuron_processing_example.py
```

### 2. Rust Implementation Example

**File:** `neuron_processing_example_rust.py`

This script demonstrates using the high-performance Rust implementation of FEAGI byte processing
utilities. It requires the optional Rust dependency to be installed. If the Rust dependency
is missing, it will fail with a clear error message.

```bash
# First ensure you have the Rust dependency installed
pip install "feagi_connector[rust]"

# Then run the example
python neuron_processing_example_rust.py
```

### 3. Implementation Comparison

**File:** `neuron_processing_comparison.py`

This script provides a side-by-side comparison of the Python and Rust implementations,
including performance benchmarks. It requires the Rust dependency to be installed.

```bash
# First ensure you have the Rust dependency installed
pip install "feagi_connector[rust]"

# Then run the comparison
python neuron_processing_comparison.py
```

## Design Philosophy

These examples demonstrate the explicit approach used in FEAGI Connector for accessing
both Python and Rust implementations:

1. **Python Implementation Example:** Uses only Python implementations with explicit naming
   (`*_python` suffix), guaranteeing it works on all systems.

2. **Rust Implementation Example:** Uses only Rust implementations with explicit naming 
   (`*_rust` suffix), requiring the Rust dependencies to be installed.

3. **Comparison Example:** Uses both implementations side-by-side for direct comparison.

This explicit approach provides:
- Maximum code clarity - always clear which implementation is being used
- Zero runtime overhead - no conditional checks at function call time
- Full control - you decide which implementation to use
- Simple error handling - ImportErrors are handled explicitly

## Expected Performance Results

When running the comparison example, you can expect significant performance improvements
with the Rust implementation, typically in the range of:

- 5-20x faster encoding
- 5-20x faster decoding

The exact improvement will depend on your system and the size of the neuron data being processed. 