#!/usr/bin/env python3
"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
FEAGI Connector Neuron Processing Comparison

This example compares Python and Rust implementations side by side.
It explicitly imports both implementations and handles the ImportError 
for the Rust implementation at the top level.
"""

import asyncio
import logging
import time
import sys
from feagi_connector.utils import NEURON_POTENTIAL_CATEGORICAL_XYZ

# Always import Python implementations
from feagi_connector.utils.processing import (
    encode_neuron_potential_xyz_python,
    decode_neuron_potential_xyz_python
)

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("example")

# Import Rust implementations at top level to abort early if not available
# This will raise ImportError if Rust is not available
try:
    from feagi_connector.utils.rust_processing import (
        encode_neuron_potential_xyz_rust,
        decode_neuron_potential_xyz_rust
    )
    logger.info("Rust implementation available - will perform comparison")
except ImportError as e:
    logger.error(f"Rust implementation not available: {e}")
    logger.error("Please install with Rust support: pip install \"feagi_connector[rust]\"")
    sys.exit(1)


def print_neuron_data(neuron_data, label="Neuron Data"):
    """Print formatted neuron data"""
    print(f"\n{label}:")
    print("-" * (len(label) + 1))
    for (x, y, z), potential in sorted(neuron_data.items()):
        print(f"  Position ({x:3d}, {y:3d}, {z:3d}): {potential:.2f}")
    print()


def benchmark_comparison(neuron_data, iterations=1000):
    """Run benchmarks comparing Python and Rust implementations"""
    print("\n===== PERFORMANCE COMPARISON =====")
    
    # Benchmark Python implementation
    start_time = time.time()
    for _ in range(iterations):
        encode_neuron_potential_xyz_python(neuron_data)
    py_encode_time = time.time() - start_time
    
    start_time = time.time()
    encoded = encode_neuron_potential_xyz_python(neuron_data)
    for _ in range(iterations):
        decode_neuron_potential_xyz_python(encoded)
    py_decode_time = time.time() - start_time
    
    # Benchmark Rust implementation
    start_time = time.time()
    for _ in range(iterations):
        encode_neuron_potential_xyz_rust(neuron_data)
    rust_encode_time = time.time() - start_time
    
    start_time = time.time()
    encoded = encode_neuron_potential_xyz_rust(neuron_data)
    for _ in range(iterations):
        decode_neuron_potential_xyz_rust(encoded)
    rust_decode_time = time.time() - start_time
    
    # Print results
    print("\nResults:")
    print("-" * 60)
    print(f"{'Operation':<15} {'Python Time':<15} {'Rust Time':<15} {'Speedup':<15}")
    print("-" * 60)
    print(f"{'Encoding':<15} {py_encode_time:.6f} s      {rust_encode_time:.6f} s      {py_encode_time/rust_encode_time:.2f}x")
    print(f"{'Decoding':<15} {py_decode_time:.6f} s      {rust_decode_time:.6f} s      {py_decode_time/rust_decode_time:.2f}x")
    print("-" * 60)
    
    return {
        "py_encode": py_encode_time,
        "py_decode": py_decode_time,
        "rust_encode": rust_encode_time,
        "rust_decode": rust_decode_time,
        "encode_speedup": py_encode_time / rust_encode_time,
        "decode_speedup": py_decode_time / rust_decode_time
    }


def compare_implementations(neuron_data):
    """Compare Python and Rust implementations"""
    print("\n===== IMPLEMENTATION COMPARISON =====")
    
    # Encode with Python
    py_start = time.time()
    py_encoded = encode_neuron_potential_xyz_python(neuron_data)
    py_encode_time = time.time() - py_start
    print(f"Python encoding: {len(py_encoded)} bytes in {py_encode_time:.6f} seconds")
    
    # Encode with Rust
    rust_start = time.time()
    rust_encoded = encode_neuron_potential_xyz_rust(neuron_data)
    rust_encode_time = time.time() - rust_start
    print(f"Rust encoding:   {len(rust_encoded)} bytes in {rust_encode_time:.6f} seconds")
    
    # Decode with Python
    py_start = time.time()
    py_decoded = decode_neuron_potential_xyz_python(py_encoded)
    py_decode_time = time.time() - py_start
    print(f"Python decoding: {len(py_decoded)} neurons in {py_decode_time:.6f} seconds")
    
    # Decode with Rust
    rust_start = time.time()
    rust_decoded = decode_neuron_potential_xyz_rust(rust_encoded)
    rust_decode_time = time.time() - rust_start
    print(f"Rust decoding:   {len(rust_decoded)} neurons in {rust_decode_time:.6f} seconds")
    
    # Compare results
    py_keys = set(py_decoded.keys())
    rust_keys = set(rust_decoded.keys())
    print(f"\nNeuron coordinates match: {py_keys == rust_keys}")
    
    # Compare values
    value_diffs = 0
    for key in py_keys:
        if abs(py_decoded[key] - rust_decoded[key]) > 0.001:  # Allow small float differences
            value_diffs += 1
    
    print(f"Value differences: {value_diffs} out of {len(py_keys)} neurons")
    
    # Test exact byte equality (may differ due to implementation details)
    print(f"Encoded byte equality: {py_encoded == rust_encoded}")
    print(f"Encoded byte lengths: Python={len(py_encoded)}, Rust={len(rust_encoded)}")
    
    return py_encoded, rust_encoded, py_decoded, rust_decoded


def main():
    """Main function for comparison"""
    print("======================================================")
    print("  FEAGI CONNECTOR - IMPLEMENTATION COMPARISON")
    print("======================================================")
    
    # Create sample neuron data
    print("Creating sample neuron data...")
    neuron_data = {}
    
    # Create a larger dataset for more meaningful benchmarks
    for x in range(10, 30):
        for y in range(20, 40):
            for z in range(1, 3):
                # Generate values between 0.0 and 1.0
                neuron_data[(x, y, z)] = (x - 10) / 20.0
    
    print(f"Created {len(neuron_data)} sample neurons")
    
    # Compare implementations
    py_encoded, rust_encoded, py_decoded, rust_decoded = compare_implementations(neuron_data)
    
    # Run benchmarks
    print("\nRunning benchmarks...")
    results = benchmark_comparison(neuron_data, iterations=1000)
    
    # Summary
    print("\n======================================================")
    print("  PERFORMANCE SUMMARY")
    print("======================================================")
    print(f"Rust encoding is {results['encode_speedup']:.2f}x faster than Python")
    print(f"Rust decoding is {results['decode_speedup']:.2f}x faster than Python")
    print(f"Overall, using Rust can significantly improve performance for")
    print(f"neuron data processing in FEAGI Connector.")
    print("======================================================")


if __name__ == "__main__":
    main() 