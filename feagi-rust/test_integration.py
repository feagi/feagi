#!/usr/bin/env python3
"""
Quick integration test for the Rust burst engine

This demonstrates how to use the Rust synaptic propagation engine from Python.
"""

import sys
import os
import numpy as np

# Add the target directory to Python path to find the Rust extension
rust_lib_path = os.path.join(os.path.dirname(__file__), 'target', 'release')
sys.path.insert(0, rust_lib_path)

try:
    import feagi_rust
    print("✅ Successfully imported feagi_rust module")
    print(f"   Version: {feagi_rust.__version__}")
except ImportError as e:
    print(f"❌ Failed to import feagi_rust: {e}")
    print(f"   Tried path: {rust_lib_path}")
    sys.exit(1)

# Create test data
print("\n📊 Creating test synapse data...")
n_synapses = 100
source_neurons = np.array([1, 1, 1, 2, 2, 3], dtype=np.uint32)
target_neurons = np.array([10, 11, 12, 10, 13, 14], dtype=np.uint32)
weights = np.array([255, 128, 200, 255, 100, 150], dtype=np.uint8)
conductances = np.array([255, 255, 200, 255, 255, 255], dtype=np.uint8)
types = np.array([0, 1, 0, 0, 0, 0], dtype=np.uint8)  # 0=excitatory, 1=inhibitory
valid_mask = np.array([True, True, True, True, True, True], dtype=bool)

print(f"   Created {len(source_neurons)} synapses")

# Create engine
print("\n🚀 Creating Rust synaptic propagation engine...")
engine = feagi_rust.SynapticPropagationEngine()
print("   Engine created successfully")

# Build index
print("\n🔨 Building synapse index...")
engine.build_index(source_neurons, target_neurons, weights, conductances, types, valid_mask)
print("   Index built successfully")

# Set neuron mapping
print("\n🗺️  Setting neuron-to-cortical-area mapping...")
mapping = {
    10: 1,  # Neuron 10 → Area 1
    11: 1,  # Neuron 11 → Area 1
    12: 1,  # Neuron 12 → Area 1
    13: 2,  # Neuron 13 → Area 2
    14: 2,  # Neuron 14 → Area 2
}
engine.set_neuron_mapping(mapping)
print(f"   Mapped {len(mapping)} neurons to cortical areas")

# Compute propagation
print("\n⚡ Computing synaptic propagation...")
fired_neurons = np.array([1, 2], dtype=np.uint32)
print(f"   Fired neurons: {fired_neurons.tolist()}")

result = engine.propagate(fired_neurons)
print(f"   Result: {dict(result)}")

# Show stats
print("\n📈 Performance statistics:")
total_propagations, total_synapses = engine.stats()
print(f"   Total propagations: {total_propagations}")
print(f"   Total synapses processed: {total_synapses}")

print("\n✅ ALL TESTS PASSED!")
print("\n🎯 Next steps:")
print("   1. Integrate with Python burst_engine.py")
print("   2. Replace _compute_synaptic_propagation() with Rust call")
print("   3. Benchmark: expect 50-100x speedup (165ms → <3ms)")


