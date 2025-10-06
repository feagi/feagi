#!/usr/bin/env python3
"""
Test Rust NPU Integration

This script tests the full Rust NPU Python bindings.
"""

import sys
import os

# Add Rust library path
rust_lib_dir = os.path.join(os.path.dirname(__file__), "feagi-rust", "target", "release")
sys.path.insert(0, rust_lib_dir)

print("=" * 70)
print("TEST 1: Import Rust Module")
print("=" * 70)
try:
    import feagi_rust
    print(f"✅ feagi_rust imported successfully (version {feagi_rust.__version__})")
except ImportError as e:
    print(f"❌ Failed to import feagi_rust: {e}")
    print(f"   Tried path: {rust_lib_dir}")
    sys.exit(1)

print("\n" + "=" * 70)
print("TEST 2: Create Rust NPU")
print("=" * 70)
try:
    npu = feagi_rust.RustNPU(
        neuron_capacity=1000,
        synapse_capacity=10000,
        fire_ledger_window=20
    )
    print(f"✅ RustNPU created successfully")
    print(f"   Neuron count: {npu.get_neuron_count()}")
    print(f"   Synapse count: {npu.get_synapse_count()}")
    print(f"   Burst count: {npu.get_burst_count()}")
except Exception as e:
    print(f"❌ Failed to create RustNPU: {e}")
    sys.exit(1)

print("\n" + "=" * 70)
print("TEST 3: Add Neurons")
print("=" * 70)
try:
    # Add 10 neurons
    neuron_ids = []
    for i in range(10):
        neuron_id = npu.add_neuron(
            threshold=1.0,
            leak_rate=0.1,
            refractory_period=5,
            excitability=1.0,
            cortical_area=1,
            x=i, y=0, z=0
        )
        neuron_ids.append(neuron_id)
    
    print(f"✅ Added {len(neuron_ids)} neurons")
    print(f"   Neuron IDs: {neuron_ids}")
    print(f"   Total neurons: {npu.get_neuron_count()}")
except Exception as e:
    print(f"❌ Failed to add neurons: {e}")
    sys.exit(1)

print("\n" + "=" * 70)
print("TEST 4: Add Synapses")
print("=" * 70)
try:
    # Add synapses in a chain: 0 -> 1 -> 2 -> 3 ...
    synapse_count = 0
    for i in range(len(neuron_ids) - 1):
        npu.add_synapse(
            source=neuron_ids[i],
            target=neuron_ids[i + 1],
            weight=128,
            conductance=255,
            synapse_type=0  # Excitatory
        )
        synapse_count += 1
    
    print(f"✅ Added {synapse_count} synapses")
    print(f"   Total synapses: {npu.get_synapse_count()}")
except Exception as e:
    print(f"❌ Failed to add synapses: {e}")
    sys.exit(1)

print("\n" + "=" * 70)
print("TEST 5: Rebuild Indexes")
print("=" * 70)
try:
    npu.rebuild_indexes()
    print(f"✅ Indexes rebuilt successfully")
except Exception as e:
    print(f"❌ Failed to rebuild indexes: {e}")
    sys.exit(1)

print("\n" + "=" * 70)
print("TEST 6: Set Neuron Mapping")
print("=" * 70)
try:
    # Map all neurons to cortical area 1
    mapping = {neuron_id: 1 for neuron_id in neuron_ids}
    npu.set_neuron_mapping(mapping)
    print(f"✅ Neuron mapping set successfully")
    print(f"   Mapped {len(mapping)} neurons to cortical area 1")
except Exception as e:
    print(f"❌ Failed to set neuron mapping: {e}")
    sys.exit(1)

print("\n" + "=" * 70)
print("TEST 7: Process Bursts (Power Injection)")
print("=" * 70)
try:
    # Process 5 bursts with power injection on neuron 0
    power_neuron = neuron_ids[0]
    
    for burst_num in range(5):
        result = npu.process_burst(power_neurons=[power_neuron])
        print(f"   Burst {result.burst}: {result.neuron_count} neurons fired")
        if result.neuron_count > 0:
            print(f"      Fired: {result.fired_neurons[:5]}{'...' if len(result.fired_neurons) > 5 else ''}")
            print(f"      Power injections: {result.power_injections}")
            print(f"      Synaptic injections: {result.synaptic_injections}")
            print(f"      Neurons processed: {result.neurons_processed}")
            print(f"      Neurons in refractory: {result.neurons_in_refractory}")
    
    print(f"✅ Processed {npu.get_burst_count()} bursts successfully")
except Exception as e:
    print(f"❌ Failed to process bursts: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("\n" + "=" * 70)
print("TEST 8: Dynamic Modifications")
print("=" * 70)
try:
    # Add a new synapse
    npu.add_synapse(
        source=neuron_ids[5],
        target=neuron_ids[7],
        weight=200,
        conductance=255,
        synapse_type=0
    )
    print(f"✅ Added new synapse (5 -> 7)")
    
    # Update a synapse weight
    updated = npu.update_synapse_weight(
        source=neuron_ids[0],
        target=neuron_ids[1],
        new_weight=255
    )
    print(f"✅ Updated synapse weight (0 -> 1): {updated}")
    
    # Rebuild indexes
    npu.rebuild_indexes()
    print(f"✅ Indexes rebuilt after modifications")
    
    print(f"   Total synapses: {npu.get_synapse_count()}")
except Exception as e:
    print(f"❌ Failed dynamic modifications: {e}")
    sys.exit(1)

print("\n" + "=" * 70)
print("INTEGRATION TEST SUMMARY")
print("=" * 70)
print("✅ All tests passed!")
print(f"\n📊 Final State:")
print(f"   - Neurons: {npu.get_neuron_count()}")
print(f"   - Synapses: {npu.get_synapse_count()}")
print(f"   - Bursts processed: {npu.get_burst_count()}")

print("\n🎯 Next step: Integrate with burst_engine.py for real genome testing")
print("   - Load essential_genome.json")
print("   - Process 1000+ bursts")
print("   - Benchmark performance vs Python")
