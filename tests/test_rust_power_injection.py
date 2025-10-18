#!/usr/bin/env python3
"""
Test Rust NPU power injection from _power cortical area.

This test validates:
1. Power neuron extraction from connectome (_power area)
2. Storage in Rust NPU via set_power_neurons()
3. Injection into FCL during burst processing
4. Power neurons appear in fire queue after burst
"""

import pytest
from pathlib import Path

def test_rust_power_neuron_extraction_and_injection():
    """Test that power neurons are extracted from _power area and injected by Rust NPU."""
    
    # Import after pytest starts (to avoid import errors if FEAGI not installed)
    from feagi.bdu.connectome_manager import ConnectomeManager
    from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
    from feagi.utils.config import FeagiConfig
    
    # Find essential genome
    possible_paths = [
        Path("feagi/evo/defaults/genome/essential_genome.json"),
        Path("../../feagi/evo/defaults/genome/essential_genome.json"),
        Path("../feagi/evo/defaults/genome/essential_genome.json"),
    ]
    
    genome_path = None
    for path in possible_paths:
        if path.exists():
            genome_path = str(path.absolute())
            break
    
    if not genome_path:
        pytest.skip("Essential genome not found")
    
    print("\n" + "=" * 80)
    print("🧪 RUST POWER INJECTION TEST")
    print("=" * 80)
    
    # Step 1: Create connectome and develop brain
    print("\n📦 Step 1: Creating connectome and developing brain...")
    config = FeagiConfig({})
    connectome = ConnectomeManager(config)
    
    embryogenesis = NeuroEmbryogenesis(connectome, config)
    success = embryogenesis.develop_brain(genome_path)
    
    assert success, "❌ Brain development failed"
    print("✅ Brain developed successfully")
    
    # Step 2: Verify _power area exists
    print("\n📦 Step 2: Verifying _power cortical area...")
    assert "_power" in connectome.cortical_areas, "❌ _power area not found in connectome"
    
    power_area = connectome.get_cortical_area("_power")
    print(f"✅ _power area found: cortical_idx={power_area.cortical_idx}")
    print(f"   Dimensions: {power_area.dimensions}")
    print(f"   Position: {power_area.position}")
    
    # Step 3: Extract power neurons
    print("\n📦 Step 3: Extracting power neurons from _power area...")
    power_neurons = connectome.get_neurons_by_area("_power")
    
    assert power_neurons is not None, "❌ get_neurons_by_area returned None"
    assert len(power_neurons) > 0, "❌ _power area has no neurons"
    
    print(f"✅ Extracted {len(power_neurons)} power neurons")
    print(f"   Power neuron IDs: {power_neurons}")
    
    # Step 4: Get Rust NPU from connectome
    print("\n📦 Step 4: Getting Rust NPU integration...")
    assert hasattr(connectome, '_npu_interface'), "❌ ConnectomeManager has no _npu_interface"
    
    npu_interface = connectome._npu_interface
    assert hasattr(npu_interface, '_rust_npu_integration'), "❌ NPU Interface has no _rust_npu_integration"
    
    rust_npu_integration = npu_interface._rust_npu_integration
    assert rust_npu_integration is not None, "❌ Rust NPU Integration is None"
    assert rust_npu_integration._rust_npu is not None, "❌ Rust NPU is None"
    
    rust_npu = rust_npu_integration._rust_npu
    print(f"✅ Rust NPU available")
    
    # Step 5: Set power neurons in Rust NPU
    print("\n📦 Step 5: Setting power neurons in Rust NPU...")
    rust_npu.set_power_neurons(power_neurons)
    print(f"✅ Set {len(power_neurons)} power neurons in Rust NPU")
    
    # Step 6: Process a burst and verify power injection
    print("\n📦 Step 6: Processing burst and checking power injection...")
    
    # Get initial burst count
    initial_burst_count = rust_npu_integration.get_burst_count()
    print(f"   Initial burst count: {initial_burst_count}")
    
    # Process one burst
    result = rust_npu_integration.process_burst(power_neurons=power_neurons)
    
    print(f"\n   Burst result:")
    print(f"   - Burst #: {result['burst']}")
    print(f"   - Power injections: {result.get('power_injections', 0)}")
    print(f"   - Synaptic injections: {result.get('synaptic_injections', 0)}")
    print(f"   - Fired neurons: {len(result['fired_neurons'])}")
    print(f"   - Fired neuron IDs: {result['fired_neurons'][:10] if len(result['fired_neurons']) > 10 else result['fired_neurons']}")
    
    # Verify power neurons were injected
    assert result.get('power_injections', 0) == len(power_neurons), \
        f"❌ Expected {len(power_neurons)} power injections, got {result.get('power_injections', 0)}"
    
    print(f"\n✅ Power injection successful: {result.get('power_injections', 0)}/{len(power_neurons)} neurons injected")
    
    # Step 7: Verify power neurons appear in fire queue (they should fire with threshold=1.0 and potential=1.0)
    print("\n📦 Step 7: Verifying power neurons fired...")
    
    fired_neurons = result['fired_neurons']
    power_neurons_fired = [n for n in power_neurons if n in fired_neurons]
    
    print(f"   Power neurons that fired: {power_neurons_fired}")
    print(f"   Expected: {power_neurons}")
    
    # Power neurons should fire because they get potential=1.0 and threshold=1.0
    assert len(power_neurons_fired) > 0, "❌ No power neurons fired! Check threshold/potential"
    
    print(f"\n✅ {len(power_neurons_fired)}/{len(power_neurons)} power neurons fired successfully")
    
    # Final summary
    print("\n" + "=" * 80)
    print("✅ RUST POWER INJECTION TEST PASSED")
    print("=" * 80)
    print(f"   ✓ Extracted {len(power_neurons)} power neurons from _power area")
    print(f"   ✓ Stored in Rust NPU")
    print(f"   ✓ Injected {result.get('power_injections', 0)} neurons into FCL")
    print(f"   ✓ {len(power_neurons_fired)} power neurons fired")
    print("=" * 80 + "\n")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])

