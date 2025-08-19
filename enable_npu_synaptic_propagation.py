#!/usr/bin/env python3
"""
FEAGI NPU Synaptic Propagation Enabler

This script shows you exactly how to enable NPU synaptic propagation in your FEAGI system.
Add these lines to your FEAGI startup code to see the debug logs and get proper synaptic propagation.
"""

import sys
import os

# Add FEAGI to path if needed
sys.path.insert(0, '/Users/nadji/code/FEAGI-2.0/feagi_core')

def enable_npu_synaptic_propagation():
    """Enable NPU synaptic propagation with full debug logging."""
    
    print("=== ENABLING NPU SYNAPTIC PROPAGATION ===")
    
    # Step 1: Import and patch BurstEngine for NPU
    from feagi.npu.burst_engine_npu_integration import patch_burst_engine_for_npu
    patch_burst_engine_for_npu()
    print("✅ Step 1: BurstEngine patched for NPU processing")
    
    # Step 2: Create your ConnectomeManager as usual
    from feagi.bdu.connectome_manager import ConnectomeManager
    config = {'max_neurons': 1000}  # Adjust to your needs
    connectome = ConnectomeManager(config, max_synapses=10000)  # Adjust to your needs
    print("✅ Step 2: ConnectomeManager created")
    
    # Step 3: Create and configure BurstEngine with NPU
    from feagi.npu.burst_engine import BurstEngine
    
    # Create BurstEngine
    burst_engine = BurstEngine(
        connectome_manager=connectome
    )
    print("✅ Step 3: BurstEngine created")
    
    # Step 4: Initialize NPU processor
    burst_engine.initialize_npu_processor(
        max_neurons=1000,      # Adjust to your needs
        max_synapses=10000,    # Adjust to your needs
        backend="cpu"          # or "cuda", "wgpu" if available
    )
    print("✅ Step 4: NPU processor initialized")
    
    # Step 5: Enable NPU processing
    burst_engine.enable_npu_processing()
    print("✅ Step 5: NPU processing enabled")
    
    # Step 6: Verify NPU is working
    if hasattr(connectome, '_npu_processor') and connectome._npu_processor:
        print("✅ Step 6: NPU processor is properly configured")
        print(f"   NPU backend: {connectome._npu_processor.backend}")
        print(f"   Max neurons: {connectome._npu_processor.max_neurons:,}")
        print(f"   Max synapses: {connectome._npu_processor.max_synapses:,}")
    else:
        print("❌ Step 6: NPU processor not configured")
        return False
    
    print("\n🎉 NPU SYNAPTIC PROPAGATION ENABLED SUCCESSFULLY!")
    print("\n📋 What you'll see now:")
    print("   - [NPU-BURST-DEBUG] logs during neural processing")
    print("   - [NPU-SYNAPSE-DEBUG] logs during synaptic propagation")
    print("   - [CONNECTOME-DEBUG] logs when delegating to NPU")
    print("   - Proper neuron ID to array index mapping")
    print("   - Working cross-cortical synaptic propagation")
    
    return True

def test_npu_propagation():
    """Test NPU synaptic propagation with a simple example."""
    
    print("\n=== TESTING NPU SYNAPTIC PROPAGATION ===")
    
    if not enable_npu_synaptic_propagation():
        return False
    
    # Get the configured connectome
    from feagi.bdu.connectome_manager import ConnectomeManager
    connectome = ConnectomeManager._instance
    
    # Create test scenario
    area1_id = connectome.add_cortical_area("source_area", dimensions=(5, 5, 1), position=(0, 0, 0))
    area2_id = connectome.add_cortical_area("target_area", dimensions=(5, 5, 1), position=(10, 0, 0))
    
    # Create neurons
    source_neuron = connectome.create_neuron(cortical_id=area1_id, position=(0, 0, 0), threshold=1.0)
    target_neuron = connectome.create_neuron(cortical_id=area2_id, position=(0, 0, 0), threshold=0.5)
    
    # Create synapse
    connectome.create_synapse(source_neuron, target_neuron, weight=2.0)
    
    # Set source neuron to fire
    connectome.set_neuron_property(source_neuron, 'membrane_potential', 1.5)
    
    print(f"\n🧪 Test setup complete:")
    print(f"   Source neuron {source_neuron}: potential=1.5, threshold=1.0")
    print(f"   Target neuron {target_neuron}: potential=0.0, threshold=0.5")
    print(f"   Synapse: {source_neuron} -> {target_neuron}, weight=2.0")
    
    # Process neural burst - this should show all our debug logs
    print(f"\n⚡ Processing neural burst...")
    fired_neurons = connectome.update_membrane_potentials(current_timestep=1)
    
    # Check results
    target_potential = connectome.get_neuron_property(target_neuron, 'membrane_potential')
    
    print(f"\n📊 Results:")
    print(f"   Fired neurons: {fired_neurons}")
    print(f"   Target neuron potential: {target_potential}")
    
    if source_neuron in fired_neurons and target_potential > 0:
        print("✅ NPU synaptic propagation working correctly!")
        return True
    else:
        print("❌ NPU synaptic propagation not working")
        return False

if __name__ == "__main__":
    success = test_npu_propagation()
    
    if success:
        print(f"\n🎯 TO USE IN YOUR FEAGI SYSTEM:")
        print(f"Add these lines to your FEAGI startup code:")
        print(f"```python")
        print(f"from feagi.npu.burst_engine_npu_integration import patch_burst_engine_for_npu")
        print(f"patch_burst_engine_for_npu()")
        print(f"")
        print(f"# After creating your BurstEngine:")
        print(f"burst_engine.initialize_npu_processor(max_neurons=YOUR_MAX, max_synapses=YOUR_MAX)")
        print(f"burst_engine.enable_npu_processing()")
        print(f"```")
        print(f"\nThen you'll see all the debug logs and proper synaptic propagation!")
    
    sys.exit(0 if success else 1)
