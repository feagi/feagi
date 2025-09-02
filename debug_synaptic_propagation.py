#!/usr/bin/env python3
"""
Debug script to trace synaptic propagation paths in FEAGI.
This will help identify which code path is being used and why NPU logs aren't showing.
"""

import sys
import os
import traceback
from typing import List

# Add FEAGI to path
sys.path.insert(0, '/Users/nadji/code/FEAGI-2.0/feagi_core')

def trace_synaptic_propagation():
    """Trace all possible synaptic propagation paths."""
    
    print("=== FEAGI SYNAPTIC PROPAGATION DEBUG TRACER ===")
    
    # 1. Check if NPU integration is available
    try:
        from feagi.npu.burst_engine_npu_integration import patch_burst_engine_for_npu, configure_npu_burst_engine
        print("✅ NPU integration module available")
        
        # Check if BurstEngine has been patched
        from feagi.npu.burst_engine import BurstEngine
        if hasattr(BurstEngine, '_process_burst_with_npu_only'):
            print("✅ BurstEngine has been patched for NPU")
        else:
            print("❌ BurstEngine NOT patched for NPU")
            
    except ImportError as e:
        print(f"❌ NPU integration not available: {e}")
    
    # 2. Check ConnectomeManager methods
    try:
        from feagi.bdu.connectome_manager import ConnectomeManager
        
        # Check if update_membrane_potentials delegates to NPU
        import inspect
        source = inspect.getsource(ConnectomeManager.update_membrane_potentials)
        if "_npu_processor.process_neural_burst" in source:
            print("✅ ConnectomeManager.update_membrane_potentials delegates to NPU")
        else:
            print("❌ ConnectomeManager.update_membrane_potentials does NOT delegate to NPU")
            
    except Exception as e:
        print(f"❌ Error checking ConnectomeManager: {e}")
    
    # 3. Check for alternative propagation methods
    propagation_methods = []
    
    try:
        from feagi.bdu.synapse_array import GlobalSynapseArray
        if hasattr(GlobalSynapseArray, 'propagate_activations_simd'):
            propagation_methods.append("BDU: GlobalSynapseArray.propagate_activations_simd")
    except:
        pass
        
    try:
        from feagi.npu.neural_processor import NPUSynapseArray
        if hasattr(NPUSynapseArray, 'propagate_simd'):
            propagation_methods.append("NPU: NPUSynapseArray.propagate_simd")
    except:
        pass
        
    try:
        from feagi.npu.optimized_structures import Connectome
        if hasattr(Connectome, 'propagate_activations'):
            propagation_methods.append("NPU: Connectome.propagate_activations")
    except:
        pass
    
    print(f"\n📋 Available propagation methods:")
    for method in propagation_methods:
        print(f"   - {method}")
    
    # 4. Test actual FEAGI initialization
    print(f"\n🧪 Testing FEAGI initialization...")
    
    try:
        # Initialize basic FEAGI components
        config = {'max_neurons': 100}
        connectome = ConnectomeManager(config, max_synapses=1000)
        
        # Check if NPU processor is set
        if hasattr(connectome, '_npu_processor') and connectome._npu_processor:
            print("✅ ConnectomeManager has NPU processor")
            print(f"   NPU type: {type(connectome._npu_processor)}")
        else:
            print("❌ ConnectomeManager does NOT have NPU processor")
            
        # Try to create a simple test scenario
        print(f"\n🔬 Creating test scenario...")
        
        # Add cortical areas
        area1_id = connectome.add_cortical_area("test_area1", dimensions=(5, 5, 1), position=(0, 0, 0))
        area2_id = connectome.add_cortical_area("test_area2", dimensions=(5, 5, 1), position=(10, 0, 0))
        
        # Create test neurons
        neuron1 = connectome.create_neuron(cortical_id=area1_id, position=(0, 0, 0), threshold=1.0)
        neuron2 = connectome.create_neuron(cortical_id=area2_id, position=(0, 0, 0), threshold=0.5)
        
        # Create synapse
        synapse_success = connectome.create_synapse(neuron1, neuron2, weight=2.0)
        print(f"   Synapse created: {synapse_success}")
        
        # Set neuron to fire
        connectome.set_neuron_property(neuron1, 'membrane_potential', 1.5)
        
        # Try neural processing
        print(f"\n⚡ Testing neural processing...")
        try:
            fired_neurons = connectome.update_membrane_potentials(current_timestep=1)
            print(f"   Fired neurons: {fired_neurons}")
            
            # Check target neuron potential
            target_potential = connectome.get_neuron_property(neuron2, 'membrane_potential')
            print(f"   Target neuron potential: {target_potential}")
            
        except Exception as e:
            print(f"   ❌ Neural processing failed: {e}")
            traceback.print_exc()
            
    except Exception as e:
        print(f"❌ FEAGI initialization failed: {e}")
        traceback.print_exc()
    
    print(f"\n=== DEBUG TRACE COMPLETE ===")

if __name__ == "__main__":
    trace_synaptic_propagation()
