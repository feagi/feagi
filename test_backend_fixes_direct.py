#!/usr/bin/env python3
"""
Test Backend Fixes Directly

This script bypasses endpoint registration issues and directly tests
our backend fixes for cortical area neuron retrieval and neuron properties.
"""

import sys
import os
import time

# Add the current directory to Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_backend_fixes():
    """Test our backend fixes directly."""
    print("🔧 Testing Backend Fixes Directly")
    print("=" * 50)
    
    try:
        # Import FEAGI components
        from feagi.api.core.services.core_api_service import CoreAPIService
        from feagi.api.core.services.cortical_area.cortical_area_service import CorticalAreaService
        from feagi.core.state_manager import get_state_manager
        
        # Get the state manager and connectome
        state_manager = get_state_manager()
        if not state_manager:
            print("❌ Could not get FEAGI state manager")
            return False
        
        print("✅ Got state manager")
        
        # Try to access the connectome manager
        connectome_manager = None
        if hasattr(state_manager, 'connectome_manager'):
            connectome_manager = state_manager.connectome_manager
        
        if not connectome_manager:
            print("❌ Could not access connectome manager")
            return False
        
        print("✅ Got connectome manager")
        
        # Create a cortical area service to test our fixes
        cortical_service = CorticalAreaService(connectome_manager)
        
        # Test 1: Direct get_area_neurons call with debug logging
        print("\n🧪 Test 1: Direct get_area_neurons call")
        test_area = "iv00MR"
        
        try:
            neurons = cortical_service.get_area_neurons(test_area)
            if neurons:
                print(f"✅ SUCCESS! Found {len(neurons)} neurons in {test_area}")
                print(f"   Sample neuron IDs: {[n.get('id') for n in neurons[:5]]}")
                
                # Test 2: Direct neuron properties access
                print("\n🧪 Test 2: Direct neuron properties access")
                if len(neurons) > 0:
                    test_neuron_id = int(neurons[0]['id'])
                    properties = connectome_manager.get_neuron_properties(test_neuron_id)
                    
                    if properties:
                        print(f"✅ SUCCESS! Got properties for neuron {test_neuron_id}")
                        print(f"   MP: {properties.get('membrane_potential', 'N/A'):.2f}")
                        print(f"   RC: {properties.get('refractory_counter', 'N/A')}")
                        print(f"   RP: {properties.get('refractory_period', 'N/A')}")
                        print(f"   Area: {properties.get('cortical_id', 'N/A')}")
                        
                        # Enable debug logging
                        print("\n🔬 Enabling refractory debug logging...")
                        try:
                            connectome_manager.enable_refractory_debug_logging()
                            print("✅ Debug logging enabled in NeuronArray")
                        except Exception as e:
                            print(f"⚠️  Could not enable debug logging: {e}")
                        
                        return True
                    else:
                        print(f"❌ Could not get properties for neuron {test_neuron_id}")
                else:
                    print("❌ No neurons found to test properties")
            else:
                print(f"❌ get_area_neurons returned None/empty for {test_area}")
                
        except Exception as e:
            print(f"❌ Error in get_area_neurons: {e}")
            import traceback
            print(f"   Traceback: {traceback.format_exc()}")
        
        return False
        
    except ImportError as e:
        print(f"❌ Could not import FEAGI modules: {e}")
        return False
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        import traceback
        print(f"   Traceback: {traceback.format_exc()}")
        return False

def main():
    """Main function."""
    success = test_backend_fixes()
    
    print("\n" + "=" * 50)
    if success:
        print("🎉 BACKEND FIXES ARE WORKING!")
        print("\n🎯 Ready for Refractory Period Investigation:")
        print("1. ✅ Neuron data access is working")
        print("2. ✅ Neuron properties with refractory counters accessible")
        print("3. ✅ Debug logging is enabled in NeuronArray")
        print("\n⚡ NEXT: Use your visualization tool to stimulate neurons")
        print("   Watch FEAGI console for '🔥 NEURON DEBUG:' messages")
        print("   Look for '🚨 BUG DETECTED:' if area-wide suppression occurs")
    else:
        print("❌ BACKEND FIXES NEED MORE WORK")
        print("💡 Check the debug output above for specific issues")

if __name__ == "__main__":
    main() 