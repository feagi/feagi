#!/usr/bin/env python3
"""
Direct Connectome Access for Refractory Period Investigation

This script bypasses the API and directly accesses the ConnectomeManager
to investigate refractory period behavior.
"""

import sys
import os
import time

# Add the current directory to Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def get_some_neuron_data():
    """Get neuron data directly from ConnectomeManager."""
    print("🔍 Accessing FEAGI components directly...")
    
    try:
        # Import FEAGI components
        from feagi.core.state_manager import get_state_manager
        
        # Get the state manager
        state_manager = get_state_manager()
        if not state_manager:
            print("❌ Could not get FEAGI state manager")
            return None
        
        print("✅ Got state manager")
        
        # Try to get connectome manager through different paths
        connectome_manager = None
        
        # Method 1: Through state manager attributes
        if hasattr(state_manager, 'connectome_manager'):
            connectome_manager = state_manager.connectome_manager
            print("✅ Got connectome manager via state_manager.connectome_manager")
        
        # Method 2: Through burst engine
        elif hasattr(state_manager, 'burst_engine'):
            burst_engine = state_manager.burst_engine
            if hasattr(burst_engine, 'connectome_manager'):
                connectome_manager = burst_engine.connectome_manager
                print("✅ Got connectome manager via burst_engine.connectome_manager")
        
        # Method 3: Try importing directly
        if not connectome_manager:
            try:
                from feagi.bdu.connectome_manager import get_connectome_manager
                connectome_manager = get_connectome_manager()
                if connectome_manager:
                    print("✅ Got connectome manager via direct import")
            except:
                pass
        
        if not connectome_manager:
            print("❌ Could not access connectome manager")
            return None
        
        # Enable debug logging for refractory period
        print("\n🔬 Enabling refractory period debug logging...")
        try:
            connectome_manager.enable_refractory_debug_logging()
        except Exception as e:
            print(f"⚠️  Could not enable debug logging: {e}")
        
        # Get some basic info
        print("\n📊 Connectome Info:")
        try:
            # Get all cortical area IDs
            cortical_ids = connectome_manager.get_all_cortical_ids()
            print(f"Available cortical areas: {cortical_ids[:8]}")
            
            # Try to get neurons from a few areas
            neuron_data = {}
            for area_id in cortical_ids[:5]:  # Check first 5 areas
                try:
                    neurons = connectome_manager.get_neurons_by_area(area_id)
                    if neurons:
                        neuron_data[area_id] = neurons[:5]  # First 5 neurons
                        print(f"Area {area_id}: {len(neurons)} neurons, first few: {neurons[:3]}")
                except:
                    continue
            
            return {
                'connectome_manager': connectome_manager,
                'cortical_ids': cortical_ids,
                'neuron_data': neuron_data
            }
            
        except Exception as e:
            print(f"❌ Error getting neuron data: {e}")
            return None
            
    except ImportError as e:
        print(f"❌ Could not import FEAGI modules: {e}")
        return None
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return None

def test_neuron_properties(connectome_manager, neuron_data):
    """Test neuron properties directly."""
    print("\n🧪 Testing neuron properties directly...")
    
    for area_id, neurons in neuron_data.items():
        if not neurons:
            continue
            
        print(f"\n📊 Area {area_id} neuron properties:")
        
        for neuron_id in neurons[:3]:  # Test first 3 neurons
            try:
                properties = connectome_manager.get_neuron_properties(neuron_id)
                if properties:
                    print(f"  Neuron {neuron_id}:")
                    print(f"    MP: {properties.get('membrane_potential', 'N/A'):.2f}")
                    print(f"    RC: {properties.get('refractory_counter', 'N/A')}")
                    print(f"    RP: {properties.get('refractory_period', 'N/A')}")
                    print(f"    Threshold: {properties.get('threshold', 'N/A'):.2f}")
                else:
                    print(f"  Neuron {neuron_id}: No properties found")
            except Exception as e:
                print(f"  Neuron {neuron_id}: Error - {e}")
        
        # If we found neurons with refractory periods, test them
        break

def monitor_neuron_activity(connectome_manager, neuron_data, rounds=5):
    """Monitor neuron activity for refractory behavior."""
    print(f"\n👁️  Monitoring neuron activity for {rounds} rounds...")
    
    # Find neurons to monitor
    test_neurons = []
    for area_id, neurons in neuron_data.items():
        test_neurons.extend(neurons[:2])  # 2 neurons per area
        if len(test_neurons) >= 4:  # Monitor up to 4 neurons
            break
    
    if not test_neurons:
        print("❌ No neurons to monitor")
        return
    
    print(f"🎯 Monitoring neurons: {test_neurons}")
    
    for round_num in range(rounds):
        print(f"\n🔄 Round {round_num + 1}/{rounds}")
        
        refractory_neurons = []
        
        for neuron_id in test_neurons:
            try:
                properties = connectome_manager.get_neuron_properties(neuron_id)
                if properties:
                    mp = properties.get('membrane_potential', 0)
                    rc = properties.get('refractory_counter', 0)
                    area = properties.get('cortical_id', 'N/A')
                    
                    status = ""
                    if rc > 0:
                        refractory_neurons.append(neuron_id)
                        status = f"🚫 REFRACTORY({rc})"
                    
                    print(f"  Neuron {neuron_id} (Area {area}): MP={mp:.2f} RC={rc} {status}")
            except Exception as e:
                print(f"  Neuron {neuron_id}: Error - {e}")
        
        # Check for area-wide suppression
        if len(refractory_neurons) > 1:
            print(f"🚨 POTENTIAL BUG: {len(refractory_neurons)} neurons refractory simultaneously!")
            print(f"   Refractory neurons: {refractory_neurons}")
        
        # Wait for next burst cycle
        time.sleep(0.5)

def main():
    """Main function."""
    print("🔬 Direct Connectome Access for Refractory Investigation")
    print("=" * 65)
    
    # Get connectome access
    data = get_some_neuron_data()
    if not data:
        print("❌ Could not access connectome data")
        print("💡 Make sure FEAGI is running and initialized")
        return
    
    connectome_manager = data['connectome_manager']
    neuron_data = data['neuron_data']
    
    if not neuron_data:
        print("❌ No neuron data found")
        print("💡 This might mean no genome is loaded or no neurons exist")
        return
    
    # Test neuron properties
    test_neuron_properties(connectome_manager, neuron_data)
    
    # Monitor activity
    monitor_neuron_activity(connectome_manager, neuron_data)
    
    print("\n" + "=" * 65)
    print("🏁 INVESTIGATION COMPLETE")
    print("💡 Debug logging is now ENABLED in NeuronArray")
    print("🔬 Watch FEAGI console for '🔥 NEURON DEBUG:' messages when neurons fire")
    print("🚨 Look for '🚨 BUG DETECTED:' messages if area-wide suppression occurs")

if __name__ == "__main__":
    main() 