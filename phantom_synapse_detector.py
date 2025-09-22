#!/usr/bin/env python3
"""
Phantom Synapse Detector

Uses the fixed get_incoming_connections method to detect phantom synapses
that are causing activation in isolated cortical areas.

This script connects to running FEAGI and directly investigates the 
ConnectomeManager to find the source of corruption.
"""

import os
import sys
import time
import requests
from typing import Dict, List, Set

def get_live_fcl_data() -> Dict:
    """Get current FCL data from running FEAGI API."""
    try:
        response = requests.get("http://127.0.0.1:8000/v1/burst_engine/fcl", timeout=5)
        if response.status_code == 200:
            return response.json()
    except Exception as e:
        print(f"❌ Failed to get FCL data: {e}")
    return {}

def investigate_phantom_synapses():
    """Investigate phantom synapses using the fixed SynapseArray."""
    print("🔍 PHANTOM SYNAPSE DETECTOR")
    print("="*50)
    
    # Set FEAGI_INITIALIZED to connect to running instance
    os.environ["FEAGI_INITIALIZED"] = "1"
    
    try:
        # Import FEAGI components 
        from feagi.bdu.connectome_manager import ConnectomeManager
        
        # Get the singleton instance (should connect to running FEAGI)
        cm = ConnectomeManager.instance()
        print(f"✅ Connected to ConnectomeManager with {len(cm.cortical_areas)} areas")
        
        # Get current FCL data
        fcl_data = get_live_fcl_data()
        if not fcl_data:
            print("❌ Could not get FCL data - using manual investigation")
            target_area = "cS7aaa"
            target_neurons = [22598]  # Known firing neuron
        else:
            print(f"📊 Current FCL state:")
            print(f"   Timestep: {fcl_data.get('timestep', 'unknown')}")
            print(f"   Active areas: {list(fcl_data.get('cortical_areas', {}).keys())}")
            
            target_area = "cS7aaa"
            target_neurons = fcl_data.get('cortical_areas', {}).get(target_area, [])
            
        if target_area not in cm.cortical_areas:
            print(f"❌ Area '{target_area}' not found in ConnectomeManager")
            print("   Available areas:")
            for area_id in sorted(cm.cortical_areas.keys())[:10]:
                print(f"     - {area_id}")
            return
            
        print(f"\n🎯 Investigating area: {target_area}")
        print(f"   Target neurons: {target_neurons}")
        
        # Get cortical index
        cortical_idx = cm.cortical_mapping.get_idx(target_area)
        if cortical_idx is None:
            print(f"❌ No cortical_idx found for area '{target_area}'")
            return
            
        print(f"   Cortical index: {cortical_idx}")
        
        # Get all neurons in the area
        try:
            area_neurons = cm.get_neurons_by_cortical_area(target_area)
            print(f"   Registered neurons in area: {len(area_neurons)}")
            if len(area_neurons) > 0:
                print(f"   Sample neurons: {area_neurons[:10]}")
        except Exception as e:
            print(f"❌ Could not get area neurons: {e}")
            area_neurons = target_neurons  # Fallback to FCL neurons
            
        # Check for phantom synapses using the fixed method
        print(f"\n🔍 PHANTOM SYNAPSE ANALYSIS:")
        phantom_synapses_found = 0
        phantom_details = []
        
        # Check each firing neuron for incoming connections
        neurons_to_check = target_neurons if target_neurons else area_neurons[:5]
        
        for neuron_id in neurons_to_check:
            print(f"\n  📍 Checking neuron {neuron_id}:")
            
            try:
                # Use the fixed get_incoming_connections method
                incoming_connections = cm.get_incoming_connections(neuron_id)
                
                if incoming_connections:
                    print(f"     🚨 PHANTOM SYNAPSES FOUND: {len(incoming_connections)}")
                    phantom_synapses_found += len(incoming_connections)
                    
                    for source_id, weight in incoming_connections[:10]:  # Show first 10
                        # Get source area
                        try:
                            source_area = cm.get_cortical_area_for_neuron(source_id)
                            print(f"       - Source: {source_id} ({source_area}) → {neuron_id} (weight: {weight:.3f})")
                            
                            phantom_details.append({
                                'target_neuron': neuron_id,
                                'source_neuron': source_id,
                                'source_area': source_area,
                                'weight': weight
                            })
                        except Exception as e:
                            print(f"       - Source: {source_id} (unknown area) → {neuron_id} (weight: {weight:.3f})")
                            phantom_details.append({
                                'target_neuron': neuron_id,
                                'source_neuron': source_id,
                                'source_area': 'unknown',
                                'weight': weight
                            })
                    
                    if len(incoming_connections) > 10:
                        print(f"       ... and {len(incoming_connections) - 10} more connections")
                        
                else:
                    print(f"     ✅ No incoming connections (as expected for isolated area)")
                    
            except KeyError as e:
                print(f"     ⚠️  Neuron mapping error: {e}")
            except Exception as e:
                print(f"     ❌ Error checking connections: {e}")
                
        # Summary
        print(f"\n🏁 INVESTIGATION SUMMARY:")
        print(f"   Target area: {target_area}")
        print(f"   Neurons investigated: {len(neurons_to_check)}")
        print(f"   Phantom synapses found: {phantom_synapses_found}")
        
        if phantom_synapses_found > 0:
            print(f"\n🚨 ROOT CAUSE IDENTIFIED!")
            print(f"   Your 'isolated' area has {phantom_synapses_found} incoming synapses!")
            print(f"   This is definitely causing the spurious activations.")
            
            # Group by source area
            source_areas = {}
            for detail in phantom_details:
                area = detail['source_area']
                source_areas[area] = source_areas.get(area, 0) + 1
                
            print(f"\n📊 PHANTOM SYNAPSE SOURCES:")
            for source_area, count in sorted(source_areas.items(), key=lambda x: x[1], reverse=True):
                print(f"     - {source_area}: {count} synapses")
                
            print(f"\n🎯 RECOMMENDATIONS:")
            print(f"   1. Investigate how these synapses were created")
            print(f"   2. Check connectome loading/genome processing")
            print(f"   3. Remove phantom synapses to fix isolation")
            
        else:
            print(f"   ✅ No phantom synapses detected")
            print(f"   The corruption may be in FCL injection logic or neuron mapping")
            
    except Exception as e:
        print(f"❌ Investigation failed: {e}")
        import traceback
        traceback.print_exc()

def main():
    """Main entry point."""
    investigate_phantom_synapses()

if __name__ == "__main__":
    main()
