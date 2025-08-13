#!/usr/bin/env python3
"""
Test Neuron Properties API Endpoint

This script tests the new /connectome/neuron/{neuron_id}/properties endpoint
and uses it to investigate refractory period behavior.
"""

import requests
import json
import time
from typing import Dict, List, Any, Optional

# Configuration
FEAGI_API_BASE = "http://localhost:8000/v1"

def api_request(method: str, endpoint: str, data: Dict = None) -> Any:
    """Make API request to FEAGI."""
    url = f"{FEAGI_API_BASE}{endpoint}"
    headers = {"Content-Type": "application/json"}
    
    try:
        if method == "GET":
            response = requests.get(url, headers=headers)
        elif method == "POST":
            response = requests.post(url, json=data, headers=headers)
        elif method == "PUT":
            response = requests.put(url, json=data, headers=headers)
        else:
            raise ValueError(f"Unsupported method: {method}")
            
        if response.status_code == 200:
            return response.json()
        else:
            print(f"❌ API Error {response.status_code}: {response.text}")
            return None
    except Exception as e:
        print(f"❌ Request failed: {e}")
        return None

def test_neuron_properties_endpoint():
    """Test the new neuron properties endpoint."""
    print("🧪 Testing new neuron properties endpoint...")
    
    # Try a range of neuron IDs to find some that exist
    test_neuron_ids = list(range(1, 100, 10)) + list(range(100, 1000, 100))
    
    found_neurons = []
    
    for neuron_id in test_neuron_ids:
        print(f"Testing neuron ID {neuron_id}...", end=" ")
        
        result = api_request("GET", f"/connectome/neuron/{neuron_id}/properties")
        
        if result and result.get("neuron_id"):
            print("✅ FOUND!")
            found_neurons.append(neuron_id)
            print(f"  Properties: MP={result.get('membrane_potential', 'N/A'):.2f}, "
                  f"RC={result.get('refractory_counter', 'N/A')}, "
                  f"RP={result.get('refractory_period', 'N/A')}, "
                  f"Area={result.get('cortical_id', 'N/A')}")
            
            if len(found_neurons) >= 5:  # Found enough for testing
                break
        else:
            print("❌")
    
    return found_neurons

def monitor_refractory_behavior(neuron_ids: List[int], rounds: int = 10):
    """Monitor refractory behavior of specific neurons."""
    print(f"\n🔬 Monitoring refractory behavior for {len(neuron_ids)} neurons over {rounds} rounds...")
    
    for round_num in range(rounds):
        print(f"\n🔄 Round {round_num + 1}/{rounds}")
        
        # Get current states
        for neuron_id in neuron_ids:
            result = api_request("GET", f"/connectome/neuron/{neuron_id}/properties")
            
            if result:
                mp = result.get('membrane_potential', 0)
                rc = result.get('refractory_counter', 0)
                rp = result.get('refractory_period', 0)
                area = result.get('cortical_id', 'N/A')
                
                status = ""
                if rc > 0:
                    status = f"🚫 REFRACTORY({rc})"
                if mp > result.get('threshold', 0):
                    status += " ⚡ ABOVE_THRESHOLD"
                
                print(f"  Neuron {neuron_id} (Area {area}): MP={mp:.2f} RC={rc}/{rp} {status}")
        
        # Wait for burst processing
        time.sleep(0.5)

def inject_power_and_observe(cortical_area: str, neuron_ids: List[int]):
    """Inject power into an area and observe neuron responses."""
    print(f"\n⚡ Injecting power into area {cortical_area} and observing neuron responses...")
    
    # Get baseline states
    print("📊 Baseline states:")
    baseline_states = {}
    for neuron_id in neuron_ids:
        result = api_request("GET", f"/connectome/neuron/{neuron_id}/properties")
        if result:
            baseline_states[neuron_id] = result
            print(f"  Neuron {neuron_id}: MP={result.get('membrane_potential', 0):.2f}, "
                  f"RC={result.get('refractory_counter', 0)}")
    
    # Try multiple stimulation methods
    stimulation_methods = [
        {"endpoint": "/power/inject", "data": {"area_id": cortical_area, "power_level": 1.0}},
        {"endpoint": "/stimulation/cortical_stimulation", "data": {"cortical_area": cortical_area, "intensity": 1.0}},
        {"endpoint": "/stimulation/area_stimulation", "data": {"area_name": cortical_area, "power_level": 0.8}}
    ]
    
    for method in stimulation_methods:
        print(f"\n🎯 Trying stimulation: {method['endpoint']}")
        result = api_request("POST", method["endpoint"], method["data"])
        if result:
            print("✅ Stimulation successful")
            break
        else:
            print("❌ Stimulation failed")
    else:
        print("⚠️  All stimulation methods failed - will monitor for natural activity")
    
    # Monitor for changes
    print("\n👁️  Monitoring for changes (next 8 rounds)...")
    for round_num in range(8):
        print(f"\n🔄 Post-stimulation round {round_num + 1}")
        
        fired_neurons = []
        refractory_neurons = []
        
        for neuron_id in neuron_ids:
            result = api_request("GET", f"/connectome/neuron/{neuron_id}/properties")
            if result:
                mp = result.get('membrane_potential', 0)
                rc = result.get('refractory_counter', 0)
                baseline_mp = baseline_states.get(neuron_id, {}).get('membrane_potential', 0)
                
                # Check if neuron fired (membrane potential reset or refractory counter > 0)
                if rc > 0:
                    refractory_neurons.append(neuron_id)
                
                if abs(mp - baseline_mp) > 1.0:  # Significant change
                    fired_neurons.append(neuron_id)
                
                status = ""
                if rc > 0:
                    status += f"🚫 REFRACTORY({rc}) "
                if abs(mp - baseline_mp) > 1.0:
                    status += "⚡ CHANGED "
                
                print(f"  Neuron {neuron_id}: MP={mp:.2f} (Δ{mp-baseline_mp:+.2f}) RC={rc} {status}")
        
        print(f"📊 Summary: {len(fired_neurons)} changed, {len(refractory_neurons)} refractory")
        
        # BUG DETECTION
        if len(refractory_neurons) > 1:
            print(f"🚨 POTENTIAL BUG: {len(refractory_neurons)} neurons refractory simultaneously!")
            print(f"   Refractory neurons: {refractory_neurons}")
            print(f"   This might indicate area-wide suppression!")
        
        time.sleep(0.5)

def main():
    """Main function to test neuron properties and investigate refractory behavior."""
    print("🔬 FEAGI Neuron Properties API Test")
    print("=" * 60)
    
    # Test the new endpoint
    found_neurons = test_neuron_properties_endpoint()
    
    if not found_neurons:
        print("❌ No neurons found - cannot proceed with testing")
        print("💡 This might mean:")
        print("  - FEAGI needs to be restarted to pick up the new endpoint")
        print("  - No genome is loaded")
        print("  - Neuron IDs are in a different range")
        return
    
    print(f"\n✅ Found {len(found_neurons)} test neurons: {found_neurons}")
    
    # Group neurons by cortical area
    neuron_areas = {}
    for neuron_id in found_neurons:
        result = api_request("GET", f"/connectome/neuron/{neuron_id}/properties")
        if result:
            area = result.get('cortical_id', 'unknown')
            if area not in neuron_areas:
                neuron_areas[area] = []
            neuron_areas[area].append(neuron_id)
    
    print(f"\n📊 Neurons by cortical area:")
    for area, neurons in neuron_areas.items():
        print(f"  {area}: {neurons}")
    
    # Test refractory behavior for each area
    for area, neurons in neuron_areas.items():
        if len(neurons) >= 2:  # Need multiple neurons to detect area-wide suppression
            print(f"\n🧪 Testing refractory behavior in area {area}")
            inject_power_and_observe(area, neurons)
            break
    else:
        # Just monitor general activity
        print(f"\n👁️  Monitoring general neuron activity...")
        monitor_refractory_behavior(found_neurons[:3])
    
    print("\n" + "=" * 60)
    print("🏁 TEST COMPLETE")
    print("💡 If you saw '🚨 POTENTIAL BUG', the area-wide refractory suppression is confirmed!")
    print("🔬 Check FEAGI logs for detailed debug output from NeuronArray")

if __name__ == "__main__":
    main() 