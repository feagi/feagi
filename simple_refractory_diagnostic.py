#!/usr/bin/env python3
"""
Simple Refractory Period Diagnostic Script

Tests refractory period behavior using existing cortical areas.
"""

import requests
import json
import time
from typing import Dict, List, Any

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

def get_area_info(cortical_id: str) -> Dict:
    """Get information about a cortical area."""
    result = api_request("GET", f"/connectome/cortical_info/{cortical_id}")
    if result and "area_info" in result:
        return result["area_info"]
    return {}

def find_best_test_area() -> str:
    """Find the best cortical area for testing (one with multiple neurons)."""
    print("🔍 Looking for cortical areas with multiple neurons...")
    
    # Get list of all areas
    areas = api_request("GET", "/connectome/cortical_areas/list/summary")
    if not areas:
        print("❌ Could not get cortical areas list")
        return None
    
    # Check areas to find one with multiple neurons
    for area_id in areas:
        if area_id in ["_death", "___pwr"]:  # Skip core areas
            continue
            
        info = get_area_info(area_id)
        if info:
            neuron_count = len(info.get("neuron_ids", []))
            dimensions = info.get("dimensions", [])
            print(f"📊 Area {area_id}: {neuron_count} neurons, dimensions: {dimensions}")
            
            if neuron_count >= 4:  # Need at least 4 neurons for good testing
                print(f"✅ Selected area {area_id} for testing ({neuron_count} neurons)")
                return area_id
    
    # Fallback to any area with more than 1 neuron
    for area_id in areas:
        if area_id in ["_death", "___pwr"]:
            continue
            
        info = get_area_info(area_id)
        if info and len(info.get("neuron_ids", [])) > 1:
            print(f"⚠️  Using area {area_id} with {len(info.get('neuron_ids', []))} neurons (limited options)")
            return area_id
    
    print("❌ No suitable test areas found")
    return None

def get_area_detailed_state(cortical_id: str) -> Dict:
    """Get detailed state of all neurons in an area."""
    # First try to get the area info
    info = get_area_info(cortical_id)
    if not info:
        return {}
    
    neuron_ids = info.get("neuron_ids", [])
    print(f"📊 Area {cortical_id} has {len(neuron_ids)} neurons")
    
    # Try to get neuron states - this might not work with all API endpoints
    states = {}
    
    # Method 1: Try individual neuron property endpoint
    print("🔍 Attempting to get individual neuron states...")
    for i, neuron_id in enumerate(neuron_ids[:5]):  # Limit to first 5 for testing
        result = api_request("GET", f"/connectome/neuron/{neuron_id}/properties")
        if result:
            states[neuron_id] = result
            print(f"  ✅ Got state for neuron {neuron_id}")
        else:
            print(f"  ❌ Failed to get state for neuron {neuron_id}")
            
        if i >= 4:  # Only test first 5 neurons
            break
    
    return {
        "area_info": info,
        "neuron_states": states
    }

def inject_power_to_area(cortical_id: str):
    """Inject power/activity into an area."""
    print(f"⚡ Injecting power into area {cortical_id}...")
    
    # Method 1: Try direct stimulation endpoint
    injection_data = {
        "cortical_area": cortical_id,
        "power_level": 1.0
    }
    
    result = api_request("POST", "/stimulation/cortical_stimulation", injection_data)
    if result:
        print(f"✅ Stimulated area {cortical_id}")
        return True
    
    # Method 2: Try power injection
    power_data = {
        "area_id": cortical_id,
        "intensity": 10.0
    }
    
    result = api_request("POST", "/power/inject", power_data)
    if result:
        print(f"✅ Injected power into area {cortical_id}")
        return True
    
    print(f"❌ Could not stimulate area {cortical_id}")
    return False

def monitor_refractory_behavior(cortical_id: str):
    """Monitor refractory behavior in a cortical area."""
    print(f"🔬 Starting refractory period monitoring for area {cortical_id}")
    print("=" * 60)
    
    # Get initial state
    print("📊 Getting initial state...")
    initial_state = get_area_detailed_state(cortical_id)
    
    if not initial_state.get("neuron_states"):
        print("❌ Could not get neuron states - API endpoints may not be available")
        print("💡 This suggests we need to check the low-level neuron array directly")
        return "API_LIMITATIONS"
    
    neuron_ids = list(initial_state["neuron_states"].keys())
    print(f"👁️  Monitoring {len(neuron_ids)} neurons: {neuron_ids}")
    
    # Show initial states
    print("\n📋 Initial States:")
    for neuron_id, state in initial_state["neuron_states"].items():
        mp = state.get("membrane_potential", "N/A")
        rc = state.get("refractory_counter", "N/A")
        th = state.get("threshold", "N/A")
        print(f"  Neuron {neuron_id}: MP={mp} RC={rc} TH={th}")
    
    # Inject stimulation
    print(f"\n⚡ Stimulating area {cortical_id}...")
    inject_power_to_area(cortical_id)
    
    # Monitor for several rounds
    for round_num in range(1, 8):
        print(f"\n🔄 Round {round_num} (waiting for burst processing...)")
        time.sleep(0.5)  # Wait for burst processing
        
        # Get current state
        current_state = get_area_detailed_state(cortical_id)
        neuron_states = current_state.get("neuron_states", {})
        
        if not neuron_states:
            print("❌ Lost connection to neuron states")
            break
        
        refractory_neurons = []
        fired_neurons = []
        
        print("📊 Current States:")
        for neuron_id, state in neuron_states.items():
            mp = state.get("membrane_potential", 0)
            rc = state.get("refractory_counter", 0)
            th = state.get("threshold", 0)
            fired = state.get("fired_recently", False)
            
            status = ""
            if fired:
                fired_neurons.append(neuron_id)
                status += "🔥FIRED "
            if rc > 0:
                refractory_neurons.append(neuron_id)
                status += f"🚫REFRACTORY({rc}) "
                
            print(f"  Neuron {neuron_id}: MP={mp:.2f} RC={rc} TH={th:.2f} {status}")
        
        print(f"🔥 Fired: {fired_neurons}")
        print(f"🚫 Refractory: {refractory_neurons}")
        
        # Bug detection
        if len(refractory_neurons) > 1:
            print(f"\n🚨 POTENTIAL BUG: {len(refractory_neurons)} neurons are refractory simultaneously")
            print(f"   This might indicate area-wide suppression if they all fired together")
        
        if len(refractory_neurons) == 0 and round_num > 5:
            print("✅ All neurons have completed refractory period")
            break
    
    return "MONITORING_COMPLETE"

def run_simple_diagnostic():
    """Run the simplified diagnostic."""
    print("🔬 FEAGI Simple Refractory Diagnostic")
    print("=" * 50)
    
    # Find a good test area
    test_area = find_best_test_area()
    if not test_area:
        print("❌ No suitable test areas found")
        return
    
    # Monitor the area
    result = monitor_refractory_behavior(test_area)
    
    # Report results
    print(f"\n" + "=" * 50)
    print("🏁 DIAGNOSTIC RESULTS")
    print("=" * 50)
    
    if result == "API_LIMITATIONS":
        print("⚠️  Could not access individual neuron states via API")
        print("💡 NEXT STEP: Direct neuron array inspection needed")
        print("🔧 SOLUTION: Add debug logging to NeuronArray.embedded_optimized_neural_update()")
    elif result == "MONITORING_COMPLETE":
        print("✅ Monitoring completed - check output above for refractory patterns")
    
    print("\n💡 RECOMMENDED NEXT STEPS:")
    print("1. Add debug logging to neuron array firing logic")
    print("2. Check if refractory_counters array has shared memory")
    print("3. Verify individual neuron indexing is correct")

if __name__ == "__main__":
    run_simple_diagnostic() 