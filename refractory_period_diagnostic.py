#!/usr/bin/env python3
"""
Refractory Period Diagnostic Script

This script diagnoses the refractory period bug where entire cortical areas
are suppressed instead of individual neurons.

Usage: python3 refractory_period_diagnostic.py
"""

import numpy as np
import requests
import json
import time
from typing import Dict, List, Any

# Configuration
FEAGI_API_BASE = "http://localhost:8000/v1"
TEST_AREA_NAME = "test_refrac"

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

def ensure_genome_loaded():
    """Ensure a genome is loaded."""
    print("🧬 Ensuring genome is loaded...")
    result = api_request("POST", "/genome/upload/barebones", {})
    if result and result.get("success"):
        print("✅ Barebones genome loaded")
        return True
    else:
        print("❌ Failed to load genome")
        return False

def create_test_area():
    """Create a test cortical area with multiple neurons."""
    print(f"🏗️  Creating test area '{TEST_AREA_NAME}'...")
    
    # Create a 2x2x1 area (4 neurons total)
    area_data = {
        "cortical_name": f"Test Refractory Area",
        "cortical_dimensions": [2, 2, 1],  # 4 neurons total
        "coordinates_3d": [10, 10, 0],
        "cortical_group": "CUSTOM",
        "cortical_type": "interconnect", 
        "sub_group_id": "TEST",
        "parent_region_id": "root",
        "neurons_per_voxel": 1,
        "firing_threshold": 5.0,
        "refractory_period": 10,  # 10 burst refractory period
        "leak_coefficient": 0
    }
    
    result = api_request("POST", "/cortical_area/custom_cortical_area", area_data)
    if result and "cortical_id" in result:
        cortical_id = result["cortical_id"]
        print(f"✅ Created test area: {cortical_id}")
        return cortical_id
    else:
        print("❌ Failed to create test area")
        return None

def get_area_neurons(cortical_id: str) -> List[int]:
    """Get all neuron IDs in a cortical area."""
    result = api_request("GET", f"/connectome/cortical_info/{cortical_id}")
    if result and "area_info" in result:
        neuron_ids = result["area_info"].get("neuron_ids", [])
        print(f"📊 Area {cortical_id} has {len(neuron_ids)} neurons: {neuron_ids}")
        return neuron_ids
    else:
        print(f"❌ Failed to get neurons for area {cortical_id}")
        return []

def inject_current_to_neuron(neuron_id: int, current: float = 10.0):
    """Inject current into a specific neuron."""
    injection_data = {
        "neuron_id": neuron_id,
        "current": current
    }
    
    result = api_request("POST", "/stimulation/direct_stimulation", injection_data)
    if result:
        print(f"⚡ Injected {current}A current into neuron {neuron_id}")
        return True
    else:
        print(f"❌ Failed to inject current into neuron {neuron_id}")
        return False

def get_neuron_states(neuron_ids: List[int]) -> Dict[int, Dict]:
    """Get detailed state information for specific neurons."""
    states = {}
    
    for neuron_id in neuron_ids:
        result = api_request("GET", f"/connectome/neuron/{neuron_id}/properties")
        if result:
            states[neuron_id] = {
                "membrane_potential": result.get("membrane_potential", 0),
                "refractory_counter": result.get("refractory_counter", 0),
                "threshold": result.get("threshold", 0),
                "fired_recently": result.get("fired_recently", False)
            }
        else:
            states[neuron_id] = {"error": "Failed to get state"}
    
    return states

def monitor_area_activity(cortical_id: str, rounds: int = 15):
    """Monitor neuron activity in the test area over multiple rounds."""
    print(f"🔍 Monitoring area {cortical_id} for {rounds} rounds...")
    
    # Get neurons in the area
    neuron_ids = get_area_neurons(cortical_id)
    if not neuron_ids:
        print("❌ No neurons found in test area")
        return
        
    print(f"👁️  Monitoring {len(neuron_ids)} neurons: {neuron_ids}")
    
    # Inject current into just the FIRST neuron
    target_neuron = neuron_ids[0]
    print(f"🎯 Target neuron for stimulation: {target_neuron}")
    
    for round_num in range(rounds):
        print(f"\n🔄 Round {round_num + 1}/{rounds}")
        
        # Inject current into just one neuron
        if round_num == 0:
            inject_current_to_neuron(target_neuron, 15.0)  # Above threshold
            print(f"⚡ Stimulated neuron {target_neuron} (should fire)")
        
        # Wait for burst processing
        time.sleep(0.1)
        
        # Get states of ALL neurons in the area
        states = get_neuron_states(neuron_ids)
        
        # Analyze states
        fired_neurons = []
        refractory_neurons = []
        
        print("📊 Neuron States:")
        for neuron_id, state in states.items():
            if "error" in state:
                print(f"  Neuron {neuron_id}: ERROR - {state['error']}")
                continue
                
            mp = state["membrane_potential"]
            rc = state["refractory_counter"] 
            th = state["threshold"]
            fired = state.get("fired_recently", False)
            
            status = ""
            if fired:
                fired_neurons.append(neuron_id)
                status += "🔥FIRED "
            if rc > 0:
                refractory_neurons.append(neuron_id)
                status += f"🚫REFRACTORY({rc}) "
            if mp >= th:
                status += "⚠️ ABOVE_THRESHOLD "
                
            print(f"  Neuron {neuron_id}: MP={mp:.2f} RC={rc} TH={th:.2f} {status}")
        
        # Analysis
        print(f"🔥 Fired neurons: {fired_neurons}")
        print(f"🚫 Refractory neurons: {refractory_neurons}")
        
        # BUG DETECTION
        if round_num == 0 and len(fired_neurons) > 1:
            print("🚨 BUG DETECTED: Multiple neurons fired when only one was stimulated!")
            
        if round_num > 0 and len(refractory_neurons) > 1:
            print("🚨 BUG DETECTED: Multiple neurons in refractory when only one should be!")
            print(f"   Expected: Only neuron {target_neuron} should be refractory")
            print(f"   Actual: {len(refractory_neurons)} neurons are refractory: {refractory_neurons}")
            
            # This is the smoking gun for area-wide suppression
            non_target_refractory = [n for n in refractory_neurons if n != target_neuron]
            if non_target_refractory:
                print(f"🎯 ROOT CAUSE: Non-target neurons {non_target_refractory} are incorrectly refractory!")
                return "AREA_WIDE_SUPPRESSION_CONFIRMED"
        
        if round_num > 10 and not refractory_neurons:
            print("✅ Refractory period completed for all neurons")
            break
    
    return "MONITORING_COMPLETE"

def cleanup_test_area(cortical_id: str):
    """Clean up the test area."""
    if cortical_id:
        result = api_request("DELETE", f"/cortical_area/{cortical_id}")
        if result:
            print(f"🧹 Cleaned up test area {cortical_id}")
        else:
            print(f"⚠️  Could not clean up test area {cortical_id}")

def run_diagnostic():
    """Run the complete refractory period diagnostic."""
    print("🔬 FEAGI Refractory Period Diagnostic")
    print("=" * 50)
    
    # Step 1: Ensure genome is loaded
    if not ensure_genome_loaded():
        return
    
    # Step 2: Create test area
    cortical_id = create_test_area()
    if not cortical_id:
        return
    
    try:
        # Step 3: Wait for area to be fully created
        print("⏳ Waiting for area creation to complete...")
        time.sleep(2)
        
        # Step 4: Monitor activity
        result = monitor_area_activity(cortical_id)
        
        # Step 5: Report results
        print("\n" + "=" * 50)
        print("🏁 DIAGNOSTIC RESULTS")
        print("=" * 50)
        
        if result == "AREA_WIDE_SUPPRESSION_CONFIRMED":
            print("🚨 BUG CONFIRMED: Area-wide refractory suppression detected!")
            print("💡 ISSUE: When one neuron fires, ALL neurons in the area become refractory")
            print("🔧 NEXT STEPS: Check NeuronArray indexing and memory layout")
        elif result == "MONITORING_COMPLETE":
            print("✅ No area-wide suppression detected")
            print("🤔 ISSUE: Might be visualization-specific or timing-related")
        else:
            print("❓ Diagnostic incomplete - check logs above")
            
    finally:
        # Always cleanup
        cleanup_test_area(cortical_id)

if __name__ == "__main__":
    run_diagnostic() 