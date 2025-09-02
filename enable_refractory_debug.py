#!/usr/bin/env python3
"""
Enable Refractory Period Debug Logging

This script enables the low-level debug logging in NeuronArray to help
identify the refractory period area-wide suppression bug.
"""

import requests
import json
import time
from typing import Dict, Any

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

def enable_debug_logging():
    """Enable debug logging for refractory periods."""
    print("🔬 Enabling refractory period debug logging...")
    
    # Try to enable debug via API (this might not exist)
    result = api_request("POST", "/debug/enable_refractory_logging", {})
    if result:
        print("✅ Debug logging enabled via API")
        return True
    
    print("⚠️  API method not available - debug logging must be enabled in code")
    print("💡 The debug logging has been added to NeuronArray.embedded_optimized_neural_update()")
    print("   It will show output when neurons fire if the flag is enabled")
    return False

def stimulate_area(area_id: str, intensity: float = 1.0):
    """Stimulate a cortical area to trigger neural activity."""
    print(f"⚡ Stimulating area {area_id} with intensity {intensity}...")
    
    # Method 1: Try power injection to specific area
    power_data = {
        "area_id": area_id,
        "power_level": intensity
    }
    
    result = api_request("POST", "/power/inject_to_area", power_data)
    if result:
        print(f"✅ Power injected to area {area_id}")
        return True
    
    # Method 2: Try general stimulation
    stim_data = {
        "cortical_area": area_id,
        "intensity": intensity
    }
    
    result = api_request("POST", "/stimulation/area_stimulation", stim_data)
    if result:
        print(f"✅ Stimulated area {area_id}")
        return True
    
    print(f"❌ Could not stimulate area {area_id}")
    return False

def trigger_activity_for_debug():
    """Trigger neural activity to see debug output in FEAGI logs."""
    print("🎯 Triggering neural activity to observe refractory period behavior...")
    print("   (Watch FEAGI console output for debug messages)")
    
    # Get list of available areas
    areas = api_request("GET", "/connectome/cortical_areas/list/summary")
    if not areas:
        print("❌ Could not get area list")
        return
    
    print(f"📊 Available areas: {areas}")
    
    # Try to stimulate some areas that likely have multiple neurons
    test_areas = ["m__rig", "iic400", "iic200", "CIHMot"]
    
    for area_id in test_areas:
        if area_id in areas:
            print(f"\n🧪 Testing area: {area_id}")
            stimulate_area(area_id, 0.8)
            
            print("⏳ Waiting for neural processing...")
            time.sleep(1)  # Wait for several burst cycles
            
            print("✅ Check FEAGI console for debug output!")
            print("   Look for lines starting with '🔥 NEURON DEBUG:'")
            print("   If you see '🚨 BUG DETECTED:', the area-wide suppression is confirmed!")
            
            break
    else:
        print("⚠️  None of the preferred test areas found")
        # Try any area that's not core
        for area_id in areas:
            if area_id not in ["_death", "_power"]:
                print(f"\n🧪 Testing available area: {area_id}")
                stimulate_area(area_id, 0.8)
                time.sleep(1)
                break

def main():
    """Main function to enable debug and trigger activity."""
    print("🔬 FEAGI Refractory Period Debug Session")
    print("=" * 50)
    
    print("\n💡 INSTRUCTIONS:")
    print("1. This script will try to trigger neural activity")
    print("2. Debug logging has been added to the NeuronArray code")
    print("3. Watch the FEAGI console output for debug messages")
    print("4. Look for '🚨 BUG DETECTED:' messages")
    print("\n🔍 If you see area-wide refractory suppression, we'll know the exact cause!")
    
    # Enable debug logging (if API exists)
    enable_debug_logging()
    
    # Trigger some activity
    trigger_activity_for_debug()
    
    print("\n" + "=" * 50)
    print("🏁 DEBUG SESSION COMPLETE")
    print("=" * 50)
    print("📊 WHAT TO LOOK FOR IN FEAGI LOGS:")
    print("  🔥 NEURON DEBUG: - Shows when neurons fire")
    print("  🧪 Area X: Y fired, Z didn't fire - Shows area breakdown")
    print("  🚨 BUG DETECTED: - Confirms area-wide suppression")
    print("  🎯 ROOT CAUSE: - Identifies if it's shared memory or indexing bug")
    print("\n💡 If no debug output appears:")
    print("  - The _debug_refractory flag might not be enabled")
    print("  - Neural activity might not be triggering")
    print("  - Check if FEAGI is processing bursts")

if __name__ == "__main__":
    main() 