#!/usr/bin/env python3
"""
Debug Cortical Area Mapping

This script debugs why cortical area mapping is failing and tries 
alternative approaches to get neuron IDs.
"""

import requests
import json
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
        else:
            raise ValueError(f"Unsupported method: {method}")
            
        if response.status_code == 200:
            return response.json()
        else:
            print(f"❌ API Error {response.status_code}: {response.text[:200]}")
            return None
    except Exception as e:
        print(f"❌ Request failed: {e}")
        return None

def test_cortical_info_endpoint():
    """Test the cortical info endpoint for various areas."""
    print("🔍 Testing cortical info endpoint for different areas...")
    
    areas_to_test = ['iv00MR', 'iv00_C', 'm__rig', '___pwr']
    
    for area_id in areas_to_test:
        print(f"\n📊 Testing {area_id}:")
        info = api_request("GET", f"/connectome/cortical_info/{area_id}")
        
        if info and "area_info" in info:
            area_data = info["area_info"]
            print(f"  ✅ Found: {area_data.get('name', 'N/A')}")
            print(f"     Dimensions: {area_data.get('dimensions', 'N/A')}")
            print(f"     Cortical idx: {area_data.get('cortical_idx', 'N/A')}")
            
            # Check if this area has neuron data embedded
            if "neuron_ids" in area_data:
                print(f"     Neuron IDs: {area_data['neuron_ids'][:5]}...")
            elif "neurons" in area_data:
                print(f"     Neurons: {len(area_data['neurons'])} found")
            else:
                print("     No direct neuron data in response")
        else:
            print(f"  ❌ Failed to get info for {area_id}")

def test_alternative_neuron_access():
    """Try alternative ways to access neuron data."""
    print("\n🧪 Testing alternative neuron access methods...")
    
    # Method 1: Try the cortical area geometry endpoint
    print("\n1️⃣ Testing cortical area geometry endpoint:")
    geometry = api_request("GET", "/cortical_area/cortical_area/geometry")
    if geometry:
        print("✅ Geometry endpoint works")
        for area_id, data in list(geometry.items())[:3]:
            print(f"   {area_id}: {data.get('neuron_count', 'N/A')} neurons")
    else:
        print("❌ Geometry endpoint failed")
    
    # Method 2: Try using estimated neuron IDs based on our earlier analysis
    print("\n2️⃣ Testing estimated neuron IDs:")
    test_ranges = [
        ('iv00MR', [4000, 4001, 4002, 4003, 4004]),
        ('iv00_C', [6000, 6001, 6002, 6003, 6004]),
        ('m__rig', [7000, 7001, 7002]),  # Might be wrong estimate
    ]
    
    for area_id, neuron_ids in test_ranges:
        print(f"\n   Testing {area_id} with estimated IDs:")
        working_ids = []
        
        for neuron_id in neuron_ids:
            # Try our new neuron properties endpoint
            result = api_request("GET", f"/connectome/neuron/{neuron_id}/properties")
            if result and result.get("neuron_id"):
                working_ids.append(neuron_id)
                area = result.get("cortical_id", "N/A")
                mp = result.get("membrane_potential", "N/A")
                rc = result.get("refractory_counter", "N/A")
                print(f"     ✅ Neuron {neuron_id} in area {area}: MP={mp}, RC={rc}")
                
                if len(working_ids) >= 2:  # Found enough for testing
                    break
            else:
                print(f"     ❌ Neuron {neuron_id} not found")
        
        if working_ids:
            print(f"   🎯 Found working neuron IDs for {area_id}: {working_ids}")

def try_stimulation_endpoints():
    """Try to find working stimulation endpoints."""
    print("\n⚡ Testing stimulation endpoints...")
    
    stimulation_tests = [
        {"endpoint": "/simulation/stimulation_string", "data": {"stimulation_script": {"iv00MR": [1.0]}}},
        {"endpoint": "/burst_engine/simulation_timestep", "data": {"simulation_timestep": 1.0}},
    ]
    
    for test in stimulation_tests:
        print(f"\n🎯 Testing {test['endpoint']}:")
        result = api_request("POST", test["endpoint"], test["data"])
        if result:
            print(f"   ✅ Success: {result}")
        else:
            print(f"   ❌ Failed")

def main():
    """Main debug function."""
    print("🔬 Debug Cortical Area Mapping and Neuron Access")
    print("=" * 60)
    
    # Test cortical info
    test_cortical_info_endpoint()
    
    # Try alternative access methods
    test_alternative_neuron_access()
    
    # Try stimulation
    try_stimulation_endpoints()
    
    print("\n" + "=" * 60)
    print("🏁 DEBUG COMPLETE")
    print("\n💡 FINDINGS:")
    print("- If neuron properties endpoint worked, use those IDs for testing")
    print("- If cortical area neurons endpoint failed, there's a mapping issue")
    print("- The debug logging in NeuronArray will still work when neurons fire")
    print("\n🎯 IMMEDIATE ACTION:")
    print("- Use your visualization tool to stimulate any area with multiple neurons")
    print("- Watch FEAGI console for '🔥 NEURON DEBUG:' messages")
    print("- The area-wide suppression will be detected automatically")

if __name__ == "__main__":
    main() 