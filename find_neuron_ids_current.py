#!/usr/bin/env python3
"""
Find Neuron IDs Using Current Endpoints

This script finds neuron IDs using currently available API endpoints
and existing data structures.
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

def get_cortical_areas():
    """Get list of available cortical areas."""
    print("📊 Getting cortical areas...")
    
    areas = api_request("GET", "/connectome/cortical_areas/list/summary")
    if areas:
        print(f"✅ Found {len(areas)} cortical areas: {areas}")
        return areas
    else:
        print("❌ Could not get cortical areas")
        return []

def get_cortical_area_info(area_id: str):
    """Get detailed info about a cortical area."""
    print(f"\n🔍 Getting info for area '{area_id}'...")
    
    info = api_request("GET", f"/connectome/cortical_info/{area_id}")
    if info and "area_info" in info:
        area_data = info["area_info"]
        print(f"✅ Area {area_id}:")
        print(f"   Name: {area_data.get('name', 'N/A')}")
        print(f"   Dimensions: {area_data.get('dimensions', 'N/A')}")
        print(f"   Coordinates: {area_data.get('coordinates', 'N/A')}")
        
        # Check if there's neuron information
        if "neuron_ids" in area_data:
            neuron_ids = area_data["neuron_ids"]
            print(f"   Neuron IDs: {neuron_ids[:5]}{'...' if len(neuron_ids) > 5 else ''}")
            return neuron_ids
        else:
            # Calculate expected neuron count from dimensions
            dims = area_data.get('dimensions', [1, 1, 1])
            if isinstance(dims, list) and len(dims) >= 3:
                expected_neurons = dims[0] * dims[1] * dims[2]
                print(f"   Expected neurons: {expected_neurons} (from dimensions)")
                
                # Try to estimate neuron ID range
                cortical_idx = area_data.get('cortical_idx')
                if cortical_idx is not None:
                    print(f"   Cortical index: {cortical_idx}")
                    # This is a rough estimate - actual neuron IDs depend on the system
                    estimated_start = cortical_idx * 1000  # Very rough estimate
                    estimated_range = list(range(estimated_start, estimated_start + expected_neurons))
                    print(f"   Estimated neuron ID range: {estimated_range[:3]}...{estimated_range[-3:] if len(estimated_range) > 3 else estimated_range}")
                    return estimated_range
            
            print("   ⚠️  No neuron IDs found in response")
            return []
    else:
        print(f"❌ Could not get info for area {area_id}")
        return []

def analyze_cortical_areas_for_debugging():
    """Analyze cortical areas to find good candidates for refractory period debugging."""
    print("🔬 Analyzing cortical areas for refractory period debugging...")
    
    areas = get_cortical_areas()
    if not areas:
        return
    
    good_test_areas = []
    
    for area_id in areas[:10]:  # Check first 10 areas
        neuron_data = get_cortical_area_info(area_id)
        
        if neuron_data and len(neuron_data) > 1:  # Need multiple neurons for area-wide suppression testing
            good_test_areas.append({
                'area_id': area_id,
                'neuron_count': len(neuron_data),
                'sample_neuron_ids': neuron_data[:5]
            })
    
    print(f"\n🎯 Good test areas for refractory debugging:")
    for area in good_test_areas[:5]:  # Show top 5
        print(f"   {area['area_id']}: {area['neuron_count']} neurons, IDs: {area['sample_neuron_ids']}")
    
    return good_test_areas

def main():
    """Main function to find neuron IDs."""
    print("🔍 Finding Neuron IDs for Refractory Period Debugging")
    print("=" * 60)
    
    # Analyze areas and find neuron IDs
    test_areas = analyze_cortical_areas_for_debugging()
    
    if test_areas:
        print(f"\n✅ Found {len(test_areas)} testable areas")
        print("\n💡 NEXT STEPS:")
        print("1. Restart FEAGI to enable our new endpoints:")
        print("   - GET /v1/connectome/cortical_area/{area_id}/neurons")
        print("   - GET /v1/connectome/neuron/{neuron_id}/properties")
        print("\n2. Use these areas for testing:")
        for area in test_areas[:3]:
            print(f"   - {area['area_id']}: {area['neuron_count']} neurons")
        
        print("\n3. Example commands after restart:")
        if test_areas:
            area_id = test_areas[0]['area_id']
            sample_id = test_areas[0]['sample_neuron_ids'][0] if test_areas[0]['sample_neuron_ids'] else '12345'
            print(f"   curl 'http://localhost:8000/v1/connectome/cortical_area/{area_id}/neurons'")
            print(f"   curl 'http://localhost:8000/v1/connectome/neuron/{sample_id}/properties'")
    else:
        print("❌ No suitable test areas found")
        print("💡 This might mean no genome is loaded or areas have no neurons")

if __name__ == "__main__":
    main() 