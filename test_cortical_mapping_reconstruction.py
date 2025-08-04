#!/usr/bin/env python3
"""
Test script to verify cortical mapping reconstruction after dimension changes.

This test creates two connected cortical areas, changes one's dimensions,
and verifies that the mappings are properly reconstructed.
"""

import sys
import os
import requests
import json
import time

# Configuration
FEAGI_API_BASE = "http://localhost:8000/v1"
TEST_TIMEOUT = 30  # seconds

def api_request(method, endpoint, data=None):
    """Make API request to FEAGI"""
    url = f"{FEAGI_API_BASE}{endpoint}"
    try:
        if method == "GET":
            response = requests.get(url, timeout=TEST_TIMEOUT)
        elif method == "POST":
            response = requests.post(url, json=data, timeout=TEST_TIMEOUT)
        elif method == "PUT":
            response = requests.put(url, json=data, timeout=TEST_TIMEOUT)
        elif method == "DELETE":
            response = requests.delete(url, timeout=TEST_TIMEOUT)
        
        if response.status_code in [200, 201]:
            return response.json() if response.content else {}
        else:
            print(f"❌ API Error {response.status_code}: {response.text}")
            return None
    except Exception as e:
        print(f"❌ Request failed: {e}")
        return None

def test_cortical_mapping_reconstruction():
    """Test that dimension changes trigger proper mapping reconstruction"""
    
    print("🧪 Testing Cortical Mapping Reconstruction After Dimension Changes")
    print("=" * 70)
    
    # Step 1: Create test cortical areas
    print("\n📍 Step 1: Verifying core cortical areas exist...")
    
    # Verify core areas exist
    areas_list = api_request("GET", "/connectome/cortical_areas/list/summary")
    if not areas_list:
        print("❌ Failed to get cortical areas")
        return False
    
    if "_death" not in areas_list or "___pwr" not in areas_list:
        print("❌ Core areas (_death, ___pwr) not found in loaded genome")
        return False
        
    print(f"✅ Using existing core area: _death")
    print(f"✅ Using existing core area: ___pwr")
    
    # Step 2: Create mapping between areas
    print("\n🔗 Step 2: Creating cortical mapping...")
    
    mapping_data = {
        "src_cortical_area": "_death",
        "dst_cortical_area": "___pwr",
        "mapping_string": [
            {
                "morphology_id": "default_excitatory",
                "morphology_scalar": [1, 1, 1],
                "postSynapticCurrent_multiplier": 1.0,
                "plasticity_flag": False,
                "plasticity_constant": 0.0,
                "ltp_multiplier": 1.0,
                "ltd_multiplier": 1.0
            }
        ]
    }
    
    mapping_result = api_request("PUT", "/cortical_mapping/mapping_properties", mapping_data)
    if not mapping_result:
        print("❌ Failed to create cortical mapping")
        return False
        
    print("✅ Created mapping: _death → ___pwr")
    
    # Step 3: Get initial mapping count using specific mapping properties endpoint
    print("\n📊 Step 3: Getting initial mapping state...")
    
    initial_mapping_data = api_request("POST", "/cortical_mapping/mapping_properties", {
        "src_cortical_area": "_death",
        "dst_cortical_area": "___pwr"
    })
    
    if initial_mapping_data is None:
        print("❌ Failed to get initial mapping properties")
        return False
        
    initial_mapping_count = len(initial_mapping_data) if initial_mapping_data else 0
    print(f"📊 Initial mapping count (_death → ___pwr): {initial_mapping_count}")
    print(f"📊 Initial mapping data: {initial_mapping_data}")
    
    # Step 4: Change dimensions of area B (should trigger reconstruction)
    print("\n🔧 Step 4: Changing dimensions of ___pwr...")
    
    new_dimensions = [4, 4, 1]  # Changed to 4x4x1
    dimension_update = {
        "cortical_dimensions": new_dimensions
    }
    
    print(f"🔄 Updating ___pwr dimensions to: {new_dimensions}")
    
    # Make the dimension change
    update_result = api_request("PUT", "/cortical_area/cortical_area", {
        "cortical_id": "___pwr",
        "cortical_dimensions": new_dimensions
    })
    
    if not update_result:
        print("❌ Failed to update cortical area dimensions")
        return False
        
    print("✅ Dimension update API call successful")
    
    # Step 5: Wait for reconstruction and verify mappings
    print("\n⏰ Step 5: Waiting for reconstruction...")
    time.sleep(2)  # Give time for reconstruction
    
    # Get final mapping state using specific mapping properties endpoint
    final_mapping_data = api_request("POST", "/cortical_mapping/mapping_properties", {
        "src_cortical_area": "_death",
        "dst_cortical_area": "___pwr"
    })
    
    if final_mapping_data is None:
        print("❌ Failed to get post-update mapping properties")
        return False
        
    final_mapping_count = len(final_mapping_data) if final_mapping_data else 0
    print(f"📊 Final mapping count (_death → ___pwr): {final_mapping_count}")
    print(f"📊 Final mapping data: {final_mapping_data}")
    
    # Step 6: Verify reconstruction happened
    print("\n✅ Step 6: Verifying reconstruction...")
    
    if final_mapping_count > 0:
        print("✅ SUCCESS: Cortical mappings exist after dimension change")
        print(f"📈 Mapping count: {initial_mapping_count} → {final_mapping_count}")
        
        # Check if reconstruction actually happened (mapping count should change due to new dimensions)
        if final_mapping_count != initial_mapping_count:
            print("✅ EXCELLENT: Mapping count changed, indicating proper reconstruction")
        else:
            print("⚠️  WARNING: Mapping count unchanged - reconstruction may not have occurred")
            
        return True
    else:
        print("❌ FAILURE: No cortical mappings found after dimension change!")
        print("🔧 This confirms the bug - mappings were not reconstructed")
        return False
    
    # Cleanup
    print("\n🧹 Cleanup: Removing test areas...")
    api_request("DELETE", "/cortical_area/cortical_area/test_a")
    api_request("DELETE", "/cortical_area/cortical_area/test_b")

if __name__ == "__main__":
    try:
        success = test_cortical_mapping_reconstruction()
        if success:
            print("\n🎉 Test completed successfully!")
            sys.exit(0)
        else:
            print("\n💥 Test failed - bug confirmed!")
            sys.exit(1)
    except KeyboardInterrupt:
        print("\n⏸️  Test interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n💥 Test crashed: {e}")
        sys.exit(1) 