#!/usr/bin/env python3
"""
Complete Memory System Test Script

This script tests the entire memory system workflow with comprehensive debugging.
It will help diagnose and verify that memory processing is working correctly.

Usage:
1. Start FEAGI: python3 feagi/main.py --debug-npu --log-level DEBUG
2. Wait for it to fully initialize
3. Run: python3 test_memory_system_complete.py

The script will:
- Check if memory processor is initialized
- Create upstream and memory areas
- Create memory mapping
- Trigger neuron activation
- Verify memory processing logs appear
"""

import sys
import time
import requests
import json
from typing import Dict, Any, List

# FEAGI API base URL
BASE_URL = "http://127.0.0.1:8000"

def make_api_call(method: str, endpoint: str, payload: Dict = None) -> Dict[str, Any]:
    """Make API call with error handling."""
    url = f"{BASE_URL}{endpoint}"
    
    try:
        if method == "GET":
            response = requests.get(url)
        elif method == "POST":
            response = requests.post(url, json=payload)
        elif method == "PUT":
            response = requests.put(url, json=payload)
        else:
            raise ValueError(f"Unsupported method: {method}")
            
        print(f"API {method} {endpoint}: {response.status_code}")
        
        if response.status_code == 200:
            return {"success": True, "data": response.json()}
        else:
            return {"success": False, "error": f"{response.status_code}: {response.text}"}
            
    except Exception as e:
        return {"success": False, "error": str(e)}

def check_feagi_health() -> bool:
    """Check if FEAGI is running and responsive."""
    print("🏥 Checking FEAGI health...")
    result = make_api_call("GET", "/v1/system/health_check")
    
    if result["success"]:
        print("✅ FEAGI is healthy and running")
        return True
    else:
        print(f"❌ FEAGI health check failed: {result['error']}")
        return False

def create_upstream_area() -> str:
    """Create upstream cortical area."""
    print("\n🧠 Creating upstream cortical area...")
    
    payload = {
        "brain_region_id": "root",
        "coordinates_2d": [20, 20],
        "coordinates_3d": [20, 20, 0],
        "cortical_dimensions": [3, 3, 1],  # 9 neurons
        "cortical_group": "CUSTOM",
        "cortical_name": "TestUpstream",
        "cortical_sub_group": "TEST",
        "per_voxel_neuron_cnt": 1,
        "visualization": True
    }
    
    result = make_api_call("POST", "/v1/cortical_area/custom_cortical_area", payload)
    
    if result["success"]:
        area_id = result["data"].get("cortical_id")
        print(f"✅ Created upstream area: {area_id}")
        return area_id
    else:
        print(f"❌ Failed to create upstream area: {result['error']}")
        return None

def create_memory_area() -> str:
    """Create memory cortical area."""
    print("\n🧠 Creating memory cortical area...")
    
    payload = {
        "brain_region_id": "root",
        "coordinates_2d": [60, 60],
        "coordinates_3d": [60, 60, 0],
        "cortical_dimensions": [2, 2, 1],  # 4 neurons
        "cortical_group": "CUSTOM",
        "cortical_name": "TestMemory",
        "cortical_sub_group": "MEMORY",
        "sub_group_id": "MEMORY",  # This makes it a memory area
        "temporal_depth": 3,  # Look back 3 timesteps
        "init_lifespan": 20,
        "lifespan_growth_rate": 1.1,
        "longterm_mem_threshold": 100,
        "per_voxel_neuron_cnt": 1,
        "visualization": True
    }
    
    result = make_api_call("POST", "/v1/cortical_area/custom_cortical_area", payload)
    
    if result["success"]:
        area_id = result["data"].get("cortical_id")
        print(f"✅ Created memory area: {area_id}")
        return area_id
    else:
        print(f"❌ Failed to create memory area: {result['error']}")
        return None

def create_memory_mapping(upstream_id: str, memory_id: str) -> bool:
    """Create memory mapping between areas."""
    print(f"\n🔗 Creating memory mapping: {upstream_id} -> {memory_id}")
    
    payload = {
        upstream_id: {
            memory_id: [{
                "morphology_id": "memory",
                "morphology_scalar": [1, 1, 1],
                "postSynapticCurrent_multiplier": 1.0,
                "plasticity_flag": False,
                "plasticity_constant": 0.0,
                "ltp_multiplier": 1.0,
                "ltd_multiplier": 1.0
            }]
        }
    }
    
    result = make_api_call("PUT", "/v1/cortical_mapping/mapping", payload)
    
    if result["success"]:
        print("✅ Memory mapping created successfully")
        return True
    else:
        print(f"❌ Failed to create memory mapping: {result['error']}")
        return False

def activate_neurons(area_id: str) -> bool:
    """Activate neurons in the area."""
    print(f"\n⚡ Activating neurons in {area_id}")
    
    payload = {
        "cortical_id": area_id,
        "coordinates": [
            {"x": 0, "y": 0, "z": 0},
            {"x": 1, "y": 0, "z": 0},
            {"x": 2, "y": 0, "z": 0}
        ],
        "intensity": 1.0
    }
    
    result = make_api_call("POST", "/v1/agent/manual_stimulation", payload)
    
    if result["success"]:
        print(f"✅ Neurons activated in {area_id}")
        return True
    else:
        print(f"❌ Failed to activate neurons: {result['error']}")
        return False

def verify_memory_areas(upstream_id: str, memory_id: str) -> bool:
    """Verify both areas have correct properties."""
    print(f"\n🔍 Verifying area properties...")
    
    # Check upstream area
    result = make_api_call("GET", f"/v1/cortical_area/{upstream_id}/properties")
    if result["success"]:
        print(f"✅ Upstream area {upstream_id} verified")
    else:
        print(f"❌ Failed to verify upstream area: {result['error']}")
        return False
    
    # Check memory area - particularly important for memory properties
    payload = {"cortical_id_list": [memory_id]}
    result = make_api_call("POST", "/v1/cortical_area/multi/cortical_area_properties", payload)
    
    if result["success"]:
        properties = result["data"].get("cortical_properties", {}).get(memory_id, {})
        temporal_depth = properties.get("temporal_depth", 0)
        init_lifespan = properties.get("init_lifespan", 0)
        
        if temporal_depth > 0 and init_lifespan > 0:
            print(f"✅ Memory area {memory_id} verified: temporal_depth={temporal_depth}, init_lifespan={init_lifespan}")
            return True
        else:
            print(f"❌ Memory area {memory_id} has invalid properties: temporal_depth={temporal_depth}, init_lifespan={init_lifespan}")
            return False
    else:
        print(f"❌ Failed to verify memory area: {result['error']}")
        return False

def main():
    """Main test workflow."""
    print("🧠 FEAGI Memory System Complete Test")
    print("=" * 50)
    
    # Step 1: Health check
    if not check_feagi_health():
        print("\n❌ FEAGI is not running. Please start FEAGI first:")
        print("   python3 feagi/main.py --debug-npu --log-level DEBUG")
        return False
    
    # Step 2: Create areas
    upstream_id = create_upstream_area()
    if not upstream_id:
        return False
        
    memory_id = create_memory_area()
    if not memory_id:
        return False
    
    # Step 3: Verify areas have correct properties
    if not verify_memory_areas(upstream_id, memory_id):
        return False
    
    # Step 4: Create memory mapping
    if not create_memory_mapping(upstream_id, memory_id):
        return False
    
    # Step 5: Activate neurons and monitor
    print("\n🔥 Starting neuron activation sequence...")
    print("Watch your FEAGI terminal for these debug logs:")
    print("   [MEMORY-INIT] Starting MemoryProcessor initialization...")
    print("   [NPU-DEBUG] BURST ENGINE: Processing memory areas for temporal patterns")
    print("   [NPU-DEBUG] Active memory areas: [...]")
    print("   [NPU-DEBUG] MEMORY PROCESSING START: Burst X")
    
    # Activate neurons multiple times to create temporal patterns
    for i in range(5):
        print(f"\nActivation {i+1}/5...")
        if activate_neurons(upstream_id):
            time.sleep(1)  # Wait for burst processing
        else:
            print("❌ Activation failed")
            return False
    
    # Final summary
    print("\n" + "=" * 50)
    print("✅ Memory System Test Complete!")
    print("\nExpected logs in FEAGI terminal:")
    print("1. [MEMORY-INIT] logs during startup")
    print("2. [NPU-DEBUG] BURST ENGINE logs during activations")
    print("3. [NPU-DEBUG] MEMORY PROCESSING logs during bursts")
    print("\nIf you don't see these logs, check:")
    print("- FEAGI started with --debug-npu flag")
    print("- Memory processor initialized successfully")
    print("- Memory areas registered with processor")
    
    return True

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1) 