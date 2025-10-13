#!/usr/bin/env python3
"""
Test script to verify FCL endpoint shows power neuron firing.
"""

import requests
import time
import json

def test_fcl_endpoint():
    """Test that the FCL endpoint returns power neuron data."""
    base_url = "http://127.0.0.1:8000"
    
    print("=" * 80)
    print("Testing FCL Endpoint for Power Neuron Visibility")
    print("=" * 80)
    
    # Wait for FEAGI to start
    print("\n1. Waiting for FEAGI to start...")
    for i in range(30):
        try:
            response = requests.get(f"{base_url}/v1/health", timeout=1)
            if response.status_code == 200:
                print("   ✓ FEAGI is ready")
                break
        except:
            pass
        time.sleep(1)
    else:
        print("   ✗ FEAGI did not start in time")
        return False
    
    # Wait for a few bursts to complete
    print("\n2. Waiting for burst engine to process...")
    time.sleep(3)
    
    # Query FCL endpoint multiple times
    print("\n3. Querying FCL endpoint...")
    for attempt in range(5):
        try:
            response = requests.get(f"{base_url}/v1/burst_engine/fcl", timeout=2)
            
            if response.status_code == 200:
                data = response.json()
                
                print(f"\n   Attempt {attempt + 1}:")
                print(f"   Status: {response.status_code}")
                print(f"   Response keys: {list(data.keys())}")
                
                if "data" in data:
                    fcl_data = data["data"]
                    print(f"   FCL data type: {type(fcl_data)}")
                    
                    if isinstance(fcl_data, dict):
                        print(f"   Cortical areas with firing neurons: {list(fcl_data.keys())}")
                        
                        for cortical_idx, neurons in fcl_data.items():
                            if isinstance(neurons, list):
                                print(f"   Area {cortical_idx}: {len(neurons)} neurons fired")
                                if neurons:
                                    print(f"      First few neuron IDs: {neurons[:5]}")
                            elif isinstance(neurons, dict) and "neuron_ids" in neurons:
                                neuron_ids = neurons["neuron_ids"]
                                print(f"   Area {cortical_idx}: {len(neuron_ids)} neurons fired")
                                if neuron_ids:
                                    print(f"      First few neuron IDs: {neuron_ids[:5]}")
                    elif isinstance(fcl_data, list):
                        print(f"   Total neurons: {len(fcl_data)}")
                        if fcl_data:
                            print(f"   First few neuron IDs: {fcl_data[:5]}")
                
                # Check if power neuron (ID 1) is present
                if "data" in data:
                    fcl_data = data["data"]
                    found_power = False
                    
                    if isinstance(fcl_data, dict):
                        for cortical_idx, neurons in fcl_data.items():
                            if isinstance(neurons, list) and 1 in neurons:
                                found_power = True
                                print(f"\n   ✓ POWER NEURON FOUND in cortical area {cortical_idx}")
                                break
                            elif isinstance(neurons, dict) and "neuron_ids" in neurons:
                                if 1 in neurons["neuron_ids"]:
                                    found_power = True
                                    print(f"\n   ✓ POWER NEURON FOUND in cortical area {cortical_idx}")
                                    break
                    elif isinstance(fcl_data, list) and 1 in fcl_data:
                        found_power = True
                        print(f"\n   ✓ POWER NEURON FOUND in global FCL")
                    
                    if not found_power and fcl_data:
                        print("\n   ⚠ Power neuron (ID 1) not found in FCL data")
                    elif not fcl_data:
                        print("\n   ⚠ FCL data is empty")
                else:
                    print("\n   ⚠ No 'data' key in response")
                
            else:
                print(f"\n   Attempt {attempt + 1}:")
                print(f"   Status: {response.status_code}")
                print(f"   Error: {response.text[:200]}")
        
        except Exception as e:
            print(f"\n   Attempt {attempt + 1} failed: {e}")
        
        time.sleep(1)
    
    print("\n" + "=" * 80)
    print("Test Complete")
    print("=" * 80)

if __name__ == "__main__":
    test_fcl_endpoint()

