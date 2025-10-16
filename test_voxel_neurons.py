#!/usr/bin/env python3
"""
Test script for the new /v1/cortical_area/voxel_neurons endpoint.

This script tests querying neurons at specific voxel coordinates.
"""

import requests
import json

# FEAGI REST API endpoint
FEAGI_URL = "http://127.0.0.1:8000"

def test_voxel_neurons(cortical_id: str, x: int, y: int, z: int):
    """Test the voxel_neurons endpoint."""
    url = f"{FEAGI_URL}/v1/cortical_area/voxel_neurons"
    
    payload = {
        "cortical_id": cortical_id,
        "x": x,
        "y": y,
        "z": z
    }
    
    print(f"\n{'='*60}")
    print(f"Testing voxel lookup: {cortical_id}[{x},{y},{z}]")
    print(f"{'='*60}")
    print(f"Request URL: {url}")
    print(f"Request payload: {json.dumps(payload, indent=2)}")
    
    try:
        response = requests.post(url, json=payload)
        print(f"\nResponse status: {response.status_code}")
        
        if response.status_code == 200:
            data = response.json()
            print(f"Response data: {json.dumps(data, indent=2)}")
            
            neuron_count = data.get('neuron_count', 0)
            neuron_ids = data.get('neuron_ids', [])
            
            if neuron_count > 0:
                print(f"\n✅ SUCCESS: Found {neuron_count} neuron(s) at this location")
                print(f"   Neuron IDs: {neuron_ids}")
            else:
                print(f"\n⚠️  No neurons found at this location (empty voxel)")
        else:
            print(f"❌ ERROR: {response.text}")
            
    except Exception as e:
        print(f"❌ Exception: {e}")

def main():
    """Run tests for multiple voxel locations."""
    
    # Test coordinates from the video agent logs
    test_cases = [
        # iic100 (bot-mid vision) - 16x16x3
        ("iic100", 0, 1, 0),
        ("iic100", 0, 0, 0),
        ("iic100", 8, 8, 1),
        ("iic100", 15, 15, 2),
        
        # iic400 (center vision) - 128x128x3
        ("iic400", 125, 4, 0),
        ("iic400", 0, 0, 0),
        ("iic400", 64, 64, 1),
        
        # iic000 (bot-left vision) - 16x16x3
        ("iic000", 0, 0, 0),
        ("iic000", 8, 8, 1),
    ]
    
    print("Testing /v1/cortical_area/voxel_neurons endpoint")
    print("=" * 60)
    
    for cortical_id, x, y, z in test_cases:
        test_voxel_neurons(cortical_id, x, y, z)
    
    print(f"\n{'='*60}")
    print("Test complete!")
    print(f"{'='*60}")

if __name__ == "__main__":
    main()

