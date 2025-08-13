#!/usr/bin/env python3
"""
Quick script to check genome loading status
"""

import sys
import requests

BASE_URL = "http://127.0.0.1:8000"

def check_genome_status():
    try:
        # Check health
        health = requests.get(f"{BASE_URL}/v1/system/health_check").json()
        print(f"Neuron count: {health.get('neuron_count', 'N/A')}")
        print(f"Synapse count: {health.get('synapse_count', 'N/A')}")
        
        # Try to get genome info via cortical areas list
        try:
            areas_response = requests.get(f"{BASE_URL}/v1/cortical_area/list")
            print(f"Cortical areas list status: {areas_response.status_code}")
            if areas_response.status_code == 200:
                areas = areas_response.json().get("cortical_areas", [])
                print(f"Number of cortical areas: {len(areas)}")
                if areas:
                    print(f"Sample areas: {areas[:5]}")
            else:
                print(f"Areas list error: {areas_response.text}")
        except Exception as e:
            print(f"Areas list failed: {e}")
            
        # Check if we can get cortical mapping
        try:
            mapping_response = requests.get(f"{BASE_URL}/v1/cortical_mapping/mapping")
            print(f"Cortical mapping status: {mapping_response.status_code}")
        except Exception as e:
            print(f"Mapping check failed: {e}")
            
        # Try to get brain regions
        try:
            regions_response = requests.get(f"{BASE_URL}/v1/brain_region/brain_regions")
            print(f"Brain regions status: {regions_response.status_code}")
            if regions_response.status_code == 200:
                regions = regions_response.json()
                print(f"Brain regions found: {len(regions) if isinstance(regions, list) else 'dict/other'}")
        except Exception as e:
            print(f"Brain regions check failed: {e}")
            
    except Exception as e:
        print(f"Health check failed: {e}")

if __name__ == "__main__":
    check_genome_status() 