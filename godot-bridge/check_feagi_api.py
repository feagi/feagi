"""
Simple script to check what API endpoints are available in FEAGI 2.0.
"""

import requests
import json

FEAGI_HOST = "127.0.0.1"
FEAGI_API_PORT = 8000

def check_api_endpoint(path):
    """Check if an API endpoint exists and return its response."""
    url = f"http://{FEAGI_HOST}:{FEAGI_API_PORT}{path}"
    print(f"Checking endpoint: {url}")
    
    try:
        response = requests.get(url, timeout=5)
        status = response.status_code
        print(f"Status code: {status}")
        
        if status == 200:
            try:
                json_data = response.json()
                print("Response content (JSON):")
                print(json.dumps(json_data, indent=2))
            except ValueError:
                print("Response is not JSON. Content:")
                print(response.text[:500])  # Print first 500 chars
        else:
            print(f"Error response ({status}):")
            print(response.text[:500])  # Print first 500 chars
            
        return status == 200
    except requests.exceptions.RequestException as e:
        print(f"Connection error: {e}")
        return False

def main():
    """Check various potential API endpoints."""
    print("\n=== Checking FEAGI API Endpoints ===\n")
    
    # Try various potential endpoints
    endpoints = [
        "/",
        "/api",
        "/api/v1",
        "/api/v1/status",
        "/api/v2/status",
        "/api/status",
        "/status",
        "/api/v1/genome/blueprint",
        "/api/v2/genome/blueprint",
        "/api/genome/blueprint",
        "/genome/blueprint",
        "/api/v1/connectome"
    ]
    
    found_endpoints = 0
    for endpoint in endpoints:
        print(f"\n--- Testing: {endpoint} ---")
        if check_api_endpoint(endpoint):
            found_endpoints += 1
        print("-" * 50)
        
    print(f"\nFound {found_endpoints} working endpoints out of {len(endpoints)} tested.")
    
    if found_endpoints == 0:
        print("\nNo endpoints found. Is FEAGI running?")

if __name__ == "__main__":
    main() 