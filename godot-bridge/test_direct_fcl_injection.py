"""
Test script for direct FCL injection into FEAGI.

This script uses raw ZMQ to send data directly to FEAGI's FCL manager,
bypassing the normal sensory input pathway. This can be useful for testing
and debugging the visualization data flow.
"""

import json
import time
import zmq
import random
import requests
from typing import List, Dict, Tuple, Optional

# FEAGI ZMQ and API configuration
FEAGI_HOST = "127.0.0.1"
FEAGI_API_PORT = 8000
SENSORIMOTOR_PORT = 5558  # Default port for FEAGI sensorimotor data
CORTICAL_AREA = "iv00CC"  # Replace with a valid cortical area in your genome

# Pattern configuration
PATTERN_SIZE = (10, 10, 1)  # 3D size of the cortical area
NUM_ACTIVE_NEURONS = 16  # Number of neurons to activate in each pattern


def check_feagi_status() -> bool:
    """Check if FEAGI is running and has a valid genome."""
    try:
        # Check FEAGI status
        status_url = f"http://{FEAGI_HOST}:{FEAGI_API_PORT}/api/v1/status"
        response = requests.get(status_url, timeout=5)
        
        if response.status_code != 200:
            print(f"Error: FEAGI API returned status code {response.status_code}")
            return False
        
        status_data = response.json()
        
        # Check if FEAGI has a genome loaded
        if not status_data.get('genome_availability', False):
            print("Error: No genome loaded in FEAGI")
            return False

        if not status_data.get('genome_validity', False):
            print("Error: Genome in FEAGI is not valid")
            return False
            
        if not status_data.get('brain_readiness', False):
            print("Error: Brain is not ready in FEAGI")
            return False
        
        print("FEAGI is running with a valid genome")
        return True
    
    except requests.exceptions.RequestException as e:
        print(f"Error connecting to FEAGI API: {e}")
        return False


def check_cortical_area_exists(cortical_area: str) -> bool:
    """Check if the specified cortical area exists in the FEAGI genome."""
    try:
        # Get the genome blueprint
        blueprint_url = f"http://{FEAGI_HOST}:{FEAGI_API_PORT}/api/v1/genome/blueprint"
        response = requests.get(blueprint_url, timeout=5)
        
        if response.status_code != 200:
            print(f"Error: Failed to get genome blueprint. Status code: {response.status_code}")
            return False
        
        blueprint = response.json()
        
        # Check if the cortical area exists
        if cortical_area not in blueprint:
            print(f"Error: Cortical area '{cortical_area}' not found in genome")
            return False
        
        # Get the dimensions of the cortical area
        dimensions = blueprint.get(cortical_area, {}).get('coordinates', {}).get('dimension', [])
        if dimensions:
            global PATTERN_SIZE
            PATTERN_SIZE = tuple(dimensions)
            print(f"Found cortical area '{cortical_area}' with dimensions {PATTERN_SIZE}")
            return True
        else:
            print(f"Error: Could not find dimensions for cortical area '{cortical_area}'")
            return False
            
    except requests.exceptions.RequestException as e:
        print(f"Error connecting to FEAGI API: {e}")
        return False
    except (KeyError, ValueError, TypeError) as e:
        print(f"Error parsing genome blueprint: {e}")
        return False


def create_direct_fcl_data() -> List[Tuple[int, int, int]]:
    """Create random FCL data for testing."""
    x_max, y_max, z_max = PATTERN_SIZE
    
    # Create a square pattern that moves across the field
    fcl_data = []
    
    # Create a 4x4 square that's positioned randomly
    square_size = min(4, x_max - 1, y_max - 1)  # Ensure square fits in the area
    x_offset = random.randint(0, max(0, x_max - square_size - 1))
    y_offset = random.randint(0, max(0, y_max - square_size - 1))
    
    for dx in range(square_size):
        for dy in range(square_size):
            x = x_offset + dx
            y = y_offset + dy
            z = 0  # Usually 0 for 2D sensory areas
            fcl_data.append((x, y, z))
    
    return fcl_data


def send_fcl_data(socket, cortical_area: str, coordinates: List[Tuple[int, int, int]]):
    """Send FCL data directly to FEAGI."""
    # Format the message
    message = {
        "fcl_injection": {
            cortical_area: coordinates
        }
    }
    
    # Convert to JSON
    json_data = json.dumps(message).encode('utf-8')
    
    # Send the message
    socket.send_multipart([b"", b"application/json", json_data])
    print(f"Sent {len(coordinates)} coordinates to {cortical_area}")


def main():
    """Main test function."""
    # First check FEAGI status and cortical area
    if not check_feagi_status():
        print("FEAGI is not ready. Exiting.")
        return
        
    if not check_cortical_area_exists(CORTICAL_AREA):
        print(f"Cortical area {CORTICAL_AREA} is not available. Exiting.")
        return
    
    # Initialize ZMQ
    context = zmq.Context()
    socket = context.socket(zmq.DEALER)
    
    # Connect to FEAGI
    print(f"Connecting to FEAGI at {FEAGI_HOST}:{SENSORIMOTOR_PORT}")
    socket.connect(f"tcp://{FEAGI_HOST}:{SENSORIMOTOR_PORT}")
    
    try:
        # Send patterns in a loop
        for i in range(50):  # 50 iterations
            # Generate random FCL data
            fcl_data = create_direct_fcl_data()
            
            # Send to FEAGI
            send_fcl_data(socket, CORTICAL_AREA, fcl_data)
            
            # Wait to see the effect
            time.sleep(0.2)
            
            # Print a progress indicator every 10 iterations
            if (i + 1) % 10 == 0:
                print(f"Progress: {i + 1}/50 iterations completed")
    
    except KeyboardInterrupt:
        print("Test interrupted")
    finally:
        # Clean up
        socket.close()
        context.term()
        print("Disconnected from FEAGI")


if __name__ == "__main__":
    main() 