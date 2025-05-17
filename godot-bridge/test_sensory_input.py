"""
Test script to generate sensory data for FEAGI.

This script connects to FEAGI using the feagi_connector library,
generates artificial sensory data, and sends it to FEAGI for processing.
The data should then be visualized by the godot bridge.
"""

import os
import time
import asyncio
import numpy as np
import requests
from typing import List, Tuple, Dict, Any, Optional
import sys
sys.path.append("..")  # Add parent directory to path

from feagi_connector.client import FeagiClient

# FEAGI configuration
FEAGI_HOST = "127.0.0.1"
FEAGI_API_PORT = 8000
SENSORY_CORTICAL_AREA = "iv00CC"  # Replace with your sensory cortical area name
PATTERN_SIZE = (10, 10)  # Size of the sensory pattern to generate


async def check_feagi_status() -> bool:
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


async def check_cortical_area_exists(cortical_area: str) -> bool:
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
        if dimensions and len(dimensions) >= 2:
            global PATTERN_SIZE
            PATTERN_SIZE = (dimensions[0], dimensions[1])
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


async def generate_and_send_pattern(client: FeagiClient, pattern_coords: List[Tuple[int, int, int]]):
    """Generate and send a pattern of activity to FEAGI."""
    print(f"Sending {len(pattern_coords)} coordinates to {SENSORY_CORTICAL_AREA}")
    
    # Prepare data format for FEAGI
    # In a real application, you would format this according to the 
    # expected input format for your specific cortical area
    sensory_data = {
        "data": {
            "direct_stimulation": {
                SENSORY_CORTICAL_AREA: pattern_coords
            }
        }
    }

    # Convert to bytes
    sensory_bytes = str(sensory_data).encode('utf-8')
    
    # Send to FEAGI - channel 1 is typically for sensory data
    await client.send_sensory_data(1, sensory_bytes)
    print("Data sent to FEAGI")


async def generate_moving_pattern():
    """Generate a moving pattern of activity over time."""
    # Check FEAGI status and cortical area
    if not await check_feagi_status():
        print("FEAGI is not ready. Exiting.")
        return
        
    if not await check_cortical_area_exists(SENSORY_CORTICAL_AREA):
        print(f"Cortical area {SENSORY_CORTICAL_AREA} is not available. Exiting.")
        return
    
    # Create a square pattern that moves across the sensory field
    x_max, y_max = PATTERN_SIZE
    
    # Connect to FEAGI
    client = FeagiClient(
        host=FEAGI_HOST,  # Default FEAGI host
        agent_id="test_sensory_generator",
        agent_type="test"
    )
    
    # Connect and register
    connected = await client.connect()
    if not connected:
        print("Failed to connect to FEAGI")
        return
    
    print("Connected to FEAGI successfully")
    
    try:
        # Generate and send different patterns every second
        for i in range(50):  # Run for 50 iterations
            # Create a square pattern
            square_size = min(4, x_max - 1, y_max - 1)  # Ensure square fits in the area
            x_offset = i % max(1, x_max - square_size)  # Move horizontally
            y_offset = (i // 5) % max(1, y_max - square_size)  # Move vertically every 5 steps
            
            pattern_coords = []
            for dx in range(square_size):
                for dy in range(square_size):
                    x = x_offset + dx
                    y = y_offset + dy
                    pattern_coords.append([x, y, 0])  # z is typically 0 for 2D sensory inputs
            
            # Send pattern
            await generate_and_send_pattern(client, pattern_coords)
            
            # Wait to see the effect
            await asyncio.sleep(0.2)
            
            # Print a progress indicator every 10 iterations
            if (i + 1) % 10 == 0:
                print(f"Progress: {i + 1}/50 iterations completed")
    
    except KeyboardInterrupt:
        print("Test interrupted")
    finally:
        # Clean up
        await client.disconnect()
        print("Disconnected from FEAGI")


if __name__ == "__main__":
    # Run the test
    asyncio.run(generate_moving_pattern()) 