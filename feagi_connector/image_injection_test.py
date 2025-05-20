#!/usr/bin/env python3
"""
FEAGI Image Injection Test

This script demonstrates how to use the SensoryClient to inject image data into FEAGI,
converting a small test image into neuron activations.
"""

import sys
import os
import time
import logging
import numpy as np
from typing import Dict, Tuple
import random
from PIL import Image
import io

# Add .venv to path to ensure we get the right zmq
venv_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), ".venv", "lib", "python3.9", "site-packages")
sys.path.insert(0, venv_path)

# Import our FEAGI clients
from feagi_client import FeagiClient
from sensory_client import SensoryClient

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("feagi_image_test")


def generate_test_image(width=20, height=20):
    """Generate a small test image with some patterns"""
    # Create a blank (black) image
    img = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Add a simple pattern (white diagonal line)
    for i in range(min(width, height)):
        img[i, i] = [255, 255, 255]
    
    # Add a small square in the upper left
    img[2:6, 2:6] = [255, 0, 0]  # Red square
    
    # Add a small square in the lower right
    img[height-6:height-2, width-6:width-2] = [0, 255, 0]  # Green square
    
    # Convert to PIL Image
    return Image.fromarray(img)


def image_to_neuron_data(image, threshold=50) -> Dict[Tuple[int, int, int], float]:
    """
    Convert an image to neuron activation data.
    
    Args:
        image: PIL Image to convert
        threshold: Brightness threshold for activation (0-255)
        
    Returns:
        Dictionary mapping (x,y,z) coordinates to activation values
    """
    # Resize image to a manageable size if needed
    if max(image.width, image.height) > 50:
        image = image.resize((min(image.width, 50), min(image.height, 50)))
        
    # Convert to grayscale and get pixel data
    if image.mode != 'L':  # If not already grayscale
        grayscale = image.convert('L')
    else:
        grayscale = image
    
    # Get pixel data as 2D array
    pixels = np.array(grayscale)
    
    # Convert to neuron activations (coordinates to values)
    neuron_data = {}
    
    # Iterate through pixels
    for y in range(pixels.shape[0]):
        for x in range(pixels.shape[1]):
            # Only activate if pixel brightness is above threshold
            if pixels[y, x] > threshold:
                # Normalize activation to 0.0-1.0 range
                activation = pixels[y, x] / 255.0
                
                # Add to neuron data (z=0 for 2D image)
                neuron_data[(x, y, 0)] = activation
    
    return neuron_data


def run_image_test():
    """Run a test injecting image data into FEAGI"""
    logger.info("Starting FEAGI image injection test")
    
    # First check if FEAGI is running using the command client
    cmd_client = FeagiClient(host="127.0.0.1", port=5555)
    
    if not cmd_client.is_running():
        logger.error("FEAGI is not running!")
        return
    
    logger.info("FEAGI is running")
    
    # Check simulation status
    status = cmd_client.get_simulation_state()
    logger.info(f"Simulation state: {status}")
    
    # Create sensory client
    sensory_client = SensoryClient(host="127.0.0.1", port=5558)
    
    try:
        # Generate test image
        logger.info("Generating test image")
        test_image = generate_test_image(width=20, height=20)
        
        # Convert image to neuron data
        logger.info("Converting image to neuron activations")
        neuron_data = image_to_neuron_data(test_image, threshold=30)
        logger.info(f"Image converted to {len(neuron_data)} neuron activations")
        
        # Get the proper cortical ID for visual input
        cortical_id = sensory_client.get_cortical_id("visual_input")  # Should return "i_00CC"
        
        # Prepare metadata
        metadata = {
            "agent_id": sensory_client.agent_id,
            "timestamp": time.time(),
            "message_type": "sensory_data",
            "cortical_id": cortical_id,  # Use cortical_id instead of cortical_area
            "data_type": "image",
            "width": test_image.width,
            "height": test_image.height
        }
        
        # Send neuron data to FEAGI
        logger.info(f"Sending image data to cortical ID {cortical_id}...")
        
        # Send multiple times to make it more visible
        for i in range(10):
            # Vary activation slightly to create animation effect
            varied_data = {}
            for coord, value in neuron_data.items():
                # Add small random variation to activation (±10%)
                varied_value = value * (0.9 + 0.2 * random.random())
                varied_data[coord] = varied_value
            
            if sensory_client.send_neuron_data(cortical_id, varied_data, metadata):
                logger.info(f"Successfully sent image data (frame {i+1}/10)")
            else:
                logger.error(f"Failed to send image data (frame {i+1}/10)")
                break
                
            time.sleep(0.5)  # Half-second between frames
            
    finally:
        # Close the sensory client
        sensory_client.close()
        
    # Check simulation status again
    status = cmd_client.get_simulation_state()
    logger.info(f"Final simulation state: {status}")
    
    logger.info("Image injection test completed")


if __name__ == "__main__":
    run_image_test() 