#!/usr/bin/env python3
"""
FEAGI Image Test Client

A test client that sends a test pattern image to FEAGI.
This demonstrates the safeguards and connection handling in a real-world scenario.
"""

import os
import sys
import time
import logging
import numpy as np
from typing import Dict, Tuple, List, Optional
import argparse

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("feagi_image_client")

# Import our sensory client
from sensory_client import SensoryClient


class ImageClient:
    """Client for sending image data to FEAGI as neuron activations"""
    
    def __init__(self, host: str = "127.0.0.1", port: int = 5558, cortical_area: str = "visual_input"):
        """
        Initialize the image client.
        
        Args:
            host: FEAGI hostname or IP
            port: ZMQ sensorimotor port (default 5558)
            cortical_area: Target cortical area or ID for visual input
        """
        self.sensory_client = SensoryClient(host=host, port=port)
        self.cortical_area = cortical_area
        
    def connect(self) -> bool:
        """Connect to FEAGI"""
        return self.sensory_client.connect()
    
    def create_test_pattern(self, width: int = 28, height: int = 28) -> np.ndarray:
        """
        Create a simple test pattern image.
        
        Args:
            width: Image width
            height: Image height
            
        Returns:
            NumPy array with pixel values 0-1
        """
        # Create a blank image
        img = np.zeros((height, width), dtype=np.float32)
        
        # Draw a simple pattern - horizontal and vertical lines
        img[height // 2, :] = 1.0
        img[:, width // 2] = 1.0
        
        # Draw a simple box in the center
        box_size = min(width, height) // 4
        center_x = width // 2
        center_y = height // 2
        
        start_x = center_x - box_size // 2
        end_x = center_x + box_size // 2
        start_y = center_y - box_size // 2
        end_y = center_y + box_size // 2
        
        img[start_y:end_y, start_x:end_x] = 0.8
        
        return img
    
    def image_to_neuron_data(self, 
                             image: np.ndarray, 
                             threshold: float = 0.1) -> Dict[Tuple[int, int, int], float]:
        """
        Convert an image to neuron data.
        
        Args:
            image: NumPy array with values 0-1
            threshold: Minimum value to activate a neuron
            
        Returns:
            Dictionary mapping neuron coordinates to activation values
        """
        # Get image dimensions
        height, width = image.shape
        
        # Create a dictionary to hold neuron data
        neuron_data = {}
        
        # Convert image pixels to neuron activations
        for y in range(height):
            for x in range(width):
                # Only include values above threshold
                if image[y, x] > threshold:
                    # Use z=0 for all neurons in a 2D image
                    neuron_data[(x, y, 0)] = float(image[y, x])
        
        return neuron_data
    
    def send_image(self, 
                  image: Optional[np.ndarray] = None, 
                  max_retries: int = 3,
                  wait_for_ready: bool = True,
                  retry_delay: float = 1.0) -> bool:
        """
        Send an image to FEAGI.
        
        Args:
            image: Image to send (or generate a test pattern if None)
            max_retries: Maximum number of retry attempts
            wait_for_ready: Whether to wait for FEAGI to be ready
            retry_delay: Delay between retries
            
        Returns:
            True if sent successfully, False otherwise
        """
        # Create a test pattern if no image is provided
        if image is None:
            logger.info("Creating test pattern")
            image = self.create_test_pattern()
        
        # Convert image to neuron data
        neuron_data = self.image_to_neuron_data(image)
        logger.info(f"Converting image to {len(neuron_data)} neuron activations")
        
        # Metadata for the message
        metadata = {
            "message_type": "image_data",
            "width": image.shape[1],
            "height": image.shape[0],
            "timestamp": time.time()
        }
        
        # Try to connect and send data
        retries = 0
        while retries < max_retries:
            # If this is a retry, wait before trying again
            if retries > 0:
                logger.info(f"Retry {retries}/{max_retries} after {retry_delay}s delay")
                time.sleep(retry_delay)
                
                # Force reconnect on retry
                self.sensory_client.connect(force=True)
            
            # If waiting for ready state, check if FEAGI is ready
            if wait_for_ready:
                ready_retries = 0
                while ready_retries < 3:  # Try checking ready state up to 3 times
                    if self.sensory_client.validate_ready_state():
                        break
                    logger.warning("FEAGI not in ready state, waiting to retry...")
                    time.sleep(retry_delay)
                    ready_retries += 1
                    
                # If still not ready after retries, skip this attempt
                if not self.sensory_client.is_genome_loaded:
                    logger.error("FEAGI has no genome loaded, cannot proceed")
                    return False
            
            # Try to send the data
            logger.info(f"Sending image data to {self.cortical_area}")
            success = self.sensory_client.send_neuron_data(
                cortical_area=self.cortical_area,
                neuron_data=neuron_data,
                metadata=metadata,
                check_status=True,
                validate_data=True
            )
            
            if success:
                logger.info("Successfully sent image data")
                return True
            
            retries += 1
        
        logger.error(f"Failed to send image data after {max_retries} attempts")
        return False
    
    def close(self):
        """Close the connection"""
        self.sensory_client.close()


def main():
    """Run the image client with command line arguments"""
    parser = argparse.ArgumentParser(description="FEAGI Image Test Client")
    parser.add_argument("--host", default="127.0.0.1", help="FEAGI host IP or hostname")
    parser.add_argument("--port", type=int, default=5558, help="FEAGI sensorimotor port")
    parser.add_argument("--continuous", action="store_true", help="Send images continuously")
    parser.add_argument("--delay", type=float, default=1.0, help="Delay between images in seconds")
    parser.add_argument("--retries", type=int, default=3, help="Maximum retry attempts")
    args = parser.parse_args()
    
    # Create the image client
    client = ImageClient(host=args.host, port=args.port)
    
    try:
        # Connect to FEAGI
        if not client.connect():
            logger.error("Failed to connect to FEAGI")
            return 1
        
        # Check FEAGI state
        if not client.sensory_client.validate_ready_state():
            logger.warning("FEAGI is not in a ready state - image data may not be processed")
            logger.warning("Make sure a genome is loaded and simulation is running")
        
        # Send a single test image or continuous images
        if args.continuous:
            logger.info(f"Sending continuous test images with {args.delay}s delay...")
            count = 0
            while True:
                count += 1
                logger.info(f"Sending test image {count}")
                
                # Generate a new test pattern
                test_image = client.create_test_pattern()
                
                # Send the image
                if not client.send_image(
                    image=test_image, 
                    max_retries=args.retries,
                    wait_for_ready=True
                ):
                    logger.error(f"Failed to send test image {count}")
                
                # Wait before sending the next image
                time.sleep(args.delay)
        else:
            # Send a single test image
            logger.info("Sending a single test image")
            if client.send_image(max_retries=args.retries):
                logger.info("Test image sent successfully")
                return 0
            else:
                logger.error("Failed to send test image")
                return 1
        
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        client.close()
    
    return 0


if __name__ == "__main__":
    sys.exit(main()) 