#!/usr/bin/env python3
"""
FEAGI Connector Sensory Injection Test

This script demonstrates connecting to FEAGI using the ZMQ REST API,
loading a test genome, and then injecting sensory data to stimulate neurons.
"""

import asyncio
import logging
import random
import sys
import time
from typing import List, Dict, Any, Optional

# Use a relative import since we're in the same package
from .zmq_rest_client import FeagiZmqRestClient

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("feagi_sensory_injection_test.log")
    ]
)
logger = logging.getLogger("sensory_injection_test")

# Configuration
DEFAULT_HOST = "127.0.0.1"
DEFAULT_ZMQ_REST_PORT = 5560
VISUAL_CORTICAL_AREA = "iv00CC"  # Typically the visual cortical area in test genomes
NUM_PATTERNS = 50  # Number of patterns to generate
PATTERN_SIZE = 16  # Number of neurons in each pattern
DELAY_BETWEEN_PATTERNS = 0.2  # Seconds


async def generate_random_pattern(size: int, dimensions: Dict[str, int]) -> List[List[int]]:
    """
    Generate a random pattern of neuron coordinates for stimulation.
    
    Args:
        size: Number of neurons to include in the pattern
        dimensions: Dictionary containing 'x', 'y', 'z' dimensions of the cortical area
        
    Returns:
        List of [x,y,z] coordinates
    """
    pattern = []
    
    # Get dimensions with defaults if not provided
    x_max = dimensions.get('x', 10)
    y_max = dimensions.get('y', 10)
    z_max = dimensions.get('z', 1)
    
    # Generate a small cluster of neurons for a more coherent pattern
    base_x = random.randint(0, max(0, x_max - 4))
    base_y = random.randint(0, max(0, y_max - 4))
    
    for _ in range(size):
        # Use a smaller area to create a cluster effect
        x = min(x_max - 1, max(0, base_x + random.randint(-2, 2)))
        y = min(y_max - 1, max(0, base_y + random.randint(-2, 2)))
        z = 0  # Usually sensory input is on z=0
        
        # Avoid duplicates
        coord = [x, y, z]
        if coord not in pattern:
            pattern.append(coord)
    
    return pattern


async def run_test():
    """
    Main test function to connect to FEAGI, load a genome, and inject sensory data.
    
    Returns:
        True if successful, False otherwise
    """
    client = FeagiZmqRestClient(
        host=DEFAULT_HOST,
        zmq_rest_port=DEFAULT_ZMQ_REST_PORT,
        agent_id=f"sensory-test-{int(time.time())}"
    )
    
    try:
        # Connect to FEAGI
        logger.info(f"Connecting to FEAGI at {DEFAULT_HOST}")
        connected = await client.connect()
        if not connected:
            logger.error("Failed to connect to FEAGI")
            return False
        
        logger.info("Successfully connected to FEAGI")
        
        # Load the test genome
        logger.info("Loading test genome")
        genome_loaded = await client.load_genome()
        if not genome_loaded:
            logger.error("Failed to load test genome")
            return False
        
        logger.info("Test genome loaded successfully")
        
        # Wait for the genome to be processed
        logger.info("Waiting for FEAGI to process the genome (5 seconds)")
        await asyncio.sleep(5)
        
        # Get available cortical areas
        cortical_areas = await client.get_available_cortical_areas()
        if not cortical_areas:
            logger.error("No cortical areas found")
            return False
        
        # Find our target cortical area
        target_area = None
        for area in cortical_areas:
            if area.get('id') == VISUAL_CORTICAL_AREA:
                target_area = area
                break
        
        if not target_area:
            logger.error(f"Target cortical area '{VISUAL_CORTICAL_AREA}' not found")
            logger.info(f"Available areas: {[area.get('id') for area in cortical_areas]}")
            
            # If our target wasn't found but we have other areas, use the first one
            if cortical_areas:
                target_area = cortical_areas[0]
                logger.info(f"Using alternate cortical area: {target_area.get('id')}")
            else:
                return False
        
        # Extract dimensions
        dimensions = target_area.get('dimensions', {'x': 10, 'y': 10, 'z': 1})
        cortical_id = target_area.get('id')
        logger.info(f"Using cortical area '{cortical_id}' with dimensions {dimensions}")
        
        # Generate and send patterns
        logger.info(f"Generating and sending {NUM_PATTERNS} patterns")
        for i in range(NUM_PATTERNS):
            # Generate a random pattern
            pattern = await generate_random_pattern(PATTERN_SIZE, dimensions)
            
            # Send to FEAGI
            success = await client.send_sensory_data_to_cortical_area(cortical_id, pattern)
            if not success:
                logger.warning(f"Failed to send pattern {i+1}")
                continue
                
            logger.info(f"Pattern {i+1}/{NUM_PATTERNS} sent successfully")
            
            # Delay between patterns
            await asyncio.sleep(DELAY_BETWEEN_PATTERNS)
        
        logger.info("Test completed successfully")
        return True
        
    except Exception as e:
        logger.exception(f"Error during test: {e}")
        return False
    finally:
        # Disconnect from FEAGI
        await client.disconnect()
        logger.info("Disconnected from FEAGI")


async def main():
    """Main entry point."""
    try:
        success = await run_test()
        if success:
            logger.info("Test completed successfully!")
        else:
            logger.error("Test failed!")
    except KeyboardInterrupt:
        logger.info("Test interrupted by user")
    except Exception as e:
        logger.exception(f"Unhandled error: {e}")


if __name__ == "__main__":
    asyncio.run(main()) 