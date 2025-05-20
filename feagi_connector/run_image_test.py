#!/usr/bin/env python3
"""
FEAGI Image Test with Session Management

This script demonstrates the proper way to send sensory data to FEAGI
by first ensuring a genome is loaded and the simulation is running.
"""

import os
import sys
import time
import logging
import argparse
from typing import Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("feagi_image_test")

# Import our modules
from feagi_session import FeagiSession
from test_image_client import ImageClient


def run_test(host: str = "127.0.0.1", 
            cmd_port: int = 5555,
            sensory_port: int = 5558, 
            genome_name: Optional[str] = None,
            genome_file: Optional[str] = None,
            continuous: bool = False,
            delay: float = 1.0) -> int:
    """
    Run the full image test with session management.
    
    Args:
        host: FEAGI hostname or IP
        cmd_port: Command API port
        sensory_port: Sensorimotor port
        genome_name: Name of genome to load (if None, will try to load any available)
        genome_file: Path to genome file to upload (if needed)
        continuous: Whether to send data continuously
        delay: Delay between sends in seconds
        
    Returns:
        Exit code (0 for success, non-zero for error)
    """
    # Create session manager and image client
    logger.info(f"Connecting to FEAGI at {host}")
    session = FeagiSession(host=host, port=cmd_port)
    image_client = ImageClient(host=host, port=sensory_port)
    
    try:
        # Step 1: Check if FEAGI is running
        if not session.check_connection():
            logger.error("FEAGI is not running or not reachable")
            return 1
        
        # Step 2: Get current status
        status = session.update_status()
        logger.info(f"FEAGI Status: Running={status.get('is_running', False)}, Genome Loaded={status.get('is_genome_loaded', False)}")
        
        # Step 3: If genome file is provided and no genome is loaded, upload it
        if genome_file and not session.is_genome_loaded:
            if os.path.exists(genome_file):
                logger.info(f"Uploading genome from {genome_file}")
                if not session.upload_genome(genome_file):
                    logger.error("Failed to upload genome")
                    return 1
                # Set genome_name to the uploaded file
                genome_name = os.path.basename(genome_file).split('.')[0]
            else:
                logger.error(f"Genome file not found: {genome_file}")
                return 1
        
        # Step 4: Ensure FEAGI is in a ready state (genome loaded and simulation running)
        logger.info("Ensuring FEAGI is in a ready state...")
        if not session.ensure_ready_state(genome_name):
            logger.error("Failed to ensure FEAGI is in a ready state")
            logger.error("Make sure a genome is available or provide one")
            return 1
        
        logger.info("FEAGI is ready with genome loaded and simulation running")
        
        # Step 5: Connect the image client
        if not image_client.connect():
            logger.error("Failed to connect image client")
            return 1
        
        # Step 6: Send image data
        if continuous:
            logger.info(f"Sending continuous test images with {delay}s delay...")
            try:
                count = 0
                while True:
                    count += 1
                    logger.info(f"Sending test image {count}")
                    
                    # Generate a new test pattern
                    test_image = image_client.create_test_pattern()
                    
                    # Send the image
                    if not image_client.send_image(
                        image=test_image, 
                        max_retries=3,
                        wait_for_ready=True
                    ):
                        logger.error(f"Failed to send test image {count}")
                    
                    # Wait before sending the next image
                    time.sleep(delay)
            except KeyboardInterrupt:
                logger.info("Interrupted by user")
        else:
            # Send a single test image
            logger.info("Sending a single test image")
            if image_client.send_image(max_retries=3):
                logger.info("Test image sent successfully")
            else:
                logger.error("Failed to send test image")
                return 1
        
        return 0
        
    finally:
        # Clean up
        image_client.close()


def main():
    """Main function with command line handling"""
    parser = argparse.ArgumentParser(description="FEAGI Image Test with Session Management")
    parser.add_argument("--host", default="127.0.0.1", help="FEAGI host IP or hostname")
    parser.add_argument("--cmd-port", type=int, default=5555, help="FEAGI command port")
    parser.add_argument("--sensory-port", type=int, default=5558, help="FEAGI sensorimotor port")
    parser.add_argument("--genome", help="Name of genome to load")
    parser.add_argument("--genome-file", help="Path to genome file to upload")
    parser.add_argument("--continuous", action="store_true", help="Send images continuously")
    parser.add_argument("--delay", type=float, default=1.0, help="Delay between images in seconds")
    args = parser.parse_args()
    
    return run_test(
        host=args.host,
        cmd_port=args.cmd_port,
        sensory_port=args.sensory_port,
        genome_name=args.genome,
        genome_file=args.genome_file,
        continuous=args.continuous,
        delay=args.delay
    )


if __name__ == "__main__":
    sys.exit(main()) 