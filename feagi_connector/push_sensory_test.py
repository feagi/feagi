#!/usr/bin/env python3
"""
FEAGI PUSH Sensory Test

This script tests sending sensory data to FEAGI via ZMQ (port 5558)
using the PUSH socket pattern (which connects to the PULL socket on FEAGI).
"""

import sys
import os
import json
import time
import logging
import numpy as np
from typing import Dict, Tuple, List
import random
import traceback

# Add .venv to path to ensure we get the right zmq
venv_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), ".venv", "lib", "python3.9", "site-packages")
sys.path.insert(0, venv_path)

import zmq
from feagi_bytes import ByteStructureEncoder, ByteStructureID

# Configure logging - use DEBUG level
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("feagi_push_test")

# Import our FEAGI client
from feagi_client import FeagiClient

# Sensory client for port 5558
class PushSensoryClient:
    """Client for sending sensory data to FEAGI on port 5558 using PUSH socket"""
    
    def __init__(self, host="127.0.0.1", port=5558):
        self.host = host
        self.port = port
        self.agent_id = f"push-agent-{random.randint(1000, 9999)}"
        self.context = zmq.Context.instance()
        self.socket = None
        
    def connect(self) -> bool:
        """Connect to the FEAGI sensorimotor server"""
        try:
            logger.info(f"Connecting to FEAGI sensorimotor server at {self.host}:{self.port}")
            # Use PUSH socket pattern to connect to FEAGI's PULL socket
            self.socket = self.context.socket(zmq.PUSH)
            logger.debug(f"Socket type: PUSH")
            
            # Configure socket for real-time data
            self.socket.setsockopt(zmq.SNDHWM, 1)  # Minimal send queue
            self.socket.setsockopt(zmq.LINGER, 0)  # Don't wait on close
            
            # Connect to the FEAGI server
            self.socket.connect(f"tcp://{self.host}:{self.port}")
            logger.info(f"Socket connected to tcp://{self.host}:{self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            return False
    
    def send_sensory_data(self, cortical_area: str, neuron_data: Dict[Tuple[int, int, int], float]) -> bool:
        """Send sensory data to FEAGI"""
        if not self.socket:
            if not self.connect():
                return False
        
        try:
            # Extract coordinate components and values
            x_coords: List[int] = []
            y_coords: List[int] = []
            z_coords: List[int] = []
            potentials: List[float] = []
            cortical_ids: List[str] = []
            
            for coord, value in neuron_data.items():
                x, y, z = coord
                x_coords.append(x)
                y_coords.append(y)
                z_coords.append(z)
                potentials.append(value)
                cortical_ids.append(cortical_area)  # Use the same cortical area for all neurons
            
            logger.debug(f"Preparing to encode {len(x_coords)} neurons")
            
            # Encode the data using encode_neuron_flat with correct parameters
            encoder = ByteStructureEncoder()
            encoded_data = encoder.encode_neuron_flat(
                cortical_ids=cortical_ids,
                x_coords=x_coords,
                y_coords=y_coords,
                z_coords=z_coords,
                potentials=potentials
            )
            
            logger.debug(f"Encoded neuron data size: {len(encoded_data)} bytes")
            logger.debug(f"First 20 bytes: {encoded_data[:20]!r}")
            
            # Create message
            message = {
                "type": "sensory_data",
                "cortical_area": cortical_area,
                "agent_id": self.agent_id,
                "timestamp": time.time()
            }
            
            logger.info(f"Sending sensory data to {cortical_area} with {len(neuron_data)} neurons")
            
            # Send the data
            self.socket.send(encoded_data)
            logger.debug("Sensory data sent")
            
            return True
            
        except Exception as e:
            logger.error(f"Error sending sensory data: {e}")
            logger.error(traceback.format_exc())
            return False
    
    def close(self):
        """Close the connection"""
        if self.socket:
            self.socket.close()
            self.socket = None
        logger.info("Sensory client closed")

def generate_test_sensory_data(size=10) -> Dict[Tuple[int, int, int], float]:
    """Generate test sensory data"""
    data = {}
    for x in range(size):
        for y in range(size):
            # Only activate some neurons randomly
            if random.random() > 0.7:
                data[(x, y, 0)] = random.random()
    return data

def push_sensory_test():
    """Run a test for sending sensory data using PUSH socket"""
    logger.info("Starting FEAGI PUSH sensory test")
    
    # Check if FEAGI is running using the command client
    cmd_client = FeagiClient(host="127.0.0.1", port=5555)
    
    try:
        if not cmd_client.is_running():
            logger.error("FEAGI is not running!")
            return
        
        logger.info("FEAGI is running")
        
        # Check simulation status
        status = cmd_client.get_simulation_state()
        logger.info(f"Simulation state: {status}")
        
        # Create sensory client
        sensory_client = PushSensoryClient(host="127.0.0.1", port=5558)
        
        try:
            # Send sensory data a few times
            for i in range(3):
                logger.info(f"Sending sensory data (iteration {i+1}/3)...")
                sensory_data = generate_test_sensory_data(size=5)  # Smaller size for debugging
                
                if sensory_client.send_sensory_data("visual_input", sensory_data):
                    logger.info(f"Successfully sent sensory data (iteration {i+1}/3)")
                else:
                    logger.error(f"Failed to send sensory data (iteration {i+1}/3)")
                    break
                    
                time.sleep(2)  # Longer pause between sends
        finally:
            # Close the sensory client
            sensory_client.close()
        
        # Check simulation status again
        status = cmd_client.get_simulation_state()
        logger.info(f"Final simulation state: {status}")
        
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        logger.error(traceback.format_exc())
    
    logger.info("PUSH sensory test completed")


if __name__ == "__main__":
    push_sensory_test() 