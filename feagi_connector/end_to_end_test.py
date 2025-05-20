#!/usr/bin/env python3
"""
FEAGI End-to-End Test

This script tests the full workflow of:
1. Loading a genome into FEAGI via ZMQ
2. Starting a simulation
3. Sending sensory data

It uses the REQ/REP pattern (port 5555) for commands and
attempts to use DEALER/ROUTER (port 5558) for sensory data.
"""

import sys
import os
import json
import time
import logging
import numpy as np
from typing import Dict, Tuple
import random

# Add .venv to path to ensure we get the right zmq
venv_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), ".venv", "lib", "python3.9", "site-packages")
sys.path.insert(0, venv_path)

import zmq
from feagi_bytes import ByteStructureEncoder, ByteStructureID

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("feagi_e2e_test")

# Import our FEAGI client
from feagi_client import FeagiClient

# Path to test genome
TEST_GENOME_PATH = os.path.join(os.path.dirname(__file__), "test_genome.json")

# Sensory client for port 5558
class SensoryClient:
    """Client for sending sensory data to FEAGI on port 5558"""
    
    def __init__(self, host="127.0.0.1", port=5558, agent_id=None, timeout=2000):
        self.host = host
        self.port = port
        self.agent_id = agent_id or f"test-agent-{random.randint(1000, 9999)}"
        self.timeout = timeout
        self.context = zmq.Context.instance()
        self.socket = None
        self.registered = False
        
    def connect(self) -> bool:
        """Connect to the FEAGI sensorimotor server"""
        try:
            logger.info(f"Connecting to FEAGI sensorimotor server at {self.host}:{self.port}")
            self.socket = self.context.socket(zmq.DEALER)
            self.socket.setsockopt(zmq.IDENTITY, self.agent_id.encode())
            self.socket.setsockopt(zmq.RCVTIMEO, self.timeout)
            self.socket.connect(f"tcp://{self.host}:{self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            return False
    
    def register_agent(self, sensory_channels=None) -> bool:
        """Register agent with FEAGI"""
        if self.registered:
            return True
            
        if not self.socket:
            if not self.connect():
                return False
        
        try:
            # Send hello message
            hello_msg = {
                "message_type": "hello",
                "agent_id": self.agent_id,
                "agent_type": "test-agent",
                "timestamp": time.time()
            }
            
            logger.info(f"Sending hello message: {hello_msg}")
            self.socket.send_multipart([
                b"",  # Empty delimiter frame
                json.dumps(hello_msg).encode()
            ])
            
            # Wait for welcome message
            try:
                logger.info("Waiting for welcome message...")
                response = self.socket.recv_multipart()
                logger.info(f"Received response with {len(response)} frames")
                
                if len(response) < 2:
                    logger.error("Invalid response format")
                    return False
                
                # Decode the response
                welcome_data = json.loads(response[1].decode())
                logger.info(f"Welcome response: {welcome_data}")
                
                if welcome_data.get("message_type") == "welcome":
                    # Send capabilities
                    capabilities = {
                        "message_type": "capabilities",
                        "agent_id": self.agent_id,
                        "supported_sensory_channels": sensory_channels or ["visual"],
                        "supported_motor_channels": [],
                        "timestamp": time.time()
                    }
                    
                    logger.info(f"Sending capabilities: {capabilities}")
                    self.socket.send_multipart([
                        b"",  # Empty delimiter frame
                        json.dumps(capabilities).encode()
                    ])
                    
                    # Wait for capabilities_ack
                    try:
                        logger.info("Waiting for capabilities_ack...")
                        response = self.socket.recv_multipart()
                        
                        if len(response) < 2:
                            logger.error("Invalid capabilities_ack response")
                            return False
                        
                        capabilities_ack = json.loads(response[1].decode())
                        logger.info(f"Capabilities response: {capabilities_ack}")
                        
                        if capabilities_ack.get("message_type") == "capabilities_ack":
                            logger.info("Registration successful!")
                            self.registered = True
                            return True
                    except zmq.error.Again:
                        logger.error("Timeout waiting for capabilities_ack")
                
            except zmq.error.Again:
                logger.error("Timeout waiting for welcome response")
                
        except Exception as e:
            logger.error(f"Error during registration: {e}")
            
        return False
    
    def send_sensory_data(self, cortical_area: str, neuron_data: Dict[Tuple[int, int, int], float]) -> bool:
        """Send sensory data to FEAGI"""
        if not self.registered:
            logger.warning("Not registered. Attempting to register...")
            if not self.register_agent():
                return False
        
        try:
            # Convert neuron data for encoding
            coordinates = []
            values = []
            
            for coord, value in neuron_data.items():
                coordinates.append(coord)
                values.append(value)
            
            # Create numpy arrays
            coordinates_array = np.array(coordinates, dtype=np.int32)
            values_array = np.array(values, dtype=np.float32)
            
            # Encode the data
            encoder = ByteStructureEncoder()
            encoded_data = encoder.encode_neuron_data(
                ByteStructureID.NEURON_DATA_V1,
                coordinates_array,
                values_array
            )
            
            # Create message
            message = {
                "message_type": "sensory_data",
                "cortical_area": cortical_area,
                "timestamp": time.time(),
                "data_format": "feagi_bytes"
            }
            
            logger.info(f"Sending sensory data to {cortical_area} with {len(neuron_data)} neurons")
            
            # Send the message
            self.socket.send_multipart([
                b"",  # Empty delimiter frame
                json.dumps(message).encode(),
                encoded_data
            ])
            
            return True
            
        except Exception as e:
            logger.error(f"Error sending sensory data: {e}")
            return False
    
    def close(self):
        """Close the connection"""
        if self.socket:
            self.socket.close()
            self.socket = None
        self.registered = False


def upload_genome(client: FeagiClient) -> bool:
    """Upload the test genome to FEAGI"""
    logger.info(f"Loading test genome from {TEST_GENOME_PATH}")
    
    try:
        # Load the genome file
        with open(TEST_GENOME_PATH, 'r') as f:
            genome_data = json.load(f)
            
        # Use the POST method to upload the genome
        logger.info("Uploading genome to FEAGI...")
        response = client.command("upload_genome", {"genome": genome_data})
        
        if "error" in response:
            logger.error(f"Failed to upload genome: {response['error']}")
            return False
            
        logger.info(f"Genome upload response: {response}")
        return True
        
    except Exception as e:
        logger.error(f"Error uploading genome: {e}")
        return False


def start_simulation(client: FeagiClient) -> bool:
    """Start the FEAGI simulation"""
    logger.info("Starting FEAGI simulation...")
    
    try:
        response = client.command("start_simulation")
        
        if "error" in response:
            logger.error(f"Failed to start simulation: {response['error']}")
            return False
            
        logger.info(f"Simulation start response: {response}")
        return True
        
    except Exception as e:
        logger.error(f"Error starting simulation: {e}")
        return False


def generate_test_sensory_data(size=10) -> Dict[Tuple[int, int, int], float]:
    """Generate test sensory data"""
    data = {}
    for x in range(size):
        for y in range(size):
            # Only activate some neurons randomly
            if random.random() > 0.7:
                data[(x, y, 0)] = random.random()
    return data


def end_to_end_test():
    """Run the end-to-end test"""
    logger.info("Starting FEAGI end-to-end test")
    
    # Create the command client
    cmd_client = FeagiClient(host="127.0.0.1", port=5555)
    
    # Check if FEAGI is running
    if not cmd_client.is_running():
        logger.error("FEAGI is not running!")
        return
        
    logger.info("FEAGI is running")
    
    # Check current simulation status
    status = cmd_client.get_simulation_state()
    logger.info(f"Initial simulation state: {status}")
    
    # Upload the genome
    if not upload_genome(cmd_client):
        logger.error("Failed to upload genome. Aborting test.")
        return
        
    # Try starting the simulation
    if not start_simulation(cmd_client):
        logger.warning("Failed to start simulation. Continuing with test...")
    
    # Create sensory client
    sensory_client = SensoryClient(host="127.0.0.1", port=5558)
    
    # Register the sensory agent
    if sensory_client.register_agent(sensory_channels=["visual"]):
        logger.info("Successfully registered sensory agent")
        
        # Send sensory data a few times
        for i in range(5):
            logger.info(f"Sending sensory data (iteration {i+1}/5)...")
            sensory_data = generate_test_sensory_data(size=10)
            
            if sensory_client.send_sensory_data("visual_input", sensory_data):
                logger.info(f"Successfully sent sensory data (iteration {i+1}/5)")
            else:
                logger.error(f"Failed to send sensory data (iteration {i+1}/5)")
                
            time.sleep(1)
            
        # Close the sensory client
        sensory_client.close()
    else:
        logger.error("Failed to register sensory agent")
    
    # Check simulation status again
    status = cmd_client.get_simulation_state()
    logger.info(f"Final simulation state: {status}")
    
    logger.info("End-to-end test completed")


if __name__ == "__main__":
    end_to_end_test() 