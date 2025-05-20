#!/usr/bin/env python3
"""
FEAGI Debug Sensory Test

This script tests sending sensory data to FEAGI via ZMQ (port 5558)
with enhanced debugging information and longer timeouts.
"""

import sys
import os
import json
import time
import logging
import numpy as np
from typing import Dict, Tuple
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
logger = logging.getLogger("feagi_debug_test")

# Import our FEAGI client
from feagi_client import FeagiClient

# Sensory client for port 5558
class SensoryClient:
    """Client for sending sensory data to FEAGI on port 5558"""
    
    def __init__(self, host="127.0.0.1", port=5558, agent_id=None, timeout=10000):
        self.host = host
        self.port = port
        self.agent_id = agent_id or f"test-agent-{random.randint(1000, 9999)}"
        self.timeout = timeout  # 10 seconds timeout
        self.context = zmq.Context.instance()
        self.socket = None
        self.registered = False
        
    def connect(self) -> bool:
        """Connect to the FEAGI sensorimotor server"""
        try:
            logger.info(f"Connecting to FEAGI sensorimotor server at {self.host}:{self.port}")
            self.socket = self.context.socket(zmq.DEALER)
            logger.debug(f"Socket type: DEALER")
            self.socket.setsockopt(zmq.IDENTITY, self.agent_id.encode())
            logger.debug(f"Socket identity: {self.agent_id}")
            self.socket.setsockopt(zmq.RCVTIMEO, self.timeout)
            logger.debug(f"Socket timeout: {self.timeout}ms")
            self.socket.connect(f"tcp://{self.host}:{self.port}")
            logger.info(f"Socket connected to tcp://{self.host}:{self.port}")
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
            
            # Debug - dump raw message
            hello_json = json.dumps(hello_msg).encode()
            logger.debug(f"Encoded hello message: {hello_json!r}")
            
            # Send as multipart with empty delimiter
            self.socket.send_multipart([
                b"",  # Empty delimiter frame
                hello_json
            ])
            logger.debug("Hello message sent with empty delimiter frame")
            
            # Try to poll for response instead of blocking recv
            logger.info("Polling for welcome message...")
            poller = zmq.Poller()
            poller.register(self.socket, zmq.POLLIN)
            
            # Poll with timeout
            poll_result = dict(poller.poll(timeout=self.timeout))
            
            if self.socket in poll_result:
                logger.debug("Poll detected incoming message")
                response = self.socket.recv_multipart()
                logger.info(f"Received response with {len(response)} frames")
                
                # Debug - dump frames
                for i, frame in enumerate(response):
                    logger.debug(f"Frame {i}: {frame!r}")
                
                if len(response) < 2:
                    logger.error("Invalid response format - fewer than 2 frames")
                    return False
                
                # Decode the response
                try:
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
                        capabilities_json = json.dumps(capabilities).encode()
                        logger.debug(f"Encoded capabilities: {capabilities_json!r}")
                        
                        self.socket.send_multipart([
                            b"",  # Empty delimiter frame
                            capabilities_json
                        ])
                        logger.debug("Capabilities sent with empty delimiter frame")
                        
                        # Poll for capabilities_ack
                        logger.info("Polling for capabilities_ack...")
                        poll_result = dict(poller.poll(timeout=self.timeout))
                        
                        if self.socket in poll_result:
                            logger.debug("Poll detected incoming capabilities_ack")
                            response = self.socket.recv_multipart()
                            logger.info(f"Received capabilities response with {len(response)} frames")
                            
                            # Debug - dump frames
                            for i, frame in enumerate(response):
                                logger.debug(f"Frame {i}: {frame!r}")
                            
                            if len(response) < 2:
                                logger.error("Invalid capabilities_ack response - fewer than 2 frames")
                                return False
                            
                            try:
                                capabilities_ack = json.loads(response[1].decode())
                                logger.info(f"Capabilities response: {capabilities_ack}")
                                
                                if capabilities_ack.get("message_type") == "capabilities_ack":
                                    logger.info("Registration successful!")
                                    self.registered = True
                                    return True
                                else:
                                    logger.error(f"Unexpected message type: {capabilities_ack.get('message_type')}")
                            except json.JSONDecodeError:
                                logger.error(f"Invalid JSON in capabilities_ack response")
                        else:
                            logger.error("Timeout waiting for capabilities_ack")
                    else:
                        logger.error(f"Unexpected message type: {welcome_data.get('message_type')}")
                except json.JSONDecodeError:
                    logger.error(f"Invalid JSON in welcome response")
            else:
                logger.error("Timeout waiting for welcome response")
                
        except Exception as e:
            logger.error(f"Error during registration: {e}")
            logger.error(traceback.format_exc())
            
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
            
            logger.debug(f"Preparing to encode {len(coordinates)} neurons")
            
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
            
            logger.debug(f"Encoded neuron data size: {len(encoded_data)} bytes")
            logger.debug(f"First 20 bytes: {encoded_data[:20]!r}")
            
            # Create message
            message = {
                "message_type": "sensory_data",
                "cortical_area": cortical_area,
                "timestamp": time.time(),
                "data_format": "feagi_bytes"
            }
            
            logger.info(f"Sending sensory data to {cortical_area} with {len(neuron_data)} neurons")
            
            message_json = json.dumps(message).encode()
            logger.debug(f"Encoded message: {message_json!r}")
            
            # Send the message
            self.socket.send_multipart([
                b"",  # Empty delimiter frame
                message_json,
                encoded_data
            ])
            logger.debug("Sensory data sent with empty delimiter frame")
            
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
        self.registered = False
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

def debug_sensory_test():
    """Run a debug test for sending sensory data"""
    logger.info("Starting FEAGI debug sensory test")
    
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
        sensory_client = SensoryClient(host="127.0.0.1", port=5558)
        
        try:
            # Try to register the sensory agent
            if sensory_client.register_agent(sensory_channels=["visual"]):
                logger.info("Successfully registered sensory agent")
                
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
            else:
                logger.error("Failed to register sensory agent")
        finally:
            # Close the sensory client
            sensory_client.close()
        
        # Check simulation status again
        status = cmd_client.get_simulation_state()
        logger.info(f"Final simulation state: {status}")
        
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        logger.error(traceback.format_exc())
    
    logger.info("Debug sensory test completed")


if __name__ == "__main__":
    debug_sensory_test() 