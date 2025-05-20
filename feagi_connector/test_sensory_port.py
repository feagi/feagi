#!/usr/bin/env python3
"""
Simple test for FEAGI's sensorimotor port (5558)
"""

import sys
import os
import json
import time
import logging

# Add .venv to path to ensure we get the right zmq
venv_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), ".venv", "lib", "python3.9", "site-packages")
sys.path.insert(0, venv_path)

import zmq

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("sensory_port_test")

def test_sensory_port():
    """Test the sensorimotor port with a raw ZMQ DEALER socket."""
    context = zmq.Context()
    socket = context.socket(zmq.DEALER)
    
    # Set socket options
    socket.setsockopt(zmq.IDENTITY, b"test-client")
    socket.setsockopt(zmq.RCVTIMEO, 2000)  # 2 second timeout
    
    try:
        # Connect to the server
        logger.info("Connecting to tcp://127.0.0.1:5558")
        socket.connect("tcp://127.0.0.1:5558")
        
        # Create a hello message
        hello_msg = {
            "message_type": "hello",
            "agent_id": "test-client",
            "agent_type": "test",
            "timestamp": time.time()
        }
        
        # Send the message with proper DEALER/ROUTER framing
        logger.info(f"Sending: {hello_msg}")
        socket.send_multipart([
            b"",  # Empty delimiter frame
            json.dumps(hello_msg).encode()
        ])
        
        # Wait for response
        logger.info("Waiting for response...")
        try:
            response = socket.recv_multipart()
            logger.info(f"Received {len(response)} frames: {response}")
            
            if len(response) > 1:
                try:
                    # Try to decode the second frame as JSON
                    data = json.loads(response[1].decode())
                    logger.info(f"Decoded JSON: {data}")
                except:
                    logger.warning("Could not decode as JSON")
                    
        except zmq.error.Again:
            logger.error("No response received (timeout)")
            
    except Exception as e:
        logger.error(f"Error: {e}")
        
    finally:
        socket.close()
        context.term()

if __name__ == "__main__":
    test_sensory_port() 