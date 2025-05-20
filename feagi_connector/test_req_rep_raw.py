#!/usr/bin/env python3
"""
Raw ZMQ REQ/REP Test for FEAGI

This script tests the raw ZMQ REQ/REP pattern on port 5555 without any
additional protocol formatting, just sending a simple string message.
"""

import sys
import os
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
logger = logging.getLogger("raw_req_rep_test")

def test_req_rep_simple(host="127.0.0.1", port=5555):
    """Send a simple raw message using REQ/REP pattern."""
    logger.info(f"Testing raw REQ/REP on {host}:{port}")
    
    # Create context and socket
    context = zmq.Context.instance()
    socket = context.socket(zmq.REQ)
    
    try:
        # Connect to port
        socket.connect(f"tcp://{host}:{port}")
        logger.info(f"Connected to {host}:{port}")
        
        # Create a simple message - just a timestamp as text
        message = f"PING {int(time.time())}"
        message_bytes = message.encode('utf-8')
        
        # Send message
        logger.info(f"Sending: {message}")
        socket.send(message_bytes)
        
        # Set timeout
        socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5 second timeout
        
        # Receive response
        try:
            response = socket.recv()
            logger.info(f"Received {len(response)} bytes: {response!r}")
            logger.info(f"Response decoded: {response.decode('utf-8', errors='replace')}")
            return True
        except zmq.ZMQError as e:
            logger.error(f"Error receiving: {e}")
            return False
            
    except Exception as e:
        logger.error(f"Error: {e}")
        return False
    finally:
        socket.close()
        logger.info("Socket closed")

if __name__ == "__main__":
    logger.info("Starting raw REQ/REP test")
    test_req_rep_simple()
    logger.info("Test completed") 