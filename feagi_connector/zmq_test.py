#!/usr/bin/env python3
"""
Basic ZMQ connection test for FEAGI

This script tests different ZMQ connection patterns with FEAGI ports
to determine which protocol each port is using.
"""

import sys
import os

# We need to ensure we're using the system's ZMQ, not our local module
# Add .venv to path to ensure we get the right zmq
venv_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), ".venv", "lib", "python3.9", "site-packages")
sys.path.insert(0, venv_path)

import zmq
import json
import time
import logging

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("zmq_test")

def test_req_rep(host="127.0.0.1", port=5555):
    """Test REQ/REP pattern on the given port."""
    logger.info(f"Testing REQ/REP pattern on {host}:{port}")
    
    # Create context and socket
    context = zmq.Context.instance()
    socket = context.socket(zmq.REQ)
    
    try:
        # Connect to port
        socket.connect(f"tcp://{host}:{port}")
        logger.info(f"Connected to {host}:{port}")
        
        # Create a simple message
        message = {"command": "ping", "timestamp": int(time.time() * 1000)}
        message_bytes = json.dumps(message).encode('utf-8')
        
        # Send message
        logger.info(f"Sending: {message}")
        socket.send(message_bytes)
        
        # Set non-blocking receive
        socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5 second timeout
        
        # Receive response
        try:
            response = socket.recv()
            logger.info(f"Received {len(response)} bytes: {response!r}")
            return True
        except zmq.ZMQError as e:
            logger.error(f"Error receiving: {e}")
            return False
            
    except Exception as e:
        logger.error(f"Error: {e}")
        return False
    finally:
        socket.close()

def test_dealer_router(host="127.0.0.1", port=5559, identity="test-client"):
    """Test DEALER/ROUTER pattern on the given port."""
    logger.info(f"Testing DEALER/ROUTER pattern on {host}:{port}")
    
    # Create context and socket
    context = zmq.Context.instance()
    socket = context.socket(zmq.DEALER)
    
    try:
        # Set socket identity
        socket.setsockopt(zmq.IDENTITY, identity.encode('utf-8'))
        
        # Connect to port
        socket.connect(f"tcp://{host}:{port}")
        logger.info(f"Connected to {host}:{port} with identity {identity}")
        
        # Create a simple message
        message = {"command": "ping", "timestamp": int(time.time() * 1000)}
        message_bytes = json.dumps(message).encode('utf-8')
        
        # Send message with empty delimiter frame
        logger.info(f"Sending: {message}")
        socket.send_multipart([b"", message_bytes])
        
        # Set non-blocking receive
        socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5 second timeout
        
        # Receive response
        try:
            response_parts = socket.recv_multipart()
            logger.info(f"Received {len(response_parts)} parts:")
            for i, part in enumerate(response_parts):
                logger.info(f"  Part {i}: {len(part)} bytes: {part!r}")
            return True
        except zmq.ZMQError as e:
            logger.error(f"Error receiving: {e}")
            return False
            
    except Exception as e:
        logger.error(f"Error: {e}")
        return False
    finally:
        socket.close()

def test_sub(host="127.0.0.1", port=5556, topic=""):
    """Test SUB pattern on the given port."""
    logger.info(f"Testing SUB pattern on {host}:{port}")
    
    # Create context and socket
    context = zmq.Context.instance()
    socket = context.socket(zmq.SUB)
    
    try:
        # Subscribe to topic (empty means all messages)
        socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        
        # Connect to port
        socket.connect(f"tcp://{host}:{port}")
        logger.info(f"Connected to {host}:{port}, subscribed to topic '{topic}'")
        
        # Set non-blocking receive
        socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5 second timeout
        
        # Try to receive a message
        try:
            message = socket.recv()
            logger.info(f"Received {len(message)} bytes: {message!r}")
            return True
        except zmq.ZMQError as e:
            logger.error(f"Error receiving: {e}")
            return False
            
    except Exception as e:
        logger.error(f"Error: {e}")
        return False
    finally:
        socket.close()

def test_pull(host="127.0.0.1", port=5557):
    """Test PULL pattern on the given port."""
    logger.info(f"Testing PULL pattern on {host}:{port}")
    
    # Create context and socket
    context = zmq.Context.instance()
    socket = context.socket(zmq.PULL)
    
    try:
        # Connect to port
        socket.connect(f"tcp://{host}:{port}")
        logger.info(f"Connected to {host}:{port}")
        
        # Set non-blocking receive
        socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5 second timeout
        
        # Try to receive a message
        try:
            message = socket.recv()
            logger.info(f"Received {len(message)} bytes: {message!r}")
            return True
        except zmq.ZMQError as e:
            logger.error(f"Error receiving: {e}")
            return False
            
    except Exception as e:
        logger.error(f"Error: {e}")
        return False
    finally:
        socket.close()

def main():
    """Run tests on all ports."""
    # Test REQ/REP on port 5555 (default)
    test_req_rep()
    
    # Test DEALER/ROUTER on control port 5559
    test_dealer_router(port=5559)
    
    # Test DEALER/ROUTER on control port 5560
    test_dealer_router(port=5560)
    
    # Test SUB on pub port 5556
    test_sub(port=5556)
    
    # Test PULL on push port 5557
    test_pull(port=5557)
    
    # Test DEALER/ROUTER on sensorimotor port 5558
    test_dealer_router(port=5558, identity="sensory-test")

if __name__ == "__main__":
    main() 