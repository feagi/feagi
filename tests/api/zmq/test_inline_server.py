#!/usr/bin/env python
"""Test script for inline ZMQ server."""
import logging
import time
import sys
import os
import threading

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("test_inline")

def main():
    """Test the inline ZMQ server functionality."""
    # Import modules
    from feagi.core.zmq.server import ZMQServer
    from feagi.core.zmq.client import ZMQClient
    
    # Create and start server
    logger.info("Creating ZMQ server...")
    server = ZMQServer(
        host="127.0.0.1", 
        pub_port=5556, 
        sub_port=5557,
        topics=["test"]
    )
    
    # Start server
    logger.info("Starting ZMQ server...")
    if not server.start():
        logger.error("Failed to start ZMQ server")
        return 1
    
    logger.info("ZMQ server started successfully")
    
    # Wait for server to fully initialize
    time.sleep(1)
    
    # Create a client to connect to the server
    logger.info("Creating ZMQ client...")
    client = ZMQClient(
        host="127.0.0.1",
        pub_port=5556,
        sub_port=5557,
        topics=["test"]
    )
    
    # Start client
    logger.info("Starting ZMQ client...")
    if not client.start():
        logger.error("Failed to start ZMQ client")
        server.shutdown()
        return 1
    
    logger.info("ZMQ client started successfully")
    
    # Setup message callback
    received = []
    def message_callback(topic, data):
        received.append((topic, data))
        logger.info(f"Received message on topic '{topic}': {data}")
    
    # Subscribe to test topic
    logger.info("Subscribing to test topic...")
    client.subscribe("test", message_callback)
    
    # Wait for subscription to be established
    time.sleep(2)
    
    # Publish test message from server
    logger.info("Publishing test message from server...")
    server.publish("test", {"message": "Test from server", "timestamp": time.time()})
    
    # Wait a bit
    time.sleep(2)
    
    # Publish test message from client
    logger.info("Publishing test message from client...")
    client.publish("test", {"message": "Test from client", "timestamp": time.time()})
    
    # Wait for message processing
    logger.info("Waiting for message processing...")
    time.sleep(3)
    
    # Check received messages
    logger.info(f"Received {len(received)} messages")
    for i, (topic, data) in enumerate(received):
        logger.info(f"Message {i+1}: {topic} -> {data}")
    
    # Shutdown
    logger.info("Shutting down client and server...")
    client.shutdown()
    server.shutdown()
    logger.info("Test complete")
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 