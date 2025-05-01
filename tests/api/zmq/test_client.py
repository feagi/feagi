#!/usr/bin/env python
"""Test client for connecting to a running FEAGI ZMQ server."""
import logging
import time
import sys
import json

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("test_feagi_client")

def main():
    """Connect to a running FEAGI ZMQ server and test messaging."""
    # Import client module
    from feagi.zmq import create_zmq_client
    
    # Create client
    logger.info("Creating ZMQ client to connect to running FEAGI...")
    client = create_zmq_client(
        host="127.0.0.1",
        pub_port=5556,
        sub_port=5557,
        topics=["test", "neural", "metrics", "heartbeat"]
    )
    
    if client is None or not client.is_healthy():
        logger.error("Failed to create ZMQ client")
        return 1
        
    logger.info("ZMQ client connected to FEAGI successfully")
    
    # Setup callbacks
    received = {"test": [], "neural": [], "metrics": [], "heartbeat": []}
    
    def callback_factory(topic_name):
        def callback(topic, data):
            received[topic_name].append((topic, data))
            logger.info(f"Received {topic_name} message: {data}")
        return callback
    
    # Subscribe to topics
    for topic in ["test", "neural", "metrics", "heartbeat"]:
        logger.info(f"Subscribing to {topic} topic...")
        client.subscribe(topic, callback_factory(topic))
    
    # Wait for subscriptions to be established
    time.sleep(2)
    
    # Publish a test message
    logger.info("Publishing test message...")
    test_message = {
        "message": "Test message from external client",
        "timestamp": time.time()
    }
    client.publish("test", test_message)
    
    # Wait and monitor messages for 10 seconds
    logger.info("Monitoring for messages for 10 seconds...")
    start_time = time.time()
    while time.time() - start_time < 10:
        message_count = sum(len(msgs) for msgs in received.values())
        sys.stdout.write(f"\rReceived {message_count} total messages so far...")
        sys.stdout.flush()
        time.sleep(0.5)
    print()  # Newline after progress
    
    # Print summary
    logger.info("Test complete. Message summary:")
    for topic, messages in received.items():
        logger.info(f"  {topic}: {len(messages)} messages")
    
    # Clean up
    logger.info("Shutting down client...")
    client.shutdown()
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 