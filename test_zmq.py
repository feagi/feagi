#!/usr/bin/env python
"""Test script for ZMQ client functionality."""
import time
from feagi.zmq import create_zmq_client

def main():
    # Create a ZMQ client
    print("Creating ZMQ client...")
    client = create_zmq_client()
    
    if client is None or not client.is_healthy():
        print("Failed to connect to ZMQ server")
        return
        
    print("ZMQ client connected successfully")
    
    # Create a list to track received messages
    received = []
    
    # Define a callback for received messages
    def message_callback(topic, data):
        received.append((topic, data))
        print(f"Received message on topic '{topic}': {data}")
    
    # Subscribe to a test topic
    print("Subscribing to 'test' topic...")
    client.subscribe("test", message_callback)
    
    # Wait to ensure subscription is processed
    time.sleep(1)
    
    # Publish a test message
    print("Publishing test message...")
    client.publish("test", {"message": "This is a test", "timestamp": time.time()})
    
    # Wait to receive the message
    print("Waiting for messages...")
    time.sleep(3)
    
    # Check if messages were received
    print(f"Total messages received: {len(received)}")
    
    # Cleanup
    print("Shutting down client...")
    client.shutdown()
    print("Test complete")

if __name__ == "__main__":
    main() 