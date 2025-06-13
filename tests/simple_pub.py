#!/usr/bin/env python3
"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
Simple ZMQ PUB Test Server

This script creates a simple ZMQ PUB socket and sends test messages
on port 5570 to help diagnose visualization connection issues.
"""

import logging
import random
import time

import zmq

# Set up logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)

logger = logging.getLogger("simple_pub")


def main():
    """Run a simple ZMQ PUB server that sends test messages."""
    port = 5570
    host = "*"  # Bind to all interfaces
    bind_address = f"tcp://{host}:{port}"

    logger.info(f"Starting simple ZMQ PUB server on {bind_address}")

    # Create a standard ZMQ context and PUB socket
    context = zmq.Context()
    socket = context.socket(zmq.PUB)

    # Set socket options
    socket.setsockopt(zmq.LINGER, 0)

    # Bind to the address
    logger.info(f"Binding to {bind_address}")
    socket.bind(bind_address)

    # Give the socket time to bind
    logger.info("Waiting for binding to complete...")
    time.sleep(1.0)

    # Send messages periodically
    logger.info("Starting message loop")
    message_count = 0

    try:
        while True:
            # Send a system message
            message_count += 1
            system_msg = f"TEST_SYSTEM_MESSAGE:{message_count}"
            logger.info(f"Sending system message: {system_msg}")
            socket.send_multipart([b"system", system_msg.encode("utf-8")])

            # Send an activity message with random data
            activity_size = random.randint(16, 64)
            activity_data = bytes(
                [random.randint(0, 255) for _ in range(activity_size)]
            )
            logger.info(f"Sending activity message: {activity_size} bytes")
            socket.send_multipart([b"activity", activity_data])

            # Send a test message on topic 'test'
            test_msg = f"TEST_MESSAGE:{message_count}"
            logger.info(f"Sending test message: {test_msg}")
            socket.send_multipart([b"test", test_msg.encode("utf-8")])

            # Wait before sending the next batch of messages
            logger.info(f"Messages sent. Total count: {message_count}")
            time.sleep(2.0)

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        # Clean up
        socket.close()
        context.term()
        logger.info(f"Server closed. Sent {message_count} message batches.")


if __name__ == "__main__":
    main()
