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
Clean Test Script for FEAGI Visualization

This script tests the visualization stream with a clean setup using a different port
to avoid conflicts with any existing sockets.
"""

import asyncio
import logging
import os
import sys
import threading
import time

import zmq
import zmq.asyncio

# Add the parent directory to the path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# Set up logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("clean_test")

# ZMQ setup
TEST_PORT = 5571  # Use a different port to avoid conflicts


class TestPublisher:
    """A simple publisher that sends test messages on different topics."""

    def __init__(self, port=TEST_PORT):
        self.port = port
        self.running = False
        self.context = zmq.asyncio.Context()
        self.socket = None

    async def start(self):
        """Start the publisher."""
        # Create and configure socket
        self.socket = self.context.socket(zmq.PUB)
        bind_addr = f"tcp://*:{self.port}"
        logger.info(f"Binding publisher to {bind_addr}")
        self.socket.bind(bind_addr)

        # Mark as running
        self.running = True
        logger.info("Publisher started")

        # Wait for connections to establish
        await asyncio.sleep(1.0)

    async def stop(self):
        """Stop the publisher and clean up resources."""
        if self.socket:
            # Close the socket properly
            try:
                self.socket.close(linger=0)
                logger.info("Socket closed")
            except Exception as e:
                logger.error(f"Error closing socket: {e}")

        self.running = False
        logger.info("Publisher stopped")

    async def send_test_messages(self, count=30):
        """Send a series of test messages on multiple topics."""
        if not self.running or not self.socket:
            logger.error("Cannot send messages - publisher not started")
            return

        logger.info(f"Sending {count} test message batches")

        for i in range(1, count + 1):
            # Send system message
            system_msg = f"TEST_SYSTEM_MESSAGE:{i}".encode("utf-8")
            await self.socket.send_multipart([b"system", system_msg])

            # Send activity message (with random binary data to simulate neuron data)
            # Here we're sending structured data that matches NEURON_CATEGORIES_ID format
            import os

            data_length = 20 + (i % 10)  # Variable length messages
            activity_data = bytes([11, 1, 0, 1]) + os.urandom(
                data_length
            )  # 11 = NEURON_CATEGORIES_ID
            await self.socket.send_multipart([b"activity", activity_data])

            # Log what we sent
            logger.info(
                f"Sent message batch #{i}: system ({len(system_msg)} bytes), activity ({len(activity_data)} bytes)"
            )

            # Sleep between batches
            await asyncio.sleep(1.0)

        logger.info(f"Finished sending {count} test message batches")


class TestSubscriber:
    """A simple subscriber that receives and logs messages."""

    def __init__(self, port=TEST_PORT, host="localhost"):
        self.port = port
        self.host = host
        self.running = False
        self.context = zmq.Context()
        self.socket = None
        self.message_count = 0
        self.activity_count = 0

    def start(self):
        """Start the subscriber in a synchronous context."""
        # Create and configure socket
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)

        # Set socket options
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout

        # Connect to publisher
        connect_addr = f"tcp://{self.host}:{self.port}"
        logger.info(f"Connecting subscriber to {connect_addr}")
        self.socket.connect(connect_addr)

        # Subscribe to topics
        self.socket.setsockopt(zmq.SUBSCRIBE, b"system")
        self.socket.setsockopt(zmq.SUBSCRIBE, b"activity")
        logger.info(f"Subscribed to 'system' and 'activity' topics")

        # Mark as running
        self.running = True
        logger.info("Subscriber started")

    def stop(self):
        """Stop the subscriber and clean up resources."""
        if self.socket:
            self.socket.close(linger=0)

        if self.context:
            self.context.term()

        self.running = False
        logger.info(
            f"Subscriber stopped. Received {self.message_count} total messages ({self.activity_count} activity messages)"
        )

    def run(self, max_time=30):
        """Run the subscriber main loop for up to max_time seconds."""
        if not self.running:
            self.start()

        start_time = time.time()
        logger.info(
            f"Starting subscriber main loop (will run for up to {max_time} seconds)"
        )

        try:
            while self.running and (time.time() - start_time < max_time):
                try:
                    # Try to receive a message
                    message = self.socket.recv_multipart()
                    self.message_count += 1

                    # Process the message based on topic
                    if len(message) >= 2:
                        topic = message[0]
                        data = message[1]

                        if topic == b"system":
                            # Handle system messages
                            try:
                                system_msg = data.decode("utf-8")
                                logger.info(f"Received system message: {system_msg}")
                            except:
                                logger.warning(
                                    f"Received binary system message: {len(data)} bytes"
                                )

                        elif topic == b"activity":
                            # Handle activity messages
                            self.activity_count += 1
                            logger.info(
                                f"Received activity message #{self.activity_count}: {len(data)} bytes"
                            )

                            # Log the first few bytes as hex
                            hex_dump = " ".join([f"{b:02x}" for b in data[:16]])
                            logger.info(f"Activity data header: {hex_dump}")

                            # Parse the data format
                            if len(data) > 0:
                                format_id = data[0] if len(data) > 0 else None
                                version = data[1] if len(data) > 1 else None
                                logger.info(
                                    f"Activity data format: ID={format_id}, version={version}"
                                )

                        else:
                            # Unknown topic
                            logger.warning(
                                f"Received message with unknown topic: {topic}"
                            )

                except zmq.Again:
                    # Timeout - no message available
                    elapsed = time.time() - start_time
                    if int(elapsed) % 5 == 0:  # Log every 5 seconds
                        logger.info(
                            f"Waiting for messages... ({elapsed:.1f}s elapsed, {self.message_count} received)"
                        )
                    time.sleep(0.1)  # Small sleep to avoid tight loop

        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        finally:
            self.stop()


async def run_publisher_test():
    """Run a test with just the publisher sending messages."""
    publisher = TestPublisher()

    try:
        # Start the publisher
        await publisher.start()

        # Send test messages
        await publisher.send_test_messages(10)

    finally:
        # Clean up
        await publisher.stop()


def run_subscriber_in_thread(duration=15):
    """Run a subscriber in a separate thread."""
    subscriber = TestSubscriber()

    # Start the subscriber in a thread
    thread = threading.Thread(target=subscriber.run, args=(duration,))
    thread.daemon = True
    thread.start()

    # Return the thread and subscriber
    return thread, subscriber


async def run_combined_test():
    """Run a test with both publisher and subscriber."""
    # Start the subscriber in a thread
    sub_thread, subscriber = run_subscriber_in_thread(20)

    # Give the subscriber time to connect
    await asyncio.sleep(1.0)

    # Create and start the publisher
    publisher = TestPublisher()
    await publisher.start()

    # Send test messages
    await publisher.send_test_messages(15)

    # Clean up publisher
    await publisher.stop()

    # Wait for subscriber thread to finish
    sub_thread.join()

    # Return results
    return subscriber.message_count, subscriber.activity_count


if __name__ == "__main__":
    # Check if we should run publisher, subscriber, or combined test
    import sys

    mode = "combined"
    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()

    if mode == "publisher":
        # Run just the publisher
        asyncio.run(run_publisher_test())

    elif mode == "subscriber":
        # Run just the subscriber
        subscriber = TestSubscriber()
        subscriber.run(max_time=30)

    else:
        # Run combined test
        logger.info("Running combined publisher/subscriber test")
        result = asyncio.run(run_combined_test())
        logger.info(
            f"Test complete. Received {result[0]} total messages ({result[1]} activity messages)"
        )
