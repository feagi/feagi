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
Simplified test script to verify binary sensorimotor data transmission to FEAGI.

This script focuses solely on sending binary-encoded sensorimotor data to FEAGI
using the ByteStructureTranslator for proper binary encoding.
"""

import asyncio
import logging
import sys

import zmq
import zmq.asyncio
from feagi_bytes import ByteStructureTranslator

# Set up logging
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("feagi_sensorimotor_test.log"),
    ],
)
logger = logging.getLogger("feagi_sensorimotor_test")


async def test_sensorimotor_binary_transmission():
    """Test sending binary sensorimotor data to FEAGI."""

    # Get ZMQ context
    context = zmq.asyncio.Context.instance()

    # Initialize the binary translator
    translator = ByteStructureTranslator()

    # Sensorimotor port configuration
    sensorimotor_port = 5558

    # Create DEALER socket for sensorimotor communication
    sm_socket = None
    try:
        logger.info(f"Connecting to sensorimotor port {sensorimotor_port}...")
        sm_socket = context.socket(zmq.DEALER)
        sm_socket.connect(f"tcp://localhost:{sensorimotor_port}")

        # Create sample sensory data
        sensory_data = {
            "type": "sensory_data",
            "channel_id": "iv00_C",  # Use correct cortical ID from the essential genome
            "data": [0.1, 0.2, 0.3, 0.4, 0.5],  # Sample neural activation values
        }

        # Convert to binary format
        binary_data = translator.create_message(sensory_data)
        logger.info(f"Created binary sensory data: {len(binary_data)} bytes")

        # Send properly formatted binary sensory message
        # First frame: empty identity (required for DEALER socket)
        # Second frame: topic/command ("sensory" in this case)
        # Third frame: actual binary data
        await sm_socket.send_multipart([b"", b"sensory", binary_data])
        logger.info("Binary sensorimotor data sent successfully")

        # Wait briefly to allow the message to be processed
        await asyncio.sleep(1)

        # No response is expected from the sensorimotor socket
        logger.info("Sensorimotor binary data transmission test completed")

    except Exception as e:
        logger.error(f"Error during sensorimotor data transmission: {e}")
    finally:
        if sm_socket:
            sm_socket.close()


if __name__ == "__main__":
    logger.info("=== Starting FEAGI binary sensorimotor data test ===")
    asyncio.run(test_sensorimotor_binary_transmission())
    logger.info("=== Test completed ===")
