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
Simplified test script to verify FEAGI connection using feagi_data_processing.
"""

import asyncio
import logging
import sys

import zmq
import zmq.asyncio

# Import the new feagi_data_processing library via the compatibility layer
from feagi.api.protocols import ByteStructureTranslator

# Set up logging
logging.basicConfig(
    level=logging.DEBUG,  # Use DEBUG level to see all connection details
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("feagi_simple_test.log"),
    ],
)
logger = logging.getLogger("feagi_simple_test")


async def test_zmq_connection():
    """Test direct ZMQ connection to FEAGI."""

    # Get ZMQ context
    context = zmq.asyncio.Context.instance()

    # Initialize the translator (now uses feagi_data_processing internally)
    translator = ByteStructureTranslator()

    # Ports from running FEAGI instance
    req_port = 5555
    pub_port = 5556
    push_port = 5557
    sensorimotor_port = 5558
    vis_port = 5560

    # Test connections to each port
    logger.info("Attempting connections to FEAGI on localhost")

    # Create REQ socket (for port 5555)
    req_socket = None
    try:
        logger.info(f"Connecting to REQ port {req_port}...")
        req_socket = context.socket(zmq.REQ)
        req_socket.connect(f"tcp://localhost:{req_port}")
        req_socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5-second timeout

        # Send a status request in binary format
        status_request = {"type": "status_request"}
        binary_message = translator.create_message(status_request)
        logger.info(f"Sending binary message of {len(binary_message)} bytes")
        await req_socket.send(binary_message)

        # Wait for response with timeout
        try:
            binary_response = await asyncio.wait_for(req_socket.recv(), timeout=5.0)
            # Try to decode the response
            try:
                response = translator.decode_message(binary_response)
                logger.info(f"Decoded response: {response}")
            except Exception as e:
                logger.warning(f"Could not decode response as byte structure: {e}")
                # Try to parse as plain JSON
                try:
                    if binary_response.startswith(b"application/json"):
                        # It's using a multipart format, skip the first part
                        parts = binary_response.split(b"\n", 1)
                        if len(parts) > 1:
                            json_data = parts[1]
                        else:
                            json_data = b"{}"
                    else:
                        json_data = binary_response

                    import json

                    json_response = json.loads(json_data)
                    logger.info(f"Parsed JSON response: {json_response}")
                except Exception as json_err:
                    logger.warning(
                        f"Could not parse response as JSON either: {json_err}"
                    )
                    logger.info(f"Raw response: {binary_response[:100]}...")
        except asyncio.TimeoutError:
            logger.warning(f"No response from REQ port {req_port} after 5 seconds")
    except Exception as e:
        logger.error(f"Error connecting to REQ port {req_port}: {e}")
    finally:
        if req_socket:
            req_socket.close()

    # Create DEALER socket (for sensorimotor port 5558)
    sm_socket = None
    try:
        logger.info(f"Connecting to sensorimotor port {sensorimotor_port}...")
        sm_socket = context.socket(zmq.DEALER)
        sm_socket.connect(f"tcp://localhost:{sensorimotor_port}")

        # Send a properly formatted binary sensory message
        # First frame is identity (empty), second is delimiter, third is the actual data
        sensory_data = {
            "type": "sensory_data",
            "channel_id": "test_channel",
            "data": [0.1, 0.2, 0.3, 0.4, 0.5],  # Sample data
        }
        binary_data = translator.create_message(sensory_data)
        await sm_socket.send_multipart([b"", b"sensory", binary_data])

        logger.info(f"Binary sensory data sent to port {sensorimotor_port}")
    except Exception as e:
        logger.error(f"Error connecting to sensorimotor port {sensorimotor_port}: {e}")
    finally:
        if sm_socket:
            sm_socket.close()

    # Create DEALER socket (for visualization port 5560)
    vis_socket = None
    try:
        logger.info(f"Connecting to visualization port {vis_port}...")
        vis_socket = context.socket(zmq.DEALER)
        vis_socket.connect(f"tcp://localhost:{vis_port}")

        # Send a properly formatted binary visualization message
        vis_data = {"type": "visualization_request", "request_type": "structure"}
        binary_vis_data = translator.create_message(vis_data)
        await vis_socket.send_multipart([b"", binary_vis_data])

        logger.info(f"Binary visualization request sent to port {vis_port}")
    except Exception as e:
        logger.error(f"Error connecting to visualization port {vis_port}: {e}")
    finally:
        if vis_socket:
            vis_socket.close()

    logger.info("Connection tests completed")


if __name__ == "__main__":
    logger.info(
        "=== Starting simplified FEAGI connection test with feagi_data_processing ==="
    )
    asyncio.run(test_zmq_connection())
    logger.info("=== Test completed ===")
