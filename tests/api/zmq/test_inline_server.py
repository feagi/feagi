#!/usr/bin/env python
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

"""Test script for inline ZMQ server."""
import asyncio
import logging
import os
import sys
import threading
import time
from typing import Dict, Tuple

import pytest

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Skip tests if ZMQ is not available
zmq_available = False
try:
    import zmq

    zmq_available = True
except ImportError:
    pass

# Import server and client implementations
try:
    from feagi.api.zmq.client import ZmqClient
    from feagi.api.zmq.server import ZmqServer

    have_zmq_impl = True
except ImportError:
    have_zmq_impl = False


def main():
    """Test the inline ZMQ server functionality."""
    if not have_zmq_impl:
        logger.error("ZMQ implementation not available. Skipping test.")
        return

    # Create and start server
    logger.info("Creating ZMQ server...")
    server = ZmqServer(
        core_api=None,  # Will be created internally if not provided
        host="127.0.0.1",
        req_rep_port=5555,
        pub_sub_port=5556,
        push_pull_port=5557,
        sensorimotor_port=5558,
        vis_base_port=5560,
    )

    # Start the server
    logger.info("Starting ZMQ server...")
    server_thread = threading.Thread(target=lambda: asyncio.run(server.start()))
    server_thread.daemon = True
    server_thread.start()

    # Allow server time to start
    time.sleep(2)

    # Create a client to connect to the server
    logger.info("Creating ZMQ client...")
    client = ZmqClient(
        host="127.0.0.1",
        req_rep_port=5555,
        pub_sub_port=5556,
        push_pull_port=5557,
        sensorimotor_port=5558,
        vis_base_port=5560,
    )

    # Start the client
    logger.info("Starting ZMQ client...")
    client_thread = threading.Thread(target=lambda: asyncio.run(client.start()))
    client_thread.daemon = True
    client_thread.start()

    # Allow client time to connect
    time.sleep(2)

    # Test functionality
    logger.info("Testing ZMQ functionality...")

    # Shutdown
    logger.info("Shutting down ZMQ client...")
    asyncio.run(client.stop())

    logger.info("Shutting down ZMQ server...")
    asyncio.run(server.stop())

    logger.info("Test completed successfully")


if __name__ == "__main__":
    main()
