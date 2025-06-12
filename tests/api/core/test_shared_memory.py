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

"""
Test script for the SharedMemoryFEAGIGateway
"""

import logging
import multiprocessing
import os
import sys
import tempfile
import threading
import time
from unittest.mock import MagicMock, patch

import pytest

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("test_shared_memory")

from feagi.api.shared_memory.data_structures import SharedConfigDict
from feagi.api.shared_memory.events import EventNotificationSystem, EventType

# Import shared memory components
from feagi.api.shared_memory.feagi_gateway import SharedMemoryFEAGIGateway
from feagi.api.shared_memory.manager import SharedMemoryManager


def test_basic_initialization():
    """Test basic initialization of shared memory components."""
    with tempfile.TemporaryDirectory() as temp_dir:
        gateway = SharedMemoryFEAGIGateway(process_name="test", temp_dir=temp_dir)

        # Verify the gateway was initialized
        assert gateway is not None

        # Verify memory manager was initialized
        assert gateway.memory_manager is not None

        # Verify configuration dictionary was created
        assert gateway.config_dict is not None

        # Clean up
        gateway.shutdown()

    return False  # TODO: Convert this to assert statements


def test_burst_engine_config():
    """Test get/set burst engine configuration via shared memory."""
    with tempfile.TemporaryDirectory() as temp_dir:
        gateway = SharedMemoryFEAGIGateway(process_name="test", temp_dir=temp_dir)

        # Set a test config
        test_config = {"burst_duration": 20, "max_firing_rate": 200}
        result = gateway.set_burst_engine_config(test_config)

        # Verify the config was set
        assert result is True

        # Get the config back and verify it matches
        config = gateway.get_burst_engine_config()
        assert config.get("burst_duration") == 20
        assert config.get("max_firing_rate") == 200

        # Clean up
        gateway.shutdown()

    return False  # TODO: Convert this to assert statements


def reader_process(temp_dir, result_queue, ready_event):
    """Process function for testing shared memory reading."""
    try:
        # This is a fresh process, so we need all imports
        import logging
        import sys
        import time

        logging.basicConfig(level=logging.INFO)
        from feagi.api.shared_memory.feagi_gateway import SharedMemoryFEAGIGateway

        print("Reader process started", file=sys.stderr)

        # Create gateway with minimal functionality (avoid event system issues)
        from feagi.api.shared_memory.data_structures import SharedConfigDict
        from feagi.api.shared_memory.manager import SharedMemoryManager

        # Initialize the components directly to avoid event system problems
        memory_manager = SharedMemoryManager(temp_dir=temp_dir)
        config_dict = SharedConfigDict("feagi_config", manager=memory_manager)

        # Signal that we're ready
        ready_event.set()

        # Wait for data to appear (poll)
        max_attempts = 10
        value = None
        event_received = False

        for _ in range(max_attempts):
            # Check for value in shared memory
            test_value = config_dict.get("test_key")
            if test_value == "integration_value":
                value = test_value
                break
            time.sleep(0.5)

        # Clean up (avoid using shutdown which might reference event_system)
        config_dict.region.close()

        # Return result
        result_queue.put((value, event_received))

    except Exception as e:
        import traceback

        print(f"Reader process exception: {e}", file=sys.stderr)
        print(traceback.format_exc(), file=sys.stderr)
        result_queue.put((None, False))


def writer_process(temp_dir, ready_event):
    """Process function for testing shared memory writing."""
    try:
        # This is a fresh process, so we need all imports
        import logging
        import sys
        import time

        logging.basicConfig(level=logging.INFO)

        print("Writer process started", file=sys.stderr)

        # Wait for reader to start
        ready_event.wait(timeout=5)

        # Initialize the components directly to avoid event system problems
        from feagi.api.shared_memory.data_structures import SharedConfigDict
        from feagi.api.shared_memory.manager import SharedMemoryManager

        memory_manager = SharedMemoryManager(temp_dir=temp_dir)
        config_dict = SharedConfigDict("feagi_config", manager=memory_manager)

        # Write a test value using the set method instead of dictionary syntax
        config_dict.set("test_key", "integration_value")

        # Clean up
        config_dict.region.close()

    except Exception as e:
        import traceback

        print(f"Writer process exception: {e}", file=sys.stderr)
        print(traceback.format_exc(), file=sys.stderr)


def test_shared_memory_integration():
    """Integration test: Simulate two processes communicating via shared memory."""
    with tempfile.TemporaryDirectory() as temp_dir:
        result_queue = multiprocessing.Queue()
        ready_event = multiprocessing.Event()
        # Start reader process first
        p_reader = multiprocessing.Process(
            target=reader_process, args=(temp_dir, result_queue, ready_event)
        )
        p_reader.start()
        # Start writer process after reader signals readiness
        p_writer = multiprocessing.Process(
            target=writer_process, args=(temp_dir, ready_event)
        )
        p_writer.start()
        p_writer.join()
        p_reader.join()
        value, event_received = result_queue.get()
        assert value == "integration_value", (
            f"Expected 'integration_value', got {value}"
        )


if __name__ == "__main__":
    print("Testing SharedMemoryFEAGIGateway...")

    if test_basic_initialization():
        print("✅ Basic initialization test passed")
    else:
        print("❌ Basic initialization test failed")

    if test_burst_engine_config():
        print("✅ Burst engine config test passed")
    else:
        print("❌ Burst engine config test failed")

    if test_shared_memory_integration():
        print("✅ Shared memory integration test passed")
    else:
        print("❌ Shared memory integration test failed")
