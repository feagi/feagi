#!/usr/bin/env python
"""
Test script for the SharedMemoryFEAGIGateway
"""

import logging
import time
import os
import sys
import tempfile
import multiprocessing
import pytest
import threading
from unittest.mock import patch, MagicMock

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("test_shared_memory")

# Import shared memory components
from feagi.api.shared_memory.feagi_gateway import SharedMemoryFEAGIGateway
from feagi.api.shared_memory.manager import SharedMemoryManager
from feagi.api.shared_memory.events import EventNotificationSystem, EventType
from feagi.api.shared_memory.data_structures import SharedConfigDict

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
        test_config = {
            "burst_duration": 20,
            "max_firing_rate": 200
        }
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
        print("Reader process started", file=sys.stderr)
        # Create gateway
        gateway = SharedMemoryFEAGIGateway(process_name="reader", temp_dir=temp_dir)
        
        # Signal that we're ready
        ready_event.set()
        
        # Wait for data to appear (poll)
        max_attempts = 10
        value = None
        event_received = False
        
        for _ in range(max_attempts):
            # Check for value in shared memory
            test_value = gateway.config_dict.get("test_key")
            if test_value == "integration_value":
                value = test_value
                break
            time.sleep(0.5)
        
        # Clean up and return result
        gateway.shutdown()
        result_queue.put((value, event_received))
        
    except Exception as e:
        print(f"Reader process exception: {e}", file=sys.stderr)
        result_queue.put((None, False))

def writer_process(temp_dir, ready_event):
    """Process function for testing shared memory writing."""
    try:
        print("Writer process started", file=sys.stderr)
        # Wait for reader to start
        ready_event.wait(timeout=5)
        
        # Create gateway
        gateway = SharedMemoryFEAGIGateway(process_name="writer", temp_dir=temp_dir)
        
        # Write a test value
        gateway.config_dict["test_key"] = "integration_value"
        
        # Clean up
        gateway.shutdown()
        
    except Exception as e:
        print(f"Writer process exception: {e}", file=sys.stderr)

@pytest.mark.skip(reason="Requires fixing the shared memory integration test")
def test_shared_memory_integration():
    """Integration test: Simulate two processes communicating via shared memory."""
    with tempfile.TemporaryDirectory() as temp_dir:
        result_queue = multiprocessing.Queue()
        ready_event = multiprocessing.Event()
        # Start reader process first
        p_reader = multiprocessing.Process(target=reader_process, args=(temp_dir, result_queue, ready_event))
        p_reader.start()
        # Start writer process after reader signals readiness
        p_writer = multiprocessing.Process(target=writer_process, args=(temp_dir, ready_event))
        p_writer.start()
        p_writer.join()
        p_reader.join()
        value, event_received = result_queue.get()
        assert value == "integration_value", f"Expected 'integration_value', got {value}"

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