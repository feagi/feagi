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
    """Test basic initialization of the shared memory gateway."""
    try:
        # Create a temporary directory for shared memory files
        with tempfile.TemporaryDirectory() as temp_dir:
            logger.info(f"Using temporary directory: {temp_dir}")
            
            # Initialize gateway
            gateway = SharedMemoryFEAGIGateway(
                process_name="test_process",
                temp_dir=temp_dir
            )
            
            logger.info("SharedMemoryFEAGIGateway initialized successfully")
            
            # Test config dictionary
            gateway.config_dict.set("test_key", "test_value")
            value = gateway.config_dict.get("test_key")
            assert value == "test_value", f"Expected 'test_value', got {value}"
            logger.info("Config dictionary test passed")
            
            # Test event system
            event_received = False
            
            def event_handler(event):
                nonlocal event_received
                event_received = True
            
            gateway.event_system.register_handler(EventType.CONFIG_UPDATED, event_handler)
            gateway.event_system.send_event(EventType.CONFIG_UPDATED, {"test": True})
            
            # Wait for event to be processed
            time.sleep(0.5)
            assert event_received, "Event was not received"
            logger.info("Event system test passed")
            
            # Clean up
            gateway.shutdown()
            logger.info("Gateway shutdown successfully")
            return True
    except Exception as e:
        logger.error(f"Test failed: {e}", exc_info=True)
        return False

def test_burst_engine_config():
    """Test getting and setting burst engine configuration."""
    try:
        # Create a temporary directory for shared memory files
        with tempfile.TemporaryDirectory() as temp_dir:
            logger.info(f"Using temporary directory: {temp_dir}")
            
            # Initialize gateway
            gateway = SharedMemoryFEAGIGateway(
                process_name="test_process",
                temp_dir=temp_dir
            )
            
            # Set burst engine config
            test_config = {
                "burst_duration": 15,
                "inter_burst_interval": 7,
                "maximum_firing_rate": 120,
                "threshold": 0.6
            }
            
            result = gateway.set_burst_engine_config(test_config)
            assert result, "Failed to set burst engine config"
            
            # Get burst engine config
            config = gateway.get_burst_engine_config()
            assert config.get("burst_duration") == 15, f"Expected burst_duration=15, got {config.get('burst_duration')}"
            assert config.get("inter_burst_interval") == 7, f"Expected inter_burst_interval=7, got {config.get('inter_burst_interval')}"
            
            logger.info("Burst engine config test passed")
            
            # Clean up
            gateway.shutdown()
            return True
    except Exception as e:
        logger.error(f"Test failed: {e}", exc_info=True)
        return False

def reader_process(temp_dir, result_queue, ready_event):
    import logging
    logger = logging.getLogger("reader_process")
    try:
        logger.info("Reader process started")
        gateway = SharedMemoryFEAGIGateway(process_name="reader", temp_dir=temp_dir)
        event_received = multiprocessing.Event()
        value_holder = multiprocessing.Manager().dict()
        def event_handler(event):
            if event.data.get("integration"):
                event_received.set()
        gateway.event_system.register_handler(EventType.CONFIG_UPDATED, event_handler)
        # Signal to the writer that the reader is ready
        ready_event.set()
        # Wait for the value and event
        timeout = 2.0
        waited = 0.0
        poll_interval = 0.05
        value = None
        while waited < timeout:
            value = gateway.config_dict.get("integration_key")
            if value == "integration_value" and event_received.is_set():
                break
            import time
            time.sleep(poll_interval)
            waited += poll_interval
        gateway.shutdown()
        logger.info(f"Reader process finished, value={value}, event_received={event_received.is_set()}")
        result_queue.put((value, event_received.is_set()))
    except Exception as e:
        logger.error(f"Reader process exception: {e}", exc_info=True)
        result_queue.put((None, False))

def writer_process(temp_dir, ready_event):
    import logging
    logger = logging.getLogger("writer_process")
    try:
        logger.info("Writer process started")
        # Wait for the reader to be ready
        ready_event.wait(timeout=2.0)
        gateway = SharedMemoryFEAGIGateway(process_name="writer", temp_dir=temp_dir)
        gateway.config_dict.set("integration_key", "integration_value")
        gateway.event_system.send_event(EventType.CONFIG_UPDATED, {"integration": True})
        gateway.shutdown()
        logger.info("Writer process finished and shutdown")
    except Exception as e:
        logger.error(f"Writer process exception: {e}", exc_info=True)

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
        assert event_received, "Integration event was not received"
        logger.info("Shared memory integration test passed")

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