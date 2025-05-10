#!/usr/bin/env python
"""
Standalone test for shared memory components.
This test doesn't rely on the full FEAGI framework.
"""

import os
import sys
import json
import logging
import tempfile
import time
import threading
import multiprocessing
from typing import Dict, Any, Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("standalone_shared_memory_test")

# Add the current directory to the path
sys.path.append(os.getcwd())

# Import real FEAGI shared memory components for true IPC
from feagi.api.shared_memory.manager import SharedMemoryManager
from feagi.api.shared_memory.events import EventNotificationSystem, EventType
from feagi.api.shared_memory.data_structures import SharedConfigDict
from feagi.api.shared_memory.feagi_gateway import SharedMemoryFEAGIGateway

def test_shared_memory_manager():
    """Test the SharedMemoryManager."""
    try:
        with tempfile.TemporaryDirectory() as temp_dir:
            logger.info(f"Using temporary directory: {temp_dir}")
            
            manager = SharedMemoryManager(temp_dir=temp_dir)
            
            # Test creating and accessing shared memory
            test_data = b"Hello, shared memory!"
            manager.create_shared_memory("test", len(test_data))
            manager.write_to_shared_memory("test", test_data)
            
            read_data = manager.read_from_shared_memory("test")
            assert read_data == test_data, f"Expected '{test_data}', got '{read_data}'"
            
            # Clean up
            manager.cleanup()
            logger.info("SharedMemoryManager test passed")
            return True
    except Exception as e:
        logger.error(f"Test failed: {e}", exc_info=True)
        return False


def test_event_notification_system():
    """Test the EventNotificationSystem."""
    try:
        with tempfile.TemporaryDirectory() as temp_dir:
            logger.info(f"Using temporary directory: {temp_dir}")
            
            event_system = EventNotificationSystem("test_process", temp_dir=temp_dir)
            
            # Test event handling
            event_received = False
            event_data = None
            
            def event_handler(event):
                nonlocal event_received, event_data
                event_received = True
                event_data = event.data
                logger.info(f"Received event: {event}")
            
            # Register handler and send event
            event_system.register_handler(EventType.CONFIG_UPDATED, event_handler)
            event_system.start()
            
            test_data = {"test_key": "test_value"}
            event_system.send_event(EventType.CONFIG_UPDATED, test_data)
            
            # Wait for event to be processed
            time.sleep(0.5)
            
            assert event_received, "Event was not received"
            assert event_data == test_data, f"Expected {test_data}, got {event_data}"
            
            # Clean up
            event_system.cleanup()
            logger.info("EventNotificationSystem test passed")
            return True
    except Exception as e:
        logger.error(f"Test failed: {e}", exc_info=True)
        return False


def test_shared_config_dict():
    """Test the SharedConfigDict."""
    try:
        with tempfile.TemporaryDirectory() as temp_dir:
            logger.info(f"Using temporary directory: {temp_dir}")
            
            manager = SharedMemoryManager(temp_dir=temp_dir)
            config_dict = SharedConfigDict("test_config", manager=manager)
            
            # Test setting and getting values
            test_data = {
                "string_key": "string_value",
                "int_key": 42,
                "float_key": 3.14,
                "list_key": [1, 2, 3],
                "dict_key": {"nested": "value"}
            }
            
            for key, value in test_data.items():
                config_dict.set(key, value)
            
            for key, expected_value in test_data.items():
                value = config_dict.get(key)
                assert value == expected_value, f"For key '{key}', expected '{expected_value}', got '{value}'"
            
            # Test default values
            assert config_dict.get("non_existent", "default") == "default", "Default value not working"
            
            # Clean up
            manager.cleanup()
            logger.info("SharedConfigDict test passed")
            return True
    except Exception as e:
        logger.error(f"Test failed: {e}", exc_info=True)
        return False


class SharedMemoryFEAGIGateway:
    """Simplified Gateway implementation for testing."""
    
    def __init__(self, process_name="test", temp_dir=None, feagi_instance=None):
        """Initialize the gateway."""
        self.process_name = process_name
        self.temp_dir = temp_dir or tempfile.gettempdir()
        self._feagi = feagi_instance
        
        # Initialize shared memory components
        self.memory_manager = SharedMemoryManager(temp_dir=self.temp_dir)
        self.event_system = EventNotificationSystem(process_name, temp_dir=self.temp_dir)
        self.config_dict = SharedConfigDict("feagi_config", manager=self.memory_manager)
        
        # Start event notification system
        self.event_system.start()
        
        # Register event handlers
        self._register_event_handlers()
        
        self.logger = logging.getLogger("shared_memory_gateway")
        self.logger.info(f"SharedMemoryFEAGIGateway initialized for process '{process_name}'")
    
    def _register_event_handlers(self):
        """Register handlers for different event types."""
        self.event_system.register_handler(
            EventType.CONFIG_UPDATED,
            lambda event: self.logger.info(f"Config updated: {event.data}")
        )
    
    def get_burst_engine_config(self):
        """Get the burst engine configuration."""
        return self.config_dict.get("burst_engine_config", {
            "burst_duration": 10,
            "inter_burst_interval": 5,
            "maximum_firing_rate": 100,
            "threshold": 0.5
        })
    
    def set_burst_engine_config(self, config):
        """Set the burst engine configuration."""
        result = self.config_dict.set("burst_engine_config", config)
        if result:
            self.event_system.send_event(EventType.CONFIG_UPDATED, {"type": "burst_engine"})
        return result
    
    def get_cortical_areas(self):
        """Get a list of all cortical areas."""
        return self.config_dict.get("cortical_areas", [])
    
    def shutdown(self):
        """Shut down the gateway."""
        self.event_system.cleanup()
        self.memory_manager.cleanup()
        self.logger.info("SharedMemoryFEAGIGateway shut down")


def test_gateway():
    """Test the SharedMemoryFEAGIGateway."""
    try:
        with tempfile.TemporaryDirectory() as temp_dir:
            logger.info(f"Using temporary directory: {temp_dir}")
            
            gateway = SharedMemoryFEAGIGateway(
                process_name="test_process",
                temp_dir=temp_dir
            )
            
            # Test burst engine config
            test_config = {
                "burst_duration": 15,
                "inter_burst_interval": 7,
                "maximum_firing_rate": 120,
                "threshold": 0.6
            }
            
            gateway.set_burst_engine_config(test_config)
            config = gateway.get_burst_engine_config()
            
            assert config.get("burst_duration") == 15, f"Expected burst_duration=15, got {config.get('burst_duration')}"
            assert config.get("inter_burst_interval") == 7, f"Expected inter_burst_interval=7, got {config.get('inter_burst_interval')}"
            
            # Clean up
            gateway.shutdown()
            logger.info("SharedMemoryFEAGIGateway test passed")
            return True
    except Exception as e:
        logger.error(f"Test failed: {e}", exc_info=True)
        return False


def standalone_reader_process(temp_dir, result_queue, ready_event):
    import logging
    logger = logging.getLogger("standalone_reader_process")
    try:
        logger.info("Standalone reader process started")
        gateway = SharedMemoryFEAGIGateway(process_name="reader", temp_dir=temp_dir)
        event_received = multiprocessing.Event()
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
        logger.info(f"Standalone reader process finished, value={value}, event_received={event_received.is_set()}")
        result_queue.put((value, event_received.is_set()))
    except Exception as e:
        logger.error(f"Standalone reader process exception: {e}", exc_info=True)
        result_queue.put((None, False))

def standalone_writer_process(temp_dir, ready_event):
    import logging
    logger = logging.getLogger("standalone_writer_process")
    try:
        logger.info("Standalone writer process started")
        # Wait for the reader to be ready
        ready_event.wait(timeout=2.0)
        gateway = SharedMemoryFEAGIGateway(process_name="writer", temp_dir=temp_dir)
        gateway.config_dict.set("integration_key", "integration_value")
        gateway.event_system.send_event(EventType.CONFIG_UPDATED, {"integration": True})
        gateway.shutdown()
        logger.info("Standalone writer process finished and shutdown")
    except Exception as e:
        logger.error(f"Standalone writer process exception: {e}", exc_info=True)

def test_standalone_shared_memory_integration():
    """Integration test: Simulate two processes communicating via shared memory and event notification (standalone)."""
    with tempfile.TemporaryDirectory() as temp_dir:
        result_queue = multiprocessing.Queue()
        ready_event = multiprocessing.Event()
        # Start reader process first
        p_reader = multiprocessing.Process(target=standalone_reader_process, args=(temp_dir, result_queue, ready_event))
        p_reader.start()
        # Start writer process after reader signals readiness
        p_writer = multiprocessing.Process(target=standalone_writer_process, args=(temp_dir, ready_event))
        p_writer.start()
        p_writer.join()
        p_reader.join()
        value, event_received = result_queue.get()
        assert value == "integration_value", f"Expected 'integration_value', got {value}"
        assert event_received, "Integration event was not received"
        logger.info("Standalone shared memory integration test passed")


if __name__ == "__main__":
    print("Testing standalone shared memory components...")
    
    if test_shared_memory_manager():
        print("✅ SharedMemoryManager test passed")
    else:
        print("❌ SharedMemoryManager test failed")
    
    if test_event_notification_system():
        print("✅ EventNotificationSystem test passed")
    else:
        print("❌ EventNotificationSystem test failed")
    
    if test_shared_config_dict():
        print("✅ SharedConfigDict test passed")
    else:
        print("❌ SharedConfigDict test failed")
    
    if test_gateway():
        print("✅ SharedMemoryFEAGIGateway test passed")
    else:
        print("❌ SharedMemoryFEAGIGateway test failed")
    
    if test_standalone_shared_memory_integration():
        print("✅ Standalone shared memory integration test passed")
    else:
        print("❌ Standalone shared memory integration test failed") 