#!/usr/bin/env python
"""
Simple test for the shared memory components
"""

import logging
import time
import os
import tempfile
import sys

sys.path.insert(0, os.path.abspath('.'))

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("simple_shared_memory_test")

# Import only the necessary shared memory components
from feagi.api.shared_memory.manager import SharedMemoryManager
from feagi.api.shared_memory.events import EventNotificationSystem, EventType, Event
from feagi.api.shared_memory.data_structures import SharedConfigDict

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
            
            def event_handler(event: Event):
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

if __name__ == "__main__":
    print("Testing shared memory components...")
    
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