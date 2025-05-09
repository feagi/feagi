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
from typing import Dict, Any, Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("standalone_shared_memory_test")

# Add the current directory to the path
sys.path.append(os.getcwd())

class SharedMemoryManager:
    """Simple shared memory manager for testing."""
    
    def __init__(self, temp_dir=None):
        """Initialize the shared memory manager."""
        self.temp_dir = temp_dir or tempfile.gettempdir()
        self.shared_memory = {}
        self.logger = logging.getLogger("shared_memory_manager")
        self.logger.info(f"SharedMemoryManager initialized with temp_dir={self.temp_dir}")
    
    def create_shared_memory(self, name, size):
        """Create a shared memory block."""
        self.shared_memory[name] = bytearray(size)
        self.logger.info(f"Created shared memory '{name}' with size {size}")
        return True
    
    def write_to_shared_memory(self, name, data):
        """Write data to shared memory."""
        if name not in self.shared_memory:
            self.create_shared_memory(name, len(data))
        
        self.shared_memory[name] = data
        self.logger.info(f"Wrote {len(data)} bytes to shared memory '{name}'")
        return True
    
    def read_from_shared_memory(self, name):
        """Read data from shared memory."""
        if name not in self.shared_memory:
            raise KeyError(f"Shared memory '{name}' does not exist")
        
        self.logger.info(f"Read {len(self.shared_memory[name])} bytes from shared memory '{name}'")
        return self.shared_memory[name]
    
    def cleanup(self):
        """Clean up resources."""
        self.shared_memory.clear()
        self.logger.info("SharedMemoryManager cleaned up")
        return True


class EventType:
    """Event types for the event notification system."""
    CONFIG_UPDATED = "config_updated"
    CORTICAL_AREA_ADDED = "cortical_area_added"
    CORTICAL_AREA_REMOVED = "cortical_area_removed"
    CORTICAL_AREA_UPDATED = "cortical_area_updated"
    GENOME_LOADED = "genome_loaded"


class Event:
    """Event class for the event notification system."""
    
    def __init__(self, event_type, source, data=None):
        """Initialize the event."""
        self.event_type = event_type
        self.source = source
        self.data = data or {}
        self.timestamp = time.time()
    
    def __str__(self):
        """String representation of the event."""
        return f"Event(type={self.event_type}, source={self.source}, data={self.data})"


class EventNotificationSystem:
    """Simple event notification system for testing."""
    
    def __init__(self, process_name, temp_dir=None):
        """Initialize the event notification system."""
        self.process_name = process_name
        self.temp_dir = temp_dir or tempfile.gettempdir()
        self.handlers = {}
        self.running = False
        self.event_queue = []
        self.event_thread = None
        self.lock = threading.Lock()
        self.logger = logging.getLogger("event_notification_system")
        self.logger.info(f"EventNotificationSystem initialized for process '{process_name}'")
    
    def register_handler(self, event_type, handler):
        """Register a handler for an event type."""
        with self.lock:
            if event_type not in self.handlers:
                self.handlers[event_type] = []
            
            self.handlers[event_type].append(handler)
            self.logger.info(f"Registered handler for event type '{event_type}'")
    
    def send_event(self, event_type, data=None):
        """Send an event."""
        event = Event(event_type, self.process_name, data)
        with self.lock:
            self.event_queue.append(event)
            self.logger.info(f"Queued event: {event}")
        
        # Process events immediately if the system is running
        if self.running:
            self._process_events()
    
    def _process_events(self):
        """Process queued events."""
        with self.lock:
            events = self.event_queue.copy()
            self.event_queue.clear()
        
        for event in events:
            handlers = self.handlers.get(event.event_type, [])
            for handler in handlers:
                try:
                    handler(event)
                except Exception as e:
                    self.logger.error(f"Error in event handler: {e}")
    
    def _event_loop(self):
        """Event processing loop."""
        while self.running:
            self._process_events()
            time.sleep(0.1)
    
    def start(self):
        """Start the event notification system."""
        if not self.running:
            self.running = True
            self.event_thread = threading.Thread(target=self._event_loop)
            self.event_thread.daemon = True
            self.event_thread.start()
            self.logger.info("EventNotificationSystem started")
    
    def stop(self):
        """Stop the event notification system."""
        if self.running:
            self.running = False
            if self.event_thread:
                self.event_thread.join(timeout=1.0)
            self.logger.info("EventNotificationSystem stopped")
    
    def cleanup(self):
        """Clean up resources."""
        self.stop()
        with self.lock:
            self.handlers.clear()
            self.event_queue.clear()
        self.logger.info("EventNotificationSystem cleaned up")


class SharedConfigDict:
    """Simple shared configuration dictionary for testing."""
    
    def __init__(self, name, manager=None):
        """Initialize the shared configuration dictionary."""
        self.name = name
        self.manager = manager or SharedMemoryManager()
        self.config = {}
        self.logger = logging.getLogger("shared_config_dict")
        self.logger.info(f"SharedConfigDict '{name}' initialized")
        
        # Create the shared memory if it doesn't exist
        try:
            data = self.manager.read_from_shared_memory(name)
            self.config = json.loads(data.decode('utf-8'))
        except Exception:
            self.manager.create_shared_memory(name, 1024)  # Initial size
            self._save_config()
    
    def _save_config(self):
        """Save the configuration to shared memory."""
        data = json.dumps(self.config).encode('utf-8')
        self.manager.write_to_shared_memory(self.name, data)
    
    def get(self, key, default=None):
        """Get a value from the configuration."""
        return self.config.get(key, default)
    
    def set(self, key, value):
        """Set a value in the configuration."""
        self.config[key] = value
        self._save_config()
        self.logger.info(f"Set '{key}' to {value} in SharedConfigDict '{self.name}'")
        return True
    
    def delete(self, key):
        """Delete a key from the configuration."""
        if key in self.config:
            del self.config[key]
            self._save_config()
            self.logger.info(f"Deleted '{key}' from SharedConfigDict '{self.name}'")
            return True
        return False


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