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
Event System Compatibility Layer for FEAGI.

This module provides a compatibility layer that bridges to the existing
shared memory events system. It maintains the expected interface while
using the proper event notification system.
"""

from typing import Any, Dict, Optional

from feagi.api.shared_memory.events import (
    EventNotificationSystem,
    EventPriority,
    EventType,
)
from feagi.utils.logger import setup_logger

logger = setup_logger()

# Global event system instance
_global_event_system: Optional[EventNotificationSystem] = None


def get_event_system() -> Optional[EventNotificationSystem]:
    """
    Get the global event system instance.

    Returns:
        EventNotificationSystem instance or None if not initialized
    """
    global _global_event_system

    if _global_event_system is None:
        try:
            # Initialize with a default process name
            _global_event_system = EventNotificationSystem("feagi_core")
            _global_event_system.start()
            logger.info("Global event system initialized successfully")
        except Exception as e:
            logger.warning(f"Failed to initialize global event system: {e}")
            return None

    return _global_event_system


def emit_event(
    event_type: EventType,
    data: Optional[Dict[str, Any]] = None,
    priority: EventPriority = EventPriority.HIGH,
) -> bool:
    """
    Emit an event through the global event system.

    Args:
        event_type: Type of event to emit
        data: Event data dictionary
        priority: Event priority level

    Returns:
        bool: True if event was sent successfully, False otherwise
    """
    event_system = get_event_system()
    if event_system is None:
        logger.warning("Event system not available - cannot emit event")
        return False

    try:
        return event_system.send_event(event_type, data, priority)
    except Exception as e:
        logger.error(f"Failed to emit event {event_type}: {e}")
        return False


def initialize_event_system(process_name: str = "feagi_core") -> bool:
    """
    Initialize the global event system with a specific process name.

    Args:
        process_name: Name for this process in the event system

    Returns:
        bool: True if initialization successful, False otherwise
    """
    global _global_event_system

    try:
        if _global_event_system is not None:
            _global_event_system.stop()

        _global_event_system = EventNotificationSystem(process_name)
        _global_event_system.start()
        logger.info(f"Event system initialized for process: {process_name}")
        return True
    except Exception as e:
        logger.error(f"Failed to initialize event system: {e}")
        return False


def shutdown_event_system():
    """Shutdown the global event system."""
    global _global_event_system

    if _global_event_system is not None:
        try:
            _global_event_system.stop()
            _global_event_system.cleanup()
            logger.info("Event system shutdown successfully")
        except Exception as e:
            logger.warning(f"Error during event system shutdown: {e}")
        finally:
            _global_event_system = None


# Export the EventType and EventPriority for compatibility
__all__ = [
    "EventType",
    "EventPriority",
    "get_event_system",
    "emit_event",
    "initialize_event_system",
    "shutdown_event_system",
]
