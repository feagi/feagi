"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
Event Notification System for FEAGI IPC.

This module provides a lightweight event notification system for inter-process
communication in FEAGI, replacing the ZMQ-based approach for higher performance.

PLATFORM COMPATIBILITY:
- Unix/Linux/macOS: Uses named pipes (FIFO) for IPC
- Windows: Event system is disabled (not supported)
- This module only works when shared memory mode is enabled
"""

import json
import logging
import os
import platform
import queue
import tempfile
import threading
import time
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Set

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class EventPriority(Enum):
    """Priority levels for events."""

    LOW = 0
    MEDIUM = 1
    HIGH = 2
    CRITICAL = 3


class EventType(Enum):
    """Types of events that can be sent between processes."""

    # Neuron-related events
    NEURON_FIRED = "neuron_fired"
    NEURON_UPDATED = "neuron_updated"

    # Connectome-related events
    CORTICAL_AREA_ADDED = "cortical_area_added"
    CORTICAL_AREA_REMOVED = "cortical_area_removed"
    CORTICAL_AREA_UPDATED = "cortical_area_updated"

    # Genome-related events
    GENOME_LOADED = "genome_loaded"
    GENOME_SAVED = "genome_saved"
    GENOME_UPDATED = "genome_updated"

    # Burst engine events
    BURST_STARTED = "burst_started"
    BURST_COMPLETED = "burst_completed"
    CONFIG_UPDATED = "config_updated"

    # System events
    PROCESS_STARTED = "process_started"
    PROCESS_STOPPED = "process_stopped"
    RESOURCE_WARNING = "resource_warning"

    # Custom event
    CUSTOM = "custom"


class Event:
    """Represents an event that can be sent between processes."""

    def __init__(
        self,
        event_type: EventType,
        source: str,
        data: Optional[Dict[str, Any]] = None,
        priority: EventPriority = EventPriority.MEDIUM,
    ):
        """Initialize an event.

        Args:
            event_type: Type of the event
            source: Process or component that generated the event
            data: Additional data for the event
            priority: Priority level of the event
        """
        self.event_type = event_type
        self.source = source
        self.data = data or {}
        self.priority = priority
        self.timestamp = time.time()

    def to_dict(self) -> Dict[str, Any]:
        """Convert the event to a dictionary for serialization."""
        return {
            "event_type": self.event_type.value,
            "source": self.source,
            "data": self.data,
            "priority": self.priority.value,
            "timestamp": self.timestamp,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Event":
        """Create an event from a dictionary."""
        return cls(
            event_type=EventType(data["event_type"]),
            source=data["source"],
            data=data["data"],
            priority=EventPriority(data["priority"]),
        )

    def __str__(self) -> str:
        """String representation of the event."""
        return f"Event({self.event_type.value}) from {self.source} at {self.timestamp}"


class EventNotificationSystem:
    """Provides event-based notification between FEAGI processes.

    This system uses a combination of file-based semaphores and shared memory
    to enable efficient, non-blocking event notifications across processes.
    """

    def __init__(
        self,
        process_name: str,
        temp_dir: Optional[str] = None,
        max_queue_size: int = 1000,
    ):
        """Initialize the event notification system.

        Args:
            process_name: Name of this process (used as event source)
            temp_dir: Directory to store event files (default: system temp dir)
            max_queue_size: Maximum number of events to queue
            
        Raises:
            RuntimeError: If the platform doesn't support named pipes (e.g., Windows)
        """
        self.process_name = process_name
        self.temp_dir = temp_dir or tempfile.gettempdir()
        self.max_queue_size = max_queue_size
        self.logger = logging.getLogger(
            f"feagi.api.shared_memory.events.{process_name}"
        )
        
        # Initialize state variables first (before any potential exceptions)
        self.event_thread = None
        self.running = False
        self.event_queue = queue.Queue(maxsize=max_queue_size)
        self.handlers: Dict[EventType, List[Callable[[Event], None]]] = {}
        self.subscriptions: Set[EventType] = set()
        self.event_file_path = None
        
        # Check platform compatibility
        if not hasattr(os, 'mkfifo'):
            raise RuntimeError(
                f"Event notification system not supported on {platform.system()}. "
                "Named pipes (os.mkfifo) are not available. "
                "This feature requires Unix/Linux/macOS."
            )

        # Create event directory
        self.event_dir = os.path.join(self.temp_dir, "feagi_events")
        os.makedirs(self.event_dir, exist_ok=True)

        # Event file path
        self.event_file_path = os.path.join(
            self.event_dir, f"events_{process_name}.fifo"
        )
        self._create_event_pipe()

    def _create_event_pipe(self):
        """Create the named pipe for event communication."""
        try:
            # Remove old pipe if it exists
            if os.path.exists(self.event_file_path):
                os.unlink(self.event_file_path)

            # Create a new named pipe (FIFO)
            os.mkfifo(self.event_file_path)
            self.logger.info(f"Created event pipe at {self.event_file_path}")
        except Exception as e:
            self.logger.error(f"Error creating event pipe: {e}")
            raise

    def start(self):
        """Start the event notification system."""
        if self.running:
            return

        self.running = True
        self.event_thread = threading.Thread(
            target=self._event_loop,
            daemon=True,
            name=f"EventLoop-{self.process_name}",
        )
        self.event_thread.start()
        self.logger.info(
            f"Started event notification system for {self.process_name}"
        )

    def stop(self):
        """Stop the event notification system."""
        self.running = False
        if self.event_thread and self.event_thread.is_alive():
            self.event_thread.join(timeout=1.0)
        self.logger.info(
            f"Stopped event notification system for {self.process_name}"
        )

    def _event_loop(self):
        """Main event processing loop."""
        self.logger.info("Event processing loop started")

        # Open the named pipe for reading
        # Using non-blocking mode to prevent hanging if no events
        pipe_fd = os.open(self.event_file_path, os.O_RDONLY | os.O_NONBLOCK)

        try:
            while self.running:
                # Check for incoming events
                ready, _, _ = select.select(
                    [pipe_fd], [], [], 0.1
                )  # 100ms timeout

                if ready:
                    # Read event data
                    try:
                        data = os.read(
                            pipe_fd, 4096
                        )  # Read up to 4KB at a time
                        if data:
                            # Parse and process events
                            self._process_event_data(data)
                    except BlockingIOError:
                        # No data available right now
                        pass

                # Process queued events
                self._process_queued_events()

                # Small sleep to prevent busy-waiting
                time.sleep(0.001)
        except Exception as e:
            self.logger.error(f"Error in event loop: {e}")
        finally:
            os.close(pipe_fd)
            self.logger.info("Event processing loop stopped")

    def _process_event_data(self, data: bytes):
        """Process binary event data received from the pipe."""
        try:
            # Split the data into individual event messages
            # Each event is a JSON string followed by a newline
            messages = data.decode("utf-8").split("\n")

            for msg in messages:
                if not msg.strip():
                    continue

                # Parse the event
                event_dict = json.loads(msg)
                event = Event.from_dict(event_dict)

                # Add to the queue if this is a subscribed event
                if event.event_type in self.subscriptions:
                    try:
                        self.event_queue.put_nowait(event)
                    except queue.Full:
                        self.logger.warning(
                            f"Event queue full, dropping event: {event}"
                        )
        except Exception as e:
            self.logger.error(f"Error processing event data: {e}")

    def _process_queued_events(self):
        """Process events in the queue."""
        # Process up to 10 events at a time to prevent blocking too long
        for _ in range(min(10, self.event_queue.qsize())):
            try:
                event = self.event_queue.get_nowait()
                self._dispatch_event(event)
                self.event_queue.task_done()
            except queue.Empty:
                break

    def _dispatch_event(self, event: Event):
        """Dispatch an event to registered handlers."""
        if event.event_type in self.handlers:
            for handler in self.handlers[event.event_type]:
                try:
                    handler(event)
                except Exception as e:
                    self.logger.error(f"Error in event handler: {e}")

    def send_event(
        self,
        event_type: EventType,
        data: Optional[Dict[str, Any]] = None,
        priority: EventPriority = EventPriority.MEDIUM,
    ) -> bool:
        """Send an event to other processes.

        Args:
            event_type: Type of the event
            data: Additional data for the event
            priority: Priority level of the event

        Returns:
            True if the event was sent successfully, False otherwise
        """
        event = Event(
            event_type=event_type,
            source=self.process_name,
            data=data or {},
            priority=priority,
        )

        # Find all event pipes except our own
        event_pipes = []
        for filename in os.listdir(self.event_dir):
            if filename.startswith("events_") and filename.endswith(".fifo"):
                pipe_path = os.path.join(self.event_dir, filename)
                if pipe_path != self.event_file_path:
                    event_pipes.append(pipe_path)

        # No other processes to send to
        if not event_pipes:
            return True

        # Serialize the event
        event_json = json.dumps(event.to_dict()) + "\n"
        event_bytes = event_json.encode("utf-8")

        # Send to all other processes
        success = True
        for pipe_path in event_pipes:
            try:
                # Open the pipe for writing (non-blocking)
                pipe_fd = os.open(pipe_path, os.O_WRONLY | os.O_NONBLOCK)
                try:
                    os.write(pipe_fd, event_bytes)
                finally:
                    os.close(pipe_fd)
            except Exception as e:
                self.logger.error(f"Error sending event to {pipe_path}: {e}")
                success = False

        return success

    def subscribe(self, event_type: EventType):
        """Subscribe to a specific event type.

        Args:
            event_type: The event type to subscribe to
        """
        self.subscriptions.add(event_type)
        self.logger.debug(f"Subscribed to event type: {event_type.value}")

    def unsubscribe(self, event_type: EventType):
        """Unsubscribe from a specific event type.

        Args:
            event_type: The event type to unsubscribe from
        """
        if event_type in self.subscriptions:
            self.subscriptions.remove(event_type)
            self.logger.debug(
                f"Unsubscribed from event type: {event_type.value}"
            )

    def register_handler(
        self, event_type: EventType, handler: Callable[[Event], None]
    ):
        """Register a handler for a specific event type.

        Args:
            event_type: The event type to handle
            handler: Function to call when this event type is received
        """
        # Make sure we're subscribed to this event type
        self.subscribe(event_type)

        # Add the handler
        if event_type not in self.handlers:
            self.handlers[event_type] = []
        self.handlers[event_type].append(handler)
        self.logger.debug(
            f"Registered handler for event type: {event_type.value}"
        )

    def unregister_handler(
        self, event_type: EventType, handler: Callable[[Event], None]
    ):
        """Unregister a handler for a specific event type.

        Args:
            event_type: The event type
            handler: The handler function to remove
        """
        if (
            event_type in self.handlers
            and handler in self.handlers[event_type]
        ):
            self.handlers[event_type].remove(handler)
            self.logger.debug(
                f"Unregistered handler for event type: {event_type.value}"
            )

            # If no more handlers for this event type, unsubscribe
            if not self.handlers[event_type]:
                del self.handlers[event_type]
                self.unsubscribe(event_type)

    def cleanup(self):
        """Clean up resources used by the event notification system."""
        # Only stop if we have a valid event_thread attribute
        if hasattr(self, 'running') and hasattr(self, 'event_thread'):
            self.stop()

        # Remove the named pipe if it was created
        try:
            if hasattr(self, 'event_file_path') and self.event_file_path and os.path.exists(self.event_file_path):
                os.unlink(self.event_file_path)
                if hasattr(self, 'logger'):
                    self.logger.info(
                        f"Removed event pipe at {self.event_file_path}"
                    )
        except Exception as e:
            if hasattr(self, 'logger'):
                self.logger.error(f"Error removing event pipe: {e}")

    def __del__(self):
        """Ensure resources are cleaned up.
        
        This method is safe to call even if __init__ raised an exception
        and the object was only partially initialized.
        """
        try:
            self.cleanup()
        except Exception:
            # Silently ignore cleanup errors during destruction
            # to prevent exceptions in __del__ from being logged
            pass
