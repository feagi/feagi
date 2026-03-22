"""
Base Monitor Classes

Provides the foundation for all observability monitors using the Observer pattern.
"""

import logging
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional
from datetime import datetime

logger = logging.getLogger("feagi.pns.observability.monitor")


class Monitor(ABC):
    """
    Base class for all PNS monitors.
    
    Monitors observe brain_input and brain_output operations and can:
    - Log data packets
    - Collect metrics
    - Validate data
    - Profile performance
    
    Monitors are attached to brain_input/brain_output and receive callbacks
    on data operations.
    """
    
    def __init__(self, enabled: bool = True, log_level: str = "INFO"):
        """
        Initialize monitor.
        
        Args:
            enabled: Whether monitoring is active
            log_level: Logging level (DEBUG, INFO, WARNING, ERROR)
        """
        self.enabled = enabled
        self.log_level = log_level
        self._logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
        self._logger.setLevel(getattr(logging, log_level.upper()))
    
    def enable(self):
        """Enable monitoring."""
        self.enabled = True
        self._logger.info(f"{self.__class__.__name__} enabled")
    
    def disable(self):
        """Disable monitoring."""
        self.enabled = False
        self._logger.info(f"{self.__class__.__name__} disabled")
    
    @abstractmethod
    def on_send_start(self, data: Dict[str, Any]):
        """
        Called when brain_input.send() starts.
        
        Args:
            data: Dictionary containing:
                - timestamp: Send start time
                - input_count: Number of registered inputs
                - cortical_areas: List of cortical area IDs
        """
        pass
    
    @abstractmethod
    def on_send_complete(self, data: Dict[str, Any]):
        """
        Called when brain_input.send() completes.
        
        Args:
            data: Dictionary containing:
                - timestamp: Send complete time
                - packet_size_bytes: Total packet size
                - neuron_count: Total neurons sent
                - duration_ms: Time taken
        """
        pass
    
    @abstractmethod
    def on_receive_start(self, data: Dict[str, Any]):
        """
        Called when brain_output.receive() starts.
        
        Args:
            data: Dictionary containing:
                - timestamp: Receive start time
                - output_count: Number of registered outputs
        """
        pass
    
    @abstractmethod
    def on_receive_complete(self, data: Dict[str, Any]):
        """
        Called when brain_output.receive() completes.
        
        Args:
            data: Dictionary containing:
                - timestamp: Receive complete time
                - command_count: Number of motor commands
                - duration_ms: Time taken
        """
        pass
    
    def on_error(self, error: Exception, context: Dict[str, Any]):
        """
        Called when an error occurs.
        
        Args:
            error: The exception that occurred
            context: Context information about the error
        """
        if self.enabled:
            self._logger.error(f"Error in {context.get('operation', 'unknown')}: {error}")


class InputMonitor(Monitor):
    """
    Monitor for brain_input operations.
    
    Tracks sensory data being sent to FEAGI.
    """
    
    def __init__(
        self,
        enabled: bool = True,
        log_packets: bool = True,
        log_level: str = "INFO"
    ):
        """
        Initialize input monitor.
        
        Args:
            enabled: Whether monitoring is active
            log_packets: Whether to log individual packets
            log_level: Logging level
        """
        super().__init__(enabled=enabled, log_level=log_level)
        self.log_packets = log_packets
        self._packet_count = 0
    
    def on_send_start(self, data: Dict[str, Any]):
        """Log send operation start."""
        if not self.enabled:
            return
        
        self._logger.debug(
            f"Sending sensory data: {data['input_count']} inputs, "
            f"areas: {data.get('cortical_areas', [])}"
        )
    
    def on_send_complete(self, data: Dict[str, Any]):
        """Log send operation completion."""
        if not self.enabled:
            return
        
        self._packet_count += 1
        
        # Log periodically (every 100 packets) to reduce overhead
        if self.log_packets and self._packet_count % 100 == 0:
            self._logger.debug(
                f"Sent packet #{self._packet_count}: "
                f"{data['neuron_count']} neurons, "
                f"{data['packet_size_bytes']} bytes, "
                f"{data['duration_ms']:.2f} ms"
            )
    
    def on_receive_start(self, data: Dict[str, Any]):
        """Not used for input monitor."""
        pass
    
    def on_receive_complete(self, data: Dict[str, Any]):
        """Not used for input monitor."""
        pass


class OutputMonitor(Monitor):
    """
    Monitor for brain_output operations.
    
    Tracks motor commands received from FEAGI.
    """
    
    def __init__(
        self,
        enabled: bool = True,
        log_commands: bool = True,
        log_level: str = "INFO"
    ):
        """
        Initialize output monitor.
        
        Args:
            enabled: Whether monitoring is active
            log_commands: Whether to log individual commands
            log_level: Logging level
        """
        super().__init__(enabled=enabled, log_level=log_level)
        self.log_commands = log_commands
        self._command_count = 0
    
    def on_send_start(self, data: Dict[str, Any]):
        """Not used for output monitor."""
        pass
    
    def on_send_complete(self, data: Dict[str, Any]):
        """Not used for output monitor."""
        pass
    
    def on_receive_start(self, data: Dict[str, Any]):
        """Log receive operation start."""
        if not self.enabled:
            return
        
        self._logger.debug(f"Receiving motor commands: {data['output_count']} outputs")
    
    def on_receive_complete(self, data: Dict[str, Any]):
        """Log receive operation completion."""
        if not self.enabled:
            return
        
        self._command_count += data.get('command_count', 0)
        
        if self.log_commands and data.get('command_count', 0) > 0:
            self._logger.info(
                f"Received {data['command_count']} commands, "
                f"{data['duration_ms']:.2f} ms"
            )

