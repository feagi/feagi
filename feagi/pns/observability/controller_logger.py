"""
Controller Logger

Provides standardized, timestamped logging for embodiment controllers.
Integrates with FEAGI PNS observability framework.
"""

import logging
import sys
from typing import Any, Dict, Optional
from datetime import datetime

from feagi.pns.observability.monitor import Monitor


class ControllerLogger(Monitor):
    """
    Standardized logger for embodiment controllers.
    
    Provides timestamped console output with consistent formatting
    across all FEAGI controllers.
    
    Example:
        from feagi.pns.observability import ControllerLogger
        from feagi.pns import brain_output
        
        # Create logger
        logger = ControllerLogger(
            controller_name="MuJoCo",
            show_timestamps=True,
            log_level="INFO"
        )
        
        # Attach to brain_output for automatic monitoring
        brain_output.attach_monitor(logger)
        
        # Use for controller logging
        logger.info("✅ Model loaded")
        logger.debug("🔍 Processing frame 120")
        logger.warning("⚠️  No motors registered")
        logger.error("❌ Connection failed")
    """
    
    def __init__(
        self,
        controller_name: str = "Controller",
        show_timestamps: bool = True,
        timestamp_format: str = "%Y-%m-%d %H:%M:%S",
        log_level: str = "INFO",
        stream: Any = None
    ):
        """
        Initialize controller logger.
        
        Args:
            controller_name: Name of the controller (for log prefix)
            show_timestamps: Whether to include timestamps in output
            timestamp_format: strftime format for timestamps
            log_level: Logging level (DEBUG, INFO, WARNING, ERROR)
            stream: Output stream (defaults to sys.stdout)
        """
        super().__init__(enabled=True, log_level=log_level)
        
        self.controller_name = controller_name
        self.show_timestamps = show_timestamps
        self.timestamp_format = timestamp_format
        self.stream = stream or sys.stdout
        
        # Configure Python logger
        self._setup_python_logger()
    
    def _setup_python_logger(self):
        """Configure Python's logging module for controller output."""
        # Get or create controller-specific logger
        self._py_logger = logging.getLogger(f"feagi.controller.{self.controller_name}")
        self._py_logger.setLevel(getattr(logging, self.log_level.upper()))
        self._py_logger.propagate = False  # Don't propagate to parent loggers
        
        # Remove existing handlers
        self._py_logger.handlers.clear()
        
        # Create console handler with custom formatter
        handler = logging.StreamHandler(self.stream)
        handler.setLevel(getattr(logging, self.log_level.upper()))
        
        # Set formatter based on timestamp preference
        if self.show_timestamps:
            formatter = logging.Formatter(
                f'[%(asctime)s] %(message)s',
                datefmt=self.timestamp_format
            )
        else:
            formatter = logging.Formatter('%(message)s')
        
        handler.setFormatter(formatter)
        self._py_logger.addHandler(handler)
    
    def _log(self, level: str, message: str):
        """Internal logging method."""
        log_method = getattr(self._py_logger, level.lower())
        log_method(message)
        self.stream.flush()  # Ensure immediate output
    
    def info(self, message: str):
        """Log info message."""
        self._log("INFO", message)
    
    def debug(self, message: str):
        """Log debug message."""
        self._log("DEBUG", message)
    
    def warning(self, message: str):
        """Log warning message."""
        self._log("WARNING", message)
    
    def error(self, message: str):
        """Log error message."""
        self._log("ERROR", message)
    
    def critical(self, message: str):
        """Log critical message."""
        self._log("CRITICAL", message)
    
    # Monitor interface implementation
    
    def on_send_start(self, data: Dict[str, Any]):
        """Log when sensory data is being sent to FEAGI."""
        if self.log_level == "DEBUG":
            self.debug(
                f"📤 Sending sensory data: "
                f"{data.get('input_count', 0)} inputs, "
                f"areas: {data.get('cortical_areas', [])}"
            )
    
    def on_send_complete(self, data: Dict[str, Any]):
        """Log when sensory data send completes."""
        if self.log_level == "DEBUG":
            self.debug(
                f"✅ Sent {data.get('neuron_count', 0)} neurons "
                f"({data.get('packet_size_bytes', 0)} bytes) "
                f"in {data.get('duration_ms', 0):.2f} ms"
            )
    
    def on_receive_start(self, data: Dict[str, Any]):
        """Log when receiving motor commands from FEAGI."""
        if self.log_level == "DEBUG":
            self.debug(
                f"📥 Receiving motor commands for "
                f"{data.get('output_count', 0)} outputs"
            )
    
    def on_receive_complete(self, data: Dict[str, Any]):
        """Log when motor command receive completes."""
        if self.log_level == "DEBUG" and data.get('command_count', 0) > 0:
            self.debug(
                f"✅ Received {data.get('command_count', 0)} commands "
                f"in {data.get('duration_ms', 0):.2f} ms"
            )
    
    def on_error(self, error: Exception, context: Dict[str, Any]):
        """Log errors that occur during operations."""
        operation = context.get('operation', 'unknown')
        self.error(f"❌ Error in {operation}: {error}")
    
    def separator(self, char: str = "=", width: int = 60):
        """Print a separator line."""
        self.info(char * width)
    
    def blank(self):
        """Print a blank line."""
        self.info("")


# Module-level convenience function
def create_controller_logger(
    controller_name: str,
    log_level: str = "INFO",
    show_timestamps: bool = True
) -> ControllerLogger:
    """
    Create a standardized controller logger.
    
    Args:
        controller_name: Name of the controller
        log_level: Logging level (DEBUG, INFO, WARNING, ERROR)
        show_timestamps: Whether to show timestamps
    
    Returns:
        Configured ControllerLogger instance
    """
    return ControllerLogger(
        controller_name=controller_name,
        log_level=log_level,
        show_timestamps=show_timestamps
    )

