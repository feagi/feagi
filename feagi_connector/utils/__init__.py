"""
FEAGI Connector Utilities

This package contains utility functions and classes for the FEAGI connector.
"""

import logging
from typing import Optional


def setup_logging(level: int = logging.INFO, log_file: Optional[str] = None) -> None:
    """
    Set up logging for the FEAGI connector.
    
    Args:
        level: Logging level (default: INFO)
        log_file: Path to log file (default: None, log to console only)
    """
    handlers = []
    
    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    ))
    handlers.append(console_handler)
    
    # File handler if specified
    if log_file:
        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
        ))
        handlers.append(file_handler)
    
    # Configure root logger
    logging.basicConfig(
        level=level,
        handlers=handlers,
        force=True
    )
    
    # Set specific levels for noisy libraries
    logging.getLogger("zmq").setLevel(logging.WARNING)


__all__ = ["setup_logging"] 