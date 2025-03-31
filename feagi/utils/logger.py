"""Logging utilities for FEAGI."""
import logging
import os
import sys
from typing import Optional

def setup_logger(
    name: str = "feagi",
    level: int = logging.INFO,
    log_file: Optional[str] = None,
    console: bool = True,
) -> logging.Logger:
    """
    Set up a logger with the given name and level.
    
    Args:
        name: Name of the logger.
        level: Logging level (default: INFO).
        log_file: Optional path to a log file.
        console: Whether to log to console (default: True).
        
    Returns:
        Configured logger.
    """
    logger = logging.getLogger(name)
    logger.setLevel(level)
    
    # Create formatters
    detailed_formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(module)s:%(lineno)d - %(message)s"
    )
    simple_formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
    
    # Remove existing handlers
    logger.handlers = []
    
    # Add file handler if log_file is provided
    if log_file:
        os.makedirs(os.path.dirname(log_file), exist_ok=True)
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(level)
        file_handler.setFormatter(detailed_formatter)
        logger.addHandler(file_handler)
    
    # Add console handler if console is True
    if console:
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(level)
        console_handler.setFormatter(simple_formatter)
        logger.addHandler(console_handler)
    
    return logger 