"""
FEAGI Connector Utilities

This package contains utility functions and classes for the FEAGI connector.
"""

import logging
from typing import Optional

# Export constants from the Python implementation
from feagi_connector.utils.processing import (
    MULTI_STRUCT_HOLDER,
    NEURON_POTENTIAL_CATEGORICAL_XYZ
)

# Export Python implementations with explicit naming
from feagi_connector.utils.processing import (
    infer_byte_structure_type_python,
    extract_sub_structures_python,
    decode_neuron_potential_xyz_python,
    encode_neuron_potential_xyz_python
)

# NOTE: Previously, this module included a byte_processing.py file with runtime checks
# for Rust availability. That has been replaced with:
#
# 1. processing.py - Pure Python implementations with _python suffix
# 2. rust_processing.py - Direct Rust wrappers with _rust suffix
#
# This provides a fully explicit approach with no runtime availability checks
# for maximum performance and clarity.

# Rust implementations are imported directly by client code


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


# Helper function for determining if Rust is available
def is_rust_available() -> bool:
    """
    Check if the Rust implementations are available.
    
    Returns:
        True if feagi-data-processing is installed, False otherwise
    """
    try:
        # Just try to import the module - don't use the result
        import feagi_data_processing
        return True
    except ImportError:
        return False


__all__ = [
    "setup_logging",
    "is_rust_available",
    "infer_byte_structure_type_python",
    "extract_sub_structures_python",
    "decode_neuron_potential_xyz_python",
    "encode_neuron_potential_xyz_python",
    "MULTI_STRUCT_HOLDER",
    "NEURON_POTENTIAL_CATEGORICAL_XYZ"
] 