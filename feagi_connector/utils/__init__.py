"""
FEAGI Connector Utilities

This package contains utility functions and classes for the FEAGI connector.
"""

import logging
from typing import Optional

from feagi_connector.utils.rust_processing import (
    MULTI_STRUCT_HOLDER,
    NEURON_POTENTIAL_CATEGORICAL_XYZ,
    extract_sub_structures_rust,
    infer_byte_structure_type_rust,
    decode_neuron_potential_xyz_rust,
    encode_neuron_potential_xyz_rust,
)

# Expose canonical helper names backed by Rust implementations
infer_byte_structure_type = infer_byte_structure_type_rust
extract_sub_structures = extract_sub_structures_rust
decode_neuron_potential_xyz = decode_neuron_potential_xyz_rust
encode_neuron_potential_xyz = encode_neuron_potential_xyz_rust


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


def is_rust_available() -> bool:
    """
    Check if the Rust implementations are available.
    
    Returns:
        True if feagi-rust-py-libs is installed, False otherwise
    """
    try:
        import feagi_rust_py_libs  # noqa: F401  # Import used for availability check only
    except ImportError:
        return False
    return True


__all__ = [
    "setup_logging",
    "is_rust_available",
    "infer_byte_structure_type",
    "extract_sub_structures",
    "decode_neuron_potential_xyz",
    "encode_neuron_potential_xyz",
    "MULTI_STRUCT_HOLDER",
    "NEURON_POTENTIAL_CATEGORICAL_XYZ",
] 