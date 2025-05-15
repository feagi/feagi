"""
Utility functions for FEAGI byte structures.

This module provides helper functions for working with FEAGI byte structures.
"""

import struct
from typing import Tuple


def get_structure_info(data: bytes) -> Tuple[int, int]:
    """
    Extract structure ID and version from byte structure header.
    
    Args:
        data: Byte data with header
        
    Returns:
        Tuple of (structure_id, version)
        
    Raises:
        ValueError: If data is too short
    """
    if len(data) < 2:
        raise ValueError("Data too short for header")
    
    structure_id, version = struct.unpack("!BB", data[:2])
    return structure_id, version


def is_compressed(data: bytes) -> bool:
    """
    Check if data is compressed.
    
    Args:
        data: Byte data to check
        
    Returns:
        True if data is compressed, False otherwise
        
    Raises:
        ValueError: If data is empty
    """
    if not data:
        raise ValueError("Empty data")
    
    return data[0] == 1


def validate_cortical_id(cortical_id: str) -> str:
    """
    Validate and normalize a cortical ID.
    
    Args:
        cortical_id: Cortical ID to validate
        
    Returns:
        Normalized cortical ID (exactly 6 chars)
        
    Raises:
        ValueError: If cortical ID is too long
    """
    if len(cortical_id) > 6:
        raise ValueError(f"Cortical ID '{cortical_id}' exceeds maximum length of 6 characters")
    
    # Pad to 6 chars if needed
    return cortical_id.ljust(6)[:6] 