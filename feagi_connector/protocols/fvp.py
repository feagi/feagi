"""
FEAGI Visualization Protocol (FVP)

This module defines the FVP message types and constants used for
visualization data communication between FEAGI and agents.
"""

import time

from typing import Dict, Any, Optional


class FVPFrameType:
    """FVP frame types."""
    # Activity data
    ACTIVITY = "activity_data"
    ACTIVITY_SNAPSHOT = "activity_snapshot"
    
    # Structure data
    STRUCTURE = "structure_data"
    CONNECTOME = "connectome_data"
    
    # Configuration
    VISUALIZATION_CONFIG = "visualization_config"
    
    # Error messages
    ERROR = "error"


class FVPDataFormat:
    """FVP data formats."""
    RAW = "raw"
    COMPRESSED = "compressed"
    SPARSE = "sparse"
    CATEGORIES = "categories"
    CUSTOM = "custom"


def create_activity_message(data_format: str = FVPDataFormat.RAW) -> dict:
    """
    Create an activity data message.
    
    Args:
        data_format: Format of the activity data
        
    Returns:
        Activity data message
    """
    return {
        "type": FVPFrameType.ACTIVITY,
        "timestamp": int(time.time() * 1000),
        "data_format": data_format
    }


def create_structure_message(data_format: str = FVPDataFormat.RAW) -> dict:
    """
    Create a structure data message.
    
    Args:
        data_format: Format of the structure data
        
    Returns:
        Structure data message
    """
    return {
        "type": FVPFrameType.STRUCTURE,
        "timestamp": int(time.time() * 1000),
        "data_format": data_format
    }


def create_connectome_message(data_format: str = FVPDataFormat.RAW) -> dict:
    """
    Create a connectome data message.
    
    Args:
        data_format: Format of the connectome data
        
    Returns:
        Connectome data message
    """
    return {
        "type": FVPFrameType.CONNECTOME,
        "timestamp": int(time.time() * 1000),
        "data_format": data_format
    }


def decode_visualization_message(data: bytes) -> Dict[str, Any]:
    """
    Decode a received visualization message.
    
    Args:
        data: Binary message data
        
    Returns:
        Decoded message details
    """
    return BinarySerializer.decode_fvp(data) 