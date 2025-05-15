"""
FEAGI Visualization Protocol (FVP)

This module implements the FEAGI Visualization Protocol used for exchanging
brain visualization data between FEAGI and agents.
"""

from typing import Dict, Any, Optional

from feagi_connector.protocols.serialization import BinarySerializer


class FVPFrameType:
    """Frame types for visualization data."""
    # Brain structure data
    STRUCTURE = 1
    
    # Neural activity data
    ACTIVITY = 2


def decode_visualization_message(data: bytes) -> Dict[str, Any]:
    """
    Decode a received visualization message.
    
    Args:
        data: Binary message data
        
    Returns:
        Decoded message details
    """
    return BinarySerializer.decode_fvp(data) 