"""
FEAGI Sensorimotor Protocol (FSMP)

This module implements the FEAGI Sensorimotor Protocol used for exchanging
sensory and motor data between FEAGI and agents.
"""

from typing import Dict, Any, Optional

from feagi_connector.protocols.serialization import BinarySerializer


class FSMPChannel:
    """Channel constants for sensorimotor data."""
    # Default channels
    VISION = 1
    AUDIO = 2
    PROPRIOCEPTION = 3
    TACTILE = 4
    
    # Default motor channels
    MOVEMENT = 101
    SPEECH = 102
    MANIPULATION = 103


def create_sensory_message(channel_id: int, data: bytes, timestamp: Optional[int] = None) -> bytes:
    """
    Create a sensory data message.
    
    Args:
        channel_id: Sensory channel identifier
        data: Binary sensory data
        timestamp: Optional timestamp (milliseconds since epoch)
        
    Returns:
        Encoded binary message
    """
    return BinarySerializer.encode_fsmp(
        channel_id=channel_id,
        payload=data,
        timestamp=timestamp
    )


def decode_motor_message(data: bytes) -> Dict[str, Any]:
    """
    Decode a received motor message.
    
    Args:
        data: Binary message data
        
    Returns:
        Decoded message details
    """
    return BinarySerializer.decode_fsmp(data) 