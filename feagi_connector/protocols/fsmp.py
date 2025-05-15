"""
FEAGI Sensorimotor Protocol (FSMP)

This module defines the FSMP message types and constants used for
sensorimotor communication between FEAGI and agents.
"""

import time

class FSMPChannel:
    """Sensorimotor channel types."""
    # Sensory channels
    VISION = 1
    AUDIO = 2
    PROPRIOCEPTION = 3
    TOUCH = 4
    SMELL = 5
    TASTE = 6
    CUSTOM_SENSORY = 100
    
    # Motor channels
    LOCOMOTION = 1001
    MANIPULATION = 1002
    VOCALIZATION = 1003
    FACIAL_EXPRESSION = 1004
    CUSTOM_MOTOR = 1100


class FSMPMessageType:
    """FSMP message types."""
    # Sensory messages
    SENSORY_DATA = "sensory_data"
    SENSORY_CONFIG = "sensory_config"
    
    # Motor messages
    MOTOR_DATA = "motor_data"
    MOTOR_CONFIG = "motor_config"
    
    # Channel messages
    REGISTER_CHANNEL = "register_channel"
    REGISTER_CHANNEL_RESPONSE = "register_channel_response"
    
    # Error messages
    ERROR = "error"


class FSMPDataFormat:
    """FSMP data formats."""
    RAW = "raw"
    ENCODED = "encoded"
    COMPRESSED = "compressed"
    TENSOR = "tensor"
    CUSTOM = "custom"


def create_sensory_data_message(channel_id: int, data_format: str = FSMPDataFormat.RAW) -> dict:
    """
    Create a sensory data message.
    
    Args:
        channel_id: Sensory channel ID
        data_format: Format of the sensory data
        
    Returns:
        Sensory data message
    """
    return {
        "type": FSMPMessageType.SENSORY_DATA,
        "channel_id": channel_id,
        "timestamp": int(time.time() * 1000),
        "data_format": data_format
    }


def create_motor_data_message(channel_id: int, data_format: str = FSMPDataFormat.RAW) -> dict:
    """
    Create a motor data message.
    
    Args:
        channel_id: Motor channel ID
        data_format: Format of the motor data
        
    Returns:
        Motor data message
    """
    import time
    
    return {
        "type": FSMPMessageType.MOTOR_DATA,
        "channel_id": channel_id,
        "timestamp": int(time.time() * 1000),
        "data_format": data_format
    }


def create_register_channel_message(channel_id: int, channel_type: int, properties: dict) -> dict:
    """
    Create a channel registration message.
    
    Args:
        channel_id: Channel ID
        channel_type: Channel type (from FSMPChannel)
        properties: Channel properties
        
    Returns:
        Channel registration message
    """
    import time
    
    return {
        "type": FSMPMessageType.REGISTER_CHANNEL,
        "channel_id": channel_id,
        "channel_type": channel_type,
        "timestamp": int(time.time() * 1000),
        "properties": properties
    } 