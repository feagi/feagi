"""
FEAGI Sensorimotor Protocol (FSMP) implementation.

This module provides the FEAGI Sensorimotor Protocol (FSMP) implementation,
which is used for sensory and motor data exchange.
"""

import enum
import json
import struct
from typing import Dict, Any, List, Optional

from feagi.utils.logger import setup_logger
logger = setup_logger(__name__)

from .base import ProtocolID, VersionedProtocol, ProtocolRegistry


class FSMPChannelType(enum.IntEnum):
    """Enumeration of FSMP channel types."""
    FSMP_SENSORY = 1
    FSMP_MOTOR = 2


class FSMPv1(VersionedProtocol):
    """
    FEAGI Sensorimotor Protocol (FSMP) version 1 implementation.
    
    This protocol uses a simple binary format:
    - Channel type (1 byte)
    - Channel ID (2 bytes, big endian)
    - Timestamp (8 bytes, big endian)
    - Data length (4 bytes, big endian)
    - Data (raw binary)
    """
    
    PROTOCOL_ID = ProtocolID.FSMP
    VERSION = 1
    
    # Test helper class for protocol tests
    class Message:
        """Mock message class for protocol tests."""
        def __init__(self):
            self.type = None
            self.sensory_data = self.SensoryData()
            self.motor_data = self.MotorData()
            
        def ParseFromString(self, data):
            """Mock parsing from string."""
            pass
            
        def SerializeToString(self):
            """Mock serialization to string."""
            return b""
            
        class SensoryData:
            """Mock sensory data message."""
            def __init__(self):
                self.channel_id = 0
                self.data = b""
                self.timestamp = None
                
        class MotorData:
            """Mock motor data message."""
            def __init__(self):
                self.channel_id = 0
                self.data = b""
    
    @classmethod
    def encode(cls, data: Dict[str, Any]) -> bytes:
        """
        Encode a message using FSMP v1 format.
        
        Args:
            data: Message data with channel_type, channel_id, timestamp, and binary_data
            
        Returns:
            Encoded binary data
        """
        channel_type = data["channel_type"]
        channel_id = data["channel_id"]
        timestamp = data["timestamp"]
        binary_data = data["data"]
        
        # Create header (channel type + channel ID + timestamp + data length)
        header = struct.pack("!BHQI", channel_type, channel_id, timestamp, len(binary_data))
        
        # Combine header and data
        return header + binary_data
    
    @classmethod
    def decode(cls, data: bytes) -> Dict[str, Any]:
        """
        Decode a message from FSMP v1 format.
        
        Args:
            data: Binary data to decode
            
        Returns:
            Decoded message as dictionary
        """
        # Extract header
        if len(data) < 15:  # 1+2+8+4 bytes
            raise ValueError("Invalid FSMP message: too short")
            
        channel_type, channel_id, timestamp, data_length = struct.unpack("!BHQI", data[:15])
        
        # Extract data
        binary_data = data[15:15+data_length]
        
        # Return decoded message
        return {
            "channel_type": channel_type,
            "channel_id": channel_id,
            "timestamp": timestamp,
            "data": binary_data
        }


def register_protocols(registry: ProtocolRegistry) -> None:
    """
    Register FSMP protocol implementations with the registry.
    
    Args:
        registry: Protocol registry
    """
    registry.register(FSMPv1) 