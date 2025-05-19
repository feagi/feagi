"""
FEAGI Visualization Protocol (FVP) implementation.

This module provides the FEAGI Visualization Protocol (FVP) implementation,
which is used for visualization of brain structure and activity.
"""

import enum
import json
import struct
from typing import Dict, Any, List, Optional

from feagi.utils.logger import setup_logger
logger = setup_logger(__name__)

from .base import ProtocolID, VersionedProtocol, ProtocolRegistry


class FVPFrameType(enum.IntEnum):
    """Enumeration of FVP frame types."""
    FVP_STRUCTURE = 1
    FVP_ACTIVITY = 2
    FVP_CONTROL = 3


class FVPv1(VersionedProtocol):
    """
    FEAGI Visualization Protocol (FVP) version 1 implementation.
    
    This protocol uses a simple binary format:
    - Frame type (1 byte)
    - Timestamp (8 bytes, big endian)
    - Data length (4 bytes, big endian)
    - Data (JSON encoded UTF-8 or raw binary depending on frame type)
    """
    
    PROTOCOL_ID = ProtocolID.FVP
    VERSION = 1
    
    # Test helper class for protocol tests
    class Message:
        """Mock message class for protocol tests."""
        def __init__(self):
            self.type = None
            self.structure_data = self.StructureData()
            self.activity_data = self.ActivityData()
            
        def ParseFromString(self, data):
            """Mock parsing from string."""
            pass
            
        def SerializeToString(self):
            """Mock serialization to string."""
            return b""
            
        class StructureData:
            """Mock structure data message."""
            def __init__(self):
                self.timestamp = None
                self.cortical_areas = {}
                
        class ActivityData:
            """Mock activity data message."""
            def __init__(self):
                self.frame_id = 0
                self.timestamp = None
                self.activity = {}
    
    @classmethod
    def encode(cls, data: Dict[str, Any]) -> bytes:
        """
        Encode a message using FVP v1 format.
        
        Args:
            data: Message data with frame_type, timestamp, and data
            
        Returns:
            Encoded binary data
        """
        frame_type = data["frame_type"]
        timestamp = data["timestamp"]
        
        # Encode data based on frame type
        if frame_type == FVPFrameType.FVP_CONTROL:
            # Control messages are JSON
            binary_data = json.dumps(data["data"]).encode("utf-8")
        else:
            # Structure and activity data are already binary
            binary_data = data["data"]
        
        # Create header (frame type + timestamp + data length)
        header = struct.pack("!BQI", frame_type, timestamp, len(binary_data))
        
        # Combine header and data
        return header + binary_data
    
    @classmethod
    def decode(cls, data: bytes) -> Dict[str, Any]:
        """
        Decode a message from FVP v1 format.
        
        Args:
            data: Binary data to decode
            
        Returns:
            Decoded message as dictionary
        """
        # Extract header
        if len(data) < 13:  # 1+8+4 bytes
            raise ValueError("Invalid FVP message: too short")
            
        frame_type, timestamp, data_length = struct.unpack("!BQI", data[:13])
        
        # Extract data
        binary_data = data[13:13+data_length]
        
        # Decode based on frame type
        if frame_type == FVPFrameType.FVP_CONTROL:
            # Control messages are JSON
            decoded_data = json.loads(binary_data.decode("utf-8"))
        else:
            # Structure and activity data stay as binary
            decoded_data = binary_data
        
        # Return decoded message
        return {
            "frame_type": frame_type,
            "timestamp": timestamp,
            "data": decoded_data
        }


def register_protocols(registry: ProtocolRegistry) -> None:
    """
    Register FVP protocol implementations with the registry.
    
    Args:
        registry: Protocol registry
    """
    registry.register(FVPv1) 