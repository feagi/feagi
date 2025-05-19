"""
FEAGI Control Protocol (FCP) implementation.

This module provides the FEAGI Control Protocol (FCP) implementation,
which is used for control and management operations.
"""

import enum
import json
import struct
from typing import Dict, Any, List, Optional

from feagi.utils.logger import setup_logger
logger = setup_logger(__name__)

from .base import ProtocolID, VersionedProtocol, ProtocolRegistry


class FCPCommandType(enum.IntEnum):
    """Enumeration of FCP command types."""
    REGISTER = 1
    DEREGISTER = 2
    GET_CONFIG = 3
    SET_CONFIG = 4
    GET_STATUS = 5
    HEARTBEAT = 6
    CUSTOM = 100


class FCPMessageType(enum.IntEnum):
    """Enumeration of FCP message types."""
    REGISTER = 1
    REGISTER_RESPONSE = 2
    DEREGISTER = 3
    DEREGISTER_RESPONSE = 4
    STATUS_REQUEST = 5
    STATUS_RESPONSE = 6
    HEARTBEAT = 7
    HEARTBEAT_RESPONSE = 8
    CONFIG_REQUEST = 9
    CONFIG_RESPONSE = 10
    EVENT = 100


class FCPv1(VersionedProtocol):
    """
    FEAGI Control Protocol (FCP) version 1 implementation.
    
    This protocol uses a simple binary format:
    - Command type (1 byte)
    - Payload length (4 bytes, big endian)
    - Payload (JSON encoded UTF-8)
    """
    
    PROTOCOL_ID = ProtocolID.FCP
    VERSION = 1
    
    # Test helper class for protocol tests
    class Message:
        """Mock message class for protocol tests."""
        def __init__(self):
            self.type = None
            self.register_confirm = self.RegisterConfirm()
            
        def ParseFromString(self, data):
            """Mock parsing from string."""
            pass
            
        def SerializeToString(self):
            """Mock serialization to string."""
            return b""
            
        class RegisterConfirm:
            """Mock register confirm message."""
            def __init__(self):
                self.status = ""
                self.message = ""
                self.timestamp = None
                
            def CopyFrom(self, other):
                """Copy from another object."""
                self.status = other.get("status", "")
                self.message = other.get("message", "")
                self.timestamp = other.get("timestamp", None)
    
    @classmethod
    def encode(cls, data: Dict[str, Any]) -> bytes:
        """
        Encode a message using FCP v1 format.
        
        Args:
            data: Message data with command_type and payload
            
        Returns:
            Encoded binary data
        """
        command_type = data["command_type"]
        payload = data["payload"]
        
        # Encode payload as JSON
        payload_json = json.dumps(payload).encode("utf-8")
        
        # Create header (command type + payload length)
        header = struct.pack("!BI", command_type, len(payload_json))
        
        # Combine header and payload
        return header + payload_json
    
    @classmethod
    def decode(cls, data: bytes) -> Dict[str, Any]:
        """
        Decode a message from FCP v1 format.
        
        Args:
            data: Binary data to decode
            
        Returns:
            Decoded message as dictionary
        """
        # Extract header
        if len(data) < 5:
            raise ValueError("Invalid FCP message: too short")
            
        command_type, payload_length = struct.unpack("!BI", data[:5])
        
        # Extract and decode payload
        payload_json = data[5:5+payload_length]
        payload = json.loads(payload_json.decode("utf-8"))
        
        # Return decoded message
        return {
            "command_type": command_type,
            "payload": payload
        }


def create_register_message(agent_id: str, agent_type: str, 
                          capabilities: Dict[str, Any]) -> Dict[str, Any]:
    """
    Create a registration message.
    
    Args:
        agent_id: Unique agent identifier
        agent_type: Type of agent
        capabilities: Agent capabilities
        
    Returns:
        FCP registration message
    """
    return {
        "command_type": FCPCommandType.REGISTER,
        "payload": {
            "agent_id": agent_id,
            "agent_type": agent_type,
            "capabilities": capabilities,
            "timestamp": None  # Will be filled in by sender
        }
    }


def register_protocols(registry: ProtocolRegistry) -> None:
    """
    Register FCP protocol implementations with the registry.
    
    Args:
        registry: Protocol registry
    """
    registry.register(FCPv1) 