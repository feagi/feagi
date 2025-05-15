"""
FEAGI Control Protocol (FCP)

This module implements the FEAGI Control Protocol used for agent
registration and management.
"""

import json
import time
from typing import Dict, Any

from feagi_connector.protocols.serialization import BinarySerializer


class FCPMessageType:
    """Message types for FEAGI Control Protocol."""
    # Registration
    REGISTER = "register"
    REGISTER_RESPONSE = "register_response"
    REGISTER_ID = 1
    
    # Deregistration
    DEREGISTER = "deregister"
    DEREGISTER_RESPONSE = "deregister_response"
    DEREGISTER_ID = 2
    
    # Status
    STATUS_REQUEST = "status_request"
    STATUS_RESPONSE = "status_response"
    STATUS_REQUEST_ID = 3
    
    # Configuration
    CONFIGURE = "configure"
    CONFIGURE_RESPONSE = "configure_response"
    CONFIGURE_ID = 4
    
    # Heartbeat
    HEARTBEAT = "heartbeat"
    HEARTBEAT_RESPONSE = "heartbeat_response"
    HEARTBEAT_ID = 5
    
    # Error
    ERROR = "error"
    ERROR_ID = 0xFF


def create_register_message(agent_id: str, agent_type: str) -> Dict[str, Any]:
    """
    Create an agent registration message.
    
    Args:
        agent_id: Unique identifier for this agent
        agent_type: Type of agent (for categorization)
        
    Returns:
        Message dictionary
    """
    return {
        "type": FCPMessageType.REGISTER,
        "data": {
            "agent_id": agent_id,
            "agent_type": agent_type,
            "protocol_versions": {
                "FCP": 1,
                "FSMP": 1,
                "FVP": 1
            }
        },
        "timestamp": time.time()
    }


def create_deregister_message(agent_id: str) -> Dict[str, Any]:
    """
    Create an agent deregistration message.
    
    Args:
        agent_id: Agent identifier
        
    Returns:
        Message dictionary
    """
    return {
        "type": FCPMessageType.DEREGISTER,
        "data": {
            "agent_id": agent_id
        },
        "timestamp": time.time()
    }


def create_heartbeat_message(agent_id: str) -> Dict[str, Any]:
    """
    Create a heartbeat message.
    
    Args:
        agent_id: Agent identifier
        
    Returns:
        Message dictionary
    """
    return {
        "type": FCPMessageType.HEARTBEAT,
        "data": {
            "agent_id": agent_id
        },
        "timestamp": time.time()
    }


def create_status_request_message() -> Dict[str, Any]:
    """
    Create a status request message.
    
    Returns:
        Message dictionary
    """
    return {
        "type": FCPMessageType.STATUS_REQUEST,
        "timestamp": time.time()
    }


def encode_message(message: Dict[str, Any], command_type: int) -> bytes:
    """
    Encode a message for transmission.
    
    Args:
        message: Message dictionary
        command_type: Command type identifier
        
    Returns:
        Encoded binary message
    """
    payload = json.dumps(message).encode()
    return BinarySerializer.encode_fcp(command_type=command_type, payload=payload)


def decode_message(data: bytes) -> Dict[str, Any]:
    """
    Decode a received message.
    
    Args:
        data: Binary message data
        
    Returns:
        Decoded message dictionary
    """
    decoded = BinarySerializer.decode_fcp(data)
    payload = decoded["payload"].decode()
    return json.loads(payload) 