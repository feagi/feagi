"""
FEAGI Control Protocol (FCP)

This module defines the FCP message types and constants used for
control communication between FEAGI and agents.
"""

import time


class FCPMessageType:
    """FCP message types."""
    # Registration messages
    REGISTER = "register"
    REGISTER_RESPONSE = "register_response"
    
    # Deregistration messages
    DEREGISTER = "deregister"
    DEREGISTER_RESPONSE = "deregister_response"
    
    # Heartbeat messages
    HEARTBEAT = "heartbeat"
    HEARTBEAT_RESPONSE = "heartbeat_response"
    
    # Status messages
    STATUS = "status"
    STATUS_RESPONSE = "status_response"
    
    # Configuration messages
    CONFIG = "config"
    CONFIG_RESPONSE = "config_response"
    
    # Error messages
    ERROR = "error"


class FCPErrorCode:
    """FCP error codes."""
    UNKNOWN = 0
    INVALID_REQUEST = 1
    AGENT_NOT_FOUND = 2
    AGENT_ALREADY_EXISTS = 3
    INVALID_PROTOCOL = 4
    INTERNAL_ERROR = 5
    UNAUTHORIZED = 6


class FCPHandshakeState:
    """FCP handshake states."""
    NONE = 0
    HELLO_SENT = 1
    WELCOME_RECEIVED = 2
    CAPABILITIES_SENT = 3
    CONFIG_RECEIVED = 4
    COMPLETED = 5


class FCPRegistrationStatus:
    """FCP registration status codes."""
    SUCCESS = "success"
    FAILURE = "failure"
    PENDING = "pending"
    REJECTED = "rejected"


def create_register_request(agent_id: str, agent_type: str, capabilities: dict) -> dict:
    """
    Create a registration request message.
    
    Args:
        agent_id: Agent identifier
        agent_type: Agent type
        capabilities: Dictionary of agent capabilities
        
    Returns:
        Registration request message
    """
    return {
        "type": FCPMessageType.REGISTER,
        "agent_id": agent_id,
        "agent_type": agent_type,
        "timestamp": int(time.time() * 1000),
        "capabilities": capabilities
    }


def create_deregister_request(agent_id: str) -> dict:
    """
    Create a deregistration request message.
    
    Args:
        agent_id: Agent identifier
        
    Returns:
        Deregistration request message
    """
    return {
        "type": FCPMessageType.DEREGISTER,
        "agent_id": agent_id,
        "timestamp": int(time.time() * 1000)
    }


def create_heartbeat_request(agent_id: str) -> dict:
    """
    Create a heartbeat request message.
    
    Args:
        agent_id: Agent identifier
        
    Returns:
        Heartbeat request message
    """
    return {
        "type": FCPMessageType.HEARTBEAT,
        "agent_id": agent_id,
        "timestamp": int(time.time() * 1000)
    }


def create_status_request() -> dict:
    """
    Create a status request message.
    
    Returns:
        Status request message
    """
    return {
        "type": FCPMessageType.STATUS,
        "timestamp": int(time.time() * 1000)
    }


def create_error_response(error_code: int, message: str) -> dict:
    """
    Create an error response message.
    
    Args:
        error_code: Error code
        message: Error message
        
    Returns:
        Error response message
    """
    return {
        "type": FCPMessageType.ERROR,
        "error_code": error_code,
        "message": message,
        "timestamp": int(time.time() * 1000)
    } 