"""
Connection State Management

Defines connection states and exceptions for FEAGI agent connections.
"""


class FeagiAgentError(Exception):
    """Custom exception for FEAGI agent errors."""
    pass


class ConnectionState:
    """Track agent connection state through the proper sequence."""
    DISCONNECTED = "disconnected"
    CONNECTING_CONTROL = "connecting_control"  
    CONTROL_CONNECTED = "control_connected"
    REGISTERING = "registering"
    REGISTERED = "registered"
    CONNECTING_STREAMS = "connecting_streams"
    READY = "ready"
    ERROR = "error" 