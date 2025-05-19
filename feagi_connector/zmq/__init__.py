"""
ZeroMQ interface for feagi_connector.
"""

import importlib.util
import sys
import logging
from typing import Dict, Any, List, Optional, Callable, Union

# Configure logging
logger = logging.getLogger("feagi_connector.zmq")

# Check for ZMQ availability
ZMQ_AVAILABLE = importlib.util.find_spec("zmq") is not None
logger.info(f"ZMQ available: {ZMQ_AVAILABLE}")

# Create dummy client class to be used as fallback
class DummyZmqClient:
    """Dummy implementation when ZMQ is not available."""
    
    def __init__(self, *args, **kwargs):
        self.connected = False
        self.agent_id = kwargs.get('agent_id', 'dummy')
        self.agent_type = kwargs.get('agent_type', 'unknown')
        logger.warning("Using dummy ZmqClient (zmq not available)")
        
    def start(self):
        return False
        
    def stop(self):
        return True
        
    def send_request(self, *args, **kwargs):
        return {"status": "error", "message": "ZMQ not available"}
        
    def register_topic_callback(self, *args, **kwargs):
        pass
        
    def unregister_topic_callback(self, *args, **kwargs):
        pass

# Create dummy classes for the feagi-specific client
class DummyZmqFeagiClient(DummyZmqClient):
    """Dummy FEAGI-specific ZMQ client when ZMQ is not available."""

    def __init__(self, 
                 host: str = "localhost",
                 req_port: int = 5555,
                 pub_port: int = 5556,
                 push_port: int = 5557,
                 stream_port: int = 5558,
                 agent_id: str = "unknown",
                 agent_type: str = "unknown"):
        """Initialize the dummy FEAGI-specific ZMQ client."""
        super().__init__(agent_id=agent_id, agent_type=agent_type)
        self.host = host
        self.req_port = req_port
        self.pub_port = pub_port
        self.push_port = push_port
        self.stream_port = stream_port

# If ZMQ is available, import from client module
if ZMQ_AVAILABLE:
    try:
        # Avoid circular imports
        from .client import ZmqClient, ZmqFeagiClient
    except ImportError as e:
        logger.error(f"Failed to import ZMQ clients: {e}")
        # Fall back to dummy
        ZmqClient = DummyZmqClient
        ZmqFeagiClient = DummyZmqFeagiClient
else:
    # Use dummy implementations
    ZmqClient = DummyZmqClient
    ZmqFeagiClient = DummyZmqFeagiClient

# Export key classes and variables
__all__ = [
    'ZMQ_AVAILABLE',
    'ZmqClient',
    'ZmqFeagiClient',
] 