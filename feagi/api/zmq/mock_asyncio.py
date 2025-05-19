"""
Mock implementation of zmq.asyncio for FEAGI.

This module provides mock implementations of zmq.asyncio classes and functions
that can be used when the real zmq.asyncio module is not available.
"""

import logging
import sys
from unittest.mock import MagicMock
from types import ModuleType

# Configure logging
logger = logging.getLogger("feagi.api.zmq.mock_asyncio")

class Context:
    """Mock implementation of zmq.asyncio.Context."""
    
    def __init__(self):
        logger.info("Created mock Context")
        
    @classmethod
    def instance(cls):
        """Return the singleton instance of the Context."""
        return cls()
    
    def socket(self, socket_type):
        """Create a socket of the given type."""
        socket = MagicMock()
        socket.bind = MagicMock()
        socket.connect = MagicMock()
        socket.setsockopt = MagicMock()
        socket.close = MagicMock()
        
        # Set up async methods
        socket.send_multipart = MagicMock(return_value=None)
        socket.recv_multipart = MagicMock(return_value=[b"", b"{}"])
        
        logger.info(f"Created mock socket of type {socket_type}")
        return socket
    
    def term(self):
        """Terminate the context."""
        logger.info("Terminated mock Context")

class Poller:
    """Mock implementation of zmq.asyncio.Poller."""
    
    def __init__(self):
        self.sockets = {}
        logger.info("Created mock Poller")
        
    def register(self, socket, flags):
        """Register a socket with the poller."""
        self.sockets[socket] = flags
        logger.info(f"Registered socket with flags {flags}")
        
    def unregister(self, socket):
        """Unregister a socket from the poller."""
        if socket in self.sockets:
            del self.sockets[socket]
            logger.info("Unregistered socket")
    
    async def poll(self, timeout=None):
        """Poll for events."""
        logger.info(f"Polling with timeout {timeout}")
        # Return an empty list of events
        return []

# Create the asyncio module
asyncio_module = ModuleType("zmq.asyncio")
asyncio_module.Context = Context
asyncio_module.Poller = Poller

# Add to sys.modules if not already there
if "zmq.asyncio" not in sys.modules:
    sys.modules["zmq.asyncio"] = asyncio_module

# Define ZMQ constants needed for testing
POLLIN = 1
POLLOUT = 2
DEALER = 5
ROUTER = 6
REQ = 3
REP = 4
PUB = 1
SUB = 2
PUSH = 8
PULL = 7
PAIR = 0
SUBSCRIBE = 6

# Export symbols
__all__ = ['Context', 'Poller', 'POLLIN', 'POLLOUT', 'DEALER', 'ROUTER', 
           'REQ', 'REP', 'PUB', 'SUB', 'PUSH', 'PULL', 'PAIR', 'SUBSCRIBE'] 