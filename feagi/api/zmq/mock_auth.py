"""
Mock implementation of zmq.auth for FEAGI.

This module provides mock implementations of zmq.auth classes and functions
that can be used when the real zmq.auth module is not available.
"""

import logging
import sys
from unittest.mock import MagicMock
from types import ModuleType

# Configure logging
logger = logging.getLogger("feagi.api.zmq.mock_auth")

class ThreadAuthenticator(MagicMock):
    """Mock implementation of zmq.auth.thread.ThreadAuthenticator."""
    
    def __init__(self, context=None):
        super().__init__()
        self.context = context
        self.started = False
        logger.info("Created mock ThreadAuthenticator")
        
    def start(self):
        """Start the authenticator thread."""
        self.started = True
        logger.info("Starting mock authenticator thread")
        
    def stop(self):
        """Stop the authenticator thread."""
        self.started = False
        logger.info("Stopping mock authenticator thread")
        
    def allow(self, addr):
        """Allow connections from addr."""
        logger.info(f"Mock authenticator allowing connections from {addr}")
        
    def deny(self, addr):
        """Deny connections from addr."""
        logger.info(f"Mock authenticator denying connections from {addr}")
        
    def configure_plain(self, domain="*", passwords=None):
        """Configure PLAIN authentication."""
        logger.info(f"Mock authenticator configuring PLAIN auth for domain {domain}")
        
    def configure_curve(self, domain="*", location=""):
        """Configure CURVE authentication."""
        logger.info(f"Mock authenticator configuring CURVE auth for domain {domain}")

# Create and configure the thread module
thread_module = ModuleType("zmq.auth.thread")
thread_module.ThreadAuthenticator = ThreadAuthenticator
sys.modules["zmq.auth.thread"] = thread_module

# Create the auth module if it doesn't exist
if "zmq.auth" not in sys.modules:
    auth_module = ModuleType("zmq.auth")
    sys.modules["zmq.auth"] = auth_module

# Add thread to zmq.auth
sys.modules["zmq.auth"].thread = thread_module

# Export mocks
__all__ = ['ThreadAuthenticator', 'thread_module'] 