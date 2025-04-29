"""Top-level ZMQ package for FEAGI.

This module provides a consistent API for ZMQ messaging within FEAGI,
following the same architectural pattern as the REST API.
"""
import os
import logging
import importlib
from typing import Optional, List, Any

__all__ = ["create_zmq_server", "create_zmq_client"]

logger = logging.getLogger(__name__)

def create_zmq_server(
    host: str = None,
    pub_port: int = None, 
    sub_port: int = None,
    topics=None,
    use_auth=False,
    use_encryption=False,
    config=None
):
    """
    Create and initialize a ZMQ server instance.
    
    This is the main factory function for creating ZMQ servers in FEAGI.
    
    Args:
        host: Host address to bind to
        pub_port: Port for the publisher socket
        sub_port: Port for the subscriber socket  
        topics: List of topics to support
        use_auth: Whether to use authentication (deprecated)
        use_encryption: Whether to use encryption (deprecated)
        config: Additional configuration (deprecated)
        
    Returns:
        Initialized ZMQServer instance or None if initialization fails
    """
    # Log deprecation warnings for removed parameters
    if use_auth:
        logger.warning("The 'use_auth' parameter is deprecated and will be ignored.")
    if use_encryption:
        logger.warning("The 'use_encryption' parameter is deprecated and will be ignored.")
    if config:
        logger.warning("The 'config' parameter is deprecated and will be ignored.")
    
    try:
        # First try to import from core module
        try:
            module = importlib.import_module("feagi.core.zmq.server")
            ZMQServer = getattr(module, "ZMQServer")
            
            server = ZMQServer(
                host=host,
                pub_port=pub_port,
                sub_port=sub_port,
                topics=topics,
                logger=logger
            )
            return server
        except (ImportError, AttributeError):
            # If that fails, fallback to legacy location
            logger.debug("Core ZMQ module not found, checking for legacy implementation")
            return _create_legacy_server(
                host=host,
                pub_port=pub_port,
                sub_port=sub_port,
                topics=topics,
                use_auth=use_auth,
                use_encryption=use_encryption,
                config=config
            )
            
    except Exception as e:
        logger.error(f"Failed to create ZMQServer: {e}")
        
    return None


def _create_legacy_server(
    host: str = None,
    pub_port: int = None, 
    sub_port: int = None,
    topics=None,
    use_auth=False,
    use_encryption=False,
    config=None
):
    """Legacy implementation for backward compatibility."""
    # This is a stub function to be replaced with actual implementation if needed
    logger.warning("Legacy ZMQ implementation not found. Using core implementation.")
    return None


def create_zmq_client(
    host: str = None,
    pub_port: int = None,
    sub_port: int = None,
    topics: List[str] = None
):
    """
    Create a ZMQ client for connecting to a FEAGI ZMQ server.
    
    This function creates a client that can publish and subscribe to 
    the specified ZMQ server.
    
    Args:
        host: Host address of the ZMQ server to connect to
        pub_port: Publisher port to connect to 
        sub_port: Subscriber port to connect to
        topics: List of topics to subscribe to
        
    Returns:
        A ZMQ client instance or None if creation fails
    """
    try:
        # Check if the server is already running in this process
        try:
            from feagi.main import get_zmq_client
            client = get_zmq_client()
            if client is not None and client.running:
                logger.info("Using in-process ZMQ server as client")
                return client
        except (ImportError, AttributeError):
            pass
            
        # Import client from core
        try:
            # Try to import the client from the core module
            from feagi.core.zmq.client import ZMQClient
            
            client = ZMQClient(
                host=host or os.environ.get("FEAGI_ZMQ_HOST", "127.0.0.1"),
                pub_port=pub_port or int(os.environ.get("FEAGI_ZMQ_PUB_PORT", "5556")),
                sub_port=sub_port or int(os.environ.get("FEAGI_ZMQ_SUB_PORT", "5557")),
                topics=topics
            )
            client.start()
            return client
        except ImportError:
            logger.warning("Core ZMQ client module not found")
            return None
            
    except Exception as e:
        logger.error(f"Failed to create ZMQ client: {e}")
        return None 