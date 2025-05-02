"""ZMQ Client for FEAGI.

This is a compatibility module that forwards to the new implementation.
"""
import logging
from typing import Dict, List, Optional, Any, Callable, Union
import os

logger = logging.getLogger(__name__)

class ZMQClient:
    """
    Client for connecting to a FEAGI ZMQ server.
    
    This class is a compatibility wrapper that forwards to the new implementation.
    """
    
    def __init__(
        self,
        server_host: str = "127.0.0.1",
        pub_port: int = 5556,
        sub_port: int = 5557,
        topics: List[str] = None,
        logger: Optional[logging.Logger] = None,
        reconnect_timeout: int = 5000,  # Milliseconds
    ) -> None:
        """
        Initialize a ZMQ client.
        
        Args:
            server_host: Host where the ZMQ server is running
            pub_port: Port for the publisher socket
            sub_port: Port for the subscriber socket
            topics: List of topics to subscribe to
            logger: Optional logger instance
            reconnect_timeout: Timeout for reconnection attempts in milliseconds
        """
        self.logger = logger or logging.getLogger(__name__)
        
        try:
            # Import the new client implementation
            from feagi.api.zmq import create_zmq_client
            
            # Create a client instance
            self._client = create_zmq_client(
                host=server_host,
                pub_port=pub_port,
                sub_port=sub_port,
                topics=topics
            )
            
            # Set properties from inner client
            self.running = False
            if self._client is not None:
                self.running = self._client.running
                
        except ImportError as e:
            self.logger.error(f"Failed to import ZMQ client: {e}")
            self._client = None
    
    def start(self) -> bool:
        """
        Start the ZMQ client.
        
        Returns:
            bool: True if started successfully
        """
        if self._client is None:
            self.logger.error("Client initialization failed")
            return False
            
        if not hasattr(self._client, 'start'):
            self.logger.error("Client doesn't have start method")
            return False
            
        result = self._client.start()
        self.running = self._client.running
        return result
    
    def shutdown(self) -> None:
        """
        Shutdown the ZMQ client.
        """
        if self._client is not None and hasattr(self._client, 'shutdown'):
            self._client.shutdown()
            self.running = False
    
    def publish(self, topic: str, message: Any) -> bool:
        """
        Publish a message to a topic.
        
        Args:
            topic: Topic to publish to
            message: Message to publish
            
        Returns:
            bool: True if published successfully
        """
        if self._client is None or not self.running:
            self.logger.error("Client not running")
            return False
            
        if not hasattr(self._client, 'publish'):
            self.logger.error("Client doesn't have publish method")
            return False
            
        return self._client.publish(topic, message)
    
    def subscribe(self, topic: str, callback: Callable) -> bool:
        """
        Subscribe to a topic.
        
        Args:
            topic: Topic to subscribe to
            callback: Callback function to call when a message is received
            
        Returns:
            bool: True if subscribed successfully
        """
        if self._client is None or not self.running:
            self.logger.error("Client not running")
            return False
            
        if not hasattr(self._client, 'subscribe'):
            self.logger.error("Client doesn't have subscribe method")
            return False
            
        return self._client.subscribe(topic, callback)

# Factory function for compatibility
def create_zmq_client(
    server_host: str = "127.0.0.1",
    pub_port: int = 5556,
    sub_port: int = 5557,
    topics: List[str] = None
) -> Optional[ZMQClient]:
    """
    Create a ZMQ client for connecting to a FEAGI ZMQ server.
    
    Args:
        server_host: Host where the ZMQ server is running
        pub_port: Port for the publisher socket
        sub_port: Port for the subscriber socket
        topics: List of topics to subscribe to
        
    Returns:
        ZMQClient instance or None if creation fails
    """
    try:
        # Forward to the new implementation
        from feagi.api.zmq import create_zmq_client as new_create_client
        
        return new_create_client(
            host=server_host,
            pub_port=pub_port,
            sub_port=sub_port,
            topics=topics
        )
    except ImportError as e:
        logger.error(f"Failed to import ZMQ client: {e}")
        return None 