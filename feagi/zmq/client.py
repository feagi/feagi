"""ZMQ Client for FEAGI.

This module provides a client-side interface for connecting to FEAGI's ZMQ server,
whether it's running locally or on a remote machine.
"""
import json
import logging
import threading
from typing import Dict, List, Optional, Any, Callable, Union

# Import ZMQ with proper error handling
try:
    import zmq
    ZMQ_AVAILABLE = True
except ImportError:
    import warnings
    warnings.warn("PyZMQ is not installed. ZMQ client will not function.")
    ZMQ_AVAILABLE = False
    
    # Create minimal stub for type checking if ZMQ is not available
    class _StubZMQ:
        PUB = 1
        SUB = 2
        RCVTIMEO = 3
        SUBSCRIBE = 4
        
        class Context:
            def __init__(self):
                pass
                
            def socket(self, socket_type):
                return None
                
            def term(self):
                pass
    
    # Use stub only if import failed
    zmq = _StubZMQ()

class ZMQClient:
    """
    Client for connecting to a FEAGI ZMQ server.
    
    This class provides functionality to:
    1. Connect to a remote or local ZMQ server
    2. Publish messages to topics
    3. Subscribe to topics and receive messages
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
        self.server_host = server_host
        self.pub_port = pub_port
        self.sub_port = sub_port
        self.topics = topics or []
        self.logger = logger or logging.getLogger(__name__)
        self.reconnect_timeout = reconnect_timeout
        
        # State tracking
        self.running = False
        self.subscribers = {}  # topic -> socket
        self.subscriber_threads = {}  # topic -> thread
        self.callbacks = {topic: [] for topic in self.topics}
        
        # ZMQ setup - initialize everything to None
        self.context = None
        self.publisher = None
        
        # Initialize ZMQ if available
        if ZMQ_AVAILABLE:
            try:
                # Extra protection to ensure zmq.Context is callable
                if not hasattr(zmq, 'Context') or not callable(zmq.Context):
                    self.logger.error("ZMQ module found but Context is missing or not callable - check your installation")
                    return
                
                self.context = zmq.Context()
                self.publisher = self.context.socket(zmq.PUB)
                self.publisher.connect(f"tcp://{self.server_host}:{self.pub_port}")
                self.logger.info(f"Connected to ZMQ server at {self.server_host}:{self.pub_port} for publishing")
            except Exception as e:
                self.logger.error(f"Error initializing ZMQ client: {str(e)}")
    
    def start(self) -> bool:
        """
        Start the ZMQ client.
        
        Returns:
            bool: True if started successfully
        """
        if self.running:
            self.logger.warning("ZMQ client already running")
            return True
            
        if not ZMQ_AVAILABLE:
            self.logger.error("ZMQ is not available")
            return False
            
        if self.context is None:
            self.logger.error("ZMQ context initialization failed")
            return False
            
        try:
            self.logger.info(f"Starting ZMQ client connected to {self.server_host}")
                    
            # Initialize subscribers for topics
            for topic in self.topics:
                self._create_subscriber(topic)
                    
            self.running = True
            self.logger.info("ZMQ client started successfully")
            return True
        except Exception as e:
            self.logger.error(f"Error starting ZMQ client: {e}")
            self.shutdown()
            return False
    
    def publish(self, topic: str, message: Any) -> bool:
        """
        Publish a message to a topic.
        
        Args:
            topic: Topic to publish to
            message: Message to publish (will be JSON serialized)
            
        Returns:
            bool: True if published successfully
        """
        if not self.running and not self.start():
            self.logger.error("Cannot publish: ZMQ client not running")
            return False
            
        if self.publisher is None:
            self.logger.error("Cannot publish: Publisher not available")
            return False
            
        try:
            # Convert to JSON if needed
            if not isinstance(message, str):
                message = json.dumps(message)
                
            # Publish message
            self.publisher.send_string(f"{topic} {message}")
            return True
        except Exception as e:
            self.logger.error(f"Error publishing to {topic}: {e}")
            return False
    
    def subscribe(self, topic: str, callback: Callable[[str, Any], None]) -> bool:
        """
        Subscribe to a topic with a callback.
        
        Args:
            topic: Topic to subscribe to
            callback: Function to call when a message is received
            
        Returns:
            bool: True if subscribed successfully
        """
        if not self.running and not self.start():
            self.logger.error("Cannot subscribe: ZMQ client not running")
            return False
            
        # Add callback to the list
        if topic not in self.callbacks:
            self.callbacks[topic] = []
            
        self.callbacks[topic].append(callback)
        
        # Create subscriber if it doesn't exist
        if topic not in self.subscribers:
            success = self._create_subscriber(topic)
            if not success:
                self.logger.error(f"Failed to create subscriber for {topic}")
                return False
                
        return True
    
    def _create_subscriber(self, topic: str) -> bool:
        """
        Create a subscriber for a topic.
        
        Args:
            topic: Topic to subscribe to
            
        Returns:
            bool: True if created successfully
        """
        if not ZMQ_AVAILABLE or self.context is None:
            return False
            
        try:
            # Create subscriber socket
            subscriber = self.context.socket(zmq.SUB)
            # Connect to server (as client)
            subscriber.connect(f"tcp://{self.server_host}:{self.sub_port}")
            subscriber.setsockopt_string(zmq.SUBSCRIBE, topic)
            
            # Set reconnection timeout
            if self.reconnect_timeout > 0:
                subscriber.setsockopt(zmq.RCVTIMEO, self.reconnect_timeout)
            
            # Store subscriber
            self.subscribers[topic] = subscriber
            self.logger.info(f"Connected to ZMQ server at {self.server_host}:{self.sub_port} for topic: {topic}")
            
            # Start subscriber thread
            thread = threading.Thread(
                target=self._subscriber_loop,
                args=(topic, subscriber),
                daemon=True
            )
            thread.start()
            
            self.subscriber_threads[topic] = thread
            self.logger.info(f"Started subscriber thread for topic: {topic}")
            return True
        except Exception as e:
            self.logger.error(f"Error creating subscriber for {topic}: {e}")
            return False
    
    def _subscriber_loop(self, topic: str, subscriber) -> None:
        """
        Main subscriber loop for receiving messages.
        
        Args:
            topic: Topic this subscriber is for
            subscriber: ZMQ subscriber socket
        """
        self.logger.info(f"Subscriber loop started for topic: {topic}")
        
        try:            
            while self.running:
                try:
                    # Receive message with timeout
                    message = subscriber.recv_string()
                    
                    # Parse and dispatch message
                    topic_str, payload = message.split(' ', 1)
                    
                    # Process callbacks
                    for callback in self.callbacks.get(topic, []):
                        try:
                            # Try parsing as JSON, fall back to raw string
                            try:
                                data = json.loads(payload)
                            except:
                                data = payload
                                
                            callback(topic_str, data)
                        except Exception as e:
                            self.logger.error(f"Error in callback for {topic}: {e}")
                except zmq.Again:
                    # Timeout - normal case for polling
                    pass
                except Exception as e:
                    if self.running:  # Only log if we're still supposed to be running
                        self.logger.error(f"Error in subscriber loop for {topic}: {e}")
                        # Try to reconnect after an error
                        try:
                            if self.running:
                                self.logger.info(f"Attempting to reconnect subscriber for topic {topic}...")
                                subscriber.close()
                                subscriber = self.context.socket(zmq.SUB)
                                subscriber.connect(f"tcp://{self.server_host}:{self.sub_port}")
                                subscriber.setsockopt_string(zmq.SUBSCRIBE, topic)
                                # Replace in subscribers dict
                                self.subscribers[topic] = subscriber
                                self.logger.info(f"Successfully reconnected for topic {topic}")
                        except Exception as reconn_err:
                            self.logger.error(f"Failed to reconnect for topic {topic}: {reconn_err}")
                            break
        finally:
            self.logger.info(f"Subscriber loop ended for topic: {topic}")
    
    def shutdown(self) -> None:
        """
        Shut down the ZMQ client.
        """
        if not self.running:
            return
            
        self.logger.info("Shutting down ZMQ client...")
        self.running = False
        
        # Close all subscribers
        for topic, subscriber in self.subscribers.items():
            try:
                subscriber.close()
                self.logger.info(f"Closed subscriber for topic: {topic}")
            except:
                pass
                
        # Close publisher
        if self.publisher:
            try:
                self.publisher.close()
                self.logger.info("Closed publisher socket")
            except:
                pass
                
        # Terminate context
        if self.context:
            try:
                self.context.term()
                self.logger.info("Terminated ZMQ context")
            except:
                pass
                
        self.logger.info("ZMQ client shutdown complete.")
    
    def is_healthy(self) -> bool:
        """
        Check if the ZMQ client is healthy.
        
        Returns:
            bool: True if healthy
        """
        return self.running and self.publisher is not None

# Factory function for creating a client
def create_zmq_client(
    server_host: str = "127.0.0.1",
    pub_port: int = 5556,
    sub_port: int = 5557,
    topics: List[str] = None,
    logger: Optional[logging.Logger] = None,
) -> ZMQClient:
    """
    Create a new ZMQ client connected to a server.
    
    Args:
        server_host: Host where the ZMQ server is running
        pub_port: Port for the publisher socket
        sub_port: Port for the subscriber socket
        topics: List of topics to subscribe to
        logger: Optional logger instance
        
    Returns:
        ZMQClient: An initialized ZMQ client
    """
    client = ZMQClient(
        server_host=server_host,
        pub_port=pub_port,
        sub_port=sub_port,
        topics=topics,
        logger=logger
    )
    
    client.start()
    return client 