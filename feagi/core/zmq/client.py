"""ZMQ Client for FEAGI.

This module provides a ZMQ client for connecting to FEAGI's ZMQ server,
whether running locally or remotely.
"""
import json
import logging
import threading
import os
import time
from typing import Dict, List, Optional, Any, Callable, Union

# Configure logging
logger = logging.getLogger(__name__)

# Define ZMQ_AVAILABLE before any imports
ZMQ_AVAILABLE = False

# Try to import ZMQ
try:
    import zmq
    ZMQ_AVAILABLE = True
    logger.debug(f"ZMQ imported successfully from {getattr(zmq, '__file__', 'unknown')}")
except ImportError:
    import warnings
    warnings.warn("PyZMQ is not installed. ZMQ client will not function.")
    
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
    ZMQ client for connecting to FEAGI's ZMQ server.
    
    Provides functionality for:
    1. Publishing messages to topics
    2. Subscribing to topics with callbacks
    """
    
    def __init__(
        self,
        host: str = None,
        pub_port: int = None,
        sub_port: int = None,
        topics: List[str] = None,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        """
        Initialize a ZMQ client.
        
        Args:
            host: The host to connect to
            pub_port: Port for the publisher socket
            sub_port: Port for the subscriber socket
            topics: List of topics to subscribe to
            logger: Optional logger instance
        """
        # Get configuration from environment variables if not provided
        self.host = host or os.environ.get("FEAGI_ZMQ_HOST", "127.0.0.1")
        self.pub_port = pub_port or int(os.environ.get("FEAGI_ZMQ_PUB_PORT", "5556"))
        self.sub_port = sub_port or int(os.environ.get("FEAGI_ZMQ_SUB_PORT", "5557"))
        
        # Get topics from environment if not provided
        if topics is None:
            env_topics = os.environ.get("FEAGI_ZMQ_TOPICS", "neural,metrics,heartbeat")
            self.topics = env_topics.split(",") if env_topics else []
        else:
            self.topics = topics
            
        self.logger = logger or logging.getLogger(__name__)
        
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
                self.context = zmq.Context()
                self.publisher = self.context.socket(zmq.PUB)
                # Don't bind, connect instead
                pub_address = f"tcp://{self.host}:{self.pub_port}"
                self.publisher.connect(pub_address)
                self.logger.info(f"Publisher connected to {pub_address}")
                self.logger.info("ZMQ context and publisher initialized")
            except Exception as e:
                self.logger.error(f"Error initializing ZMQ: {str(e)}")
    
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
            self.logger.info(f"Starting ZMQ client connected to {self.host}")
            self.logger.info(f"PUB port: {self.pub_port}, SUB port: {self.sub_port}")
            
            # Initialize subscribers for topics
            for topic in self.topics:
                self._create_subscriber(topic)
                    
            self.running = True
            self.logger.info("ZMQ client started successfully.")
            return True
        except Exception as e:
            self.logger.error(f"Error starting ZMQ client: {e}")
            self.shutdown()
            return False
    
    def publish(self, topic: str, message: Union[str, dict, list, Any]) -> bool:
        """
        Publish a message to a topic.
        
        Args:
            topic: Topic to publish to
            message: Message to publish (will be JSON serialized if not a string)
            
        Returns:
            bool: True if published successfully
        """
        if not self.running:
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
        if not self.running:
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
            sub_address = f"tcp://{self.host}:{self.sub_port}"
            # For client, we connect to the SUB port
            subscriber.connect(sub_address)
            subscriber.setsockopt_string(zmq.SUBSCRIBE, topic)
            
            # Store subscriber
            self.subscribers[topic] = subscriber
            self.logger.info(f"Created subscriber for topic: {topic}")
            
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
            subscriber.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout
            
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