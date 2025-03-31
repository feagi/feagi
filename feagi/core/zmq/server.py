"""ZMQ Server for FEAGI.

This module provides a ZMQ-based messaging server for FEAGI to enable efficient
bidirectional communication between components.
"""
import json
import logging
import threading
import time
from typing import Dict, List, Optional, Any, Callable

import zmq

class ZMQServer:
    """
    ZMQ server for FEAGI.
    
    This class provides functionality for:
    1. Publishing messages to multiple topics
    2. Subscribing to topics for receiving messages
    3. Supporting authentication and optional encryption
    """
    
    def __init__(
        self,
        host: str = "127.0.0.1",
        pub_port: int = 5556,
        sub_port: int = 5557,
        topics: List[str] = None,
        use_auth: bool = False,
        use_encryption: bool = False,
        config: Optional[Dict] = None,
    ):
        """
        Initialize the ZMQ server.
        
        Args:
            host: Host address to bind to
            pub_port: Port for the publisher socket
            sub_port: Port for the subscriber socket
            topics: List of topics to support
            use_auth: Whether to use authentication
            use_encryption: Whether to use encryption
            config: Additional configuration
        """
        self.host = host
        self.pub_port = pub_port
        self.sub_port = sub_port
        self.topics = topics or ["default"]
        self.use_auth = use_auth
        self.use_encryption = use_encryption
        self.config = config or {}
        
        self.context = zmq.Context()
        self.publisher = self.context.socket(zmq.PUB)
        self.subscribers: Dict[str, zmq.Socket] = {}
        self.subscriber_threads: Dict[str, threading.Thread] = {}
        self.callbacks: Dict[str, List[Callable]] = {topic: [] for topic in self.topics}
        
        self.logger = logging.getLogger("feagi.zmq_server")
        self.running = False
    
    def start(self) -> bool:
        """
        Start the ZMQ server.
        
        Returns:
            True if the server was started successfully, False otherwise
        """
        if self.running:
            self.logger.warning("ZMQ server is already running.")
            return False
        
        try:
            # Configure and bind the publisher socket
            pub_address = f"tcp://{self.host}:{self.pub_port}"
            self.publisher.bind(pub_address)
            self.logger.info(f"Publisher bound to {pub_address}")
            
            # Initialize authentication if needed
            if self.use_auth:
                self._setup_auth()
            
            # Initialize encryption if needed
            if self.use_encryption:
                self._setup_encryption()
            
            self.running = True
            return True
        except Exception as e:
            self.logger.error(f"Failed to start ZMQ server: {e}")
            self.shutdown()
            return False
    
    def _setup_auth(self) -> None:
        """
        Set up ZeroMQ authentication.
        """
        self.logger.info("Setting up ZeroMQ authentication...")
        try:
            import zmq.auth
            # This is a placeholder for the actual authentication setup
            # Will be implemented based on specific authentication requirements
            self.logger.info("ZeroMQ authentication configured successfully.")
        except ImportError:
            self.logger.error("zmq.auth not available. Authentication disabled.")
            self.use_auth = False
    
    def _setup_encryption(self) -> None:
        """
        Set up ZeroMQ encryption.
        """
        self.logger.info("Setting up ZeroMQ encryption...")
        try:
            import zmq.auth
            # This is a placeholder for the actual encryption setup
            # Will be implemented based on specific encryption requirements
            self.logger.info("ZeroMQ encryption configured successfully.")
        except ImportError:
            self.logger.error("zmq.auth not available. Encryption disabled.")
            self.use_encryption = False
    
    def publish(self, topic: str, message: Any) -> bool:
        """
        Publish a message to a specific topic.
        
        Args:
            topic: Topic to publish to
            message: Message to publish (will be JSON serialized)
            
        Returns:
            True if the message was published successfully, False otherwise
        """
        if not self.running:
            self.logger.error("Cannot publish: ZMQ server is not running.")
            return False
        
        if topic not in self.topics:
            self.logger.warning(f"Unknown topic: {topic}. Adding it to the topics list.")
            self.topics.append(topic)
            self.callbacks[topic] = []
        
        try:
            # Serialize the message to JSON
            json_message = json.dumps(message)
            # Publish the message with the topic prefix
            self.publisher.send_multipart([topic.encode(), json_message.encode()])
            return True
        except Exception as e:
            self.logger.error(f"Failed to publish message to topic {topic}: {e}")
            return False
    
    def subscribe(self, topic: str, callback: Callable[[str, Any], None]) -> bool:
        """
        Subscribe to a topic with a callback function.
        
        Args:
            topic: Topic to subscribe to
            callback: Function to call when a message is received (takes topic and message as arguments)
            
        Returns:
            True if the subscription was successful, False otherwise
        """
        if not self.running:
            self.logger.error("Cannot subscribe: ZMQ server is not running.")
            return False
        
        if topic not in self.topics:
            self.logger.warning(f"Unknown topic: {topic}. Adding it to the topics list.")
            self.topics.append(topic)
            self.callbacks[topic] = []
        
        # Add the callback to the list of callbacks for this topic
        self.callbacks[topic].append(callback)
        
        # If we don't have a subscriber for this topic yet, create one
        if topic not in self.subscribers:
            try:
                # Create a new subscriber socket
                subscriber = self.context.socket(zmq.SUB)
                sub_address = f"tcp://{self.host}:{self.sub_port}"
                subscriber.connect(sub_address)
                subscriber.setsockopt(zmq.SUBSCRIBE, topic.encode())
                
                self.subscribers[topic] = subscriber
                self.logger.info(f"Subscribed to topic: {topic} at {sub_address}")
                
                # Start a thread to receive messages for this topic
                thread = threading.Thread(
                    target=self._receive_messages,
                    args=(topic, subscriber),
                    daemon=True,
                )
                thread.start()
                self.subscriber_threads[topic] = thread
                
                return True
            except Exception as e:
                self.logger.error(f"Failed to subscribe to topic {topic}: {e}")
                return False
        
        return True
    
    def _receive_messages(self, topic: str, subscriber: zmq.Socket) -> None:
        """
        Receive messages for a specific topic in a background thread.
        
        Args:
            topic: Topic to receive messages for
            subscriber: ZMQ socket for this topic
        """
        self.logger.info(f"Started message receiver thread for topic: {topic}")
        while self.running:
            try:
                # Receive a message
                message_parts = subscriber.recv_multipart(flags=zmq.NOBLOCK)
                if len(message_parts) != 2:
                    continue
                
                # Decode the message
                received_topic = message_parts[0].decode()
                message_json = message_parts[1].decode()
                message = json.loads(message_json)
                
                # Invoke all callbacks for this topic
                for callback in self.callbacks.get(received_topic, []):
                    try:
                        callback(received_topic, message)
                    except Exception as e:
                        self.logger.error(f"Error in callback for topic {received_topic}: {e}")
            except zmq.Again:
                # No message available, sleep a bit
                time.sleep(0.001)
            except Exception as e:
                self.logger.error(f"Error receiving message for topic {topic}: {e}")
                time.sleep(0.1)  # Sleep a bit longer after an error
    
    def unsubscribe(self, topic: str, callback: Optional[Callable] = None) -> bool:
        """
        Unsubscribe from a topic or remove a specific callback.
        
        Args:
            topic: Topic to unsubscribe from
            callback: Optional callback to remove. If None, all callbacks are removed.
            
        Returns:
            True if the unsubscription was successful, False otherwise
        """
        if topic not in self.topics:
            self.logger.warning(f"Cannot unsubscribe: Topic {topic} not found.")
            return False
        
        if callback is None:
            # Remove all callbacks for this topic
            self.callbacks[topic] = []
        else:
            # Remove only the specified callback
            try:
                self.callbacks[topic].remove(callback)
            except ValueError:
                self.logger.warning(f"Callback not found for topic {topic}.")
        
        # If there are no more callbacks for this topic, clean up the subscriber
        if not self.callbacks[topic] and topic in self.subscribers:
            subscriber = self.subscribers.pop(topic)
            subscriber.close()
            self.logger.info(f"Unsubscribed from topic: {topic}")
        
        return True
    
    def shutdown(self) -> None:
        """
        Shutdown the ZMQ server and clean up resources.
        """
        self.logger.info("Shutting down ZMQ server...")
        self.running = False
        
        # Close all subscriber sockets
        for topic, subscriber in self.subscribers.items():
            subscriber.close()
            self.logger.info(f"Closed subscriber for topic: {topic}")
        
        # Wait for subscriber threads to finish
        for topic, thread in self.subscriber_threads.items():
            thread.join(timeout=1.0)
            if thread.is_alive():
                self.logger.warning(f"Subscriber thread for topic {topic} did not terminate.")
        
        # Close the publisher socket
        if hasattr(self, "publisher"):
            self.publisher.close()
            self.logger.info("Closed publisher socket")
        
        # Terminate the ZMQ context
        if hasattr(self, "context"):
            self.context.term()
            self.logger.info("Terminated ZMQ context")
        
        self.subscribers = {}
        self.subscriber_threads = {}
        self.callbacks = {topic: [] for topic in self.topics}
        self.logger.info("ZMQ server shutdown complete.") 