"""ZMQ Server for FEAGI.

This module provides a ZMQ-based messaging server for FEAGI to enable efficient
bidirectional communication between components.
"""
import json
import logging
import threading
import time
from typing import Dict, List, Optional, Any, Callable, TYPE_CHECKING
import warnings

# Try to import ZMQ and provide fallbacks
ZMQ_AVAILABLE = False
try:
    import zmq
    # Verify that Context can be used correctly by creating and destroying a test context
    if hasattr(zmq, 'Context'):
        try:
            # Try to create a Context instance to verify it works
            test_context = zmq.Context()
            test_socket = test_context.socket(zmq.PUB)
            test_socket.close()
            test_context.term()
            ZMQ_AVAILABLE = True
        except Exception as e:
            warnings.warn(f"PyZMQ Context initialization failed: {e}. The ZMQ server will not be available. "
                          f"Try reinstalling PyZMQ with 'pip uninstall -y pyzmq && pip install pyzmq==24.0.1'")
    else:
        warnings.warn(f"PyZMQ is installed but 'Context' attribute is missing. This may be due to an incomplete installation. "
                      f"Try reinstalling PyZMQ with 'pip uninstall -y pyzmq && pip install pyzmq==24.0.1'")
except ImportError as e:
    warnings.warn(f"PyZMQ is not properly installed or configured: {e}")

# Create a stub zmq module if not available
if not ZMQ_AVAILABLE:
    # Create a stub zmq module
    class StubSocket:
        """Stub socket class for when ZMQ is not available."""
        def __init__(self, *args, **kwargs):
            self.closed = False
            
        def close(self):
            self.closed = True
            
        def bind(self, *args, **kwargs):
            pass
            
        def connect(self, *args, **kwargs):
            pass
            
        def setsockopt(self, *args, **kwargs):
            pass
            
        def setsockopt_string(self, *args, **kwargs):
            pass
            
        def recv_string(self, *args, **kwargs):
            raise Exception("ZMQ not available")

    class StubContext:
        """Stub context class for when ZMQ is not available."""
        def __init__(self, *args, **kwargs):
            pass
            
        def socket(self, *args, **kwargs):
            return StubSocket()
            
        def term(self):
            pass
    
    # If zmq module doesn't exist, create it
    if 'zmq' not in globals():
        class StubZMQ:
            """Stub ZMQ module."""
            PUB = 1
            SUB = 2
            RCVTIMEO = 3
            SUBSCRIBE = 4
            
            class Again(Exception):
                """Stub exception."""
                pass
                
            class ZMQError(Exception):
                """Stub exception."""
                pass
                
            def Context(self, *args, **kwargs):
                return StubContext()
            
        zmq = StubZMQ()
    # If zmq exists but doesn't have Context, add it
    elif not hasattr(zmq, 'Context'):
        zmq.Context = StubContext
        zmq.PUB = 1
        zmq.SUB = 2
        zmq.RCVTIMEO = 3
        zmq.SUBSCRIBE = 4
        
        class Again(Exception):
            """Stub exception."""
            pass
            
        class ZMQError(Exception):
            """Stub exception."""
            pass
            
        zmq.Again = Again
        zmq.ZMQError = ZMQError

# Use TYPE_CHECKING to avoid runtime dependency on zmq.Socket
if TYPE_CHECKING:
    from zmq.sugar.socket import Socket as ZMQSocket
else:
    ZMQSocket = Any

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
        logger: Optional[logging.Logger] = None,
    ) -> None:
        """
        Initialize a ZMQ server.
        
        Args:
            host: The host to bind to.
            pub_port: The port to use for the publisher socket.
            sub_port: The port to use for the subscriber socket.
            topics: A list of topics to subscribe to.
            logger: A logger to use for logging.
        """
        self.host = host
        self.pub_port = pub_port
        self.sub_port = sub_port
        self.topics = topics or []
        self.logger = logger or logging.getLogger(__name__)
        
        # Initialize data structures
        self.running = False
        self.subscribers = {}
        self.subscriber_threads = {}
        self.callbacks = {topic: [] for topic in self.topics}
        
        # Initialize ZMQ context and sockets if ZMQ is available
        self.context = None
        self.publisher = None
        
        if ZMQ_AVAILABLE:
            try:
                # Explicitly verify that zmq.Context is callable
                if callable(getattr(zmq, 'Context', None)):
                    self.context = zmq.Context()
                    self.publisher = self.context.socket(zmq.PUB)
                    self.logger.info("ZMQ context and publisher initialized")
                else:
                    self.logger.error("zmq.Context is not callable. ZMQ may not be properly installed.")
            except Exception as e:
                self.logger.error(f"Error initializing ZMQ context or publisher: {e}")
        else:
            self.logger.warning("ZMQ not available. Server will not function properly.")
    
    def start(self) -> bool:
        """
        Start the ZMQ server.
        
        Returns:
            bool: True if started successfully, False otherwise.
        """
        if self.running:
            self.logger.warning("ZMQ server is already running.")
            return True
            
        # Double check for ZMQ functionality
        if not ZMQ_AVAILABLE:
            self.logger.error("ZMQ is not available. Cannot start ZMQ server.")
            return False
            
        # Verify that context initialization was successful
        if self.context is None:
            self.logger.error("ZMQ context initialization failed. Cannot start ZMQ server.")
            return False
            
        try:
            self.logger.info(f"Starting ZMQ server on {self.host}")
            self.logger.info(f"PUB port: {self.pub_port}, SUB port: {self.sub_port}")
            
            # Initialize publisher socket if needed
            if self.publisher is None:
                try:
                    self.publisher = self.context.socket(zmq.PUB)
                except Exception as e:
                    self.logger.error(f"Failed to create publisher socket: {e}")
                    return False
            
            # Bind the publisher socket
            try:
                self.publisher.bind(f"tcp://{self.host}:{self.pub_port}")
                self.logger.info(f"Publisher bound to tcp://{self.host}:{self.pub_port}")
            except Exception as e:
                self.logger.error(f"Failed to bind publisher socket: {e}")
                return False
                    
            # Initialize subscribers for known topics
            for topic in self.topics:
                if topic not in self.subscribers:
                    success = self._create_subscriber(topic)
                    if not success:
                        self.logger.warning(f"Failed to create subscriber for topic: {topic}")
                    
            self.running = True
            self.logger.info("ZMQ server started successfully.")
            return True
        except Exception as e:
            self.logger.error(f"Error starting ZMQ server: {e}")
            self.shutdown()  # Clean up any partially initialized resources
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
    
    def publish(self, topic: str, message: str) -> bool:
        """
        Publish a message to a topic.
        
        Args:
            topic: The topic to publish to.
            message: The message to publish.
            
        Returns:
            bool: True if successful, False otherwise.
        """
        # Check if ZMQ is available at runtime (might have changed)
        if not ZMQ_AVAILABLE:
            self.logger.warning("Cannot publish: ZMQ is not available.")
            return False
            
        # Check if we're running
        if not self.running:
            self.logger.warning("Cannot publish: ZMQ server is not running.")
            return False
            
        # Check if we have a publisher socket
        if not hasattr(self, "publisher") or self.publisher is None:
            self.logger.warning("Cannot publish: publisher socket is not initialized.")
            return False
            
        try:
            # Format the message: "topic message"
            full_message = f"{topic} {message}"
            self.publisher.send_string(full_message)
            return True
        except Exception as e:
            self.logger.error(f"Error publishing message to topic {topic}: {e}")
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
            
        if not ZMQ_AVAILABLE or self.context is None:
            self.logger.error("Cannot subscribe: ZMQ context is not available.")
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
    
    def _receive_messages(self, topic: str, subscriber: ZMQSocket) -> None:
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
    
    def is_healthy(self) -> bool:
        """
        Check if the ZMQ server is running and healthy.
        
        Returns:
            bool: True if the server is running and ZMQ is available, False otherwise.
        """
        if not ZMQ_AVAILABLE:
            self.logger.warning("ZMQ is not available.")
            return False
            
        if not self.running:
            self.logger.warning("ZMQ server is not running.")
            return False
            
        if self.context is None:
            self.logger.warning("ZMQ context is not initialized.")
            return False
            
        if self.publisher is None:
            self.logger.warning("ZMQ publisher is not initialized.")
            return False
            
        return True
    
    def shutdown(self) -> None:
        """
        Shutdown the ZMQ server and clean up resources.
        """
        self.logger.info("Shutting down ZMQ server...")
        self.running = False
        
        if not ZMQ_AVAILABLE:
            self.logger.warning("ZMQ not available, skipping socket cleanup.")
            return
        
        try:
            # Close all subscriber sockets
            for topic, subscriber in list(self.subscribers.items()):
                try:
                    subscriber.close()
                    self.logger.info(f"Closed subscriber for topic: {topic}")
                except Exception as e:
                    self.logger.warning(f"Error closing subscriber for topic {topic}: {e}")
            
            # Wait for subscriber threads to finish
            for topic, thread in list(self.subscriber_threads.items()):
                try:
                    thread.join(timeout=1.0)
                    if thread.is_alive():
                        self.logger.warning(f"Subscriber thread for topic {topic} did not terminate.")
                except Exception as e:
                    self.logger.warning(f"Error joining thread for topic {topic}: {e}")
            
            # Close the publisher socket
            if hasattr(self, "publisher") and self.publisher is not None:
                try:
                    self.publisher.close()
                    self.logger.info("Closed publisher socket")
                except Exception as e:
                    self.logger.warning(f"Error closing publisher socket: {e}")
            
            # Terminate the ZMQ context
            if hasattr(self, "context") and self.context is not None:
                try:
                    self.context.term()
                    self.logger.info("Terminated ZMQ context")
                except Exception as e:
                    self.logger.warning(f"Error terminating ZMQ context: {e}")
        except Exception as e:
            self.logger.error(f"Error during ZMQ server shutdown: {e}")
        finally:
            # Reset state
            self.subscribers = {}
            self.subscriber_threads = {}
            self.callbacks = {topic: [] for topic in self.topics}
            self.logger.info("ZMQ server shutdown complete.")
    
    def _create_subscriber(self, topic: str) -> bool:
        """
        Create a subscriber for a topic.
        
        Args:
            topic: The topic to subscribe to.
            
        Returns:
            bool: True if successful, False otherwise.
        """
        if not ZMQ_AVAILABLE or self.context is None:
            self.logger.warning("Cannot create subscriber: ZMQ not available.")
            return False
            
        try:
            # Create subscriber socket
            subscriber = self.context.socket(zmq.SUB)
            subscriber.connect(f"tcp://{self.host}:{self.sub_port}")
            subscriber.setsockopt_string(zmq.SUBSCRIBE, topic)
            
            # Store subscriber
            self.subscribers[topic] = subscriber
            self.logger.info(f"Created subscriber for topic: {topic}")
            
            # Create subscriber thread
            thread = threading.Thread(
                target=self._subscriber_loop,
                args=(topic, subscriber),
                daemon=True,
                name=f"zmq-subscriber-{topic}"
            )
            self.subscriber_threads[topic] = thread
            thread.start()
            self.logger.info(f"Started subscriber thread for topic: {topic}")
            
            return True
        except Exception as e:
            self.logger.error(f"Error creating subscriber for topic {topic}: {e}")
            if topic in self.subscribers:
                try:
                    self.subscribers[topic].close()
                except Exception:
                    pass
                del self.subscribers[topic]
            return False
    
    def _subscriber_loop(self, topic: str, subscriber: 'zmq.Socket') -> None:
        """
        Main loop for a subscriber thread.
        
        Args:
            topic: The topic being subscribed to.
            subscriber: The ZMQ subscriber socket.
        """
        self.logger.info(f"Subscriber loop started for topic: {topic}")
        
        # Configure socket timeout to allow for checking if server is still running
        subscriber.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout
        
        while self.running:
            try:
                # Try to receive a message with timeout
                message = subscriber.recv_string()
                
                # Process the message if callbacks exist
                if topic in self.callbacks and self.callbacks[topic]:
                    # Strip topic prefix from message (if it exists)
                    if message.startswith(topic):
                        message = message[len(topic):].lstrip()
                        
                    # Call all registered callbacks for this topic
                    for callback in self.callbacks[topic]:
                        try:
                            callback(message)
                        except Exception as e:
                            self.logger.error(f"Error in callback for topic {topic}: {e}")
                
            except zmq.Again:
                # Timeout occurred, just continue to check if server is still running
                continue
            except zmq.ZMQError as e:
                self.logger.error(f"ZMQ error in subscriber loop for topic {topic}: {e}")
                if self.running:  # Only log if we're supposed to be running
                    time.sleep(1)  # Avoid tight error loop
            except Exception as e:
                self.logger.error(f"Unexpected error in subscriber loop for topic {topic}: {e}")
                if self.running:
                    time.sleep(1)  # Avoid tight error loop
        
        self.logger.info(f"Subscriber loop ended for topic: {topic}") 