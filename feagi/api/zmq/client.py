"""ZeroMQ client implementation for FEAGI."""

import logging
import threading
import time
import uuid
from typing import Dict, Any, List, Optional, Callable, Tuple

import zmq

logger = logging.getLogger(__name__)

class ZmqClient:
    """
    ZeroMQ client for FEAGI.
    
    This class implements a ZeroMQ client that connects to a FEAGI ZMQ server
    using various ZMQ patterns (Request-Reply, Pub-Sub, Push-Pull, Stream).
    """
    
    def __init__(
        self,
        host: str = "127.0.0.1",
        req_port: int = 5555,
        pub_port: int = 5556,
        push_port: int = 5557,
        stream_port: int = 5558,
        topics: Optional[List[str]] = None,
    ):
        """
        Initialize the ZeroMQ client.
        
        Args:
            host: Host address of the server to connect to.
            req_port: Port for Request-Reply socket.
            pub_port: Port for Publish-Subscribe socket.
            push_port: Port for Push-Pull socket.
            stream_port: Port for Stream socket.
            topics: List of topics to subscribe to.
        """
        self.host = host
        self.req_port = req_port
        self.pub_port = pub_port
        self.push_port = push_port
        self.stream_port = stream_port
        
        self.topics = topics or ["fcl", "status"]
        
        self.context = zmq.Context()
        self.req_socket = None
        self.sub_socket = None
        self.pull_socket = None
        self.stream_socket = None
        
        self.running = False
        self.threads = []
        
        # Callbacks
        self.topic_callbacks: Dict[str, List[Callable[[bytes], None]]] = {}
        
    def start(self):
        """Start the ZeroMQ client."""
        if self.running:
            logger.warning("ZMQ client already running")
            return
            
        # Create and connect sockets
        self._setup_req_socket()
        self._setup_sub_socket()
        self._setup_pull_socket()
        
        # Start threads
        self._start_sub_thread()
        
        self.running = True
        logger.info("ZMQ client started")
        
    def stop(self):
        """Stop the ZeroMQ client."""
        self.running = False
        
        # Wait for threads to stop
        for thread in self.threads:
            thread.join(timeout=2.0)
            
        # Close sockets
        if self.req_socket:
            self.req_socket.close()
        if self.sub_socket:
            self.sub_socket.close()
        if self.pull_socket:
            self.pull_socket.close()
            
        # Terminate context
        self.context.term()
        
        logger.info("ZMQ client stopped")
        
    def _setup_req_socket(self):
        """Set up the Request-Reply socket."""
        self.req_socket = self.context.socket(zmq.REQ)
        self.req_socket.connect(f"tcp://{self.host}:{self.req_port}")
        logger.info(f"REQ socket connected to tcp://{self.host}:{self.req_port}")
        
    def _setup_sub_socket(self):
        """Set up the Publish-Subscribe socket."""
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://{self.host}:{self.pub_port}")
        
        # Subscribe to topics
        for topic in self.topics:
            self.sub_socket.setsockopt(zmq.SUBSCRIBE, topic.encode())
            
        logger.info(f"SUB socket connected to tcp://{self.host}:{self.pub_port}")
        
    def _setup_pull_socket(self):
        """Set up the Push-Pull socket."""
        self.pull_socket = self.context.socket(zmq.PULL)
        self.pull_socket.connect(f"tcp://{self.host}:{self.push_port}")
        logger.info(f"PULL socket connected to tcp://{self.host}:{self.push_port}")
        
    def _start_sub_thread(self):
        """Start a thread to handle subscription messages."""
        thread = threading.Thread(target=self._sub_handler)
        thread.daemon = True
        thread.start()
        self.threads.append(thread)
        
    def _sub_handler(self):
        """Handle subscription messages."""
        poller = zmq.Poller()
        poller.register(self.sub_socket, zmq.POLLIN)
        
        while self.running:
            try:
                # Wait for a message with timeout
                socks = dict(poller.poll(1000))
                if self.sub_socket in socks and socks[self.sub_socket] == zmq.POLLIN:
                    # Receive multipart message
                    multipart = self.sub_socket.recv_multipart()
                    
                    # Check if message has topic and data
                    if len(multipart) >= 2:
                        topic = multipart[0].decode()
                        data = multipart[1]
                        
                        # Call registered callbacks
                        self._call_topic_callbacks(topic, data)
                    else:
                        logger.warning(f"Received invalid multipart message: {multipart}")
                    
            except zmq.ZMQError as e:
                logger.error(f"ZMQ error in SUB handler: {e}")
            except Exception as e:
                logger.exception(f"Error in SUB handler: {e}")
                
    def _call_topic_callbacks(self, topic: str, data: bytes):
        """
        Call registered callbacks for a topic.
        
        Args:
            topic: Topic name.
            data: Message data.
        """
        if topic in self.topic_callbacks:
            for callback in self.topic_callbacks[topic]:
                try:
                    callback(data)
                except Exception as e:
                    logger.exception(f"Error in callback for topic {topic}: {e}")
                    
    def register_topic_callback(self, topic: str, callback: Callable[[bytes], None]):
        """
        Register a callback for a topic.
        
        Args:
            topic: Topic name.
            callback: Callback function that takes the message data as an argument.
        """
        if topic not in self.topic_callbacks:
            self.topic_callbacks[topic] = []
            
        self.topic_callbacks[topic].append(callback)
        
        # Subscribe to topic if not already subscribed
        if self.sub_socket and topic not in self.topics:
            self.sub_socket.setsockopt(zmq.SUBSCRIBE, topic.encode())
            self.topics.append(topic)
            
    def unregister_topic_callback(self, topic: str, callback: Callable[[bytes], None]):
        """
        Unregister a callback for a topic.
        
        Args:
            topic: Topic name.
            callback: Callback function to unregister.
        """
        if topic in self.topic_callbacks:
            if callback in self.topic_callbacks[topic]:
                self.topic_callbacks[topic].remove(callback)
                
            # If no more callbacks for topic, unsubscribe
            if not self.topic_callbacks[topic] and self.sub_socket:
                self.sub_socket.setsockopt(zmq.UNSUBSCRIBE, topic.encode())
                self.topics.remove(topic)
                
    def send_request(self, command: str, params: Optional[Dict[str, Any]] = None, timeout: float = 5.0) -> Dict[str, Any]:
        """
        Send a request to the server.
        
        Args:
            command: Command to send.
            params: Parameters for the command.
            timeout: Timeout in seconds.
            
        Returns:
            Response from the server.
        """
        if not self.req_socket:
            raise RuntimeError("REQ socket not initialized")
            
        message = {
            "command": command,
            "params": params or {},
            "id": str(uuid.uuid4()),
            "version": "1.0"
        }
        
        try:
            # Set timeout for receiving response
            self.req_socket.setsockopt(zmq.RCVTIMEO, int(timeout * 1000))
            
            # Send message
            self.req_socket.send_json(message)
            
            # Receive response
            response = self.req_socket.recv_json()
            
            return response
        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                logger.error(f"Timeout waiting for response to command {command}")
                return {
                    "error": "Timeout waiting for response",
                    "id": message["id"],
                    "version": "1.0"
                }
            else:
                logger.error(f"ZMQ error in send_request: {e}")
                return {
                    "error": f"ZMQ error: {e}",
                    "id": message["id"],
                    "version": "1.0"
                }
        except Exception as e:
            logger.exception(f"Error in send_request: {e}")
            return {
                "error": str(e),
                "id": message["id"],
                "version": "1.0"
            }
            
    def push_data(self, data: bytes) -> bool:
        """
        Push data to the server.
        
        Args:
            data: Data to push.
            
        Returns:
            True if successful, False otherwise.
        """
        if not self.pull_socket:
            logger.error("PULL socket not initialized")
            return False
            
        try:
            self.pull_socket.send(data)
            return True
        except zmq.ZMQError as e:
            logger.error(f"ZMQ error in push_data: {e}")
            return False
        except Exception as e:
            logger.exception(f"Error in push_data: {e}")
            return False
            
    # Convenience methods for common operations
    
    def get_cortical_areas(self) -> List[Dict[str, Any]]:
        """
        Get all cortical areas.
        
        Returns:
            List of cortical areas.
        """
        response = self.send_request("get_cortical_areas")
        if "error" in response:
            logger.error(f"Error getting cortical areas: {response['error']}")
            return []
        return response.get("result", [])
        
    def get_cortical_area(self, area_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a cortical area by ID.
        
        Args:
            area_id: ID of the cortical area.
            
        Returns:
            Cortical area information or None if not found.
        """
        response = self.send_request("get_cortical_area", {"area_id": area_id})
        if "error" in response:
            logger.error(f"Error getting cortical area {area_id}: {response['error']}")
            return None
        return response.get("result", None)
        
    def start_simulation(self) -> bool:
        """
        Start the simulation.
        
        Returns:
            True if successful, False otherwise.
        """
        response = self.send_request("start_simulation")
        if "error" in response:
            logger.error(f"Error starting simulation: {response['error']}")
            return False
        return response.get("result", False)
        
    def stop_simulation(self) -> bool:
        """
        Stop the simulation.
        
        Returns:
            True if successful, False otherwise.
        """
        response = self.send_request("stop_simulation")
        if "error" in response:
            logger.error(f"Error stopping simulation: {response['error']}")
            return False
        return response.get("result", False)
        
    def get_simulation_status(self) -> Dict[str, Any]:
        """
        Get the current simulation status.
        
        Returns:
            Simulation status.
        """
        response = self.send_request("get_simulation_status")
        if "error" in response:
            logger.error(f"Error getting simulation status: {response['error']}")
            return {}
        return response.get("result", {})
    

def create_zmq_client(
    host: str = "127.0.0.1",
    req_port: int = 5555,
    pub_port: int = 5556,
    push_port: int = 5557,
    stream_port: int = 5558,
    topics: Optional[List[str]] = None,
) -> ZmqClient:
    """
    Create and start a ZeroMQ client.
    
    Args:
        host: Host address of the server to connect to.
        req_port: Port for Request-Reply socket.
        pub_port: Port for Publish-Subscribe socket.
        push_port: Port for Push-Pull socket.
        stream_port: Port for Stream socket.
        topics: List of topics to subscribe to.
        
    Returns:
        Running ZmqClient instance.
    """
    client = ZmqClient(
        host=host,
        req_port=req_port,
        pub_port=pub_port,
        push_port=push_port,
        stream_port=stream_port,
        topics=topics,
    )
    client.start()
    return client 