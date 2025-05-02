"""ZeroMQ server implementation for FEAGI."""

import logging
import threading
import time
from typing import Dict, Any, List, Optional, Callable

import zmq

from feagi.api.core.services import CoreAPIService
from feagi.api.gateway import APIGateway

logger = logging.getLogger(__name__)

class ZMQServer:
    """
    ZeroMQ server for FEAGI.
    
    This class implements a ZeroMQ server that provides interfaces to FEAGI's
    functionality using various ZMQ patterns (Request-Reply, Pub-Sub, Push-Pull, Stream).
    """
    
    def __init__(
        self,
        host: str = "127.0.0.1",
        req_port: int = 5555,
        pub_port: int = 5556,
        push_port: int = 5557,
        stream_port: int = 5558,
        api_gateway: Optional[APIGateway] = None,
    ):
        """
        Initialize the ZeroMQ server.
        
        Args:
            host: Host address to bind to.
            req_port: Port for Request-Reply socket.
            pub_port: Port for Publish-Subscribe socket.
            push_port: Port for Push-Pull socket.
            stream_port: Port for Stream socket.
            api_gateway: Optional API Gateway instance. If not provided,
                        a new instance will be created.
        """
        self.host = host
        self.req_port = req_port
        self.pub_port = pub_port
        self.push_port = push_port
        self.stream_port = stream_port
        
        self.api_gateway = api_gateway or APIGateway()
        self.core_api = self.api_gateway.core_api
        
        self.context = zmq.Context()
        self.req_socket = None
        self.pub_socket = None
        self.push_socket = None
        self.stream_socket = None
        
        self.running = False
        self.threads = []
        
    def start(self):
        """Start the ZeroMQ server."""
        if self.running:
            logger.warning("ZMQ server already running")
            return
            
        # Create and bind sockets
        self._setup_req_socket()
        self._setup_pub_socket()
        self._setup_push_socket()
        self._setup_stream_socket()
        
        # Start threads
        self._start_req_thread()
        self._start_pub_thread()
        
        self.running = True
        logger.info("ZMQ server started")
        
    def stop(self):
        """Stop the ZeroMQ server."""
        self.running = False
        
        # Wait for threads to stop
        for thread in self.threads:
            thread.join(timeout=2.0)
            
        # Close sockets
        if self.req_socket:
            self.req_socket.close()
        if self.pub_socket:
            self.pub_socket.close()
        if self.push_socket:
            self.push_socket.close()
        if self.stream_socket:
            self.stream_socket.close()
            
        # Terminate context
        self.context.term()
        
        logger.info("ZMQ server stopped")
        
    def _setup_req_socket(self):
        """Set up the Request-Reply socket."""
        self.req_socket = self.context.socket(zmq.REP)
        self.req_socket.bind(f"tcp://{self.host}:{self.req_port}")
        logger.info(f"REQ socket bound to tcp://{self.host}:{self.req_port}")
        
    def _setup_pub_socket(self):
        """Set up the Publish-Subscribe socket."""
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://{self.host}:{self.pub_port}")
        logger.info(f"PUB socket bound to tcp://{self.host}:{self.pub_port}")
        
    def _setup_push_socket(self):
        """Set up the Push-Pull socket."""
        self.push_socket = self.context.socket(zmq.PUSH)
        self.push_socket.bind(f"tcp://{self.host}:{self.push_port}")
        logger.info(f"PUSH socket bound to tcp://{self.host}:{self.push_port}")
        
    def _setup_stream_socket(self):
        """Set up the Stream socket."""
        self.stream_socket = self.context.socket(zmq.STREAM)
        self.stream_socket.bind(f"tcp://{self.host}:{self.stream_port}")
        logger.info(f"STREAM socket bound to tcp://{self.host}:{self.stream_port}")
        
    def _start_req_thread(self):
        """Start a thread to handle Request-Reply socket."""
        thread = threading.Thread(target=self._req_handler)
        thread.daemon = True
        thread.start()
        self.threads.append(thread)
        
    def _start_pub_thread(self):
        """Start a thread to publish data."""
        thread = threading.Thread(target=self._pub_handler)
        thread.daemon = True
        thread.start()
        self.threads.append(thread)
        
    def _req_handler(self):
        """Handle Request-Reply socket messages."""
        while self.running:
            try:
                # Wait for a message with timeout
                if self.req_socket.poll(1000, zmq.POLLIN):
                    message = self.req_socket.recv_json()
                    
                    # Process message and send response
                    response = self._process_request(message)
                    self.req_socket.send_json(response)
                    
            except zmq.ZMQError as e:
                logger.error(f"ZMQ error in REQ handler: {e}")
            except Exception as e:
                logger.exception(f"Error in REQ handler: {e}")
                
    def _pub_handler(self):
        """Publish data periodically."""
        while self.running:
            try:
                # Get data to publish
                fcl_data = self._get_fcl_data()
                if fcl_data:
                    # Publish data
                    self.pub_socket.send_multipart([
                        b"fcl",
                        fcl_data
                    ])
                    
                # Get simulation status
                status_data = self._get_simulation_status()
                if status_data:
                    # Publish status
                    self.pub_socket.send_multipart([
                        b"status",
                        status_data
                    ])
                    
                # Sleep for a short interval
                time.sleep(0.1)
                
            except zmq.ZMQError as e:
                logger.error(f"ZMQ error in PUB handler: {e}")
            except Exception as e:
                logger.exception(f"Error in PUB handler: {e}")
                
    def _process_request(self, message: Dict[str, Any]) -> Dict[str, Any]:
        """
        Process a request message.
        
        Args:
            message: Request message as a dictionary.
            
        Returns:
            Response message as a dictionary.
        """
        try:
            # Extract message fields
            command = message.get("command")
            params = message.get("params", {})
            message_id = message.get("id")
            version = message.get("version", "1.0")
            
            # Check if command is valid
            if not command:
                return {
                    "error": "Missing command",
                    "id": message_id,
                    "version": version
                }
                
            # Route command to appropriate handler
            if command == "get_cortical_areas":
                result = self.core_api.get_cortical_areas()
            elif command == "get_cortical_area":
                area_id = params.get("area_id")
                if not area_id:
                    return {
                        "error": "Missing area_id parameter",
                        "id": message_id,
                        "version": version
                    }
                result = self.core_api.get_cortical_area(area_id)
            elif command == "start_simulation":
                result = self.core_api.start_simulation()
            elif command == "stop_simulation":
                result = self.core_api.stop_simulation()
            elif command == "get_simulation_status":
                result = self.core_api.get_simulation_status()
            else:
                return {
                    "error": f"Unknown command: {command}",
                    "id": message_id,
                    "version": version
                }
                
            # Return result
            return {
                "result": result,
                "id": message_id,
                "version": version
            }
            
        except Exception as e:
            logger.exception(f"Error processing request: {e}")
            return {
                "error": str(e),
                "id": message_id,
                "version": version
            }
            
    def _get_fcl_data(self) -> bytes:
        """
        Get FCL data to publish.
        
        Returns:
            FCL data as a binary string.
        """
        # This would be implemented to get real FCL data from FEAGI
        # For now, return empty bytes
        return b""
        
    def _get_simulation_status(self) -> bytes:
        """
        Get simulation status data to publish.
        
        Returns:
            Simulation status data as a binary string.
        """
        # This would be implemented to get real simulation status from FEAGI
        # For now, return empty bytes
        return b""


def create_zmq_server(
    host: str = "127.0.0.1",
    req_port: int = 5555,
    pub_port: int = 5556,
    push_port: int = 5557,
    stream_port: int = 5558,
    api_gateway: Optional[APIGateway] = None,
) -> ZMQServer:
    """
    Create and start a ZeroMQ server.
    
    Args:
        host: Host address to bind to.
        req_port: Port for Request-Reply socket.
        pub_port: Port for Publish-Subscribe socket.
        push_port: Port for Push-Pull socket.
        stream_port: Port for Stream socket.
        api_gateway: Optional API Gateway instance.
        
    Returns:
        Running ZMQServer instance.
    """
    server = ZMQServer(
        host=host,
        req_port=req_port,
        pub_port=pub_port,
        push_port=push_port,
        stream_port=stream_port,
        api_gateway=api_gateway,
    )
    server.start()
    return server 