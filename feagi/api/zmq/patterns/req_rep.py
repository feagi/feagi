"""Request-Reply pattern implementation for FEAGI ZeroMQ interface."""

import logging
import json
import threading
import uuid
from typing import Dict, Any, Optional, Callable, List

import zmq

logger = logging.getLogger(__name__)

class RequestReplyPattern:
    """
    Request-Reply pattern implementation for FEAGI ZeroMQ interface.
    
    This pattern is used for CRUD operations and other request-response interactions.
    """
    
    def __init__(
        self,
        context: zmq.Context,
        is_server: bool = True,
        host: str = "127.0.0.1",
        port: int = 5555,
        handlers: Optional[Dict[str, Callable]] = None
    ):
        """
        Initialize the Request-Reply pattern.
        
        Args:
            context: ZeroMQ context.
            is_server: Whether this is a server (Rep) or client (Req).
            host: Host address to bind/connect to.
            port: Port to bind/connect to.
            handlers: Dictionary of command handlers for server mode.
        """
        self.context = context
        self.is_server = is_server
        self.host = host
        self.port = port
        self.handlers = handlers or {}
        
        self.socket = None
        self.running = False
        self.thread = None
        
    def start(self):
        """Start the Request-Reply pattern."""
        if self.running:
            logger.warning("Request-Reply pattern already running")
            return
            
        if self.is_server:
            self._setup_server_socket()
            self._start_server_thread()
        else:
            self._setup_client_socket()
            
        self.running = True
        logger.info(f"{'Server' if self.is_server else 'Client'} Request-Reply pattern started")
        
    def stop(self):
        """Stop the Request-Reply pattern."""
        self.running = False
        
        if self.thread:
            self.thread.join(timeout=2.0)
            
        if self.socket:
            self.socket.close()
            
        logger.info(f"{'Server' if self.is_server else 'Client'} Request-Reply pattern stopped")
        
    def _setup_server_socket(self):
        """Set up the server-side Rep socket."""
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(f"tcp://{self.host}:{self.port}")
        logger.info(f"REP socket bound to tcp://{self.host}:{self.port}")
        
    def _setup_client_socket(self):
        """Set up the client-side Req socket."""
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(f"tcp://{self.host}:{self.port}")
        logger.info(f"REQ socket connected to tcp://{self.host}:{self.port}")
        
    def _start_server_thread(self):
        """Start a thread to handle server requests."""
        self.thread = threading.Thread(target=self._server_handler)
        self.thread.daemon = True
        self.thread.start()
        
    def _server_handler(self):
        """Handle server requests."""
        while self.running:
            try:
                if self.socket.poll(1000, zmq.POLLIN):
                    message = self.socket.recv_json()
                    
                    # Process message and send response
                    response = self._process_request(message)
                    self.socket.send_json(response)
                    
            except zmq.ZMQError as e:
                logger.error(f"ZMQ error in server handler: {e}")
            except Exception as e:
                logger.exception(f"Error in server handler: {e}")
                
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
            message_id = message.get("id", str(uuid.uuid4()))
            version = message.get("version", "1.0")
            
            # Check if command is valid
            if not command:
                return {
                    "error": "Missing command",
                    "id": message_id,
                    "version": version
                }
                
            # Check if handler exists
            if command not in self.handlers:
                return {
                    "error": f"Unknown command: {command}",
                    "id": message_id,
                    "version": version
                }
                
            # Call handler
            handler = self.handlers[command]
            result = handler(**params)
            
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
        if self.is_server:
            raise RuntimeError("Cannot send request in server mode")
            
        if not self.socket:
            raise RuntimeError("Socket not initialized")
            
        message = {
            "command": command,
            "params": params or {},
            "id": str(uuid.uuid4()),
            "version": "1.0"
        }
        
        try:
            # Set timeout for receiving response
            self.socket.setsockopt(zmq.RCVTIMEO, int(timeout * 1000))
            
            # Send message
            self.socket.send_json(message)
            
            # Receive response
            response = self.socket.recv_json()
            
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
            
    def register_handler(self, command: str, handler: Callable):
        """
        Register a handler for a command.
        
        Args:
            command: Command name.
            handler: Handler function.
        """
        if not self.is_server:
            raise RuntimeError("Cannot register handler in client mode")
            
        self.handlers[command] = handler 