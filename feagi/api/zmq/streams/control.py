"""
ZeroMQ Control Stream Implementation for FEAGI API

This module implements a bidirectional control stream for FEAGI.
It provides:
- Bidirectional communication between FEAGI and agents
- JSON-based message format for control commands and health status
- REST API wrapper support for unified control interface
- Reliable message delivery with request-reply pattern

The control stream is designed for administrative and control purposes:
- Agent registration and configuration
- Health status monitoring
- Command and control operations
- System status monitoring

It uses a ROUTER-DEALER pattern to support asynchronous request-reply with
multiple clients, while still providing reliable message delivery.
"""

import asyncio
from feagi.utils.logger import setup_logger
logger = setup_logger(__name__)
import time
import json
from typing import Dict, Any, Optional, Callable, List, Tuple, Callable

import zmq
import zmq.asyncio

from ...core.services.core_api_service import CoreAPIService


class ControlStream:
    """
    ZeroMQ Control Stream implementation for bidirectional control messages.
    
    This implementation uses a ROUTER socket to handle bidirectional communication
    with multiple clients using the request-reply pattern, while still allowing
    asynchronous operation.
    
    The control stream handles:
    - REST API requests (primary protocol)
    - Agent registration and configuration (legacy)
    - Health status monitoring and heartbeats (legacy)
    - Command and control operations (legacy)
    - System status monitoring (legacy)
    """
    
    def __init__(
        self,
        core_api: CoreAPIService,
        host: str = "*",
        port: int = 5559,
        context: Optional[zmq.asyncio.Context] = None
    ):
        """
        Initialize the control stream.
        
        Args:
            core_api: Core API service for accessing FEAGI
            host: Host to bind to
            port: Port for the control socket
            context: Optional existing ZMQ context to use
        """
        self.core_api = core_api
        self.host = host
        self.port = port
        self.context = context or zmq.asyncio.Context.instance()
        
        # State
        self.server_id = "feagi_server"
        self.running = False
        
        # Connected clients
        self.clients = {}
        
        # Initialize sockets
        self.router_socket = None  # Front-facing socket for external clients
        self.dealer_socket = None  # Back-end socket for internal processing
        
        # Tasks
        self.tasks = []
        
        # Message handlers
        self.message_handlers = {}
        
    async def start(self):
        """Start the control stream."""
        if self.running:
            return
            
        logger.info(f"Starting Control Stream on {self.host}:{self.port}")
        
        # Create ROUTER socket (for external clients)
        self.router_socket = self.context.socket(zmq.ROUTER)
        self.router_socket.bind(f"tcp://{self.host}:{self.port}")
        
        # Create DEALER socket (for internal routing)
        self.dealer_socket = self.context.socket(zmq.DEALER)
        self.dealer_socket.bind("inproc://control_backend")
        
        # Start worker threads
        self.tasks.append(asyncio.create_task(self._router_dealer_proxy()))
        self.tasks.append(asyncio.create_task(self._process_control_messages()))
        
        # Start client tracking
        self.tasks.append(asyncio.create_task(self._cleanup_inactive_clients()))
        
        self.running = True
        logger.info("Control Stream started")
        
    async def stop(self):
        """Stop the control stream."""
        if not self.running:
            return
            
        logger.info("Stopping Control Stream")
        
        # Cancel all tasks
        for task in self.tasks:
            task.cancel()
            
        # Wait for tasks to complete
        if self.tasks:
            await asyncio.gather(*self.tasks, return_exceptions=True)
            self.tasks = []
        
        # Close sockets
        if self.router_socket:
            self.router_socket.close()
            self.router_socket = None
            
        if self.dealer_socket:
            self.dealer_socket.close()
            self.dealer_socket = None
            
        self.running = False
        logger.info("Control Stream stopped")
        
    async def _router_dealer_proxy(self):
        """
        Run a ROUTER-DEALER proxy to route messages between external clients and internal workers.
        """
        try:
            # Create a poller for the sockets
            poller = zmq.asyncio.Poller()
            poller.register(self.router_socket, zmq.POLLIN)
            poller.register(self.dealer_socket, zmq.POLLIN)
            
            while self.running:
                try:
                    events = dict(await poller.poll(timeout=1000))
                    
                    # Forward messages from router to dealer
                    if self.router_socket in events:
                        message = await self.router_socket.recv_multipart()
                        await self.dealer_socket.send_multipart(message)
                        
                    # Forward messages from dealer to router
                    if self.dealer_socket in events:
                        message = await self.dealer_socket.recv_multipart()
                        await self.router_socket.send_multipart(message)
                        
                except asyncio.CancelledError:
                    break
                except Exception as e:
                    logger.error(f"Error in ROUTER-DEALER proxy: {e}")
                    await asyncio.sleep(0.1)
                    
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.error(f"Fatal error in ROUTER-DEALER proxy: {e}")
            
    async def _process_control_messages(self):
        """Process control messages from the dealer socket."""
        # Create a worker socket to connect to the dealer
        worker_socket = self.context.socket(zmq.DEALER)
        worker_socket.connect("inproc://control_backend")
        
        try:
            while self.running:
                try:
                    # Receive message
                    message_parts = await worker_socket.recv_multipart()
                    
                    # Message format: [client_id, empty, message]
                    if len(message_parts) < 3:
                        logger.warning(f"Invalid message format: {message_parts}")
                        continue
                        
                    client_id = message_parts[0]
                    message_data = message_parts[2]
                    
                    # Try to decode as JSON
                    try:
                        message = json.loads(message_data.decode('utf-8'))
                    except json.JSONDecodeError:
                        logger.error(f"Invalid JSON in control message: {message_data}")
                        # Send error response
                        error_response = {
                            "message_type": "error",
                            "error": "Invalid JSON format",
                            "timestamp": int(time.time() * 1000)
                        }
                        await worker_socket.send_multipart([
                            client_id, 
                            b'', 
                            json.dumps(error_response).encode('utf-8')
                        ])
                        continue
                    
                    # Handle ONLY legacy control message format (message_type field)
                    # REST API messages (method + route) are handled by dedicated REST stream
                    if isinstance(message, dict) and 'route' in message and 'method' in message:
                        # This is a REST API format message - it should go to the REST stream!
                        logger.warning(f"Received REST format message on control stream. "
                                     f"REST messages should be sent to the dedicated REST stream port.")
                        
                        error_response = {
                            "message_type": "error",
                            "error": "REST API messages should be sent to the dedicated REST stream port",
                            "timestamp": int(time.time() * 1000)
                        }
                        await worker_socket.send_multipart([
                            client_id,
                            b'',
                            json.dumps(error_response).encode('utf-8')
                        ])
                        continue
                    
                    # Process legacy control message
                    response = await self._handle_legacy_control_message(client_id, message)
                    
                    # Send response if any
                    if response:
                        await worker_socket.send_multipart([
                            client_id,
                            b'',
                            json.dumps(response).encode('utf-8')
                        ])
                        
                except asyncio.CancelledError:
                    break
                except Exception as e:
                    logger.error(f"Error processing control message: {e}")
                    await asyncio.sleep(0.1)
                    
        except asyncio.CancelledError:
            # Task was cancelled, exit gracefully
            pass
        except Exception as e:
            logger.error(f"Fatal error in control message processor: {e}")
        finally:
            # Clean up worker socket
            worker_socket.close()
            
    async def _handle_legacy_control_message(self, client_id: bytes, message: Dict[str, Any]) -> Dict[str, Any]:
        """
        Handle a legacy control message (backward compatibility).
        
        Args:
            client_id: Client's ZMQ identity
            message: Decoded JSON message
            
        Returns:
            Response data if a response is needed, otherwise None
        """
        message_type = message.get("message_type")
        
        if not message_type:
            logger.warning(f"Legacy message missing 'message_type' field: {message}")
            return {
                "message_type": "error",
                "error": "Missing message_type field",
                "timestamp": int(time.time() * 1000)
            }
            
        # Update client's last seen time
        client_id_str = client_id.decode('utf-8')
        if client_id_str in self.clients:
            self.clients[client_id_str]['last_seen'] = time.time()
            
        # Process standard message types
        if message_type == "hello":
            # Client registration
            agent_id = message.get("agent_id")
            agent_type = message.get("agent_type", "unknown")
            
            if not agent_id:
                return {
                    "message_type": "error",
                    "error": "Missing agent_id field in hello message",
                    "timestamp": int(time.time() * 1000)
                }
                
            # Register client
            self.clients[client_id_str] = {
                'agent_id': agent_id,
                'agent_type': agent_type,
                'last_seen': time.time()
            }
            
            logger.info(f"Client {agent_id} ({agent_type}) registered with ID {client_id_str}")
            
            # Send welcome response
            return {
                "message_type": "welcome",
                "server_id": self.server_id,
                "timestamp": int(time.time() * 1000)
            }
            
        elif message_type == "heartbeat":
            # Simple heartbeat, just respond with current status
            return {
                "message_type": "heartbeat_response",
                "status": "ok",
                "timestamp": int(time.time() * 1000)
            }
            
        # Forward to custom handlers if registered
        if message_type in self.message_handlers:
            try:
                return await self.message_handlers[message_type](client_id, message)
            except Exception as e:
                logger.error(f"Error in custom handler for {message_type}: {e}")
                return {
                    "message_type": "error",
                    "error": f"Internal error processing {message_type}",
                    "timestamp": int(time.time() * 1000)
                }
                
        # If we get here, we don't know how to handle this message
        logger.warning(f"Unknown message type: {message_type}")
        return {
            "message_type": "error",
            "error": f"Unknown message type: {message_type}",
            "timestamp": int(time.time() * 1000)
        }
    
    async def _cleanup_inactive_clients(self) -> None:
        """Periodically clean up inactive clients."""
        # Timeout for inactive clients (seconds)
        timeout = 60
        
        try:
            while self.running:
                # Check for inactive clients
                current_time = time.time()
                to_remove = []
                
                for client_id, info in self.clients.items():
                    if current_time - info['last_seen'] > timeout:
                        to_remove.append(client_id)
                        
                # Remove inactive clients
                for client_id in to_remove:
                    logger.info(f"Removing inactive client {self.clients[client_id].get('agent_id', 'unknown')} ({client_id})")
                    del self.clients[client_id]
                    
                # Sleep for a while
                await asyncio.sleep(10)
                
        except asyncio.CancelledError:
            # Task was cancelled, exit gracefully
            pass
        except Exception as e:
            logger.error(f"Error in client cleanup task: {e}")
    
    async def send_control_message(self, client_id: str, message: Dict[str, Any]) -> bool:
        """
        Send a control message to a specific client.
        
        Args:
            client_id: Client's ID (as stored in self.clients)
            message: Message to send (will be JSON encoded)
            
        Returns:
            True if message was sent, False otherwise
        """
        if not self.running or not self.router_socket:
            logger.warning("Cannot send control message: server not running")
            return False
            
        # Find the client's ZMQ ID
        zmq_id = None
        for id, info in self.clients.items():
            if info.get('agent_id') == client_id:
                zmq_id = id.encode('utf-8')
                break
                
        if not zmq_id:
            logger.warning(f"Client {client_id} not found")
            return False
            
        try:
            # Add timestamp if not present
            if "timestamp" not in message:
                message["timestamp"] = int(time.time() * 1000)
                
            # Send the message
            await self.router_socket.send_multipart([
                zmq_id,
                b'',
                json.dumps(message).encode('utf-8')
            ])
            
            return True
            
        except Exception as e:
            logger.error(f"Error sending control message to client {client_id}: {e}")
            return False
    
    def register_message_handler(self, message_type: str, handler: Callable) -> None:
        """
        Register a handler for a specific message type.
        
        Args:
            message_type: Type of message to handle
            handler: Function to call when a message of this type is received
                    (parameters: client_id, message, returns: response)
        """
        self.message_handlers[message_type] = handler
        logger.debug(f"Registered handler for message type: {message_type}")
        
    def get_connected_clients(self) -> List[Dict[str, Any]]:
        """
        Get information about connected clients.
        
        Returns:
            List of client information dictionaries
        """
        return [
            {
                'client_id': client_id,
                'agent_id': info.get('agent_id', 'unknown'),
                'agent_type': info.get('agent_type', 'unknown'),
                'last_seen': info.get('last_seen', 0)
            }
            for client_id, info in self.clients.items()
        ] 