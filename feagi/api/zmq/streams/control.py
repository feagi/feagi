"""
ZeroMQ Control Protocol Stream for FEAGI

This module implements the ZMQ stream for the FEAGI Control Protocol (FCP),
handling agent registration, configuration, status updates, and other
administrative functions.
"""

import asyncio
from feagi.utils.logger import setup_logger
logger = setup_logger(__name__)
import time
from typing import Dict, Any, Optional, Callable, List, Tuple

import zmq
import zmq.asyncio

# Update imports to use constants instead of base/fcp modules
from feagi.api.protocols.constants import ProtocolID, FCPCommandType
from feagi.api.protocols.translator import ByteStructureTranslator

# Define simplified message types just for compatibility
class FCPMessageType:
    """Compatibility class for FCPMessageType constants."""
    REGISTER = FCPCommandType.REGISTER
    DEREGISTER = FCPCommandType.DEREGISTER
    HEARTBEAT = FCPCommandType.HEARTBEAT
    STATUS_REQUEST = FCPCommandType.STATUS_REQUEST
    STATUS_RESPONSE = FCPCommandType.STATUS_RESPONSE
    CONFIGURE = FCPCommandType.CONFIGURE
    ERROR = FCPCommandType.ERROR
    
    # Response types for backwards compatibility
    REGISTER_RESPONSE = FCPCommandType.STATUS_RESPONSE  # Use STATUS_RESPONSE as a stand-in
    DEREGISTER_RESPONSE = FCPCommandType.STATUS_RESPONSE
    HEARTBEAT_RESPONSE = FCPCommandType.STATUS_RESPONSE
    CONFIGURE_RESPONSE = FCPCommandType.STATUS_RESPONSE

class ControlStream:
    """
    ZeroMQ stream for FEAGI Control Protocol (FCP).
    
    This class implements a specialized ZMQ stream for handling control
    messages between FEAGI and agents, including registration, configuration,
    status updates, and heartbeats.
    """
    
    def __init__(
        self,
        core_api,
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
        
        # Initialize server_id
        self.server_id = "feagi_server"
        
        # Initialize sockets
        self.router_socket = None
        self.dealer_socket = None
        
        # Protocol translator
        self.translator = ByteStructureTranslator()
        
        # Tracking
        self._running = False
        self._tasks = []
        
    async def start(self):
        """Start the control stream."""
        if self._running:
            return
            
        logger.info(f"Starting FCP control stream on {self.host}:{self.port}")
        
        # Create ROUTER socket (for external clients)
        self.router_socket = self.context.socket(zmq.ROUTER)
        self.router_socket.bind(f"tcp://{self.host}:{self.port}")
        
        # Create DEALER socket (for internal routing)
        self.dealer_socket = self.context.socket(zmq.DEALER)
        self.dealer_socket.bind("inproc://control_backend")
        
        # Start worker threads
        self._tasks.append(asyncio.create_task(self._router_dealer_proxy()))
        self._tasks.append(asyncio.create_task(self._process_control_messages()))
        
        self._running = True
        logger.info("FCP control stream started")
        
    async def stop(self):
        """Stop the control stream."""
        if not self._running:
            return
            
        logger.info("Stopping FCP control stream")
        
        # Cancel all tasks
        for task in self._tasks:
            task.cancel()
            
        # Wait for tasks to complete
        if self._tasks:
            await asyncio.gather(*self._tasks, return_exceptions=True)
            self._tasks = []
        
        # Close sockets
        if self.router_socket:
            self.router_socket.close()
            self.router_socket = None
            
        if self.dealer_socket:
            self.dealer_socket.close()
            self.dealer_socket = None
            
        self._running = False
        logger.info("FCP control stream stopped")
        
    async def _router_dealer_proxy(self):
        """
        Run a ROUTER-DEALER proxy to route messages between external clients and internal workers.
        
        This implements the Pirate Pattern from ZeroMQ, providing reliable messaging with heartbeats.
        """
        try:
            # Create a poller for the sockets
            poller = zmq.asyncio.Poller()
            poller.register(self.router_socket, zmq.POLLIN)
            poller.register(self.dealer_socket, zmq.POLLIN)
            
            while self._running:
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
            while self._running:
                try:
                    # Receive message
                    client_id, empty, message = await worker_socket.recv_multipart()
                    
                    # Decode the message
                    # Update to use the new decode method
                    decoded = self.translator.decode_message(message)
                    
                    # Process the message based on type
                    response = await self._handle_control_message(client_id, decoded)
                    
                    # Encode and send response
                    if response:
                        # Update to use the new encode method
                        encoded_response = self.translator.create_fcp_message(
                            response.get("type", FCPCommandType.FCP_REGISTER_RESPONSE),
                            response
                        )
                        await worker_socket.send_multipart([client_id, b'', encoded_response])
                        
                except asyncio.CancelledError:
                    break
                except Exception as e:
                    logger.error(f"Error processing control message: {e}")
                    await asyncio.sleep(0.1)
                    
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.error(f"Fatal error in control message processor: {e}")
        finally:
            worker_socket.close()
            
    async def _handle_control_message(self, client_id: bytes, message: Dict[str, Any]) -> Dict[str, Any]:
        """
        Handle a control message.
        
        Args:
            client_id: Client ID from the ROUTER socket
            message: Decoded message
            
        Returns:
            Response message
        """
        try:
            message_type = message.get("type")
            
            if message_type == FCPMessageType.REGISTER:
                # Handle agent registration
                agent_info = message.get("data", {})
                agent_id = agent_info.get("agent_id")
                agent_type = agent_info.get("agent_type")
                protocol_versions = agent_info.get("protocol_versions", {})
                
                # Register with core API
                registration = await self.core_api.register_agent(
                    agent_id=agent_id,
                    agent_type=agent_type,
                    protocol_versions=protocol_versions,
                    client_id=client_id.decode()
                )
                
                return {
                    "type": FCPMessageType.REGISTER_RESPONSE,
                    "status": "success" if registration else "error",
                    "data": registration
                }
                
            elif message_type == FCPMessageType.DEREGISTER:
                # Handle agent deregistration
                agent_id = message.get("data", {}).get("agent_id")
                
                # Deregister with core API
                success = await self.core_api.deregister_agent(agent_id)
                
                return {
                    "type": FCPMessageType.DEREGISTER_RESPONSE,
                    "status": "success" if success else "error"
                }
                
            elif message_type == FCPMessageType.HEARTBEAT:
                # Handle heartbeat
                agent_id = message.get("data", {}).get("agent_id")
                
                # Update agent heartbeat
                await self.core_api.update_agent_heartbeat(agent_id)
                
                return {
                    "type": FCPMessageType.HEARTBEAT_RESPONSE,
                    "timestamp": time.time()
                }
                
            elif message_type == FCPMessageType.STATUS_REQUEST:
                # Handle status request
                status = await self.core_api.get_status()
                
                return {
                    "type": FCPMessageType.STATUS_RESPONSE,
                    "data": status
                }
                
            elif message_type == FCPMessageType.CONFIGURE:
                # Handle configuration request
                config_data = message.get("data", {})
                agent_id = config_data.get("agent_id")
                config = config_data.get("config", {})
                
                # Apply configuration
                success = await self.core_api.configure_agent(agent_id, config)
                
                return {
                    "type": FCPMessageType.CONFIGURE_RESPONSE,
                    "status": "success" if success else "error"
                }
                
            else:
                logger.warning(f"Unknown control message type: {message_type}")
                return {
                    "type": FCPMessageType.ERROR,
                    "error": f"Unknown message type: {message_type}"
                }
                
        except Exception as e:
            logger.error(f"Error handling control message: {e}")
            return {
                "type": FCPMessageType.ERROR,
                "error": str(e)
            }
            
    async def send_control_message(self, agent_id: str, message_type: str, data: Dict[str, Any] = None) -> bool:
        """
        Send a control message to an agent.
        
        Args:
            agent_id: Agent ID
            message_type: Message type
            data: Message data
            
        Returns:
            True if sent successfully, False otherwise
        """
        if not self._running or not self.dealer_socket:
            return False
            
        try:
            # Get client ID for agent
            client_id = await self.core_api.get_agent_client_id(agent_id)
            if not client_id:
                logger.error(f"No client ID found for agent {agent_id}")
                return False
                
            # Create message
            message = {
                "type": message_type,
                "data": data or {},
                "timestamp": time.time()
            }
            
            # Encode message
            encoded = self.translator.encode(message, ProtocolID.FCP)
            
            # Send message
            await self.dealer_socket.send_multipart([client_id.encode(), b'', encoded])
            return True
            
        except Exception as e:
            logger.error(f"Error sending control message: {e}")
            return False 