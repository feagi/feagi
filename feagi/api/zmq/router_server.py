"""
ZeroMQ ROUTER-DEALER server implementation for FEAGI.

This module implements the ZMQ server that handles communication with
external clients using the ROUTER-DEALER pattern and Cap'n Proto serialization.
"""

import asyncio
import logging
import os
import uuid
from typing import Dict, Any, Optional, List, Callable, Coroutine

import zmq
import zmq.asyncio

from feagi.api.zmq.connection_manager import ConnectionManager
from feagi.api.zmq.message_handlers import MessageHandler, start_message_handlers, stop_message_handlers
from feagi.api.protocols.translator import ProtocolTranslator

# Configure logging
logger = logging.getLogger(__name__)


class ZMQRouterServer:
    """
    ZeroMQ server using ROUTER-DEALER pattern for FEAGI.
    
    This class integrates the ConnectionManager, MessageHandlers, and
    ProtocolTranslator to handle Cap'n Proto messages from multiple clients.
    """
    
    def __init__(self,
                context: Optional[zmq.asyncio.Context] = None,
                control_port: int = 5559,
                sensorimotor_port: int = 5558,
                visualization_port: int = 5560,
                schema_path: Optional[str] = None):
        """
        Initialize the ZMQ server.
        
        Args:
            context: ZMQ async context (will create a new one if None)
            control_port: Port for control messages (FCP protocol)
            sensorimotor_port: Port for sensorimotor data (FSMP protocol)
            visualization_port: Port for visualization data (FVP protocol)
            schema_path: Path to Cap'n Proto schemas
        """
        self.context = context or zmq.asyncio.Context.instance()
        self.running = False
        self.server_id = f"feagi-{uuid.uuid4().hex[:8]}"
        
        # Create connection manager
        self.connection_manager = ConnectionManager(
            context=self.context,
            control_port=control_port,
            sensorimotor_port=sensorimotor_port,
            visualization_port=visualization_port
        )
        
        # Create protocol translator
        self.translator = ProtocolTranslator(schema_path=schema_path)
        
        # Store message handlers (created during start())
        self.message_handlers: Dict[str, MessageHandler] = {}
        
        # Track clients awaiting handshake completion
        self.pending_clients: Dict[str, Dict[str, Any]] = {}
        
        # Message processing callbacks
        self.message_processors = {
            "handshake": self._process_handshake_message,
            "fcp": self._process_fcp_message,
            "fsmp": self._process_fsmp_message,
            "fvp": self._process_fvp_message
        }
        
        # Schema loaders
        self.schema_loaders = {
            "handshake": self.translator.get_schema_loader("handshake"),
            "fcp": self.translator.get_schema_loader("fcp"),
            "fsmp": self.translator.get_schema_loader("fsmp"),
            "fvp": self.translator.get_schema_loader("fvp")
        }
        
        # Cleanup task
        self.cleanup_task = None
    
    async def start(self) -> None:
        """Start the ZMQ server."""
        if self.running:
            logger.warning("ZMQ ROUTER server is already running")
            return
            
        logger.info(f"Starting ZMQ ROUTER server with ID {self.server_id}")
        
        # Start message handlers
        self.message_handlers = await start_message_handlers(
            self.connection_manager,
            self.schema_loaders,
            self.message_processors
        )
        
        # Start client cleanup task
        self.cleanup_task = asyncio.create_task(self._cleanup_inactive_clients())
        
        self.running = True
        logger.info("ZMQ ROUTER server started")
    
    async def stop(self) -> None:
        """Stop the ZMQ server."""
        if not self.running:
            logger.warning("ZMQ ROUTER server is not running")
            return
            
        logger.info("Stopping ZMQ ROUTER server")
        
        # Stop cleanup task
        if self.cleanup_task:
            self.cleanup_task.cancel()
            try:
                await self.cleanup_task
            except asyncio.CancelledError:
                pass
        
        # Stop message handlers
        await stop_message_handlers(self.message_handlers)
        
        # Close connection manager
        self.connection_manager.close()
        
        self.running = False
        logger.info("ZMQ ROUTER server stopped")
    
    async def _cleanup_inactive_clients(self) -> None:
        """Periodically clean up inactive clients."""
        try:
            while True:
                # Wait for a while
                await asyncio.sleep(30)
                
                # Find inactive clients
                inactive_clients = self.connection_manager.get_inactive_clients(timeout_seconds=60)
                
                # Deregister inactive clients
                for agent_id in inactive_clients:
                    logger.info(f"Deregistering inactive client {agent_id}")
                    self.connection_manager.deregister_client(agent_id)
                    
                # Clean up pending clients
                now = asyncio.get_running_loop().time()
                for client_id in list(self.pending_clients.keys()):
                    client_info = self.pending_clients[client_id]
                    if now - client_info["timestamp"] > 30:  # 30 seconds timeout
                        logger.info(f"Removing pending client {client_id} due to timeout")
                        del self.pending_clients[client_id]
                        
        except asyncio.CancelledError:
            logger.info("Cleanup task cancelled")
            
        except Exception as e:
            logger.error(f"Error in cleanup task: {e}")
    
    async def _process_handshake_message(self, agent_id: str, message: Any) -> Optional[Dict[str, Any]]:
        """
        Process a handshake message.
        
        Args:
            agent_id: Agent ID (may be None for hello messages)
            message: Decoded Cap'n Proto handshake message
            
        Returns:
            Response data if a response is needed, otherwise None
        """
        try:
            # Convert message to dictionary for easier handling
            message_dict = self.translator.handshake_message_to_dict(message)
            
            # Handle different message types
            if message.type == message.type.hello:
                client_id = message.hello.agentId
                client_type = message.hello.agentType
                zmq_id = None  # We don't have the ZMQ ID yet, handled by HandshakeMessageHandler
                
                logger.info(f"Processing hello from {client_type} client '{client_id}'")
                
                # Store client ID -> agent ID mapping for subsequent messages
                self.pending_clients[client_id] = {
                    "agent_id": client_id,
                    "agent_type": client_type,
                    "timestamp": asyncio.get_running_loop().time()
                }
                
                # Create welcome message
                welcome_msg = self.translator.create_handshake_welcome(
                    server_id=self.server_id,
                    message=f"Welcome to FEAGI {client_type} '{client_id}'"
                )
                
                # Convert to dictionary for response
                return {
                    "protocolId": welcome_msg.protocolId,
                    "version": welcome_msg.version,
                    "type": welcome_msg.type,
                    "welcome": {
                        "serverId": welcome_msg.welcome.serverId,
                        "message": welcome_msg.welcome.message,
                        "timestamp": welcome_msg.welcome.timestamp
                    }
                }
                
            elif message.type == message.type.capabilities:
                if agent_id not in self.pending_clients:
                    logger.warning(f"Received capabilities from unknown client {agent_id}")
                    return None
                    
                # Extract capabilities
                sensory_channels = list(message.capabilities.supportedSensoryChannels)
                motor_channels = list(message.capabilities.supportedMotorChannels)
                
                # Extract protocol versions
                protocol_versions = {
                    "fcp": message.capabilities.protocolVersions.fcpVersion,
                    "fsmp": message.capabilities.protocolVersions.fsmpVersion,
                    "fvp": message.capabilities.protocolVersions.fvpVersion
                }
                
                logger.info(f"Received capabilities from {agent_id}: "
                           f"sensory={sensory_channels}, motor={motor_channels}, "
                           f"protocols={protocol_versions}")
                
                # Register client in connection manager
                # Note: The HandshakeMessageHandler should have updated the ZMQ ID
                client_info = self.pending_clients.get(agent_id)
                if client_info:
                    # Get ZMQ ID from somewhere (needs to be passed from handler)
                    zmq_id = client_info.get("zmq_id")
                    if zmq_id:
                        # Register client with connection manager
                        self.connection_manager.register_client(
                            agent_id=agent_id,
                            zmq_id=zmq_id,
                            supported_protocols=protocol_versions
                        )
                        
                        # Clean up pending client
                        del self.pending_clients[agent_id]
                        
                        # Create configuration message
                        config_msg = self.translator.create_handshake_configuration({
                            # Add server configuration here
                        })
                        
                        # Convert to dictionary for response
                        return {
                            "protocolId": config_msg.protocolId,
                            "version": config_msg.version,
                            "type": config_msg.type,
                            "configuration": {
                                "timestamp": config_msg.configuration.timestamp
                                # Add configuration fields
                            }
                        }
                
            return None
                
        except Exception as e:
            logger.error(f"Error processing handshake message: {e}")
            return None
    
    async def _process_fcp_message(self, agent_id: str, message: Any) -> Optional[Dict[str, Any]]:
        """
        Process an FCP message.
        
        Args:
            agent_id: Agent ID
            message: Decoded Cap'n Proto FCP message
            
        Returns:
            Response data if a response is needed, otherwise None
        """
        logger.info(f"Processing FCP message from {agent_id}")
        
        # Handle specific FCP message types here
        # For now, just log and return None (no response)
        
        return None
    
    async def _process_fsmp_message(self, agent_id: str, message: Any) -> Optional[Dict[str, Any]]:
        """
        Process an FSMP message.
        
        Args:
            agent_id: Agent ID
            message: Decoded Cap'n Proto FSMP message
            
        Returns:
            Response data if a response is needed, otherwise None
        """
        logger.info(f"Processing FSMP message from {agent_id}")
        
        # Handle specific FSMP message types here
        # For now, just log and return None (no response)
        
        return None
    
    async def _process_fvp_message(self, agent_id: str, message: Any) -> Optional[Dict[str, Any]]:
        """
        Process an FVP message.
        
        Args:
            agent_id: Agent ID
            message: Decoded Cap'n Proto FVP message
            
        Returns:
            Response data if a response is needed, otherwise None
        """
        logger.info(f"Processing FVP message from {agent_id}")
        
        # Handle specific FVP message types here
        # For now, just log and return None (no response)
        
        return None
    
    def get_server_stats(self) -> Dict[str, Any]:
        """
        Get server statistics.
        
        Returns:
            Dictionary with server statistics
        """
        return {
            "server_id": self.server_id,
            "running": self.running,
            "connections": self.connection_manager.get_connection_stats(),
            "pending_clients": len(self.pending_clients)
        }
    
    async def broadcast_message(self, 
                              protocol_type: str, 
                              message_data: Dict[str, Any],
                              filter_func: Optional[Callable[[str, Dict[str, Any]], bool]] = None) -> int:
        """
        Broadcast a message to all connected clients or a filtered subset.
        
        Args:
            protocol_type: Protocol type ("fcp", "fsmp", or "fvp")
            message_data: Message data to broadcast
            filter_func: Function to filter clients (agent_id, client_info) -> bool
            
        Returns:
            Number of clients the message was sent to
        """
        # Check protocol type
        if protocol_type not in ["fcp", "fsmp", "fvp"]:
            raise ValueError(f"Invalid protocol type: {protocol_type}")
            
        # Get all connected clients
        count = 0
        for agent_id, client_info in self.connection_manager.connections.items():
            # Apply filter if provided
            if filter_func and not filter_func(agent_id, client_info):
                continue
                
            # Create and encode message
            if protocol_type == "fcp":
                message = self.translator.fcp_schema.FCPMessage.new_message(**message_data)
            elif protocol_type == "fsmp":
                message = self.translator.fsmp_schema.FSMPMessage.new_message(**message_data)
            elif protocol_type == "fvp":
                message = self.translator.fvp_schema.FVPMessage.new_message(**message_data)
                
            # Send message
            encoded_message = message.to_bytes()
            success = await self.connection_manager.send_message(
                agent_id=agent_id,
                protocol_type=protocol_type,
                message=encoded_message
            )
            
            if success:
                count += 1
                
        return count 