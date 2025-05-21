"""
ZeroMQ ROUTER-DEALER server implementation for FEAGI.

This module implements the ZMQ server that handles communication with
external clients using the ROUTER-DEALER pattern and custom byte structure serialization.
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
from feagi.api.protocols.translator import ByteStructureTranslator
from feagi.api.protocols.constants import ProtocolID, FCPCommandType

# Import the utility function for converting raw data to neuron data
from feagi_bytes.utils import convert_raw_to_neuron_data

# Configure logging
logger = logging.getLogger(__name__)


class ZMQRouterServer:
    """
    ZeroMQ server using ROUTER-DEALER pattern for FEAGI.
    
    This class integrates the ConnectionManager, MessageHandlers, and
    ByteStructureTranslator to handle byte structure messages from multiple clients.
    """
    
    def __init__(self,
                context: Optional[zmq.asyncio.Context] = None,
                control_port: int = 5559,
                sensorimotor_port: int = 5558,
                visualization_port: int = 5560):
        """
        Initialize the ZMQ server.
        
        Args:
            context: ZMQ async context (will create a new one if None)
            control_port: Port for control messages (FCP protocol)
            sensorimotor_port: Port for sensorimotor data (FSMP protocol)
            visualization_port: Port for visualization data (FVP protocol)
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
        self.translator = ByteStructureTranslator()
        
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
            None,  # No schema loaders with new implementation
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
    
    async def _process_handshake_message(self, agent_id: str, message: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Process a handshake message.
        
        Args:
            agent_id: Agent ID (may be None for hello messages)
            message: Decoded byte structure handshake message
            
        Returns:
            Response data if a response is needed, otherwise None
        """
        try:
            # Handle different message types
            if message["message_type"] == "hello":
                client_id = message["agent_id"]
                client_type = message["agent_type"]
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
                
                return welcome_msg
                
            elif message["message_type"] == "capabilities":
                if agent_id not in self.pending_clients:
                    logger.warning(f"Received capabilities from unknown client {agent_id}")
                    return None
                    
                # Extract capabilities
                sensory_channels = message["supported_sensory_channels"]
                motor_channels = message["supported_motor_channels"]
                
                # Extract protocol versions
                protocol_versions = message["protocol_versions"]
                
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
                        return self.translator.create_handshake_configuration({
                            # Add server configuration here
                            "server_id": self.server_id,
                            "server_time": asyncio.get_running_loop().time()
                        })
                
            return None
                
        except Exception as e:
            logger.error(f"Error processing handshake message: {e}")
            return None
    
    async def _process_fcp_message(self, agent_id: str, message: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Process an FCP message.
        
        Args:
            agent_id: Agent ID
            message: Decoded byte structure FCP message
            
        Returns:
            Response data if a response is needed, otherwise None
        """
        logger.info(f"Processing FCP message from {agent_id}")
        
        # Handle specific FCP message types here
        # For now, just log and return None (no response)
        
        return None
    
    async def _process_fsmp_message(self, agent_id: str, message: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Process an FSMP message.
        
        Args:
            agent_id: Agent ID
            message: Decoded byte structure FSMP message
            
        Returns:
            Response data if a response is needed, otherwise None
        """
        try:
            # Log detailed message info
            message_type = message.get("message_type", "unknown")
            logger.info(f"Processing FSMP {message_type} message from agent '{agent_id}'")
            
            if message_type == "sensory":
                # Handle sensory data
                channel_id = message.get("channel_id")
                
                # Check if data is present
                if "data" in message:
                    data = message.get("data")
                    data_type = type(data).__name__
                    data_size = len(data) if isinstance(data, (bytes, list, tuple)) else "unknown"
                    
                    # Log detailed information about the received data
                    logger.info(f"Received sensory data from agent '{agent_id}': "
                              f"channel={channel_id}, type={data_type}, size={data_size}")
                    
                    # For numeric data, log statistics if possible
                    if isinstance(data, list) and all(isinstance(x, (int, float)) for x in data):
                        # Calculate some basic statistics
                        data_min = min(data)
                        data_max = max(data)
                        data_avg = sum(data) / len(data)
                        logger.info(f"Sensory data statistics: min={data_min:.2f}, max={data_max:.2f}, avg={data_avg:.2f}")
                        
                        # Convert to neuron data format (array type)
                        # Generate a standard 6-character cortical_id format
                        # This ensures we're using proper cortical_id (string identifier)
                        # rather than converting cortical_idx to string
                        cortical_id = f"CH{channel_id}".ljust(6)[:6]
                        
                        # Using the parameter name 'cortical_area_id' here because we're calling 
                        # an external library API which requires that specific parameter name
                        neuron_data = convert_raw_to_neuron_data(
                            data=data,
                            data_type="array",
                            cortical_area_id=cortical_id  # This is the expected parameter name in feagi_bytes library
                        )
                    
                    # For binary data, log the first few bytes
                    elif isinstance(data, bytes) and len(data) > 0:
                        preview = " ".join([f"{b:02x}" for b in data[:16]])
                        logger.info(f"Sensory data preview: {preview}...")
                        
                        # Try to determine if this is image data
                        # Here we're assuming it's image data with standard dimensions
                        # In a real implementation, the client would provide this metadata
                        if data_size in [28*28, 64*64, 64*64*3, 128*128, 128*128*3]:
                            # Guess dimensions and channels
                            if data_size == 28*28:
                                dimensions = (28, 28)
                                channels = 1
                            elif data_size == 64*64:
                                dimensions = (64, 64)
                                channels = 1
                            elif data_size == 64*64*3:
                                dimensions = (64, 64)
                                channels = 3
                            elif data_size == 128*128:
                                dimensions = (128, 128)
                                channels = 1
                            else:
                                dimensions = (128, 128)
                                channels = 3
                                
                            logger.info(f"Treating binary data as image with dimensions {dimensions} and {channels} channels")
                            
                            # Convert to neuron data format (image type)
                            # Note: router_server receives raw binary data directly without byte structure headers
                            # Generate a standard 6-character cortical_id format
                            # This ensures we're using proper cortical_id (string identifier)
                            # rather than converting cortical_idx to string
                            cortical_id = f"CH{channel_id}".ljust(6)[:6]
                            
                            # Using the parameter name 'cortical_area_id' here because we're calling 
                            # an external library API which requires that specific parameter name
                            neuron_data = convert_raw_to_neuron_data(
                                data=data,
                                data_type="image",
                                dimensions=dimensions,
                                channels=channels,
                                cortical_area_id=cortical_id  # This is the expected parameter name in feagi_bytes library
                            )
                        else:
                            # Treat as unknown binary data
                            logger.warning(f"Unknown binary data format with size {data_size}, not converting to neuron data")
                            return None
                    
                    # For other data types
                    else:
                        logger.warning(f"Unsupported data type: {data_type}, not converting to neuron data")
                        return None
                    
                    # Log the neuron data conversion result
                    for area_cortical_id, area_data in neuron_data.items():
                        num_neurons = len(area_data["x"])
                        if num_neurons > 0:
                            pot_min = min(area_data["potentials"]) if area_data["potentials"] else 0
                            pot_max = max(area_data["potentials"]) if area_data["potentials"] else 0
                            pot_avg = sum(area_data["potentials"]) / len(area_data["potentials"]) if area_data["potentials"] else 0
                            
                            logger.info(f"Converted to {num_neurons} neurons in area {area_cortical_id}: "
                                      f"potential min={pot_min:.2f}, max={pot_max:.2f}, avg={pot_avg:.2f}")
                    
                    # TODO: Forward this neuron data to the appropriate processing module
                    # In a real implementation, you would pass this to the FEAGI core
                    
                    # No response needed for sensory data
                    return None
                
                else:
                    logger.warning(f"Received sensory message without data from agent '{agent_id}'")
            
            elif message_type == "motor":
                # Handle motor data request (client requesting motor output)
                logger.info(f"Received motor data request from agent '{agent_id}'")
                # TODO: Implement motor data handling
                
            else:
                logger.warning(f"Unknown FSMP message type: {message_type} from agent '{agent_id}'")
            
            # No response needed
            return None
            
        except Exception as e:
            logger.exception(f"Error processing FSMP message from {agent_id}: {e}")
            return None
    
    async def _process_fvp_message(self, agent_id: str, message: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Process an FVP message.
        
        Args:
            agent_id: Agent ID
            message: Decoded byte structure FVP message
            
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
                
            # Create and encode message depending on protocol
            if protocol_type == "fcp":
                message = self.translator.create_fcp_message(
                    command_type=message_data.get("command_type", FCPCommandType.STATUS_RESPONSE),
                    payload=message_data.get("payload", {})
                )
            elif protocol_type == "fsmp":
                if "channel_id" not in message_data:
                    logger.warning(f"Missing channel_id in FSMP message for {agent_id}")
                    continue
                    
                if message_data.get("message_type") == "sensory":
                    message = self.translator.create_fsmp_sensory_data(
                        channel_id=message_data["channel_id"],
                        data=message_data.get("data", [])
                    )
                else:  # Default to motor
                    message = self.translator.create_fsmp_motor_data(
                        channel_id=message_data["channel_id"],
                        data=message_data.get("data", [])
                    )
            elif protocol_type == "fvp":
                # For now, just encode as JSON
                message = self.translator.encoder.encode_json(message_data)
                
            # Send message
            success = await self.connection_manager.send_message(
                agent_id=agent_id,
                protocol_type=protocol_type,
                message=message  # Already encoded
            )
            
            if success:
                count += 1
                
        return count 