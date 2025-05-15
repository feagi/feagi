"""
ZeroMQ message handlers for FEAGI.

This module implements asynchronous message handlers for different protocols
using the ROUTER-DEALER pattern and Cap'n Proto serialization.
"""

import asyncio
import logging
from typing import Any, Callable, Coroutine, Dict, Optional

import zmq
import zmq.asyncio
import capnp
from capnp import KjException

from feagi.api.zmq.connection_manager import ConnectionManager

# Configure logging
logger = logging.getLogger(__name__)


class MessageHandler:
    """
    Base class for protocol-specific message handlers.
    
    This class handles the common logic for receiving and processing
    messages from ZMQ ROUTER sockets with Cap'n Proto serialization.
    """
    
    def __init__(self,
                connection_manager: ConnectionManager,
                schema_loader: Callable,
                protocol_type: str):
        """
        Initialize the message handler.
        
        Args:
            connection_manager: Connection manager instance
            schema_loader: Function to load Cap'n Proto schema
            protocol_type: Protocol type ("fcp", "fsmp", or "fvp")
        """
        self.connection_manager = connection_manager
        self.protocol_type = protocol_type
        self.running = False
        self.task = None
        
        # Get appropriate socket based on protocol type
        if protocol_type == "fcp":
            self.socket = connection_manager.control_socket
        elif protocol_type == "fsmp":
            self.socket = connection_manager.sensorimotor_socket
        elif protocol_type == "fvp":
            self.socket = connection_manager.visualization_socket
        else:
            raise ValueError(f"Unsupported protocol type: {protocol_type}")
            
        # Load protocol schema
        try:
            self.schema = schema_loader()
            logger.info(f"Loaded schema for {protocol_type} protocol")
        except Exception as e:
            logger.error(f"Failed to load schema for {protocol_type}: {e}")
            raise
    
    async def start(self) -> None:
        """Start the message handler."""
        if self.running:
            return
            
        self.running = True
        self.task = asyncio.create_task(self._handle_messages())
        logger.info(f"Started {self.protocol_type} message handler")
    
    async def stop(self) -> None:
        """Stop the message handler."""
        if not self.running:
            return
            
        self.running = False
        if self.task:
            self.task.cancel()
            try:
                await self.task
            except asyncio.CancelledError:
                pass
        
        logger.info(f"Stopped {self.protocol_type} message handler")
    
    async def _handle_messages(self) -> None:
        """Main message handling loop."""
        logger.info(f"Listening for {self.protocol_type} messages")
        
        while self.running:
            try:
                # Receive message parts: [client_id, empty_frame, message_data]
                message_parts = await self.socket.recv_multipart()
                
                if len(message_parts) != 3:
                    logger.warning(f"Received malformed message: {message_parts}")
                    continue
                    
                client_id, empty, message_data = message_parts
                
                # Look up client by ZMQ identity
                agent_id, client_info = self.connection_manager.get_client_by_zmq_id(client_id)
                
                if not agent_id:
                    logger.warning(f"Received message from unknown client: {client_id.hex()}")
                    # We might want to handle new clients here, e.g., for initial handshake
                    continue
                
                # Update client activity
                self.connection_manager.update_client_activity(agent_id)
                
                # Process the message
                try:
                    # Decode Cap'n Proto message
                    decoded_message = self._decode_message(message_data)
                    
                    # Process message and generate response
                    response_data = await self._process_message(agent_id, decoded_message)
                    
                    # Send response if needed
                    if response_data:
                        # Encode response
                        encoded_response = self._encode_response(response_data)
                        
                        # Send response back to the client
                        await self.socket.send_multipart([client_id, b"", encoded_response])
                        
                except KjException as e:
                    logger.error(f"Cap'n Proto error processing message from {agent_id}: {e}")
                except Exception as e:
                    logger.error(f"Error processing message from {agent_id}: {e}")
                    
            except zmq.ZMQError as e:
                if e.errno == zmq.EAGAIN:
                    # No message available
                    await asyncio.sleep(0.01)
                else:
                    logger.error(f"ZMQ error in {self.protocol_type} handler: {e}")
                    await asyncio.sleep(1)  # Avoid tight loop on persistent errors
                    
            except asyncio.CancelledError:
                logger.info(f"{self.protocol_type} message handler cancelled")
                break
                
            except Exception as e:
                logger.error(f"Unexpected error in {self.protocol_type} handler: {e}")
                await asyncio.sleep(1)  # Avoid tight loop on persistent errors
    
    def _decode_message(self, message_data: bytes) -> Any:
        """
        Decode a Cap'n Proto message.
        
        Args:
            message_data: Raw message data
            
        Returns:
            Decoded message
        """
        # Override in subclass
        raise NotImplementedError
    
    async def _process_message(self, agent_id: str, message: Any) -> Optional[Any]:
        """
        Process a decoded message.
        
        Args:
            agent_id: Agent identifier
            message: Decoded message
            
        Returns:
            Response data if a response is needed, otherwise None
        """
        # Override in subclass
        raise NotImplementedError
    
    def _encode_response(self, response_data: Any) -> bytes:
        """
        Encode a response message.
        
        Args:
            response_data: Response data
            
        Returns:
            Encoded Cap'n Proto message
        """
        # Override in subclass
        raise NotImplementedError


class FCPMessageHandler(MessageHandler):
    """Message handler for FEAGI Control Protocol (FCP)."""
    
    def __init__(self, 
                connection_manager: ConnectionManager,
                process_message_callback: Callable[[str, Any], Coroutine[Any, Any, Optional[Any]]],
                schema_loader: Callable):
        """
        Initialize the FCP message handler.
        
        Args:
            connection_manager: Connection manager instance
            process_message_callback: Callback for processing decoded messages
            schema_loader: Function to load FCP Cap'n Proto schema
        """
        super().__init__(connection_manager, schema_loader, "fcp")
        self.process_message_callback = process_message_callback
    
    def _decode_message(self, message_data: bytes) -> Any:
        """Decode an FCP message."""
        return self.schema.FCPMessage.from_bytes(message_data)
    
    async def _process_message(self, agent_id: str, message: Any) -> Optional[Any]:
        """Process an FCP message."""
        return await self.process_message_callback(agent_id, message)
    
    def _encode_response(self, response_data: Any) -> bytes:
        """Encode an FCP response."""
        message = self.schema.FCPMessage.new_message(**response_data)
        return message.to_bytes()


class FSMPMessageHandler(MessageHandler):
    """Message handler for FEAGI Sensorimotor Protocol (FSMP)."""
    
    def __init__(self, 
                connection_manager: ConnectionManager,
                process_message_callback: Callable[[str, Any], Coroutine[Any, Any, Optional[Any]]],
                schema_loader: Callable):
        """
        Initialize the FSMP message handler.
        
        Args:
            connection_manager: Connection manager instance
            process_message_callback: Callback for processing decoded messages
            schema_loader: Function to load FSMP Cap'n Proto schema
        """
        super().__init__(connection_manager, schema_loader, "fsmp")
        self.process_message_callback = process_message_callback
    
    def _decode_message(self, message_data: bytes) -> Any:
        """Decode an FSMP message."""
        return self.schema.FSMPMessage.from_bytes(message_data)
    
    async def _process_message(self, agent_id: str, message: Any) -> Optional[Any]:
        """Process an FSMP message."""
        return await self.process_message_callback(agent_id, message)
    
    def _encode_response(self, response_data: Any) -> bytes:
        """Encode an FSMP response."""
        message = self.schema.FSMPMessage.new_message(**response_data)
        return message.to_bytes()


class FVPMessageHandler(MessageHandler):
    """Message handler for FEAGI Visualization Protocol (FVP)."""
    
    def __init__(self, 
                connection_manager: ConnectionManager,
                process_message_callback: Callable[[str, Any], Coroutine[Any, Any, Optional[Any]]],
                schema_loader: Callable):
        """
        Initialize the FVP message handler.
        
        Args:
            connection_manager: Connection manager instance
            process_message_callback: Callback for processing decoded messages
            schema_loader: Function to load FVP Cap'n Proto schema
        """
        super().__init__(connection_manager, schema_loader, "fvp")
        self.process_message_callback = process_message_callback
    
    def _decode_message(self, message_data: bytes) -> Any:
        """Decode an FVP message."""
        return self.schema.FVPMessage.from_bytes(message_data)
    
    async def _process_message(self, agent_id: str, message: Any) -> Optional[Any]:
        """Process an FVP message."""
        return await self.process_message_callback(agent_id, message)
    
    def _encode_response(self, response_data: Any) -> bytes:
        """Encode an FVP response."""
        message = self.schema.FVPMessage.new_message(**response_data)
        return message.to_bytes()


class HandshakeMessageHandler(MessageHandler):
    """Message handler for Handshake Protocol."""
    
    def __init__(self, 
                connection_manager: ConnectionManager,
                process_message_callback: Callable[[str, Any], Coroutine[Any, Any, Optional[Any]]],
                schema_loader: Callable):
        """
        Initialize the Handshake message handler.
        
        Args:
            connection_manager: Connection manager instance
            process_message_callback: Callback for processing decoded messages
            schema_loader: Function to load Handshake Cap'n Proto schema
        """
        # Handshake uses the control socket (FCP)
        super().__init__(connection_manager, schema_loader, "fcp")
        self.protocol_type = "handshake"  # Override protocol type for logging
        self.process_message_callback = process_message_callback
    
    def _decode_message(self, message_data: bytes) -> Any:
        """Decode a Handshake message."""
        return self.schema.HandshakeMessage.from_bytes(message_data)
    
    async def _process_message(self, agent_id: str, message: Any) -> Optional[Any]:
        """
        Process a Handshake message.
        
        For new clients (agent_id is None), we generate an agent ID from the hello message.
        """
        if message.type == self.schema.HandshakeMessageType.hello:
            # For hello messages, we might not have an agent ID yet
            # The client ID is provided in the hello message
            client_id = message.hello.agentId
            client_type = message.hello.agentType
            logger.info(f"Received hello from {client_type} client '{client_id}'")
            
            # Use the provided agent_id or generate a new one
            agent_id = client_id
        
        # Process the message using the callback
        return await self.process_message_callback(agent_id, message)
    
    def _encode_response(self, response_data: Any) -> bytes:
        """Encode a Handshake response."""
        message = self.schema.HandshakeMessage.new_message(**response_data)
        return message.to_bytes()


async def start_message_handlers(connection_manager: ConnectionManager, 
                               schema_loaders: Dict[str, Callable],
                               message_processors: Dict[str, Callable]) -> Dict[str, MessageHandler]:
    """
    Start all message handlers.
    
    Args:
        connection_manager: Connection manager instance
        schema_loaders: Dictionary mapping protocol types to schema loader functions
        message_processors: Dictionary mapping protocol types to message processor callbacks
        
    Returns:
        Dictionary of message handlers
    """
    handlers = {}
    
    # Create handlers
    handlers["handshake"] = HandshakeMessageHandler(
        connection_manager, 
        message_processors["handshake"], 
        schema_loaders["handshake"]
    )
    
    handlers["fcp"] = FCPMessageHandler(
        connection_manager, 
        message_processors["fcp"], 
        schema_loaders["fcp"]
    )
    
    handlers["fsmp"] = FSMPMessageHandler(
        connection_manager, 
        message_processors["fsmp"], 
        schema_loaders["fsmp"]
    )
    
    handlers["fvp"] = FVPMessageHandler(
        connection_manager, 
        message_processors["fvp"], 
        schema_loaders["fvp"]
    )
    
    # Start handlers
    for protocol, handler in handlers.items():
        await handler.start()
    
    return handlers


async def stop_message_handlers(handlers: Dict[str, MessageHandler]) -> None:
    """
    Stop all message handlers.
    
    Args:
        handlers: Dictionary of message handlers
    """
    for protocol, handler in handlers.items():
        await handler.stop() 