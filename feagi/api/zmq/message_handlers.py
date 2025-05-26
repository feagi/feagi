"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
ZeroMQ message handlers for FEAGI.

This module implements asynchronous message handlers for different protocols
using the ROUTER-DEALER pattern and custom byte structure serialization.
"""

import asyncio
import logging
from typing import Any, Callable, Coroutine, Dict, Optional, Callable

import zmq
import zmq.asyncio

from feagi.api.zmq.connection_manager import ConnectionManager
from feagi.api.protocols.translator import ByteStructureTranslator

# Configure logging
logger = logging.getLogger(__name__)


class MessageHandler:
    """
    Base class for protocol-specific message handlers.
    
    This class handles the common logic for receiving and processing
    messages from ZMQ ROUTER sockets with byte structure serialization.
    """
    
    def __init__(self,
                connection_manager: ConnectionManager,
                translator: ByteStructureTranslator,
                protocol_type: str):
        """
        Initialize the message handler.
        
        Args:
            connection_manager: Connection manager instance
            translator: ByteStructureTranslator instance
            protocol_type: Protocol type ("fcp", "fsmp", or "fvp")
        """
        self.connection_manager = connection_manager
        self.translator = translator
        self.protocol_type = protocol_type
        self.running = False
        self.task = None
        
        # Get appropriate socket based on protocol type
        if protocol_type == "fcp":
            self.socket = connection_manager.control_socket
        elif protocol_type == "fsmp":
            # Using sensory socket for FSMP as it's primarily for receiving data
            self.socket = connection_manager.sensory_socket
        elif protocol_type == "fsmp_motor":
            # Separate handler type for motor data
            self.socket = connection_manager.motor_socket
        elif protocol_type == "fvp":
            self.socket = connection_manager.visualization_socket
        else:
            raise ValueError(f"Unsupported protocol type: {protocol_type}")
    
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
                    # Decode byte structure message
                    decoded_message = self._decode_message(message_data)
                    
                    # Process message and generate response
                    response_data = await self._process_message(agent_id, decoded_message)
                    
                    # Send response if needed
                    if response_data:
                        # Response data is already encoded by protocol handlers
                        await self.socket.send_multipart([client_id, b"", response_data])
                        
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
    
    def _decode_message(self, message_data: bytes) -> Dict[str, Any]:
        """
        Decode a byte structure message.
        
        Args:
            message_data: Raw message data
            
        Returns:
            Decoded message as dictionary
        """
        # Use the translator to decode the message
        return self.translator.decode_message(message_data)
    
    async def _process_message(self, agent_id: str, message: Dict[str, Any]) -> Optional[bytes]:
        """
        Process a decoded message.
        
        Args:
            agent_id: Agent identifier
            message: Decoded message
            
        Returns:
            Response bytes if a response is needed, otherwise None
        """
        # Override in subclass
        raise NotImplementedError


class FCPMessageHandler(MessageHandler):
    """Message handler for FEAGI Control Protocol (FCP)."""
    
    def __init__(self, 
                connection_manager: ConnectionManager,
                process_message_callback: Callable[[str, Dict[str, Any]], Coroutine[Any, Any, Optional[Dict[str, Any]]]],
                translator: ByteStructureTranslator):
        """
        Initialize the FCP message handler.
        
        Args:
            connection_manager: Connection manager instance
            process_message_callback: Callback for processing decoded messages
            translator: ByteStructureTranslator instance
        """
        super().__init__(connection_manager, translator, "fcp")
        self.process_message_callback = process_message_callback
    
    async def _process_message(self, agent_id: str, message: Dict[str, Any]) -> Optional[bytes]:
        """Process an FCP message."""
        response_data = await self.process_message_callback(agent_id, message)
        if response_data is not None:
            return response_data  # Already encoded
        return None


class FSMPMessageHandler(MessageHandler):
    """Message handler for FEAGI Sensorimotor Protocol (FSMP)."""
    
    def __init__(self, 
                connection_manager: ConnectionManager,
                process_message_callback: Callable[[str, Dict[str, Any]], Coroutine[Any, Any, Optional[Dict[str, Any]]]],
                translator: ByteStructureTranslator):
        """
        Initialize the FSMP message handler.
        
        Args:
            connection_manager: Connection manager instance
            process_message_callback: Callback for processing decoded messages
            translator: ByteStructureTranslator instance
        """
        super().__init__(connection_manager, translator, "fsmp")
        self.process_message_callback = process_message_callback
    
    async def _process_message(self, agent_id: str, message: Dict[str, Any]) -> Optional[bytes]:
        """Process an FSMP message."""
        response_data = await self.process_message_callback(agent_id, message)
        if response_data is not None:
            return response_data  # Already encoded
        return None


class FVPMessageHandler(MessageHandler):
    """Message handler for FEAGI Visualization Protocol (FVP)."""
    
    def __init__(self, 
                connection_manager: ConnectionManager,
                process_message_callback: Callable[[str, Dict[str, Any]], Coroutine[Any, Any, Optional[Dict[str, Any]]]],
                translator: ByteStructureTranslator):
        """
        Initialize the FVP message handler.
        
        Args:
            connection_manager: Connection manager instance
            process_message_callback: Callback for processing decoded messages
            translator: ByteStructureTranslator instance
        """
        super().__init__(connection_manager, translator, "fvp")
        self.process_message_callback = process_message_callback
    
    async def _process_message(self, agent_id: str, message: Dict[str, Any]) -> Optional[bytes]:
        """Process an FVP message."""
        response_data = await self.process_message_callback(agent_id, message)
        if response_data is not None:
            return response_data  # Already encoded
        return None


class HandshakeMessageHandler(MessageHandler):
    """Message handler for Handshake Protocol."""
    
    def __init__(self, 
                connection_manager: ConnectionManager,
                process_message_callback: Callable[[str, Dict[str, Any]], Coroutine[Any, Any, Optional[Dict[str, Any]]]],
                translator: ByteStructureTranslator):
        """
        Initialize the Handshake message handler.
        
        Args:
            connection_manager: Connection manager instance
            process_message_callback: Callback for processing decoded messages
            translator: ByteStructureTranslator instance
        """
        # Handshake uses the control socket (FCP)
        super().__init__(connection_manager, translator, "fcp")
        self.protocol_type = "handshake"  # Override protocol type for logging
        self.process_message_callback = process_message_callback
    
    async def _process_message(self, agent_id: str, message: Dict[str, Any]) -> Optional[bytes]:
        """
        Process a Handshake message.
        
        For new clients (agent_id is None), we generate an agent ID from the hello message.
        """
        if message.get("message_type") == "hello":
            # For hello messages, we might not have an agent ID yet
            # The client ID is provided in the hello message
            client_id = message.get("agent_id")
            client_type = message.get("agent_type")
            logger.info(f"Received hello from {client_type} client '{client_id}'")
            
            # Use the provided agent_id
            agent_id = client_id
        
        # Process the message using the callback
        response_data = await self.process_message_callback(agent_id, message)
        return response_data  # Already encoded


async def start_message_handlers(connection_manager: ConnectionManager, 
                               schema_loaders: Optional[Dict[str, Callable]],
                               message_processors: Dict[str, Callable]) -> Dict[str, MessageHandler]:
    """
    Start all message handlers.
    
    Args:
        connection_manager: Connection manager instance
        schema_loaders: Ignored, kept for backward compatibility
        message_processors: Dictionary mapping protocol types to message processor callbacks
        
    Returns:
        Dictionary of message handlers
    """
    handlers = {}
    
    # Create byte structure translator
    translator = ByteStructureTranslator()
    
    # Create handlers
    handlers["handshake"] = HandshakeMessageHandler(
        connection_manager, 
        message_processors["handshake"], 
        translator
    )
    
    handlers["fcp"] = FCPMessageHandler(
        connection_manager, 
        message_processors["fcp"], 
        translator
    )
    
    handlers["fsmp"] = FSMPMessageHandler(
        connection_manager, 
        message_processors["fsmp"], 
        translator
    )
    
    handlers["fvp"] = FVPMessageHandler(
        connection_manager, 
        message_processors["fvp"], 
        translator
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