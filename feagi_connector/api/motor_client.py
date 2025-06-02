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
FEAGI Motor Client

Client for receiving motor data from FEAGI using ZMQ PUB/SUB pattern.
"""

import json
import logging
import uuid
import asyncio
from typing import Dict, Any, Optional, List, Union, Tuple, Callable, Awaitable

import zmq
import zmq.asyncio
from feagi_bytes import ByteStructureEncoder, ByteStructureDecoder, ByteStructureID

# Import constants
from feagi_connector.utils import NEURON_POTENTIAL_CATEGORICAL_XYZ

# Import default implementation - this should be the Python implementation
# for backward compatibility
from feagi_connector.utils.processing import (
    infer_byte_structure_type_python as infer_byte_structure_type,
    decode_neuron_potential_xyz_python as decode_neuron_potential_xyz
)

# Configure logging
logger = logging.getLogger("feagi_connector.motor")


class FeagiMotorClient:
    """
    Client for receiving motor data from FEAGI using PUB/SUB pattern (port 5564).
    
    This client properly handles binary data reception for motor output,
    using the correct topic subscription for PUB/SUB.
    """
    
    def __init__(
        self, 
        host: str = "127.0.0.1", 
        port: int = 5564, 
        agent_id: Optional[str] = None,
        socket_timeout: int = 1000
    ):
        """
        Initialize the motor client.
        
        Args:
            host: FEAGI hostname or IP
            port: ZMQ PUB/SUB port (default 5564)
            agent_id: Agent ID for topic subscription (default: auto-generated)
            socket_timeout: Socket timeout in milliseconds
        """
        self.host = host
        self.port = port
        self.agent_id = agent_id or f"agent-{uuid.uuid4().hex[:8]}"
        self.timeout = socket_timeout
        self.context = zmq.asyncio.Context.instance()
        self.socket = None
        self.running = False
        
        # Callback registry for motor data
        self.motor_callbacks = {}
        
        # Background tasks
        self.tasks = []
        
    async def connect(self) -> bool:
        """
        Create and connect a socket.
        
        Returns:
            True if connection was successful
        """
        try:
            if self.socket:
                self.socket.close()
                
            self.socket = self.context.socket(zmq.SUB)
            
            # Configure socket for real-time data with no queuing
            self.socket.setsockopt(zmq.RCVHWM, 1)  # Minimal receive queue
            self.socket.setsockopt(zmq.CONFLATE, 1)  # Only keep most recent message
            self.socket.setsockopt(zmq.LINGER, 0)  # Don't wait for messages when closing
            
            # Set a connect timeout
            self.socket.setsockopt(zmq.CONNECT_TIMEOUT, 1000)  # 1 second timeout
            
            # Subscribe to our agent_id as topic
            self.socket.setsockopt_string(zmq.SUBSCRIBE, self.agent_id)
            
            # Also subscribe to system messages
            self.socket.setsockopt_string(zmq.SUBSCRIBE, "system")
            
            logger.debug(f"Connecting to motor server at tcp://{self.host}:{self.port}")
            self.socket.connect(f"tcp://{self.host}:{self.port}")
            logger.debug(f"Connected to motor server")
            return True
            
        except zmq.error.ZMQError as e:
            logger.error(f"ZMQ Error connecting to {self.host}:{self.port}: {e}")
            if self.socket:
                self.socket.close()
                self.socket = None
            return False
            
    async def close(self) -> None:
        """Close the socket and stop all tasks."""
        self.running = False
        
        # Cancel all tasks
        for task in self.tasks:
            if not task.done():
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
        
        if self.socket:
            self.socket.close()
            self.socket = None
    
    def register_motor_callback(self, callback: Callable, channel_id: str = "*") -> None:
        """
        Register a callback for motor data.
        
        Args:
            callback: Function to call when motor data is received
            channel_id: Channel ID to register for, or "*" for all channels
        """
        self.motor_callbacks[channel_id] = callback
        logger.info(f"Registered motor callback for channel: {channel_id}")
    
    async def start(self) -> None:
        """Start the motor client."""
        logger.info(f"Starting Motor Client to {self.host}:{self.port}")
        
        self.running = True
        
        # Start motor data receiver
        self.tasks.append(asyncio.create_task(self._motor_data_receiver()))

    async def _motor_data_receiver(self) -> None:
        """Background task to receive motor data."""
        logger.info("Motor data receiver started")
        
        while self.running:
            try:
                # Receive motor data with short timeout to allow clean shutdown
                try:
                    multipart = await asyncio.wait_for(
                        self.socket.recv_multipart(),
                        timeout=0.5
                    )
                    
                    if len(multipart) < 2:
                        logger.warning(f"Received malformed motor data: {multipart}")
                        continue
                    
                    # First part is the channel/topic, second is the data
                    channel_id = multipart[0].decode()
                    data = multipart[1]
                    
                    # Process motor data
                    await self._process_motor_data(channel_id, data)
                    
                except asyncio.TimeoutError:
                    # Timeout is expected for clean shutdown checks
                    continue
                    
            except asyncio.CancelledError:
                logger.debug("Motor data receiver cancelled")
                break
            except Exception as e:
                logger.error(f"Error receiving motor data: {e}")
                await asyncio.sleep(0.1)  # Avoid tight loop on errors

    async def _process_motor_data(self, channel_id: str, data: bytes) -> None:
        """
        Process received motor data.
        
        Args:
            channel_id: Motor channel ID
            data: Binary motor data
        """
        logger.debug(f"Received motor data on channel {channel_id}: {len(data)} bytes")
        
        # Call registered callback for this channel if any
        if channel_id in self.motor_callbacks:
            try:
                await self.motor_callbacks[channel_id](data)
            except Exception as e:
                logger.error(f"Error in motor callback for channel {channel_id}: {e}")
        
        # Call global motor callback if registered
        if "*" in self.motor_callbacks:
            try:
                await self.motor_callbacks["*"](channel_id, data)
            except Exception as e:
                logger.error(f"Error in global motor callback: {e}")
                
    async def process_motor_data(self, data: bytes) -> Optional[Dict]:
        """
        Process motor data and convert to Python data structure.
        
        Args:
            data: Binary motor data
            
        Returns:
            Decoded motor data or None if decoding failed
        """
        try:
            # First detect what type of binary structure this is
            structure_type = infer_byte_structure_type(data)
            
            # If it's neuron potential data, decode it using the specialized decoder
            if structure_type == ByteStructureID.NEURON_POTENTIAL_CATEGORICAL_XYZ:
                return decode_neuron_potential_xyz(data)
                
            # Otherwise use the general decoder
            decoder = ByteStructureDecoder(data)
            return decoder.decode()
            
        except Exception as e:
            logger.error(f"Error processing motor data: {e}")
            return None
            
    async def receive_motor_data(self, timeout: float = None) -> Optional[Tuple[str, bytes]]:
        """
        Receive motor data directly.
        
        This is a low-level method for direct access to motor data.
        Consider using callbacks with start() instead for most use cases.
        
        Args:
            timeout: Timeout in seconds, or None for no timeout
            
        Returns:
            Tuple of (channel_id, data) or None if no data received
        """
        if not self.socket:
            logger.error("Not connected")
            return None
            
        try:
            # Use timeout if specified
            if timeout is not None:
                try:
                    multipart = await asyncio.wait_for(
                        self.socket.recv_multipart(),
                        timeout=timeout
                    )
                except asyncio.TimeoutError:
                    return None
            else:
                multipart = await self.socket.recv_multipart()
                
            if len(multipart) < 2:
                logger.warning(f"Received malformed motor data: {multipart}")
                return None
                
            # First part is topic/channel, second is data
            channel_id = multipart[0].decode()
            data = multipart[1]
            
            return channel_id, data
            
        except zmq.error.Again:
            # Timed out waiting for data
            return None
        except Exception as e:
            logger.error(f"Error receiving motor data: {e}")
            return None 