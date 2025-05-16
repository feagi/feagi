"""
ZeroMQ Sensorimotor Stream Implementation for FEAGI API

This module implements specialized streaming patterns for sensorimotor data.
It provides:
- Separate optimized streams for sensory and motor data
- Efficient binary serialization for high-performance data exchange
- Prioritized motor data streaming
"""

import asyncio
from feagi.utils.logger import setup_logger
logger = setup_logger(__name__)
import time
import uuid
import struct
from typing import Dict, Any, List, Optional, Set, Tuple, Union, Callable

import zmq
import zmq.asyncio
import numpy as np

from ...core.service import CoreApiService
from ...utils.rate_limit import RateLimiter


class SensorimotorStream:
    """
    ZeroMQ Sensorimotor Streams implementation with separate streams for sensory and motor data.
    
    This implementation uses separate streams for sensory input and motor output:
    - PULL socket for receiving sensory data (agents → FEAGI)
    - PUB socket for broadcasting motor data (FEAGI → agents)
    
    This design ensures that high-volume sensory data never blocks or delays critical motor commands.
    """
    
    def __init__(
        self, 
        core_api: CoreApiService,
        host: str = "*", 
        sensory_port: int = 5558,
        motor_port: int = 5559,
        context: Optional[zmq.asyncio.Context] = None
    ):
        """
        Initialize the Sensorimotor Streams.
        
        Args:
            core_api: The CoreApiService instance to delegate calls to
            host: Host address to bind to
            sensory_port: Port for receiving sensory data
            motor_port: Port for broadcasting motor data
            context: Optional existing ZMQ context to use
        """
        self.core_api = core_api
        self.host = host
        self.sensory_port = sensory_port
        self.motor_port = motor_port
        self.running = False
        self.context = context or zmq.asyncio.Context.instance()
        
        # PULL socket for receiving sensory data (agents → FEAGI)
        self.sensory_socket = self.context.socket(zmq.PULL)
        self.sensory_socket.bind(f"tcp://{host}:{sensory_port}")
        
        # PUB socket for broadcasting motor data (FEAGI → agents)
        self.motor_socket = self.context.socket(zmq.PUB)
        self.motor_socket.bind(f"tcp://{host}:{motor_port}")
        
        # For compatibility with existing code
        self.socket = self.sensory_socket
        
        # Connected clients and their configurations
        self.clients = {}
        
        # Rate limiter for throttling high-frequency data
        self.rate_limiter = RateLimiter()
        
        # Periodic task references
        self.periodic_tasks = {}

    async def start(self) -> None:
        """Start the sensorimotor stream server."""
        logger.info(f"Starting Sensorimotor Streams server:")
        logger.info(f"  - Sensory stream (PULL) on {self.host}:{self.sensory_port}")
        logger.info(f"  - Motor stream (PUB) on {self.host}:{self.motor_port}")
        
        self.running = True
        
        # Store the current event loop for this method
        self._event_loop = asyncio.get_event_loop()
        
        # Start sensory data handler
        self.periodic_tasks["sensory_handler"] = self._event_loop.create_task(
            self._handle_sensory_data()
        )

    async def stop(self) -> None:
        """Stop the sensorimotor stream server."""
        logger.info("Stopping Sensorimotor Streams server")
        self.running = False
        
        # Cancel all periodic tasks
        for task_name, task in self.periodic_tasks.items():
            if not task.done():
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    logger.debug(f"Cancelled periodic task: {task_name}")
        
        # Close the sockets
        self.sensory_socket.close()
        self.motor_socket.close()

    async def _handle_sensory_data(self) -> None:
        """Main loop for handling incoming sensory data."""
        while self.running:
            try:
                # Wait for sensory data
                sensory_data = await self.sensory_socket.recv_multipart()
                
                if len(sensory_data) < 2:
                    logger.error(f"Received malformed sensory data: {sensory_data}")
                    continue
                
                # First frame is the channel ID, second is the actual data
                channel_id = sensory_data[0].decode()
                data = sensory_data[1]
                
                logger.debug(f"Received sensory data on channel {channel_id}: {len(data)} bytes")
                
                # Process the sensory data
                await self._process_sensory_data(channel_id, data)
            
            except asyncio.CancelledError:
                logger.debug("Sensory data handler cancelled")
                break
            except Exception as e:
                logger.error(f"Error handling sensory data: {e}")
                await asyncio.sleep(0.1)  # Avoid tight loop on errors

    async def _process_sensory_data(self, channel_id: str, data: bytes) -> None:
        """
        Process incoming sensory data.
        
        Args:
            channel_id: Channel identifier
            data: Binary sensory data
        """
        try:
            # Pass the binary data directly to the core API for processing
            # The core API will pass it to fcl_manager to convert directly to roaring bitmap
            await self.core_api.process_sensory_data(channel_id, data)
        except Exception as e:
            logger.error(f"Error processing sensory data: {e}")

    async def send_motor_data(self, channel_id: str, data: bytes) -> None:
        """
        Send motor data to subscribed agents.
        
        Args:
            channel_id: Channel identifier (used as topic)
            data: Binary motor data
        """
        if not self.running:
            logger.warning("Cannot send motor data: streams not running")
            return
            
        try:
            # Send multipart message with channel as topic
            await self.motor_socket.send_multipart([
                channel_id.encode(),  # Topic (channel ID)
                data                  # Binary data
            ])
            
            logger.debug(f"Sent motor data on channel {channel_id}: {len(data)} bytes")
        except Exception as e:
            logger.error(f"Error sending motor data: {e}")


class SensorimotorClient:
    """
    ZeroMQ Sensorimotor Client implementation with separate streams.
    
    This client uses:
    - PUSH socket for sending sensory data to FEAGI (agent → FEAGI)
    - SUB socket for receiving motor data from FEAGI (FEAGI → agent)
    
    This separation ensures high-priority motor commands are never delayed by
    sensory data congestion.
    """
    
    def __init__(
        self, 
        host: str = "localhost", 
        sensory_port: int = 5558,
        motor_port: int = 5559,
        context: Optional[zmq.asyncio.Context] = None,
        timeout: float = 5.0
    ):
        """
        Initialize a new Sensorimotor Client.
        
        Args:
            host: Server host to connect to
            sensory_port: Port for sending sensory data
            motor_port: Port for receiving motor data
            context: Optional existing ZMQ context to use
            timeout: Request timeout in seconds
        """
        self.host = host
        self.sensory_port = sensory_port
        self.motor_port = motor_port
        self.timeout = timeout
        self.context = context or zmq.asyncio.Context.instance()
        
        # PUSH socket for sending sensory data (agent → FEAGI)
        self.sensory_socket = self.context.socket(zmq.PUSH)
        self.sensory_socket.connect(f"tcp://{host}:{sensory_port}")
        
        # SUB socket for receiving motor data (FEAGI → agent)
        self.motor_socket = self.context.socket(zmq.SUB)
        self.motor_socket.connect(f"tcp://{host}:{motor_port}")
        
        # Default to subscribing to all channels
        self.motor_socket.setsockopt(zmq.SUBSCRIBE, b"")
        
        # Client ID
        self.client_id = str(uuid.uuid4())
        
        # Callback registry for motor data
        self.motor_callbacks = {}
        
        # Running flag
        self.running = False
        
        # Background tasks
        self.tasks = []

    async def start(self) -> None:
        """Start the sensorimotor client."""
        logger.info(f"Starting Sensorimotor Client to {self.host}:")
        logger.info(f"  - Sensory connection (PUSH): {self.host}:{self.sensory_port}")
        logger.info(f"  - Motor connection (SUB): {self.host}:{self.motor_port}")
        
        self.running = True
        
        # Start motor data receiver
        self.tasks.append(asyncio.create_task(self._motor_data_receiver()))

    async def stop(self) -> None:
        """Stop the sensorimotor client."""
        logger.info("Stopping Sensorimotor Client")
        self.running = False
        
        # Cancel all tasks
        for task in self.tasks:
            if not task.done():
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
        
        # Close sockets
        self.sensory_socket.close()
        self.motor_socket.close()

    async def _motor_data_receiver(self) -> None:
        """Background task to receive motor data."""
        logger.info("Motor data receiver started")
        
        while self.running:
            try:
                # Receive motor data with short timeout to allow clean shutdown
                try:
                    multipart = await asyncio.wait_for(
                        self.motor_socket.recv_multipart(),
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

    async def send_sensory_data(self, channel_id: str, data: bytes) -> bool:
        """
        Send sensory data to FEAGI.
        
        Args:
            channel_id: Sensory channel ID
            data: Sensory data as binary bytes
            
        Returns:
            True if sent successfully, False otherwise
        """
        if not self.running:
            logger.warning("Cannot send sensory data: client not running")
            return False
        
        if not isinstance(data, bytes):
            logger.error("Sensory data must be in binary bytes format")
            return False
        
        try:
            # Send as multipart message with channel ID and binary data
            await self.sensory_socket.send_multipart([
                channel_id.encode(),
                data
            ])
            
            logger.debug(f"Sent sensory data on channel {channel_id}: {len(data)} bytes")
            return True
            
        except Exception as e:
            logger.error(f"Error sending sensory data: {e}")
            return False

    def subscribe_to_motor_channel(self, channel_id: str) -> None:
        """
        Subscribe to a specific motor channel.
        
        Args:
            channel_id: Motor channel ID to subscribe to
        """
        # SUB socket subscription filter
        self.motor_socket.setsockopt(zmq.SUBSCRIBE, channel_id.encode())
        logger.info(f"Subscribed to motor channel: {channel_id}")

    def register_motor_callback(self, callback: Callable, channel_id: str = "*") -> None:
        """
        Register a callback for motor data.
        
        Args:
            callback: Function to call when motor data is received
            channel_id: Channel ID to register for, or "*" for all channels
        """
        self.motor_callbacks[channel_id] = callback
        logger.info(f"Registered motor callback for channel: {channel_id}") 