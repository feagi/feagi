"""
ZeroMQ Sensorimotor Stream Implementation for FEAGI API

This module implements specialized streaming patterns for sensorimotor data.
It provides:
- Separate optimized streams for sensory and motor data
- Efficient binary serialization for high-performance data exchange
- Prioritized motor data streaming
- Genome-dependent state management (standby when no genome loaded)

Performance Optimization:
- All sockets are configured for real-time operation with minimal latency
- Messages are treated as ephemeral - no queueing is performed
- ZMQ_CONFLATE ensures only the latest message is kept, preventing stale data processing
- High water marks (HWM) are set to minimal values to prevent buffer buildup
- Non-blocking operations ensure system responsiveness

This approach ensures that sensorimotor data, which is time-sensitive, is handled
with priority and never queued if it cannot be processed immediately, preventing
the system from wasting resources on outdated information.
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
from feagi.core.state_manager import GenomeState


class SensorimotorStream:
    """
    ZeroMQ Sensorimotor Streams implementation with separate streams for sensory and motor data.
    
    This implementation uses separate streams for sensory input and motor output:
    - PULL socket for receiving sensory data (agents → FEAGI)
    - PUB socket for broadcasting motor data (FEAGI → agents)
    
    This design ensures that high-volume sensory data never blocks or delays critical motor commands.
    
    The stream automatically adjusts to the genome availability state:
    - When no genome is loaded, it operates in standby mode, rejecting data processing
    - When a genome is loaded, it transitions to active mode
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
        
        # State tracking
        self._active_mode = False  # True when genome is loaded and ready
        
        # PULL socket for receiving sensory data (agents → FEAGI)
        self.sensory_socket = self._setup_sensory_socket()
        
        # PUB socket for broadcasting motor data (FEAGI → agents)
        self.motor_socket = self._setup_motor_socket()
        
        # For compatibility with existing code
        self.socket = self.sensory_socket
        
        # Connected clients and their configurations
        self.clients = {}
        
        # Rate limiter for throttling high-frequency data
        self.rate_limiter = RateLimiter()
        
        # Periodic task references
        self.periodic_tasks = {}
        
        # Register for genome state change notifications
        if hasattr(core_api, 'register_genome_change_listener'):
            core_api.register_genome_change_listener(self._on_genome_state_change)
        
        # Initialize state based on current genome availability
        self._update_active_mode()

    def _setup_sensory_socket(self):
        """
        Set up the sensory (PULL) socket.
        """
        socket = self.context.socket(zmq.PULL)
        
        # Configure socket for real-time data with no queuing
        socket.setsockopt(zmq.RCVHWM, 1)  # Minimal receive queue
        socket.setsockopt(zmq.LINGER, 0)  # Don't wait for messages to be sent when closing
        
        bind_addr = f"tcp://{self.host}:{self.sensory_port}"
        logger.info(f"Binding sensory PULL socket to {bind_addr}")
        socket.bind(bind_addr)
        return socket

    def _setup_motor_socket(self):
        """
        Set up the motor (PUB) socket.
        """
        socket = self.context.socket(zmq.PUB)
        
        # Configure socket for real-time data with no queuing
        socket.setsockopt(zmq.SNDHWM, 1)  # Minimal send queue
        socket.setsockopt(zmq.CONFLATE, 1)  # Only keep most recent message
        socket.setsockopt(zmq.LINGER, 0)  # Don't wait for messages to be sent when closing
        
        bind_addr = f"tcp://{self.host}:{self.motor_port}"
        logger.info(f"Binding motor PUB socket to {bind_addr}")
        socket.bind(bind_addr)
        return socket
        
    def _update_active_mode(self):
        """Update active mode based on genome availability."""
        old_mode = self._active_mode
        
        # Safely check genome loaded state with defensive programming
        try:
            self._active_mode = self.core_api.genome_is_loaded() if self.core_api else False
        except Exception as e:
            # If there's any error accessing genome state, default to standby mode
            logger.warning(f"Error checking genome state: {e}, defaulting to standby mode")
            self._active_mode = False
        
        if old_mode != self._active_mode:
            if self._active_mode:
                logger.info("SensorimotorStream entering ACTIVE mode (genome loaded)")
                # Only broadcast if streams are running
                if self.running:
                    asyncio.create_task(self._broadcast_state_change("active"))
            else:
                logger.info("SensorimotorStream entering STANDBY mode (no genome loaded)")
                # Only broadcast if streams are running
                if self.running:
                    asyncio.create_task(self._broadcast_state_change("standby"))
    
    async def _broadcast_state_change(self, state: str):
        """Broadcast state change to all connected clients.
        
        Args:
            state: New state ("active" or "standby")
        """
        try:
            # Send on motor channel as system message
            message = f"FEAGI_STATE_CHANGE:{state}".encode()
            await self.motor_socket.send_multipart([
                b"system",  # Topic
                message     # Message
            ])
            logger.debug(f"Broadcasted state change to {state}")
        except Exception as e:
            logger.error(f"Error broadcasting state change: {e}")

    def _on_genome_state_change(self, old_state, new_state):
        """Handle genome state changes.
        
        Args:
            old_state: Previous genome state
            new_state: New genome state
        """
        logger.debug(f"Received genome state change: {old_state} → {new_state}")
        
        try:
            # Only care about LOADED vs other states
            if new_state == GenomeState.LOADED:
                # Transition to active mode when genome is loaded
                self._active_mode = True
                if self.running:
                    logger.info("SensorimotorStream entering ACTIVE mode (genome loaded)")
                    asyncio.create_task(self._broadcast_state_change("active"))
            else:
                # Any other state means genome not fully loaded
                self._active_mode = False 
                if self.running:
                    logger.info("SensorimotorStream entering STANDBY mode (genome not loaded)")
                    asyncio.create_task(self._broadcast_state_change("standby"))
        except Exception as e:
            logger.error(f"Error handling genome state change: {e}")
            # Default to standby mode on error
            self._active_mode = False
            if self.running:
                asyncio.create_task(self._broadcast_state_change("standby"))

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
        
        # Determine initial state (active or standby)
        self._update_active_mode()
        
        # Broadcast initial state to clients
        if self._active_mode:
            await self._broadcast_state_change("active")
        else:
            await self._broadcast_state_change("standby")

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
                
                # Handle system messages differently
                if channel_id == "system":
                    await self._handle_system_message(data)
                    continue
                
                # Check if system is in active mode before processing data
                if not self._active_mode:
                    logger.warning(f"Ignoring sensory data on channel {channel_id}: system in standby mode (no genome loaded)")
                    # Could send a standby notification here if needed
                    continue
                
                logger.debug(f"Received sensory data on channel {channel_id}: {len(data)} bytes")
                
                # Process the sensory data
                await self._process_sensory_data(channel_id, data)
            
            except asyncio.CancelledError:
                logger.debug("Sensory data handler cancelled")
                break
            except Exception as e:
                logger.error(f"Error handling sensory data: {e}")
                await asyncio.sleep(0.1)  # Avoid tight loop on errors
                
    async def _handle_system_message(self, data: bytes) -> None:
        """Handle system messages from clients.
        
        Args:
            data: Message data
        """
        try:
            message = data.decode()
            if message.startswith("STATUS_CHECK"):
                # Client is checking status - respond with current state
                state = "active" if self._active_mode else "standby"
                await self.motor_socket.send_multipart([
                    b"system",
                    f"FEAGI_STATE:{state}".encode()
                ])
                logger.debug(f"Responded to status check with state: {state}")
        except Exception as e:
            logger.error(f"Error handling system message: {e}")

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
            
        # Check if system is in active mode before sending data
        if not self._active_mode and channel_id != "system":
            logger.warning(f"Not sending motor data on channel {channel_id}: system in standby mode (no genome loaded)")
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
    ZeroMQ-based client for sensorimotor data streaming.
    
    This class provides separate optimized streams for:
    - Sending sensory data to FEAGI (uses PUSH socket)
    - Receiving motor data from FEAGI (uses SUB socket)
    
    Both streams use high-performance binary serialization and are configured
    for real-time operation with minimal latency.
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
        self.sensory_socket = self._setup_sensory_socket()
        
        # SUB socket for receiving motor data (FEAGI → agent)
        self.motor_socket = self._setup_motor_socket()
        
        # Client ID
        self.client_id = str(uuid.uuid4())
        
        # Callback registry for motor data
        self.motor_callbacks = {}
        
        # Running flag
        self.running = False
        
        # Background tasks
        self.tasks = []

    def _setup_sensory_socket(self):
        """
        Set up the sensory (PUSH) socket.
        """
        socket = self.context.socket(zmq.PUSH)
        
        # Configure socket for real-time data with no queuing
        socket.setsockopt(zmq.SNDHWM, 1)  # Minimal send queue
        socket.setsockopt(zmq.LINGER, 0)  # Don't wait for messages to be sent when closing
        
        connect_addr = f"tcp://{self.host}:{self.sensory_port}"
        logger.info(f"Connecting sensory PUSH socket to {connect_addr}")
        socket.connect(connect_addr)
        return socket

    def _setup_motor_socket(self):
        """
        Set up the motor (SUB) socket.
        """
        socket = self.context.socket(zmq.SUB)
        
        # Configure socket for real-time data with no queuing
        socket.setsockopt(zmq.RCVHWM, 1)  # Minimal receive queue
        socket.setsockopt(zmq.CONFLATE, 1)  # Only keep most recent message
        socket.setsockopt(zmq.LINGER, 0)  # Don't wait for messages to be sent when closing
        
        # Subscribe to all messages (empty string = no filtering)
        socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
        connect_addr = f"tcp://{self.host}:{self.motor_port}"
        logger.info(f"Connecting motor SUB socket to {connect_addr}")
        socket.connect(connect_addr)
        return socket

    async def connect(self) -> bool:
        """
        Connect to FEAGI.
        
        Returns:
            True if connected successfully, False otherwise
        """
        try:
            # Create and connect sockets
            self.sensory_socket = self._setup_sensory_socket()
            self.motor_socket = self._setup_motor_socket()
            
            self.connected = True
            logger.info(f"Connected to FEAGI on {self.host}:{self.sensory_port}/{self.motor_port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to FEAGI: {e}")
            self.connected = False
            return False

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