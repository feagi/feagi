"""
ZeroMQ Sensory Stream Implementation for FEAGI API

This module implements a specialized streaming pattern for sensory data.
It provides:
- One-directional flow from agents to FEAGI
- Efficient binary serialization for high-performance data exchange
- Genome-dependent state management (standby when no genome loaded)

Performance Optimization:
- Socket is configured for real-time operation with minimal latency
- Messages are treated as ephemeral - no queueing is performed
- High water marks (HWM) are set to minimal values to prevent buffer buildup
- Non-blocking operations ensure system responsiveness

This approach ensures that sensory data, which is time-sensitive, is handled
with priority and never queued if it cannot be processed immediately, preventing
the system from wasting resources on outdated information.
"""

import asyncio
from feagi.utils.logger import setup_logger
logger = setup_logger(__name__)
import time
from typing import Dict, Any, Optional, Callable

import zmq
import zmq.asyncio

from ...core.service import CoreApiService
from ...utils.rate_limit import RateLimiter
from feagi.core.state_manager import GenomeState


class SensoryStream:
    """
    ZeroMQ Sensory Stream implementation.
    
    This implementation uses a PULL socket for receiving sensory data (agents → FEAGI).
    The stream automatically adjusts to the genome availability state:
    - When no genome is loaded, it operates in standby mode, rejecting data processing
    - When a genome is loaded, it transitions to active mode
    """
    
    def __init__(
        self, 
        core_api: CoreApiService,
        host: str = "*", 
        port: int = 5558,
        context: Optional[zmq.asyncio.Context] = None
    ):
        """
        Initialize the Sensory Stream.
        
        Args:
            core_api: The CoreApiService instance to delegate calls to
            host: Host address to bind to
            port: Port for receiving sensory data
            context: Optional existing ZMQ context to use
        """
        self.core_api = core_api
        self.host = host
        self.port = port
        self.running = False
        self.context = context or zmq.asyncio.Context.instance()
        
        # State tracking
        self._active_mode = False  # True when genome is loaded and ready
        
        # PULL socket for receiving sensory data (agents → FEAGI)
        self.socket = self._setup_socket()
        
        # Rate limiter for throttling high-frequency data
        self.rate_limiter = RateLimiter()
        
        # Sensory data callback
        self.sensory_callback = None
        
        # Task for handling incoming data
        self.data_handler_task = None
        
        # Register for genome state change notifications
        if hasattr(core_api, 'register_genome_change_listener'):
            core_api.register_genome_change_listener(self._on_genome_state_change)
        
        # Initialize state based on current genome availability
        self._update_active_mode()

    def _setup_socket(self):
        """
        Set up the sensory (PULL) socket.
        
        Returns:
            Configured ZMQ socket
        """
        socket = self.context.socket(zmq.PULL)
        
        # Configure socket for real-time data with no queuing
        socket.setsockopt(zmq.RCVHWM, 1)  # Minimal receive queue
        socket.setsockopt(zmq.LINGER, 0)  # Don't wait for messages to be sent when closing
        
        bind_addr = f"tcp://{self.host}:{self.port}"
        logger.info(f"Binding sensory PULL socket to {bind_addr}")
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
                logger.info("SensoryStream entering ACTIVE mode (genome loaded)")
            else:
                logger.info("SensoryStream entering STANDBY mode (no genome loaded)")

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
                    logger.info("SensoryStream entering ACTIVE mode (genome loaded)")
            else:
                # Any other state means genome not fully loaded
                self._active_mode = False 
                if self.running:
                    logger.info("SensoryStream entering STANDBY mode (no genome loaded)")
        except Exception as e:
            logger.error(f"Error handling genome state change: {e}")
            # Default to standby mode on error
            self._active_mode = False

    async def start(self) -> None:
        """Start the sensory stream server."""
        if self.running:
            return
            
        logger.info(f"Starting Sensory Stream server on {self.host}:{self.port}")
        self.running = True
        
        # Start the data handler task
        self.data_handler_task = asyncio.create_task(self._handle_sensory_data())
        
        logger.info("Sensory Stream server started")

    async def stop(self) -> None:
        """Stop the sensory stream server."""
        if not self.running:
            return
            
        logger.info("Stopping Sensory Stream server")
        self.running = False
        
        # Cancel the data handler task
        if self.data_handler_task:
            self.data_handler_task.cancel()
            try:
                await self.data_handler_task
            except asyncio.CancelledError:
                pass
            self.data_handler_task = None
        
        # Close the socket
        if self.socket:
            self.socket.close()
            self.socket = None
            
        logger.info("Sensory Stream server stopped")

    async def _handle_sensory_data(self) -> None:
        """Handle incoming sensory data."""
        if not self.socket:
            logger.error("Sensory socket not initialized")
            return
            
        try:
            while self.running:
                try:
                    # Wait for incoming data with a timeout
                    try:
                        # Use recv_multipart to handle frames properly
                        frames = await asyncio.wait_for(
                            self.socket.recv_multipart(), 
                            timeout=0.5
                        )
                        
                        # Extract channel ID and data
                        if len(frames) >= 2:
                            channel_id = frames[0].decode('utf-8')
                            data = frames[1]
                            
                            # Process the sensory data
                            await self._process_sensory_data(channel_id, data)
                        else:
                            logger.warning(f"Received invalid sensory data format: {frames}")
                            
                    except asyncio.TimeoutError:
                        # No data received within timeout, just continue
                        continue
                        
                except asyncio.CancelledError:
                    # Task was cancelled, exit gracefully
                    break
                    
                except Exception as e:
                    logger.error(f"Error processing sensory data: {e}")
                    await asyncio.sleep(0.1)  # Throttle on error
                    
        except asyncio.CancelledError:
            # Task was cancelled, exit gracefully
            pass
        except Exception as e:
            logger.error(f"Fatal error in sensory data handler: {e}")

    async def _process_sensory_data(self, channel_id: str, data: bytes) -> None:
        """
        Process incoming sensory data.
        
        Args:
            channel_id: Sensory channel ID
            data: Binary sensory data
        """
        # Skip processing if in standby mode
        if not self._active_mode:
            logger.debug(f"Ignoring sensory data (channel {channel_id}) in standby mode")
            return
            
        # Call the callback if registered
        if self.sensory_callback:
            await self.sensory_callback(channel_id, data)
        else:
            logger.debug(f"Received sensory data on channel {channel_id} but no callback registered")

    def register_sensory_callback(self, callback: Callable) -> None:
        """
        Register a callback for processing incoming sensory data.
        
        Args:
            callback: Async function to call when sensory data is received
                     (parameters: channel_id, data)
        """
        self.sensory_callback = callback 