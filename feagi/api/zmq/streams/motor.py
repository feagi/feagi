"""
ZeroMQ Motor Stream Implementation for FEAGI API

This module implements a specialized streaming pattern for motor data.
It provides:
- One-directional flow from FEAGI to agents
- Efficient binary serialization for high-performance data exchange
- Genome-dependent state management (standby when no genome loaded)

Performance Optimization:
- Socket is configured for real-time operation with minimal latency
- Messages are treated as ephemeral - no queueing is performed
- ZMQ_CONFLATE ensures only the latest message is kept, preventing stale data processing
- High water marks (HWM) are set to minimal values to prevent buffer buildup
- Non-blocking operations ensure system responsiveness

This approach ensures that motor data, which is time-sensitive, is handled
with priority and never queued if it cannot be processed immediately, preventing
the system from wasting resources on outdated information.
"""

import asyncio
import json
import logging
import time
import threading
from typing import Dict, Any, Optional, List, Union, Callable
import uuid

import zmq
import zmq.asyncio

from feagi.utils.logger import setup_logger

# Import the unified CoreAPIService  
from ...core.services.core_api_service import CoreAPIService
from ...utils.rate_limit import RateLimiter
from feagi.core.state_manager import GenomeState

logger = setup_logger()

class MotorStream:
    """
    ZeroMQ Motor Stream implementation.
    
    This implementation uses a PUB socket for sending motor data (FEAGI → agents).
    The stream automatically adjusts to the genome availability state:
    - When no genome is loaded, it operates in standby mode
    - When a genome is loaded, it transitions to active mode
    """
    
    def __init__(
        self, 
        core_api: CoreAPIService,
        host: str = "*", 
        port: int = 5564,
        context: Optional[zmq.asyncio.Context] = None
    ):
        """
        Initialize the Motor Stream.
        
        Args:
            core_api: The CoreAPIService instance to delegate calls to
            host: Host address to bind to
            port: Port for sending motor data
            context: Optional existing ZMQ context to use
        """
        self.core_api = core_api
        self.host = host
        self.port = port
        self.running = False
        self.context = context or zmq.asyncio.Context.instance()
        
        # State tracking
        self._active_mode = False  # True when genome is loaded and ready
        
        # PUB socket for sending motor data (FEAGI → agents)
        self.socket = self._setup_socket()
        
        # Rate limiter for throttling high-frequency data
        self.rate_limiter = RateLimiter()
        
        # Register for genome state change notifications
        if hasattr(core_api, 'register_genome_change_listener'):
            core_api.register_genome_change_listener(self._on_genome_state_change)
        
        # Initialize state based on current genome availability
        self._update_active_mode()

    def _setup_socket(self):
        """
        Set up the motor (PUB) socket.
        
        Returns:
            Configured ZMQ socket
        """
        socket = self.context.socket(zmq.PUB)
        
        # Configure socket for real-time data with no queuing
        socket.setsockopt(zmq.SNDHWM, 1)  # Minimal send queue
        socket.setsockopt(zmq.CONFLATE, 1)  # Only keep most recent message
        socket.setsockopt(zmq.LINGER, 0)  # Don't wait for messages to be sent when closing
        
        bind_addr = f"tcp://{self.host}:{self.port}"
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
                logger.info("MotorStream entering ACTIVE mode (genome loaded)")
            else:
                logger.info("MotorStream entering STANDBY mode (no genome loaded)")

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
                    logger.info("MotorStream entering ACTIVE mode (genome loaded)")
            else:
                # Any other state means genome not fully loaded
                self._active_mode = False 
                if self.running:
                    logger.info("MotorStream entering STANDBY mode (no genome loaded)")
        except Exception as e:
            logger.error(f"Error handling genome state change: {e}")
            # Default to standby mode on error
            self._active_mode = False

    async def start(self) -> None:
        """Start the motor stream server."""
        if self.running:
            return
            
        logger.info(f"Starting Motor Stream server on {self.host}:{self.port}")
        self.running = True
        logger.info("Motor Stream server started")

    async def stop(self) -> None:
        """Stop the motor stream server."""
        if not self.running:
            return
            
        logger.info("Stopping Motor Stream server")
        self.running = False
        
        # Close the socket
        if self.socket:
            self.socket.close()
            self.socket = None
            
        logger.info("Motor Stream server stopped")

    async def send_motor_data(self, channel_id: str, data: bytes) -> None:
        """
        Send motor data to agents.
        
        Args:
            channel_id: Motor channel ID
            data: Binary motor data
        """
        if not self.running or not self.socket:
            logger.warning("Cannot send motor data: server not running")
            return
            
        # Skip if in standby mode
        if not self._active_mode:
            logger.debug(f"Suppressing motor output (channel {channel_id}) in standby mode")
            return
            
        try:
            # Apply rate limiting if needed
            if not self.rate_limiter.check_rate(f"motor_{channel_id}", 0.01):  # Max 100Hz per channel
                logger.debug(f"Rate limiting motor data on channel {channel_id}")
                return
                
            # Send multipart message with topic (channel_id) and data
            await self.socket.send_multipart([
                channel_id.encode('utf-8'),  # Topic (channel ID)
                data                         # Binary data
            ])
            
            logger.debug(f"Sent {len(data)} bytes of motor data on channel {channel_id}")
            
        except Exception as e:
            logger.error(f"Error sending motor data on channel {channel_id}: {e}")
            
    async def broadcast_system_message(self, message: str) -> None:
        """
        Broadcast a system message to all connected agents.
        
        Args:
            message: System message to broadcast
        """
        try:
            # Send on system channel
            await self.socket.send_multipart([
                b"system",                # System channel
                message.encode('utf-8')   # Message
            ])
            
            logger.debug(f"Broadcast system message: {message}")
            
        except Exception as e:
            logger.error(f"Error broadcasting system message: {e}") 