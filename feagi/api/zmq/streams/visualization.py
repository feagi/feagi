"""
ZeroMQ Visualization Stream Implementation for FEAGI API

This module implements a specialized streaming pattern for visualization data.
It provides:
- One-directional flow from FEAGI to agents for neuron activity data
- Efficient binary serialization for high-performance data exchange
- Level-of-detail mechanisms for performance optimization
- Genome-dependent state management (standby when no genome loaded)

Performance Optimization:
- Socket is configured for real-time operation with minimal latency
- Messages are treated as ephemeral - no queueing is performed
- ZMQ_CONFLATE ensures only the latest message is kept, preventing stale data processing
- High water marks (HWM) are set to minimal values to prevent buffer buildup
- Non-blocking operations ensure system responsiveness
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


class VisualizationStream:
    """
    ZeroMQ Visualization Stream implementation.
    
    This implementation uses a PUB socket for sending neural activity data (FEAGI → agents).
    
    The stream automatically adjusts to the genome availability state:
    - When no genome is loaded, it operates in standby mode, sending status updates but no data
    - When a genome is loaded, it transitions to active mode with full functionality
    """
    
    def __init__(
        self, 
        core_api: CoreApiService,
        host: str = "*", 
        port: int = 5560,
        context: Optional[zmq.asyncio.Context] = None,
        fcl_sampler: Optional[Any] = None,
        fcl_sampler_queue: Optional[Any] = None
    ):
        """
        Initialize a new Visualization Stream.
        
        Args:
            core_api: The CoreApiService instance to delegate calls to
            host: Host address to bind to
            port: Port for visualization data
            context: Optional existing ZMQ context to use
            fcl_sampler: Optional FCL sampler instance for visualization data
            fcl_sampler_queue: Optional queue for FCL data from the sampler
        """
        self.core_api = core_api
        self.host = host
        self.port = port
        self.running = False
        self.context = context or zmq.asyncio.Context.instance()
        
        # State tracking
        self._active_mode = False  # True when genome is loaded and ready
        
        # Socket for visualization data
        self.socket = self._setup_socket()
        
        # Rate limiter for throttling high-frequency data
        self.rate_limiter = RateLimiter()
        
        # FCL Sampler integration
        self.fcl_sampler = fcl_sampler
        self.fcl_sampler_queue = fcl_sampler_queue
        
        # Tasks
        self.tasks = []
        
        # Register for genome state change notifications
        if hasattr(core_api, 'register_genome_change_listener'):
            core_api.register_genome_change_listener(self._on_genome_state_change)
        
        # Initialize state based on current genome availability
        self._update_active_mode()

    def _setup_socket(self) -> zmq.asyncio.Socket:
        """
        Set up a visualization socket with real-time optimization.
        
        Returns:
            Configured ZMQ socket
        """
        socket = self.context.socket(zmq.PUB)
        
        # Configure for real-time with no queuing
        socket.setsockopt(zmq.SNDHWM, 1)  # Minimal send queue
        socket.setsockopt(zmq.CONFLATE, 1)  # Only keep most recent message
        socket.setsockopt(zmq.LINGER, 0)  # Don't wait when closing
        
        bind_addr = f"tcp://{self.host}:{self.port}"
        logger.info(f"Binding visualization PUB socket to {bind_addr}")
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
                logger.info("VisualizationStream entering ACTIVE mode (genome loaded)")
            else:
                logger.info("VisualizationStream entering STANDBY mode (no genome loaded)")

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
                    logger.info("VisualizationStream entering ACTIVE mode (genome loaded)")
                    asyncio.create_task(self._broadcast_system_message("FEAGI_STATE_CHANGE:active"))
            else:
                # Any other state means genome not fully loaded
                self._active_mode = False 
                if self.running:
                    logger.info("VisualizationStream entering STANDBY mode (no genome loaded)")
                    asyncio.create_task(self._broadcast_system_message("FEAGI_STATE_CHANGE:standby"))
        except Exception as e:
            logger.error(f"Error handling genome state change: {e}")
            # Default to standby mode on error
            self._active_mode = False

    async def _broadcast_system_message(self, message: str):
        """Broadcast a system message to all connected clients.
        
        Args:
            message: System message to broadcast
        """
        try:
            if not self.running or not self.socket:
                return
                
            # Send on system channel
            await self.socket.send_multipart([
                b"system",                # System channel
                message.encode('utf-8')   # Message
            ])
            
            logger.debug(f"Broadcast system message: {message}")
            
        except Exception as e:
            logger.error(f"Error broadcasting system message: {e}")

    async def start(self) -> None:
        """Start the visualization stream server."""
        if self.running:
            return
            
        logger.info(f"Starting Visualization Stream server on {self.host}:{self.port}")
        self.running = True
        
        # Start FCL processing tasks if FCL sampler is available
        if self.fcl_sampler_queue:
            self.tasks.append(asyncio.create_task(self._process_fcl_data()))
            
        logger.info("Visualization Stream server started")

    async def stop(self) -> None:
        """Stop the visualization stream server."""
        if not self.running:
            return
            
        logger.info("Stopping Visualization Stream server")
        self.running = False
        
        # Cancel all tasks
        for task in self.tasks:
            task.cancel()
            
        # Wait for tasks to complete
        if self.tasks:
            await asyncio.gather(*self.tasks, return_exceptions=True)
            self.tasks = []
        
        # Close the socket
        if self.socket:
            self.socket.close()
            self.socket = None
            
        logger.info("Visualization Stream server stopped")

    async def _process_fcl_data(self) -> None:
        """Process FCL data from the sampler queue."""
        if not self.fcl_sampler_queue:
            logger.error("FCL sampler queue not available")
            return
            
        try:
            while self.running:
                try:
                    # Get data from the queue with a timeout
                    try:
                        fcl_data = await asyncio.wait_for(
                            self.fcl_sampler_queue.get(),
                            timeout=0.5
                        )
                        
                        # Process the FCL data
                        await self.send_visualization_data(fcl_data)
                        
                    except asyncio.TimeoutError:
                        # No data available, continue
                        await asyncio.sleep(0.01)
                        continue
                        
                except asyncio.CancelledError:
                    # Task was cancelled, exit gracefully
                    break
                    
                except Exception as e:
                    logger.error(f"Error processing FCL data: {e}")
                    await asyncio.sleep(0.1)  # Throttle on error
                    
        except asyncio.CancelledError:
            # Task was cancelled, exit gracefully
            pass
        except Exception as e:
            logger.error(f"Fatal error in FCL data processor: {e}")

    async def send_visualization_data(self, data: bytes) -> None:
        """
        Send visualization data to clients.
        
        Args:
            data: Binary visualization data
        """
        if not self.running or not self.socket:
            logger.debug("Cannot send visualization data: server not running")
            return
            
        # Skip if in standby mode
        if not self._active_mode:
            logger.debug("Suppressing visualization data in standby mode")
            return
            
        try:
            # Apply rate limiting if needed
            if not self.rate_limiter.check_rate("visualization", 0.05):  # Max 20Hz
                logger.debug("Rate limiting visualization data")
                return
                
            # Send multipart message with topic and data
            await self.socket.send_multipart([
                b"activity",  # Topic
                data          # Binary data
            ])
            
            logger.debug(f"Sent {len(data)} bytes of visualization data")
            
        except Exception as e:
            logger.error(f"Error sending visualization data: {e}")
            
    async def broadcast_update(self, data_type: str, data: bytes) -> None:
        """
        Broadcast an update to all connected agents.
        
        Args:
            data_type: Type of data ("activity", "structure", "system")
            data: Binary data
        """
        if not self.running or not self.socket:
            logger.debug(f"Cannot broadcast {data_type} update: server not running")
            return
            
        # Skip if in standby mode (except for system messages)
        if not self._active_mode and data_type != "system":
            logger.debug(f"Suppressing {data_type} update in standby mode")
            return
            
        try:
            # Apply rate limiting if needed
            if not self.rate_limiter.check_rate(f"broadcast_{data_type}", 0.05):  # Max 20Hz
                logger.debug(f"Rate limiting {data_type} broadcast")
                return
                
            # Send multipart message with topic and data
            await self.socket.send_multipart([
                data_type.encode('utf-8'),  # Topic
                data                         # Binary data
            ])
            
            logger.debug(f"Broadcast {len(data)} bytes of {data_type} data")
            
        except Exception as e:
            logger.error(f"Error broadcasting {data_type} data: {e}") 