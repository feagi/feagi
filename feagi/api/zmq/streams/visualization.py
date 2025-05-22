"""
ZeroMQ Visualization Stream Implementation for FEAGI API

This module implements a specialized streaming pattern for visualization data.
It provides:
- One-directional flow from FEAGI to agents for neuron activity data
- Efficient binary serialization using feagi_bytes
- Genome-dependent state management (standby when no genome loaded)
- Real-time performance optimization with minimal buffering
"""

import asyncio
import time
from typing import Dict, Any, Optional, List
import zmq
import zmq.asyncio

from feagi.utils.logger import setup_logger
from ...core.service import CoreApiService
from ...utils.rate_limit import RateLimiter
from feagi.core.state_manager import GenomeState

logger = setup_logger(__name__)


class VisualizationStream:
    """
    ZeroMQ Visualization Stream implementation.
    
    Uses a PUB socket for sending neural activity data (FEAGI → agents).
    Automatically adjusts to genome availability state.
    """
    
    def __init__(
        self, 
        core_api: CoreApiService,
        host: str = "*", 
        port: int = 5560,
        context: Optional[zmq.asyncio.Context] = None,
        fq_sampler: Optional[Any] = None,
        fq_sampler_queue: Optional[Any] = None
    ):
        """
        Initialize a new Visualization Stream.
        
        Args:
            core_api: The CoreApiService instance
            host: Host address to bind to
            port: Port for visualization data
            context: Optional existing ZMQ context
            fq_sampler: Optional FQ sampler instance
            fq_sampler_queue: Queue for FQ data from the sampler
        """
        self.core_api = core_api
        self.host = host
        self.port = port
        self.running = False
        self.context = context or zmq.asyncio.Context.instance()
        
        # State tracking
        self._active_mode = False  # True when genome is loaded and ready
        
        # Connected clients tracking
        self.client_last_heartbeat = {}  # Mapping of client_id -> last heartbeat time
        self.client_heartbeat_timeout = 30  # Consider clients disconnected after 30s
        
        # Socket for visualization data
        self.socket = self._setup_socket()
        
        # Rate limiter for throttling high-frequency data
        self.rate_limiter = RateLimiter()
        
        # FQ Sampler integration
        self.fq_sampler = fq_sampler
        self.fq_sampler_queue = fq_sampler_queue
        
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
        
        # Configure for real-time with minimal queuing
        socket.setsockopt(zmq.SNDHWM, 1)  # Minimal send queue
        socket.setsockopt(zmq.CONFLATE, 1)  # Only keep most recent message
        socket.setsockopt(zmq.LINGER, 0)  # Don't wait when closing
        
        bind_addr = f"tcp://{self.host}:{self.port}"
        socket.bind(bind_addr)
        logger.info(f"Visualization stream bound to {bind_addr}")
        
        return socket

    def _update_active_mode(self):
        """Update active mode based on genome availability."""
        old_mode = self._active_mode
        
        try:
            self._active_mode = self.core_api.genome_is_loaded() if self.core_api else False
        except Exception as e:
            logger.warning(f"Error checking genome state: {e}, defaulting to standby mode")
            self._active_mode = False
        
        if old_mode != self._active_mode:
            if self._active_mode:
                logger.info("VisualizationStream entering ACTIVE mode")
            else:
                logger.info("VisualizationStream entering STANDBY mode")

    def _on_genome_state_change(self, old_state, new_state):
        """Handle genome state changes."""
        try:
            if new_state == GenomeState.LOADED:
                self._active_mode = True
                if self.running:
                    logger.info("VisualizationStream entering ACTIVE mode")
            else:
                self._active_mode = False 
                if self.running:
                    logger.info("VisualizationStream entering STANDBY mode")
        except Exception as e:
            logger.error(f"Error handling genome state change: {e}")
            self._active_mode = False

    async def start(self) -> None:
        """Start the visualization stream server."""
        if self.running:
            return
            
        logger.info(f"Starting Visualization Stream server on {self.host}:{self.port}")
        self.running = True
        
        # Start FQ processing tasks if FQ sampler is available
        if self.fq_sampler_queue:
            self.tasks.append(asyncio.create_task(self._process_fq_data()))
            
        # Start client tracking task
        self.tasks.append(asyncio.create_task(self._cleanup_disconnected_clients()))
            
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

    async def _process_fq_data(self) -> None:
        """Process FQ data from the sampler queue."""
        if not self.fq_sampler_queue:
            logger.debug("No FQ sampler queue available")
            return
            
        logger.info("Starting FQ data processing")
        
        while self.running:
            try:
                # Get data from queue (non-blocking)
                fq_data = None
                try:
                    if hasattr(self.fq_sampler_queue, 'get'):
                        fq_data = self.fq_sampler_queue.get(block=False)
                    elif hasattr(self.fq_sampler_queue, '_queue') and len(self.fq_sampler_queue._queue) > 0:
                        fq_data = self.fq_sampler_queue._queue.pop(0)
                    else:
                        await asyncio.sleep(0.01)
                        continue
                except Exception:
                    await asyncio.sleep(0.01)
                    continue 
                
                if fq_data is None:
                    await asyncio.sleep(0.01)
                    continue
                
                # Handle different data types
                if isinstance(fq_data, bytes):
                    # Already encoded binary data
                    if self.get_connected_client_count() > 0:
                        await self._send_binary_data(fq_data)
                
                elif isinstance(fq_data, tuple) and len(fq_data) == 2:
                    # Handle (cortical_id, fire_queue_data) tuple format
                    await self._process_fq_tuple(fq_data)
                
                elif isinstance(fq_data, dict):
                    # Handle fire queue dict directly
                    await self._process_fq_dict(fq_data)
                
                elif isinstance(fq_data, str) and fq_data == "STOP":
                    logger.info("Received STOP signal")
                    break
                
                else:
                    logger.warning(f"Unsupported FQ data type: {type(fq_data)}")
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in FQ data processing: {e}")
                await asyncio.sleep(0.1)

    async def _process_fq_tuple(self, fq_data):
        """Process a 2-element FQ tuple and convert to visualization data."""
        try:
            cortical_id, fire_queue_data = fq_data
            
            if not fire_queue_data or not fire_queue_data.get('neuron_ids'):
                return
                
            if self.get_connected_client_count() == 0:
                return
                
            # Extract data from fire queue
            neuron_ids = fire_queue_data['neuron_ids']
            membrane_potentials = fire_queue_data.get('membrane_potentials', [])
            coordinates = fire_queue_data.get('coordinates', [])
            
            # Use coordinates if available, otherwise generate from IDs
            if coordinates and len(coordinates) == len(neuron_ids):
                x_values = [coord[0] for coord in coordinates]
                y_values = [coord[1] for coord in coordinates]
                z_values = [coord[2] for coord in coordinates]
            else:
                # Fallback to ID-based coordinates
                x_values = [nid % 100 for nid in neuron_ids]
                y_values = [(nid // 100) % 100 for nid in neuron_ids]
                z_values = [nid // 10000 for nid in neuron_ids]
            
            # Use membrane potentials if available, otherwise default to 1.0
            if membrane_potentials and len(membrane_potentials) == len(neuron_ids):
                potentials = membrane_potentials
            else:
                potentials = [1.0] * len(neuron_ids)
            
            # Create cortical ID list (one per neuron)
            cortical_ids = [cortical_id] * len(neuron_ids)
            
            # Encode using feagi_bytes
            try:
                from feagi_bytes import ByteStructureEncoder
                encoder = ByteStructureEncoder()
                
                binary_data = encoder.encode_neuron_flat(
                    cortical_ids=cortical_ids,
                    x_coords=x_values,
                    y_coords=y_values,
                    z_coords=z_values,
                    potentials=potentials
                )
                
                await self._send_binary_data(binary_data)
                
            except Exception as e:
                logger.error(f"Error encoding visualization data: {e}")
                
        except Exception as e:
            logger.error(f"Error processing FQ tuple: {e}")

    async def _process_fq_dict(self, fire_queue_data):
        """Process a fire queue dictionary directly."""
        try:
            if not fire_queue_data or not fire_queue_data.get('neuron_ids'):
                return
                
            if self.get_connected_client_count() == 0:
                return
                
            # Extract data from fire queue
            neuron_ids = fire_queue_data['neuron_ids']
            membrane_potentials = fire_queue_data.get('membrane_potentials', [])
            coordinates = fire_queue_data.get('coordinates', [])
            
            # Use coordinates if available, otherwise generate from IDs
            if coordinates and len(coordinates) == len(neuron_ids):
                x_values = [coord[0] for coord in coordinates]
                y_values = [coord[1] for coord in coordinates]
                z_values = [coord[2] for coord in coordinates]
            else:
                # Fallback to ID-based coordinates
                x_values = [nid % 100 for nid in neuron_ids]
                y_values = [(nid // 100) % 100 for nid in neuron_ids]
                z_values = [nid // 10000 for nid in neuron_ids]
            
            # Use membrane potentials if available, otherwise default to 1.0
            if membrane_potentials and len(membrane_potentials) == len(neuron_ids):
                potentials = membrane_potentials
            else:
                potentials = [1.0] * len(neuron_ids)
            
            # Use default cortical ID
            cortical_ids = ['default'] * len(neuron_ids)
            
            # Encode using feagi_bytes
            try:
                from feagi_bytes import ByteStructureEncoder
                encoder = ByteStructureEncoder()
                
                binary_data = encoder.encode_neuron_flat(
                    cortical_ids=cortical_ids,
                    x_coords=x_values,
                    y_coords=y_values,
                    z_coords=z_values,
                    potentials=potentials
                )
                
                await self._send_binary_data(binary_data)
                
            except Exception as e:
                logger.error(f"Error encoding visualization data: {e}")
                
        except Exception as e:
            logger.error(f"Error processing FQ dict: {e}")

    async def _send_binary_data(self, binary_data: bytes):
        """Send binary data to visualization clients."""
        try:
            # Skip if in standby mode
            if not self._active_mode:
                return
                
            # Apply rate limiting
            if not self.rate_limiter.check_rate("visualization", 0.05):  # Max 20Hz
                return
            
            # Send data on activity topic
            await self.socket.send_multipart([
                b"activity",
                binary_data
            ])
            
            logger.debug(f"Sent {len(binary_data)} bytes of visualization data")
            
        except Exception as e:
            logger.error(f"Error sending binary data: {e}")

    async def _cleanup_disconnected_clients(self) -> None:
        """Periodically clean up disconnected clients based on heartbeat timeout."""
        while self.running:
            try:
                current_time = time.time()
                client_ids = list(self.client_last_heartbeat.keys())
                
                for client_id in client_ids:
                    last_heartbeat = self.client_last_heartbeat[client_id]
                    if current_time - last_heartbeat > self.client_heartbeat_timeout:
                        logger.info(f"Client {client_id} disconnected (timeout)")
                        del self.client_last_heartbeat[client_id]
                        
            except Exception as e:
                logger.error(f"Error cleaning up clients: {e}")
                
            await asyncio.sleep(10)

    def get_connected_client_count(self) -> int:
        """Get the estimated number of connected visualization clients."""
        return len(self.client_last_heartbeat)

    async def record_client_heartbeat(self, client_id: str) -> None:
        """Record a heartbeat from a client."""
        now = time.time()
        old_time = self.client_last_heartbeat.get(client_id, 0)
        self.client_last_heartbeat[client_id] = now
        
        if old_time == 0:
            logger.info(f"New visualization client connected: {client_id}")

    async def send_visualization_data(self, data) -> None:
        """
        Send visualization data to clients.
        
        Args:
            data: Visualization data (bytes, 2-element tuple, or fire queue dict)
        """
        if not self.running or not self.socket:
            return
            
        # Skip if in standby mode
        if not self._active_mode:
            return
            
        # Skip if no clients connected
        if self.get_connected_client_count() == 0:
            return
        
        try:
            if isinstance(data, bytes):
                await self._send_binary_data(data)
            elif isinstance(data, tuple) and len(data) == 2:
                await self._process_fq_tuple(data)
            elif isinstance(data, dict):
                await self._process_fq_dict(data)
            else:
                logger.warning(f"Unsupported data type: {type(data)}")
                
        except Exception as e:
            logger.error(f"Error in send_visualization_data: {e}")

    async def broadcast_update(self, data_type: str, data: bytes) -> None:
        """
        Broadcast an update to all connected agents.
        
        Args:
            data_type: Type of data ("activity", "structure", "system")
            data: Binary data
        """
        if not self.running or not self.socket:
            return
            
        # Skip if in standby mode (except for system messages)
        if not self._active_mode and data_type != "system":
            return
            
        try:
            # Apply rate limiting
            if not self.rate_limiter.check_rate(f"broadcast_{data_type}", 0.05):
                return
                
            await self.socket.send_multipart([
                data_type.encode('utf-8'),
                data
            ])
            
            logger.debug(f"Broadcast {len(data)} bytes of {data_type} data")
            
        except Exception as e:
            logger.error(f"Error broadcasting {data_type} data: {e}")

    async def process_system_message(self, message: str) -> None:
        """Process system messages from clients."""
        try:
            if "HEARTBEAT:" in message:
                parts = message.split(":")
                if len(parts) >= 2:
                    client_id = parts[1].strip()
                    await self.record_client_heartbeat(client_id)
                    
        except Exception as e:
            logger.error(f"Error processing system message: {e}")

    async def receive_control_message(self, message: Dict[str, Any]) -> Dict[str, Any]:
        """Process control messages from clients."""
        try:
            message_type = message.get("message_type", "unknown")
            
            if message_type == "heartbeat":
                client_id = message.get("agent_id", f"unknown_{time.time()}")
                await self.record_client_heartbeat(client_id)
                return {
                    "status": "ok",
                    "message": "Heartbeat received",
                    "timestamp": time.time()
                }
            
            return {
                "status": "error",
                "message": f"Unknown message type: {message_type}",
                "timestamp": time.time()
            }
            
        except Exception as e:
            logger.error(f"Error processing control message: {e}")
            return {
                "status": "error",
                "message": str(e),
                "timestamp": time.time()
            } 