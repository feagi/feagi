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
FEAGI Motor Stream - For Robot/Agent Motor Control ONLY

⚠️ IMPORTANT: This stream is for MOTOR CONTROL, NOT brain visualization!
   - Motor data uses Type 10 (NEURON_FLAT) format and should stay that way
   - Do NOT change this to Type 11 for "DPR compatibility" 
   - DPR (Direct Point Rendering) is ONLY for the visualization stream
   - Motor commands are sent to robots/agents for movement control
   - Completely separate from brain visualization data

This stream handles:
- Real-time motor commands to robotic agents
- Low-latency control signals  
- Motor cortex output (OPU areas)
- Agent/robot movement commands

This stream does NOT handle:
- Brain visualization data (that's the visualization stream)
- Neural activity rendering 
- Brain monitoring/analysis
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
    
    Motor Subscriber Management:
    - Automatically detects motor stream subscribers via heartbeat tracking
    - Controls FQ sampler to sample OPU cortical areas at burst frequency
    - Provides efficient motor data delivery for real-time control applications
    """
    
    def __init__(
        self, 
        core_api: CoreAPIService,
        host: str = "*", 
        port: int = 5564,
        context: Optional[zmq.asyncio.Context] = None,
        fq_sampler: Optional[Any] = None,
        fq_sampler_queue: Optional[Any] = None,
        stream_config: Optional[Dict[str, Any]] = None
    ):
        """
        Initialize the Motor Stream.
        
        Args:
            core_api: The CoreAPIService instance to delegate calls to
            host: Host address to bind to
            port: Port for sending motor data
            context: Optional existing ZMQ context to use
            fq_sampler: Optional FQ sampler instance for motor data sampling
            fq_sampler_queue: Optional queue for receiving motor data from FQ sampler
            stream_config: Optional stream configuration
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
        
        # Motor subscriber management
        self.fq_sampler = fq_sampler
        self.fq_sampler_queue = fq_sampler_queue
        self.client_last_heartbeat: Dict[str, float] = {}
        self.client_heartbeat_timeout = 30.0  # 30 seconds timeout
        self.subscriber_check_interval = 2.0  # Check every 2 seconds
        self._last_subscriber_count = 0
        self._fq_sampler_enabled = False
        self._subscriber_count = 0
        
        # Motor stream processing task
        self._motor_data_task: Optional[asyncio.Task] = None
        self._subscriber_monitor_task: Optional[asyncio.Task] = None
        
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
        
        # Start motor data processing if FQ sampler queue is available
        if self.fq_sampler_queue:
            self._motor_data_task = asyncio.create_task(self._process_motor_data())
            
        # Start subscriber monitoring
        self._subscriber_monitor_task = asyncio.create_task(self._monitor_subscribers())
        
        logger.info("Motor Stream server started")

    async def stop(self) -> None:
        """Stop the motor stream server."""
        if not self.running:
            return
            
        logger.info("Stopping Motor Stream server")
        self.running = False
        
        # Stop tasks
        if self._motor_data_task:
            self._motor_data_task.cancel()
            try:
                await self._motor_data_task
            except asyncio.CancelledError:
                pass
            self._motor_data_task = None
            
        if self._subscriber_monitor_task:
            self._subscriber_monitor_task.cancel()
            try:
                await self._subscriber_monitor_task
            except asyncio.CancelledError:
                pass
            self._subscriber_monitor_task = None
        
        # Disable FQ sampler if it was enabled
        if self._fq_sampler_enabled:
            await self._control_fq_sampler(False)
        
        # Close the socket
        if self.socket:
            self.socket.close()
            self.socket = None
            
        logger.info("Motor Stream server stopped")

    async def _process_motor_data(self) -> None:
        """Process motor data from FQ sampler queue."""
        if not self.fq_sampler_queue:
            logger.warning("No FQ sampler queue available for motor data processing")
            return
            
        logger.debug("Starting motor data processing")
            
        while self.running:
            try:
                # Get data from queue (non-blocking)
                motor_data = None
                try:
                    if hasattr(self.fq_sampler_queue, 'get'):
                        motor_data = self.fq_sampler_queue.get(block=False)
                    elif hasattr(self.fq_sampler_queue, '_queue') and len(self.fq_sampler_queue._queue) > 0:
                        motor_data = self.fq_sampler_queue._queue.pop(0)
                    else:
                        await asyncio.sleep(0.01)
                        continue
                except Exception:
                    await asyncio.sleep(0.01)
                    continue 
                
                if motor_data is None:
                    await asyncio.sleep(0.01)
                    continue
                    
                # Handle different data types for motor processing
                if isinstance(motor_data, bytes):
                    # Already encoded binary data
                    if self.get_connected_client_count() > 0:
                        await self._send_motor_binary_data(motor_data)
                
                elif isinstance(motor_data, dict) and 'target' in motor_data:
                    # Handle new tagged format from enhanced FQ sampler
                    await self._process_tagged_motor_data(motor_data)
                
                elif isinstance(motor_data, tuple) and len(motor_data) == 2:
                    # Handle (cortical_id, fire_queue_data) tuple format
                    await self._process_motor_tuple(motor_data)
                
                elif isinstance(motor_data, dict):
                    # Handle fire queue dict directly
                    await self._process_motor_dict(motor_data)
                
                elif isinstance(motor_data, str) and motor_data == "STOP":
                    logger.info("Received STOP signal")
                    break 
                
                else:
                    logger.debug(f"Unsupported motor data type: {type(motor_data)}")
                
            except asyncio.CancelledError:
                break 
            except Exception as e: 
                logger.error(f"Error in motor data processing: {e}")
                await asyncio.sleep(0.1) 

    async def _process_tagged_motor_data(self, motor_data):
        """Process tagged data from enhanced FQ sampler for motor streams."""
        try:
            target = motor_data.get('target', 'motor')
            
            # Only process motor-targeted data in motor stream
            if target != 'motor':
                logger.debug(f"Skipping non-motor data (target: {target})")
                return
                
            # Extract the actual fire queue data
            if 'cortical_id' in motor_data and 'fire_queue_data' in motor_data:
                # Area-specific data (OPU areas)
                cortical_id = motor_data['cortical_id']
                fire_queue_data = motor_data['fire_queue_data']
                await self._process_motor_tuple((cortical_id, fire_queue_data))
                
            elif 'fire_queue_data' in motor_data:
                # Global data (filtered for motor)
                fire_queue_data = motor_data['fire_queue_data']
                await self._process_motor_dict(fire_queue_data)
                
            else:
                logger.warning(f"Invalid tagged motor data format: {motor_data.keys()}")
                
        except Exception as e:
            logger.error(f"Error processing tagged motor data: {e}")

    async def _process_motor_tuple(self, motor_data):
        """Process a 2-element motor tuple and convert to motor data."""
        try:
            cortical_id, fire_queue_data = motor_data
            
            if not fire_queue_data or not fire_queue_data.get('neuron_ids'):
                return
                
            # Check if we have connected clients
            client_count = self.get_connected_client_count()
            
            if client_count == 0:
                logger.debug(f"No motor clients connected, skipping data for {cortical_id}")
                return
                
            # Extract data from fire queue
            neuron_ids = fire_queue_data['neuron_ids']
            membrane_potentials = fire_queue_data.get('membrane_potentials', [])
            coordinates = fire_queue_data.get('coordinates', [])
            
            # Use membrane potentials if available, otherwise default to 1.0
            if membrane_potentials and len(membrane_potentials) == len(neuron_ids):
                potentials = membrane_potentials
            else:
                potentials = [1.0] * len(neuron_ids)
            
            # Create motor data structure
            motor_data_dict = {
                'cortical_id': cortical_id,
                'neuron_ids': neuron_ids,
                'potentials': potentials,
                'coordinates': coordinates,
                'timestamp': time.time()
            }
                        
            # Encode using feagi_bytes for motor data - USE TYPE 11 FOR DPR CONSISTENCY
            try:
                from feagi_bytes import ByteStructureEncoder
                encoder = ByteStructureEncoder()
                            
                # Use Type 11 (NEURON_CATEGORIES) format for consistency with DPR system
                if coordinates and len(coordinates) == len(neuron_ids):
                    x_values = [coord[0] for coord in coordinates]
                    y_values = [coord[1] for coord in coordinates]
                    z_values = [coord[2] for coord in coordinates]
                else:
                    # Fallback to ID-based coordinates
                    x_values = [(nid % 100) if nid > 0 else 1 for nid in neuron_ids]
                    y_values = [((nid // 100) % 100) if nid > 0 else 1 for nid in neuron_ids]
                    z_values = [(nid // 10000) if nid > 0 else 0 for nid in neuron_ids]
                
                # Convert to Type 11 (NEURON_CATEGORIES) format
                cortical_data = {
                    cortical_id: {
                        'x': x_values,
                        'y': y_values,
                        'z': z_values,
                        'potentials': potentials
                    }
                }
                
                binary_data = encoder.encode_neuron_categories(cortical_data)
                
                await self._send_motor_binary_data(binary_data, channel=cortical_id)
                
            except Exception as e:
                logger.error(f"Error encoding motor data: {e}")
                        
        except Exception as e:
            logger.error(f"Error processing motor tuple: {e}")

    async def _process_motor_dict(self, fire_queue_data):
        """Process a fire queue dictionary directly for motor data."""
        try:
            if not fire_queue_data or not fire_queue_data.get('neuron_ids'):
                return
                
            # Check if we have connected clients
            client_count = self.get_connected_client_count()
            
            if client_count == 0:
                logger.debug("No motor clients connected, skipping dict data")
                return
                
            # Extract data from fire queue
            neuron_ids = fire_queue_data['neuron_ids']
            membrane_potentials = fire_queue_data.get('membrane_potentials', [])
            coordinates = fire_queue_data.get('coordinates', [])
            
            # Use membrane potentials if available, otherwise default to 1.0
            if membrane_potentials and len(membrane_potentials) == len(neuron_ids):
                potentials = membrane_potentials
            else:
                potentials = [1.0] * len(neuron_ids)
            
            # Use default cortical ID for motor
            cortical_ids = ['motor'] * len(neuron_ids)
            
            # Encode using feagi_bytes for motor data - USE TYPE 11 FOR DPR CONSISTENCY
            try:
                from feagi_bytes import ByteStructureEncoder
                encoder = ByteStructureEncoder()
                
                # Use Type 11 (NEURON_CATEGORIES) format for consistency with DPR system
                if coordinates and len(coordinates) == len(neuron_ids):
                    x_values = [coord[0] for coord in coordinates]
                    y_values = [coord[1] for coord in coordinates]
                    z_values = [coord[2] for coord in coordinates]
                else:
                    # Fallback to ID-based coordinates
                    x_values = [nid % 100 for nid in neuron_ids]
                    y_values = [(nid // 100) % 100 for nid in neuron_ids]
                    z_values = [nid // 10000 for nid in neuron_ids]
                            
                binary_data = encoder.encode_neuron_flat(
                    cortical_ids=cortical_ids,
                    x_coords=x_values,
                    y_coords=y_values,
                    z_coords=z_values,
                    potentials=potentials
                )
                
                await self._send_motor_binary_data(binary_data, channel="motor")
                
            except Exception as e:
                logger.error(f"Error encoding motor data: {e}")
                
        except Exception as e:
            logger.error(f"Error processing motor dict: {e}")

    async def _send_motor_binary_data(self, binary_data: bytes, channel: str = "motor"):
        """Send binary motor data to motor clients."""
        try:
            # Skip if in standby mode
            if not self._active_mode:
                logger.debug("Motor stream in STANDBY mode, skipping data send")
                return
                
            # Send data on specified motor channel
            await self.socket.send_multipart([
                channel.encode('utf-8'),
                binary_data
            ])
            
            logger.debug(f"Sent {len(binary_data)} bytes of motor data on channel {channel}")
            
        except Exception as e:
            logger.error(f"Error sending motor binary data: {e}")

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

    def get_connected_client_count(self) -> int:
        """Get the current number of connected motor clients."""
        try:
            now = time.time()
            active_clients = 0
            
            for client_id, last_heartbeat in self.client_last_heartbeat.items():
                if now - last_heartbeat < self.client_heartbeat_timeout:
                    active_clients += 1
            
            return active_clients
        except Exception as e:
            logger.warning(f"Error getting motor client count: {e}")
            return 0

    async def record_client_heartbeat(self, client_id: str) -> None:
        """Record a heartbeat from a motor client."""
        current_time = time.time()
        self.client_last_heartbeat[client_id] = current_time

    async def _monitor_subscribers(self) -> None:
        """Monitor ZMQ motor subscribers and automatically enable/disable FQ sampler."""
        logger.info("Starting motor subscriber monitoring for automatic FQ sampler control")
        
        while self.running:
            try:
                # Check current subscriber count
                current_count = self.get_connected_client_count()
                
                # Update subscriber count
                if current_count != self._last_subscriber_count:
                    logger.info(f"Motor subscriber count changed: {self._last_subscriber_count} -> {current_count}")
                    self._last_subscriber_count = current_count
                    
                    # Auto-enable/disable FQ sampler based on subscriber count
                    should_enable = current_count > 0
                    
                    if should_enable != self._fq_sampler_enabled:
                        await self._control_fq_sampler(should_enable)
                
                # Wait for next check
                await asyncio.sleep(self.subscriber_check_interval)
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in motor subscriber monitoring: {e}")
                await asyncio.sleep(self.subscriber_check_interval)
        
        logger.info("Motor subscriber monitoring stopped")

    async def register_motor_client(self, client_id: str) -> None:
        """Register a motor client and update heartbeat."""
        current_time = time.time()
        self.client_last_heartbeat[client_id] = current_time
        logger.info(f"🚗 Motor client registered: {client_id}")
        
        # Force a subscriber count update
        current_count = self.get_connected_client_count()
        if current_count != self._last_subscriber_count:
            self._last_subscriber_count = current_count
            should_enable = current_count > 0
            if should_enable != self._fq_sampler_enabled:
                await self._control_fq_sampler(should_enable)

    async def unregister_motor_client(self, client_id: str) -> None:
        """Unregister a motor client."""
        if client_id in self.client_last_heartbeat:
            del self.client_last_heartbeat[client_id]
            logger.info(f"🚗 Motor client unregistered: {client_id}")
            
            # Force a subscriber count update
            current_count = self.get_connected_client_count()
            if current_count != self._last_subscriber_count:
                self._last_subscriber_count = current_count
                should_enable = current_count > 0
                if should_enable != self._fq_sampler_enabled:
                    await self._control_fq_sampler(should_enable)

    async def heartbeat_motor_client(self, client_id: str) -> None:
        """Update heartbeat for a motor client."""
        self.client_last_heartbeat[client_id] = time.time()
        # Don't log every heartbeat to avoid spam, just update the timestamp

    async def _control_fq_sampler(self, enable: bool) -> None:
        """Enable or disable the FQ sampler based on motor subscriber presence."""
        try:
            if not self.fq_sampler:
                # Try to get FQ sampler from process manager
                try:
                    from feagi.process_manager import get_process_manager
                    process_manager = get_process_manager()
                    if process_manager and hasattr(process_manager, '_fq_sampler'):
                        self.fq_sampler = process_manager._fq_sampler
                        logger.info("Found FQ sampler from process manager")
                except Exception:
                    pass
            
            if self.fq_sampler and hasattr(self.fq_sampler, 'set_motor_subscribers'):
                if enable:
                    logger.info("🔔 Enabling FQ sampler for motor - motor clients connected")
                    self.fq_sampler.set_motor_subscribers(True)
                    self._fq_sampler_enabled = True
                else:
                    logger.info("🔕 Disabling FQ sampler for motor - no motor clients")
                    self.fq_sampler.set_motor_subscribers(False)
                    self._fq_sampler_enabled = False
            else:
                if enable:
                    logger.warning("FQ sampler not available or doesn't support set_motor_subscribers")
                
        except Exception as e:
            logger.error(f"Error controlling FQ sampler for motor: {e}") 