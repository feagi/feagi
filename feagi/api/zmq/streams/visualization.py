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
FEAGI Visualization Stream - UnifiedFQSampler Implementation

This is the primary visualization stream implementation for FEAGI, using the
new UnifiedFQSampler architecture with cortical area-based data format.

Features:
- UnifiedFQSampler integration with 'visualization' mode
- Cortical area-based data format (no legacy compatibility)
- Threading-based implementation for RTOS compatibility
- Client tracking with heartbeat timeouts
- Automatic FQ sampler control based on subscriber presence
- Standby mode when genome not loaded
- High performance binary encoding using feagi_bytes

Design principles:
- Threading-based (RTOS compatible) instead of async
- Synchronous ZMQ context (no async/sync conflicts)
- Only supports new UnifiedFQSampler cortical area format
- No backward compatibility with legacy formats
"""

import time
import threading
import uuid
import json
from typing import Dict, List, Any, Optional, Set, Tuple, Union, Callable
from collections import defaultdict
from dataclasses import dataclass, field
from queue import Empty
import zmq  # Import standard synchronous ZMQ (not zmq.asyncio)

from feagi.utils.logger import setup_logger
from feagi.utils.zmq_debug import log_outbound, MessageType
from ...core.services.core_api_service import CoreAPIService

logger = setup_logger(__name__)


class VisualizationStream:
    """
    FEAGI Visualization Stream - UnifiedFQSampler Implementation.
    
    This visualization stream only supports the new UnifiedFQSampler architecture
    with cortical area-based data format. No legacy compatibility is maintained.
    
    Features:
    - UnifiedFQSampler integration with 'visualization' mode
    - Cortical area-based data format only
    - Client tracking with heartbeat timeouts  
    - Automatic FQ sampler control based on subscriber presence
    - Standby mode when genome not loaded
    - High performance binary encoding
    """
    
    def __init__(
        self, 
        host: str = "*", 
        port: int = 5562,
        context: Optional[zmq.Context] = None,
        fire_queue_provider = None,
        stream_config: Optional[Dict[str, Any]] = None,
        core_api: Optional[Any] = None,
        connectome_manager = None
    ):
        """Initialize the visualization stream with UnifiedFQSampler only."""
        # Core API integration for genome state management
        self.core_api = core_api
        
        # Basic connection settings
        self.host = host
        self.port = port
        self.running = False
        
        # ALWAYS create a NEW sync context - NEVER use shared contexts
        self.context = zmq.Context()
        
        # UnifiedFQSampler integration - REQUIRED, no fallbacks
        if not fire_queue_provider:
            raise ValueError("fire_queue_provider is required for UnifiedFQSampler")
        
        # Create UnifiedFQSampler for visualization mode
        from feagi.npu.fq_sampler import UnifiedFQSampler
        self.fq_sampler = UnifiedFQSampler(
            fire_queue_provider=fire_queue_provider,
            sample_frequency_hz=30.0,  # 30Hz for visualization
            sampling_mode='visualization',  # Always use visualization mode
            connectome_manager=connectome_manager or getattr(core_api, '_connectome_manager', None)
        )
        print(f"🔧 VIZ DEBUG: Created UnifiedFQSampler: {self.fq_sampler}")
        print(f"   - fire_queue_provider: {fire_queue_provider}")
        print(f"   - connectome_manager: {connectome_manager or getattr(core_api, '_connectome_manager', None)}")
        logger.info("Created UnifiedFQSampler for visualization with 'visualization' mode")
        
        # IMPORTANT: Start FQ sampler in DISABLED state (no clients connected yet)
        if hasattr(self.fq_sampler, 'set_visualization_subscribers'):
            self.fq_sampler.set_visualization_subscribers(False)
            print(f"🔧 VIZ DEBUG: UnifiedFQSampler initialized in DISABLED state")
            logger.debug("UnifiedFQSampler initialized in DISABLED state - will enable when clients connect")
        else:
            print(f"⚠️ VIZ DEBUG: UnifiedFQSampler does NOT have 'set_visualization_subscribers' method!")
        
        # Stream configuration
        self.stream_config = stream_config or {}
        self.auto_enable_on_subscribers = self.stream_config.get('auto_enable_on_subscribers', True)
        self.subscriber_check_interval = self.stream_config.get('subscriber_check_interval', 1.0)
        
        # Genome state management
        self._active_mode = False  # True when genome is loaded and ready
        
        # Client tracking with heartbeat timeouts
        self.client_last_heartbeat = {}  # Mapping of client_id -> last heartbeat time
        self.client_heartbeat_timeout = 30  # Consider clients disconnected after 30s
        self._client_lock = threading.Lock()  # Thread-safe client access
        
        # Subscriber monitoring for automatic FQ sampler control
        self._subscriber_count = 0
        self._last_subscriber_count = 0
        self._fq_sampler_enabled = False
        
        # Socket setup
        self.socket = None
        self._setup_socket()
        
        # Worker threads
        self.worker_threads = []
        self._stop_event = threading.Event()
        
        # Register for genome state change notifications
        if hasattr(core_api, 'register_genome_change_listener'):
            core_api.register_genome_change_listener(self._on_genome_state_change)
        
        # Initialize state based on current genome availability
        self._update_active_mode()
        
        # Statistics
        self.stats = {
            'data_sent': 0,
            'bytes_sent': 0,
            'start_time': time.time()
        }

    def _setup_socket(self) -> None:
        """Set up the ZMQ PUB socket with optimal settings."""
        self.socket = self.context.socket(zmq.PUB)
        
        # Optimize for real-time streaming
        self.socket.setsockopt(zmq.SNDHWM, 1000)   # Higher send buffer to prevent drops
        self.socket.setsockopt(zmq.LINGER, 1000)   # Wait briefly on close to send pending messages
        
        bind_addr = f"tcp://{self.host}:{self.port}"
        self.socket.bind(bind_addr)
        logger.info(f"[COMM] Visualization stream bound to {bind_addr}")

    def _update_active_mode(self):
        """Update active mode based on genome availability."""
        old_mode = self._active_mode
        
        try:
            if self.core_api:
                self._active_mode = self.core_api.genome_is_loaded()
            else:
                # If no core_api, assume active mode
                self._active_mode = True
        except Exception as e:
            logger.warning(f"Error checking genome state: {e}, defaulting to active mode")
            self._active_mode = True
        
        if old_mode != self._active_mode:
            if self._active_mode:
                logger.info("VisualizationStream entering ACTIVE mode")
            else:
                logger.info("VisualizationStream entering STANDBY mode")

    def _on_genome_state_change(self, old_state, new_state):
        """Handle genome state changes."""
        try:
            from feagi.core.state_manager import GenomeState
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

    def start(self) -> None:
        """Start the visualization stream with worker threads."""
        if self.running:
            return
            
        logger.info("[START] Starting visualization stream")
        self.running = True
        self._stop_event.clear()
        
        # Thread 1: FQ data processing (main data processing)
        fq_thread = threading.Thread(
            target=self._data_worker,
            name="VisualizationFQ",
            daemon=True
        )
        fq_thread.start()
        self.worker_threads.append(fq_thread)
        logger.debug("FQ data processing thread started")
            
        # Thread 2: Client cleanup and monitoring
        cleanup_thread = threading.Thread(
            target=self._client_cleanup_worker,
            name="VisualizationCleanup", 
            daemon=True
        )
        cleanup_thread.start()
        self.worker_threads.append(cleanup_thread)
        logger.debug("Client cleanup thread started")
        
        # Thread 3: Subscriber monitoring for automatic FQ sampler control
        if self.auto_enable_on_subscribers:
            monitor_thread = threading.Thread(
                target=self._subscriber_monitor_worker,
                name="VisualizationMonitor",
                daemon=True
            )
            monitor_thread.start()
            self.worker_threads.append(monitor_thread)
            logger.debug("Subscriber monitoring thread started")
            
        logger.info(f"[OK] Visualization stream started with {len(self.worker_threads)} worker threads")

    def stop(self) -> None:
        """Stop the visualization stream gracefully."""
        if not self.running:
            return
            
        logger.info("[HALT] Stopping visualization stream...")
        self.running = False
        self._stop_event.set()
        
        # Wait for worker threads BEFORE closing socket to prevent race conditions
        total_threads = len(self.worker_threads)
        if total_threads > 0:
            logger.debug(f"Waiting for {total_threads} worker threads to stop before socket cleanup...")
            
            MAX_TOTAL_WAIT = 3.0  # Maximum 3 seconds total wait
            PER_THREAD_TIMEOUT = min(1.0, MAX_TOTAL_WAIT / max(total_threads, 1))

            import time
            start_time = time.time()

            for i, thread in enumerate(self.worker_threads, 1):
                # Check global timeout
                elapsed = time.time() - start_time
                if elapsed >= MAX_TOTAL_WAIT:
                    logger.warning(f"Global timeout reached, abandoning remaining {total_threads - i + 1} threads")
                    break

                if thread.is_alive():
                    remaining_timeout = min(PER_THREAD_TIMEOUT, MAX_TOTAL_WAIT - elapsed)
                    logger.debug(f"Waiting for thread {i}/{total_threads}: {thread.name} (timeout: {remaining_timeout:.1f}s)")

                    thread.join(timeout=remaining_timeout)
                    
                    if thread.is_alive():
                        logger.warning(f"Thread {thread.name} didn't stop after {remaining_timeout:.1f}s - continuing anyway")
                    else:
                        logger.debug(f"Thread {thread.name} stopped gracefully")
                else:
                    logger.debug(f"Thread {i}/{total_threads}: {thread.name} already stopped")

        # Close socket AFTER worker threads have stopped
        if self.socket:
            logger.debug("Closing ZMQ socket after worker threads stopped...")
            try:
                self.socket.close(linger=0)  # Don't wait for pending messages
                logger.debug("Socket closed successfully")
            except Exception as e:
                logger.warning(f"Error closing socket: {e}")
            finally:
                self.socket = None

        # Clear worker thread list
        self.worker_threads.clear()
            
        logger.info("[OK] Visualization stream stopped")

    def _data_worker(self) -> None:
        """
        Data processing worker using UnifiedFQSampler cortical area format only.
        Only samples data when visualization clients are connected.
        """
        logger.debug("Visualization data worker started")
        
        while self.running and not self._stop_event.is_set():
            try:
                # Quick exit check for shutdown
                if self._stop_event.is_set():
                    logger.debug("Data worker received stop signal")
                    break
                
                # Skip processing if in standby mode
                if not self._active_mode:
                    self._update_active_mode()
                    if not self._active_mode:
                        if self._stop_event.wait(timeout=0.1):  # Brief pause in standby mode
                            logger.debug("Data worker stopping during standby mode")
                            break
                        continue
                
                # CRITICAL: Only sample data if there are connected clients
                if not self._fq_sampler_enabled or self.get_connected_client_count() == 0:
                    # No clients connected - sleep and continue
                    client_count = self.get_connected_client_count()
                    enabled_state = self._fq_sampler_enabled
                    
                    # DEBUG: Show why we're not sampling
                    if not hasattr(self, '_debug_idle_count'):
                        self._debug_idle_count = 0
                    self._debug_idle_count += 1
                    
                    if self._debug_idle_count % 20 == 1:  # Every 10 seconds (20 * 0.5s)
                        print(f"🔍 VIZ DEBUG: Not sampling - clients={client_count}, fq_enabled={enabled_state}")
                        print(f"   - To test with no real clients, call: stream.heartbeat_visualization_client('debug_client')")
                    
                    if self._stop_event.wait(timeout=0.5):  # Wait 500ms before checking again
                        logger.debug("Data worker stopping while waiting for clients")
                        break
                    continue
                
                # Get data from UnifiedFQSampler ONLY when clients are connected
                vis_data = None
                try:
                    print(f"🔄 VIZ DEBUG: About to sample from fq_sampler (clients={self.get_connected_client_count()})")
                    vis_data = self.fq_sampler.sample()
                    print(f"📊 VIZ DEBUG: fq_sampler.sample() returned: {type(vis_data)} with content: {vis_data}")
                    
                    if vis_data:
                        print(f"✅ VIZ DEBUG: Got data from UnifiedFQSampler: {len(vis_data)} cortical areas")
                        logger.debug(f"Got data from UnifiedFQSampler: {len(vis_data)} cortical areas")
                    else:
                        print(f"❌ VIZ DEBUG: No data returned from UnifiedFQSampler")
                        
                except Exception as e:
                    print(f"💥 VIZ DEBUG: UnifiedFQSampler sampling error: {e}")
                    logger.debug(f"UnifiedFQSampler sampling error: {e}")
                    if self._stop_event.wait(timeout=0.01):  # Brief pause on error
                        logger.debug("Data worker stopping after sampling error")
                        break
                    continue
                
                if vis_data is None:
                    continue
                
                # Additional stop check before processing data
                if self._stop_event.is_set():
                    logger.debug("Data worker stopping before data processing")
                    break
                
                # Process cortical area format data only
                if isinstance(vis_data, dict):
                    logger.debug(f"Processing cortical area format: {len(vis_data)} areas")
                    if self.socket and self.running:
                        self._process_cortical_area_data(vis_data)
                    else:
                        logger.debug("Skipping processing: socket or stream not available")
                else:
                    logger.warning(f"UnifiedFQSampler returned unexpected data type: {type(vis_data)}")
                
            except Exception as e:
                logger.error(f"Error in data worker: {e}")
                if self._stop_event.wait(timeout=0.1):  # Brief pause on error
                    logger.debug("Data worker stopping after error")
                    break 
                
        logger.debug("Visualization data worker stopped")

    def _process_cortical_area_data(self, cortical_data: Dict[str, Any]) -> None:
        """Process data in the cortical area format from UnifiedFQSampler."""
        try:
            logger.debug(f"Processing cortical area format: {len(cortical_data)} areas")
            
            # Encode using feagi_bytes binary format - USE TYPE 11 (NEURON_CATEGORIES)
            try:
                from feagi_bytes import ByteStructureEncoder
                encoder = ByteStructureEncoder()

                # Convert cortical area data to the format expected by encoder
                encoder_data = {}
                for area_id, area_data in cortical_data.items():
                    if area_data and area_data.get('neuron_ids'):
                        neuron_ids = area_data['neuron_ids']
                        membrane_potentials = area_data.get('membrane_potentials', [])
                        coordinates = area_data.get('coordinates', [])
                        
                        # Use membrane potentials if available, otherwise default to 1.0
                        if membrane_potentials and len(membrane_potentials) == len(neuron_ids):
                            potentials = membrane_potentials
                        else:
                            potentials = [1.0] * len(neuron_ids)
                        
                        # Generate coordinates if not available
                        if coordinates and len(coordinates) == len(neuron_ids):
                            x_coords = [coord[0] for coord in coordinates]
                            y_coords = [coord[1] for coord in coordinates]
                            z_coords = [coord[2] for coord in coordinates]
                        else:
                            # Fallback to ID-based coordinates
                            x_coords = [nid % 100 for nid in neuron_ids]
                            y_coords = [(nid // 100) % 100 for nid in neuron_ids]
                            z_coords = [nid // 10000 for nid in neuron_ids]
                        
                        encoder_data[area_id] = {
                            'x': x_coords,
                            'y': y_coords,
                            'z': z_coords,
                            'potentials': potentials
                        }
                
                if encoder_data:
                    binary_data = encoder.encode_neuron_categories(encoder_data)

                    # DEBUG: Log the structure ID being generated
                    if binary_data and len(binary_data) > 0:
                        structure_id = binary_data[0]
                        logger.debug(f"VISUALIZATION STREAM: Generated {len(binary_data)} bytes")
                        logger.debug(f"   Structure ID (bytes[0]): {structure_id} (0x{structure_id:02X})")
                        logger.debug(f"   First 8 bytes: {list(binary_data[:min(8, len(binary_data))])}")

                        if structure_id == 11:
                            logger.debug(f"   ✅ Generated Type 11 (NEURON_CATEGORIES)")
                        else:
                            logger.debug(f"   ❓ Unknown structure type: {structure_id}")

                    # Publish the binary data
                    total_neurons = sum(len(area_data.get('neuron_ids', [])) for area_data in cortical_data.values())
                    self._publish_data(binary_data)
                    logger.debug(f"Published cortical area data: {len(cortical_data)} areas, {total_neurons} neurons, {len(binary_data)} bytes")
                    
            except ImportError:
                logger.error("feagi_bytes library not available - cannot encode binary data")
            except Exception as e:
                logger.error(f"Error encoding cortical area binary data: {e}")
                
        except Exception as e:
            logger.error(f"Error processing cortical area data: {e}")

    def _publish_data(self, data: bytes) -> None:
        """
        Publish data on the 'activity' topic with comprehensive error handling.
        """
        # Defensive null check to prevent race condition
        if not self.socket:
            logger.debug("Cannot publish data: socket is None (likely during shutdown)")
            return
            
        # Additional running state check
        if not self.running:
            logger.debug("Cannot publish data: stream is not running")
            return
            
        try:
            # Atomic socket reference to prevent mid-operation changes
            socket_ref = self.socket
            if not socket_ref:
                logger.debug("Socket became None during operation")
                return
            
            # Debug logging for outbound visualization data
            debug_endpoint = f"tcp://{self.host}:{self.port}"
            log_outbound(
                endpoint=debug_endpoint,
                data=[b"activity", data],  # PUB/SUB multipart message
                message_type=MessageType.VISUALIZATION,
                topic="activity",
                context=f"vis_msg_{self.stats['data_sent'] + 1}"
            )
                
            # Use synchronous send operations
            socket_ref.send(b"activity", zmq.SNDMORE)
            socket_ref.send(data)
            
            # Update statistics
            self.stats['data_sent'] += 1
            self.stats['bytes_sent'] += len(data)
            
            # Periodic status logging (every 100 messages)
            if self.stats['data_sent'] % 100 == 0:
                logger.debug(f"Published {self.stats['data_sent']} messages, {self.stats['bytes_sent']} bytes total")
            
            # Log first few messages to confirm publishing is working
            if self.stats['data_sent'] <= 3:
                logger.info(f"Successfully published message #{self.stats['data_sent']} ({len(data)} bytes)")
            
        except AttributeError as e:
            # Specific handling for socket = None race condition
            if "'NoneType' object has no attribute 'send'" in str(e):
                logger.debug("Socket became None during send operation (race condition during shutdown)")
                return
            else:
                logger.error(f"Unexpected AttributeError in publish_data: {e}")
                
        except zmq.ZMQError as e:
            # Enhanced ZMQ-specific error handling
            if e.errno == zmq.ETERM:
                logger.debug("ZMQ context terminated - stopping publish operations")
                return
            elif e.errno == zmq.EAGAIN:
                logger.warning("ZMQ socket not ready for sending (EAGAIN) - dropping message")
                return
            elif "Operation cannot be accomplished in current state" in str(e):
                logger.warning("ZMQ socket corrupted, attempting recreation...")
                try:
                    self._recreate_socket()
                    # Retry once with null check
                    if self.socket and self.running:
                        self.socket.send(b"activity", zmq.SNDMORE)
                        self.socket.send(data)
                        logger.info("Socket recreated and retry successful")
                    else:
                        logger.debug("Cannot retry: socket or stream not available after recreation")
                except Exception as retry_error:
                    logger.error(f"Socket recreation failed: {retry_error}")
            else:
                logger.error(f"ZMQ error in publish_data: {e} (errno: {e.errno})")
                
        except Exception as e:
            # Generic exception handling with detailed logging
            logger.error(f"Failed to publish data: {e}")
            logger.error(f"Publishing error type: {type(e).__name__}")
            if logger.isEnabledFor(10):  # DEBUG level
                import traceback
                logger.debug(f"Publishing traceback: {traceback.format_exc()}")

    def _recreate_socket(self):
        """Recreate the ZMQ socket when it gets corrupted."""
        logger.warning("Recreating corrupted ZMQ socket...")
        
        if not self.running:
            logger.debug("Not recreating socket: stream is shutting down")
            return
            
        if not self.context:
            logger.error("Cannot recreate socket: ZMQ context is None")
            return
        
        # Close old socket if it exists
        if self.socket:
            try:
                self.socket.close(linger=0)
                logger.debug("Old socket closed")
            except Exception as e:
                logger.warning(f"Error closing old socket (continuing): {e}")
        
        # Recreate socket with same settings
        try:
            self.socket = self.context.socket(zmq.PUB)
            self.socket.setsockopt(zmq.SNDHWM, 1000)   
            self.socket.setsockopt(zmq.LINGER, 1000)   
            
            bind_addr = f"tcp://{self.host}:{self.port}"
            self.socket.bind(bind_addr)
            logger.info(f"Socket recreated and bound to {bind_addr}")
            
        except zmq.ZMQError as e:
            logger.error(f"ZMQ error recreating socket: {e} (errno: {e.errno})")
            self.socket = None
            raise
        except Exception as e:
            logger.error(f"Failed to recreate socket: {e}")
            self.socket = None
            raise

    def get_stats(self) -> Dict[str, Any]:
        """Get visualization stream statistics."""
        runtime = time.time() - self.stats['start_time']
        return {
            'running': self.running,
            'data_sent': self.stats['data_sent'],
            'bytes_sent': self.stats['bytes_sent'],
            'runtime_seconds': runtime,
            'messages_per_second': self.stats['data_sent'] / max(runtime, 1)
        }

    def heartbeat_visualization_client(self, client_id: str) -> None:
        """Process client heartbeat and update FQ sampler accordingly."""
        current_time = time.time()
        
        with self._client_lock:  # Thread-safe access
            # Check if this is a new client
            is_new_client = client_id not in self.client_last_heartbeat
            
            self.client_last_heartbeat[client_id] = current_time
            total_clients = len(self.client_last_heartbeat)
            
            # If this is a new client, log it and update FQ sampler
            if is_new_client:
                logger.info(f"New visualization client connected: {client_id}")
                
                # Force a subscriber count update and FQ sampler notification
                current_count = total_clients
                last_count = self._last_subscriber_count
                
                if current_count != last_count:
                    self._last_subscriber_count = current_count
                    should_enable = current_count > 0
                    current_enabled = self._fq_sampler_enabled
                    
                    if should_enable != current_enabled:
                        try:
                            self._control_fq_sampler(should_enable)
                            logger.debug(f"FQ sampler {'enabled' if should_enable else 'disabled'} for {total_clients} clients")
                        except Exception as e:
                            logger.error(f"Error controlling FQ sampler: {e}")
            else:
                # Just log debug info for existing clients
                logger.debug(f"Heartbeat received from existing client: {client_id}")
        
        logger.debug(f"Visualization heartbeat processed for {client_id} (total clients: {total_clients})")

    def get_connected_client_count(self) -> int:
        """Get the number of connected visualization clients."""
        with self._client_lock:  # Thread-safe access
            return len(self.client_last_heartbeat)

    def _control_fq_sampler(self, enable: bool) -> None:
        """Enable or disable the FQ sampler based on subscriber presence."""
        try:
            if self.fq_sampler and hasattr(self.fq_sampler, 'set_visualization_subscribers'):
                if enable:
                    logger.info("Enabling FQ sampler - visualization clients connected", status="[CONFIG]")
                    self.fq_sampler.set_visualization_subscribers(True)
                    self._fq_sampler_enabled = True
                else:
                    logger.info("Disabling FQ sampler - no visualization clients", status="[CONFIG]")
                    self.fq_sampler.set_visualization_subscribers(False)
                    self._fq_sampler_enabled = False
            else:
                if enable:
                    logger.warning("FQ sampler doesn't support set_visualization_subscribers")
                
        except Exception as e:
            logger.error(f"Error controlling FQ sampler: {e}")
            if logger.isEnabledFor(10):  # DEBUG level  
                import traceback
                logger.debug(f"FQ sampler control traceback: {traceback.format_exc()}")

    def _client_cleanup_worker(self) -> None:
        """
        Client cleanup worker thread.
        Manages client heartbeat timeouts and automatic cleanup.
        """
        cleanup_interval = 5.0  # Check every 5 seconds
        
        while self.running and not self._stop_event.is_set():
            try:
                # Quick exit check for shutdown
                if self._stop_event.is_set():
                    logger.debug("Client cleanup worker received stop signal")
                    break
                
                current_time = time.time()
                
                with self._client_lock:  # Thread-safe client access
                    client_ids = list(self.client_last_heartbeat.keys())
                    
                    for client_id in client_ids:
                        last_heartbeat = self.client_last_heartbeat[client_id]
                        if current_time - last_heartbeat > self.client_heartbeat_timeout:
                            logger.info(f"Client {client_id} disconnected (timeout)")
                            del self.client_last_heartbeat[client_id]
                        
            except Exception as e:
                logger.error(f"Error cleaning up clients: {e}")
                
            # Use responsive wait with frequent stop event checks
            for _ in range(int(cleanup_interval * 4)):  # Check every 250ms
                if self._stop_event.wait(timeout=0.25):
                    logger.debug("Client cleanup worker stopping due to stop event")
                    return
        
        logger.debug("Client cleanup worker stopped")

    def _subscriber_monitor_worker(self) -> None:
        """
        Subscriber monitoring worker thread.
        Automatically enables/disables FQ sampler based on subscriber presence.
        """
        logger.debug("Starting subscriber monitoring for automatic FQ sampler control")
        
        while self.running and not self._stop_event.is_set():
            try:
                # Quick exit check for shutdown
                if self._stop_event.is_set():
                    logger.debug("Subscriber monitor received stop signal")
                    break
                
                # Check current subscriber count via client heartbeats
                current_count = self.get_connected_client_count()
                
                # Update subscriber count
                if current_count != self._last_subscriber_count:
                    logger.info(f"Visualization subscriber count changed: {self._last_subscriber_count} -> {current_count}")
                    self._last_subscriber_count = current_count
                    
                    # Auto-enable/disable FQ sampler based on subscriber count
                    should_enable = current_count > 0
                    
                    if should_enable != self._fq_sampler_enabled:
                        self._control_fq_sampler(should_enable)
                
                # Use responsive wait with frequent stop event checks
                wait_time = min(self.subscriber_check_interval, 1.0)  # Max 1 second intervals
                
                # Check every 200ms for faster shutdown response
                for _ in range(int(wait_time * 5)):  # Check every 200ms
                    if self._stop_event.wait(timeout=0.2):
                        logger.debug("Subscriber monitor stopping due to stop event")
                        return
                
            except Exception as e:
                logger.error(f"Error in subscriber monitoring: {e}")
                # Use responsive wait on error
                for _ in range(int(min(self.subscriber_check_interval, 1.0) * 5)):  # Check every 200ms
                    if self._stop_event.wait(timeout=0.2):
                        return
        
        logger.debug("Subscriber monitoring stopped")

    def enable_debug_sampling(self) -> None:
        """
        DEBUG ONLY: Enable FQ sampling for testing without real clients.
        This bypasses the normal client connection requirement.
        """
        print(f"🐛 VIZ DEBUG: Manually enabling FQ sampling for debugging")
        self._fq_sampler_enabled = True
        if hasattr(self.fq_sampler, 'set_visualization_subscribers'):
            self.fq_sampler.set_visualization_subscribers(True)
            print(f"✅ VIZ DEBUG: FQ sampler enabled via set_visualization_subscribers(True)")
        else:
            print(f"⚠️ VIZ DEBUG: No set_visualization_subscribers method available")
        print(f"   - _fq_sampler_enabled: {self._fq_sampler_enabled}")
        print(f"   - fq_sampler object: {self.fq_sampler}") 