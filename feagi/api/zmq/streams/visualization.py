"""
FEAGI Visualization Stream - Threading-Based Implementation

This is the primary visualization stream implementation for FEAGI, using a
threading-based approach for optimal reliability and RTOS compatibility.

This implementation includes all crucial features needed for visualization:
- Core API integration for genome state management
- Stream configuration support
- Enhanced client tracking with heartbeat timeouts
- Automatic FQ sampler control based on subscriber presence
- Standby mode when genome not loaded
- Full parameter compatibility with server expectations

Design principles:
- Threading-based (RTOS compatible) instead of async
- Synchronous ZMQ context (no async/sync conflicts)
- High performance: Minimal overhead, no artificial bottlenecks
- High reliability: No complex async state dependencies 
- Easy debugging: Simple data flow, clear logging

This implementation replaces the previous async-based visualization stream
that had context compatibility issues.
"""

import time
import threading
from typing import Dict, Any, Optional
from queue import Queue, Empty
import zmq  # Import standard synchronous ZMQ (not zmq.asyncio)

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class VisualizationStream:
    """
    FEAGI Visualization Stream - Threading-Based Implementation.
    
    This is the primary visualization stream for FEAGI, implementing all crucial features:
    - Core API integration for genome state management
    - Stream configuration support  
    - Enhanced client tracking with heartbeat timeouts
    - Automatic FQ sampler control based on subscriber presence
    - Standby mode when genome not loaded
    
    Uses a reliable threading-based approach instead of async for RTOS compatibility
    and to avoid async/sync context conflicts that plagued previous implementations.
    """
    
    def __init__(
        self, 
        host: str = "*", 
        port: int = 5562,
        context: Optional[zmq.Context] = None,
        fq_sampler: Optional[Any] = None,
        fq_sampler_queue: Optional[Any] = None,
        stream_config: Optional[Dict[str, Any]] = None,
        core_api: Optional[Any] = None  # Made optional and moved to end for compatibility
    ):
        """Initialize the primary FEAGI visualization stream."""
        # Core API integration (crucial for genome state management) - optional for compatibility
        self.core_api = core_api
        
        # Basic connection settings
        self.host = host
        self.port = port
        self.running = False
        
        # ALWAYS create a NEW sync context - NEVER use shared contexts that might be async
        self.context = zmq.Context()
        
        # FQ Sampler integration
        self.fq_sampler = fq_sampler
        self.fq_sampler_queue = fq_sampler_queue
        
        # Stream configuration
        self.stream_config = stream_config or {}
        self.auto_enable_on_subscribers = self.stream_config.get('auto_enable_on_subscribers', True)
        self.subscriber_check_interval = self.stream_config.get('subscriber_check_interval', 1.0)
        
        # Genome state management (crucial feature from full version)
        self._active_mode = False  # True when genome is loaded and ready
        
        # Enhanced client tracking (crucial feature from full version)
        self.client_last_heartbeat = {}  # Mapping of client_id -> last heartbeat time
        self.client_heartbeat_timeout = 30  # Consider clients disconnected after 30s
        self._client_lock = threading.Lock()  # Thread-safe client access
        
        # Subscriber monitoring for automatic FQ sampler control
        self._subscriber_count = 0
        self._last_subscriber_count = 0
        self._fq_sampler_enabled = False
        
        # Simple socket setup
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
        
        # Simple statistics
        self.stats = {
            'data_sent': 0,
            'bytes_sent': 0,
            'start_time': time.time()
        }

    def _setup_socket(self) -> None:
        """Set up the ZMQ PUB socket with optimal settings."""
        # Create a synchronous PUB socket 
        self.socket = self.context.socket(zmq.PUB)
        
        # Optimize for real-time streaming but allow queuing when no subscribers
        self.socket.setsockopt(zmq.SNDHWM, 1000)   # Higher send buffer to prevent drops
        self.socket.setsockopt(zmq.LINGER, 1000)   # Wait briefly on close to send pending messages
        
        bind_addr = f"tcp://{self.host}:{self.port}"
        self.socket.bind(bind_addr)
        logger.info(f"📡 Visualization stream bound to {bind_addr}")

    def _update_active_mode(self):
        """Update active mode based on genome availability (crucial feature from full version)."""
        old_mode = self._active_mode
        
        try:
            if self.core_api:
                self._active_mode = self.core_api.genome_is_loaded()
            else:
                # If no core_api, assume active mode for backward compatibility
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
        """Handle genome state changes (crucial feature from full version)."""
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
        """Start the enhanced visualization stream with multiple worker threads."""
        if self.running:
            return
            
        logger.info("🚀 Starting visualization stream")
        self.running = True
        self._stop_event.clear()
        
        # Start multiple worker threads for different responsibilities
        
        # Thread 1: FQ data processing (main data processing)
        if self.fq_sampler_queue:
            fq_thread = threading.Thread(
                target=self._data_worker,
                name="VisualizationFQ",
                daemon=True
            )
            fq_thread.start()
            self.worker_threads.append(fq_thread)
            logger.debug("FQ data processing thread started")
        else:
            logger.warning("No FQ sampler queue provided - no data will be processed")
            
        # Thread 2: Client cleanup and monitoring (crucial feature from full version)
        cleanup_thread = threading.Thread(
            target=self._client_cleanup_worker,
            name="VisualizationCleanup", 
            daemon=True
        )
        cleanup_thread.start()
        self.worker_threads.append(cleanup_thread)
        logger.debug("Client cleanup thread started")
        
        # Thread 3: Subscriber monitoring for automatic FQ sampler control (crucial feature)
        if self.auto_enable_on_subscribers:
            monitor_thread = threading.Thread(
                target=self._subscriber_monitor_worker,
                name="VisualizationMonitor",
                daemon=True
            )
            monitor_thread.start()
            self.worker_threads.append(monitor_thread)
            logger.debug("Subscriber monitoring thread started")
            
        logger.info(f"✅ Visualization stream started with {len(self.worker_threads)} worker threads")

    def stop(self) -> None:
        """Stop the visualization stream gracefully."""
        if not self.running:
            return
            
        logger.info("🛑 Stopping visualization stream...")
        self.running = False
        self._stop_event.set()
        
        # Wait for worker threads with individual monitoring
        total_threads = len(self.worker_threads)
        if total_threads > 0:
            logger.debug(f"Waiting for {total_threads} worker threads to stop...")
            
            for i, thread in enumerate(self.worker_threads, 1):
                if thread.is_alive():
                    logger.debug(f"Waiting for thread {i}/{total_threads}: {thread.name}")
                    thread.join(timeout=5.0)
                    
                    if thread.is_alive():
                        logger.warning(f"Thread {thread.name} didn't stop after 5 seconds")
                    else:
                        logger.debug(f"Thread {thread.name} stopped gracefully")
        
        # Close socket AFTER all threads have stopped
        if self.socket:
            logger.debug("Closing ZMQ socket...")
            self.socket.close(linger=0)  # Don't wait for pending messages
            self.socket = None
        
        # Clear worker thread list
        self.worker_threads.clear()
            
        logger.info("✅ Visualization stream stopped")

    def _data_worker(self) -> None:
        """
        Enhanced data processing worker with standby mode support.
        Includes crucial features from full version while keeping threading approach.
        """
        logger.debug("Visualization data worker started")
        
        while self.running and not self._stop_event.is_set():
            try:
                # Quick exit check for shutdown
                if self._stop_event.is_set():
                    logger.debug("Data worker received stop signal")
                    break
                
                # Skip processing if in standby mode (crucial feature from full version)
                if not self._active_mode:
                    # Update genome state and continue
                    self._update_active_mode()
                    if not self._active_mode:
                        # Use wait() instead of sleep() for faster shutdown response
                        if self._stop_event.wait(timeout=0.1):  # Brief pause in standby mode
                            logger.debug("Data worker stopping during standby mode")
                            break
                        continue
                
                # Try to get data from queue (non-blocking with shorter timeout)
                fq_data = None
                try:
                    if hasattr(self.fq_sampler_queue, 'get'):
                        # Reduced timeout from 0.1 to 0.05 for faster shutdown response
                        fq_data = self.fq_sampler_queue.get(timeout=0.05)
                    elif hasattr(self.fq_sampler_queue, '_queue') and len(self.fq_sampler_queue._queue) > 0:
                        fq_data = self.fq_sampler_queue._queue.pop(0)
                except Empty:
                    # Check stop signal more frequently during empty queue periods
                    if self._stop_event.is_set():
                        logger.debug("Data worker stopping during empty queue")
                        break
                        continue
                except Exception as e:
                    logger.debug(f"Queue access error: {e}")
                    # Use wait() instead of sleep() for faster shutdown response
                    if self._stop_event.wait(timeout=0.01):  # Brief pause on error
                        logger.debug("Data worker stopping after queue error")
                        break
                    continue 
                
                if fq_data is None:
                    continue
                    
                # Additional stop check before processing data
                if self._stop_event.is_set():
                    logger.debug("Data worker stopping before data processing")
                    break
                
                # Process and send data based on type (enhanced processing like full version)
                if isinstance(fq_data, bytes):
                    # Already serialized data
                    self._publish_data(fq_data)
                    
                elif isinstance(fq_data, dict) and 'target' in fq_data:
                    # Tagged format from enhanced FQ sampler (crucial feature from full version)
                    if fq_data.get('target') == 'visualization':
                        if 'cortical_id' in fq_data and 'fire_queue_data' in fq_data:
                            # Area-specific data
                            cortical_id = fq_data['cortical_id']
                            fire_queue_data = fq_data['fire_queue_data']
                            self._process_tuple_data((cortical_id, fire_queue_data))
                        elif 'fire_queue_data' in fq_data:
                            # Global data
                            fire_queue_data = fq_data['fire_queue_data']
                            self._process_dict_data(fire_queue_data)
                        elif 'data' in fq_data:
                            # Pre-encoded data
                            data = fq_data.get('data')
                            if isinstance(data, bytes):
                                self._publish_data(data)
                
                elif isinstance(fq_data, tuple) and len(fq_data) == 2:
                    # Legacy (cortical_id, fire_data) tuple format
                    logger.debug(f"Processing neural data for: {fq_data[0]}")
                    self._process_tuple_data(fq_data)
                
                elif isinstance(fq_data, dict):
                    # Legacy fire queue dict format
                    self._process_dict_data(fq_data)
                
                elif isinstance(fq_data, str) and fq_data == "STOP":
                    logger.info("Received STOP signal")
                    break 
                
                else:
                    logger.debug(f"Ignoring unsupported data type: {type(fq_data)}")
                
            except Exception as e:
                logger.error(f"Error in data worker: {e}")
                # Use wait() instead of sleep() for faster shutdown response
                if self._stop_event.wait(timeout=0.1):  # Brief pause on error
                    logger.debug("Data worker stopping after error")
                break 
                
        logger.debug("Visualization data worker stopped")

    def _publish_data(self, data: bytes) -> None:
        """
        Publish data on the 'activity' topic.
        Use synchronous ZMQ operations with error handling.
        """
        try:
            # Use basic synchronous send operations
            self.socket.send(b"activity", zmq.SNDMORE)
            self.socket.send(data)
            
            # Update statistics
            self.stats['data_sent'] += 1
            self.stats['bytes_sent'] += len(data)
            
            # Periodic status logging (every 100 messages)
            if self.stats['data_sent'] % 100 == 0:
                logger.debug(f"Published {self.stats['data_sent']} messages, {self.stats['bytes_sent']} bytes total")
            
            # Log first few messages to confirm publishing is working
            if self.stats['data_sent'] <= 3:
                logger.info(f"Successfully published message #{self.stats['data_sent']} ({len(data)} bytes)")
            
        except Exception as e:
            logger.error(f"Failed to publish data: {e}")
            
            # Handle ZMQ state corruption specifically
            if "Operation cannot be accomplished in current state" in str(e):
                logger.warning("ZMQ socket corrupted, attempting recreation...")
                try:
                    self._recreate_socket()
                    # Retry once
                    self.socket.send(b"activity", zmq.SNDMORE)
                    self.socket.send(data)
                    logger.info("Socket recreated and retry successful")
                except Exception as retry_error:
                    logger.error(f"Socket recreation failed: {retry_error}")
            else:
                # Log error details for non-corruption errors
                logger.error(f"Publishing error type: {type(e).__name__}")
                if logger.isEnabledFor(10):  # DEBUG level
                    import traceback
                    logger.debug(f"Publishing traceback: {traceback.format_exc()}")

    def _recreate_socket(self):
        """Recreate the ZMQ socket when it gets corrupted."""
        logger.warning("Recreating corrupted ZMQ socket...")
        
        # Close old socket if it exists
        if self.socket:
            try:
                self.socket.close(linger=0)
                logger.debug("Old socket closed")
            except Exception as e:
                logger.error(f"Error closing old socket: {e}")
        
        # Recreate socket with same settings
        try:
            self.socket = self.context.socket(zmq.PUB)
            self.socket.setsockopt(zmq.SNDHWM, 1000)   
            self.socket.setsockopt(zmq.LINGER, 1000)   
            
            bind_addr = f"tcp://{self.host}:{self.port}"
            self.socket.bind(bind_addr)
            logger.info(f"Socket recreated and bound to {bind_addr}")
            
        except Exception as e:
            logger.error(f"Failed to recreate socket: {e}")
            raise

    def _process_tuple_data(self, fq_data) -> None:
        """Process legacy tuple format data."""
        try:
            cortical_id, fire_data = fq_data
            
            if fire_data and 'neuron_ids' in fire_data:
                neuron_ids = fire_data.get('neuron_ids', [])
                neuron_count = len(neuron_ids)
                
                # Handle coordinates
                coordinates = fire_data.get('coordinates', [])
                x_coords = []
                y_coords = []
                z_coords = []
                
                if isinstance(coordinates, list) and len(coordinates) > 0:
                    # If coordinates is a list of [x, y, z] triplets
                    if isinstance(coordinates[0], (list, tuple)) and len(coordinates[0]) >= 3:
                        x_coords = [coord[0] for coord in coordinates]
                        y_coords = [coord[1] for coord in coordinates]  
                        z_coords = [coord[2] for coord in coordinates]
                    else:
                        # If coordinates is a flat list, assume it's organized as [x1,y1,z1,x2,y2,z2,...]
                        coords_per_neuron = 3
                        x_coords = coordinates[0::coords_per_neuron]
                        y_coords = coordinates[1::coords_per_neuron]
                        z_coords = coordinates[2::coords_per_neuron]
                elif isinstance(coordinates, dict):
                    # If coordinates is a dict with x, y, z keys
                    x_coords = coordinates.get('x', [])
                    y_coords = coordinates.get('y', [])
                    z_coords = coordinates.get('z', [])
                else:
                    logger.error(f"Missing coordinates for {cortical_id}")
                    return
                
                # Handle membrane potentials
                membrane_potentials = fire_data.get('membrane_potentials', [])
                
                if not membrane_potentials:
                    logger.error(f"Missing membrane potentials for {cortical_id}")
                    return
                
                if len(membrane_potentials) != neuron_count:
                    logger.error(f"Membrane potential count mismatch for {cortical_id}: {len(membrane_potentials)} vs {neuron_count}")
                    return
                
                # Validate coordinate arrays
                if len(x_coords) != neuron_count or len(y_coords) != neuron_count or len(z_coords) != neuron_count:
                    logger.error(f"Coordinate count mismatch for {cortical_id}: x={len(x_coords)}, y={len(y_coords)}, z={len(z_coords)}, neurons={neuron_count}")
                    return
                
                # Create cortical IDs list (same cortical ID for all neurons)
                cortical_ids = [cortical_id] * neuron_count
                
                # Encode using feagi_bytes binary format
                try:
                    from feagi_bytes import ByteStructureEncoder
                    encoder = ByteStructureEncoder()
                            
                    binary_data = encoder.encode_neuron_flat(
                        cortical_ids=cortical_ids,
                        x_coords=x_coords,
                        y_coords=y_coords,
                        z_coords=z_coords,
                        potentials=membrane_potentials
                    )
                    
                    # Publish the binary data
                    self._publish_data(binary_data)
                    logger.debug(f"Published {cortical_id}: {neuron_count} neurons, {len(binary_data)} bytes")
                    
                except ImportError:
                    logger.error("feagi_bytes library not available - cannot encode binary data")
                except Exception as e:
                    logger.error(f"Error encoding binary data: {e}")
            else:
                logger.error(f"Invalid fire_data for {cortical_id} - missing neuron_ids")
                
        except Exception as e:
            logger.error(f"Error processing {fq_data[0] if len(fq_data) > 0 else 'unknown'}: {e}")
            if logger.isEnabledFor(10):  # DEBUG level
                import traceback
                logger.debug(f"Processing traceback: {traceback.format_exc()}")

    def _process_dict_data(self, fire_data) -> None:
        """Process legacy dict format data."""
        try:
            if fire_data and 'neuron_ids' in fire_data:
                neuron_count = len(fire_data.get('neuron_ids', []))
                logger.debug(f"Received dict data: {neuron_count} neurons")
                # TODO: Implement proper binary serialization
                
        except Exception as e:
            logger.error(f"Error processing dict data: {e}")

    def get_stats(self) -> Dict[str, Any]:
        """Get simple statistics."""
        runtime = time.time() - self.stats['start_time']
        return {
            'running': self.running,
            'data_sent': self.stats['data_sent'],
            'bytes_sent': self.stats['bytes_sent'],
            'runtime_seconds': runtime,
            'messages_per_second': self.stats['data_sent'] / max(runtime, 1)
        }

    # COMPATIBILITY METHODS (so existing code doesn't break)
    
    def register_visualization_client(self, client_id: str) -> None:
        """Compatibility method - does nothing in simple mode."""
        logger.debug(f"Simple mode: ignoring client registration for {client_id}")
        
    def unregister_visualization_client(self, client_id: str) -> None:
        """Compatibility method - does nothing in simple mode."""
        logger.debug(f"Simple mode: ignoring client unregistration for {client_id}")
        
    def heartbeat_visualization_client(self, client_id: str) -> None:
        """Enhanced heartbeat method with proper client tracking."""
        # Track client heartbeat with proper threading (crucial feature from full version)
        current_time = time.time()
        
        with self._client_lock:  # Thread-safe access
            # Check if this is a new client (no previous heartbeat recorded)
            is_new_client = client_id not in self.client_last_heartbeat
            
            self.client_last_heartbeat[client_id] = current_time
            total_clients = len(self.client_last_heartbeat)
            
            # If this is a new client, log it and update FQ sampler
            if is_new_client:
                logger.info(f"📺 New visualization client connected: {client_id}")
                
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
        """Get the number of connected visualization clients (crucial feature from full version)."""
        with self._client_lock:  # Thread-safe access
            return len(self.client_last_heartbeat)

    def _control_fq_sampler(self, enable: bool) -> None:
        """Enable or disable the FQ sampler based on subscriber presence (crucial feature from full version)."""
        try:
            if not self.fq_sampler:
                # Try to get FQ sampler from process manager
                try:
                    from feagi.process_manager import get_process_manager
                    process_manager = get_process_manager()
                    
                    if process_manager and hasattr(process_manager, '_fq_sampler'):
                        self.fq_sampler = process_manager._fq_sampler
                        logger.debug(f"Found FQ sampler from process manager")
                    else:
                        logger.debug("Process manager has no _fq_sampler attribute")
                        
                except Exception as e:
                    logger.debug(f"Could not get FQ sampler from process manager: {e}")
            
            if self.fq_sampler and hasattr(self.fq_sampler, 'set_visualization_subscribers'):
                if enable:
                    logger.info("🔔 Enabling FQ sampler - visualization clients connected")
                    self.fq_sampler.set_visualization_subscribers(True)
                    self._fq_sampler_enabled = True
                else:
                    logger.info("🔕 Disabling FQ sampler - no visualization clients")
                    self.fq_sampler.set_visualization_subscribers(False)
                    self._fq_sampler_enabled = False
            else:
                if enable:
                    logger.warning("FQ sampler not available or doesn't support set_visualization_subscribers")
                
        except Exception as e:
            logger.error(f"Error controlling FQ sampler: {e}")
            if logger.isEnabledFor(10):  # DEBUG level  
                import traceback
                logger.debug(f"FQ sampler control traceback: {traceback.format_exc()}")

    def send_visualization_data(self, data) -> None:
        """Compatibility method for external data sending."""
        if isinstance(data, bytes):
            self._publish_data(data)

    def _client_cleanup_worker(self) -> None:
        """
        Client cleanup worker thread (crucial feature from full version).
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
        Subscriber monitoring worker thread (crucial feature from full version).
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
                # Use responsive wait on error to be more responsive to shutdown
                for _ in range(int(min(self.subscriber_check_interval, 1.0) * 5)):  # Check every 200ms
                    if self._stop_event.wait(timeout=0.2):
                        return
        
        logger.debug("Subscriber monitoring stopped") 