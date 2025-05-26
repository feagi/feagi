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
        # Create a PURELY SYNCHRONOUS PUB socket 
        self.socket = self.context.socket(zmq.PUB)
        
        # Ensure this is synchronous by checking the socket type
        logger.info(f"🔧 Created ZMQ socket type: {type(self.socket)}")
        
        # Optimize for real-time streaming but ALLOW queuing when no subscribers
        self.socket.setsockopt(zmq.SNDHWM, 1000)   # Higher send buffer to prevent drops
        self.socket.setsockopt(zmq.LINGER, 1000)   # Wait briefly on close to send pending messages
        # REMOVED: self.socket.setsockopt(zmq.IMMEDIATE, 1) - This drops messages when no subscribers!
        
        bind_addr = f"tcp://{self.host}:{self.port}"
        self.socket.bind(bind_addr)
        logger.info(f"📡 Primary visualization stream bound to {bind_addr}")
        logger.info(f"🔧 Socket will queue messages even when no subscribers are connected")

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
                    logger.info("SimpleVisualizationStream entering ACTIVE mode")
            else:
                self._active_mode = False 
                if self.running:
                    logger.info("SimpleVisualizationStream entering STANDBY mode")
        except Exception as e:
            logger.error(f"Error handling genome state change: {e}")
            self._active_mode = False

    def start(self) -> None:
        """Start the enhanced visualization stream with multiple worker threads."""
        if self.running:
            return
            
        logger.info("🚀 Starting primary visualization stream")
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
            logger.info("✅ FQ data processing thread started")
        else:
            logger.warning("⚠️ No FQ sampler queue provided - no data will be processed")
            
        # Thread 2: Client cleanup and monitoring (crucial feature from full version)
        cleanup_thread = threading.Thread(
            target=self._client_cleanup_worker,
            name="VisualizationCleanup", 
            daemon=True
        )
        cleanup_thread.start()
        self.worker_threads.append(cleanup_thread)
        logger.info("✅ Client cleanup thread started")
        
        # Thread 3: Subscriber monitoring for automatic FQ sampler control (crucial feature)
        if self.auto_enable_on_subscribers:
            monitor_thread = threading.Thread(
                target=self._subscriber_monitor_worker,
                name="VisualizationMonitor",
                daemon=True
            )
            monitor_thread.start()
            self.worker_threads.append(monitor_thread)
            logger.info("✅ Subscriber monitoring thread started")
            
        logger.info(f"✅ Primary visualization stream started with {len(self.worker_threads)} worker threads")

    def stop(self) -> None:
        """Stop the visualization stream gracefully."""
        if not self.running:
            return
            
        logger.info("🛑 Stopping primary visualization stream...")
        self.running = False
        self._stop_event.set()
        
        # Wait for worker threads with longer timeout and individual monitoring
        total_threads = len(self.worker_threads)
        logger.info(f"⏳ Waiting for {total_threads} worker threads to stop...")
        
        for i, thread in enumerate(self.worker_threads, 1):
            if thread.is_alive():
                logger.info(f"⏳ Waiting for thread {i}/{total_threads}: {thread.name}")
                thread.join(timeout=5.0)  # Increased from 2.0 to 5.0 seconds
                
                if thread.is_alive():
                    logger.warning(f"⚠️ Thread {thread.name} didn't stop after 5 seconds, forcing shutdown")
                else:
                    logger.info(f"✅ Thread {thread.name} stopped gracefully")
        
        # Close socket AFTER all threads have stopped
        if self.socket:
            logger.info("🔌 Closing ZMQ socket...")
            self.socket.close(linger=0)  # Don't wait for pending messages
            self.socket = None
            logger.info("✅ ZMQ socket closed")
        
        # Clear worker thread list
        self.worker_threads.clear()
            
        logger.info("✅ Primary visualization stream stopped completely")

    def _data_worker(self) -> None:
        """
        Enhanced data processing worker with standby mode support.
        Includes crucial features from full version while keeping threading approach.
        """
        logger.info("🎬 Primary visualization data worker started")
        
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
                    logger.info(f"📤 Processing neural data for: {fq_data[0]}")
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
                logger.error(f"Error in enhanced data worker: {e}")
                # Use wait() instead of sleep() for faster shutdown response
                if self._stop_event.wait(timeout=0.1):  # Brief pause on error
                    logger.debug("Data worker stopping after error")
                    break
                
        logger.info("🛑 Primary visualization data worker stopped")

    def _publish_data(self, data: bytes) -> None:
        """
        Publish data on the 'activity' topic.
        Use the most basic synchronous ZMQ operations with comprehensive debugging.
        """
        try:
            # 🔧 COMPREHENSIVE SOCKET STATE DEBUGGING
            logger.info(f"🔧 DEBUG: ===== SOCKET STATE BEFORE PUBLISH =====")
            logger.info(f"🔧 DEBUG: Socket exists: {self.socket is not None}")
            
            if self.socket:
                logger.info(f"🔧 DEBUG: Socket type: {type(self.socket)}")
                logger.info(f"🔧 DEBUG: Socket closed: {self.socket.closed}")
                logger.info(f"🔧 DEBUG: Socket hwm: {self.socket.getsockopt(zmq.SNDHWM)}")
                logger.info(f"🔧 DEBUG: Socket linger: {self.socket.getsockopt(zmq.LINGER)}")
                logger.info(f"🔧 DEBUG: Socket identity: {self.socket.getsockopt(zmq.IDENTITY)}")
                
                try:
                    events = self.socket.getsockopt(zmq.EVENTS)
                    logger.info(f"🔧 DEBUG: Socket events: {events} (POLLIN={zmq.POLLIN}, POLLOUT={zmq.POLLOUT})")
                    logger.info(f"🔧 DEBUG: Socket can send: {(events & zmq.POLLOUT) != 0}")
                except Exception as e:
                    logger.error(f"🔧 DEBUG: Error checking socket events: {e}")
                
                try:
                    last_endpoint = self.socket.getsockopt(zmq.LAST_ENDPOINT)
                    logger.info(f"🔧 DEBUG: Last endpoint: {last_endpoint}")
                except Exception as e:
                    logger.error(f"🔧 DEBUG: Error getting last endpoint: {e}")
                    
            logger.info(f"🔧 DEBUG: Context exists: {self.context is not None}")
            if self.context:
                logger.info(f"🔧 DEBUG: Context closed: {self.context.closed}")
            
            logger.info(f"🔧 DEBUG: Data size: {len(data)} bytes")
            logger.info(f"🔧 DEBUG: Data preview: {data[:100] if len(data) > 100 else data}")
            
            # Use the most basic synchronous send operations
            logger.info(f"🔧 DEBUG: Sending topic frame...")
            self.socket.send(b"activity", zmq.SNDMORE)
            logger.info(f"🔧 DEBUG: Topic frame sent successfully")
            
            logger.info(f"🔧 DEBUG: Sending data frame...")
            self.socket.send(data)
            logger.info(f"🔧 DEBUG: Data frame sent successfully")
            
            # Update simple statistics
            self.stats['data_sent'] += 1
            self.stats['bytes_sent'] += len(data)
            
            # More frequent debug logging to verify publishing
            if self.stats['data_sent'] % 10 == 0:  # Every 10 messages instead of 100
                logger.info(f"📊 Published {self.stats['data_sent']} messages, {self.stats['bytes_sent']} bytes total")
            
            # Log first few messages to confirm publishing is working
            if self.stats['data_sent'] <= 5:
                logger.info(f"🚀 Successfully published message #{self.stats['data_sent']} ({len(data)} bytes)")
            
            logger.info(f"🔧 DEBUG: ===== SOCKET STATE AFTER PUBLISH =====")
            if self.socket:
                logger.info(f"🔧 DEBUG: Socket still exists: True")
                logger.info(f"🔧 DEBUG: Socket still open: {not self.socket.closed}")
            else:
                logger.info(f"🔧 DEBUG: Socket is None after publish!")
            
        except Exception as e:
            logger.error(f"❌ SOCKET ERROR: Failed to publish data: {e}")
            logger.error(f"❌ ERROR TYPE: {type(e)}")
            
            # Enhanced socket error diagnostics
            if self.socket:
                try:
                    logger.error(f"❌ Socket state after error: closed={self.socket.closed}")
                    logger.error(f"❌ Socket type after error: {type(self.socket)}")
                except Exception as diag_error:
                    logger.error(f"❌ Cannot diagnose socket: {diag_error}")
            else:
                logger.error(f"❌ Socket is None when error occurred")
                
            if self.context:
                try:
                    logger.error(f"❌ Context state after error: closed={self.context.closed}")
                except Exception as diag_error:
                    logger.error(f"❌ Cannot diagnose context: {diag_error}")
            else:
                logger.error(f"❌ Context is None when error occurred")
                
            import traceback
            logger.error(f"❌ Publishing traceback: {traceback.format_exc()}")
            
            # Try to recreate socket if it's corrupted
            if "Operation cannot be accomplished in current state" in str(e):
                logger.error(f"❌ DETECTED ZMQ STATE CORRUPTION - attempting socket recreation")
                try:
                    self._recreate_socket()
                    logger.info(f"✅ Socket recreated successfully - retrying publish")
                    # Retry once
                    self.socket.send(b"activity", zmq.SNDMORE)
                    self.socket.send(data)
                    logger.info(f"✅ Retry publish successful")
                except Exception as retry_error:
                    logger.error(f"❌ Retry publish failed: {retry_error}")
                    
    def _recreate_socket(self):
        """Recreate the ZMQ socket when it gets corrupted."""
        logger.warning(f"🔧 Recreating corrupted ZMQ socket...")
        
        # Close old socket if it exists
        if self.socket:
            try:
                self.socket.close(linger=0)
                logger.info(f"🔧 Old socket closed")
            except Exception as e:
                logger.error(f"🔧 Error closing old socket: {e}")
        
        # Recreate socket with same settings
        try:
            self.socket = self.context.socket(zmq.PUB)
            self.socket.setsockopt(zmq.SNDHWM, 1000)   
            self.socket.setsockopt(zmq.LINGER, 1000)   
            
            bind_addr = f"tcp://{self.host}:{self.port}"
            self.socket.bind(bind_addr)
            logger.info(f"🔧 Socket recreated and bound to {bind_addr}")
            
        except Exception as e:
            logger.error(f"🔧 Failed to recreate socket: {e}")
            raise

    def _process_tuple_data(self, fq_data) -> None:
        """Process legacy tuple format data."""
        try:
            cortical_id, fire_data = fq_data
            
            # LOG RAW DATA FOR DEBUGGING
            logger.info(f"🔬 RAW FIRE_DATA for {cortical_id}:")
            logger.info(f"   📋 Keys: {list(fire_data.keys()) if fire_data else 'None'}")
            if fire_data:
                for key, value in fire_data.items():
                    if isinstance(value, list):
                        logger.info(f"   🔑 {key}: {len(value)} items - FULL DATA: {value}")
                    else:
                        logger.info(f"   🔑 {key}: {value}")
            
            if fire_data and 'neuron_ids' in fire_data:
                neuron_ids = fire_data.get('neuron_ids', [])
                neuron_count = len(neuron_ids)
                
                # Handle coordinates - NO FALLBACKS, fail if missing
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
                        logger.info(f"✅ Using provided coordinates (triplet format)")
                    else:
                        # If coordinates is a flat list, assume it's organized as [x1,y1,z1,x2,y2,z2,...]
                        coords_per_neuron = 3
                        x_coords = coordinates[0::coords_per_neuron]
                        y_coords = coordinates[1::coords_per_neuron]
                        z_coords = coordinates[2::coords_per_neuron]
                        logger.info(f"✅ Using provided coordinates (flat format)")
                elif isinstance(coordinates, dict):
                    # If coordinates is a dict with x, y, z keys
                    x_coords = coordinates.get('x', [])
                    y_coords = coordinates.get('y', [])
                    z_coords = coordinates.get('z', [])
                    logger.info(f"✅ Using provided coordinates (dict format)")
                else:
                    # NO FALLBACK - FAIL if coordinates are missing
                    logger.error(f"❌ MISSING COORDINATES for {cortical_id} - fire_data has no valid coordinates!")
                    logger.error(f"❌ Coordinates field: {coordinates}")
                    return
                
                # Handle membrane potentials - NO FALLBACKS, fail if missing
                membrane_potentials = fire_data.get('membrane_potentials', [])
                
                if not membrane_potentials:
                    logger.error(f"❌ MISSING MEMBRANE POTENTIALS for {cortical_id} - fire_data has no membrane_potentials field!")
                    return
                
                if len(membrane_potentials) != neuron_count:
                    logger.error(f"❌ MEMBRANE POTENTIAL COUNT MISMATCH for {cortical_id}:")
                    logger.error(f"   🧠 Neuron count: {neuron_count}")
                    logger.error(f"   ⚡ Membrane potential count: {len(membrane_potentials)}")
                    return
                
                # Validate coordinate arrays
                if len(x_coords) != neuron_count or len(y_coords) != neuron_count or len(z_coords) != neuron_count:
                    logger.error(f"❌ COORDINATE COUNT MISMATCH for {cortical_id}:")
                    logger.error(f"   🧠 Neuron count: {neuron_count}")
                    logger.error(f"   📍 X coords: {len(x_coords)}")
                    logger.error(f"   📍 Y coords: {len(y_coords)}")
                    logger.error(f"   📍 Z coords: {len(z_coords)}")
                    return
                
                # LOG FINAL DATA BEFORE ENCODING - SHOW ALL DATA
                logger.info(f"🎯 VALIDATED DATA for {cortical_id} ({neuron_count} neurons):")
                logger.info(f"   🔢 ALL Neuron IDs: {neuron_ids}")
                logger.info(f"   📍 ALL X coords: {x_coords}")
                logger.info(f"   📍 ALL Y coords: {y_coords}")
                logger.info(f"   📍 ALL Z coords: {z_coords}")
                logger.info(f"   ⚡ ALL Potentials: {membrane_potentials}")
                
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
                    logger.info(f"✅ Published {cortical_id}: {neuron_count} neurons, {len(binary_data)} bytes (BINARY)")
                    
                except ImportError:
                    logger.error("❌ feagi_bytes library not available - cannot encode binary data")
                except Exception as e:
                    logger.error(f"❌ Error encoding binary data: {e}")
            else:
                logger.error(f"❌ INVALID FIRE_DATA for {cortical_id} - missing neuron_ids or fire_data is None")
                
        except Exception as e:
            logger.error(f"❌ Error processing {fq_data[0] if len(fq_data) > 0 else 'unknown'}: {e}")
            import traceback
            logger.error(f"❌ Traceback: {traceback.format_exc()}")

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
        logger.info(f"🔧 DEBUG: _control_fq_sampler called with enable={enable}")
        
        try:
            if not self.fq_sampler:
                logger.info(f"🔧 DEBUG: No FQ sampler reference, trying to get from process manager")
                # Try to get FQ sampler from process manager
                try:
                    from feagi.process_manager import get_process_manager
                    process_manager = get_process_manager()
                    logger.info(f"🔧 DEBUG: Process manager instance: {process_manager is not None}")
                    
                    if process_manager and hasattr(process_manager, '_fq_sampler'):
                        self.fq_sampler = process_manager._fq_sampler
                        logger.info(f"✅ Found FQ sampler from process manager: {self.fq_sampler is not None}")
                    else:
                        logger.warning(f"❌ Process manager has no _fq_sampler attribute")
                        
                except Exception as e:
                    logger.warning(f"Could not get FQ sampler from process manager: {e}")
            else:
                logger.info(f"🔧 DEBUG: Using existing FQ sampler reference")
            
            logger.info(f"🔧 DEBUG: FQ sampler available: {self.fq_sampler is not None}")
            logger.info(f"🔧 DEBUG: FQ sampler has set_visualization_subscribers: {hasattr(self.fq_sampler, 'set_visualization_subscribers') if self.fq_sampler else False}")
            
            if self.fq_sampler and hasattr(self.fq_sampler, 'set_visualization_subscribers'):
                if enable:
                    logger.info("🔔 Enabling FQ sampler - visualization clients connected")
                    self.fq_sampler.set_visualization_subscribers(True)
                    self._fq_sampler_enabled = True
                    logger.info("✅ FQ sampler enabled successfully")
                else:
                    logger.info("🔕 Disabling FQ sampler - no visualization clients")
                    self.fq_sampler.set_visualization_subscribers(False)
                    self._fq_sampler_enabled = False
                    logger.info("✅ FQ sampler disabled successfully")
            else:
                if enable:
                    logger.warning("❌ FQ sampler not available or doesn't support set_visualization_subscribers")
                
        except Exception as e:
            logger.error(f"❌ Error controlling FQ sampler: {e}")
            import traceback
            logger.error(f"❌ Traceback: {traceback.format_exc()}")

    def send_visualization_data(self, data) -> None:
        """Compatibility method for external data sending."""
        if isinstance(data, bytes):
            self._publish_data(data)

    def _client_cleanup_worker(self) -> None:
        """
        Client cleanup worker thread (crucial feature from full version).
        Manages client heartbeat timeouts and automatic cleanup.
        """
        cleanup_interval = 5.0  # Reduced from 10.0 to 5.0 seconds for faster shutdown
        
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
        
        logger.info("🛑 Client cleanup worker stopped gracefully")

    def _subscriber_monitor_worker(self) -> None:
        """
        Subscriber monitoring worker thread (crucial feature from full version).
        Automatically enables/disables FQ sampler based on subscriber presence.
        """
        logger.info("Starting subscriber monitoring for automatic FQ sampler control")
        
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
        
        logger.info("🛑 Subscriber monitoring stopped gracefully") 