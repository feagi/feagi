"""
ZeroMQ Visualization Stream Implementation for FEAGI API

This module implements a specialized streaming pattern for visualization data.
It provides:
- One-directional flow from FEAGI to agents for neuron activity data
- Efficient binary serialization using feagi_bytes
- Genome-dependent state management (standby when no genome loaded)
- Real-time performance optimization with minimal buffering
- Automatic subscriber detection and FQ sampler control

RTOS COMPATIBILITY:
- Uses threading.Thread instead of async/await for deterministic timing
- Queue-based communication for thread-safe data passing
- Busy-wait loops instead of time.sleep() for precise timing
- Pre-allocated data structures to avoid dynamic memory allocation
"""

import time
import threading
from typing import Dict, Any, Optional, List, Callable
from queue import Queue, Empty
import zmq

from feagi.utils.logger import setup_logger
from ...core.services.core_api_service import CoreAPIService
from ...utils.rate_limit import RateLimiter
from feagi.core.state_manager import GenomeState

logger = setup_logger(__name__)


class VisualizationStream:
    """
    ZeroMQ Visualization Stream implementation.
    
    Uses a PUB socket for sending neural activity data (FEAGI → agents).
    Automatically adjusts to genome availability state.
    
    RTOS COMPATIBLE:
    - Uses threading.Thread instead of async tasks
    - Queue-based communication patterns
    - Deterministic timing with busy-wait loops
    - Pre-allocated data structures
    """
    
    def __init__(
        self, 
        core_api: CoreAPIService,
        host: str = "*", 
        port: int = 5560,
        context: Optional[zmq.Context] = None,
        fq_sampler: Optional[Any] = None,
        fq_sampler_queue: Optional[Any] = None,
        stream_config: Optional[Dict[str, Any]] = None
    ):
        """
        Initialize a new Visualization Stream.
        
        Args:
            core_api: The CoreAPIService instance
            host: Host address to bind to
            port: Port for visualization data
            context: Optional existing ZMQ context
            fq_sampler: Optional FQ sampler instance
            fq_sampler_queue: Queue for FQ data from the sampler
            stream_config: Stream configuration from TOML
        """
        self.core_api = core_api
        self.host = host
        self.port = port
        self.running = False
        self.context = context or zmq.Context.instance()
        
        # Stream configuration
        self.stream_config = stream_config or {}
        self.auto_enable_on_subscribers = self.stream_config.get('auto_enable_on_subscribers', True)
        self.subscriber_check_interval = self.stream_config.get('subscriber_check_interval', 1.0)
        
        # State tracking
        self._active_mode = False  # True when genome is loaded and ready
        
        # Connected clients tracking
        self.client_last_heartbeat = {}  # Mapping of client_id -> last heartbeat time
        self.client_heartbeat_timeout = 30  # Consider clients disconnected after 30s
        self._client_lock = threading.Lock()  # RTOS: Thread-safe client access
        
        # Subscriber monitoring
        self._subscriber_count = 0
        self._last_subscriber_count = 0
        self._fq_sampler_enabled = False
        
        # Socket for visualization data
        self.socket = self._setup_socket()
        
        # Rate limiter for throttling high-frequency data
        self.rate_limiter = RateLimiter()
        
        # FQ Sampler integration
        self.fq_sampler = fq_sampler
        self.fq_sampler_queue = fq_sampler_queue
        
        # RTOS: Thread management with pre-allocated worker threads
        self.worker_threads = []
        self._stop_event = threading.Event()  # RTOS: Event-based coordination
        
        # RTOS: Pre-allocated queues with fixed sizes
        self.control_queue = Queue(maxsize=100)  # For control messages
        self.data_queue = Queue(maxsize=50)      # For outbound data
        
        # Register for genome state change notifications
        if hasattr(core_api, 'register_genome_change_listener'):
            core_api.register_genome_change_listener(self._on_genome_state_change)
        
        # Initialize state based on current genome availability
        self._update_active_mode()

    def _setup_socket(self) -> zmq.Socket:
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

    def start(self) -> None:
        """Start the visualization stream server."""
        if self.running:
            return
            
        logger.info(f"Starting Visualization Stream server on {self.host}:{self.port}")
        self.running = True
        self._stop_event.clear()
        
        # RTOS: Start dedicated worker threads with specific responsibilities
        
        # Thread 1: FQ data processing
        if self.fq_sampler_queue:
            fq_thread = threading.Thread(
                target=self._fq_data_worker,
                name="VisualizationFQ",
                daemon=True
            )
            fq_thread.start()
            self.worker_threads.append(fq_thread)
            
        # Thread 2: Client cleanup and monitoring
        cleanup_thread = threading.Thread(
            target=self._client_cleanup_worker,
            name="VisualizationCleanup", 
            daemon=True
        )
        cleanup_thread.start()
        self.worker_threads.append(cleanup_thread)
        
        # Thread 3: Subscriber monitoring (if enabled)
        if self.auto_enable_on_subscribers:
            monitor_thread = threading.Thread(
                target=self._subscriber_monitor_worker,
                name="VisualizationMonitor",
                daemon=True
            )
            monitor_thread.start()
            self.worker_threads.append(monitor_thread)
            
        logger.info("Visualization Stream server started")

    def stop(self) -> None:
        """Stop the visualization stream server."""
        if not self.running:
            return
            
        logger.info("🛑 Stopping Visualization Stream server")
        self.running = False
        self._stop_event.set()
        
        # Wait for all worker threads to complete with more aggressive timeout handling
        threads_to_stop = self.worker_threads.copy()
        logger.debug(f"Stopping {len(threads_to_stop)} worker threads")
        
        for i, thread in enumerate(threads_to_stop):
            if thread.is_alive():
                logger.debug(f"Waiting for thread {i+1}/{len(threads_to_stop)}: {thread.name}")
                thread.join(timeout=2.0)
                
                if thread.is_alive():
                    logger.warning(f"Thread {thread.name} did not stop within timeout")
                else:
                    logger.debug(f"Thread {thread.name} stopped successfully")
        
        # Clear the thread list
        self.worker_threads.clear()
        
        # Close the socket with immediate termination
        if self.socket:
            try:
                self.socket.close()
                self.socket = None
                logger.debug("Visualization socket closed")
            except Exception as e:
                logger.warning(f"Error closing visualization socket: {e}")
            
        logger.info("✅ Visualization Stream server stopped")

    def _fq_data_worker(self) -> None:
        """
        RTOS-compatible worker thread for processing FQ data.
        Replaces async _process_fq_data() with deterministic thread.
        """
        if not self.fq_sampler_queue:
            logger.debug("No FQ sampler queue configured")
            return
            
        logger.info("🎬 DEBUG: VisualizationStream FQ data processing started")
        logger.info(f"🔧 DEBUG: Queue type: {type(self.fq_sampler_queue)}")
        logger.info(f"🔧 DEBUG: Queue available: {self.fq_sampler_queue is not None}")
        
        consecutive_errors = 0
        max_consecutive_errors = 10
        
        # RTOS: Fixed timing interval for deterministic behavior
        check_interval = 0.01
        
        while self.running and not self._stop_event.is_set():
            try:
                # Quick exit check for shutdown
                if self._stop_event.is_set():
                    logger.debug("FQ data worker received stop signal")
                    break
                
                # RTOS: Get FQ data from queue without blocking indefinitely
                try:
                    # Use get with timeout for non-blocking operation
                    fq_data = self.fq_sampler_queue.get(timeout=check_interval)
                    
                    if fq_data:
                        # Process the FQ data
                        self._process_fq_data_item(fq_data)
                        consecutive_errors = 0  # Reset error counter on success
                        
                        # Mark task as done
                        self.fq_sampler_queue.task_done()
                        
                except Empty:
                    # No data available, continue to next iteration
                    # This is normal and expected - don't count as error
                    continue
                    
            except Exception as e:
                consecutive_errors += 1
                logger.error(f"Error processing FQ data: {e}")
                
                if consecutive_errors >= max_consecutive_errors:
                    logger.error(f"Too many consecutive errors ({consecutive_errors}), stopping FQ worker")
                    break
                    
                # Small delay before retrying to avoid tight error loops
                self._deterministic_wait(0.1)
                
            # Quick check for shutdown between iterations
            if self._stop_event.is_set():
                logger.debug("FQ data worker stopping due to stop event")
                break
        
        logger.info("🛑 FQ data worker thread stopped gracefully")

    def _deterministic_wait(self, interval: float) -> None:
        """
        RTOS-compatible deterministic wait using busy-wait.
        Replaces time.sleep() for precise, deterministic timing.
        """
        target_end_time = time.perf_counter() + interval
        while time.perf_counter() < target_end_time and not self._stop_event.is_set():
            pass  # Busy-wait for deterministic timing

    def _client_cleanup_worker(self) -> None:
        """
        RTOS-compatible worker thread for client cleanup.
        Replaces async _cleanup_disconnected_clients().
        """
        cleanup_interval = 10.0  # 10 second cleanup cycle
        
        while self.running and not self._stop_event.is_set():
            try:
                # Quick exit check for shutdown
                if self._stop_event.is_set():
                    logger.debug("Client cleanup worker received stop signal")
                    break
                
                current_time = time.time()
                
                with self._client_lock:  # RTOS: Thread-safe client access
                    client_ids = list(self.client_last_heartbeat.keys())
                    
                    for client_id in client_ids:
                        last_heartbeat = self.client_last_heartbeat[client_id]
                        if current_time - last_heartbeat > self.client_heartbeat_timeout:
                            logger.info(f"Client {client_id} disconnected (timeout)")
                            del self.client_last_heartbeat[client_id]
                        
            except Exception as e:
                logger.error(f"Error cleaning up clients: {e}")
                
            # RTOS: Use deterministic wait with frequent stop event checks
            start_time = time.time()
            while (time.time() - start_time) < cleanup_interval:
                if self._stop_event.is_set():
                    logger.debug("Client cleanup worker stopping due to stop event")
                    return
                time.sleep(0.5)  # Check stop event every 500ms
        
        logger.info("🛑 Client cleanup worker stopped gracefully")

    def _subscriber_monitor_worker(self) -> None:
        """
        RTOS-compatible worker thread for subscriber monitoring.
        Replaces async _monitor_subscribers().
        """
        logger.info("Starting subscriber monitoring for automatic FQ sampler control")
        
        while self.running and not self._stop_event.is_set():
            try:
                # Quick exit check for shutdown
                if self._stop_event.is_set():
                    logger.debug("Subscriber monitor received stop signal")
                    break
                
                # Check current subscriber count
                current_count = self._get_subscriber_count()
                
                # Update subscriber count
                if current_count != self._last_subscriber_count:
                    logger.info(f"Visualization subscriber count changed: {self._last_subscriber_count} -> {current_count}")
                    self._last_subscriber_count = current_count
                    
                    # Auto-enable/disable FQ sampler based on subscriber count
                    should_enable = current_count > 0
                    
                    if should_enable != self._fq_sampler_enabled:
                        self._control_fq_sampler(should_enable)
                
                # Use smaller intervals to check for shutdown more frequently
                wait_time = min(self.subscriber_check_interval, 1.0)  # Max 1 second intervals
                
                # RTOS: Use deterministic wait with stop event check
                start_time = time.time()
                while (time.time() - start_time) < wait_time:
                    if self._stop_event.is_set():
                        logger.debug("Subscriber monitor stopping due to stop event")
                        return
                    time.sleep(0.1)  # Check stop event every 100ms
                
            except Exception as e:
                logger.error(f"Error in subscriber monitoring: {e}")
                # Use shorter wait on error to be more responsive to shutdown
                self._deterministic_wait(min(self.subscriber_check_interval, 1.0))
        
        logger.info("🛑 Subscriber monitoring stopped gracefully")

    def _process_fq_data_item(self, fq_data):
        """Process a single FQ data item."""
        try:
            logger.debug(f"🎯 Processing FQ data type: {type(fq_data)}")
            
            # Handle different data types
            if isinstance(fq_data, bytes):
                # Already encoded binary data
                logger.debug(f"💾 Binary data ({len(fq_data)} bytes)")
                if self.get_connected_client_count() > 0:
                    self._send_binary_data(fq_data)
            
            elif isinstance(fq_data, dict) and 'target' in fq_data:
                # Handle new tagged format from enhanced FQ sampler
                logger.debug(f"🏷️ Tagged data with target: {fq_data.get('target')}")
                self._process_tagged_fq_data(fq_data)
            
            elif isinstance(fq_data, tuple) and len(fq_data) == 2:
                # Handle (cortical_id, fire_queue_data) tuple format (legacy visualization)
                cortical_id, fire_data = fq_data
                logger.debug(f"📦 Tuple data for {cortical_id}: {len(fire_data.get('neuron_ids', [])) if fire_data else 0} neurons")
                self._process_fq_tuple(fq_data)
            
            elif isinstance(fq_data, dict):
                # Handle fire queue dict directly (legacy format)
                logger.debug(f"📚 Dict data: {len(fq_data.get('neuron_ids', [])) if fq_data else 0} neurons")
                self._process_fq_dict(fq_data)
            
            elif isinstance(fq_data, str) and fq_data == "STOP":
                logger.info("Received STOP signal")
                return False  # Signal to stop processing
            
            else:
                logger.warning(f"❌ Unsupported FQ data type: {type(fq_data)}")
                
        except Exception as e:
            logger.error(f"Error processing FQ data item: {e}")
            raise
        
        return True  # Continue processing

    def _process_tagged_fq_data(self, fq_data):
        """Process tagged data from enhanced FQ sampler."""
        try:
            target = fq_data.get('target', 'visualization')
            
            # Only process visualization-targeted data in visualization stream
            if target != 'visualization':
                logger.debug(f"Skipping non-visualization data (target: {target})")
                return
                
            # Extract the actual fire queue data
            if 'cortical_id' in fq_data and 'fire_queue_data' in fq_data:
                # Area-specific data
                cortical_id = fq_data['cortical_id']
                fire_queue_data = fq_data['fire_queue_data']
                self._process_fq_tuple((cortical_id, fire_queue_data))
                
            elif 'fire_queue_data' in fq_data:
                # Global data
                fire_queue_data = fq_data['fire_queue_data']
                self._process_fq_dict(fire_queue_data)
                
            else:
                logger.warning(f"Invalid tagged FQ data format: {fq_data.keys()}")
                
        except Exception as e:
            logger.error(f"Error processing tagged FQ data: {e}")

    def _process_fq_tuple(self, fq_data):
        """Process a 2-element FQ tuple and convert to visualization data."""
        try:
            cortical_id, fire_queue_data = fq_data
            
            logger.info(f"🧠 DEBUG: Processing tuple for {cortical_id}")
            
            if not fire_queue_data or not fire_queue_data.get('neuron_ids'):
                logger.info(f"❌ DEBUG: No fire queue data or neuron_ids for {cortical_id}")
                return
                
            neuron_count = len(fire_queue_data.get('neuron_ids', []))
            logger.info(f"🔥 DEBUG: {cortical_id} has {neuron_count} firing neurons")
                
            # Check if we have connected clients OR if in test mode (where we assume clients)
            client_count = self.get_connected_client_count()
            is_test_mode = self._is_test_visualization_mode()
            
            logger.info(f"👥 DEBUG: Client count: {client_count}, Test mode: {is_test_mode}, Active mode: {self._active_mode}")
            
            if client_count == 0 and not is_test_mode:
                logger.warning(f"🚫 DEBUG: BLOCKING - No clients connected and not in test mode, skipping data for {cortical_id}")
                return
                
            if is_test_mode and client_count == 0:
                logger.info(f"🧪 DEBUG: Test mode: assuming clients for area {cortical_id} ({neuron_count} neurons)")
                
            # Extract data from fire queue
            neuron_ids = fire_queue_data['neuron_ids']
            membrane_potentials = fire_queue_data.get('membrane_potentials', [])
            coordinates = fire_queue_data.get('coordinates', [])
            
            logger.info(f"📊 DEBUG: Data extracted - neurons: {len(neuron_ids)}, potentials: {len(membrane_potentials)}, coords: {len(coordinates)}")
            
            # Use coordinates if available, otherwise generate from IDs
            if coordinates and len(coordinates) == len(neuron_ids):
                x_values = [coord[0] for coord in coordinates]
                y_values = [coord[1] for coord in coordinates]
                z_values = [coord[2] for coord in coordinates]
                logger.info(f"📍 DEBUG: Using provided coordinates")
            else:
                # Fallback to ID-based coordinates
                x_values = [(nid % 100) if nid > 0 else 1 for nid in neuron_ids]
                y_values = [((nid // 100) % 100) if nid > 0 else 1 for nid in neuron_ids]
                z_values = [(nid // 10000) if nid > 0 else 0 for nid in neuron_ids]
                logger.info(f"📍 DEBUG: Generated coordinates from neuron IDs")
            
            # Use membrane potentials if available, otherwise default to 1.0
            if membrane_potentials and len(membrane_potentials) == len(neuron_ids):
                potentials = membrane_potentials
                logger.info(f"⚡ DEBUG: Using provided membrane potentials")
            else:
                potentials = [1.0] * len(neuron_ids)
                logger.info(f"⚡ DEBUG: Using default membrane potentials (1.0)")
            
            # Create cortical ID list (one per neuron)
            cortical_ids = [cortical_id] * len(neuron_ids)
            
            logger.info(f"🔮 DEBUG: Encoding {len(neuron_ids)} neurons for {cortical_id}")
                        
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
                
                logger.info(f"✅ DEBUG: Encoded {len(binary_data)} bytes, sending...")
                self._send_binary_data(binary_data)
                
            except Exception as e:
                logger.error(f"💥 DEBUG: Error encoding visualization data: {e}")
                        
        except Exception as e:
            logger.error(f"💥 DEBUG: Error processing FQ tuple: {e}")

    def _process_fq_dict(self, fire_queue_data):
        """Process a fire queue dictionary directly."""
        try:
            if not fire_queue_data or not fire_queue_data.get('neuron_ids'):
                return
                
            # Check if we have connected clients OR if in test mode (where we assume clients)
            client_count = self.get_connected_client_count()
            is_test_mode = self._is_test_visualization_mode()
            
            if client_count == 0 and not is_test_mode:
                logger.debug("No clients connected and not in test mode, skipping dict data")
                return
                
            if is_test_mode and client_count == 0:
                logger.debug(f"Test mode: assuming clients for dict data ({len(fire_queue_data.get('neuron_ids', []))} neurons)")
                
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
                
                self._send_binary_data(binary_data)
                
            except Exception as e:
                logger.error(f"Error encoding visualization data: {e}")
                
        except Exception as e:
            logger.error(f"Error processing FQ dict: {e}")

    def _send_binary_data(self, binary_data: bytes):
        """Send binary data to visualization clients."""
        try:
            logger.info(f"📡 DEBUG: Attempting to send {len(binary_data)} bytes")
            logger.info(f"📊 DEBUG: Active mode: {self._active_mode}, Running: {self.running}")
            
            # Skip if in standby mode
            if not self._active_mode:
                logger.warning(f"🚫 DEBUG: BLOCKING - Visualization stream in STANDBY mode, skipping data send")
                return
                
            logger.info(f"📡 DEBUG: Sending data on 'activity' topic...")
                
            # No artificial rate limiting - let the natural sampling frequency determine the rate
            
            # Send data on activity topic
            self.socket.send_multipart([
                b"activity",
                binary_data
            ])
            
            logger.info(f"✅ DEBUG: Successfully sent {len(binary_data)} bytes of visualization data")
            
        except Exception as e:
            logger.error(f"💥 DEBUG: Error sending binary data: {e}")

    def get_connected_client_count(self) -> int:
        """Get the estimated number of connected visualization clients."""
        with self._client_lock:  # RTOS: Thread-safe access
            return len(self.client_last_heartbeat)

    def _is_test_visualization_mode(self) -> bool:
        """Check if FEAGI is running in test visualization mode."""
        try:
            from feagi.core.state_manager import FeagiStateManager
            state_manager = FeagiStateManager.instance()
            return state_manager.get_test_visualization_mode()
        except Exception:
            return False

    def record_client_heartbeat(self, client_id: str) -> None:
        """Record a heartbeat from a client."""
        now = time.time()
        with self._client_lock:  # RTOS: Thread-safe access
            old_time = self.client_last_heartbeat.get(client_id, 0)
            self.client_last_heartbeat[client_id] = now
            
            if old_time == 0:
                logger.info(f"New visualization client connected: {client_id}")

    def send_visualization_data(self, data) -> None:
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
            
        # Skip if no clients connected AND not in test mode
        client_count = self.get_connected_client_count()
        is_test_mode = self._is_test_visualization_mode()
        
        if client_count == 0 and not is_test_mode:
            return
        
        try:
            if isinstance(data, bytes):
                self._send_binary_data(data)
            elif isinstance(data, tuple) and len(data) == 2:
                self._process_fq_tuple(data)
            elif isinstance(data, dict):
                self._process_fq_dict(data)
            else:
                logger.warning(f"Unsupported data type: {type(data)}")
            
        except Exception as e:
            logger.error(f"Error in send_visualization_data: {e}")
            
    def broadcast_update(self, data_type: str, data: bytes) -> None:
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
            # No artificial rate limiting - let the natural sampling frequency determine the rate
                
            self.socket.send_multipart([
                data_type.encode('utf-8'),
                data
            ])
            
            logger.debug(f"Broadcast {len(data)} bytes of {data_type} data")
            
        except Exception as e:
            logger.error(f"Error broadcasting {data_type} data: {e}") 

    def process_system_message(self, message: str) -> None:
        """Process system messages from clients."""
        try:
            if "HEARTBEAT:" in message:
                parts = message.split(":")
                if len(parts) >= 2:
                    client_id = parts[1].strip()
                    self.record_client_heartbeat(client_id)
            
        except Exception as e:
            logger.error(f"Error processing system message: {e}")
            
    def receive_control_message(self, message: Dict[str, Any]) -> Dict[str, Any]:
        """Process control messages from clients."""
        try:
            message_type = message.get("message_type", "unknown")
            
            if message_type == "heartbeat":
                client_id = message.get("agent_id", f"unknown_{time.time()}")
                self.record_client_heartbeat(client_id)
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

    def _get_subscriber_count(self) -> int:
        """Get the current number of ZMQ subscribers connected to the visualization socket."""
        try:
            if self.socket:
                # Method 1: Use client heartbeat tracking as a proxy
                now = time.time()
                heartbeat_clients = 0
                
                with self._client_lock:  # RTOS: Thread-safe access
                    for client_id, last_heartbeat in self.client_last_heartbeat.items():
                        if now - last_heartbeat < self.client_heartbeat_timeout:
                            heartbeat_clients += 1
                
                # Store the count for this iteration
                self._subscriber_count = heartbeat_clients
                
                # For debugging, log when we detect clients
                if heartbeat_clients > 0:
                    logger.debug(f"Detected {heartbeat_clients} active visualization clients via heartbeat")
                
                return heartbeat_clients
            return 0
        except Exception as e:
            logger.warning(f"Error getting subscriber count: {e}")
            return 0

    def register_visualization_client(self, client_id: str) -> None:
        """Register a visualization client and update heartbeat."""
        current_time = time.time()
        with self._client_lock:  # RTOS: Thread-safe access
            self.client_last_heartbeat[client_id] = current_time
        logger.info(f"📺 Visualization client registered: {client_id}")
        
        # Force a subscriber count update
        current_count = self._get_subscriber_count()
        if current_count != self._last_subscriber_count:
            self._last_subscriber_count = current_count
            should_enable = current_count > 0
            if should_enable != self._fq_sampler_enabled:
                self._control_fq_sampler(should_enable)

    def unregister_visualization_client(self, client_id: str) -> None:
        """Unregister a visualization client."""
        with self._client_lock:  # RTOS: Thread-safe access
            if client_id in self.client_last_heartbeat:
                del self.client_last_heartbeat[client_id]
                logger.info(f"📺 Visualization client unregistered: {client_id}")
                
        # Force a subscriber count update
        current_count = self._get_subscriber_count()
        if current_count != self._last_subscriber_count:
            self._last_subscriber_count = current_count
            should_enable = current_count > 0
            if should_enable != self._fq_sampler_enabled:
                self._control_fq_sampler(should_enable)

    def heartbeat_visualization_client(self, client_id: str) -> None:
        """Update heartbeat for a visualization client and enable FQ sampler if this is a new client."""
        with self._client_lock:  # RTOS: Thread-safe access
            # Check if this is a new client (no previous heartbeat recorded)
            is_new_client = client_id not in self.client_last_heartbeat
            self.client_last_heartbeat[client_id] = time.time()
            
            # If this is a new client, log it and update FQ sampler
            if is_new_client:
                logger.info(f"📺 New visualization client connected via heartbeat: {client_id}")
                
                # Force a subscriber count update and FQ sampler notification
                current_count = self._get_subscriber_count()
                if current_count != self._last_subscriber_count:
                    self._last_subscriber_count = current_count
                    should_enable = current_count > 0
                    if should_enable != self._fq_sampler_enabled:
                        self._control_fq_sampler(should_enable)

    def _control_fq_sampler(self, enable: bool) -> None:
        """Enable or disable the FQ sampler based on subscriber presence."""
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