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

import threading
import time
from typing import Any, Dict, Optional

# CRITICAL FIX: Import numpy at module level to prevent scoping issues
import numpy as np
import zmq  # Import standard synchronous ZMQ (not zmq.asyncio)

from feagi.utils.compression import create_lz4_compressor
from feagi.utils.logger import setup_logger
from feagi.utils.zmq_debug import MessageType, log_outbound

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
        fire_queue_provider=None,
        stream_config: Optional[Dict[str, Any]] = None,
        core_api: Optional[Any] = None,
        connectome_manager=None,
        # Accept pre-existing FQ sampler from process manager
        fq_sampler=None,
        # Accept process manager for on-demand FQ sampler creation
        process_manager=None,
    ):
        """Initialize the visualization stream with existing FQ sampler
        (Rust/RTOS compatible)."""
        # Core API integration for genome state management
        self.core_api = core_api
        self.process_manager = process_manager  # Store process manager reference

        # Basic connection settings
        self.host = host
        self.port = port
        self.running = False

        # ALWAYS create a NEW sync context - NEVER use shared contexts
        self.context = zmq.Context()

        # RUST/RTOS COMPATIBLE: Use FQ sampler from Process Manager
        # (created on-demand)
        self.fq_sampler = fq_sampler  # Use provided FQ sampler instance

        if self.fq_sampler:
            logger.info(
                "VisualizationStream using FQ sampler from Process Manager "
                "(created on-demand when visualization agents connect)"
            )
        else:
            logger.info(
                "No visualization FQ sampler available - will be created "
                "on-demand when visualization agents connect"
            )

        # Stream configuration
        self.stream_config = stream_config or {}
        self.auto_enable_on_subscribers = self.stream_config.get(
            "auto_enable_on_subscribers", True
        )
        self.subscriber_check_interval = self.stream_config.get(
            "subscriber_check_interval", 1.0
        )

        # 🗜️ LZ4 COMPRESSION: Simple LZ4 compression for bandwidth reduction
        compression_enabled = self.stream_config.get("compression_enabled", True)
        min_size_threshold = self.stream_config.get("min_size_threshold", 100)
        self.compressor = (
            create_lz4_compressor(min_size_threshold) if compression_enabled else None
        )

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
        if hasattr(core_api, "register_genome_change_listener"):
            core_api.register_genome_change_listener(self._on_genome_state_change)

        # Initialize state based on current genome availability
        self._update_active_mode()

        # Statistics (enhanced with compression stats)
        self.stats = {
            "data_sent": 0,
            "bytes_sent": 0,
            "bytes_saved_compression": 0,
            "compression_time_ms": 0.0,
            "start_time": time.time(),
        }

        # Sample rate for data processing timing
        self.sample_rate = 30.0  # 30 Hz default sample rate

        # State management
        self.running = False
        self._stop_event = threading.Event()
        self._client_lock = threading.Lock()

        # Add flag to prevent duplicate logging
        self._fq_sampler_unavailable_logged = False

    def _setup_socket(self) -> None:
        """Set up the ZMQ PUB socket with optimal settings."""
        self.socket = self.context.socket(zmq.PUB)

        # Optimize for real-time streaming
        self.socket.setsockopt(zmq.SNDHWM, 1000)  # Higher send buffer to prevent drops
        self.socket.setsockopt(
            zmq.LINGER, 1000
        )  # Wait briefly on close to send pending messages

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
            logger.warning(
                f"Error checking genome state: {e}, defaulting to active mode"
            )
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
            target=self._data_worker, name="VisualizationFQ", daemon=True
        )
        fq_thread.start()
        self.worker_threads.append(fq_thread)
        logger.debug("FQ data processing thread started")

        # Thread 2: Client cleanup and monitoring
        cleanup_thread = threading.Thread(
            target=self._client_cleanup_worker, name="VisualizationCleanup", daemon=True
        )
        cleanup_thread.start()
        self.worker_threads.append(cleanup_thread)
        logger.debug("Client cleanup thread started")

        # Thread 3: Subscriber monitoring for automatic FQ sampler control
        if self.auto_enable_on_subscribers:
            monitor_thread = threading.Thread(
                target=self._subscriber_monitor_worker,
                name="VisualizationMonitor",
                daemon=True,
            )
            monitor_thread.start()
            self.worker_threads.append(monitor_thread)
            logger.debug("Subscriber monitoring thread started")

        logger.info(
            f"[OK] Visualization stream started with "
            f"{len(self.worker_threads)} worker threads"
        )

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
            logger.debug(
                f"Waiting for {total_threads} worker threads to stop "
                "before socket cleanup..."
            )

            MAX_TOTAL_WAIT = 3.0  # Maximum 3 seconds total wait
            PER_THREAD_TIMEOUT = min(1.0, MAX_TOTAL_WAIT / max(total_threads, 1))

            import time

            start_time = time.time()

            for i, thread in enumerate(self.worker_threads, 1):
                # Check global timeout
                elapsed = time.time() - start_time
                if elapsed >= MAX_TOTAL_WAIT:
                    logger.warning(
                        f"Global timeout reached, abandoning remaining "
                        f"{total_threads - i + 1} threads"
                    )
                    break

                if thread.is_alive():
                    remaining_timeout = min(
                        PER_THREAD_TIMEOUT, MAX_TOTAL_WAIT - elapsed
                    )
                    logger.debug(
                        f"Waiting for thread {i}/{total_threads}: {thread.name} "
                        f"(timeout: {remaining_timeout:.1f}s)"
                    )

                    thread.join(timeout=remaining_timeout)

                    if thread.is_alive():
                        logger.warning(
                            f"Thread {thread.name} didn't stop after "
                            f"{remaining_timeout:.1f}s - continuing anyway"
                        )
                    else:
                        logger.debug(f"Thread {thread.name} stopped gracefully")
                else:
                    logger.debug(
                        f"Thread {i}/{total_threads}: {thread.name} already stopped"
                    )

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
        RUST/RTOS COMPATIBLE: FQ sampler exists from startup, uses
        enable/disable states.
        """
        logger.debug("Visualization data worker started")

        while self.running and not self._stop_event.is_set():
            try:
                # Quick exit check for shutdown
                if self._stop_event.is_set():
                    logger.debug("Data worker received stop signal")
                    break

                # Skip processing if in standby mode (genome not loaded)
                if not self._active_mode:
                    self._update_active_mode()
                    if not self._active_mode:
                        time.sleep(0.5)  # @architecture:acceptable - standby mode
                        continue

                # RTOS/RUST COMPATIBLE: Fail fast if FQ sampler dependency not
                # satisfied. Instead of dynamic discovery, require explicit
                # dependency injection at startup
                if not self.fq_sampler:
                    if self.process_manager:
                        # Try to get FQ sampler from process manager
                        viz_fq_sampler = self.process_manager.get_viz_fq_sampler()
                        if viz_fq_sampler:
                            self.fq_sampler = viz_fq_sampler
                            logger.info(
                                f"🎨 Visualization FQ sampler dependency resolved: "
                                f"{viz_fq_sampler.instance_id}"
                            )
                        else:
                            # RTOS: Log dependency failure but continue
                            # (fail gracefully). Only log once to prevent spam
                            if not self._fq_sampler_unavailable_logged:
                                logger.debug(
                                    "🔧 Visualization FQ sampler not available - "
                                    "visualization disabled"
                                )
                                self._fq_sampler_unavailable_logged = True
                    else:
                        # RTOS: Dependency injection failure - this should be
                        # caught at startup
                        logger.warning(
                            "⚠️ Process manager not available - visualization "
                            "stream dependency not satisfied"
                        )
                    continue

                # RUST/RTOS COMPATIBLE: FQ sampler exists but may be disabled
                # Skip processing if no FQ sampler exists (stream disabled) OR
                # if sampler is disabled (no clients)
                # PROFESSIONAL FIX: Check the FQ sampler's actual state instead of our own flag
                fq_sampler_has_subscribers = (
                    getattr(self.fq_sampler, "_has_visualization_subscribers", False)
                    if self.fq_sampler
                    else False
                )

                if not self.fq_sampler or not fq_sampler_has_subscribers:
                    # Brief wait to avoid busy-waiting when no clients connected
                    time.sleep(0.1)  # @architecture:acceptable - no clients connected
                    continue

                # Get data from UnifiedFQSampler ONLY when enabled (clients connected)
                try:
                    sample_data = self.fq_sampler.sample()

                    if sample_data:
                        # Convert UnifiedFQSampler format to visualization format
                        for_visualization = self._convert_fq_format_to_viz_format(
                            sample_data
                        )

                        # Only broadcast if we have visualization clients
                        with self._client_lock:
                            if len(self.client_last_heartbeat) > 0:
                                broadcast_data = self._prepare_broadcast_data(
                                    for_visualization
                                )
                                self._broadcast_to_clients(broadcast_data)

                except Exception as e:
                    logger.error(f"Error sampling from FQ sampler: {e}")

                # Maintain sample rate timing
                time.sleep(
                    1.0 / self.sample_rate
                )  # @architecture:acceptable - sample timing

            except Exception as e:
                logger.error(f"Error in visualization data worker: {e}")
                time.sleep(0.1)  # @architecture:acceptable - error recovery

        logger.debug("Visualization data worker stopped")

    def _process_cortical_area_data(self, cortical_data: Dict[str, Any]) -> None:
        """Process data in the cortical area format from UnifiedFQSampler."""
        try:
            logger.debug(f"Processing cortical area format: {len(cortical_data)} areas")

            # Encode using feagi_data_processing binary format - USE TYPE 11
            # (NEURON_CATEGORIES)
            try:
                import feagi_data_processing as fdp

                # Create the main mapped neuron data container
                generated_mapped_neuron_data = (
                    fdp.neuron_data.neuron_mappings.CorticalMappedXYZPNeuronData()
                )

                total_neurons = 0
                for area_id, area_data in cortical_data.items():
                    if not area_data or not area_data.get("neuron_ids"):
                        continue

                    neuron_ids = area_data["neuron_ids"]
                    coordinates = area_data.get("coordinates", [])
                    membrane_potentials = area_data.get("membrane_potentials", [])

                    # Validate coordinates exist
                    if not coordinates or len(coordinates) != len(neuron_ids):
                        # ❌ NO FALLBACKS - Coordinates must exist
                        raise ValueError(
                            f"Failed to get coordinates for {len(neuron_ids)} "
                            f"neurons in area {area_id}"
                        )

                    # Use high-performance coordinate extraction - real data only
                    coords_result = self.core_api.get_neuron_coordinates(neuron_ids)
                    if coords_result and "coordinates_x" in coords_result:
                        x_coords = coords_result["coordinates_x"]
                        y_coords = coords_result["coordinates_y"]
                        z_coords = coords_result["coordinates_z"]
                        valid_indices = coords_result.get("valid_indices", [True] * len(neuron_ids))
                        
                        # ROBUSTNESS: Filter out invalid neurons instead of failing completely
                        # This prevents bridge freeze when some neurons become invalid during reconstruction
                        if not all(valid_indices):
                            valid_count = sum(valid_indices)
                            logger.warning(
                                f"[VIZ-ROBUST] Area {area_id}: {len(neuron_ids) - valid_count} of {len(neuron_ids)} "
                                f"neurons have invalid coordinates (likely due to reconstruction). Filtering them out."
                            )
                            # Filter to only valid neurons
                            valid_neuron_ids = [neuron_ids[i] for i, valid in enumerate(valid_indices) if valid]
                            valid_x_coords = [x_coords[i] for i, valid in enumerate(valid_indices) if valid]
                            valid_y_coords = [y_coords[i] for i, valid in enumerate(valid_indices) if valid]
                            valid_z_coords = [z_coords[i] for i, valid in enumerate(valid_indices) if valid]
                            valid_potentials = [membrane_potentials[i] for i, valid in enumerate(valid_indices) if i < len(membrane_potentials) and valid]
                            
                            # Update variables to use only valid data
                            neuron_ids = valid_neuron_ids
                            x_coords = valid_x_coords
                            y_coords = valid_y_coords
                            z_coords = valid_z_coords
                            membrane_potentials = valid_potentials
                            
                            if len(neuron_ids) == 0:
                                logger.info(f"[VIZ-ROBUST] Area {area_id}: No valid neurons remaining, skipping area")
                                continue
                    else:
                        # ❌ Complete failure - no coordinate data available at all
                        logger.warning(
                            f"[VIZ-ROBUST] Failed to get any coordinates for area {area_id} "
                            f"({len(neuron_ids)} neurons). Skipping area instead of crashing."
                        )
                        continue

                    # Add neurons to the cortical mapping
                    for i in range(len(neuron_ids)):
                        x = int(x_coords[i]) if i < len(x_coords) else 0
                        y = int(y_coords[i]) if i < len(y_coords) else 0
                        z = int(z_coords[i]) if i < len(z_coords) else 0
                        p = (
                            int(membrane_potentials[i])
                            if i < len(membrane_potentials)
                            else 0
                        )

                        neuron_obj = fdp.neuron_data.neurons.NeuronXYZP(
                            x=x, y=y, z=z, p=p
                        )
                        cortical_id = (
                            int(area_id) if area_id.isdigit() else hash(area_id) % 1000
                        )
                        generated_mapped_neuron_data.insert(neuron_obj, cortical_id)
                        total_neurons += 1

                if total_neurons > 0:
                    # Create the byte structure
                    byte_structure = (
                        generated_mapped_neuron_data.as_new_feagi_byte_structure()
                    )
                    binary_data = byte_structure.copy_out_as_byte_vector()
                    print("raw binary data:", binary_data)
                else:
                    binary_data = b""

                # DEBUG: Log the structure ID being generated
                if binary_data and len(binary_data) > 0:
                    structure_type = byte_structure.try_get_structure_type()
                    logger.debug(
                        f"VISUALIZATION STREAM: Generated {len(binary_data)} bytes"
                    )
                    logger.debug(f"   Structure Type: {structure_type}")
                    logger.debug(f"   Version: {byte_structure.get_version()}")
                    logger.debug(
                        f"   First 8 bytes: "
                        f"{list(binary_data[: min(8, len(binary_data))])}"
                    )

                    if structure_type == 11:
                        logger.debug("   ✅ Generated Type 11 (NEURON_CATEGORIES)")
                    else:
                        logger.debug(f"   ❓ Unknown structure type: {structure_type}")

                # Publish the binary data
                self._publish_data(binary_data)
                logger.debug(
                    f"Published cortical area data: {len(cortical_data)} areas, "
                    f"{total_neurons} neurons, {len(binary_data)} bytes"
                )

            except ImportError:
                logger.error(
                    "feagi_data_processing library not available - "
                    "cannot encode binary data"
                )
            except Exception as e:
                logger.error(f"Error encoding cortical area binary data: {e}")

        except Exception as e:
            logger.error(f"Error processing cortical area data: {e}")

    def _publish_data(self, data: bytes) -> None:
        """
        Publish data on the 'activity' topic with comprehensive error handling.
        Includes optional LZ4/Zstandard compression for reduced network usage.
        """
        # Defensive null check to prevent race condition
        if not self.socket:
            logger.debug("Cannot publish data: socket is None (likely during shutdown)")
            return

        # Additional running state check
        if not self.running:
            logger.debug("Cannot publish data: stream is not running")
            return

        # 🗜️ COMPRESSION: Apply LZ4 compression if enabled
        final_data = data
        compression_ratio = 1.0
        compression_time_ms = 0.0

        if self.compressor:
            try:
                final_data, compression_info = self.compressor.compress(data)
                compression_time_ms = compression_info["time_ms"]
                compression_ratio = compression_info["ratio"]
                bytes_saved = compression_info["bytes_saved"]

                # Update statistics if compression helped
                if compression_info["compressed"]:
                    self.stats["bytes_saved_compression"] += bytes_saved
                    self.stats["compression_time_ms"] += compression_time_ms

                    # Log compression stats for first few messages or
                    # significant savings
                    if (
                        self.stats["data_sent"] <= 3
                        or compression_ratio < 0.5  # 50%+ compression
                        or self.stats["data_sent"] % 100 == 0
                    ):  # Every 100 messages
                        logger.debug(
                            f"[LZ4] {len(data)} → {len(final_data)} bytes "
                            f"({compression_ratio * 100:.1f}%) in "
                            f"{compression_time_ms:.1f}ms"
                        )
                else:
                    # Compression didn't help, use original data - gate with debug flag
                    from feagi.core.state_manager import FeagiStateManager
                    state_manager = FeagiStateManager.instance()
                    if state_manager.is_debug_zmq_outbound_enabled():
                        logger.info(f"[ZMQ-OUT-DEBUG] LZ4 Skipped - {compression_info['reason']}")

            except Exception as e:
                logger.warning(f"[LZ4] Compression failed: {e}, sending uncompressed")
                final_data = data
                compression_ratio = 1.0

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
                data=[b"activity", final_data],  # PUB/SUB multipart message
                message_type=MessageType.VISUALIZATION,
                topic="activity",
                context=f"vis_msg_{self.stats['data_sent'] + 1}",
            )

            # Use synchronous send operations
            socket_ref.send(b"activity", zmq.SNDMORE)
            socket_ref.send(final_data)

            # Update statistics
            self.stats["data_sent"] += 1
            self.stats["bytes_sent"] += len(
                final_data
            )  # Track actual bytes sent (compressed size)

            # Periodic status logging (every 100 messages)
            if self.stats["data_sent"] % 100 == 0:
                total_saved = self.stats["bytes_saved_compression"]
                avg_compression_time = self.stats["compression_time_ms"] / max(
                    self.stats["data_sent"], 1
                )
                logger.debug(
                    f"Published {self.stats['data_sent']} messages, "
                    f"{self.stats['bytes_sent']} bytes total "
                    f"(saved {total_saved} bytes via compression, "
                    f"avg {avg_compression_time:.1f}ms/msg)"
                )

            # Log first few messages to confirm publishing is working
            if self.stats["data_sent"] <= 3:
                savings_info = (
                    f", saved {len(data) - len(final_data)} bytes"
                    if len(final_data) < len(data)
                    else ""
                )
                logger.info(
                    f"Successfully published message #{self.stats['data_sent']} "
                    f"({len(final_data)} bytes{savings_info})"
                )

        except AttributeError as e:
            # Specific handling for socket = None race condition
            if "'NoneType' object has no attribute 'send'" in str(e):
                logger.debug(
                    "Socket became None during send operation "
                    "(race condition during shutdown)"
                )
                return
            else:
                logger.error(f"Unexpected AttributeError in publish_data: {e}")

        except zmq.ZMQError as e:
            # Enhanced ZMQ-specific error handling
            if e.errno == zmq.ETERM:
                logger.debug("ZMQ context terminated - stopping publish operations")
                return
            elif e.errno == zmq.EAGAIN:
                logger.warning(
                    "ZMQ socket not ready for sending (EAGAIN) - dropping message"
                )
                return
            elif "Operation cannot be accomplished in current state" in str(e):
                logger.warning("ZMQ socket corrupted, attempting recreation...")
                try:
                    self._recreate_socket()
                    # Retry once with null check
                    if self.socket and self.running:
                        self.socket.send(b"activity", zmq.SNDMORE)
                        self.socket.send(final_data)
                        logger.info("Socket recreated and retry successful")
                    else:
                        logger.debug(
                            "Cannot retry: socket or stream not available "
                            "after recreation"
                        )
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
        """Get visualization stream statistics including compression performance."""
        runtime = time.time() - self.stats["start_time"]
        # total_messages = max(self.stats["data_sent"], 1)  # Unused variable removed

        base_stats = {
            "running": self.running,
            "data_sent": self.stats["data_sent"],
            "bytes_sent": self.stats["bytes_sent"],
            "runtime_seconds": runtime,
            "messages_per_second": self.stats["data_sent"] / max(runtime, 1),
        }

        # Add compression statistics if compressor is available
        if self.compressor:
            compressor_stats = self.compressor.get_stats()
            compression_stats = {
                "compression_enabled": True,
                "compression_type": "lz4",
                "bytes_saved_compression": compressor_stats["bytes_saved"],
                "avg_compression_time_ms": compressor_stats["avg_compression_time_ms"],
                "overall_compression_ratio": compressor_stats["compression_ratio"],
                "bandwidth_savings_percent": compressor_stats[
                    "bandwidth_savings_percent"
                ],
                "failed_compressions": compressor_stats["failed_compressions"],
                "skipped_compressions": compressor_stats["skipped_compressions"],
                "compression_success_rate": compressor_stats["success_rate"],
            }
            return {**base_stats, **compression_stats}
        else:
            compression_stats = {
                "compression_enabled": False,
                "compression_type": "none",
            }
            return {**base_stats, **compression_stats}

    def heartbeat_visualization_client(self, client_id: str) -> None:
        """
        Process heartbeat from a visualization client.

        Args:
            client_id: Unique identifier of the visualization client (bridge instance)
        """
        current_time = time.time()

        with self._client_lock:  # Thread-safe client access
            was_new_client = client_id not in self.client_last_heartbeat
            self.client_last_heartbeat[client_id] = current_time

            if was_new_client:
                logger.info(f"New visualization client connected: {client_id}")
                logger.info(
                    "Note: Agent registration must be done explicitly via "
                    "REST API - no automatic registration"
                )
            else:
                logger.debug(f"Heartbeat from visualization client: {client_id}")

    def get_connected_client_count(self) -> int:
        """Get the number of connected visualization clients."""
        with self._client_lock:  # Thread-safe access
            return len(self.client_last_heartbeat)

    def _get_registered_visualization_agents(self) -> int:
        """
        Get the number of registered visualization agents from the agent registry.
        This bridges the old heartbeat system with the new agent registration system.
        """
        try:
            if not self.core_api:
                logger.debug("🔍 AGENT REGISTRY: No core_api available")
                return 0

            # Get agent registry summary from core API with retry
            max_retries = 2
            for attempt in range(max_retries):
                try:
                    agent_summary = self.core_api.get_agent_registry_summary()
                    break
                except Exception as retry_e:
                    if attempt < max_retries - 1:
                        logger.debug(
                            f"🔍 AGENT REGISTRY: Retry {attempt + 1}/{max_retries} "
                            f"after error: {retry_e}"
                        )
                        time.sleep(0.1)  # Brief delay before retry
                        continue
                    else:
                        raise retry_e

            if not agent_summary:
                logger.debug("🔍 AGENT REGISTRY: No agent summary returned")
                return 0

            # Count agents with visualization capability - use the direct count
            # from registry
            viz_agent_count = agent_summary.get("agent_count_viz", 0)
            connected_viz_agents = agent_summary.get(
                "connected_visualization_agents", []
            )

            # Enhanced logging for debugging
            if viz_agent_count > 0:
                logger.debug(
                    f"🔍 AGENT REGISTRY: Found {viz_agent_count} registered "
                    f"visualization agents: {connected_viz_agents}"
                )
            # Note: Removed spam debug log for "no agents found" - this is normal
            # when no agents are connected

            return viz_agent_count

        except Exception as e:
            logger.error(f"Error checking agent registry for visualization agents: {e}")
            # Add more detailed error information
            if logger.isEnabledFor(10):  # DEBUG level
                import traceback

                logger.debug(
                    f"Agent registry lookup traceback: {traceback.format_exc()}"
                )
            return 0

    def _control_fq_sampler(self, enable: bool) -> None:
        """
        DISABLED: FQ sampler control has been moved to the Agent API (feagi_agent.py).

        This method is now a no-op to prevent conflicts between the visualization stream
        and the Agent API's FQ sampler management.
        """
        logger.debug(
            f"FQ SAMPLER CONTROL: Ignoring call to "
            f"_control_fq_sampler(enable={enable}) - handled by Agent API"
        )
        return

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
                clients_to_remove = []

                with self._client_lock:  # Thread-safe client access
                    for client_id, last_heartbeat in self.client_last_heartbeat.items():
                        if (
                            current_time - last_heartbeat
                            > self.client_heartbeat_timeout
                        ):
                            clients_to_remove.append(client_id)

                    # Remove timed out clients
                    for client_id in clients_to_remove:
                        del self.client_last_heartbeat[client_id]
                        logger.info(f"💔 Client {client_id} disconnected (timeout)")
                        logger.info(
                            "ℹ️ Note: Agent deregistration must be done explicitly "
                            "via REST API - no automatic deregistration"
                        )

                    # Log final client count
                    total_clients = len(self.client_last_heartbeat)
                    if len(clients_to_remove) > 0:
                        logger.info(f"💡 Remaining heartbeat clients: {total_clients}")

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

        DEPRECATED: FQ sampler management is now handled by the Agent API
        (feagi_agent.py) based on agent registration/deregistration with
        visualization capabilities.

        This monitor now only tracks client connections for internal statistics
        and does NOT control the FQ sampler to prevent conflicts.
        """
        logger.info(
            "SUBSCRIBER MONITOR: Starting subscriber monitoring "
            "(FQ sampler control DISABLED - handled by Agent API)"
        )
        logger.info(
            f"SUBSCRIBER MONITOR: core_api available: "
            f"{self.core_api is not None}, check_interval: "
            f"{self.subscriber_check_interval}s"
        )

        while self.running and not self._stop_event.is_set():
            try:
                # Quick exit check for shutdown
                if self._stop_event.is_set():
                    logger.debug("Subscriber monitor received stop signal")
                    break

                # STATISTICS ONLY: Track client connections for internal use
                heartbeat_count = self.get_connected_client_count()
                registered_viz_agents = self._get_registered_visualization_agents()
                total_subscribers = heartbeat_count + registered_viz_agents

                # Update internal statistics (no FQ sampler control)
                if total_subscribers != self._last_subscriber_count:
                    logger.debug(
                        f"SUBSCRIBER MONITOR: Subscriber count changed: "
                        f"{self._last_subscriber_count} -> {total_subscribers} "
                        f"(heartbeats: {heartbeat_count}, "
                        f"agents: {registered_viz_agents})"
                    )
                    self._last_subscriber_count = total_subscribers

                # LONGER INTERVAL: Since we're not controlling FQ sampler, check
                # less frequently
                wait_time = max(
                    self.subscriber_check_interval, 10.0
                )  # Minimum 10 seconds between checks

                # Check every 2 seconds for faster shutdown response
                for _ in range(int(wait_time / 2)):
                    if self._stop_event.wait(timeout=2.0):
                        logger.debug("Subscriber monitor stopping due to stop event")
                        return

            except Exception as e:
                logger.error(f"Error in subscriber monitoring: {e}")
                # Use responsive wait on error
                for _ in range(5):  # Check every 2 seconds for 10 seconds total
                    if self._stop_event.wait(timeout=2.0):
                        return

        logger.debug("Subscriber monitoring stopped")

    def _convert_fq_format_to_viz_format(
        self, sample_data: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Convert UnifiedFQSampler format to visualization format.
        This is a pass-through since UnifiedFQSampler already provides the
        correct format.
        """
        return sample_data

    def _prepare_broadcast_data(self, for_visualization: Dict[str, Any]) -> bytes:
        """
        Prepare data for broadcasting to visualization clients.
        Convert to binary format using feagi_data_processing with
        high-performance NumPy arrays.
        """
        try:
            # Encode using feagi_data_processing binary format - USE
            # HIGH-PERFORMANCE NUMPY APPROACH
            import feagi_data_processing as fdp

            # Create the main mapped neuron data container
            generated_mapped_neuron_data = (
                fdp.neuron_data.neuron_mappings.CorticalMappedXYZPNeuronData()
            )

            # Convert cortical area data to the format expected by the new encoder
            for area_id, area_data in for_visualization.items():
                if area_data and area_data.get("neuron_ids"):
                    neuron_ids = area_data.get("neuron_ids", [])
                    membrane_potentials = area_data.get("membrane_potentials", [])

                    # MEMORY AREA FIX: Check if coordinates are already provided (for memory areas)
                    provided_coordinates = area_data.get("coordinates", [])
                    
                    if provided_coordinates:
                        # Use pre-provided coordinates (memory areas)
                        # Convert from list of tuples to separate x,y,z lists
                        x_coords = [coord[0] for coord in provided_coordinates]
                        y_coords = [coord[1] for coord in provided_coordinates]
                        z_coords = [coord[2] for coord in provided_coordinates]
                        logger.info(f"[VIZ-DEBUG] Using provided coordinates for {area_id}: {provided_coordinates}")
                    else:
                        # Use high-performance coordinate extraction - real data only (regular areas)
                        coords_result = self.core_api.get_neuron_coordinates(neuron_ids)
                        if coords_result and "coordinates_x" in coords_result:
                            x_coords = coords_result["coordinates_x"]
                            y_coords = coords_result["coordinates_y"]
                            z_coords = coords_result["coordinates_z"]
                            logger.info(f"[VIZ-DEBUG] Looked up coordinates for {area_id}: {len(x_coords)} coords")
                        else:
                            # ❌ NO FALLBACKS - Coordinates must exist
                            raise ValueError(
                                f"Failed to get coordinates for {len(neuron_ids)} "
                                f"neurons in area {area_id}"
                            )

                    # Ensure all arrays are the same length
                    max_len = len(neuron_ids)
                    if max_len == 0:
                        continue

                    # DEBUG: Log array lengths before processing
                    logger.info(f"[VIZ-DEBUG] {area_id}: neuron_ids={len(neuron_ids)}, membrane_potentials={len(membrane_potentials)}, x_coords={len(x_coords)}, y_coords={len(y_coords)}, z_coords={len(z_coords)}")

                    # Pad membrane potentials if needed
                    if len(membrane_potentials) < max_len:
                        membrane_potentials.extend(
                            [0.0] * (max_len - len(membrane_potentials))
                        )
                    elif len(membrane_potentials) > max_len:
                        membrane_potentials = membrane_potentials[:max_len]

                    # DEBUG: Log final array lengths
                    logger.info(f"[VIZ-DEBUG] {area_id} FINAL: neuron_ids={len(neuron_ids)}, membrane_potentials={len(membrane_potentials)}, x_coords={len(x_coords)}, y_coords={len(y_coords)}, z_coords={len(z_coords)}")

                    # Create NumPy arrays with proper dtypes for performance
                    # (following neuron_c example)
                    neurons_x = np.asarray(x_coords[:max_len], dtype=np.uint32)
                    neurons_y = np.asarray(y_coords[:max_len], dtype=np.uint32)
                    neurons_z = np.asarray(z_coords[:max_len], dtype=np.uint32)
                    neurons_p = np.asarray(
                        membrane_potentials[:max_len], dtype=np.float32
                    )

                    # DEBUG: Log NumPy array shapes
                    logger.info(f"[VIZ-DEBUG] {area_id} NUMPY: x.shape={neurons_x.shape}, y.shape={neurons_y.shape}, z.shape={neurons_z.shape}, p.shape={neurons_p.shape}")

                    # Create cortical ID
                    cortical_id_obj = fdp.cortical_data.CorticalID(str(area_id))

                    # Use high-performance NumPy approach (neuron_c pattern)
                    neurons_array = (
                        fdp.neuron_data.neuron_arrays.NeuronXYZPArrays.new_from_numpy(
                            neurons_x, neurons_y, neurons_z, neurons_p
                        )
                    )

                    # Insert the neuron array into the mapped data with its cortical ID
                    generated_mapped_neuron_data.insert(cortical_id_obj, neurons_array)

            # Create the final byte structure from the mapped data
            byte_structure = generated_mapped_neuron_data.as_new_feagi_byte_structure()
            binary_data = byte_structure.copy_out_as_byte_vector()

            # Gate encoding logs with debug flag
            from feagi.core.state_manager import FeagiStateManager
            state_manager = FeagiStateManager.instance()
            if state_manager.is_debug_zmq_outbound_enabled():
                logger.info(
                    f"[ZMQ-OUT-DEBUG] Encoded {len(for_visualization)} areas into {len(binary_data)} "
                    f"bytes using high-performance NumPy approach (neuron_c pattern)"
                )
            return binary_data

        except ImportError:
            logger.error(
                "feagi_data_processing library not available - "
                "cannot encode binary data"
            )
            return b""
        except Exception as e:
            logger.error(f"Error encoding visualization data: {e}")
            return b""

    def _broadcast_to_clients(self, broadcast_data: bytes) -> None:
        """
        Broadcast data to all connected visualization clients.
        """
        if broadcast_data:
            self._publish_data(broadcast_data)
        else:
            logger.debug("No data to broadcast")
