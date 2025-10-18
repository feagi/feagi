"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
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
from pathlib import Path
import threading
import time
import struct as _struct
import mmap as _mmap
import os as _os

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

    This visualization stream only supports the new UnifiedFQSampler
    architecture with cortical area-based data format.
    No legacy compatibility is maintained.

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
        self.process_manager = (
            process_manager  # Store process manager reference
        )

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
        compression_enabled = self.stream_config.get(
            "compression_enabled", True
        )
        min_size_threshold = self.stream_config.get("min_size_threshold", 100)
        self.compressor = (
            create_lz4_compressor(min_size_threshold)
            if compression_enabled
            else None
        )

        # Genome state management
        self._active_mode = False  # True when genome is loaded and ready

        # Client tracking with heartbeat timeouts
        self.client_last_heartbeat = (
            {}
        )  # Mapping of client_id -> last heartbeat time
        self.client_heartbeat_timeout = (
            30  # Consider clients disconnected after 30s
        )
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
            core_api.register_genome_change_listener(
                self._on_genome_state_change
            )

        # Initialize state based on current genome availability
        self._update_active_mode()

        # Real-time: inactivity TTL config and last-send tracker
        self._inactivity_ttl_sec = 0.5
        self._last_shm_send_time = 0.0

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
        self._shm_lock = threading.Lock()

        # Add flag to prevent duplicate logging
        self._fq_sampler_unavailable_logged = False

        # Optional SHM writer for BV
        self._shm_writer = None
        try:
            from feagi.core.state_manager import FeagiStateManager

            sm = FeagiStateManager.instance()
            shm = sm.get_shared_memory_registry() if hasattr(sm, "get_shared_memory_registry") else {}
            viz_path = shm.get("visualization_stream", "")
            if viz_path:
                with self._shm_lock:
                    if self._shm_writer is None:
                        self._shm_writer = _ShmRingWriter(Path(viz_path))
                        logger.info(f"𒓉 [SHM] Visualization stream writing to: {viz_path}")
            else:
                logger.info("𒓉 [SHM] Visualization shared memory not configured; using ZMQ PUB only")
        except Exception as e:
            logger.info(f"𒓉 [SHM] Visualization SHM registry unavailable; using ZMQ PUB only ({e})")

    def _setup_socket(self) -> None:
        """Set up the ZMQ PUB socket with optimal settings."""
        self.socket = self.context.socket(zmq.PUB)

        # Optimize for real-time streaming
        self.socket.setsockopt(
            zmq.SNDHWM, 1000
        )  # Higher send buffer to prevent drops
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
            target=self._client_cleanup_worker,
            name="VisualizationCleanup",
            daemon=True,
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

        # 𒓉 [SHM] Ensure the visualization SHM header is initialized early
        # Some clients (Godot) attempt to open the SHM immediately after register
        # but before any data is published. The header is written when the writer
        # is created, so attach proactively in a short background task.
        def _attach_shm_writer_background():
            try:
                # Try a few times over ~3 seconds
                for _ in range(12):
                    if not self.running:
                        return
                    if self._shm_writer is not None:
                        return
                    try:
                        from feagi.core.state_manager import FeagiStateManager

                        sm = FeagiStateManager.instance()
                        shm = (
                            sm.get_shared_memory_registry()
                            if hasattr(sm, "get_shared_memory_registry")
                            else {}
                        )
                        viz_path = shm.get("visualization_stream", "")
                        if viz_path:
                            with self._shm_lock:
                                if self._shm_writer is None:
                                    self._shm_writer = _ShmRingWriter(Path(viz_path))
                                    logger.info(
                                        f"𒓉 [SHM] Visualization stream attached early to: {viz_path}"
                                    )
                                    return
                    except Exception as e:
                        logger.debug(f"𒓉 [SHM] Early attach attempt failed: {e}")
                    time.sleep(0.25)
            except Exception:
                pass

        try:
            t = threading.Thread(
                target=_attach_shm_writer_background,
                name="VisualizationShmAttach",
                daemon=True,
            )
            t.start()
        except Exception:
            pass

    def stop(self) -> None:
        """Stop the visualization stream gracefully."""
        if not self.running:
            return

        logger.info("[HALT] Stopping visualization stream...")
        self.running = False
        self._stop_event.set()

        #  Wait for worker threads BEFORE closing socket to prevent race
        #  conditions
        total_threads = len(self.worker_threads)
        if total_threads > 0:
            logger.debug(
                f"Waiting for {total_threads} worker threads to stop..."
            )
            for t in self.worker_threads:
                try:
                    if t.is_alive():
                        t.join(timeout=1.0)
                except Exception:
                    pass

        # Close socket AFTER threads stop
        if self.socket:
            try:
                self.socket.close(linger=0)
            except Exception:
                pass
                self.socket = None

        # Ensure threads list is cleared
        self.worker_threads.clear()
        logger.info("[OK] Visualization stream stopped")

        if self._shm_writer:
            try:
                self._shm_writer.close()
            except Exception:
                pass
            self._shm_writer = None

    def _data_worker(self) -> None:
        """Data processing worker using UnifiedFQSampler cortical area format
        only.

        This worker:
        1. Gets the latest cortical area data from the UnifiedFQSampler
        2. Encodes data to binary format using feagi_bytes/feagi_data_processing
        3. Publishes to ZMQ (and writes to SHM if available)
        """
        # PROFILING: Track timing for bottleneck analysis
        _profiling_enabled = True
        _sample_count = 0
        _last_profile_log = time.time()
        _profile_stats = {
            "fq_sample_time": [],
            "encoding_time": [],
            "shm_write_time": [],
            "zmq_write_time": [],
            "total_loop_time": []
        }
        
        while self.running:
            try:
                if not self.fq_sampler:
                    # Try to acquire a viz FQ sampler from ProcessManager after agent registration
                    if hasattr(self, "process_manager") and self.process_manager:
                        try:
                            sampler = None
                            if hasattr(self.process_manager, "get_viz_fq_sampler"):
                                sampler = self.process_manager.get_viz_fq_sampler()
                            if sampler is not None:
                                self.fq_sampler = sampler
                                logger.info(
                                    "🎨 Visualization FQ sampler attached to stream after registration"
                                )
                            else:
                                # Handle FQ sampler unavailability with reduced logging frequency
                                if not self._fq_sampler_unavailable_logged:
                                    logger.warning(
                                        "Visualization FQ sampler is not available yet; data worker waiting"
                                    )
                                    self._fq_sampler_unavailable_logged = True
                                time.sleep(0.5)
                                continue
                        except Exception as e:
                            if not self._fq_sampler_unavailable_logged:
                                logger.warning(
                                    f"Visualization FQ sampler lookup failed: {e}; waiting"
                                )
                                self._fq_sampler_unavailable_logged = True
                            time.sleep(0.5)
                            continue
                    else:
                        if not self._fq_sampler_unavailable_logged:
                            logger.warning(
                                "Visualization FQ sampler is not available yet; data worker waiting"
                            )
                            self._fq_sampler_unavailable_logged = True
                        time.sleep(0.5)
                        continue

                self._fq_sampler_unavailable_logged = False

                # PROFILING: Start timing
                _loop_start = time.perf_counter()
                
                # Get latest cortical area data from UnifiedFQSampler
                _sample_start = time.perf_counter()
                cortical_data = self.fq_sampler.sample()
                _sample_time = (time.perf_counter() - _sample_start) * 1000
                
                # FQ sampler rate-limited: sleep to avoid busy-wait
                if cortical_data is None:
                    # DEBUG: Track how often we get None (rate limiting or no new bursts)
                    if not hasattr(self, '_none_count'):
                        self._none_count = 0
                        self._none_log_time = time.time()
                    self._none_count += 1
                    if time.time() - self._none_log_time >= 5.0:
                        logger.warning(f"⚠️ [FQ-NONE] Got None from FQ sampler {self._none_count} times in 5 sec (either rate-limiting or no new bursts)")
                        self._none_count = 0
                        self._none_log_time = time.time()
                    time.sleep(0.001)  # 1ms sleep to prevent CPU spinning
                    continue
                
                if _profiling_enabled:
                    _profile_stats["fq_sample_time"].append(_sample_time)
                    
                    # Log FQ sampler frequency on first sample
                    if _sample_count == 0 and hasattr(self.fq_sampler, 'sample_frequency_hz'):
                        configured_freq = self.fq_sampler.sample_frequency_hz
                        logger.warning(f"🔍 [FQ-FREQ-CHECK] FQ Sampler configured frequency: {configured_freq} Hz")
                
                # Debug: Check if we're getting sensory data that shouldn't be firing
                from feagi.core.state_manager import FeagiStateManager
                state_manager = FeagiStateManager.instance()
                
                # Log only when there's actual activity (reduce spam)
                if cortical_data:
                    area_count = len(cortical_data)
                    total_neurons = sum(len(area_data.get("neuron_ids", [])) for area_data in cortical_data.values() if area_data)
                    # Removed verbose FQ sampler logging
                
                if state_manager and state_manager.is_debug_npu_enabled() and cortical_data:
                    sensory_areas_found = []
                    for area_id, area_data in cortical_data.items():
                        # Convert area_id to string for comparison (it might be an int)
                        area_id_str = str(area_id)
                        if area_id_str.startswith('iic') and area_data and area_data.get("neuron_ids"):
                            neuron_count = len(area_data.get("neuron_ids", []))
                            potentials = area_data.get("membrane_potentials", [])
                            avg_potential = sum(potentials) / len(potentials) if potentials else 0
                            sensory_areas_found.append(f"{area_id_str}({neuron_count}n,avg_p={avg_potential:.3f})")
                    
                    if sensory_areas_found:
                        logger.error("[VIZ-DEBUG] ❌ FQ SAMPLER RETURNING SENSORY DATA: %s", ", ".join(sensory_areas_found))
                        logger.error("[VIZ-DEBUG] This indicates neurons with high thresholds are incorrectly firing!")

                if not cortical_data:
                    # Real-time: if idle beyond TTL, send empty to clear BV
                    # Always publish via _publish_data so it reaches SHM and/or ZMQ
                    now = time.time()
                    last = getattr(self, "_last_shm_send_time", 0.0)
                    if (now - last) > self._inactivity_ttl_sec:
                        try:
                            self._publish_data(bytes([1, 1]) + b'{"type":11,"areas":{}}')
                            self._last_shm_send_time = now
                        except Exception:
                            pass
                    time.sleep(0.01)
                    continue

                # Publish to BOTH SHM and ZMQ when both are available
                # This ensures local (SHM) and remote (ZMQ/bridge) clients both receive data
                
                # PROFILING: Encoding time
                _encode_start = time.perf_counter()
                binary_data = self._prepare_broadcast_data(cortical_data)
                _encode_time = (time.perf_counter() - _encode_start) * 1000
                if _profiling_enabled:
                    _profile_stats["encoding_time"].append(_encode_time)
                
                # EXCLUSIVE: Use SHM if available, otherwise fall back to ZMQ
                if self._shm_writer:
                    try:
                        # SHM path: Write directly to shared memory (local, no network)
                        if binary_data and len(binary_data) > 0:
                            _shm_start = time.perf_counter()
                            self._shm_writer.write_payload(binary_data)
                            _shm_time = (time.perf_counter() - _shm_start) * 1000
                            if _profiling_enabled:
                                _profile_stats["shm_write_time"].append(_shm_time)
                        else:
                            time.sleep(0.01)
                    except Exception as e:
                        logger.error(f"[SHM] Error writing SHM payload: {e}")
                elif self.socket:
                    # ZMQ path: Only used when SHM not available (remote clients)
                    try:
                        if binary_data and len(binary_data) > 0:
                            # Verify header before publishing
                            header_check = f"[{binary_data[0]},{binary_data[1]}]" if len(binary_data) >= 2 else "EMPTY"
                            area_count = len(cortical_data)
                            logger.debug(f"[VIZ-PUBLISH] Publishing {len(binary_data)} bytes ({area_count} areas, Type 11) to ZMQ:5562 - Header: {header_check}")
                            
                            _zmq_start = time.perf_counter()
                            self._publish_zmq_only(binary_data)
                            _zmq_time = (time.perf_counter() - _zmq_start) * 1000
                            if _profiling_enabled:
                                _profile_stats["zmq_write_time"].append(_zmq_time)
                        else:
                            logger.debug("[VIZ-PUBLISH] Encoder returned empty data (no visualizable areas), skipping ZMQ publish")
                    except Exception as e:
                        logger.error(f"Error encoding visualization data for ZMQ: {e}", exc_info=True)
                
                # PROFILING: Total loop time and periodic logging
                if _profiling_enabled:
                    _loop_time = (time.perf_counter() - _loop_start) * 1000
                    _profile_stats["total_loop_time"].append(_loop_time)
                    _sample_count += 1
                    
                    # Log every 100 samples or every 5 seconds
                    if _sample_count >= 100 or (time.time() - _last_profile_log) >= 5.0:
                        if _profile_stats["total_loop_time"]:
                            import statistics
                            logger.warning(f"[STATS] [PROFILE] {_sample_count} samples over {time.time() - _last_profile_log:.1f}s:")
                            logger.warning(f"  FQ Sample:  avg={statistics.mean(_profile_stats['fq_sample_time']):.2f}ms  max={max(_profile_stats['fq_sample_time']):.2f}ms")
                            logger.warning(f"  Encoding:   avg={statistics.mean(_profile_stats['encoding_time']):.2f}ms  max={max(_profile_stats['encoding_time']):.2f}ms")
                            if _profile_stats["shm_write_time"]:
                                logger.warning(f"  SHM Write:  avg={statistics.mean(_profile_stats['shm_write_time']):.2f}ms  max={max(_profile_stats['shm_write_time']):.2f}ms")
                            if _profile_stats["zmq_write_time"]:
                                logger.warning(f"  ZMQ Write:  avg={statistics.mean(_profile_stats['zmq_write_time']):.2f}ms  max={max(_profile_stats['zmq_write_time']):.2f}ms")
                            logger.warning(f"  Total Loop: avg={statistics.mean(_profile_stats['total_loop_time']):.2f}ms  max={max(_profile_stats['total_loop_time']):.2f}ms")
                            logger.warning(f"  Actual Rate: {_sample_count / (time.time() - _last_profile_log):.1f} Hz")
                            
                            # Highlight bottlenecks
                            avg_total = statistics.mean(_profile_stats['total_loop_time'])
                            if avg_total > 33.0:
                                logger.warning(f"  ⚠️  BOTTLENECK: Average loop time {avg_total:.1f}ms exceeds 30Hz budget (33.3ms)")
                        
                        # Reset stats
                        _sample_count = 0
                        _last_profile_log = time.time()
                        for key in _profile_stats:
                            _profile_stats[key].clear()

            except Exception as e:
                logger.error(f"Error processing cortical area data: {e}")

    def _process_cortical_area_data(
        self, cortical_data: Dict[str, Any]
    ) -> None:
        """Process data in the cortical area format from UnifiedFQSampler."""
        try:
            logger.debug(
                f"Processing cortical area format: {len(cortical_data)} areas"
            )

            # Encode using feagi_rust binary format (published crates)
            try:
                import feagi_rust

                # Create the visualization encoder
                encoder = feagi_rust.VisualizationEncoder()

                total_neurons = 0
                for area_id, area_data in cortical_data.items():
                    if not area_data or not area_data.get("neuron_ids"):
                        continue

                    neuron_ids = area_data["neuron_ids"]
                    coordinates = area_data.get("coordinates", [])
                    membrane_potentials = area_data.get(
                        "membrane_potentials", []
                    )

                    # Validate coordinates exist
                    if not coordinates or len(coordinates) != len(neuron_ids):
                        # ❌ NO FALLBACKS - Coordinates must exist
                        raise ValueError(
                            f"Failed to get coordinates for {len(neuron_ids)} "
                            f"neurons in area {area_id}"
                        )

                    #  Use high-performance coordinate extraction - real data
                    #  only
                    coords_result = self.core_api.get_neuron_coordinates(
                        neuron_ids
                    )
                    if coords_result and "coordinates_x" in coords_result:
                        x_coords = coords_result["coordinates_x"]
                        y_coords = coords_result["coordinates_y"]
                        z_coords = coords_result["coordinates_z"]
                        valid_indices = coords_result.get(
                            "valid_indices", [True] * len(neuron_ids)
                        )

                        #  ROBUSTNESS: Filter out invalid neurons instead of
                        #  failing completely
                        #  This prevents bridge freeze when some neurons become
                        #  invalid during reconstruction
                        if not all(valid_indices):
                            valid_count = sum(valid_indices)
                            logger.warning(
                                f"[VIZ-ROBUST] Area {area_id}: "
                                f"{len(neuron_ids) - valid_count} of {len(neuron_ids)} "
                                f"neurons have invalid coordinates (likely due to reconstruction). "
                                f"Filtering them out."
                            )
                            # Filter to only valid neurons
                            valid_neuron_ids = [
                                neuron_ids[i]
                                for i, valid in enumerate(valid_indices)
                                if valid
                            ]
                            valid_x_coords = [
                                x_coords[i]
                                for i, valid in enumerate(valid_indices)
                                if valid
                            ]
                            valid_y_coords = [
                                y_coords[i]
                                for i, valid in enumerate(valid_indices)
                                if valid
                            ]
                            valid_z_coords = [
                                z_coords[i]
                                for i, valid in enumerate(valid_indices)
                                if valid
                            ]
                            valid_potentials = [
                                membrane_potentials[i]
                                for i, valid in enumerate(valid_indices)
                                if i < len(membrane_potentials) and valid
                            ]

                            # Update variables to use only valid data
                            neuron_ids = valid_neuron_ids
                            x_coords = valid_x_coords
                            y_coords = valid_y_coords
                            z_coords = valid_z_coords
                            membrane_potentials = valid_potentials

                            if len(neuron_ids) == 0:
                                logger.info(
                                    f"[VIZ-ROBUST] Area {area_id}: "
                                    f"No valid neurons remaining, skipping area"
                                )
                                continue
                    else:
                        #  ❌ Complete failure - no coordinate data available at
                        #  all
                        logger.warning(
                            f"[VIZ-ROBUST] Failed to get any coordinates for area {area_id} "
                            f"({len(neuron_ids)} neurons). Skipping area instead of crashing."
                        )
                        continue

                    # Prepare data for encoder - ensure all lists are same length
                    num_neurons = len(neuron_ids)
                    
                    # Pad membrane_potentials if needed
                    if len(membrane_potentials) < num_neurons:
                        membrane_potentials = membrane_potentials + [0.0] * (num_neurons - len(membrane_potentials))
                    
                    # Convert to proper types (u32 for coords, f32 for potentials)
                    x_coords_u32 = [int(x) for x in x_coords[:num_neurons]]
                    y_coords_u32 = [int(y) for y in y_coords[:num_neurons]]
                    z_coords_u32 = [int(z) for z in z_coords[:num_neurons]]
                    potentials_f32 = [float(p) for p in membrane_potentials[:num_neurons]]
                    
                    # Add neurons for this cortical area
                    encoder.add_neurons(
                        cortical_id=str(area_id),
                        x_coords=x_coords_u32,
                        y_coords=y_coords_u32,
                        z_coords=z_coords_u32,
                        potentials=potentials_f32
                    )
                    total_neurons += num_neurons

                if total_neurons > 0:
                    # Encode to binary
                    binary_data = encoder.encode()
                    logger.debug(f"Encoded {total_neurons} neurons into {len(binary_data)} bytes")
                else:
                    binary_data = b""

                # DEBUG: Log the structure ID being generated
                if binary_data and len(binary_data) > 0:
                    # Type 11 (NeuronCategoricalXYZP) is hardcoded in our encoder
                    logger.debug(
                        f"VISUALIZATION STREAM: Generated {len(binary_data)} bytes (Type 11)"
                    )
                    logger.debug(
                        f"   First 8 bytes: "
                        f"{list(binary_data[: min(8, len(binary_data))])}"
                    )
                    logger.debug("   ✅ Generated Type 11 (NEURON_CATEGORIES)")

                # Publish the binary data
                self._publish_data(binary_data)
                logger.debug(
                    f"Published cortical area data: {len(cortical_data)} areas, "
                    f"{total_neurons} neurons, {len(binary_data)} bytes"
                )

            except ImportError as e:
                logger.error(
                    f"feagi_rust library not available - "
                    f"cannot encode binary data: {e}"
                )
            except Exception as e:
                logger.error(f"Error encoding cortical area binary data: {e}")

        except Exception as e:
            logger.error(f"Error processing cortical area data: {e}")

    def _publish_zmq_only(self, data: bytes) -> None:
        """Publish data to ZMQ only (not SHM).
        
        Used when SHM already has a different format (e.g., JSON) and we don't
        want to overwrite it with binary data.
        
        The data parameter should already be binary Type 11 format with [11, 1] header from encoder.
        """
        if not self.socket:
            logger.debug("Cannot publish to ZMQ: no socket available")
            return
            
        if not self.running:
            logger.debug("Cannot publish to ZMQ: stream is not running")
            return
        
        # ✅ CRITICAL: The encoder already includes [11, 1] header - DO NOT prepend again!
        # Verify header is present for logging (but don't fail if missing)
        has_header = len(data) >= 2 and data[0] == 11 and data[1] == 1
        if not has_header:
            logger.warning(f"[VIZ-ZMQ] Data missing [11,1] header! First 4 bytes: {list(data[:4]) if len(data) >= 4 else list(data)}")
            # Prepend header as fallback
            data = bytes([11, 1]) + data

        # Apply compression if enabled
        final_data = data
        compression_ratio = 1.0
        compression_time_ms = 0.0
        
        if self.compressor:
            try:
                final_data, compression_info = self.compressor.compress(data)
                compression_time_ms = compression_info["time_ms"]
                compression_ratio = compression_info["ratio"]
                
                if compression_info["compressed"]:
                    self.stats["bytes_saved_compression"] += compression_info["bytes_saved"]
                    self.stats["compression_time_ms"] += compression_time_ms
            except Exception as e:
                logger.warning(f"Compression error, sending uncompressed: {e}")
                final_data = data
        
        # Publish to ZMQ socket only (skip SHM)
        try:
            self.socket.send_multipart([b"activity", final_data])
            # Outbound debug logging (tied to global debug setting)
            if logger.isEnabledFor(10):  # DEBUG level
                endpoint = f"tcp://{self.host}:{self.port}"
                log_outbound(
                    endpoint=endpoint,
                    data=[b"activity", final_data],
                    message_type=MessageType.VISUALIZATION,
                    topic="activity",
                    context=f"len={len(final_data)} cr={compression_ratio:.3f} t={compression_time_ms:.3f}ms",
                )
            self.stats["data_sent"] += 1
            self.stats["bytes_sent"] += len(final_data)
        except Exception as e:
            logger.error(f"Error publishing to ZMQ: {e}")
    
    def _publish_data(self, data: bytes) -> None:
        """Publish data on the 'activity' topic with comprehensive error
        handling.

        Includes optional LZ4/Zstandard compression for reduced network usage.
        Also writes uncompressed payload into SHM if configured.
        """
        # Defensive null check to prevent race condition
        if not self.socket and not self._shm_writer:
            logger.debug(
                "Cannot publish data: no socket and no SHM writer (likely during shutdown)"
            )
            return

        # Additional running state check
        if not self.running:
            logger.debug("Cannot publish data: stream is not running")
            return

        # Write to SHM uncompressed if available (attach lazily if registry now set)
        if not self._shm_writer:
            try:
                from feagi.core.state_manager import FeagiStateManager

                sm = FeagiStateManager.instance()
                shm = sm.get_shared_memory_registry() if hasattr(sm, "get_shared_memory_registry") else {}
                viz_path = shm.get("visualization_stream", "")
                if viz_path:
                    with self._shm_lock:
                        if not self._shm_writer:
                            self._shm_writer = _ShmRingWriter(Path(viz_path))
                            logger.info(f"𒓉 [SHM] Visualization stream attached to: {viz_path}")
            except Exception as e:
                logger.debug(f"[SHM] Attach failed: {e}")

        if self._shm_writer:
            try:
                self._shm_writer.write_payload(data)
                # Update stats and log only the first few SHM writes
                self.stats["data_sent"] = self.stats.get("data_sent", 0) + 1
                self.stats["bytes_sent"] = self.stats.get("bytes_sent", 0) + len(data)
                if self.stats["data_sent"] <= 3:
                    logger.info(f"𒓉 [SHM] Visualization wrote {len(data)} bytes to SHM")
                # Real-time: update last sent time for inactivity TTL
                self._last_shm_send_time = time.time()
            except Exception as e:
                logger.debug(f"𒓉 [SHM] Visualization write failed: {e}")
            # Do NOT return here; continue to also publish to ZMQ if socket present

        # 🗜️ COMPRESSION: Apply LZ4 compression if enabled
        final_data = data
        compression_ratio = 1.0
        compression_time_ms = 0.0

        if self.compressor and self.socket:
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
                            f"{compression_time_ms:.3f}ms"
                        )
            except Exception as e:
                logger.warning(f"Compression error, sending uncompressed: {e}")
                final_data = data

        # Detect JSON wrapper [1,1] used for SHM-only payloads (Type 1 JSON wrapper)
        is_json_wrapper = len(data) >= 2 and data[0] == 1 and data[1] == 1

        # Publish to ZMQ if socket present and payload is NOT the SHM JSON wrapper
        if self.socket and not is_json_wrapper:
            # Apply global Byte Structure Container header if missing
            # Spec: first byte = structure_id (11 for NeuronCategoricalXYZP), second byte = version (1)
            # Ref: feagi-data-processing Byte Structure Container docs
            if not (len(final_data) >= 2 and final_data[0] == 11 and final_data[1] == 1):
                try:
                    final_data = bytes([11, 1]) + final_data
                except Exception:
                    # If header application fails, fall back to original data
                    pass
            try:
                self.socket.send_multipart([b"activity", final_data])
                if logger.isEnabledFor(10):
                    # Correct parameter order for debug logging
                    endpoint = f"tcp://{self.host}:{self.port}"
                    log_outbound(
                        endpoint=endpoint,
                        data=[b"activity", final_data],
                        message_type=MessageType.VISUALIZATION,
                        topic="activity",
                        context=f"len={len(final_data)} cr={compression_ratio:.3f} t={compression_time_ms:.3f}ms",
                    )
            except Exception as e:
                logger.error(f"Error publishing visualization data: {e}")

        self.stats["data_sent"] += 1
        self.stats["bytes_sent"] += len(final_data)

    def _build_shm_json_payload(self, cortical_data: Dict[str, Any]) -> bytes:
        """Build a compact Type-11 style JSON payload for SHM consumers.

        Structure:
          [11][1] + JSON UTF-8 bytes where JSON has keys:
          {
            "type": 11,
            "areas": {
              "<area_id>": {
                "neuron_ids": [...],
                "x": [...],
                "y": [...],
                "z": [...],
                "p": [...]
              }, ...
            }
          }
        Godot SHM reader already expects a length-prefixed byte block and forwards
        to the same processing path as WS Type 11.
        """
        try:
            # Assemble minimal JSON with arrays to avoid FDP
            out: Dict[str, Any] = {"type": 11, "areas": {}}
            total = 0
            for area_id, area in cortical_data.items():
                if not area:
                    continue
                neuron_ids = area.get("neuron_ids", [])
                pots = area.get("membrane_potentials", [])
                
                # NEW FORMAT: Rust FQ Sampler returns separate coordinate arrays
                coords_x = area.get("coordinates_x", [])
                coords_y = area.get("coordinates_y", [])
                coords_z = area.get("coordinates_z", [])
                
                # Removed verbose JSON payload logging
                
                if not neuron_ids:
                    logger.debug(f"[JSON-PAYLOAD] Skipping area {area_id}: no neuron IDs")
                    continue
                ids = list(neuron_ids)
                xs = ys = zs = None
                
                # NEW: Check for separate coordinate arrays (Rust FQ Sampler format)
                if coords_x and coords_y and coords_z and len(coords_x) == len(ids):
                    xs = [int(x) for x in coords_x]
                    ys = [int(y) for y in coords_y]
                    zs = [int(z) for z in coords_z]
                    # Using Rust FQ Sampler coordinates
                # LEGACY: Check for combined coordinates array (old format)
                elif "coordinates" in area:
                    coords = area.get("coordinates", [])
                    if coords and len(coords) >= len(ids):
                        xs = [int(c[0]) for c in coords[: len(ids)]]
                        ys = [int(c[1]) for c in coords[: len(ids)]]
                        zs = [int(c[2]) for c in coords[: len(ids)]]
                        logger.debug(f"[JSON-PAYLOAD] Area {area_id}: using legacy coordinate format ({len(xs)} neurons)")
                
                # If coordinates not provided, fetch from core_api
                if xs is None or ys is None or zs is None:
                    logger.warning(f"[JSON-PAYLOAD] Area {area_id}: coords={len(coords) if coords else 0}, need to fetch from core_api")
                    try:
                        if self.core_api and hasattr(self.core_api, "get_neuron_coordinates"):
                            coord_res = self.core_api.get_neuron_coordinates(ids)
                            if coord_res and "coordinates_x" in coord_res:
                                xs = list(map(int, coord_res.get("coordinates_x", [])))
                                ys = list(map(int, coord_res.get("coordinates_y", [])))
                                zs = list(map(int, coord_res.get("coordinates_z", [])))
                                logger.debug(f"[JSON-PAYLOAD] Area {area_id}: fetched {len(xs)} coords from core_api")
                                valid = coord_res.get("valid_indices")
                                if valid and len(valid) == len(ids):
                                    # Filter to valid
                                    ids = [nid for nid, ok in zip(ids, valid) if ok]
                                    xs = [x for x, ok in zip(xs, valid) if ok]
                                    ys = [y for y, ok in zip(ys, valid) if ok]
                                    zs = [z for z, ok in zip(zs, valid) if ok]
                                    logger.debug(f"[JSON-PAYLOAD] Area {area_id}: filtered to {len(ids)} valid neurons")
                            else:
                                logger.error(f"[JSON-PAYLOAD] Area {area_id}: core_api returned no coordinates")
                        else:
                            logger.error(f"[JSON-PAYLOAD] Area {area_id}: core_api not available for coordinate fetch")
                    except Exception as e:
                        logger.error(f"[JSON-PAYLOAD] Area {area_id}: error fetching coordinates: {e}")
                        xs = ys = zs = None
                if xs is None or ys is None or zs is None or len(xs) == 0:
                    # Unable to resolve coordinates; skip this area
                    logger.error(f"[JSON-PAYLOAD] Area {area_id}: SKIPPING - no valid coordinates (xs={xs is not None}, ys={ys is not None}, zs={zs is not None}, len={len(xs) if xs else 0})")
                    continue
                count = min(len(ids), len(xs), len(ys), len(zs))
                ids = ids[:count]
                xs = xs[:count]
                ys = ys[:count]
                zs = zs[:count]
                ps = [float(p) for p in pots[:count]] if pots else [0.0] * count
                # Convert to 6-letter cortical_id if possible
                area_key = None
                try:
                    if isinstance(area_id, int) and self.core_api and hasattr(self.core_api, "get_cortical_id_for_idx"):
                        area_key = self.core_api.get_cortical_id_for_idx(area_id)
                    elif isinstance(area_id, str) and len(area_id) == 6:
                        area_key = area_id
                    else:
                        area_key = str(area_id)
                except Exception:
                    area_key = str(area_id)
                out["areas"][area_key] = {
                    "neuron_ids": ids,
                    "x": xs,
                    "y": ys,
                    "z": zs,
                    "p": ps,
                }
                total += count
            if total == 0:
                return b""
            import json

            body = json.dumps(out, separators=(",", ":")).encode("utf-8")
            # Prefix with [1][1] for JSON wrapper containing Type 11 data (structure_id=1 for JSON, version=1)
            # The JSON itself contains "type": 11 to indicate it's neuron categorical data
            return bytes([1, 1]) + body
        except Exception as e:
            logger.debug(f"[SHM] JSON payload build failed: {e}")
            return b""


    def get_stats(self) -> Dict[str, Any]:
        """Get visualization stream statistics including compression
        performance."""
        runtime = time.time() - self.stats["start_time"]

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
                "avg_compression_time_ms": compressor_stats[
                    "avg_compression_time_ms"
                ],
                "overall_compression_ratio": compressor_stats[
                    "compression_ratio"
                ],
                "bandwidth_savings_percent": compressor_stats[
                    "bandwidth_savings_percent"
                ],
                "failed_compressions": compressor_stats["failed_compressions"],
                "skipped_compressions": compressor_stats[
                    "skipped_compressions"
                ],
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
        """Process heartbeat from a visualization client.

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
                logger.debug(
                    f"Heartbeat from visualization client: {client_id}"
                )

            # Ensure FQ sampler starts sampling when clients are present
            if self.fq_sampler and hasattr(
                self.fq_sampler, "set_visualization_subscribers"
            ):
                try:
                    self.fq_sampler.set_visualization_subscribers(True)
                    logger.debug(
                        "Visualization sampler subscriber flag set to True (heartbeat)"
                    )
                except Exception as e:
                    logger.error(
                        f"Failed to set visualization subscribers on sampler: {e}"
                    )

    def get_connected_client_count(self) -> int:
        """Get the number of connected visualization clients."""
        with self._client_lock:  # Thread-safe access
            return len(self.client_last_heartbeat)

    def _get_registered_visualization_agents(self) -> int:
        """Get the number of registered visualization agents from the agent
        registry.

        This bridges the old heartbeat system with the new agent registration
        system.
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

            return viz_agent_count

        except Exception as e:
            logger.error(
                f"Error checking agent registry for visualization agents: {e}"
            )
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
        """Client cleanup worker thread.

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
                    for (
                        client_id,
                        last_heartbeat,
                    ) in self.client_last_heartbeat.items():
                        if (
                            current_time - last_heartbeat
                            > self.client_heartbeat_timeout
                        ):
                            clients_to_remove.append(client_id)

                    # Remove timed out clients
                    for client_id in clients_to_remove:
                        del self.client_last_heartbeat[client_id]
                        logger.info(
                            f"💔 Client {client_id} disconnected (timeout)"
                        )
                        logger.info(
                            "ℹ️ Note: Agent deregistration must be done explicitly "
                            "via REST API - no automatic deregistration"
                        )

                    # Log final client count
                    total_clients = len(self.client_last_heartbeat)
                    if len(clients_to_remove) > 0:
                        logger.info(
                            f"💡 Remaining heartbeat clients: {total_clients}"
                        )

                    # Update sampler subscriber flag when last client disconnects
                    if total_clients == 0 and self.fq_sampler and hasattr(
                        self.fq_sampler, "set_visualization_subscribers"
                    ):
                        try:
                            self.fq_sampler.set_visualization_subscribers(False)
                            logger.debug(
                                "Visualization sampler subscriber flag set to False (no clients)"
                            )
                        except Exception as e:
                            logger.error(
                                f"Failed to clear visualization subscribers on sampler: {e}"
                            )

            except Exception as e:
                logger.error(f"Error cleaning up clients: {e}")

            # Use responsive wait with frequent stop event checks
            for _ in range(int(cleanup_interval * 4)):  # Check every 250ms
                if self._stop_event.wait(timeout=0.25):
                    logger.debug(
                        "Client cleanup worker stopping due to stop event"
                    )
                    return

        logger.debug("Client cleanup worker stopped")

    def _subscriber_monitor_worker(self) -> None:
        """Subscriber monitoring worker thread.

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
                registered_viz_agents = (
                    self._get_registered_visualization_agents()
                )
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

                #  LONGER INTERVAL: Since we're not controlling FQ sampler,
                #  check
                # less frequently
                wait_time = max(
                    self.subscriber_check_interval, 10.0
                )  # Minimum 10 seconds between checks

                # Check every 2 seconds for faster shutdown response
                for _ in range(int(wait_time / 2)):
                    if self._stop_event.wait(timeout=2.0):
                        logger.debug(
                            "Subscriber monitor stopping due to stop event"
                        )
                        return

            except Exception as e:
                logger.error(f"Error in subscriber monitoring: {e}")
                # Use responsive wait on error
                for _ in range(
                    5
                ):  # Check every 2 seconds for 10 seconds total
                    if self._stop_event.wait(timeout=2.0):
                        return

        logger.debug("Subscriber monitoring stopped")

    def _convert_fq_format_to_viz_format(
        self, sample_data: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Convert UnifiedFQSampler format to visualization format.

        This is a pass-through since UnifiedFQSampler already provides the
        correct format.
        """
        return sample_data

    def _prepare_broadcast_data(
        self, for_visualization: Dict[str, Any]
    ) -> bytes:
        """Prepare data for broadcasting to visualization clients.

        Convert to binary format using feagi_data_processing with high-
        performance NumPy arrays.
        """
        try:
            # Encode using feagi_rust binary format (published crates)
            import feagi_rust

            # Create the visualization encoder
            encoder = feagi_rust.VisualizationEncoder()

            #  Convert cortical area data to the format expected by the new
            #  encoder
            for area_id, area_data in for_visualization.items():
                if not area_data:
                    continue

                # Accept multiple input shapes:
                # 1) New sampler format: coordinates_x/y/z + p_array (preferred)
                # 2) Legacy 'coordinates' list of tuples
                # 3) Fallback: fetch coordinates via neuron_ids from core_api

                # Potentials may be under different keys
                potentials_field = (
                    area_data.get("membrane_potentials")
                    or area_data.get("potentials")
                    or area_data.get("p_array")
                    or area_data.get("p")
                    or []
                )

                x_coords = y_coords = z_coords = None

                # (1) New format: separate coordinate arrays
                if (
                    isinstance(area_data.get("coordinates_x"), list)
                    and isinstance(area_data.get("coordinates_y"), list)
                    and isinstance(area_data.get("coordinates_z"), list)
                ):
                    x_coords = area_data.get("coordinates_x") or []
                    y_coords = area_data.get("coordinates_y") or []
                    z_coords = area_data.get("coordinates_z") or []

                # (2) Legacy combined coordinates
                elif isinstance(area_data.get("coordinates"), list):
                    combined = area_data.get("coordinates") or []
                    if combined:
                        x_coords = [c[0] for c in combined]
                        y_coords = [c[1] for c in combined]
                        z_coords = [c[2] for c in combined]

                # (3) Fallback: fetch by neuron_ids
                else:
                    neuron_ids = area_data.get("neuron_ids", [])
                    if neuron_ids:
                        coords_result = self.core_api.get_neuron_coordinates(neuron_ids)
                        if coords_result and "coordinates_x" in coords_result:
                            x_coords = coords_result.get("coordinates_x", [])
                            y_coords = coords_result.get("coordinates_y", [])
                            z_coords = coords_result.get("coordinates_z", [])
                    # If still missing, skip area
                if not x_coords or not y_coords or not z_coords:
                    continue

                # Determine count by available arrays
                max_len = min(len(x_coords), len(y_coords), len(z_coords))
                if max_len <= 0:
                    continue

                # Normalize potentials length
                if len(potentials_field) < max_len:
                    potentials = list(potentials_field) + [0.0] * (max_len - len(potentials_field))
                else:
                    potentials = list(potentials_field)[:max_len]

                # Convert to proper types
                x_coords_u32 = [int(x) for x in x_coords[:max_len]]
                y_coords_u32 = [int(y) for y in y_coords[:max_len]]
                z_coords_u32 = [int(z) for z in z_coords[:max_len]]
                potentials_f32 = [float(p) for p in potentials[:max_len]]

                # Cortical ID conversion if needed
                if isinstance(area_id, int):
                    cortical_id_str = self.core_api.get_cortical_id_for_idx(area_id)
                    if cortical_id_str is None:
                        logger.debug(
                            f"Skipping area {area_id}: cannot map cortical_idx to cortical_id"
                        )
                        continue
                    area_str = cortical_id_str
                else:
                    area_str = str(area_id)

                encoder.add_neurons(
                    cortical_id=area_str,
                    x_coords=x_coords_u32,
                    y_coords=y_coords_u32,
                    z_coords=z_coords_u32,
                    potentials=potentials_f32,
                )

            # Encode to binary
            binary_data = encoder.encode()

            # Gate encoding logs with debug flag
            from feagi.core.state_manager import FeagiStateManager

            state_manager = FeagiStateManager.instance()
            if state_manager.is_debug_zmq_outbound_enabled():
                logger.info(
                    f"[ZMQ-OUT-DEBUG] Encoded {len(for_visualization)} areas into {len(binary_data)} "
                    f"bytes using feagi_rust encoder"
                )
            return binary_data

        except ImportError as e:
            logger.error(
                f"feagi_rust library not available - "
                f"cannot encode binary data: {e}"
            )
            return b""
        except Exception as e:
            # Encoding failed (expected when coordinates unavailable in legacy mode)
            return b""

    def _broadcast_to_clients(self, broadcast_data: bytes) -> None:
        """Broadcast data to all connected visualization clients."""
        if broadcast_data:
            self._publish_data(broadcast_data)
        else:
            logger.debug("No data to broadcast")

class _ShmRingWriter:
    """Simple ring writer for variable-length byte payloads.

    Header (256 bytes total):
      '<8sIIIQI' => magic, version, num_slots, slot_size, frame_seq, write_index
    Then num_slots slots, each slot has u32 length + payload bytes up to slot_size-4.
    """

    MAGIC = b"FEAGIVIS"
    VERSION = 1
    HEADER_SIZE = 256
    HEADER_FMT = "<8sIIIQI"

    def __init__(self, path: Path, num_slots: int = 64, slot_size: int = 1 * 1024 * 1024):
        self.path = Path(path)
        self.num_slots = int(max(2, num_slots))
        self.slot_size = int(max(1024, slot_size))
        self._mm = None
        self._fd = None
        self._frame_seq = 0
        self._write_index = 0
        self._open()

    def _open(self) -> None:
        total_size = self.HEADER_SIZE + self.num_slots * self.slot_size
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fd = _os.open(str(self.path), _os.O_CREAT | _os.O_RDWR)
        _os.ftruncate(self._fd, total_size)
        self._mm = _mmap.mmap(self._fd, total_size, access=_mmap.ACCESS_WRITE)
        header = _struct.pack(
            self.HEADER_FMT,
            self.MAGIC,
            self.VERSION,
            self.num_slots,
            self.slot_size,
            0,
            0,
        )
        self._mm.seek(0)
        self._mm.write(header)
        if self.HEADER_SIZE > len(header):
            self._mm.write(b"\x00" * (self.HEADER_SIZE - len(header)))

    def write_payload(self, payload: bytes) -> None:
        if not self._mm:
            return
        if len(payload) + 4 > self.slot_size:
            payload = payload[: self.slot_size - 4]
        slot_off = self.HEADER_SIZE + self._write_index * self.slot_size
        self._mm.seek(slot_off)
        self._mm.write(_struct.pack("<I", len(payload)))
        self._mm.write(payload)
        rem = self.slot_size - 4 - len(payload)
        if rem > 0:
            self._mm.write(b"\x00" * rem)
        self._frame_seq += 1
        self._write_index = (self._write_index + 1) % self.num_slots
        # Update header
        self._mm.seek(0)
        header = _struct.pack(
            self.HEADER_FMT,
            self.MAGIC,
            self.VERSION,
            self.num_slots,
            self.slot_size,
            self._frame_seq,
            self._write_index,
        )
        self._mm.write(header)

    def close(self) -> None:
        try:
            if self._mm:
                self._mm.flush()
                self._mm.close()
        except Exception:
            pass
        if self._fd is not None:
            try:
                _os.close(self._fd)
            except Exception:
                pass
        self._mm = None
        self._fd = None

    def get_stats(self) -> Dict[str, Any]:
        """Get visualization stream statistics including compression
        performance."""
        runtime = time.time() - self.stats["start_time"]
        #  total_messages = max(self.stats["data_sent"], 1) # Unused variable
        #  removed

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
                "avg_compression_time_ms": compressor_stats[
                    "avg_compression_time_ms"
                ],
                "overall_compression_ratio": compressor_stats[
                    "compression_ratio"
                ],
                "bandwidth_savings_percent": compressor_stats[
                    "bandwidth_savings_percent"
                ],
                "failed_compressions": compressor_stats["failed_compressions"],
                "skipped_compressions": compressor_stats[
                    "skipped_compressions"
                ],
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
        """Process heartbeat from a visualization client.

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
                logger.debug(
                    f"Heartbeat from visualization client: {client_id}"
                )

            # Ensure FQ sampler starts sampling when clients are present
            if self.fq_sampler and hasattr(
                self.fq_sampler, "set_visualization_subscribers"
            ):
                try:
                    self.fq_sampler.set_visualization_subscribers(True)
                    logger.debug(
                        "Visualization sampler subscriber flag set to True (heartbeat)"
                    )
                except Exception as e:
                    logger.error(
                        f"Failed to set visualization subscribers on sampler: {e}"
                    )

    def get_connected_client_count(self) -> int:
        """Get the number of connected visualization clients."""
        with self._client_lock:  # Thread-safe access
            return len(self.client_last_heartbeat)

    def _get_registered_visualization_agents(self) -> int:
        """Get the number of registered visualization agents from the agent
        registry.

        This bridges the old heartbeat system with the new agent registration
        system.
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
            #  Note: Removed spam debug log for "no agents found" - this is
            #  normal
            # when no agents are connected

            return viz_agent_count

        except Exception as e:
            logger.error(
                f"Error checking agent registry for visualization agents: {e}"
            )
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
        """Client cleanup worker thread.

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
                    for (
                        client_id,
                        last_heartbeat,
                    ) in self.client_last_heartbeat.items():
                        if (
                            current_time - last_heartbeat
                            > self.client_heartbeat_timeout
                        ):
                            clients_to_remove.append(client_id)

                    # Remove timed out clients
                    for client_id in clients_to_remove:
                        del self.client_last_heartbeat[client_id]
                        logger.info(
                            f"💔 Client {client_id} disconnected (timeout)"
                        )
                        logger.info(
                            "ℹ️ Note: Agent deregistration must be done explicitly "
                            "via REST API - no automatic deregistration"
                        )

                    # Log final client count
                    total_clients = len(self.client_last_heartbeat)
                    if len(clients_to_remove) > 0:
                        logger.info(
                            f"💡 Remaining heartbeat clients: {total_clients}"
                        )

                    # Update sampler subscriber flag when last client disconnects
                    if total_clients == 0 and self.fq_sampler and hasattr(
                        self.fq_sampler, "set_visualization_subscribers"
                    ):
                        try:
                            self.fq_sampler.set_visualization_subscribers(False)
                            logger.debug(
                                "Visualization sampler subscriber flag set to False (no clients)"
                            )
                        except Exception as e:
                            logger.error(
                                f"Failed to clear visualization subscribers on sampler: {e}"
                            )

            except Exception as e:
                logger.error(f"Error cleaning up clients: {e}")

            # Use responsive wait with frequent stop event checks
            for _ in range(int(cleanup_interval * 4)):  # Check every 250ms
                if self._stop_event.wait(timeout=0.25):
                    logger.debug(
                        "Client cleanup worker stopping due to stop event"
                    )
                    return

        logger.debug("Client cleanup worker stopped")

    def _subscriber_monitor_worker(self) -> None:
        """Subscriber monitoring worker thread.

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
                registered_viz_agents = (
                    self._get_registered_visualization_agents()
                )
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

                #  LONGER INTERVAL: Since we're not controlling FQ sampler,
                #  check
                # less frequently
                wait_time = max(
                    self.subscriber_check_interval, 10.0
                )  # Minimum 10 seconds between checks

                # Check every 2 seconds for faster shutdown response
                for _ in range(int(wait_time / 2)):
                    if self._stop_event.wait(timeout=2.0):
                        logger.debug(
                            "Subscriber monitor stopping due to stop event"
                        )
                        return

            except Exception as e:
                logger.error(f"Error in subscriber monitoring: {e}")
                # Use responsive wait on error
                for _ in range(
                    5
                ):  # Check every 2 seconds for 10 seconds total
                    if self._stop_event.wait(timeout=2.0):
                        return

        logger.debug("Subscriber monitoring stopped")

    def _convert_fq_format_to_viz_format(
        self, sample_data: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Convert UnifiedFQSampler format to visualization format.

        This is a pass-through since UnifiedFQSampler already provides the
        correct format.
        """
        return sample_data

    def _prepare_broadcast_data(
        self, for_visualization: Dict[str, Any]
    ) -> bytes:
        """Prepare data for broadcasting to visualization clients.

        Convert to binary format using feagi_data_processing with high-
        performance NumPy arrays.
        """
        try:
            # Encode using feagi_rust binary format (published crates)
            import feagi_rust

            # Create the visualization encoder
            encoder = feagi_rust.VisualizationEncoder()

            #  Convert cortical area data to the format expected by the new
            #  encoder
            for area_id, area_data in for_visualization.items():
                if area_data and area_data.get("neuron_ids"):
                    neuron_ids = area_data.get("neuron_ids", [])
                    membrane_potentials = area_data.get(
                        "membrane_potentials", []
                    )

                    #  MEMORY AREA FIX: Check if coordinates are already
                    #  provided (for memory areas)
                    provided_coordinates = area_data.get("coordinates", [])


                    if provided_coordinates:
                        # Use pre-provided coordinates (memory areas)
                        # Convert from list of tuples to separate x,y,z lists
                        x_coords = [coord[0] for coord in provided_coordinates]
                        y_coords = [coord[1] for coord in provided_coordinates]
                        z_coords = [coord[2] for coord in provided_coordinates]

                    else:
                        #  Use high-performance coordinate extraction - real
                        #  data only (regular areas)

                        coords_result = self.core_api.get_neuron_coordinates(
                            neuron_ids
                        )
                        if coords_result and "coordinates_x" in coords_result:
                            x_coords = coords_result["coordinates_x"]
                            y_coords = coords_result["coordinates_y"]
                            z_coords = coords_result["coordinates_z"]

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

                    # Pad membrane potentials if needed
                    if len(membrane_potentials) < max_len:
                        membrane_potentials.extend(
                            [0.0] * (max_len - len(membrane_potentials))
                        )
                    elif len(membrane_potentials) > max_len:
                        membrane_potentials = membrane_potentials[:max_len]

                    # Convert to proper types (u32 for coords, f32 for potentials)
                    x_coords_u32 = [int(x) for x in x_coords[:max_len]]
                    y_coords_u32 = [int(y) for y in y_coords[:max_len]]
                    z_coords_u32 = [int(z) for z in z_coords[:max_len]]
                    potentials_f32 = [float(p) for p in membrane_potentials[:max_len]]

                    # PROPER CORTICAL ID CONVERSION: Use CoreAPI to convert cortical_idx to cortical_id
                    if isinstance(area_id, int):
                        # Convert integer cortical_idx to proper 6-character cortical_id
                        cortical_id_str = self.core_api.get_cortical_id_for_idx(area_id)
                        if cortical_id_str is None:
                            logger.warning(f"❌ Failed to convert cortical_idx {area_id} to cortical_id - skipping area")
                            continue
                        area_str = cortical_id_str
                    else:
                        # Already a string, use as-is
                        area_str = str(area_id)

                    # Add neurons for this cortical area to the encoder
                    encoder.add_neurons(
                        cortical_id=area_str,
                        x_coords=x_coords_u32,
                        y_coords=y_coords_u32,
                        z_coords=z_coords_u32,
                        potentials=potentials_f32
                    )

            # Encode to binary
            binary_data = encoder.encode()

            # Gate encoding logs with debug flag
            from feagi.core.state_manager import FeagiStateManager

            state_manager = FeagiStateManager.instance()
            if state_manager.is_debug_zmq_outbound_enabled():
                logger.info(
                    f"[ZMQ-OUT-DEBUG] Encoded {len(for_visualization)} areas into {len(binary_data)} "
                    f"bytes using feagi_rust encoder"
                )
            return binary_data

        except ImportError as e:
            logger.error(
                f"feagi_rust library not available - "
                f"cannot encode binary data: {e}"
            )
            return b""
        except Exception as e:
            # Encoding failed (expected when coordinates unavailable in legacy mode)
            return b""

    def _broadcast_to_clients(self, broadcast_data: bytes) -> None:
        """Broadcast data to all connected visualization clients."""
        if broadcast_data:
            self._publish_data(broadcast_data)
        else:
            logger.debug("No data to broadcast")
