"""High-performance sensory stream for neural data ingestion.

This stream implements zero-copy neural data reception from FEAGI_Connector
with latest-only shared slots to prevent temporal pattern replay bugs.
"""

import asyncio
import threading
import time
from enum import IntEnum
from typing import Any, Dict, Optional
from pathlib import Path
import mmap as _mmap
import os as _os
import struct as _struct

import numpy as np
import zmq
import zmq.asyncio

from feagi.utils.logger import setup_logger

from ..memory import NeuralBufferPool
from ..neural import NeuralDataHeader, NeuralProtocolID, ZeroCopyRingBuffer
from ..neural.latest_only_slot import (
    LatestOnlySharedSlot, 
    LatestOnlyReader, 
    LatestOnlyWriter,
    create_agent_slot_path,
    cleanup_agent_slots
)
from ..platform import optimize_socket_for_neural_data

logger = setup_logger(__name__)

# Global pool for latest-only slot readers per agent
_GLOBAL_SLOT_READERS: Dict[str, LatestOnlyReader] = {}
_GLOBAL_SLOT_LOCK = threading.Lock()
_SHM_BASE_DIR = Path("/tmp/feagi_shm")

def _acquire_slot_reader(agent_id: str, max_age_ms: float = 100.0) -> Optional[LatestOnlyReader]:
    """Get or create a latest-only slot reader for an agent.
    
    Args:
        agent_id: Agent identifier
        max_age_ms: Maximum data age before considering stale
        
    Returns:
        LatestOnlyReader instance or None if creation fails
    """
    with _GLOBAL_SLOT_LOCK:
        existing_reader = _GLOBAL_SLOT_READERS.get(agent_id)
        if existing_reader:
            logger.info(f"[SLOT] Reusing reader for agent {agent_id}")
            return existing_reader
            
        try:
            slot_path = create_agent_slot_path(_SHM_BASE_DIR, agent_id, "sensory")
            slot = LatestOnlySharedSlot(slot_path)
            reader = LatestOnlyReader(slot, max_age_ms)
            _GLOBAL_SLOT_READERS[agent_id] = reader
            logger.info(f"[SLOT] Created reader for agent {agent_id}: {slot_path}")
            
            # Auto-register agent in state manager when slot reader is created
            # This ensures connected_agents registry stays synchronized with active slots
            try:
                from feagi.core.state_manager import FeagiStateManager
                sm = FeagiStateManager.instance()
                connected_agents = sm.get_connected_agents()
                
                if agent_id not in connected_agents:
                    # Register agent with minimal information (SHM-based connection)
                    sm.register_agent(
                        agent_id=agent_id,
                        agent_type="shm_connected",
                        capabilities={"sensory": {"method": "shared_memory"}}
                    )
                    logger.info(f"[SLOT] Auto-registered agent {agent_id} in state manager")
            except Exception as reg_err:
                logger.warning(f"[SLOT] Failed to auto-register agent {agent_id}: {reg_err}")
            
            return reader
        except Exception as e:
            logger.error(f"[SLOT] Failed to create reader for agent {agent_id}: {e}")
            return None

def _release_slot_reader(agent_id: str) -> None:
    """Release and cleanup latest-only slot reader for an agent."""
    with _GLOBAL_SLOT_LOCK:
        reader = _GLOBAL_SLOT_READERS.pop(agent_id, None)
        if reader:
            try:
                reader.slot.close()
                cleanup_agent_slots(_SHM_BASE_DIR, agent_id)
                logger.info(f"[SLOT] Released and cleaned up reader for agent {agent_id}")
                
                # Deregister agent from state manager when slot reader is released
                try:
                    from feagi.core.state_manager import FeagiStateManager
                    sm = FeagiStateManager.instance()
                    sm.deregister_agent(agent_id)
                    logger.info(f"[SLOT] Deregistered agent {agent_id} from state manager")
                except Exception as dereg_err:
                    logger.debug(f"[SLOT] Failed to deregister agent {agent_id}: {dereg_err}")
                    
            except Exception as e:
                logger.warning(f"[SLOT] Error cleaning up reader for agent {agent_id}: {e}")


class StreamResult(IntEnum):
    """Result codes for stream operations."""

    SUCCESS = 0
    NO_DATA = 1
    BUFFER_FULL = 2
    BUFFER_EXHAUSTED = 3
    UNKNOWN_PROTOCOL = 4
    DECODE_ERROR = 5
    API_ERROR = 6


class SensoryNeuralStream:
    """High-performance neural data ingestion from FEAGI_Connector.

    Features:
    - Zero-copy neural data processing
    - Pre-allocated ring buffers
    - Direct FCL injection without intermediate copies
    - Platform-specific socket optimizations
    - Configurable backpressure handling
    - High-performance debug hooks
    """

    def __init__(
        self,
        core_api: Any,  # CoreAPIService
        host: str = "*",
        port: int = 5558,
        context: Optional[zmq.asyncio.Context] = None,
        ring_buffer_slots: int = 1024,
        slot_size: int = 2097152,  # 2MB per slot (min cap applied below)
        cortical_config: Optional[Dict[str, Dict[str, Any]]] = None,
    ):
        """Initialize neural sensory stream.

        Args:
            core_api: Core API service for FCL injection
            host: Host to bind to
            port: Port to bind to
            context: ZMQ context (optional)
            ring_buffer_slots: Number of ring buffer slots
            slot_size: Size of each slot in bytes
            cortical_config: Cortical area configuration for buffer pools
        """
        logger.info("Initializing sensory stream.")
        self.core_api = core_api
        self.host = host
        self.port = port
        self.context = context or zmq.asyncio.Context.instance()

        # Debug endpoint for logging
        self.debug_endpoint = f"tcp://{self.host}:{self.port}"

        # Latest-only buffer for incoming ZMQ data to prevent temporal pattern replay
        # CRITICAL: Uses single-slot buffer to eliminate frequency mismatch caching
        # This prevents 20Hz agents from filling buffers when FEAGI runs at 1Hz
        self._latest_only_buffer = {
            'data': None,
            'timestamp_ns': 0,
            'lock': threading.Lock()
        }
        self._max_slot_size = max(524288, int(slot_size))  # Keep size limit

        # Neural-specific buffer pools
        if cortical_config:
            self.neural_buffers = NeuralBufferPool(cortical_config)
        else:
            self.neural_buffers = None

        # Socket setup - delay until start() is called
        self.socket = None

        # State
        self.running = False
        self._stats = {
            "messages_received": 0,
            "bytes_received": 0,
            "decode_errors": 0,
            "buffer_overruns": 0,
            "api_errors": 0,
            "last_message_time": 0,
        }

        # Enable SHM debug when configured via StateManager.debug_shm (set by --debug-shm)
        self._debug_shm = False
        try:
            from feagi.core.state_manager import FeagiStateManager

            sm = FeagiStateManager.instance()
            dbg = getattr(sm, "_debug_config", {}) or {}
            self._debug_shm = bool(dbg.get("debug_shm", False))
        except Exception:
            self._debug_shm = False

    def _shm_log(self, message: str) -> None:
        """Emit SHM diagnostics to both logger and stdout when debug_shm is enabled."""
        if not getattr(self, "_debug_shm", False):
            return
        try:
            logger.info(f"𒓉 [SHM] {message}")
        except Exception:
            pass
        try:
            print(f"𒓉 [SHM-DBG] {message}", flush=True)
        except Exception:
            pass

    def _ensure_slot_structures(self) -> None:
        """Ensure latest-only slot reader structures exist."""
        if not hasattr(self, "_slot_lock"):
            import threading as _threading
            self._slot_lock = _threading.Lock()
        if not hasattr(self, "_slot_readers"):
            self._slot_readers = {}
        if self._debug_shm:
            self._shm_log("Initialized latest-only slot reader structures")

        # Protocol handlers
        self._protocol_handlers = {
            NeuralProtocolID.NEURON_FLAT: self._handle_neuron_flat,
            NeuralProtocolID.NEURON_SPARSE: self._handle_neuron_sparse,
            NeuralProtocolID.NEURON_MULTI: self._handle_neuron_multi,
            NeuralProtocolID.CORTICAL_MAP: self._handle_cortical_map,
        }

        # Initialize latest-only slot readers for connected agents
        self._slot_readers: Dict[str, LatestOnlyReader] = {}
        self._slot_lock = threading.Lock()
        
        # Set maximum age for sensory data (100ms default)
        # Data older than this will be automatically rejected
        self._max_sensory_age_ms = 100.0
        
        try:
            from feagi.core.state_manager import FeagiStateManager
            sm = FeagiStateManager.instance()
            
            # Get connected agents instead of shared memory mappings  
            # This avoids the StateManager registry fluctuations that caused the bug
            connected_agents = sm.get_connected_agents() if hasattr(sm, 'get_connected_agents') else {}
            
            for agent_id, agent_data in connected_agents.items():
                # Only create readers for agents that have sensory capabilities
                capabilities = agent_data.get('capabilities', {})
                if capabilities.get('sensory') or capabilities.get('neurons_stream'):
                    try:
                        reader = _acquire_slot_reader(agent_id, self._max_sensory_age_ms)
                        if reader:
                            self._slot_readers[agent_id] = reader
                            logger.info(f"✅ [SLOT] Initialized sensory slot reader for agent {agent_id}")
                    except Exception as e:
                        logger.warning(f"[SLOT] Failed to initialize sensory slot reader for {agent_id}: {e}")
                        
        except Exception as e:
            logger.info(f"[SLOT] Sensory slot registry unavailable; using ZMQ only ({e})")

    def _is_debug_npu_enabled(self) -> bool:
        """Check if NPU debug logging is enabled via state manager.

        Returns:
            True when --debug-npu is enabled, else False.
        """
        try:
            from feagi.core.state_manager import FeagiStateManager

            return FeagiStateManager.instance().is_debug_npu_enabled()
        except Exception:
            # @architecture:acceptable - test isolation (no logging without state manager)
            return False

    def _setup_socket(self) -> zmq.Socket:
        """Set up optimized PULL socket for neural data."""
        socket = self.context.socket(zmq.PULL)

        # Apply platform-specific optimizations
        optimize_socket_for_neural_data(socket, zmq.PULL, is_sender=False)

        # Bind to address
        bind_addr = f"tcp://{self.host}:{self.port}"
        logger.info(f"Binding neural sensory PULL socket to {bind_addr}")
        socket.bind(bind_addr)

        return socket

    async def start(self) -> None:
        """Start the neural sensory stream."""
        if self.running:
            return

        logger.info(
            f"Starting neural sensory stream on {self.host}:{self.port}"
        )
        
        # Ensure latest-only slot structures initialized before tasks start
        self._ensure_slot_structures()

        # Create and bind socket
        self.socket = self._setup_socket()
        self.running = True

        # Start processing tasks
        self._process_task = asyncio.create_task(self._process_loop())
        # Start latest-only slot poller for connected agents
        self._slot_task = asyncio.create_task(self._process_slot_loop())
        # Optional periodic slot summary
        if self._debug_shm:
            try:
                self._slot_summary_task = asyncio.create_task(self._log_slot_summary_loop())
            except Exception:
                self._slot_summary_task = None

        logger.info("Neural sensory stream started")


    async def stop(self) -> None:
        """Stop the neural sensory stream."""
        if not self.running:
            return

        logger.info("Stopping neural sensory stream")
        self.running = False

        # Cancel processing task with error handling
        if hasattr(self, "_process_task"):
            self._process_task.cancel()
            try:
                await self._process_task
            except asyncio.CancelledError:
                pass
            except Exception as e:
                logger.warning(f"Error cancelling process task: {e}")
        if hasattr(self, "_slot_task"):
            self._slot_task.cancel()
            try:
                await self._slot_task
            except asyncio.CancelledError:
                pass
            except Exception as e:
                logger.warning(f"Error cancelling slot task: {e}")
        if hasattr(self, "_slot_summary_task"):
            self._slot_summary_task.cancel()
            try:
                await self._slot_summary_task
            except asyncio.CancelledError:
                pass
            except Exception as e:
                logger.warning(f"Error cancelling slot summary task: {e}")

        # Close socket with error handling
        if self.socket:
            try:
                self.socket.close()
            except Exception as e:
                logger.warning(f"Error closing socket: {e}")

        # Clear latest-only buffer
        try:
            with self._latest_only_buffer['lock']:
                self._latest_only_buffer['data'] = None
                self._latest_only_buffer['timestamp_ns'] = 0
        except Exception:
            pass

        # Close buffer pools with error handling
        if self.neural_buffers:
            try:
                self.neural_buffers.close()
            except Exception as e:
                logger.warning(f"Error closing neural buffers: {e}")

        logger.info("Neural sensory stream stopped")

        # Close latest-only slot readers
        try:
            with self._slot_lock:
                for agent_id in list(self._slot_readers.keys()):
                    try:
                        _release_slot_reader(agent_id)
                    except Exception:
                        pass
                self._slot_readers.clear()
        except Exception:
            pass

    def __del__(self):
        """Destructor to ensure cleanup even if stop() isn't called
        explicitly."""
        try:
            #  Only attempt cleanup if we haven't already cleaned up and are
            #  still running
            if getattr(self, "running", False):
                # Try to stop gracefully but don't await (we're in destructor)
                if hasattr(self, "socket") and self.socket:
                    try:
                        self.socket.close()
                    except Exception:
                        pass

                if hasattr(self, "neural_buffers") and self.neural_buffers:
                    try:
                        self.neural_buffers.close()
                    except Exception:
                        pass
        except Exception:
            # In destructor, we can't do much about errors
            # Just ensure we don't raise exceptions from __del__
            pass

    async def _process_loop(self) -> None:
        """Main processing loop for neural data."""
        while self.running:
            try:
                result = await self._process_neural_data()

                if result == StreamResult.NO_DATA:
                    # No data available, yield to other tasks
                    await asyncio.sleep(0.001)  # 1ms
                elif result == StreamResult.BUFFER_FULL:
                    # Ring buffer full, apply backpressure
                    self._stats["buffer_overruns"] += 1
                    logger.warning(
                        f"Ring buffer full, applying backpressure. Buffer stats: "
                        f"latest_only_buffer active, "
                        f"polling_based_on_capability_rates"
                    )
                    await asyncio.sleep(0.01)  # 10ms backpressure

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in neural processing loop: {e}")
                await asyncio.sleep(0.1)  # Error throttling

    async def _process_slot_loop(self) -> None:
        """🦀 DEPRECATED: Python sensory polling has been replaced by Rust.
        
        Sensory injection now happens in pure Rust threads (zero Python overhead).
        See: feagi-rust/crates/feagi-burst-engine/src/sensory/
        
        This method is kept for backward compatibility but does nothing.
        """
        logger.info("🦀 [RUST-SENSORY] Python sensory polling DISABLED - using Rust threads")
        logger.info("   All sensory data is now injected directly by Rust SHM polling threads")
    
    def _calculate_next_poll_time(self, capability_manager, current_time_ns: int) -> int:
        """Calculate when the next agent should be polled based on registered rates."""
        next_poll_times = []
        
        try:
            from feagi.api.v1.capability_rates import CapabilityType
            
            # Get all registered agents
            for agent_id in capability_manager.get_all_registered_agents():
                registry = capability_manager.get_agent_capabilities(agent_id)
                if registry:
                    # Check sensory capability
                    sensory_rate = registry.get_capability_rate(CapabilityType.SENSORY)
                    if sensory_rate:
                        next_poll = sensory_rate.last_poll_time_ns + sensory_rate.poll_interval_ns
                        if next_poll > current_time_ns:  # Only future poll times
                            next_poll_times.append(next_poll)
                    
                    # Check legacy neurons_stream capability
                    neurons_rate = registry.get_capability_rate(CapabilityType.NEURONS_STREAM)
                    if neurons_rate:
                        next_poll = neurons_rate.last_poll_time_ns + neurons_rate.poll_interval_ns
                        if next_poll > current_time_ns:
                            next_poll_times.append(next_poll)
            
            if next_poll_times:
                return min(next_poll_times)
        except Exception as e:
            logger.debug(f"[RATE] Error calculating next poll time: {e}")
        
        # Default to 10ms in the future if calculation fails
        return current_time_ns + 10_000_000  # 10ms
    

    async def _log_slot_summary_loop(self) -> None:
        """Periodically log attached slot readers and their stats when --debug-shm is enabled."""
        while self.running and self._debug_shm:
            try:
                items = []
                with self._slot_lock:
                    for agent_id, reader in self._slot_readers.items():
                        try:
                            slot_path = str(getattr(reader.slot, "path", "?"))
                            last_seq = getattr(reader, "_last_sequence", 0) 
                            total_reads = reader.slot.stats.total_reads
                            stale_rejections = reader.slot.stats.stale_data_rejections
                            items.append((agent_id, slot_path, last_seq, total_reads, stale_rejections))
                        except Exception:
                            pass
                            
                if items:
                    for agent_id, path, last_seq, reads, stale in items[:6]:
                        logger.info(f"✅ [SLOT] Summary: agent={agent_id} last_seq={last_seq} reads={reads} stale_rejected={stale}")
                    extra = len(items) - 6
                    if extra > 0:
                        logger.info(f"✅ [SLOT] Summary: (+{extra} more readers)")
                else:
                    logger.info("✅ [SLOT] Summary: no slot readers active")
            except asyncio.CancelledError:
                break
            except Exception:
                pass
            await asyncio.sleep(1.0)

    async def _process_neural_data(self) -> StreamResult:
        """Process incoming neural data - feagi_data_processing format only."""
        
        # Check if socket is available (stream may not be started yet)
        if not self.socket:
            return StreamResult.NO_DATA
        
        # CRITICAL: Check burst engine readiness before processing sensory data
        try:
            from feagi.core.state_manager import FeagiStateManager, ServiceState
            state_manager = FeagiStateManager.instance()
            burst_engine_state = state_manager.get_burst_engine_state()
            
            if burst_engine_state != ServiceState.READY.value:
                # Burst engine not ready - drop data silently (standby mode)
                try:
                    # Drain incoming messages to prevent backlog
                    data = await self.socket.recv(flags=zmq.NOBLOCK)
                    self._stats["messages_dropped_not_ready"] = getattr(self._stats, "messages_dropped_not_ready", 0) + 1
                    
                    # DEBUG: Log when dropping messages (only periodically to avoid spam)
                    dropped_count = self._stats["messages_dropped_not_ready"]
                    if self._is_debug_npu_enabled() and (dropped_count % 50 == 1):  # Log every 50 drops
                        logger.info(f"🔒 STANDBY MODE: Dropped {dropped_count} sensory messages - burst engine not ready (state={burst_engine_state})")
                        
                    return StreamResult.NO_DATA  # Don't process
                except zmq.Again:
                    return StreamResult.NO_DATA  # No data anyway
            else:
                # Burst engine is ready - check if we're transitioning from standby
                dropped_count = getattr(self._stats, "messages_dropped_not_ready", 0)
                if self._is_debug_npu_enabled() and dropped_count > 0 and not hasattr(self, '_ready_logged'):
                    logger.info(f"✅ ACTIVE MODE: Burst engine ready - resuming sensory processing (dropped {dropped_count} messages while in standby)")
                    self._ready_logged = True  # Prevent spam
                    
        except Exception:
            # If state check fails, proceed anyway to avoid breaking stream
            pass
        
        
        data_processed = False

        try:
            # ZMQ receive - async sockets use recv() not recv_into()
            try:
                data = await self.socket.recv(flags=zmq.NOBLOCK)
                nbytes = len(data)

                # Check if data fits in buffer limit
                if nbytes > self._max_slot_size:
                    logger.error(
                        f"Message too large: {nbytes} bytes > {self._max_slot_size} max size"
                    )
                    return StreamResult.BUFFER_FULL

                # Store in latest-only buffer (overwrites previous data)
                with self._latest_only_buffer['lock']:
                    self._latest_only_buffer['data'] = data
                    self._latest_only_buffer['timestamp_ns'] = time.time_ns()
                    
                data_processed = True

            except zmq.Again:
                # No data available
                return StreamResult.NO_DATA

            self._stats["messages_received"] += 1
            self._stats["bytes_received"] += nbytes
            self._stats["last_message_time"] = time.time()

            # Decode feagi_data_processing format using native feagi_rust module
            try:
                import feagi_rust

                # Create FeagiByteStructure directly from raw bytes (native Rust API)
                raw_bytes = data[:nbytes]
                
                # Debug: Log byte structure details
                if len(raw_bytes) < 10:
                    logger.error(f"[DECODE-DEBUG] Received very short data: {len(raw_bytes)} bytes, hex={raw_bytes.hex()}")
                elif len(raw_bytes) < 100:
                    logger.debug(f"[DECODE-DEBUG] Received data: {len(raw_bytes)} bytes, first 20 bytes hex={raw_bytes[:20].hex()}")
                
                byte_structure = feagi_rust.FeagiByteStructure(raw_bytes)

                # Get structure type using FEAGI's API (informational only)
                try:
                    structure_type = byte_structure.structure_type
                    # Log once if not the typical neuron categories type; do not early-return
                    if structure_type != 11 and not hasattr(self, "_stype_warned"):
                        logger.info(
                            f"[NPU] structure_type={structure_type} (proceeding to decode)"
                        )
                        self._stype_warned = True
                except Exception:
                    structure_type = None

                # Create CorticalMappedXYZPNeuronVoxelsDecoder from the byte structure (native Rust API)
                cortical_mapped = feagi_rust.CorticalMappedXYZPNeuronVoxelsDecoder.new_from_feagi_byte_structure(
                    byte_structure
                )

                # Extract neuron data using iter_full()
                cortical_areas = {}
                neuron_count = 0

                for (
                    cortical_id_obj,
                    neuron_arrays,
                ) in cortical_mapped.iter_full():
                    #  neuron_arrays is a tuple: (x_coords, y_coords, z_coords,
                    #  potentials)
                    x_coords, y_coords, z_coords, potentials = neuron_arrays

                    # CRITICAL FIX: Handle both CorticalID objects and strings
                    if hasattr(cortical_id_obj, "as_ascii_string"):
                        # It's a CorticalID object - convert to string
                        cortical_id = cortical_id_obj.as_ascii_string()
                    else:
                        #  It's already a string, but might be
                        #  'CorticalID(iic000)' format
                        cortical_id_str = str(cortical_id_obj)
                        if cortical_id_str.startswith(
                            "CorticalID("
                        ) and cortical_id_str.endswith(")"):
                            #  Extract the actual cortical ID from
                            #  'CorticalID(iic000)' format
                            cortical_id = cortical_id_str[
                                11:-1
                            ]  # Remove 'CorticalID(' and ')'
                        else:
                            # Use as is
                            cortical_id = cortical_id_str

                    # Optional anomaly log (debug only) if non-normalized forms were observed
                    if self._is_debug_npu_enabled():
                        try:
                            srepr = str(cortical_id_obj)
                            if (srepr.startswith("CorticalID(") and srepr.endswith(")")) or srepr.startswith("'") or srepr.startswith('"'):
                                if not hasattr(self, "_cid_warned_zmq"):
                                    logger.warning(f"[NPU] Non-normalized cortical ID observed in ZMQ path: {srepr} -> {cortical_id}")
                                    self._cid_warned_zmq = True
                        except Exception:
                            pass

                    if cortical_id not in cortical_areas:
                        cortical_areas[cortical_id] = {
                            "coordinates_x": [],
                            "coordinates_y": [],
                            "coordinates_z": [],
                            "membrane_potentials": [],
                        }

                    # Convert numpy arrays to lists and extend
                    cortical_areas[cortical_id]["coordinates_x"].extend(
                        x_coords.tolist()
                    )
                    cortical_areas[cortical_id]["coordinates_y"].extend(
                        y_coords.tolist()
                    )
                    cortical_areas[cortical_id]["coordinates_z"].extend(
                        z_coords.tolist()
                    )
                    cortical_areas[cortical_id]["membrane_potentials"].extend(
                        potentials.tolist()
                    )

                    neuron_count += len(x_coords)

                if self._is_debug_npu_enabled():
                    logger.info(
                        f"✅ Decoded {neuron_count} neurons using native feagi_rust decoder"
                    )

                # Convert back to numpy arrays for SIMD performance
                import numpy as np

                neural_data = {}

                for cortical_id, data in cortical_areas.items():
                    neural_data[cortical_id] = {
                        "coordinates_x": np.array(
                            data["coordinates_x"], dtype=np.uint32
                        ),
                        "coordinates_y": np.array(
                            data["coordinates_y"], dtype=np.uint32
                        ),
                        "coordinates_z": np.array(
                            data["coordinates_z"], dtype=np.uint32
                        ),
                        "membrane_potentials": np.array(
                            data["membrane_potentials"], dtype=np.float32
                        ),
                    }
                    if self._is_debug_npu_enabled():
                        logger.info(
                            f"🧠 Cortical area {cortical_id}: {len(data['coordinates_x'])} neurons"
                        )

                # Optional decode summary for diagnostics (debug only)
                if self._is_debug_npu_enabled():
                    try:
                        area_count = len(cortical_areas)
                        logger.info(f"[NPU] Sensory decoded (ZMQ): areas={area_count}, points={neuron_count}")
                        if area_count:
                            _ids = list(cortical_areas.keys())
                            _preview = ", ".join(_ids[:6])
                            _more = area_count - 6
                            _suffix = f" (+{_more} more)" if _more > 0 else ""
                            logger.info(f"[NPU] Areas (ZMQ): {_preview}{_suffix}")
                            # Log first 10 tuples for up to 3 areas
                            for _cid in _ids[:3]:
                                _data = cortical_areas[_cid]
                                _xs = _data["coordinates_x"][:10]
                                _ys = _data["coordinates_y"][:10]
                                _zs = _data["coordinates_z"][:10]
                                _ps = _data["membrane_potentials"][:10]
                                _tuples = ", ".join(
                                    f"({int(x)},{int(y)},{int(z)},{float(p):.3f})" for x, y, z, p in zip(_xs, _ys, _zs, _ps)
                                )
                                logger.info(f"[NPU] {_cid} xyzp first10: {_tuples}")
                    except Exception:
                        pass

                # Inject into FCL using SIMD-optimized stimulate_neurons method
                result = self.core_api.stimulate_neurons(neural_data)
                # Detailed area mapping diagnostics (debug only)
                if self._is_debug_npu_enabled():
                    try:
                        area_results = result.get("area_results") or {}
                        for aid, meta in area_results.items():
                            logger.info(
                                f"[NPU] Map {aid}: unique={meta.get('unique_coordinates')}, "
                                f"found={meta.get('total_neurons_found')}, "
                                f"stimulated={meta.get('stimulated_count')}, failed={meta.get('failed_count')}, "
                                f"ok={meta.get('success')}, err={meta.get('error', '')}"
                            )
                        logger.info(
                            f"[NPU] Injection summary: injected={result.get('injected_count', 0)}, "
                            f"total_stimulated={result.get('total_stimulated', 0)}"
                        )
                    except Exception:
                        pass

                if result.get("success", False):
                    if self._is_debug_npu_enabled():
                        logger.info(
                            f"📥 Successfully QUEUED {neuron_count} neurons for FCL injection across {len(neural_data)} cortical areas (pending burst processing)"
                        )
                    return StreamResult.SUCCESS
                else:
                    logger.error(
                        f"❌ FCL injection failed: {result.get('error', 'Unknown error')}"
                    )
                    self._stats["api_errors"] += 1
                    return StreamResult.API_ERROR

            except Exception as e:
                logger.error(
                    f"❌ Failed to decode using native feagi_rust decoder: {e}"
                )
                import traceback

                logger.error(f"🔍 Decode traceback: {traceback.format_exc()}")
                self._stats["decode_errors"] += 1
                return StreamResult.DECODE_ERROR

        except Exception as e:
            self._stats["decode_errors"] += 1
            # Provide more context for buffer-related errors
            if "buffer" in str(e).lower():
                logger.error(
                    f"Failed to process neural data (buffer issue): {e}. "
                    f"Ring buffer state: used={self.ring_buffer.used_slots}/{self.ring_buffer.slots}"
                )
            else:
                logger.error(f"Failed to process neural data: {e}")
            return StreamResult.DECODE_ERROR

        finally:
            # Latest-only buffer requires no explicit cleanup - data is automatically 
            # overwritten on next write, eliminating temporal pattern replay bug
            pass

    async def _process_neural_payload_bytes(self, raw_bytes: bytes) -> None:
        """Process neural payload provided as bytes (from SHM)."""
        try:
            # Enforce rust_py_libs-only decoding; reject custom zero-serialize payloads
            if len(raw_bytes) >= 8 and raw_bytes[:4] == b"ZS1N":
                logger.error("[SHM] Received zero-serialized 'ZS1N' payload; FEAGI is configured for Rust-only decoding. Payload rejected.")
                self._stats["decode_errors"] = self._stats.get("decode_errors", 0) + 1
                return

            # Fallback decoding - simplified
            structure_type = raw_bytes[0] if len(raw_bytes) > 0 else 0
            
            # Log once if not the typical neuron categories type
            if structure_type != 11 and not hasattr(self, "_stype_warned_fallback"):
                logger.info(
                    f"[NPU-FALLBACK] structure_type={structure_type} (use REST API for sensory injection)"
                )
                self._stype_warned_fallback = True

            # For now, skip binary decoding - agents should use REST API for sensory injection
            logger.debug("[SENSORY-SHM-FALLBACK] Binary sensory data - use REST API instead")
            return
            
            # Placeholder for future Rust-based decoder
            cortical_mapped = None  # TODO: Add feagi_rust.DataDecoder when available
            if False:  # Disabled until decoder is available
                byte_structure = None
            cortical_areas: Dict[str, Dict[str, Any]] = {}
            neuron_count = 0
            for (cortical_id_obj, neuron_arrays) in cortical_mapped.iter_full():
                x_coords, y_coords, z_coords, potentials = neuron_arrays
                # Normalize cortical ID to plain 6-char ID (e.g., "iic400")
                if hasattr(cortical_id_obj, "as_ascii_string"):
                    cortical_id = str(cortical_id_obj.as_ascii_string())
                else:
                    cortical_id = str(cortical_id_obj)
                # Strip wrapper like "CorticalID(iic400)" if present
                if cortical_id.startswith("CorticalID(") and cortical_id.endswith(")"):
                    cortical_id = cortical_id[len("CorticalID("):-1]
                # Optional anomaly log (debug only)
                if self._is_debug_npu_enabled():
                    try:
                        srepr = str(cortical_id_obj)
                        if (srepr.startswith("CorticalID(") and srepr.endswith(")")) or srepr.startswith("'") or srepr.startswith('"'):
                            if not hasattr(self, "_cid_warned_shm"):
                                logger.warning(f"[SHM] Non-normalized cortical ID observed: {srepr} -> {cortical_id}")
                                self._cid_warned_shm = True
                    except Exception:
                        pass
                if cortical_id not in cortical_areas:
                    cortical_areas[cortical_id] = {
                        "coordinates_x": [],
                        "coordinates_y": [],
                        "coordinates_z": [],
                        "membrane_potentials": [],
                    }
                cortical_areas[cortical_id]["coordinates_x"].extend(x_coords.tolist())
                cortical_areas[cortical_id]["coordinates_y"].extend(y_coords.tolist())
                cortical_areas[cortical_id]["coordinates_z"].extend(z_coords.tolist())
                cortical_areas[cortical_id]["membrane_potentials"].extend(potentials.tolist())
                neuron_count += len(x_coords)

            # Log decoded summary for diagnostics
            try:
                area_count = len(cortical_areas)
                logger.info(f"𒓉 [SHM] Sensory decoded: areas={area_count}, points={neuron_count}")
                # Include cortical IDs (short preview to avoid log spam)
                if area_count:
                    _ids = list(cortical_areas.keys())
                    _preview = ", ".join(_ids[:6])
                    _more = area_count - 6
                    _suffix = f" (+{_more} more)" if _more > 0 else ""
                    logger.info(f"𒓉 [SHM] Sensory areas: {_preview}{_suffix}")
                # Log first 10 (x,y,z,p) tuples per cortical area for diagnostics
                # Gate on --debug-shm, env FEAGI_DEBUG_SHM=1, or --debug-npu
                if (
                    getattr(self, "_debug_shm", False)
                    or self._is_debug_npu_enabled()
                    or (__import__("os").environ.get("FEAGI_DEBUG_SHM") == "1")
                ):
                    try:
                        for _cid, _data in cortical_areas.items():
                            _xs = _data["coordinates_x"][:10]
                            _ys = _data["coordinates_y"][:10]
                            _zs = _data["coordinates_z"][:10]
                            _ps = _data["membrane_potentials"][:10]
                            _tuples = ", ".join(
                                f"({int(x)},{int(y)},{int(z)},{float(p):.3f})"
                                for x, y, z, p in zip(_xs, _ys, _zs, _ps)
                            )
                            logger.info(f"𒓉 [SHM] { _cid } xyzp first10: {_tuples}")
                    except Exception:
                        pass
            except Exception:
                pass

            neural_data = {}
            import numpy as _np
            for cid, data in cortical_areas.items():
                neural_data[cid] = {
                    "coordinates_x": _np.array(data["coordinates_x"], dtype=_np.uint16),
                    "coordinates_y": _np.array(data["coordinates_y"], dtype=_np.uint16),
                    "coordinates_z": _np.array(data["coordinates_z"], dtype=_np.uint16),
                    "membrane_potentials": _np.array(data["membrane_potentials"], dtype=_np.float32),
                }
            if neural_data:
                result = self.core_api.stimulate_neurons(neural_data)
                # # Detailed area mapping diagnostics
                # try:
                #     area_results = result.get("area_results") or {}
                #     for aid, meta in area_results.items():
                #         logger.info(
                #             f"𒓉 [SHM] Map {aid}: unique={meta.get('unique_coordinates')}, "
                #             f"found={meta.get('total_neurons_found')}, "
                #             f"stimulated={meta.get('stimulated_count')}, failed={meta.get('failed_count')}, "
                #             f"ok={meta.get('success')}, err={meta.get('error', '')}"
                #         )
                #     logger.info(
                #         f"𒓉 [SHM] Injection summary: injected={result.get('injected_count', 0)}, "
                #         f"total_stimulated={result.get('total_stimulated', 0)}"
                #     )
                # except Exception:
                #     pass
                try:
                    _tid_list = list(neural_data.keys())
                    _tid_preview = ", ".join(_tid_list[:6])
                    _more = len(_tid_list) - 6
                    _suffix = f" (+{_more} more)" if _more > 0 else ""
                    logger.info(
                        f"𒓉 [SHM] Sensory injected: areas={len(neural_data)}, points={neuron_count}, "
                        f"ok={result.get('success', False)}, targets=[{_tid_preview}{_suffix}]"
                    )
                except Exception:
                    pass
        except Exception as e:
            logger.debug(f"[SHM] Sensory decode failed: {e}")


    async def _handle_neuron_flat(
        self, header: NeuralDataHeader, payload: memoryview
    ) -> StreamResult:
        """Handle dense neural array data.

        Note: This method must be defined on SensoryNeuralStream, since it
        relies on self.core_api and self.neural_buffers.
        """
        try:
            if self._is_debug_npu_enabled():
                logger.info("Handling dense neural array data.")
            # Get buffer for this cortical area if available
            buffer = None
            if self.neural_buffers:
                buffer = self.neural_buffers.get_buffer_for_area(
                    str(header.cortical_area_id)
                )

            # Decode neural data views without copying
            neuron_count = header.neuron_count

            # Create numpy arrays as views into the payload
            #  Layout: [firing_rates(float32) | x_coords(int32) |
            #  y_coords(int32) | z_coords(int32)]
            bytes_per_neuron = 16  # 4 + 4 + 4 + 4

            if len(payload) < neuron_count * bytes_per_neuron:
                logger.error(f"Payload too small for {neuron_count} neurons")
                return StreamResult.DECODE_ERROR

            # Zero-copy numpy views
            offset = 0
            firing_rates = np.frombuffer(
                payload[offset : offset + neuron_count * 4], dtype=np.float32
            )
            offset += neuron_count * 4

            if header.has_coordinates:
                x_coords = np.frombuffer(
                    payload[offset : offset + neuron_count * 4], dtype=np.int32
                )
                offset += neuron_count * 4

                y_coords = np.frombuffer(
                    payload[offset : offset + neuron_count * 4], dtype=np.int32
                )
                offset += neuron_count * 4

                z_coords = np.frombuffer(
                    payload[offset : offset + neuron_count * 4], dtype=np.int32
                )
            else:
                x_coords = y_coords = z_coords = None

            # Convert to unified neural data format with SAFE uint16 conversion
            try:
                if self._is_debug_npu_enabled():
                    logger.info("Processing neural data.. .. ..")
                #  CRITICAL FIX: Validate coordinate ranges before conversion
                #  to uint16
                if x_coords is not None:
                    max_x = x_coords.max() if len(x_coords) > 0 else 0
                    if max_x > 65535:
                        logger.error(
                            f"Coordinate X values exceed uint16 range! Max: {max_x}, limit: 65535"
                        )
                        return StreamResult.DECODE_ERROR

                if y_coords is not None:
                    max_y = y_coords.max() if len(y_coords) > 0 else 0
                    if max_y > 65535:
                        logger.error(
                            f"Coordinate Y values exceed uint16 range! Max: {max_y}, limit: 65535"
                        )
                        return StreamResult.DECODE_ERROR

                if z_coords is not None:
                    max_z = z_coords.max() if len(z_coords) > 0 else 0
                    if max_z > 65535:
                        logger.error(
                            f"Coordinate Z values exceed uint16 range! Max: {max_z}, limit: 65535"
                        )
                        return StreamResult.DECODE_ERROR

                #  Build neural data with SAFE uint16 conversion (after
                #  validation)
                neural_data = {
                    str(header.cortical_area_id): {
                        "coordinates_x": (
                            x_coords.astype(np.uint16)
                            if x_coords is not None
                            else np.array([], dtype=np.uint16)
                        ),
                        "coordinates_y": (
                            y_coords.astype(np.uint16)
                            if y_coords is not None
                            else np.array([], dtype=np.uint16)
                        ),
                        "coordinates_z": (
                            z_coords.astype(np.uint16)
                            if z_coords is not None
                            else np.array([], dtype=np.uint16)
                        ),
                        "membrane_potentials": firing_rates.astype(np.float32),
                    }
                }

                if self._is_debug_npu_enabled():
                    logger.info(f"About to stimulate sensory data: {neural_data}")

                # Use unified stimulation method
                result = self.core_api.stimulate_neurons(neural_data)

                if not result.get("success", False):
                    self._stats["api_errors"] += 1
                    logger.error(
                        f"Unified stimulation failed: {result.get('error', 'Unknown error')}"
                    )
                    return StreamResult.API_ERROR

            except Exception as e:
                logger.error(f"Neural data injection failed: {e}")
                self._stats["api_errors"] += 1
                return StreamResult.API_ERROR

            return StreamResult.SUCCESS

        finally:
            # Release buffer if we acquired one
            if buffer and self.neural_buffers:
                self.neural_buffers.release_buffer(buffer)

    async def _handle_neuron_sparse(
        self, header: NeuralDataHeader, payload: memoryview
    ) -> StreamResult:
        """Handle sparse neural data format."""
        logger.warning("Sparse neural format not yet implemented")
        return StreamResult.UNKNOWN_PROTOCOL

    async def _handle_neuron_multi(
        self, header: NeuralDataHeader, payload: memoryview
    ) -> StreamResult:
        """Handle multi-modal neural data."""
        logger.warning("Multi-modal neural format not yet implemented")
        return StreamResult.UNKNOWN_PROTOCOL

    async def _handle_cortical_map(
        self, header: NeuralDataHeader, payload: memoryview
    ) -> StreamResult:
        """Handle cortical topology updates."""
        logger.warning("Cortical map updates not yet implemented")
        return StreamResult.UNKNOWN_PROTOCOL

    def get_stats(self) -> Dict[str, Any]:
        """Get stream statistics."""
        stats = self._stats.copy()

        # Add latest-only buffer stats
        stats["latest_only_buffer"] = {
            "active": self._latest_only_buffer['data'] is not None,
            "last_update_ns": self._latest_only_buffer['timestamp_ns'],
            "max_size_bytes": self._max_slot_size
        }

        # Add buffer pool stats if available
        if self.neural_buffers:
            stats["buffer_pools"] = self.neural_buffers.get_stats()

        # Add latest-only slot reader stats
        slot_stats = {
            "active_readers": len(getattr(self, '_slot_readers', {})),
            "total_reads": 0,
            "total_writes": 0, 
            "stale_rejections": 0,
            "overwrites": 0,
            "readers": {}
        }
        
        try:
            if hasattr(self, '_slot_lock') and hasattr(self, '_slot_readers'):
                with self._slot_lock:
                    for agent_id, reader in self._slot_readers.items():
                        reader_stats = {
                            "total_reads": reader.slot.stats.total_reads,
                            "total_writes": reader.slot.stats.total_writes,
                            "stale_rejections": reader.slot.stats.stale_data_rejections,
                            "overwrites": reader.slot.stats.overwrites,
                            "last_sequence": getattr(reader, "_last_sequence", 0)
                        }
                        slot_stats["readers"][agent_id] = reader_stats
                        slot_stats["total_reads"] += reader_stats["total_reads"]
                        slot_stats["total_writes"] += reader_stats["total_writes"] 
                        slot_stats["stale_rejections"] += reader_stats["stale_rejections"]
                        slot_stats["overwrites"] += reader_stats["overwrites"]
        except Exception:
            pass
            
        stats["latest_only_slots"] = slot_stats

        return stats

    
# _ShmRingReader class removed - replaced with LatestOnlyReader system 
# This eliminates the temporal pattern replay bug by using latest-only semantics
# instead of buffering historical data that causes loops after agent death

    async def _handle_neuron_flat(
        self, header: NeuralDataHeader, payload: memoryview
    ) -> StreamResult:
        """Handle dense neural array data."""
        try:
            if self._is_debug_npu_enabled():
                logger.info("Handling dense neural array data.")
            # Get buffer for this cortical area if available
            buffer = None
            if self.neural_buffers:
                buffer = self.neural_buffers.get_buffer_for_area(
                    str(header.cortical_area_id)
                )

            # Decode neural data views without copying
            neuron_count = header.neuron_count

            # Create numpy arrays as views into the payload
            #  Layout: [firing_rates(float32) | x_coords(int32) |
            #  y_coords(int32) | z_coords(int32)]
            bytes_per_neuron = 16  # 4 + 4 + 4 + 4

            if len(payload) < neuron_count * bytes_per_neuron:
                logger.error(f"Payload too small for {neuron_count} neurons")
                return StreamResult.DECODE_ERROR

            # Zero-copy numpy views
            offset = 0
            firing_rates = np.frombuffer(
                payload[offset : offset + neuron_count * 4], dtype=np.float32
            )
            offset += neuron_count * 4

            if header.has_coordinates:
                x_coords = np.frombuffer(
                    payload[offset : offset + neuron_count * 4], dtype=np.int32
                )
                offset += neuron_count * 4

                y_coords = np.frombuffer(
                    payload[offset : offset + neuron_count * 4], dtype=np.int32
                )
                offset += neuron_count * 4

                z_coords = np.frombuffer(
                    payload[offset : offset + neuron_count * 4], dtype=np.int32
                )
            else:
                x_coords = y_coords = z_coords = None

            # Convert to unified neural data format with SAFE uint16 conversion
            try:
                if self._is_debug_npu_enabled():
                    logger.info("Processing neural data.. .. ..")
                #  CRITICAL FIX: Validate coordinate ranges before conversion
                #  to uint16
                if x_coords is not None:
                    max_x = x_coords.max() if len(x_coords) > 0 else 0
                    if max_x > 65535:
                        logger.error(
                            f"Coordinate X values exceed uint16 range! Max: {max_x}, limit: 65535"
                        )
                        return StreamResult.DECODE_ERROR

                if y_coords is not None:
                    max_y = y_coords.max() if len(y_coords) > 0 else 0
                    if max_y > 65535:
                        logger.error(
                            f"Coordinate Y values exceed uint16 range! Max: {max_y}, limit: 65535"
                        )
                        return StreamResult.DECODE_ERROR

                if z_coords is not None:
                    max_z = z_coords.max() if len(z_coords) > 0 else 0
                    if max_z > 65535:
                        logger.error(
                            f"Coordinate Z values exceed uint16 range! Max: {max_z}, limit: 65535"
                        )
                        return StreamResult.DECODE_ERROR

                #  Build neural data with SAFE uint16 conversion (after
                #  validation)
                neural_data = {
                    str(header.cortical_area_id): {
                        "coordinates_x": (
                            x_coords.astype(np.uint16)
                            if x_coords is not None
                            else np.array([], dtype=np.uint16)
                        ),
                        "coordinates_y": (
                            y_coords.astype(np.uint16)
                            if y_coords is not None
                            else np.array([], dtype=np.uint16)
                        ),
                        "coordinates_z": (
                            z_coords.astype(np.uint16)
                            if z_coords is not None
                            else np.array([], dtype=np.uint16)
                        ),
                        "membrane_potentials": firing_rates.astype(np.float32),
                    }
                }

                if self._is_debug_npu_enabled():
                    logger.info(f"About to stimulate sensory data: {neural_data}")

                # Use unified stimulation method
                result = self.core_api.stimulate_neurons(neural_data)

                if not result.get("success", False):
                    self._stats["api_errors"] += 1
                    logger.error(
                        f"Unified stimulation failed: {result.get('error', 'Unknown error')}"
                    )
                    return StreamResult.API_ERROR

            except Exception as e:
                logger.error(f"Neural data injection failed: {e}")
                self._stats["api_errors"] += 1
                return StreamResult.API_ERROR

            return StreamResult.SUCCESS

        finally:
            # Release buffer if we acquired one
            if buffer and self.neural_buffers:
                self.neural_buffers.release_buffer(buffer)

    async def _handle_neuron_sparse(
        self, header: NeuralDataHeader, payload: memoryview
    ) -> StreamResult:
        """Handle sparse neural data format."""
        # TODO: Implement sparse format decoding
        logger.warning("Sparse neural format not yet implemented")
        return StreamResult.UNKNOWN_PROTOCOL

    async def _handle_neuron_multi(
        self, header: NeuralDataHeader, payload: memoryview
    ) -> StreamResult:
        """Handle multi-modal neural data."""
        # TODO: Implement multi-modal format
        logger.warning("Multi-modal neural format not yet implemented")
        return StreamResult.UNKNOWN_PROTOCOL

    async def _handle_cortical_map(
        self, header: NeuralDataHeader, payload: memoryview
    ) -> StreamResult:
        """Handle cortical topology updates."""
        # TODO: Implement topology updates
        logger.warning("Cortical map updates not yet implemented")
        return StreamResult.UNKNOWN_PROTOCOL

    def get_stats(self) -> Dict[str, Any]:
        """Get stream statistics."""
        stats = self._stats.copy()

        # Add latest-only buffer stats
        stats["latest_only_buffer"] = {
            "active": self._latest_only_buffer['data'] is not None,
            "last_update_ns": self._latest_only_buffer['timestamp_ns'],
            "max_size_bytes": self._max_slot_size
        }

        # Add buffer pool stats if available
        if self.neural_buffers:
            stats["buffer_pools"] = self.neural_buffers.get_stats()

        return stats
