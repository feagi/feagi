"""High-performance sensory stream for neural data ingestion.

This stream implements zero-copy neural data reception from FEAGI_Connector
with pre-allocated buffers and platform-specific optimizations.
"""

import asyncio
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
from ..platform import optimize_socket_for_neural_data

logger = setup_logger(__name__)


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

        # Zero-copy ring buffer for incoming data
        # Reduce backlog for real-time behavior
        # Ensure sufficient slot size for large sensory frames while keeping slot count bounded
        self.ring_buffer = ZeroCopyRingBuffer(
            slots=min(ring_buffer_slots, 128),
            slot_size=max(524288, int(slot_size)),
            use_shared_memory=True,
        )

        # Neural-specific buffer pools
        if cortical_config:
            self.neural_buffers = NeuralBufferPool(cortical_config)
        else:
            self.neural_buffers = None

        # Socket setup
        self.socket = self._setup_socket()

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

        # Protocol handlers
        self._protocol_handlers = {
            NeuralProtocolID.NEURON_FLAT: self._handle_neuron_flat,
            NeuralProtocolID.NEURON_SPARSE: self._handle_neuron_sparse,
            NeuralProtocolID.NEURON_MULTI: self._handle_neuron_multi,
            NeuralProtocolID.CORTICAL_MAP: self._handle_cortical_map,
        }

        # Optional SHM readers (per-agent sensory/neurons_stream)
        self._shm_readers: Dict[str, _ShmRingReader] = {}
        try:
            from feagi.core.state_manager import FeagiStateManager

            sm = FeagiStateManager.instance()
            agent_map = getattr(sm, "_agent_shared_memory", {})
            for aid, mapping in agent_map.items():
                # Support both new capability key 'sensory' and legacy 'neurons_stream'
                p = mapping.get("sensory") or mapping.get("neurons_stream")
                if p:
                    try:
                        self._shm_readers[aid] = _ShmRingReader(Path(p))
                        logger.info(f"𒓉 [SHM] Sensory stream reading from agent {aid}: {p}")
                    except Exception as e:
                        logger.warning(f"[SHM] Failed to open sensory SHM for {aid}: {e}")
        except Exception as e:
            logger.info(f"[SHM] Sensory SHM registry unavailable; using ZMQ only ({e})")

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
        self.running = True

        # Start processing tasks
        self._process_task = asyncio.create_task(self._process_loop())
        # Always start SHM poller; it will attach readers dynamically
        self._shm_task = asyncio.create_task(self._process_shm_loop())

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
        if hasattr(self, "_shm_task"):
            self._shm_task.cancel()
            try:
                await self._shm_task
            except asyncio.CancelledError:
                pass
            except Exception as e:
                logger.warning(f"Error cancelling shm task: {e}")

        # Close socket with error handling
        if self.socket:
            try:
                self.socket.close()
            except Exception as e:
                logger.warning(f"Error closing socket: {e}")

        # CRITICAL: Always ensure ring buffer cleanup
        try:
            self.ring_buffer.close()
        except Exception as e:
            logger.error(f"Error closing ring buffer: {e}")
            # Still try to cleanup - this is critical for shared memory
            try:
                if hasattr(self.ring_buffer, "__del__"):
                    self.ring_buffer.__del__()
            except Exception as e2:
                logger.error(f"Error in ring buffer destructor: {e2}")

        # Close buffer pools with error handling
        if self.neural_buffers:
            try:
                self.neural_buffers.close()
            except Exception as e:
                logger.warning(f"Error closing neural buffers: {e}")

        logger.info("Neural sensory stream stopped")

        # Close SHM readers
        for reader in self._shm_readers.values():
            try:
                reader.close()
            except Exception:
                pass
        self._shm_readers.clear()

    def __del__(self):
        """Destructor to ensure cleanup even if stop() isn't called
        explicitly."""
        try:
            #  Only attempt cleanup if we haven't already cleaned up and are
            #  still running
            if getattr(self, "running", False):
                # Try to stop gracefully but don't await (we're in destructor)
                if hasattr(self, "ring_buffer"):
                    try:
                        self.ring_buffer.close()
                    except Exception:
                        pass

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
                        f"used={self.ring_buffer.used_slots}/{self.ring_buffer.slots}, "
                        f"full_count={self.ring_buffer.stats.buffer_full_count}"
                    )
                    await asyncio.sleep(0.01)  # 10ms backpressure

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in neural processing loop: {e}")
                await asyncio.sleep(0.1)  # Error throttling

    async def _process_shm_loop(self) -> None:
        """Poll SHM ring buffers and process latest payloads."""
        while self.running:
            try:
                # Refresh readers from StateManager in case of new agents
                try:
                    from feagi.core.state_manager import FeagiStateManager

                    sm = FeagiStateManager.instance()
                    agent_map = getattr(sm, "_agent_shared_memory", {})
                    # Add new readers
                    for aid, mapping in agent_map.items():
                        if aid not in self._shm_readers:
                            p = mapping.get("sensory") or mapping.get("neurons_stream")
                            if p:
                                try:
                                    self._shm_readers[aid] = _ShmRingReader(Path(p))
                                    logger.info(f"𒓉 [SHM] Sensory stream reading from agent {aid}: {p}")
                                except Exception:
                                    pass
                    # Remove stale readers
                    for aid in list(self._shm_readers.keys()):
                        if aid not in agent_map:
                            try:
                                self._shm_readers[aid].close()
                            except Exception:
                                pass
                            self._shm_readers.pop(aid, None)
                except Exception:
                    pass

                for aid, reader in list(self._shm_readers.items()):
                    payload = reader.read_latest()
                    if payload:
                        try:
                            logger.info(f"𒓉 [SHM] Sensory payload from {aid}: {len(payload)} bytes")
                            await self._process_neural_payload_bytes(payload)
                            self._stats["shm_payloads"] = self._stats.get("shm_payloads", 0) + 1
                        except Exception as pe:
                            logger.debug(f"[SHM] Sensory payload decode error from {aid}: {pe}")
                await asyncio.sleep(0.005)  # 5ms poll
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.debug(f"[SHM] Sensory poll error: {e}")
                await asyncio.sleep(0.05)

    async def _process_neural_data(self) -> StreamResult:
        """Process incoming neural data - feagi_data_processing format only."""
        
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
        
        
        slot = None
        data_processed = False

        try:
            # Get write slot from ring buffer
            slot = self.ring_buffer.get_write_slot()
            if not slot:
                self._stats["buffer_full"] += 1
                # DIAGNOSTIC: Log buffer state when full
                logger.warning(
                    f"Ring buffer full! Used slots: {self.ring_buffer.used_slots}/{self.ring_buffer.slots}, "
                    f"Write index: {self.ring_buffer.write_index.value}, "
                    f"Read index: {self.ring_buffer.read_index.value}"
                )
                return StreamResult.BUFFER_FULL

            # ZMQ receive - async sockets use recv() not recv_into()
            try:
                data = await self.socket.recv(flags=zmq.NOBLOCK)
                nbytes = len(data)

                # Check if data fits in the slot
                if nbytes > len(slot.memory_view):
                    logger.error(
                        f"Message too large: {nbytes} bytes > {len(slot.memory_view)} slot size"
                    )
                    return StreamResult.BUFFER_FULL

                # Copy data into the memory slot
                slot.memory_view[:nbytes] = data
                data_processed = True

            except zmq.Again:
                # No data available
                return StreamResult.NO_DATA

            self._stats["messages_received"] += 1
            self._stats["bytes_received"] += nbytes
            self._stats["last_message_time"] = time.time()

            # Decode feagi_data_processing format
            try:
                import feagi_rust_py_libs as fdp

                # Create FeagiByteStructure directly from raw bytes (modern API)
                raw_bytes = slot.memory_view[:nbytes].tobytes()
                byte_structure = fdp.data_serialization.FeagiByteStructure(raw_bytes)

                # Get structure type using FEAGI's API
                structure_type = byte_structure.structure_type

                if structure_type != 11:
                    logger.warning(
                        f"Unexpected structure type: {structure_type}, expected 11 (NEURON_CATEGORIES)"
                    )
                    logger.debug(
                        f"Raw data (first 20 bytes): {raw_bytes[:20].hex()}"
                    )
                    return StreamResult.SUCCESS
                print(">> > >", byte_structure)
                # Create CorticalMappedXYZPNeuronData from the byte structure (modern API)
                cortical_mapped = fdp.data_structures.neurons.xyzp.CorticalMappedXYZPNeuronData.new_from_feagi_byte_structure(
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
                        f"✅ Decoded {neuron_count} neurons using feagi_data_processing direct decoding"
                    )

                # Convert back to numpy arrays for SIMD performance
                import numpy as np

                neural_data = {}

                for cortical_id, data in cortical_areas.items():
                    neural_data[cortical_id] = {
                        "coordinates_x": np.array(
                            data["coordinates_x"], dtype=np.uint16
                        ),
                        "coordinates_y": np.array(
                            data["coordinates_y"], dtype=np.uint16
                        ),
                        "coordinates_z": np.array(
                            data["coordinates_z"], dtype=np.uint16
                        ),
                        "membrane_potentials": np.array(
                            data["membrane_potentials"], dtype=np.float32
                        ),
                    }
                    if self._is_debug_npu_enabled():
                        logger.info(
                            f"🧠 Cortical area {cortical_id}: {len(data['coordinates_x'])} neurons"
                        )

                # Inject into FCL using SIMD-optimized stimulate_neurons method
                result = self.core_api.stimulate_neurons(neural_data)
                # Detailed area mapping diagnostics to aid BV issues
                try:
                    area_results = result.get("area_results") or {}
                    for aid, meta in area_results.items():
                        logger.info(
                            f"𒓉 [SHM] Map {aid}: unique={meta.get('unique_coordinates')}, "
                            f"found={meta.get('total_neurons_found')}, "
                            f"stimulated={meta.get('stimulated_count')}, failed={meta.get('failed_count')}, "
                            f"ok={meta.get('success')}, err={meta.get('error', '')}"
                        )
                    logger.info(
                        f"𒓉 [SHM] Injection summary: injected={result.get('injected_count', 0)}, "
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
                    f"❌ Failed to decode feagi_data_processing format: {e}"
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
            #  Only commit the write slot if we actually processed data -
            #  CORRECT METHOD
            if data_processed and slot:
                self.ring_buffer.commit_write(slot)

                #  CRITICAL FIX: Auto-drain ring buffer after successful
                #  processing
                # Since we process data immediately (not in separate consumer),
                # we need to advance read index to prevent buffer_full errors
                read_slot = self.ring_buffer.get_read_slot()
                if read_slot and read_slot.index == slot.index:
                    self.ring_buffer.commit_read(read_slot)

            # Clear the slot reference to help with memory cleanup
            slot = None

    async def _process_neural_payload_bytes(self, raw_bytes: bytes) -> None:
        """Process neural payload provided as bytes (from SHM)."""
        try:
            # Zero-serialize fast path: 'ZS1N' magic
            if len(raw_bytes) >= 8 and raw_bytes[:4] == b"ZS1N":
                import struct as _struct
                import numpy as _np

                try:
                    # Header: magic(4), version u8, num_areas u8, pad u16
                    version = raw_bytes[4]
                    num_areas = raw_bytes[5]
                    offset = 8

                    cortical_areas: Dict[str, Dict[str, Any]] = {}
                    total_points = 0

                    for _ in range(num_areas):
                        if offset + 6 > len(raw_bytes):
                            raise ValueError("ZS1N truncated before area id")
                        cid_bytes = raw_bytes[offset:offset+6]
                        offset += 6
                        if offset + 4 > len(raw_bytes):
                            raise ValueError("ZS1N truncated before count")
                        (count,) = _struct.unpack_from("<I", raw_bytes, offset)
                        offset += 4
                        # Required sizes
                        need = count * 2 * 3 + count * 4  # x,y,z (u16) + p (f32)
                        if offset + need > len(raw_bytes):
                            raise ValueError("ZS1N truncated in arrays")

                        # Slices
                        xs_b = raw_bytes[offset:offset + count*2]; offset += count*2
                        ys_b = raw_bytes[offset:offset + count*2]; offset += count*2
                        zs_b = raw_bytes[offset:offset + count*2]; offset += count*2
                        ps_b = raw_bytes[offset:offset + count*4]; offset += count*4

                        # Views
                        xs = _np.frombuffer(xs_b, dtype=_np.uint16, count=count)
                        ys = _np.frombuffer(ys_b, dtype=_np.uint16, count=count)
                        zs = _np.frombuffer(zs_b, dtype=_np.uint16, count=count)
                        ps = _np.frombuffer(ps_b, dtype=_np.float32, count=count)

                        cid = cid_bytes.decode("ascii", errors="ignore").strip().strip("'\"")
                        if not cid:
                            continue
                        cortical_areas[cid] = {
                            "coordinates_x": xs.copy(),
                            "coordinates_y": ys.copy(),
                            "coordinates_z": zs.copy(),
                            "membrane_potentials": ps.copy(),
                        }
                        total_points += int(count)

                    # Build neural_data
                    neural_data = {
                        cid: {
                            "coordinates_x": _np.asarray(data["coordinates_x"], dtype=_np.uint16),
                            "coordinates_y": _np.asarray(data["coordinates_y"], dtype=_np.uint16),
                            "coordinates_z": _np.asarray(data["coordinates_z"], dtype=_np.uint16),
                            "membrane_potentials": _np.asarray(data["membrane_potentials"], dtype=_np.float32),
                        }
                        for cid, data in cortical_areas.items()
                    }

                    if neural_data:
                        result = self.core_api.stimulate_neurons(neural_data)
                        try:
                            area_results = result.get("area_results") or {}
                            for aid, meta in area_results.items():
                                logger.info(
                                    f"𒓉 [ZS] Map {aid}: unique={meta.get('unique_coordinates')}, "
                                    f"found={meta.get('total_neurons_found')}, "
                                    f"stimulated={meta.get('stimulated_count')}, failed={meta.get('failed_count')}, "
                                    f"ok={meta.get('success')}, err={meta.get('error', '')}"
                                )
                            logger.info(
                                f"𒓉 [ZS] Injection summary: injected={result.get('injected_count', 0)}, "
                                f"total_stimulated={result.get('total_stimulated', 0)}"
                            )
                        except Exception:
                            pass
                    return
                except Exception as e:
                    logger.debug(f"[ZS] Zero-serialize parse failed, falling back: {e}")

            # Fallback: standard feagi_data_processing byte structure
            import feagi_rust_py_libs as fdp

            byte_structure = fdp.data_serialization.FeagiByteStructure(raw_bytes)
            structure_type = byte_structure.structure_type
            if structure_type != 11:
                return

            print(">> >>", byte_structure)
            cortical_mapped = fdp.data_structures.neurons.xyzp.CorticalMappedXYZPNeuronData.new_from_feagi_byte_structure(
                byte_structure
            )
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
                # Detailed area mapping diagnostics
                try:
                    area_results = result.get("area_results") or {}
                    for aid, meta in area_results.items():
                        logger.info(
                            f"𒓉 [SHM] Map {aid}: unique={meta.get('unique_coordinates')}, "
                            f"found={meta.get('total_neurons_found')}, "
                            f"stimulated={meta.get('stimulated_count')}, failed={meta.get('failed_count')}, "
                            f"ok={meta.get('success')}, err={meta.get('error', '')}"
                        )
                    logger.info(
                        f"𒓉 [SHM] Injection summary: injected={result.get('injected_count', 0)}, "
                        f"total_stimulated={result.get('total_stimulated', 0)}"
                    )
                except Exception:
                    pass
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

        # Add ring buffer stats
        stats["ring_buffer"] = {
            "available_slots": self.ring_buffer.available_slots,
            "used_slots": self.ring_buffer.used_slots,
            "stats": self.ring_buffer.stats.__dict__,
        }

        # Add buffer pool stats if available
        if self.neural_buffers:
            stats["buffer_pools"] = self.neural_buffers.get_stats()

        return stats

    
class _ShmRingReader:
    """Reader for variable-length byte payloads ring buffer.

    Header: '<8sIIIQI' => magic, version, num_slots, slot_size, frame_seq, write_index
    Each slot: u32 length + data
    """

    MAGIC_SET = {b"FEAGIVIS", b"FEAGIBIN", b"FEAGIMOT"}
    HEADER_SIZE = 256
    HEADER_FMT = "<8sIIIQI"

    def __init__(self, path: Path):
        self.path = Path(path)
        self._mm = None
        self._fd = None
        self.num_slots = 0
        self.slot_size = 0
        self.last_seq = -1
        self._open()

    def _open(self) -> None:
        self._fd = _os.open(str(self.path), _os.O_RDWR)
        self._mm = _mmap.mmap(self._fd, 0, access=_mmap.ACCESS_READ)
        self._mm.seek(0)
        header = self._mm.read(self.HEADER_SIZE)
        magic, version, num_slots, slot_size, frame_seq, write_index = _struct.unpack(
            self.HEADER_FMT, header[: _struct.calcsize(self.HEADER_FMT)]
        )
        if magic not in self.MAGIC_SET:
            raise ValueError("Invalid SHM magic")
        self.num_slots = int(num_slots)
        self.slot_size = int(slot_size)
        self.last_seq = int(frame_seq) - 1

    def read_latest(self) -> Optional[bytes]:
        try:
            self._mm.seek(0)
            header = self._mm.read(self.HEADER_SIZE)
            magic, version, num_slots, slot_size, frame_seq, write_index = _struct.unpack(
                self.HEADER_FMT, header[: _struct.calcsize(self.HEADER_FMT)]
            )
            if int(frame_seq) <= self.last_seq:
                return None
            wi = int(write_index)
            idx = (wi - 1) % self.num_slots
            slot_off = self.HEADER_SIZE + idx * self.slot_size
            self._mm.seek(slot_off)
            (length,) = _struct.unpack("<I", self._mm.read(4))
            if length <= 0 or length > (self.slot_size - 4):
                self.last_seq = int(frame_seq)
                return None
            data = self._mm.read(length)
            self.last_seq = int(frame_seq)
            return bytes(data)
        except Exception:
            return None

    def close(self) -> None:
        try:
            if self._mm:
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

        # Add ring buffer stats
        stats["ring_buffer"] = {
            "available_slots": self.ring_buffer.available_slots,
            "used_slots": self.ring_buffer.used_slots,
            "stats": self.ring_buffer.stats.__dict__,
        }

        # Add buffer pool stats if available
        if self.neural_buffers:
            stats["buffer_pools"] = self.neural_buffers.get_stats()

        return stats
