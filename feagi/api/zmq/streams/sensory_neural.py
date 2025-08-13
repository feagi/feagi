"""
High-performance sensory stream for neural data ingestion.

This stream implements zero-copy neural data reception from FEAGI_Connector
with pre-allocated buffers and platform-specific optimizations.
"""

import asyncio
import time
from enum import IntEnum
from typing import Any, Dict, Optional

import numpy as np
import zmq
import zmq.asyncio
import json

from feagi.utils.logger import setup_logger
from feagi.utils.zmq_debug import MessageType, log_inbound

from ..memory import NeuralBufferPool
from ..neural import NeuralDataHeader, NeuralProtocolID, ZeroCopyRingBuffer
from ..platform import optimize_socket_for_neural_data

logger = setup_logger()


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
    """
    High-performance neural data ingestion from FEAGI_Connector.

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
        slot_size: int = 1048576,  # 1MB per slot
        cortical_config: Optional[Dict[str, Dict[str, Any]]] = None,
    ):
        """
        Initialize neural sensory stream.

        Args:
            core_api: Core API service for FCL injection
            host: Host to bind to
            port: Port to bind to
            context: ZMQ context (optional)
            ring_buffer_slots: Number of ring buffer slots
            slot_size: Size of each slot in bytes
            cortical_config: Cortical area configuration for buffer pools
        """
        self.core_api = core_api
        self.host = host
        self.port = port
        self.context = context or zmq.asyncio.Context.instance()

        # Debug endpoint for logging
        self.debug_endpoint = f"tcp://{self.host}:{self.port}"

        # Zero-copy ring buffer for incoming data
        self.ring_buffer = ZeroCopyRingBuffer(
            slots=ring_buffer_slots, slot_size=slot_size, use_shared_memory=True
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

        logger.info(f"Starting neural sensory stream on {self.host}:{self.port}")
        self.running = True

        # Start processing task
        self._process_task = asyncio.create_task(self._process_loop())

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

    def __del__(self):
        """Destructor to ensure cleanup even if stop() isn't called explicitly."""
        try:
            # Only attempt cleanup if we haven't already cleaned up and are still running
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
                    logger.warning(f"Ring buffer full, applying backpressure. Buffer stats: "
                                 f"used={self.ring_buffer.used_slots}/{self.ring_buffer.slots}, "
                                 f"full_count={self.ring_buffer.stats.buffer_full_count}")
                    await asyncio.sleep(0.01)  # 10ms backpressure

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in neural processing loop: {e}")
                await asyncio.sleep(0.1)  # Error throttling

    async def _process_neural_data(self) -> StreamResult:
        """Process incoming neural data - feagi_data_processing format only."""
        slot = None
        data_processed = False
        
        try:
            # Get write slot from ring buffer
            slot = self.ring_buffer.get_write_slot()
            if not slot:
                self._stats["buffer_full"] += 1
                # DIAGNOSTIC: Log buffer state when full
                logger.warning(f"Ring buffer full! Used slots: {self.ring_buffer.used_slots}/{self.ring_buffer.slots}, "
                             f"Write index: {self.ring_buffer.write_index.value}, "
                             f"Read index: {self.ring_buffer.read_index.value}")
                return StreamResult.BUFFER_FULL

            # ZMQ receive - async sockets use recv() not recv_into()
            try:
                data = await self.socket.recv(flags=zmq.NOBLOCK)
                nbytes = len(data)
                
                # Check if data fits in the slot
                if nbytes > len(slot.memory_view):
                    logger.error(f"Message too large: {nbytes} bytes > {len(slot.memory_view)} slot size")
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
                import feagi_data_processing as fdp
                
                # Create FeagiByteStructure directly from raw bytes
                raw_bytes = slot.memory_view[:nbytes].tobytes()
                byte_structure = fdp.io_processing.bytes.FeagiByteStructure(raw_bytes)
                
                # Get structure type using FEAGI's API
                structure_type = byte_structure.structure_type
                
                if structure_type != 11:
                    logger.warning(f"Unexpected structure type: {structure_type}, expected 11 (NEURON_CATEGORIES)")
                    logger.debug(f"Raw data (first 20 bytes): {raw_bytes[:20].hex()}")
                    return StreamResult.SUCCESS
                
                # Create CorticalMappedXYZPNeuronData from the byte structure
                cortical_mapped = fdp.neuron_data.xyzp.CorticalMappedXYZPNeuronData.new_from_feagi_byte_structure(byte_structure)
                
                # Extract neuron data using iter_full()
                cortical_areas = {}
                neuron_count = 0
                
                for cortical_id, neuron_arrays in cortical_mapped.iter_full():
                    # neuron_arrays is a tuple: (x_coords, y_coords, z_coords, potentials)
                    x_coords, y_coords, z_coords, potentials = neuron_arrays
                    
                    if cortical_id not in cortical_areas:
                        cortical_areas[cortical_id] = {
                            'coordinates_x': [],
                            'coordinates_y': [],
                            'coordinates_z': [],
                            'membrane_potentials': []
                        }
                    
                    # Convert numpy arrays to lists and extend
                    cortical_areas[cortical_id]['coordinates_x'].extend(x_coords.tolist())
                    cortical_areas[cortical_id]['coordinates_y'].extend(y_coords.tolist())
                    cortical_areas[cortical_id]['coordinates_z'].extend(z_coords.tolist())
                    cortical_areas[cortical_id]['membrane_potentials'].extend(potentials.tolist())
                    
                    neuron_count += len(x_coords)
                
                logger.info(f"✅ Decoded {neuron_count} neurons using feagi_data_processing direct decoding")
                
                # Convert back to numpy arrays for SIMD performance
                import numpy as np
                neural_data = {}
                
                for cortical_id, data in cortical_areas.items():
                    neural_data[cortical_id] = {
                        'coordinates_x': np.array(data['coordinates_x'], dtype=np.uint16),
                        'coordinates_y': np.array(data['coordinates_y'], dtype=np.uint16),
                        'coordinates_z': np.array(data['coordinates_z'], dtype=np.uint16),
                        'membrane_potentials': np.array(data['membrane_potentials'], dtype=np.float32)
                    }
                    logger.info(f"🧠 Cortical area {cortical_id}: {len(data['coordinates_x'])} neurons")
                
                # Inject into FCL using SIMD-optimized stimulate_neurons method
                result = self.core_api.stimulate_neurons(neural_data)
                
                if result.get("success", False):
                    logger.info(f"🧠 Successfully injected {neuron_count} neurons into FCL across {len(neural_data)} cortical areas (VECTORIZED)")
                    return StreamResult.SUCCESS
                else:
                    logger.error(f"❌ FCL injection failed: {result.get('error', 'Unknown error')}")
                    self._stats["api_errors"] += 1
                    return StreamResult.API_ERROR
                    
            except Exception as e:
                logger.error(f"❌ Failed to decode feagi_data_processing format: {e}")
                import traceback
                logger.error(f"🔍 Decode traceback: {traceback.format_exc()}")
                self._stats["decode_errors"] += 1
                return StreamResult.DECODE_ERROR

        except Exception as e:
            self._stats["decode_errors"] += 1
            # Provide more context for buffer-related errors
            if "buffer" in str(e).lower():
                logger.error(f"Failed to process neural data (buffer issue): {e}. "
                           f"Ring buffer state: used={self.ring_buffer.used_slots}/{self.ring_buffer.slots}")
            else:
                logger.error(f"Failed to process neural data: {e}")
            return StreamResult.DECODE_ERROR

        finally:
            # Only commit the write slot if we actually processed data - CORRECT METHOD
            if data_processed and slot:
                self.ring_buffer.commit_write(slot)
                
                # CRITICAL FIX: Auto-drain ring buffer after successful processing
                # Since we process data immediately (not in separate consumer), 
                # we need to advance read index to prevent buffer_full errors
                read_slot = self.ring_buffer.get_read_slot()
                if read_slot and read_slot.index == slot.index:
                    self.ring_buffer.commit_read(read_slot)
                    
            # Clear the slot reference to help with memory cleanup
            slot = None

    async def _handle_neuron_flat(
        self, header: NeuralDataHeader, payload: memoryview
    ) -> StreamResult:
        """Handle dense neural array data."""
        try:
            # Get buffer for this cortical area if available
            buffer = None
            if self.neural_buffers:
                buffer = self.neural_buffers.get_buffer_for_area(
                    str(header.cortical_area_id)
                )

            # Decode neural data views without copying
            neuron_count = header.neuron_count

            # Create numpy arrays as views into the payload
            # Layout: [firing_rates(float32) | x_coords(int32) | y_coords(int32) | z_coords(int32)]
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
                # CRITICAL FIX: Validate coordinate ranges before conversion to uint16
                if x_coords is not None:
                    max_x = x_coords.max() if len(x_coords) > 0 else 0
                    if max_x > 65535:
                        logger.error(f"Coordinate X values exceed uint16 range! Max: {max_x}, limit: 65535")
                        return StreamResult.DECODE_ERROR
                        
                if y_coords is not None:
                    max_y = y_coords.max() if len(y_coords) > 0 else 0
                    if max_y > 65535:
                        logger.error(f"Coordinate Y values exceed uint16 range! Max: {max_y}, limit: 65535")
                        return StreamResult.DECODE_ERROR
                        
                if z_coords is not None:
                    max_z = z_coords.max() if len(z_coords) > 0 else 0
                    if max_z > 65535:
                        logger.error(f"Coordinate Z values exceed uint16 range! Max: {max_z}, limit: 65535")
                        return StreamResult.DECODE_ERROR

                # Build neural data with SAFE uint16 conversion (after validation)
                neural_data = {
                    str(header.cortical_area_id): {
                        'coordinates_x': x_coords.astype(np.uint16) if x_coords is not None else np.array([], dtype=np.uint16),
                        'coordinates_y': y_coords.astype(np.uint16) if y_coords is not None else np.array([], dtype=np.uint16),
                        'coordinates_z': z_coords.astype(np.uint16) if z_coords is not None else np.array([], dtype=np.uint16),
                        'membrane_potentials': firing_rates.astype(np.float32)
                    }
                }
                
                # Use unified stimulation method
                result = self.core_api.stimulate_neurons(neural_data)

                if not result.get("success", False):
                    self._stats["api_errors"] += 1
                    logger.error(f"Unified stimulation failed: {result.get('error', 'Unknown error')}")
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
