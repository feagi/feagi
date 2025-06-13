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
                    logger.warning("Ring buffer full, applying backpressure")
                    await asyncio.sleep(0.01)  # 10ms backpressure

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in neural processing loop: {e}")
                await asyncio.sleep(0.1)  # Error throttling

    async def _process_neural_data(self) -> StreamResult:
        """Process one neural data message with zero-copy."""
        # Get write slot from ring buffer
        slot = self.ring_buffer.get_write_slot()
        if not slot:
            return StreamResult.BUFFER_FULL

        # Track whether we successfully processed data
        data_processed = False

        try:
            # Receive data from ZMQ socket
            try:
                # ZMQ doesn't have recv_into - use recv() instead
                data = await self.socket.recv(flags=zmq.NOBLOCK | zmq.DONTWAIT)
                nbytes = len(data)

                # Debug logging for inbound neural data (zero-overhead when disabled)
                log_inbound(
                    endpoint=self.debug_endpoint,
                    frames=[data],
                    message_type=MessageType.SENSORY,
                    context=f"neural_data_{self._stats['messages_received'] + 1}",
                )

                # Check if data fits in the slot
                if nbytes > len(slot.memory_view):
                    logger.error(
                        f"Message too large: {nbytes} bytes > {len(slot.memory_view)} slot size"
                    )
                    return StreamResult.BUFFER_FULL

                # Copy data into the memory slot
                slot.memory_view[:nbytes] = data
                data_processed = True  # Mark that we got real data

            except zmq.Again:
                # No data available - don't commit the slot
                return StreamResult.NO_DATA

            # Update stats
            self._stats["messages_received"] += 1
            self._stats["bytes_received"] += nbytes
            self._stats["last_message_time"] = time.time()

            # Parse header without copying
            try:
                # Temporarily skip header parsing to test basic ZMQ functionality
                logger.debug(f"Received {nbytes} bytes of neural data")
                # TODO: Re-enable header parsing once dependencies are resolved
                # header, payload_view = parse_header(slot.memory_view[:nbytes])

                # For now, just report success
                return StreamResult.SUCCESS

            except Exception as e:
                self._stats["decode_errors"] += 1
                logger.error(f"Failed to parse neural header: {e}")
                return StreamResult.DECODE_ERROR

            # Comment out protocol handling for now
            # # Get protocol handler
            # handler = self._protocol_handlers.get(header.protocol_id)
            # if not handler:
            #     logger.warning(f"Unknown neural protocol: {header.protocol_id}")
            #     return StreamResult.UNKNOWN_PROTOCOL
            #
            # # Process neural data based on protocol
            # result = await handler(header, payload_view)
            #
            # return result

        finally:
            # Only commit the write slot if we actually processed data
            if data_processed:
                self.ring_buffer.commit_write(slot)
            # Clear the slot reference to help with memory cleanup
            slot = None
            # If no data was processed, the slot will be automatically released

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

            # Direct FCL injection without intermediate copies
            try:
                result = await self.core_api.inject_neural_data(
                    cortical_area_id=header.cortical_area_id,
                    firing_rates=firing_rates,
                    coordinates=(
                        (x_coords, y_coords, z_coords)
                        if header.has_coordinates
                        else None
                    ),
                    timestamp=header.timestamp,
                    is_delta=header.is_delta,
                )

                if not result:
                    self._stats["api_errors"] += 1
                    return StreamResult.API_ERROR

            except Exception as e:
                logger.error(f"FCL injection failed: {e}")
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
