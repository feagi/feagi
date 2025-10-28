"""
Rust-backed sensory client for FEAGI.

This implementation encodes neuron dictionaries into FEAGI's binary XYZP
format using feagi_rust_py_libs and sends the payload via ZeroMQ PUSH.
"""

from __future__ import annotations

import logging
from typing import Dict, Optional, Tuple

import numpy as np
import zmq

import feagi_rust_py_libs as frpl

logger = logging.getLogger(__name__)


class FeagiSensoryClient:
    """High-performance sensory stream publisher using Rust encoders."""

    def __init__(self, host: str = "localhost", port: int = 5558, timeout: int = 5) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self._context: Optional[zmq.Context] = None
        self._socket: Optional[zmq.Socket] = None

    def connect(self) -> bool:
        """Connect to the FEAGI sensory PUSH endpoint."""
        if self._socket is not None:
            return True

        try:
            context = zmq.Context()
            socket = context.socket(zmq.PUSH)
            socket.setsockopt(zmq.LINGER, 1000)
            socket.setsockopt(zmq.SNDHWM, 100)
            socket.setsockopt(zmq.IMMEDIATE, 1)
            socket.setsockopt(zmq.SNDTIMEO, self.timeout * 1000)

            address = f"tcp://{self.host}:{self.port}"
            socket.connect(address)

            self._context = context
            self._socket = socket
            logger.info("Connected to FEAGI sensory stream at %s", address)
            return True
        except Exception:  # pragma: no cover - initialization failure is logged
            logger.exception("Failed to connect to FEAGI sensory stream")
            self.close()
            return False

    def send_sensory_data(
        self, cortical_area: str, neuron_data: Dict[Tuple[int, int, int], float]
    ) -> bool:
        """Encode and publish neuron data for a single cortical area."""
        if self._socket is None:
            logger.error("Sensory client is not connected")
            return False

        if not neuron_data:
            logger.debug("No neuron data to publish for cortical area '%s'", cortical_area)
            return True

        try:
            payload = self._encode_neurons(cortical_area, neuron_data)
            self._socket.send(payload, zmq.NOBLOCK)
            logger.debug("Sent %d neurons for '%s' (%d bytes)", len(neuron_data), cortical_area, len(payload))
            return True
        except zmq.Again:
            logger.warning("Sensory socket backpressure detected while sending '%s'", cortical_area)
            return False
        except Exception:  # pragma: no cover - unexpected encoding/runtime errors are logged
            logger.exception("Failed to send sensory data for '%s'", cortical_area)
            return False

    @staticmethod
    def _encode_neurons(
        cortical_area: str, neuron_data: Dict[Tuple[int, int, int], float]
    ) -> bytes:
        """Convert neuron dictionary to FEAGI XYZP binary using Rust bindings."""
        neuron_count = len(neuron_data)
        coords = np.array(list(neuron_data.keys()), dtype=np.uint32)
        if coords.shape != (neuron_count, 3):
            raise ValueError("Neuron coordinate array did not resolve to expected shape")

        x_coords = coords[:, 0]
        y_coords = coords[:, 1]
        z_coords = coords[:, 2]
        potentials = np.fromiter((float(p) for p in neuron_data.values()), dtype=np.float32, count=neuron_count)

        neuron_arrays = frpl.data_structures.neurons_voxels.xyzp.NeuronVoxelXYZPArrays.new_from_numpy(
            x_coords, y_coords, z_coords, potentials
        )

        cortical_id = frpl.data_structures.genomic.CorticalID.try_new_from_string(cortical_area)
        mapped_neurons = frpl.data_structures.neurons_voxels.xyzp.CorticalMappedXYZPNeuronVoxels()
        mapped_neurons.insert(cortical_id, neuron_arrays)

        byte_structure = mapped_neurons.as_new_feagi_byte_structure()
        return bytes(byte_structure.copy_out_as_byte_vector())

    def close(self) -> None:
        """Close underlying ZeroMQ resources."""
        try:
            if self._socket is not None:
                self._socket.close()
            if self._context is not None:
                self._context.term()
        finally:
            self._socket = None
            self._context = None
            logger.info("Sensory client closed") 