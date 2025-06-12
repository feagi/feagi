#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""
FEAGI Sensorimotor Protocol (FSMP) version 1 implementation.

This module provides the implementation of version 1 of the FSMP protocol.
"""

import struct
import time
from typing import Any, ClassVar, Dict, List, Tuple

from feagi.api.protocols.base import ProtocolID, VersionedProtocol
from feagi.api.protocols.fsmp.common import FSMPDataType, FSMPMessageFormat
from feagi.utils.logger import setup_logger

logger = setup_logger()


class FSMPv1(VersionedProtocol):
    """FEAGI Sensorimotor Protocol version 1."""

    PROTOCOL_ID: ClassVar[ProtocolID] = ProtocolID.FSMP
    VERSION: ClassVar[int] = 1

    @classmethod
    def encode(cls, data: Dict[str, Any]) -> bytes:
        """
        Encode FSMP data to binary format.

        Args:
            data: Dictionary containing:
                - channel_id: int (Channel identifier)
                - data_type: FSMPDataType (Type of data structure)
                - timestamp_ms: int (milliseconds since epoch)
                - payload: Dict or bytes containing data-specific content

        Returns:
            Binary FSMP data
        """
        channel_id = data.get("channel_id", 0)
        data_type = FSMPDataType(data.get("data_type", FSMPDataType.ERROR))
        timestamp_ms = data.get("timestamp_ms", int(time.time() * 1000))
        payload = data.get("payload", {})

        # Convert payload to binary based on data type
        if data_type == FSMPDataType.NEURON_POTENTIAL_DATA:
            binary_payload = cls._encode_neuron_potential_data(payload)
        elif data_type == FSMPDataType.SENSORY_DATA:
            binary_payload = cls._encode_sensory_data(payload)
        elif data_type == FSMPDataType.MOTOR_DATA:
            binary_payload = cls._encode_motor_data(payload)
        elif data_type == FSMPDataType.PROPRIOCEPTIVE_DATA:
            binary_payload = cls._encode_proprioceptive_data(payload)
        else:
            raise ValueError(f"Unsupported data type: {data_type}")

        # Add data type as first byte in payload
        binary_payload = bytes([data_type]) + binary_payload
        payload_length = len(binary_payload)

        # Pack header and payload
        header = FSMPMessageFormat.pack_header(channel_id, timestamp_ms, payload_length)
        return header + binary_payload

    @classmethod
    def decode(cls, data: bytes) -> Dict[str, Any]:
        """
        Decode binary FSMP data.

        Args:
            data: Binary FSMP data

        Returns:
            Dictionary containing:
                - channel_id: int (Channel identifier)
                - data_type: FSMPDataType (Type of data structure)
                - timestamp_ms: int (milliseconds since epoch)
                - payload: Dict containing data-specific content

        Raises:
            ValueError: If the data format is invalid
        """
        if len(data) < FSMPMessageFormat.HEADER_SIZE + 1:  # +1 for data_type byte
            raise ValueError("FSMP data too short")

        # Unpack header
        header_data = data[: FSMPMessageFormat.HEADER_SIZE]
        payload_data = data[FSMPMessageFormat.HEADER_SIZE :]

        channel_id, timestamp_ms, payload_length = FSMPMessageFormat.unpack_header(
            header_data
        )

        # Verify payload length
        if len(payload_data) != payload_length:
            raise ValueError(
                f"FSMP payload length mismatch: expected {payload_length}, got {len(payload_data)}"
            )

        # Extract data_type from first byte of payload
        data_type = FSMPDataType(payload_data[0])
        content_data = payload_data[1:]

        # Decode content based on data type
        if data_type == FSMPDataType.NEURON_POTENTIAL_DATA:
            payload = cls._decode_neuron_potential_data(content_data)
        elif data_type == FSMPDataType.SENSORY_DATA:
            payload = cls._decode_sensory_data(content_data)
        elif data_type == FSMPDataType.MOTOR_DATA:
            payload = cls._decode_motor_data(content_data)
        elif data_type == FSMPDataType.PROPRIOCEPTIVE_DATA:
            payload = cls._decode_proprioceptive_data(content_data)
        else:
            raise ValueError(f"Unsupported data type: {data_type}")

        return {
            "channel_id": channel_id,
            "data_type": data_type,
            "timestamp_ms": timestamp_ms,
            "payload": payload,
        }

    @classmethod
    def _encode_neuron_potential_data(cls, data: Dict[str, Any]) -> bytes:
        """
        Encode neuron potential data according to FSMP.md specification.

        Format:
        +-------------+-------------+-------------+----------+-------------+------------------+
        | Global      | Global      | Cortical    | Neuron   | Neuron      | Neuron           |
        | Header ID   | Header Ver  | Area Count  | Data     | Data        | Data             |
        | (1 byte)    | (1 byte)    | (2 bytes)   | Header1  | ...         | HeaderN          |
        +-------------+-------------+-------------+----------+-------------+------------------+

        Args:
            data: Dictionary containing:
                - cortical_areas: List of cortical areas with neuron data

        Returns:
            Binary neuron potential data
        """
        cortical_areas = data.get("cortical_areas", [])

        # Global header (ID=11, Version=1)
        result = struct.pack("!BB", FSMPDataType.NEURON_POTENTIAL_DATA, 1)

        # Section header (cortical area count)
        result += struct.pack("!H", len(cortical_areas))

        # Initialize offset counter to calculate reading start indexes
        # Start after global header (2 bytes) + section header (2 bytes) + all secondary headers
        current_offset = 2 + 2 + (len(cortical_areas) * 14)

        # Calculate neuron data for each cortical area to determine offsets
        cortical_data = []
        for area in cortical_areas:
            area_id = area.get("cortical_id", "").encode("ascii")
            # Pad ID to 6 bytes
            area_id = area_id[:6].ljust(6, b" ")
            neurons = area.get("neurons", [])

            # Store for later processing
            cortical_data.append((area_id, neurons, current_offset))

            # Update offset for next area (each neuron is 16 bytes: x, y, z, potential)
            current_offset += len(neurons) * 16

        # Write secondary headers
        for area_id, neurons, offset in cortical_data:
            # Secondary header: Cortical ID (6 bytes), Reading Start Index (4 bytes), Number of Neurons (4 bytes)
            result += area_id
            result += struct.pack("!II", offset, len(neurons))

        # Write neuron data
        for area_id, neurons, _ in cortical_data:
            # X coordinates (4 bytes per neuron)
            for neuron in neurons:
                result += struct.pack("!I", neuron.get("x", 0))

            # Y coordinates (4 bytes per neuron)
            for neuron in neurons:
                result += struct.pack("!I", neuron.get("y", 0))

            # Z coordinates (4 bytes per neuron)
            for neuron in neurons:
                result += struct.pack("!I", neuron.get("z", 0))

            # Neuron potentials (4 bytes per neuron as FLOAT)
            for neuron in neurons:
                result += struct.pack("!f", neuron.get("potential", 0.0))

        return result

    @classmethod
    def _decode_neuron_potential_data(cls, data: bytes) -> Dict[str, Any]:
        """
        Decode neuron potential data according to FSMP.md specification.

        Args:
            data: Binary neuron potential data

        Returns:
            Dictionary containing decoded neuron data
        """
        if len(data) < 4:  # At least global header (2) + section header (2)
            raise ValueError("Neuron potential data too short")

        # Skip global header (already processed in decode method) and get cortical area count
        offset = 0
        cortical_area_count = struct.unpack("!H", data[offset : offset + 2])[0]
        offset += 2

        if offset + (14 * cortical_area_count) > len(data):
            raise ValueError("Neuron potential data truncated (secondary headers)")

        # Read secondary headers
        cortical_areas = []
        for _ in range(cortical_area_count):
            cortical_id = data[offset : offset + 6].decode("ascii").strip()
            offset += 6

            reading_start_index, neuron_count = struct.unpack(
                "!II", data[offset : offset + 8]
            )
            offset += 8

            cortical_areas.append(
                {
                    "cortical_id": cortical_id,
                    "reading_start_index": reading_start_index,
                    "neuron_count": neuron_count,
                    "neurons": [],
                }
            )

        # Read neuron data for each cortical area
        for area in cortical_areas:
            neuron_count = area["neuron_count"]
            area_start = (
                area["reading_start_index"] - 1
            )  # Convert from 1-indexed to 0-indexed

            # Read X coordinates
            x_offset = area_start
            x_values = []
            for i in range(neuron_count):
                if x_offset + 4 > len(data):
                    raise ValueError("Neuron potential data truncated (X coordinates)")
                x_values.append(struct.unpack("!I", data[x_offset : x_offset + 4])[0])
                x_offset += 4

            # Read Y coordinates
            y_offset = x_offset
            y_values = []
            for i in range(neuron_count):
                if y_offset + 4 > len(data):
                    raise ValueError("Neuron potential data truncated (Y coordinates)")
                y_values.append(struct.unpack("!I", data[y_offset : y_offset + 4])[0])
                y_offset += 4

            # Read Z coordinates
            z_offset = y_offset
            z_values = []
            for i in range(neuron_count):
                if z_offset + 4 > len(data):
                    raise ValueError("Neuron potential data truncated (Z coordinates)")
                z_values.append(struct.unpack("!I", data[z_offset : z_offset + 4])[0])
                z_offset += 4

            # Read potentials
            p_offset = z_offset
            p_values = []
            for i in range(neuron_count):
                if p_offset + 4 > len(data):
                    raise ValueError(
                        "Neuron potential data truncated (potential values)"
                    )
                p_values.append(struct.unpack("!f", data[p_offset : p_offset + 4])[0])
                p_offset += 4

            # Combine data
            for i in range(neuron_count):
                area["neurons"].append(
                    {
                        "x": x_values[i],
                        "y": y_values[i],
                        "z": z_values[i],
                        "potential": p_values[i],
                    }
                )

        return {"cortical_areas": cortical_areas}

    @classmethod
    def _encode_sensory_data(cls, data: Dict[str, Any]) -> bytes:
        """
        Encode sensory data.

        Format:
        +-------------+-------------------+-------------------+
        | Cortical    | Data Size         | Neuron            |
        | Area ID     | (pixels/elements) | Activations       |
        | (6 bytes)   | (4 bytes)         | (variable)        |
        +-------------+-------------------+-------------------+

        Args:
            data: Dictionary containing sensory data

        Returns:
            Binary sensory data
        """
        cortical_id = data.get("cortical_id", "").encode("ascii")
        # Pad ID to 6 bytes
        cortical_id = cortical_id[:6].ljust(6, b" ")

        activations = data.get("activations", [])
        data_size = len(activations)

        # Pack header
        result = cortical_id + struct.pack("!I", data_size)

        # Pack activations as floats
        for activation in activations:
            result += struct.pack("!f", float(activation))

        return result

    @classmethod
    def _decode_sensory_data(cls, data: bytes) -> Dict[str, Any]:
        """
        Decode sensory data.

        Args:
            data: Binary sensory data

        Returns:
            Dictionary containing sensory data
        """
        if len(data) < 10:  # At least cortical ID (6) + data size (4)
            raise ValueError("Sensory data too short")

        offset = 0
        cortical_id = data[offset : offset + 6].decode("ascii").strip()
        offset += 6

        data_size = struct.unpack("!I", data[offset : offset + 4])[0]
        offset += 4

        # Calculate expected data length and verify
        expected_length = offset + (4 * data_size)  # 4 bytes per float activation
        if len(data) < expected_length:
            raise ValueError(
                f"Sensory data truncated: expected {expected_length}, got {len(data)}"
            )

        # Read activations
        activations = []
        for i in range(data_size):
            activation = struct.unpack("!f", data[offset : offset + 4])[0]
            activations.append(activation)
            offset += 4

        return {
            "cortical_id": cortical_id,
            "data_size": data_size,
            "activations": activations,
        }

    @classmethod
    def _encode_motor_data(cls, data: Dict[str, Any]) -> bytes:
        """
        Encode motor data.

        Format:
        +-------------+-------------------+-------------------+
        | Cortical    | Data Size         | Neuron            |
        | Area ID     | (neurons/elements)| Activations       |
        | (6 bytes)   | (4 bytes)         | (variable)        |
        +-------------+-------------------+-------------------+

        Args:
            data: Dictionary containing motor data

        Returns:
            Binary motor data
        """
        # Motor data follows same format as sensory data
        return cls._encode_sensory_data(data)

    @classmethod
    def _decode_motor_data(cls, data: bytes) -> Dict[str, Any]:
        """
        Decode motor data.

        Args:
            data: Binary motor data

        Returns:
            Dictionary containing motor data
        """
        # Motor data follows same format as sensory data
        return cls._decode_sensory_data(data)

    @classmethod
    def _encode_proprioceptive_data(cls, data: Dict[str, Any]) -> bytes:
        """
        Encode proprioceptive data.

        Format:
        +-------------+-------------------+-------------------+
        | Cortical    | Data Size         | Proprioceptive    |
        | Area ID     | (values)          | Values            |
        | (6 bytes)   | (4 bytes)         | (variable)        |
        +-------------+-------------------+-------------------+

        Args:
            data: Dictionary containing proprioceptive data

        Returns:
            Binary proprioceptive data
        """
        # Proprioceptive data follows similar format to sensory/motor data
        cortical_id = data.get("cortical_id", "").encode("ascii")
        # Pad ID to 6 bytes
        cortical_id = cortical_id[:6].ljust(6, b" ")

        values = data.get("values", {})
        data_size = len(values)

        # Pack header
        result = cortical_id + struct.pack("!I", data_size)

        # Pack values as key-value pairs (2 bytes key, 4 bytes float value)
        for key, value in values.items():
            try:
                key_int = int(key)
                if key_int < 0 or key_int > 65535:  # 2-byte limit
                    key_int = 0
            except (ValueError, TypeError):
                key_int = 0

            result += struct.pack("!Hf", key_int, float(value))

        return result

    @classmethod
    def _decode_proprioceptive_data(cls, data: bytes) -> Dict[str, Any]:
        """
        Decode proprioceptive data.

        Args:
            data: Binary proprioceptive data

        Returns:
            Dictionary containing proprioceptive data
        """
        if len(data) < 10:  # At least cortical ID (6) + data size (4)
            raise ValueError("Proprioceptive data too short")

        offset = 0
        cortical_id = data[offset : offset + 6].decode("ascii").strip()
        offset += 6

        data_size = struct.unpack("!I", data[offset : offset + 4])[0]
        offset += 4

        # Calculate expected data length and verify
        expected_length = offset + (6 * data_size)  # 6 bytes per key-value pair (2+4)
        if len(data) < expected_length:
            raise ValueError(
                f"Proprioceptive data truncated: expected {expected_length}, got {len(data)}"
            )

        # Read values
        values = {}
        for i in range(data_size):
            key, value = struct.unpack("!Hf", data[offset : offset + 6])
            values[str(key)] = value
            offset += 6

        return {"cortical_id": cortical_id, "data_size": data_size, "values": values}
