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

"""Binary protocol implementation for FEAGI."""

import struct

from feagi.utils.logger import setup_logger

logger = setup_logger()
import zlib
from typing import Any

import numpy as np

from feagi.protocols.protocol_factory import Protocol, register_protocol


@register_protocol
class BinaryProtocol(Protocol):
    """
    Binary protocol implementation.

    This protocol serializes data in a binary format optimized for performance
    and is used primarily for high-throughput data like sensorimotor and visualization.
    """

    name = "binary"
    content_type = "application/octet-stream"

    # Format types
    FORMAT_JSON = 0
    FORMAT_BINARY = 1
    FORMAT_NUMPY = 2

    @classmethod
    def serialize(cls, data: Any, compression_level: int = 0) -> bytes:
        """
        Serialize data to binary format.

        Args:
            data: Data to serialize.
            compression_level: Compression level (0-9, 0 = no compression).

        Returns:
            Binary data.
        """
        try:
            # Determine data type and serialize
            if (
                isinstance(data, dict)
                or isinstance(data, list)
                or isinstance(data, str)
                or isinstance(data, int)
                or isinstance(data, float)
                or isinstance(data, bool)
                or data is None
            ):
                # JSON-compatible data
                import json

                binary_data = json.dumps(data).encode()
                format_type = cls.FORMAT_JSON
            elif isinstance(data, bytes) or isinstance(data, bytearray):
                # Already binary
                binary_data = data
                format_type = cls.FORMAT_BINARY
            elif isinstance(data, np.ndarray):
                # Numpy array
                binary_data = cls._serialize_numpy(data)
                format_type = cls.FORMAT_NUMPY
            else:
                # Fallback to string representation
                binary_data = str(data).encode()
                format_type = cls.FORMAT_JSON

            # Compress if needed
            if compression_level > 0:
                binary_data = zlib.compress(binary_data, compression_level)

            # Create header
            header = struct.pack(
                ">BBI", format_type, compression_level, len(binary_data)
            )

            return header + binary_data
        except Exception as e:
            logger.exception(f"Error serializing data to binary: {e}")
            import json

            error_data = json.dumps({"error": str(e)}).encode()
            header = struct.pack(">BBI", cls.FORMAT_JSON, 0, len(error_data))
            return header + error_data

    @classmethod
    def deserialize(cls, data: bytes) -> Any:
        """
        Deserialize binary data.

        Args:
            data: Binary data to deserialize.

        Returns:
            Deserialized data.
        """
        try:
            # Parse header
            if len(data) < 6:
                raise ValueError("Invalid binary data: too short")

            format_type, compression_level, data_length = struct.unpack(
                ">BBI", data[:6]
            )
            payload = data[6:]

            # Check data length
            if len(payload) != data_length:
                logger.warning(
                    f"Data length mismatch: expected {data_length}, got {len(payload)}"
                )

            # Decompress if needed
            if compression_level > 0:
                payload = zlib.decompress(payload)

            # Deserialize based on format type
            if format_type == cls.FORMAT_JSON:
                import json

                return json.loads(payload.decode())
            elif format_type == cls.FORMAT_BINARY:
                return payload
            elif format_type == cls.FORMAT_NUMPY:
                return cls._deserialize_numpy(payload)
            else:
                raise ValueError(f"Unsupported format type: {format_type}")
        except Exception as e:
            logger.exception(f"Error deserializing binary data: {e}")
            return {"error": str(e)}

    @classmethod
    def _serialize_numpy(cls, array: np.ndarray) -> bytes:
        """
        Serialize a numpy array to binary.

        Args:
            array: Numpy array to serialize.

        Returns:
            Binary data.
        """
        # Encode shape and dtype
        shape = array.shape
        dtype = str(array.dtype)

        # Create header
        header = struct.pack(">B", len(shape))  # Number of dimensions
        for dim in shape:
            header += struct.pack(">I", dim)  # Size of each dimension

        # Add dtype
        dtype_bytes = dtype.encode()
        header += struct.pack(">B", len(dtype_bytes))
        header += dtype_bytes

        # Add data
        data = array.tobytes()

        return header + data

    @classmethod
    def _deserialize_numpy(cls, data: bytes) -> np.ndarray:
        """
        Deserialize binary data to a numpy array.

        Args:
            data: Binary data to deserialize.

        Returns:
            Numpy array.
        """
        # Parse header
        offset = 0
        dims = struct.unpack(">B", data[offset : offset + 1])[0]
        offset += 1

        shape = []
        for i in range(dims):
            shape.append(struct.unpack(">I", data[offset : offset + 4])[0])
            offset += 4

        dtype_len = struct.unpack(">B", data[offset : offset + 1])[0]
        offset += 1

        dtype = data[offset : offset + dtype_len].decode()
        offset += dtype_len

        # Create array
        array_data = data[offset:]
        return np.frombuffer(array_data, dtype=dtype).reshape(shape)
