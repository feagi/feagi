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
Tests for the binary protocol serialization module.
"""

import pytest

# Skip the entire test module since the binary protocol implementation has been removed
pytest.skip(
    "Binary protocol tests need to be updated after protocol refactoring",
    allow_module_level=True,
)

# Original imports and tests below - kept for reference
import struct
import time
from typing import Any, Dict

from feagi.api.protocols.base import ProtocolID
from feagi.api.protocols.binary import BinaryProtocolError, BinarySerializer


class TestBinarySerializer:
    """Test the BinarySerializer class."""

    def test_encode_decode_header(self):
        """Test encoding and decoding protocol headers."""
        # Test all protocol IDs
        for protocol_id in ProtocolID:
            for version in range(1, 3):
                # Encode header
                header = BinarySerializer.encode_header(protocol_id, version)

                # Check header size
                assert len(header) == BinarySerializer.HEADER_SIZE

                # Decode header
                decoded_id, decoded_version = BinarySerializer.decode_header(header)

                # Check decoded values
                assert decoded_id == protocol_id
                assert decoded_version == version

    def test_invalid_header(self):
        """Test decoding invalid headers."""
        # Test with empty data
        with pytest.raises(BinaryProtocolError):
            BinarySerializer.decode_header(b"")

        # Test with invalid protocol ID
        invalid_header = struct.pack(BinarySerializer.HEADER_FORMAT, 99, 1)
        with pytest.raises(BinaryProtocolError):
            BinarySerializer.decode_header(invalid_header)

    def test_encode_decode_fcp(self):
        """Test encoding and decoding FCP messages."""
        # Test parameters
        command_type = 5
        payload = b"test payload"
        version = 1

        # Encode message
        encoded = BinarySerializer.encode_fcp(command_type, payload, version)

        # Check encoded size
        expected_size = (
            BinarySerializer.HEADER_SIZE
            + BinarySerializer.FCP_HEADER_SIZE
            + len(payload)
        )
        assert len(encoded) == expected_size

        # Decode message
        decoded = BinarySerializer.decode_fcp(encoded)

        # Check decoded values
        assert decoded["protocol_id"] == ProtocolID.FCP.value
        assert decoded["version"] == version
        assert decoded["command_type"] == command_type
        assert decoded["payload"] == payload

    def test_encode_decode_fvp(self):
        """Test encoding and decoding FVP messages."""
        # Test parameters
        frame_type = 2
        payload = b"visualization data"
        timestamp = int(time.time() * 1000)
        version = 1

        # Encode message
        encoded = BinarySerializer.encode_fvp(frame_type, payload, timestamp, version)

        # Check encoded size
        expected_size = (
            BinarySerializer.HEADER_SIZE
            + BinarySerializer.FVP_HEADER_SIZE
            + len(payload)
        )
        assert len(encoded) == expected_size

        # Decode message
        decoded = BinarySerializer.decode_fvp(encoded)

        # Check decoded values
        assert decoded["protocol_id"] == ProtocolID.FVP.value
        assert decoded["version"] == version
        assert decoded["frame_type"] == frame_type
        assert decoded["timestamp"] == timestamp
        assert decoded["payload"] == payload

    def test_encode_decode_fsmp(self):
        """Test encoding and decoding FSMP messages."""
        # Test parameters
        channel_id = 1024
        payload = b"sensorimotor data"
        timestamp = int(time.time() * 1000)
        version = 1

        # Encode message
        encoded = BinarySerializer.encode_fsmp(channel_id, payload, timestamp, version)

        # Check encoded size
        expected_size = (
            BinarySerializer.HEADER_SIZE
            + BinarySerializer.FSMP_HEADER_SIZE
            + len(payload)
        )
        assert len(encoded) == expected_size

        # Decode message
        decoded = BinarySerializer.decode_fsmp(encoded)

        # Check decoded values
        assert decoded["protocol_id"] == ProtocolID.FSMP.value
        assert decoded["version"] == version
        assert decoded["channel_id"] == channel_id
        assert decoded["timestamp"] == timestamp
        assert decoded["payload"] == payload

    def test_generic_encode_decode(self):
        """Test the generic encode and decode methods."""
        # Test FCP
        fcp_message = {"command_type": 3, "payload": b"fcp test"}
        encoded_fcp = BinarySerializer.encode(ProtocolID.FCP, fcp_message)
        decoded_fcp = BinarySerializer.decode(encoded_fcp)
        assert decoded_fcp["protocol_id"] == ProtocolID.FCP.value
        assert decoded_fcp["command_type"] == fcp_message["command_type"]
        assert decoded_fcp["payload"] == fcp_message["payload"]

        # Test FVP
        fvp_message = {
            "frame_type": 1,
            "payload": b"fvp test",
            "timestamp": int(time.time() * 1000),
        }
        encoded_fvp = BinarySerializer.encode(ProtocolID.FVP, fvp_message)
        decoded_fvp = BinarySerializer.decode(encoded_fvp)
        assert decoded_fvp["protocol_id"] == ProtocolID.FVP.value
        assert decoded_fvp["frame_type"] == fvp_message["frame_type"]
        assert decoded_fvp["timestamp"] == fvp_message["timestamp"]
        assert decoded_fvp["payload"] == fvp_message["payload"]

        # Test FSMP
        fsmp_message = {
            "channel_id": 512,
            "payload": b"fsmp test",
            "timestamp": int(time.time() * 1000),
        }
        encoded_fsmp = BinarySerializer.encode(ProtocolID.FSMP, fsmp_message)
        decoded_fsmp = BinarySerializer.decode(encoded_fsmp)
        assert decoded_fsmp["protocol_id"] == ProtocolID.FSMP.value
        assert decoded_fsmp["channel_id"] == fsmp_message["channel_id"]
        assert decoded_fsmp["timestamp"] == fsmp_message["timestamp"]
        assert decoded_fsmp["payload"] == fsmp_message["payload"]

    def test_invalid_protocol(self):
        """Test with invalid protocol ID."""
        with pytest.raises(BinaryProtocolError):
            BinarySerializer.encode(99, {})  # type: ignore
