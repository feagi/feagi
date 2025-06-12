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
FEAGI Control Protocol (FCP) version 1 implementation.

This module provides the implementation of version 1 of the FCP protocol.
"""

import json
from typing import Any, ClassVar, Dict

from feagi.api.protocols.base import ProtocolID, VersionedProtocol
from feagi.api.protocols.fcp.common import FCPCommandType, FCPMessageFormat
from feagi.utils.logger import setup_logger

logger = setup_logger()


class FCPv1(VersionedProtocol):
    """FEAGI Control Protocol version 1."""

    PROTOCOL_ID: ClassVar[ProtocolID] = ProtocolID.FCP
    VERSION: ClassVar[int] = 1

    @classmethod
    def encode(cls, data: Dict[str, Any]) -> bytes:
        """
        Encode FCP data to binary format.

        Args:
            data: Dictionary containing:
                - command_type: FCPCommandType
                - payload: Dict containing command-specific data

        Returns:
            Binary FCP data
        """
        command_type = FCPCommandType(data.get("command_type", FCPCommandType.ERROR))
        payload = data.get("payload", {})

        # Convert payload to JSON
        json_payload = json.dumps(payload).encode("utf-8")
        payload_length = len(json_payload)

        # Pack header and payload
        header = FCPMessageFormat.pack_header(command_type, payload_length)
        return header + json_payload

    @classmethod
    def decode(cls, data: bytes) -> Dict[str, Any]:
        """
        Decode binary FCP data.

        Args:
            data: Binary FCP data

        Returns:
            Dictionary containing:
                - command_type: FCPCommandType
                - payload: Dict containing command-specific data

        Raises:
            ValueError: If the data format is invalid
        """
        if len(data) < FCPMessageFormat.HEADER_SIZE:
            raise ValueError("FCP data too short")

        # Unpack header
        header_data = data[: FCPMessageFormat.HEADER_SIZE]
        payload_data = data[FCPMessageFormat.HEADER_SIZE :]

        command_type, payload_length = FCPMessageFormat.unpack_header(header_data)

        # Verify payload length
        if len(payload_data) != payload_length:
            raise ValueError(
                f"FCP payload length mismatch: expected {payload_length}, got {len(payload_data)}"
            )

        # Parse JSON payload
        try:
            payload = json.loads(payload_data.decode("utf-8"))
        except json.JSONDecodeError as e:
            raise ValueError(f"Invalid FCP JSON payload: {str(e)}")

        return {"command_type": command_type, "payload": payload}
