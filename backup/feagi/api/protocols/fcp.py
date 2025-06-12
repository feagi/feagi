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
FEAGI Control Protocol (FCP) implementation.

This module provides the implementation of the FEAGI Control Protocol, which
is used for administrative and management commands between FEAGI and agents.
"""

import json
import struct
import uuid
from enum import Enum, IntEnum
from typing import Any, ClassVar, Dict, Optional, Tuple

from feagi.api.protocols.base import ProtocolID, ProtocolRegistry, VersionedProtocol
from feagi.utils.logger import setup_logger

logger = setup_logger()


class FCPCommandType(IntEnum):
    """Command types for FCP."""

    REGISTER = 0x01
    DEREGISTER = 0x02
    CONFIGURE = 0x03
    STATUS_REQUEST = 0x04
    STATUS_RESPONSE = 0x05
    HEARTBEAT = 0x06
    ERROR = 0xFF


class FCPMessageFormat:
    """
    FCP message format utilities.

    Format:
    +-------------+-------------+-------------+----------------+------------------+
    | Protocol ID | Version     | Command     | Message Length | Message Payload  |
    | (1 byte)    | (1 byte)    | Type        | (4 bytes)      | (variable)       |
    |             |             | (1 byte)    |                |                  |
    +-------------+-------------+-------------+----------------+------------------+
    """

    HEADER_FORMAT = "!BI"  # Command type (1 byte) + message length (4 bytes)
    HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

    @staticmethod
    def pack_header(command_type: FCPCommandType, payload_length: int) -> bytes:
        """Pack FCP header."""
        return struct.pack(FCPMessageFormat.HEADER_FORMAT, command_type, payload_length)

    @staticmethod
    def unpack_header(data: bytes) -> Tuple[FCPCommandType, int]:
        """Unpack FCP header."""
        command_type, payload_length = struct.unpack(
            FCPMessageFormat.HEADER_FORMAT, data
        )
        return FCPCommandType(command_type), payload_length


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


# Register the protocol with the registry
def register_protocols():
    """Register FCP protocol versions with the registry."""
    from feagi.api.protocols.base import ProtocolRegistry

    registry = ProtocolRegistry()
    registry.register(FCPv1)


# Helper functions for creating common FCP messages


def create_register_message(
    agent_id: str, agent_type: str, capabilities: Dict[str, Any], version: str = "1.0"
) -> Dict[str, Any]:
    """
    Create an agent registration message.

    Args:
        agent_id: Unique agent identifier
        agent_type: Agent type (e.g., "monitor", "robot", etc.)
        capabilities: Dictionary of agent capabilities
        version: Agent version

    Returns:
        FCP message data
    """
    return {
        "command_type": FCPCommandType.REGISTER,
        "payload": {
            "agent_id": agent_id,
            "agent_type": agent_type,
            "capabilities": capabilities,
            "version": version,
            "timestamp": None,  # Will be filled in by the protocol translator
            "message_id": str(uuid.uuid4()),
        },
    }


def create_heartbeat_message(agent_id: str, status: Dict[str, Any]) -> Dict[str, Any]:
    """
    Create a heartbeat message.

    Args:
        agent_id: Agent identifier
        status: Status information

    Returns:
        FCP message data
    """
    return {
        "command_type": FCPCommandType.HEARTBEAT,
        "payload": {
            "agent_id": agent_id,
            "status": status,
            "timestamp": None,  # Will be filled in by the protocol translator
            "message_id": str(uuid.uuid4()),
        },
    }
