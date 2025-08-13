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
FEAGI Control Protocol (FCP) common definitions.

This module provides the common structures and enums used by all versions of FCP.
"""

import struct
from enum import IntEnum
from typing import Tuple


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
