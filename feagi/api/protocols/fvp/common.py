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
FEAGI Visualization Protocol (FVP) common definitions.

This module provides the common structures and enums used by all versions of FVP.
"""

import struct
from enum import IntEnum
from typing import Tuple


class FVPFrameType(IntEnum):
    """Frame types for FVP."""
    NEURON_ACTIVATIONS = 0x01
    CONNECTION_STRENGTHS = 0x02
    AREA_SUMMARY = 0x03
    GLOBAL_STATS = 0x04
    STRUCTURE_DATA = 0x05
    ERROR = 0xFF


class FVPMessageFormat:
    """
    FVP message format utilities.
    
    Format:
    +-------------+-------------+-------------+----------------+-------------+------------------+
    | Protocol ID | Version     | Frame       | Timestamp      | Data        | Data Payload     |
    | (1 byte)    | (1 byte)    | Type        | (8 bytes)      | Length      | (variable)       |
    |             |             | (1 byte)    |                | (4 bytes)   |                  |
    +-------------+-------------+-------------+----------------+-------------+------------------+
    """
    
    HEADER_FORMAT = "!BQI"  # Frame type (1 byte) + timestamp (8 bytes) + data length (4 bytes)
    HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
    
    @staticmethod
    def pack_header(frame_type: FVPFrameType, timestamp_ms: int, payload_length: int) -> bytes:
        """Pack FVP header."""
        return struct.pack(FVPMessageFormat.HEADER_FORMAT, frame_type, timestamp_ms, payload_length)
    
    @staticmethod
    def unpack_header(data: bytes) -> Tuple[FVPFrameType, int, int]:
        """Unpack FVP header."""
        frame_type, timestamp_ms, payload_length = struct.unpack(FVPMessageFormat.HEADER_FORMAT, data)
        return FVPFrameType(frame_type), timestamp_ms, payload_length 