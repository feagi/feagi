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
FEAGI Sensorimotor Protocol (FSMP) common definitions.

This module provides the common structures and enums used by all versions of FSMP.
"""

import struct
from enum import IntEnum
from typing import Tuple


class FSMPChannelType(IntEnum):
    """Channel types for FSMP."""
    SENSORY = 0x00  # 0x0000-0x7FFF for sensory channels
    MOTOR = 0x80    # 0x8000-0xFFFF for motor channels
    ERROR = 0xFF


class FSMPDataType(IntEnum):
    """Data structure types for FSMP."""
    NEURON_POTENTIAL_DATA = 0x0B  # ID 11: Neuron Potential Data (Categories, XYZ)
    SENSORY_DATA = 0x01
    MOTOR_DATA = 0x02
    PROPRIOCEPTIVE_DATA = 0x03
    ERROR = 0xFF


class FSMPMessageFormat:
    """
    FSMP message format utilities.
    
    Format:
    +-------------+-------------+-------------+----------------+-------------+------------------+
    | Protocol ID | Version     | Channel     | Timestamp      | Data        | Data Payload     |
    | (1 byte)    | (1 byte)    | ID          | (8 bytes)      | Length      | (variable)       |
    |             |             | (2 bytes)   |                | (4 bytes)   |                  |
    +-------------+-------------+-------------+----------------+-------------+------------------+
    """
    
    HEADER_FORMAT = "!HQI"  # Channel ID (2 bytes) + timestamp (8 bytes) + data length (4 bytes)
    HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
    
    @staticmethod
    def pack_header(channel_id: int, timestamp_ms: int, payload_length: int) -> bytes:
        """Pack FSMP header."""
        return struct.pack(FSMPMessageFormat.HEADER_FORMAT, channel_id, timestamp_ms, payload_length)
    
    @staticmethod
    def unpack_header(data: bytes) -> Tuple[int, int, int]:
        """Unpack FSMP header."""
        channel_id, timestamp_ms, payload_length = struct.unpack(FSMPMessageFormat.HEADER_FORMAT, data)
        return channel_id, timestamp_ms, payload_length 