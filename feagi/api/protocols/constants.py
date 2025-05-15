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
Constants for FEAGI protocols and byte structures.

This module defines constants used throughout the protocol implementation,
including protocol identifiers and byte structure IDs.
"""

from enum import IntEnum


class ProtocolID(IntEnum):
    """Protocol IDs for different FEAGI protocols."""
    
    FCP = 1  # FEAGI Control Protocol
    FVP = 2  # FEAGI Visualization Protocol
    FSMP = 3  # FEAGI Sensorimotor Protocol


class ByteStructureID(IntEnum):
    """
    Byte structure IDs for different data types.
    
    These IDs correspond to the structure IDs defined in the 
    FEAGI Byte Structures documentation.
    """
    
    JSON = 1            # JSON data (slower, for non-performance-critical operations)
    RAW_IMAGE = 8       # Single raw image (BGR format)
    MULTI_HOLDER = 9    # Container for multiple byte structures
    NEURON_FLAT = 10    # Neuron potential data (flat format)
    NEURON_CATEGORIES = 11  # Neuron potential data (categorized by cortical area)


class FCPCommandType(IntEnum):
    """Command types for FEAGI Control Protocol."""
    
    REGISTER = 1
    DEREGISTER = 2
    CONFIGURE = 3
    STATUS_REQUEST = 4
    STATUS_RESPONSE = 5
    HEARTBEAT = 6
    ERROR = 255


class FVPFrameType(IntEnum):
    """Frame types for FEAGI Visualization Protocol."""
    
    STRUCTURE = 1
    ACTIVITY = 2
    CONFIG = 3
    ERROR = 255


class FSMPChannelType(IntEnum):
    """Channel types for FEAGI Sensorimotor Protocol."""
    
    SENSORY = 1
    MOTOR = 2
    PROPRIOCEPTIVE = 3
    ERROR = 255 