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
FEAGI Protocol Package - Byte Structures Implementation

This module provides protocol definitions for binary communication between FEAGI
and its clients using specialized byte structures optimized for neural data.
"""

from feagi.utils.logger import setup_logger
logger = setup_logger(__name__)

# Import protocol definitions and constants
from feagi.api.protocols.constants import ProtocolID, ByteStructureID, FCPCommandType, FSMPChannelType, FVPFrameType

# Import base protocol components
from feagi.api.protocols.base import ProtocolManager, VersionedProtocol, ProtocolRegistry

# Import specific protocol implementations
from feagi.api.protocols.fcp import FCPv1, register_protocols as register_fcp
from feagi.api.protocols.fsmp import FSMPv1, register_protocols as register_fsmp
from feagi.api.protocols.fvp import FVPv1, register_protocols as register_fvp

# Import protocol byte structures
try:
    from feagi.api.protocols.byte_structures import ByteStructureEncoder, ByteStructureDecoder
except ImportError:
    from feagi.api.protocols.byte_structures.encoder import ByteStructureEncoder
    from feagi.api.protocols.byte_structures.decoder import ByteStructureDecoder

# Import and re-export translator
from feagi.api.protocols.translator import ByteStructureTranslator, ProtocolTranslator, default_translator

# Initialize the protocol manager
protocol_manager = ProtocolManager()

# Register all protocol implementations
register_fcp(protocol_manager.registry)
register_fsmp(protocol_manager.registry)
register_fvp(protocol_manager.registry)

__all__ = [
    'ProtocolID',
    'ByteStructureID',
    'FCPCommandType',
    'FSMPChannelType',
    'FVPFrameType',
    'ByteStructureEncoder',
    'ByteStructureDecoder',
    'ByteStructureTranslator',
    'default_translator',
    'ProtocolTranslator',
    'ProtocolManager',
    'protocol_manager',
    'VersionedProtocol',
    'ProtocolRegistry',
] 