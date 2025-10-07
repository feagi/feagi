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
#  ==============================================================================
"""
FEAGI Protocol Package - Byte Structures Implementation

This module provides protocol definitions for binary communication between FEAGI
and its clients using specialized byte structures optimized for neural data.

Note: Moving away from Python dependencies towards Rust-native implementations.
- Visualization encoding: feagi_rust.VisualizationEncoder
- Agent encoding/decoding: handled in feagi-connector using published crates
"""

# Import protocol definitions and constants
from feagi.api.protocols.constants import ByteStructureID, ProtocolID

# Re-export translator (still needed for ZMQ server, but will be migrated to Rust)
from feagi.api.protocols.translator import default_translator

__all__ = [
    "ProtocolID",
    "ByteStructureID",
    "default_translator",
]