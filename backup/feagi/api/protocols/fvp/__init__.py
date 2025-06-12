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
FEAGI Visualization Protocol (FVP) module.

This module provides access to all FVP versions and helper functions.
"""

import time
from typing import Any, Dict, List

# Import common FVP components
from feagi.api.protocols.fvp.common import FVPFrameType, FVPMessageFormat

# Import all versions
from feagi.api.protocols.fvp.v1 import FVPv1

# Re-export helper functions


def create_neuron_activations_message(
    cortical_area_id: int, activations: Dict[str, float]
) -> Dict[str, Any]:
    """
    Create a neuron activations message.

    Args:
        cortical_area_id: ID of the cortical area
        activations: Dictionary mapping neuron IDs to activation values

    Returns:
        FVP message data
    """
    return {
        "frame_type": FVPFrameType.NEURON_ACTIVATIONS,
        "timestamp_ms": int(time.time() * 1000),
        "payload": {"cortical_area_id": cortical_area_id, "activations": activations},
    }


def create_global_stats_message(stats: Dict[str, Any]) -> Dict[str, Any]:
    """
    Create a global statistics message.

    Args:
        stats: Dictionary of global stats

    Returns:
        FVP message data
    """
    return {
        "frame_type": FVPFrameType.GLOBAL_STATS,
        "timestamp_ms": int(time.time() * 1000),
        "payload": {"stats": stats},
    }


def create_structure_data_message(
    areas: Dict[str, Dict[str, Any]], connections: List[Dict[str, Any]]
) -> Dict[str, Any]:
    """
    Create a structure data message.

    Args:
        areas: Dictionary mapping area IDs to area properties
        connections: List of connections between areas

    Returns:
        FVP message data
    """
    return {
        "frame_type": FVPFrameType.STRUCTURE_DATA,
        "timestamp_ms": int(time.time() * 1000),
        "payload": {"areas": areas, "connections": connections},
    }


def register_protocols():
    """Register FVP protocol versions with the registry."""
    from feagi.api.protocols.base import ProtocolRegistry

    registry = ProtocolRegistry()
    registry.register(FVPv1)


# Define all exported symbols
__all__ = [
    "FVPFrameType",
    "FVPMessageFormat",
    "FVPv1",
    "create_neuron_activations_message",
    "create_global_stats_message",
    "create_structure_data_message",
    "register_protocols",
]
