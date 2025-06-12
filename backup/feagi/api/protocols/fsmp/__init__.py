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
FEAGI Sensorimotor Protocol (FSMP) module.

This module provides access to all FSMP versions and helper functions.
"""

import time
from typing import Any, Dict, List

# Import common FSMP components
from feagi.api.protocols.fsmp.common import (
    FSMPChannelType,
    FSMPDataType,
    FSMPMessageFormat,
)

# Import all versions
from feagi.api.protocols.fsmp.v1 import FSMPv1
from feagi.api.protocols.fsmp.v2 import FSMPv2

# Re-export helper functions


def create_sensory_data_message(
    channel_id: int, cortical_id: str, activations: List[float]
) -> Dict[str, Any]:
    """
    Create a sensory data message.

    Args:
        channel_id: Sensory channel identifier
        cortical_id: ID of the cortical area
        activations: List of activation values

    Returns:
        FSMP message data
    """
    return {
        "channel_id": channel_id,
        "data_type": FSMPDataType.SENSORY_DATA,
        "timestamp_ms": int(time.time() * 1000),
        "payload": {"cortical_id": cortical_id, "activations": activations},
    }


def create_motor_data_message(
    channel_id: int, cortical_id: str, activations: List[float]
) -> Dict[str, Any]:
    """
    Create a motor data message.

    Args:
        channel_id: Motor channel identifier
        cortical_id: ID of the cortical area
        activations: List of activation values

    Returns:
        FSMP message data
    """
    return {
        "channel_id": channel_id | 0x8000,  # Set high bit for motor channels
        "data_type": FSMPDataType.MOTOR_DATA,
        "timestamp_ms": int(time.time() * 1000),
        "payload": {"cortical_id": cortical_id, "activations": activations},
    }


def create_neuron_potential_message(
    cortical_areas: List[Dict[str, Any]],
) -> Dict[str, Any]:
    """
    Create a neuron potential data message.

    Args:
        cortical_areas: List of cortical areas with neuron data

    Returns:
        FSMP message data
    """
    return {
        "channel_id": 0,  # Generic channel
        "data_type": FSMPDataType.NEURON_POTENTIAL_DATA,
        "timestamp_ms": int(time.time() * 1000),
        "payload": {"cortical_areas": cortical_areas},
    }


def register_protocols():
    """Register FSMP protocol versions with the registry."""
    from feagi.api.protocols.base import ProtocolRegistry

    registry = ProtocolRegistry()
    registry.register(FSMPv1)
    registry.register(FSMPv2)


# Define all exported symbols
__all__ = [
    "FSMPChannelType",
    "FSMPDataType",
    "FSMPMessageFormat",
    "FSMPv1",
    "FSMPv2",
    "create_sensory_data_message",
    "create_motor_data_message",
    "create_neuron_potential_message",
    "register_protocols",
]
