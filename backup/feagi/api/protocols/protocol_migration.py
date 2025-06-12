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
Protocol Migration Guide

This file demonstrates how to transition from the old single-file protocol structure
to the new versioned file structure. It shows equivalent code for both approaches.
"""

import time
from typing import Any, Dict, List

from feagi.api.protocols.base import ProtocolRegistry
from feagi.api.protocols.fsmp import (
    FSMPDataType,
    create_motor_data_message,
    create_sensory_data_message,
)

# New approach (versioned files):
# ------------------------------
# Direct import from a specific version
from feagi.api.protocols.fsmp.v1 import FSMPv1
from feagi.api.protocols.fsmp.v2 import FSMPv2

# Old approach (single file):
# ---------------------------
# from feagi.api.protocols.fsmp import FSMPv1, FSMPDataType, create_sensory_data_message

# # Encode a message using FSMPv1
# message_data = create_sensory_data_message(
#     channel_id=1,
#     cortical_id="vision",
#     activations=[0.5, 0.2, 0.7]
# )
# binary_data = FSMPv1.encode(message_data)

# # Get the registry and find appropriate protocol
# registry = ProtocolRegistry()
# protocol_class = registry.get_protocol(protocol_id=1, version=1)  # FSMP is protocol_id 1
# decoded_data = protocol_class.decode(binary_data)


# Encode using a specific version
def example_encode_specific_version():
    # Create message data
    message_data = create_sensory_data_message(
        channel_id=1, cortical_id="vision", activations=[0.5, 0.2, 0.7]
    )

    # Encode using a specific version
    binary_data = FSMPv1.encode(message_data)
    return binary_data


# Use the registry (version agnostic approach)
def example_use_registry():
    binary_data = example_encode_specific_version()

    # Get the registry and find appropriate protocol
    registry = ProtocolRegistry()
    protocol_class = registry.get_protocol(
        protocol_id=1, version=1
    )  # FSMP is protocol_id 1

    # Decode using the registry-provided protocol class
    decoded_data = protocol_class.decode(binary_data)
    return decoded_data


# Using newer versions
def example_use_v2():
    message_data = create_motor_data_message(
        channel_id=1, cortical_id="motor", activations=[0.9, 0.1]
    )

    # Use v2 directly
    binary_data = FSMPv2.encode(message_data)

    # Or use the registry to get the highest available version:
    registry = ProtocolRegistry()
    latest_fsmp = registry.get_latest_protocol(protocol_id=1)  # Get latest FSMP version
    binary_data = latest_fsmp.encode(message_data)
    return binary_data


# Negotiating versions between client and server
def example_version_negotiation():
    """
    Example of client-server version negotiation.

    In a real-world scenario, the client would inform the server about
    supported protocol versions, and the server would select the highest
    version that both support.
    """
    # Client supported versions
    client_supported_versions = [1, 2]

    # Server gets this information and selects the best version
    registry = ProtocolRegistry()
    available_versions = registry.get_available_versions(protocol_id=1)  # FSMP

    # Find highest common version
    common_versions = [v for v in client_supported_versions if v in available_versions]
    if not common_versions:
        raise ValueError("No compatible protocol version found")

    # Select highest common version
    selected_version = max(common_versions)

    # Get the protocol class for this version
    protocol_class = registry.get_protocol(protocol_id=1, version=selected_version)

    # Now use this protocol class for communication
    message_data = create_sensory_data_message(
        channel_id=1, cortical_id="vision", activations=[0.5, 0.2, 0.7]
    )
    binary_data = protocol_class.encode(message_data)
    return binary_data
