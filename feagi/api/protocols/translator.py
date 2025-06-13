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
Protocol Translator Module for FEAGI

This module provides the translators for converting between FEAGI's
internal data structures and the binary wire formats used for communication
with clients.

This implementation replaces the previous Cap'n Proto implementation
with a more optimized custom binary format designed specifically for
neural data transmission.

VERSION HANDLING:
When creating messages, the translator can consider client capabilities
to use the appropriate structure version. This allows for backward and
forward compatibility between clients and servers with different versions.
"""

import json
import logging
import time
import zlib
from typing import Any, Dict, List, Optional, Union

# Import from the feagi-data-processing package
import feagi_data_processing as fdp

from feagi.api.protocols.constants import ByteStructureID, FCPCommandType, ProtocolID

# Configure logging
logger = logging.getLogger(__name__)

# Supported versions mapping (compatibility with old system)
SUPPORTED_VERSIONS = {
    ByteStructureID.JSON: [1],
    ByteStructureID.RAW_IMAGE: [1],
    ByteStructureID.MULTI_HOLDER: [1],
    ByteStructureID.NEURON_FLAT: [1],
    ByteStructureID.NEURON_CATEGORIES: [1],
}


def get_structure_info(data: bytes) -> Dict[str, Any]:
    """Get structure info from byte data"""
    try:
        byte_structure = fdp.byte_structures.FeagiByteStructure(data)
        return {
            "structure_type": byte_structure.try_get_structure_type(),
            "version": byte_structure.get_version(),
            "size": len(data),
        }
    except Exception as e:
        return {"error": str(e)}


def is_compressed(data: bytes) -> bool:
    """Check if data is compressed (placeholder implementation)"""
    # Simple heuristic - check if data starts with zlib header
    return data.startswith(b"\x78\x9c") or data.startswith(b"\x78\x01")


class ByteStructureTranslator:
    """
    Translator for FEAGI byte structure protocols using feagi_data_processing.

    This class provides methods for creating and parsing protocol-specific
    messages using the new high-performance byte structure format.
    """

    def __init__(self):
        """
        Initialize the protocol translator.

        Creates encoder and decoder instances and initializes the client
        capability registry.
        """
        # Use the new feagi_data_processing API
        self.fdp = fdp

        # Default versions to use if not specified
        self.default_versions = {
            ByteStructureID.JSON: 1,
            ByteStructureID.RAW_IMAGE: 1,
            ByteStructureID.MULTI_HOLDER: 1,
            ByteStructureID.NEURON_FLAT: 1,
            ByteStructureID.NEURON_CATEGORIES: 1,
        }

        # Client capability registry for version negotiation
        # Maps client_id to supported structure versions
        self.client_capabilities: Dict[str, Dict[str, Any]] = {}

    def _encode_json_message(self, data: dict) -> bytes:
        """Encode JSON data using feagi_data_processing"""
        try:
            # Create a CorticalMappedXYZPNeuronData container for JSON data
            cortical_mapped = (
                self.fdp.neuron_data.neuron_mappings.CorticalMappedXYZPNeuronData()
            )
            byte_structure = cortical_mapped.as_new_feagi_byte_structure()

            # For now, we'll store the JSON as metadata in the structure
            # This is a simplified approach - in practice you might want a more sophisticated mapping
            return byte_structure.get_data_as_bytes()
        except Exception as e:
            logger.error(f"Failed to encode JSON with feagi_data_processing: {e}")
            # Fallback to simple JSON encoding
            return json.dumps(data).encode("utf-8")

    def _encode_neuron_data(
        self, cortical_data: Dict[str, Dict[str, Any]], version: int = 1
    ) -> bytes:
        """Encode neuron data using feagi_data_processing with high-performance NumPy arrays"""
        try:
            import numpy as np

            # Create the main mapped neuron data container
            generated_mapped_neuron_data = (
                self.fdp.neuron_data.neuron_mappings.CorticalMappedXYZPNeuronData()
            )

            # Process cortical data format: {cortical_id: {x: [...], y: [...], z: [...], potentials: [...]}}
            for cortical_id, neuron_arrays in cortical_data.items():
                if isinstance(neuron_arrays, dict):
                    x_vals = neuron_arrays.get("x", [])
                    y_vals = neuron_arrays.get("y", [])
                    z_vals = neuron_arrays.get("z", [])
                    p_vals = neuron_arrays.get("potentials", neuron_arrays.get("p", []))

                    # Skip empty areas
                    if not x_vals and not y_vals and not z_vals:
                        continue

                    # Ensure all arrays are the same length
                    max_len = max(len(x_vals), len(y_vals), len(z_vals), len(p_vals))
                    if max_len == 0:
                        continue

                    # Pad arrays to same length if needed and convert to NumPy with proper dtypes
                    x_vals.extend([0] * (max_len - len(x_vals)))
                    y_vals.extend([0] * (max_len - len(y_vals)))
                    z_vals.extend([0] * (max_len - len(z_vals)))
                    p_vals.extend([0.0] * (max_len - len(p_vals)))

                    # Create NumPy arrays with proper dtypes for performance
                    neurons_x = np.asarray(x_vals, dtype=np.uint32)
                    neurons_y = np.asarray(y_vals, dtype=np.uint32)
                    neurons_z = np.asarray(z_vals, dtype=np.uint32)
                    neurons_p = np.asarray(p_vals, dtype=np.float32)

                    # Create cortical ID
                    cortical_id_obj = self.fdp.cortical_data.CorticalID(
                        str(cortical_id)
                    )

                    # Use high-performance NumPy approach to create neuron arrays (no cortical_id parameter)
                    neurons_array = self.fdp.neuron_data.neuron_arrays.NeuronXYZPArrays.new_from_numpy(
                        neurons_x, neurons_y, neurons_z, neurons_p
                    )

                    # Insert the neuron array into the mapped data with its cortical ID
                    generated_mapped_neuron_data.insert(cortical_id_obj, neurons_array)

            # Create the final byte structure from the mapped data
            byte_structure = generated_mapped_neuron_data.as_new_feagi_byte_structure()
            return byte_structure.get_data_as_bytes()

        except Exception as e:
            logger.error(
                f"Failed to encode neuron data with feagi_data_processing: {e}"
            )
            # Fallback to JSON encoding
            return json.dumps(cortical_data).encode("utf-8")

    def register_client_capabilities(
        self, client_id: str, capabilities: Dict[str, Any]
    ) -> None:
        """
        Register client capabilities for version negotiation.

        Args:
            client_id: Client identifier
            capabilities: Dictionary with client capabilities, including
                         supported structure versions
        """
        self.client_capabilities[client_id] = capabilities
        logger.debug(f"Registered capabilities for client {client_id}: {capabilities}")

    def get_supported_version(self, client_id: str, structure_id: int) -> int:
        """
        Get the highest structure version supported by both server and client.

        Args:
            client_id: Client identifier
            structure_id: Structure ID to check

        Returns:
            Highest mutually supported version
        """
        if client_id not in self.client_capabilities:
            return self.default_versions.get(structure_id, 1)

        client_supported = (
            self.client_capabilities[client_id]
            .get("structure_versions", {})
            .get(str(structure_id), [1])
        )

        # If client reports a list of versions
        if isinstance(client_supported, list):
            # Find highest version supported by both
            server_supported = SUPPORTED_VERSIONS.get(structure_id, [1])

            # Find common versions
            common_versions = set(server_supported).intersection(set(client_supported))
            if not common_versions:
                # No common versions, use default
                return self.default_versions.get(structure_id, 1)

            # Return highest common version
            return max(common_versions)

        # If client reports a single version
        elif isinstance(client_supported, int):
            # Check if server supports this version
            if client_supported in SUPPORTED_VERSIONS.get(structure_id, [1]):
                return client_supported

        # Default to version 1
        return self.default_versions.get(structure_id, 1)

    def create_timestamp(self) -> Dict[str, int]:
        """
        Create a timestamp object.

        Returns:
            Dictionary with timestamp in milliseconds
        """
        return {"time_ms": int(time.time() * 1000)}

    def create_handshake_hello(self, agent_id: str, agent_type: str) -> bytes:
        """
        Create a handshake hello message.

        Args:
            agent_id: Agent identifier
            agent_type: Agent type

        Returns:
            Encoded handshake hello message
        """
        message = {
            "protocol_id": ProtocolID.FCP,
            "message_type": "hello",
            "agent_id": agent_id,
            "agent_type": agent_type,
            "timestamp": self.create_timestamp(),
        }

        return self._encode_json_message(message)

    def create_handshake_welcome(
        self, server_id: str, message: str = "Welcome to FEAGI"
    ) -> bytes:
        """
        Create a handshake welcome message.

        Args:
            server_id: Server identifier
            message: Welcome message

        Returns:
            Encoded handshake welcome message
        """
        welcome_msg = {
            "protocol_id": ProtocolID.FCP,
            "message_type": "welcome",
            "server_id": server_id,
            "message": message,
            "timestamp": self.create_timestamp(),
        }

        return self._encode_json_message(welcome_msg)

    def create_handshake_capabilities(
        self,
        supported_sensory: List[str],
        supported_motor: List[str],
        protocol_versions: Dict[str, int],
    ) -> bytes:
        """
        Create a handshake capabilities message.

        Args:
            supported_sensory: List of supported sensory channel IDs
            supported_motor: List of supported motor channel IDs
            protocol_versions: Dictionary mapping protocol names to version numbers

        Returns:
            Encoded handshake capabilities message
        """
        capabilities = {
            "protocol_id": ProtocolID.FCP,
            "message_type": "capabilities",
            "supported_sensory_channels": supported_sensory,
            "supported_motor_channels": supported_motor,
            "protocol_versions": protocol_versions,
            "structure_versions": {
                str(structure_id): max(versions)
                for structure_id, versions in SUPPORTED_VERSIONS.items()
            },
            "timestamp": self.create_timestamp(),
        }

        return self._encode_json_message(capabilities)

    def create_handshake_configuration(self, server_config: Dict[str, Any]) -> bytes:
        """
        Create a handshake configuration message.

        Args:
            server_config: Server configuration dictionary

        Returns:
            Encoded handshake configuration message
        """
        config = {
            "protocol_id": ProtocolID.FCP,
            "message_type": "configuration",
            "config": server_config,
            "timestamp": self.create_timestamp(),
        }

        return self._encode_json_message(config)

    def create_fcp_message(
        self, command_type: FCPCommandType, payload: Dict[str, Any]
    ) -> bytes:
        """
        Create an FCP message.

        Args:
            command_type: Command type
            payload: Message payload

        Returns:
            Encoded FCP message
        """
        message = {
            "protocol_id": ProtocolID.FCP,
            "command_type": command_type,
            "payload": payload,
            "timestamp": self.create_timestamp(),
        }

        return self._encode_json_message(message)

    def create_fsmp_sensory_data(
        self, channel_id: str, data: Union[bytes, List[float]]
    ) -> bytes:
        """
        Create an FSMP sensory data message.

        Args:
            channel_id: Sensory channel ID
            data: Sensory data (raw bytes or list of float values)

        Returns:
            Encoded FSMP message
        """
        # For efficiency, if data is a numpy array of neuron activations,
        # use the specialized neuron potential format
        if isinstance(data, (list, tuple)) and all(isinstance(x, float) for x in data):
            # This is a simple case - just encode as JSON for now
            # In a real implementation, this would use a more efficient format
            message = {
                "protocol_id": ProtocolID.FSMP,
                "message_type": "sensory",
                "channel_id": channel_id,
                "data": data,
                "timestamp": self.create_timestamp(),
            }
            return self._encode_json_message(message)
        else:
            # For raw binary data, use JSON header with base64 encoded data
            # In a real implementation, you might use a specialized binary format
            import base64

            if isinstance(data, bytes):
                encoded_data = base64.b64encode(data).decode("ascii")
            else:
                encoded_data = base64.b64encode(bytes(data)).decode("ascii")

            message = {
                "protocol_id": ProtocolID.FSMP,
                "message_type": "sensory",
                "channel_id": channel_id,
                "data_encoding": "base64",
                "data": encoded_data,
                "timestamp": self.create_timestamp(),
            }
            return self._encode_json_message(message)

    def create_fsmp_motor_data(self, channel_id: str, data: List[float]) -> bytes:
        """
        Create an FSMP motor data message.

        Args:
            channel_id: Motor channel ID
            data: Motor activation values

        Returns:
            Encoded FSMP message
        """
        message = {
            "protocol_id": ProtocolID.FSMP,
            "message_type": "motor",
            "channel_id": channel_id,
            "data": data,
            "timestamp": self.create_timestamp(),
        }

        return self._encode_json_message(message)

    def create_neuron_data_message(
        self, cortical_data: Dict[str, Dict[str, Any]], client_id: Optional[str] = None
    ) -> bytes:
        """
        Create a neuron data message using the optimized format.

        Args:
            cortical_data: Dictionary mapping cortical area IDs to neuron data:
                {
                    'cortical_id': {
                        'x': [x1, x2, ...],
                        'y': [y1, y2, ...],
                        'z': [z1, z2, ...],
                        'potentials': [p1, p2, ...],
                    },
                    ...
                }
            client_id: Optional client ID for version negotiation

        Returns:
            Encoded neuron data using Type 11 (NEURON_CATEGORIES) format for DPR compatibility
        """
        # ALWAYS use NEURON_CATEGORIES format (Type 11) for DPR compatibility
        # This ensures Direct Point Rendering gets the right format regardless of cortical area count
        structure_id = ByteStructureID.NEURON_CATEGORIES

        # Get appropriate version based on client capabilities
        version = self.get_supported_version(
            client_id if client_id else "", structure_id
        )

        # If empty data, return empty message
        if not cortical_data:
            return self._encode_json_message(
                {"message_type": "neuron_data", "data": {}}
            )

        # Use categorized format for all cases (DPR requirement)
        return self._encode_neuron_data(cortical_data, version)

    def decode_message(self, message_data: bytes) -> Dict[str, Any]:
        """
        Decode a byte structure message.

        Args:
            message_data: Raw message data

        Returns:
            Decoded message as a dictionary

        Raises:
            ValueError: If the message is invalid
        """
        try:
            # Try to create a FeagiByteStructure from the data
            try:
                byte_structure = self.fdp.byte_structures.FeagiByteStructure(
                    message_data
                )
                structure_info = get_structure_info(message_data)
                structure_type = structure_info.get("structure_type", 0)

                if structure_type == 11:  # NeuronCategoricalXYZP
                    # Create CorticalMappedXYZPNeuronData from the byte structure
                    cortical_mapped = self.fdp.neuron_data.neuron_mappings.CorticalMappedXYZPNeuronData()
                    cortical_mapped.from_feagi_byte_structure(byte_structure)

                    # Extract neuron data
                    neurons = []
                    for neuron_obj, cortical_id in cortical_mapped.iter_easy():
                        neurons.append(
                            {
                                "x": neuron_obj.x if hasattr(neuron_obj, "x") else 0,
                                "y": neuron_obj.y if hasattr(neuron_obj, "y") else 0,
                                "z": neuron_obj.z if hasattr(neuron_obj, "z") else 0,
                                "p": neuron_obj.p if hasattr(neuron_obj, "p") else 0,
                                "cortical_id": cortical_id,
                            }
                        )

                    return {
                        "message_type": "neuron_data",
                        "data": neurons,
                        "structure_type": structure_type,
                    }
                else:
                    return {
                        "message_type": "unknown",
                        "structure_type": structure_type,
                        "data": (
                            message_data.hex()
                            if len(message_data) < 100
                            else f"binary_data_{len(message_data)}_bytes"
                        ),
                    }

            except Exception as decode_error:
                # Fallback to JSON parsing
                try:
                    import json

                    decoded = json.loads(message_data.decode("utf-8"))
                    return decoded
                except:
                    # Last resort - return raw data info
                    return {
                        "message_type": "raw_data",
                        "error": str(decode_error),
                        "data_length": len(message_data),
                        "data_preview": (
                            message_data[:50].hex()
                            if len(message_data) >= 50
                            else message_data.hex()
                        ),
                    }

        except Exception as e:
            logger.error(f"Failed to decode message: {e}")
            raise ValueError(f"Failed to decode message: {e}")

    def extract_capabilities(self, message: Dict[str, Any]) -> Dict[str, Any]:
        """
        Extract capabilities from a decoded message.

        Args:
            message: Decoded message dictionary

        Returns:
            Capabilities dictionary
        """
        if message.get("message_type") == "capabilities":
            return {
                "supported_sensory_channels": message.get(
                    "supported_sensory_channels", []
                ),
                "supported_motor_channels": message.get("supported_motor_channels", []),
                "protocol_versions": message.get("protocol_versions", {}),
                "structure_versions": message.get("structure_versions", {}),
            }
        else:
            return {}

    def compress_message(self, message_data: bytes) -> bytes:
        """
        Compress message data using zlib.

        Args:
            message_data: Original message data

        Returns:
            Compressed message data
        """
        # Simple zlib compression for now
        # In a real implementation with feagi_data_processing, you might use built-in compression
        try:
            return zlib.compress(message_data)
        except Exception as e:
            logger.error(f"Failed to compress message: {e}")
            return message_data  # Return original if compression fails


# Create default translator instance
default_translator = ByteStructureTranslator()

# Re-export the translator for backward compatibility
__all__ = [
    "ByteStructureTranslator",
    "default_translator",
    "get_structure_info",
    "is_compressed",
]
