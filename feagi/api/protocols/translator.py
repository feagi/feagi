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
Protocol translator for FEAGI communications.

This module provides tools for translating between different protocol versions
and formats, enabling backward compatibility and client-server negotiation.
"""

import asyncio
import json
import struct
import time
import zlib
import logging
from typing import Dict, Any, List, Optional, Union, Type, Set, Tuple

from feagi.utils.logger import setup_logger
logger = setup_logger()

# Mock implementations for the feagi_bytes classes
class ByteStructureEncoder:
    """Mock implementation of the ByteStructureEncoder class."""
    
    def encode_json(self, data):
        """Encode JSON data."""
        return json.dumps(data).encode('utf-8')
    
    def compress(self, data):
        """Compress binary data."""
        return zlib.compress(data)

class ByteStructureDecoder:
    """Mock implementation of the ByteStructureDecoder class."""
    
    def decode_json(self, data):
        """Decode JSON data."""
        return json.loads(data.decode('utf-8'))
    
    def decompress(self, data):
        """Decompress binary data."""
        return zlib.decompress(data)

# Import after the mock classes to avoid circular import issues
from feagi.api.protocols.constants import ProtocolID, ByteStructureID, FCPCommandType

# Attempt to import the protocol byte structures, but provide fallbacks if they're not available
try:
    from feagi.api.protocols.byte_structures.fcp import ControlMessage
except ImportError:
    class ControlMessage:
        @staticmethod
        def encode(command_type, payload): return b""
        @staticmethod
        def decode(data): return {}

try:
    from feagi.api.protocols.byte_structures.fsmp import SensorimotorMessage
except ImportError:
    class SensorimotorMessage:
        @staticmethod
        def encode(message_type, channel_id, data): return b""
        @staticmethod
        def decode(data): return {}

try:
    from feagi.api.protocols.byte_structures.fvp import VisualizationMessage
except ImportError:
    class VisualizationMessage:
        @staticmethod
        def encode(message_type, data): return b""
        @staticmethod
        def decode(data): return {}

# These imports are provided as a fallback
try:
    from feagi_bytes.serialization import ByteSerializer, deserialize_message, serialize_message
    from feagi_bytes.utils import get_structure_info, is_compressed
    from feagi_bytes.serialization import SUPPORTED_VERSIONS
except ImportError:
    # Mock implementations if real ones are not available
    ByteSerializer = object
    deserialize_message = lambda x: {}
    serialize_message = lambda x: b""
    get_structure_info = lambda x: (1, 1)  # Default structure ID and version
    is_compressed = lambda x: False
    SUPPORTED_VERSIONS = {ByteStructureID.JSON: [1], ByteStructureID.RAW_IMAGE: [1],
                         ByteStructureID.MULTI_HOLDER: [1], ByteStructureID.NEURON_FLAT: [1],
                         ByteStructureID.NEURON_CATEGORIES: [1]}

try:
    from .base import ProtocolManager
except ImportError:
    # Mock implementation if real one is not available
    class ProtocolManager:
        def __init__(self):
            pass
        
        def encode_message(self, data, protocol_id, version):
            return b""
            
        def decode_message(self, data):
            return {}, ProtocolID.FCP, 1
            
        def get_compatible_version(self, protocol_id, supported_versions):
            return 1 if supported_versions else None
            
        def get_latest_version(self, protocol_id):
            return 1

# Configure logging
logger = setup_logger(__name__)

# Use the provided ByteStructureTranslator directly
default_translator = None  # Will be set after class definition

# Re-export the translator for backward compatibility
__all__ = ['ByteStructureTranslator', 'default_translator', 'ProtocolTranslator']


class ByteStructureTranslator:
    """
    Translator for FEAGI byte structure protocols.
    
    This class provides methods for creating and parsing protocol-specific
    messages using the byte structure format.
    """
    
    def __init__(self):
        """
        Initialize the protocol translator.
        
        Creates encoder and decoder instances and initializes the client
        capability registry.
        """
        self.encoder = ByteStructureEncoder()
        self.decoder = ByteStructureDecoder()
        
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
    
    def register_client_capabilities(self, client_id: str, capabilities: Dict[str, Any]) -> None:
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
        
        client_supported = self.client_capabilities[client_id].get(
            "structure_versions", {}
        ).get(str(structure_id), [1])
        
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
            "timestamp": self.create_timestamp()
        }
        
        return self.encoder.encode_json(message)
    
    def create_handshake_welcome(self, server_id: str, message: str = "Welcome to FEAGI") -> bytes:
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
            "timestamp": self.create_timestamp()
        }
        
        return self.encoder.encode_json(welcome_msg)
    
    def create_handshake_capabilities(self,
                                    supported_sensory: List[str],
                                    supported_motor: List[str],
                                    protocol_versions: Dict[str, int]) -> bytes:
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
            "timestamp": self.create_timestamp()
        }
        
        return self.encoder.encode_json(capabilities)
    
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
            "timestamp": self.create_timestamp()
        }
        
        return self.encoder.encode_json(config)
    
    def create_fcp_message(self, 
                         command_type: FCPCommandType, 
                         payload: Dict[str, Any]) -> bytes:
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
            "timestamp": self.create_timestamp()
        }
        
        return self.encoder.encode_json(message)
    
    def create_fsmp_sensory_data(self, 
                                channel_id: str, 
                                data: Union[bytes, List[float]]) -> bytes:
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
                "timestamp": self.create_timestamp()
            }
            return self.encoder.encode_json(message)
        else:
            # For raw binary data, use JSON header with base64 encoded data
            # In a real implementation, you might use a specialized binary format
            import base64
            if isinstance(data, bytes):
                encoded_data = base64.b64encode(data).decode('ascii')
            else:
                encoded_data = base64.b64encode(bytes(data)).decode('ascii')
                
            message = {
                "protocol_id": ProtocolID.FSMP,
                "message_type": "sensory",
                "channel_id": channel_id,
                "data_encoding": "base64",
                "data": encoded_data,
                "timestamp": self.create_timestamp()
            }
            return self.encoder.encode_json(message)
    
    def create_fsmp_motor_data(self, 
                              channel_id: str, 
                              data: List[float]) -> bytes:
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
            "timestamp": self.create_timestamp()
        }
        
        return self.encoder.encode_json(message)
    
    def create_neuron_data_message(self,
                                  cortical_data: Dict[str, Dict[str, Any]],
                                  client_id: Optional[str] = None) -> bytes:
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
            Encoded neuron data using the appropriate format
        """
        # Determine format based on number of cortical areas
        if len(cortical_data) > 1:
            # Use categorized format for multiple cortical areas
            structure_id = ByteStructureID.NEURON_CATEGORIES
            
            # Get appropriate version based on client capabilities
            version = self.get_supported_version(
                client_id if client_id else "", 
                structure_id
            )
            
            return self.encoder.encode_neuron_categories(
                cortical_data=cortical_data,
                version=version
            )
        
        # Use flat format for a single cortical area
        elif len(cortical_data) == 1:
            cortical_id, data = list(cortical_data.items())[0]
            
            # Extract data
            x_coords = data['x']
            y_coords = data['y']
            z_coords = data['z']
            potentials = data['potentials']
            
            # Create a list of cortical IDs (same ID for all neurons)
            neuron_count = len(x_coords)
            cortical_ids = [cortical_id] * neuron_count
            
            # Get appropriate version based on client capabilities
            structure_id = ByteStructureID.NEURON_FLAT
            version = self.get_supported_version(
                client_id if client_id else "", 
                structure_id
            )
            
            return self.encoder.encode_neuron_flat(
                cortical_ids=cortical_ids,
                x_coords=x_coords,
                y_coords=y_coords,
                z_coords=z_coords,
                potentials=potentials,
                version=version
            )
        else:
            # Empty data
            return self.encoder.encode_json({"message_type": "neuron_data", "data": {}})
    
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
            # Check if data is compressed
            try:
                # Try to decompress, but handle case where it's not compressed
                if is_compressed(message_data):
                    decompressed_data = self.decoder.decompress(message_data)
                    message_data = decompressed_data
            except:
                # Not compressed, continue with original data
                pass
            
            # Get structure type
            if len(message_data) < 1:
                raise ValueError("Message too short")
                
            structure_id, version = get_structure_info(message_data)
            
            # Decode based on structure type
            if structure_id == ByteStructureID.JSON:
                return self.decoder.decode_json(message_data)
            elif structure_id == ByteStructureID.RAW_IMAGE:
                return {"message_type": "raw_image", "data": self.decoder.decode_raw_image(message_data)}
            elif structure_id == ByteStructureID.MULTI_HOLDER:
                contained_structures = self.decoder.decode_multi_holder(message_data)
                decoded_structures = [self.decode_message(struct) for struct in contained_structures]
                return {"message_type": "multi_holder", "structures": decoded_structures}
            elif structure_id == ByteStructureID.NEURON_FLAT:
                return {"message_type": "neuron_data", "data": self.decoder.decode_neuron_flat(message_data)}
            elif structure_id == ByteStructureID.NEURON_CATEGORIES:
                return {"message_type": "neuron_data", "data": self.decoder.decode_neuron_categories(message_data)}
            else:
                raise ValueError(f"Unknown structure type: {structure_id}")
                
        except Exception as e:
            logger.error(f"Error decoding message: {e}")
            raise ValueError(f"Failed to decode message: {e}")
    
    def extract_capabilities(self, message: Dict[str, Any]) -> Dict[str, Any]:
        """
        Extract client capabilities from a handshake message.
        
        Args:
            message: Decoded handshake capabilities message
            
        Returns:
            Dictionary of client capabilities
        """
        capabilities = {}
        
        # Extract structure versions
        if "structure_versions" in message:
            # Convert string keys back to integers
            structure_versions = {
                int(k): v for k, v in message["structure_versions"].items()
            }
            capabilities["structure_versions"] = structure_versions
            
        # Extract protocol versions
        if "protocol_versions" in message:
            capabilities["protocol_versions"] = message["protocol_versions"]
            
        # Extract sensory/motor channels
        if "supported_sensory_channels" in message:
            capabilities["supported_sensory_channels"] = message["supported_sensory_channels"]
            
        if "supported_motor_channels" in message:
            capabilities["supported_motor_channels"] = message["supported_motor_channels"]
        
        return capabilities
    
    def compress_message(self, message_data: bytes) -> bytes:
        """
        Compress a message using the Deflate algorithm.
        
        Args:
            message_data: Raw message data
            
        Returns:
            Compressed message data
        """
        return self.encoder.compress(message_data)


class ProtocolTranslator:
    """
    Translates between different protocol versions.
    
    This class helps negotiate compatible protocol versions between
    the server and clients, and provides encoding/decoding services.
    """
    
    def __init__(self):
        """Initialize the protocol translator."""
        self.protocol_manager = ProtocolManager()
        self.client_protocols: Dict[str, Dict[str, List[int]]] = {}
    
    def register_agent(self, agent_id: str, protocol_versions: Dict[str, Union[int, List[int]]]) -> Dict[str, int]:
        """
        Register an agent and negotiate compatible protocol versions.
        
        Args:
            agent_id: Unique agent identifier
            protocol_versions: Protocol versions supported by the agent, format:
                               {"protocol_name": version} or {"protocol_name": [versions]}
                               
        Returns:
            Dictionary of negotiated protocol versions {"protocol_name": version}
            
        Raises:
            ValueError: If no compatible protocol versions are found
        """
        normalized_versions: Dict[str, List[int]] = {}
        
        # Normalize input
        for protocol_name, versions in protocol_versions.items():
            if isinstance(versions, int):
                normalized_versions[protocol_name] = [versions]
            else:
                normalized_versions[protocol_name] = versions
                
        # Store client protocols
        self.client_protocols[agent_id] = normalized_versions
        
        # Negotiate compatible versions
        compatible_versions = {}
        
        for protocol_name, versions in normalized_versions.items():
            # Map protocol name to ID
            try:
                protocol_id = ProtocolID[protocol_name]
            except (KeyError, ValueError):
                continue
                
            # Get compatible version
            compatible_version = self.protocol_manager.get_compatible_version(protocol_id, versions)
            
            if compatible_version is not None:
                compatible_versions[protocol_name] = compatible_version
            elif len(versions) > 0:
                # No compatible version found, raise error
                raise ValueError(f"No compatible version for protocol {protocol_name}")
        
        if not compatible_versions:
            raise ValueError("No compatible protocols found")
            
        return compatible_versions
    
    def encode(self, agent_id: str, message: Dict[str, Any], protocol_name: str) -> bytes:
        """
        Encode a message for a specific agent using the negotiated protocol version.
        
        Args:
            agent_id: Agent ID
            message: Message to encode
            protocol_name: Protocol name
            
        Returns:
            Encoded binary message
            
        Raises:
            ValueError: If the agent is not registered or the protocol is not supported
        """
        if agent_id not in self.client_protocols:
            raise ValueError(f"Agent {agent_id} not registered")
            
        if protocol_name not in self.client_protocols[agent_id]:
            raise ValueError(f"Protocol {protocol_name} not supported by agent {agent_id}")
            
        try:
            protocol_id = ProtocolID[protocol_name]
        except (KeyError, ValueError):
            raise ValueError(f"Unknown protocol {protocol_name}")
            
        # Get the version as an integer (not a list)
        version = self.client_protocols[agent_id][protocol_name]
        if isinstance(version, list) and version:
            version = version[0]  # Use the first version in the list
        
        # Encode message
        encoded = self.protocol_manager.encode_message(message, protocol_id, version)
        
        return encoded
    
    def decode(self, binary_data: bytes) -> Tuple[Dict[str, Any], str, int]:
        """
        Decode a binary message with protocol header.
        
        Args:
            binary_data: Binary data to decode
            
        Returns:
            Tuple of (decoded message, protocol name, version)
            
        Raises:
            ValueError: If the protocol is not supported
        """
        # Let the protocol manager decode the message
        decoded, protocol_id, version = self.protocol_manager.decode_message(binary_data)
        
        # Convert protocol ID to name
        protocol_name = protocol_id.name
        
        return decoded, protocol_name, version

# Set the default_translator after class definition
default_translator = ByteStructureTranslator() 