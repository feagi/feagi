"""
FEAGI Protocol Translator for Byte Structures

This module provides a translator between FEAGI's internal data structures
and the binary byte structure format used for efficient communication.
"""

import json
import logging
from typing import Dict, Any, List, Optional, Union

from .constants import ByteStructureID
from .serialization import ByteStructureEncoder, ByteStructureDecoder, SUPPORTED_VERSIONS
from .utils import get_structure_info, is_compressed

# Configure logging
logger = logging.getLogger(__name__)


class ByteStructureTranslator:
    """
    Translator for FEAGI byte structure protocols.
    
    This class provides methods for creating and parsing protocol-specific
    messages using the byte structure format with version negotiation.
    """
    
    def __init__(self):
        """
        Initialize the protocol translator.
        
        Creates encoder and decoder instances and initializes the client
        capability registry.
        """
        self.encoder = ByteStructureEncoder()
        self.decoder = ByteStructureDecoder()
        
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
            return self.encoder.default_versions.get(structure_id, 1)
        
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
                return self.encoder.default_versions.get(structure_id, 1)
                
            # Return highest common version
            return max(common_versions)
        
        # If client reports a single version
        elif isinstance(client_supported, int):
            # Check if server supports this version
            if client_supported in SUPPORTED_VERSIONS.get(structure_id, [1]):
                return client_supported
            
        # Default to version 1
        return self.encoder.default_versions.get(structure_id, 1)
    
    def create_message(self, data: Dict[str, Any], client_id: Optional[str] = None) -> bytes:
        """
        Create a message using JSON byte structure.
        
        Args:
            data: Message data to encode
            client_id: Client identifier for version negotiation (optional)
            
        Returns:
            Encoded message bytes
        """
        # Get appropriate version based on client capabilities
        version = self.get_supported_version(
            client_id if client_id else "", 
            ByteStructureID.JSON
        )
        
        return self.encoder.encode_json(data, version)
    
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
            except Exception as e:
                # Not compressed or decompression failed, continue with original data
                logger.debug(f"Decompression failed or not needed: {e}")
                pass
            
            # Get structure type
            if len(message_data) < 2:
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
            
    def compress_message(self, message_data: bytes) -> bytes:
        """
        Compress a message using the Deflate algorithm.
        
        Args:
            message_data: Raw message data
            
        Returns:
            Compressed message data
        """
        return self.encoder.compress(message_data)
        
    def create_neuron_data_message(
        self,
        cortical_data: Dict[str, Dict[str, Any]],
        client_id: Optional[str] = None
    ) -> bytes:
        """
        Create a neuron data message using the appropriate format.
        
        Args:
            cortical_data: Dictionary mapping cortical IDs to neuron data
                          Each value contains 'x', 'y', 'z', and 'potentials'
            client_id: Client identifier for version negotiation (optional)
            
        Returns:
            Encoded neuron data message
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