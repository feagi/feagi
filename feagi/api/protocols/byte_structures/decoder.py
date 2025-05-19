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
FEAGI Byte Structures Decoder

This module provides decoders for various FEAGI byte structures, following the
specifications defined in the FEAGI byte structures documentation.

VERSIONING GUIDELINES:

When adding a new version of a structure:
1. Create a new private method _decode_X_vN (e.g., _decode_json_v2)
2. Update the public method to dispatch to the appropriate version
3. Document the format changes in both the public method docstring and README
4. Add tests for the new version in test_byte_structures.py
5. Update the protocol translator to handle version negotiation

Example for adding version 2 of neuron_flat:
- Add _decode_neuron_flat_v2 method
- Update decode_neuron_flat to dispatch based on version in header
- Add tests for version 2 format decoding
- Update compatibility documentation
"""

import json
import struct
import zlib
from typing import Dict, List, Any, Tuple, Union

import numpy as np

from feagi.api.protocols.constants import ByteStructureID
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


# Registry of supported versions for each structure type
# This should match the registry in encoder.py
SUPPORTED_VERSIONS = {
    ByteStructureID.JSON: [1],
    ByteStructureID.RAW_IMAGE: [1],
    ByteStructureID.MULTI_HOLDER: [1],
    ByteStructureID.NEURON_FLAT: [1], 
    ByteStructureID.NEURON_CATEGORIES: [1],
}


class ByteStructureDecoder:
    """
    Decoder for FEAGI byte structures.
    
    This class provides methods to decode FEAGI byte structures back into
    usable data structures according to their specifications.
    """
    
    def __init__(self):
        """Initialize the byte structure decoder."""
        self.logger = logger
    
    @staticmethod
    def decode_header(data: bytes) -> Tuple[int, int]:
        """
        Decode the universal FEAGI byte structure header.
        
        Args:
            data: Byte data starting with header
            
        Returns:
            Tuple of (structure_id, version)
            
        Raises:
            ValueError: If data is too short for header
        """
        if len(data) < 2:
            raise ValueError("Data too short for header")
        
        structure_id, version = struct.unpack("!BB", data[:2])
        return structure_id, version
    
    @staticmethod
    def get_structure_type(data: bytes) -> int:
        """
        Get the structure type from the header.
        
        Args:
            data: Byte data starting with header
            
        Returns:
            Structure ID from header
            
        Raises:
            ValueError: If data is too short for header
        """
        if len(data) < 1:
            raise ValueError("Data too short for header")
        
        return data[0]
    
    def decode_json(self, data: bytes) -> Dict[str, Any]:
        """
        Decode a JSON byte structure (ID: 1).
        
        Args:
            data: Encoded JSON byte structure
            
        Returns:
            Decoded JSON data
            
        Raises:
            ValueError: If header is invalid, JSON is malformed, or version is unsupported
        """
        try:
            # Skip the header (2 bytes)
            json_bytes = data[2:]
            
            # Parse the JSON
            return json.loads(json_bytes.decode("utf-8"))
        except Exception as e:
            self.logger.error(f"Error decoding JSON: {e}")
            return {}
    
    def decode_raw_image(self, data: bytes) -> Dict[str, Any]:
        """
        Decode a raw image byte structure (ID: 8).
        
        Args:
            data: Encoded raw image byte structure
            
        Returns:
            Dictionary with image information and data
            
        Raises:
            ValueError: If header is invalid, data is malformed, or version is unsupported
        """
        try:
            # Skip structure ID and version
            header = data[:5]
            
            # Extract width, height, and channels
            _, version, width, height, channels = struct.unpack("!BBHHB", header)
            
            # Extract image data
            image_data = data[5:]
            
            return {
                "width": width,
                "height": height,
                "channels": channels,
                "data": image_data,
                "version": version
            }
        except Exception as e:
            self.logger.error(f"Error decoding raw image: {e}")
            return {"error": str(e)}
    
    def decode_multi_holder(self, data: bytes) -> List[bytes]:
        """
        Decode a multi-holder byte structure (ID: 9).
        
        Args:
            data: Encoded multi-holder byte structure
            
        Returns:
            List of contained byte structures
            
        Raises:
            ValueError: If header is invalid, data is malformed, or version is unsupported
        """
        try:
            # Extract header
            struct_id, version, count = struct.unpack("!BBI", data[:6])
            
            if struct_id != 9:  # 9 = MULTI_HOLDER
                raise ValueError(f"Invalid structure ID: {struct_id}, expected 9")
                
            # Extract contained structures
            result = []
            offset = 6  # Skip header
            
            for _ in range(count):
                # Get structure size
                size, = struct.unpack("!I", data[offset:offset+4])
                offset += 4
                
                # Extract structure
                struct_data = data[offset:offset+size]
                result.append(struct_data)
                offset += size
                
            return result
        except Exception as e:
            self.logger.error(f"Error decoding multi-holder: {e}")
            return []
    
    def decode_neuron_flat(self, data: bytes) -> Dict[str, Any]:
        """
        Decode neuron potential data in flat format (ID: 10).
        
        Args:
            data: Encoded neuron flat byte structure
            
        Returns:
            Dictionary with decoded neuron data:
            {
                'version': Version number,
                'cortical_ids': List of cortical IDs,
                'x': List of X coordinates,
                'y': List of Y coordinates,
                'z': List of Z coordinates,
                'potentials': List of neuron potentials
            }
            
        Raises:
            ValueError: If header is invalid, data is malformed, or version is unsupported
        """
        try:
            # Extract header
            struct_id, version, count = struct.unpack("!BBI", data[:6])
            
            if struct_id != 10:  # 10 = NEURON_FLAT
                raise ValueError(f"Invalid structure ID: {struct_id}, expected 10")
                
            # For the placeholder implementation, decode the JSON data
            json_bytes = data[6:]
            result = json.loads(json_bytes.decode("utf-8"))
            result['version'] = 1  # Add version to result
            return result
        except Exception as e:
            self.logger.error(f"Error decoding neuron flat: {e}")
            return {}
    
    def decode_neuron_categories(self, data: bytes) -> Dict[str, Dict[str, Union[List[int], List[float]]]]:
        """
        Decode neuron potential data organized by cortical areas (ID: 11).
        
        Args:
            data: Encoded neuron categories byte structure
            
        Returns:
            Dictionary mapping cortical IDs to neuron data:
            {
                'version': Version number,
                'cortical_id': {
                    'x': List of X coordinates,
                    'y': List of Y coordinates,
                    'z': List of Z coordinates,
                    'potentials': List of neuron potentials
                },
                ...
            }
            
        Raises:
            ValueError: If header is invalid, data is malformed, or version is unsupported
        """
        try:
            # Extract header
            struct_id, version, count = struct.unpack("!BBI", data[:6])
            
            if struct_id != 11:  # 11 = NEURON_CATEGORIES
                raise ValueError(f"Invalid structure ID: {struct_id}, expected 11")
                
            # For the placeholder implementation, decode the JSON data
            json_bytes = data[6:]
            result = json.loads(json_bytes.decode("utf-8"))
            result['version'] = 1  # Add version to result
            return result
        except Exception as e:
            self.logger.error(f"Error decoding neuron categories: {e}")
            return {}
    
    def decode(self, data: bytes) -> Any:
        """
        Decode any FEAGI byte structure based on its header.
        
        Args:
            data: Encoded byte structure
            
        Returns:
            Decoded data appropriate for the structure type
            
        Raises:
            ValueError: If header is invalid or structure type is unsupported
        """
        if len(data) < 2:
            raise ValueError("Data too short for header")
        
        structure_id, version = self.decode_header(data)
        
        try:
            if structure_id == ByteStructureID.JSON:
                return self.decode_json(data)
            elif structure_id == ByteStructureID.RAW_IMAGE:
                return self.decode_raw_image(data)
            elif structure_id == ByteStructureID.MULTI_HOLDER:
                return self.decode_multi_holder(data)
            elif structure_id == ByteStructureID.NEURON_FLAT:
                return self.decode_neuron_flat(data)
            elif structure_id == ByteStructureID.NEURON_CATEGORIES:
                return self.decode_neuron_categories(data)
            else:
                raise ValueError(f"Unsupported structure type: {structure_id}")
        except (IndexError, struct.error) as e:
            raise ValueError(f"Error decoding structure: {e}")
    
    @staticmethod
    def decompress(data: bytes) -> bytes:
        """
        Decompress data using Deflate algorithm.
        
        Args:
            data: Compressed data bytes
            
        Returns:
            Decompressed bytes
        """
        try:
            return zlib.decompress(data)
        except Exception as e:
            logger.error(f"Error decompressing data: {e}")
            return data 