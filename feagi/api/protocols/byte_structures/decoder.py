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
        # Validate header and get version
        structure_id, version = self.decode_header(data)
        if structure_id != ByteStructureID.JSON:
            raise ValueError(f"Expected JSON structure (ID: 1), got {structure_id}")
        
        # Check if version is supported
        if version not in SUPPORTED_VERSIONS[ByteStructureID.JSON]:
            raise ValueError(f"Unsupported version {version} for JSON structure")
        
        # Dispatch to version-specific decoder
        if version == 1:
            return self._decode_json_v1(data)
        else:
            # Should never reach here due to the check above
            raise ValueError(f"Version {version} implementation missing for JSON structure")
    
    def _decode_json_v1(self, data: bytes) -> Dict[str, Any]:
        """Decode version 1 of JSON structure."""
        # Extract JSON data (skip header)
        json_bytes = data[2:]
        
        try:
            # Decode JSON from UTF-8 bytes
            json_str = json_bytes.decode('utf-8')
            return json.loads(json_str)
        except (UnicodeDecodeError, json.JSONDecodeError) as e:
            raise ValueError(f"Failed to decode JSON: {e}")
    
    def decode_raw_image(self, data: bytes) -> np.ndarray:
        """
        Decode a raw image byte structure (ID: 8).
        
        Args:
            data: Encoded raw image byte structure
            
        Returns:
            Image as numpy array (height, width, 3) in BGR format
            
        Raises:
            ValueError: If header is invalid, data is malformed, or version is unsupported
        """
        # Validate header and get version
        structure_id, version = self.decode_header(data)
        if structure_id != ByteStructureID.RAW_IMAGE:
            raise ValueError(f"Expected raw image structure (ID: 8), got {structure_id}")
        
        # Check if version is supported
        if version not in SUPPORTED_VERSIONS[ByteStructureID.RAW_IMAGE]:
            raise ValueError(f"Unsupported version {version} for raw image structure")
        
        # Dispatch to version-specific decoder
        if version == 1:
            return self._decode_raw_image_v1(data)
        else:
            # Should never reach here due to the check above
            raise ValueError(f"Version {version} implementation missing for raw image structure")
    
    def _decode_raw_image_v1(self, data: bytes) -> np.ndarray:
        """Decode version 1 of raw image structure."""
        # Minimum size check: header (2) + sub-header (4)
        if len(data) < 6:
            raise ValueError("Data too short for raw image structure")
        
        # Extract dimensions from sub-header
        width, height = struct.unpack("!HH", data[2:6])
        
        # Calculate expected size
        expected_size = 6 + (width * height * 3)  # Header + sub-header + pixel data
        if len(data) < expected_size:
            raise ValueError(f"Incomplete image data: expected {expected_size} bytes, got {len(data)}")
        
        # Extract image data
        image_bytes = data[6:expected_size]
        
        # Reshape into numpy array
        try:
            image = np.frombuffer(image_bytes, dtype=np.uint8).reshape(height, width, 3)
            return image
        except ValueError as e:
            raise ValueError(f"Failed to reshape image data: {e}")
    
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
        # Validate header and get version
        structure_id, version = self.decode_header(data)
        if structure_id != ByteStructureID.MULTI_HOLDER:
            raise ValueError(f"Expected multi-holder structure (ID: 9), got {structure_id}")
        
        # Check if version is supported
        if version not in SUPPORTED_VERSIONS[ByteStructureID.MULTI_HOLDER]:
            raise ValueError(f"Unsupported version {version} for multi-holder structure")
        
        # Dispatch to version-specific decoder
        if version == 1:
            return self._decode_multi_holder_v1(data)
        else:
            # Should never reach here due to the check above
            raise ValueError(f"Version {version} implementation missing for multi-holder structure")
    
    def _decode_multi_holder_v1(self, data: bytes) -> List[bytes]:
        """Decode version 1 of multi-holder structure."""
        # Minimum size check: header (2) + sub-header 1 (1)
        if len(data) < 3:
            raise ValueError("Data too short for multi-holder structure")
        
        # Extract number of structures
        num_structures = struct.unpack("!B", data[2:3])[0]
        
        # Validate size of sub-header 2
        sub_header_2_size = num_structures * 8  # 8 bytes per entry
        if len(data) < 3 + sub_header_2_size:
            raise ValueError(f"Data too short for {num_structures} structure entries")
        
        # Extract structures
        structures = []
        for i in range(num_structures):
            # Extract entry from sub-header 2
            offset = 3 + (i * 8)
            start_idx, length = struct.unpack("!II", data[offset:offset+8])
            
            # Validate range
            if start_idx + length > len(data):
                raise ValueError(f"Structure {i} range out of bounds")
            
            # Extract structure data
            structure_data = data[start_idx:start_idx+length]
            structures.append(structure_data)
        
        return structures
    
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
        # Validate header and get version
        structure_id, version = self.decode_header(data)
        if structure_id != ByteStructureID.NEURON_FLAT:
            raise ValueError(f"Expected neuron flat structure (ID: 10), got {structure_id}")
        
        # Check if version is supported
        if version not in SUPPORTED_VERSIONS[ByteStructureID.NEURON_FLAT]:
            raise ValueError(f"Unsupported version {version} for neuron flat structure")
        
        # Dispatch to version-specific decoder
        if version == 1:
            result = self._decode_neuron_flat_v1(data)
            result['version'] = 1  # Add version to result
            return result
        else:
            # Should never reach here due to the check above
            raise ValueError(f"Version {version} implementation missing for neuron flat structure")
    
    def _decode_neuron_flat_v1(self, data: bytes) -> Dict[str, Any]:
        """Decode version 1 of neuron flat format."""
        # Calculate neuron count
        # Size = 2 (header) + N*(6+4+4+4+4) where N is neuron count
        data_size = len(data) - 2  # Subtract header size
        if data_size % 22 != 0:
            raise ValueError("Data size not aligned with neuron structure")
        
        neuron_count = data_size // 22
        
        # Extract cortical IDs (6 bytes each)
        cortical_ids_end = 2 + (6 * neuron_count)
        if len(data) < cortical_ids_end:
            raise ValueError("Data too short for cortical IDs")
        
        cortical_ids = []
        for i in range(neuron_count):
            offset = 2 + (i * 6)
            cortical_id = data[offset:offset+6].decode('ascii')
            cortical_ids.append(cortical_id)
        
        # Extract X coordinates (4 bytes each)
        x_end = cortical_ids_end + (4 * neuron_count)
        if len(data) < x_end:
            raise ValueError("Data too short for X coordinates")
        
        x_format = f"!{neuron_count}i"
        x_coords = list(struct.unpack(x_format, data[cortical_ids_end:x_end]))
        
        # Extract Y coordinates (4 bytes each)
        y_end = x_end + (4 * neuron_count)
        if len(data) < y_end:
            raise ValueError("Data too short for Y coordinates")
        
        y_format = f"!{neuron_count}i"
        y_coords = list(struct.unpack(y_format, data[x_end:y_end]))
        
        # Extract Z coordinates (4 bytes each)
        z_end = y_end + (4 * neuron_count)
        if len(data) < z_end:
            raise ValueError("Data too short for Z coordinates")
        
        z_format = f"!{neuron_count}i"
        z_coords = list(struct.unpack(z_format, data[y_end:z_end]))
        
        # Extract potentials (4 bytes each)
        potentials_end = z_end + (4 * neuron_count)
        if len(data) < potentials_end:
            raise ValueError("Data too short for potentials")
        
        potentials_format = f"!{neuron_count}f"
        potentials = list(struct.unpack(potentials_format, data[z_end:potentials_end]))
        
        return {
            'cortical_ids': cortical_ids,
            'x': x_coords,
            'y': y_coords,
            'z': z_coords,
            'potentials': potentials
        }
    
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
        # Validate header and get version
        structure_id, version = self.decode_header(data)
        if structure_id != ByteStructureID.NEURON_CATEGORIES:
            raise ValueError(f"Expected neuron categories structure (ID: 11), got {structure_id}")
        
        # Check if version is supported
        if version not in SUPPORTED_VERSIONS[ByteStructureID.NEURON_CATEGORIES]:
            raise ValueError(f"Unsupported version {version} for neuron categories structure")
        
        # Dispatch to version-specific decoder
        if version == 1:
            result = self._decode_neuron_categories_v1(data)
            result['version'] = 1  # Add version to result
            return result
        else:
            # Should never reach here due to the check above
            raise ValueError(f"Version {version} implementation missing for neuron categories structure")
    
    def _decode_neuron_categories_v1(self, data: bytes) -> Dict[str, Dict[str, Union[List[int], List[float]]]]:
        """Decode version 1 of neuron categories structure."""
        # Minimum size check: header (2) + initial header (2)
        if len(data) < 4:
            raise ValueError("Data too short for neuron categories structure")
        
        # Extract cortical area count
        cortical_count = struct.unpack("!H", data[2:4])[0]
        
        # Validate secondary header size
        sec_headers_size = cortical_count * 14  # 6 bytes for ID + 8 bytes for index/count
        if len(data) < 4 + sec_headers_size:
            raise ValueError("Data too short for secondary headers")
        
        # Process each cortical area
        result = {}
        for i in range(cortical_count):
            # Extract secondary header
            offset = 4 + (i * 14)
            cortical_id = data[offset:offset+6].decode('ascii')
            start_idx, neuron_count = struct.unpack("!II", data[offset+6:offset+14])
            
            # Validate range
            data_size = neuron_count * 16  # 4 bytes each for x, y, z, potential
            if start_idx + data_size > len(data):
                raise ValueError(f"Cortical area {cortical_id} data range out of bounds")
            
            # Extract coordinates and potentials
            x_end = start_idx + (neuron_count * 4)
            y_end = x_end + (neuron_count * 4)
            z_end = y_end + (neuron_count * 4)
            p_end = z_end + (neuron_count * 4)
            
            x_format = f"!{neuron_count}I"
            y_format = f"!{neuron_count}I"
            z_format = f"!{neuron_count}I"
            p_format = f"!{neuron_count}f"
            
            x_coords = list(struct.unpack(x_format, data[start_idx:x_end]))
            y_coords = list(struct.unpack(y_format, data[x_end:y_end]))
            z_coords = list(struct.unpack(z_format, data[y_end:z_end]))
            potentials = list(struct.unpack(p_format, data[z_end:p_end]))
            
            # Store in result dictionary
            result[cortical_id] = {
                'x': x_coords,
                'y': y_coords,
                'z': z_coords,
                'potentials': potentials
            }
        
        return result
    
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
        return zlib.decompress(data) 