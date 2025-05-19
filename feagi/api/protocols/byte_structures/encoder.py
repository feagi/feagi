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
FEAGI Byte Structures Encoder

This module provides encoders for various FEAGI byte structures, following the
specifications defined in the FEAGI byte structures documentation.

VERSIONING GUIDELINES:

When adding a new version of a structure:
1. Create a new private method _encode_X_vN (e.g., _encode_json_v2)
2. Update the public method to dispatch to the appropriate version
3. Document the format changes in both the public method docstring and README
4. Add tests for the new version
5. Update the protocol translator to handle version negotiation

Example for adding version 2 of neuron_flat:
- Add _encode_neuron_flat_v2 method with new parameters
- Update encode_neuron_flat to dispatch to v1 or v2 based on version parameter
- Add tests for version 2 format
- Update translator to select appropriate version based on client capabilities
"""

import json
import struct
import zlib
from typing import List, Dict, Any, Union, Optional, Tuple

import numpy as np

from feagi.api.protocols.constants import ByteStructureID
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


# Registry of supported versions for each structure type
SUPPORTED_VERSIONS = {
    ByteStructureID.JSON: [1],
    ByteStructureID.RAW_IMAGE: [1],
    ByteStructureID.MULTI_HOLDER: [1],
    ByteStructureID.NEURON_FLAT: [1], 
    ByteStructureID.NEURON_CATEGORIES: [1],
}


class ByteStructureEncoder:
    """
    Encoder for FEAGI byte structures.
    
    This class provides methods to encode various data types into FEAGI
    byte structures as defined in the documentation.
    """
    
    def __init__(self):
        """Initialize the byte structure encoder."""
        self.logger = logger
    
    @staticmethod
    def encode_header(structure_id: int, version: int = 1) -> bytes:
        """
        Encode the universal FEAGI byte structure header.
        
        Args:
            structure_id: Byte structure ID (see ByteStructureID enum)
            version: Structure version (default: 1)
        
        Returns:
            Encoded header bytes
        """
        return struct.pack("!BB", structure_id, version)
    
    def encode_json(self, data: Dict[str, Any]) -> bytes:
        """
        Encode data as JSON.
        
        Args:
            data: Dictionary to encode
            
        Returns:
            JSON encoded as bytes
        """
        try:
            # Convert data to JSON string
            json_str = json.dumps(data)
            
            # Add structure ID and version (1 for JSON)
            header = struct.pack("!BB", 1, 1)  # ID=1 (JSON), Version=1
            
            # Encode as UTF-8 and add header
            return header + json_str.encode("utf-8")
        except Exception as e:
            self.logger.error(f"Error encoding JSON: {e}")
            # Return minimal valid structure on error
            return struct.pack("!BB", 1, 1) + b"{}"
    
    def encode_raw_image(self, image: np.ndarray, version: int = 1) -> bytes:
        """
        Encode a raw image as byte structure (ID: 8).
        
        Args:
            image: Image data as numpy array (height, width, 3)
                  Expected format is BGR (uint8)
            version: Structure version to use (default: 1)
                  
        Returns:
            Encoded byte structure
            
        Raises:
            ValueError: If the specified version is not supported
        """
        if version not in SUPPORTED_VERSIONS[ByteStructureID.RAW_IMAGE]:
            raise ValueError(f"Unsupported version {version} for raw image structure")
            
        if version == 1:
            return self._encode_raw_image_v1(image)
        else:
            # Should never reach here due to the check above
            raise ValueError(f"Version {version} implementation missing for raw image structure")
    
    def _encode_raw_image_v1(self, image: np.ndarray) -> bytes:
        """Version 1 implementation of raw image structure."""
        # Validate image dimensions
        if len(image.shape) != 3 or image.shape[2] != 3:
            raise ValueError("Image must be a 3D array with shape (height, width, 3)")
        
        if image.dtype != np.uint8:
            raise ValueError("Image must be of type uint8")
        
        # Get dimensions
        height, width = image.shape[:2]
        
        # Create header
        header = self.encode_header(ByteStructureID.RAW_IMAGE, version=1)
        
        # Create sub-header with dimensions
        sub_header = struct.pack("!HH", width, height)
        
        # Image data is already in the right format (BGR bytes)
        # Just flatten it to get raw bytes
        image_bytes = image.tobytes()
        
        return header + sub_header + image_bytes
    
    def encode_multi_holder(self, byte_structures: List[bytes], version: int = 1) -> bytes:
        """
        Encode multiple byte structures into a single container (ID: 9).
        
        Args:
            byte_structures: List of encoded byte structures to include
            version: Structure version to use (default: 1)
                           
        Returns:
            Encoded multi-holder byte structure
            
        Raises:
            ValueError: If the specified version is not supported
        """
        if version not in SUPPORTED_VERSIONS[ByteStructureID.MULTI_HOLDER]:
            raise ValueError(f"Unsupported version {version} for multi-holder structure")
            
        if version == 1:
            return self._encode_multi_holder_v1(byte_structures)
        else:
            # Should never reach here due to the check above
            raise ValueError(f"Version {version} implementation missing for multi-holder structure")
    
    def _encode_multi_holder_v1(self, byte_structures: List[bytes]) -> bytes:
        """Version 1 implementation of multi-holder structure."""
        # Calculate number of structures
        num_structures = len(byte_structures)
        if num_structures > 255:
            raise ValueError("Too many structures (maximum 255 allowed)")
        
        # Create header
        header = self.encode_header(ByteStructureID.MULTI_HOLDER, version=1)
        
        # Create sub-header with count
        sub_header_1 = struct.pack("!B", num_structures)
        
        # Calculate starting indices and sizes
        sub_header_2 = bytearray()
        data_section = bytearray()
        
        # Track current position (after all headers)
        # Start with size of global header + sub-header 1
        # + size of sub-header 2 (which is 8 bytes per structure)
        current_pos = 2 + 1 + (8 * num_structures)
        
        # Build sub-header 2 and data section
        for struct_data in byte_structures:
            # Add entry to sub-header
            sub_header_2.extend(struct.pack("!II", current_pos, len(struct_data)))
            
            # Add data to data section
            data_section.extend(struct_data)
            
            # Update position for next structure
            current_pos += len(struct_data)
        
        # Combine all parts
        return header + sub_header_1 + sub_header_2 + data_section
    
    def encode_neuron_flat(
        self,
        cortical_ids: List[str],
        x_coords: List[int],
        y_coords: List[int],
        z_coords: List[int],
        potentials: List[float],
        version: int = 1
    ) -> bytes:
        """
        Encode neuron potential data in flat format (ID: 10).
        
        Args:
            cortical_ids: List of cortical area IDs (6 chars each)
            x_coords: X coordinates for neurons
            y_coords: Y coordinates for neurons
            z_coords: Z coordinates for neurons
            potentials: Neuron potentials (float values)
            version: Structure version to use (default: 1)
            
        Returns:
            Encoded byte structure
            
        Raises:
            ValueError: If the specified version is not supported
        """
        if version not in SUPPORTED_VERSIONS[ByteStructureID.NEURON_FLAT]:
            raise ValueError(f"Unsupported version {version} for neuron flat structure")
            
        if version == 1:
            return self._encode_neuron_flat_v1(cortical_ids, x_coords, y_coords, z_coords, potentials)
        else:
            # Should never reach here due to the check above
            raise ValueError(f"Version {version} implementation missing for neuron flat structure")
    
    def _encode_neuron_flat_v1(
        self,
        cortical_ids: List[str],
        x_coords: List[int],
        y_coords: List[int],
        z_coords: List[int],
        potentials: List[float]
    ) -> bytes:
        """Version 1 implementation of neuron flat format."""
        # Validate input data
        neuron_count = len(x_coords)
        if (len(cortical_ids) != neuron_count or 
            len(y_coords) != neuron_count or
            len(z_coords) != neuron_count or
            len(potentials) != neuron_count):
            raise ValueError("All input lists must have the same length")
        
        # Create header
        header = self.encode_header(ByteStructureID.NEURON_FLAT, version=1)
        
        # Encode cortical IDs (each 6 ASCII chars)
        cortical_id_bytes = bytearray()
        for cort_id in cortical_ids:
            # Ensure each cortical ID is exactly 6 chars
            if len(cort_id) != 6:
                cort_id = cort_id.ljust(6)[:6]  # Pad or truncate to 6 chars
            cortical_id_bytes.extend(cort_id.encode('ascii'))
        
        # Encode coordinates
        x_bytes = struct.pack(f"!{neuron_count}i", *x_coords)  # 4 bytes per int32
        y_bytes = struct.pack(f"!{neuron_count}i", *y_coords)  # 4 bytes per int32
        z_bytes = struct.pack(f"!{neuron_count}i", *z_coords)  # 4 bytes per int32
        
        # Encode potentials
        potential_bytes = struct.pack(f"!{neuron_count}f", *potentials)  # 4 bytes per float
        
        # Combine all sections in correct order
        return header + cortical_id_bytes + x_bytes + y_bytes + z_bytes + potential_bytes
    
    def encode_neuron_categories(
        self,
        cortical_data: Dict[str, Dict[str, Union[List[int], List[float]]]],
        version: int = 1
    ) -> bytes:
        """
        Encode neuron potential data organized by cortical areas (ID: 11).
        
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
            version: Structure version to use (default: 1)
            
        Returns:
            Encoded byte structure
            
        Raises:
            ValueError: If the specified version is not supported
        """
        if version not in SUPPORTED_VERSIONS[ByteStructureID.NEURON_CATEGORIES]:
            raise ValueError(f"Unsupported version {version} for neuron categories structure")
            
        if version == 1:
            return self._encode_neuron_categories_v1(cortical_data)
        else:
            # Should never reach here due to the check above
            raise ValueError(f"Version {version} implementation missing for neuron categories structure")
    
    def _encode_neuron_categories_v1(
        self,
        cortical_data: Dict[str, Dict[str, Union[List[int], List[float]]]]
    ) -> bytes:
        """Version 1 implementation of neuron categories structure."""
        # Create header
        header = self.encode_header(ByteStructureID.NEURON_CATEGORIES, version=1)
        
        # Count cortical areas
        cortical_count = len(cortical_data)
        if cortical_count > 65535:  # Max value for uint16
            raise ValueError("Too many cortical areas (maximum 65535 allowed)")
        
        # Encode initial section header
        init_header = struct.pack("!H", cortical_count)
        
        # Prepare placeholders
        secondary_headers = bytearray()
        neuron_data = bytearray()
        
        # Current reading start index (relative to entire struct)
        # Starts after global header (2 bytes) + initial header (2 bytes)
        # + secondary headers (14 bytes per cortical area)
        current_index = 2 + 2 + (14 * cortical_count)
        
        # Process each cortical area
        for cortical_id, data in cortical_data.items():
            # Validate cortical ID
            if len(cortical_id) != 6:
                cortical_id = cortical_id.ljust(6)[:6]  # Pad or truncate to 6 chars
            
            # Get coordinates and potentials
            x_coords = data['x']
            y_coords = data['y']
            z_coords = data['z']
            potentials = data['potentials']
            
            # Validate neuron count
            neuron_count = len(x_coords)
            if (len(y_coords) != neuron_count or 
                len(z_coords) != neuron_count or
                len(potentials) != neuron_count):
                raise ValueError(f"Inconsistent neuron count for cortical area {cortical_id}")
            
            # Encode secondary header
            cortical_id_bytes = cortical_id.encode('ascii')
            secondary_headers.extend(cortical_id_bytes)  # 6 bytes
            secondary_headers.extend(struct.pack("!II", current_index, neuron_count))  # 8 bytes
            
            # Encode neuron data
            area_data = bytearray()
            area_data.extend(struct.pack(f"!{neuron_count}I", *x_coords))  # 4 bytes per uint32
            area_data.extend(struct.pack(f"!{neuron_count}I", *y_coords))  # 4 bytes per uint32
            area_data.extend(struct.pack(f"!{neuron_count}I", *z_coords))  # 4 bytes per uint32
            area_data.extend(struct.pack(f"!{neuron_count}f", *potentials))  # 4 bytes per float
            
            # Add to neuron data section
            neuron_data.extend(area_data)
            
            # Update index for next area
            current_index += len(area_data)
        
        # Combine all parts
        return header + init_header + secondary_headers + neuron_data
    
    def compress(self, data: bytes) -> bytes:
        """
        Compress data using Deflate algorithm.
        
        Args:
            data: Data bytes to compress
            
        Returns:
            Compressed bytes
        """
        if len(data) < 100:
            # Don't compress very small data
            return data
            
        try:
            return zlib.compress(data, level=6)
        except Exception as e:
            self.logger.error(f"Error compressing data: {e}")
            return data
    
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