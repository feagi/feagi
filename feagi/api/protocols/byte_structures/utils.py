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
Utility functions for FEAGI byte structures.

This module provides helper functions for working with FEAGI byte structures.
"""

import struct
import zlib
from typing import Dict, Any, Tuple, List, Optional

import numpy as np

from feagi.api.protocols.constants import ByteStructureID


def validate_cortical_id(cortical_id: str) -> str:
    """
    Validate and normalize a cortical ID.
    
    Args:
        cortical_id: Cortical area ID
        
    Returns:
        Normalized cortical ID (exactly 6 characters)
    """
    if not cortical_id:
        raise ValueError("Cortical ID cannot be empty")
    
    # Ensure exactly 6 characters
    if len(cortical_id) > 6:
        # Truncate if too long
        return cortical_id[:6]
    elif len(cortical_id) < 6:
        # Pad with spaces if too short
        return cortical_id.ljust(6)
    else:
        return cortical_id


def validate_neuron_data(cortical_data: Dict[str, Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    """
    Validate neuron data for consistency and proper formatting.
    
    Args:
        cortical_data: Dictionary of cortical areas with neuron data
        
    Returns:
        Validated and normalized cortical data
        
    Raises:
        ValueError: If data is inconsistent or invalid
    """
    result = {}
    
    for cortical_id, data in cortical_data.items():
        # Normalize cortical ID
        normalized_id = validate_cortical_id(cortical_id)
        
        # Check required fields
        required_fields = ['x', 'y', 'z', 'potentials']
        for field in required_fields:
            if field not in data:
                raise ValueError(f"Missing required field '{field}' for cortical area {cortical_id}")
        
        # Get data arrays
        x_coords = data['x']
        y_coords = data['y']
        z_coords = data['z']
        potentials = data['potentials']
        
        # Check array lengths match
        neuron_count = len(x_coords)
        if (len(y_coords) != neuron_count or 
            len(z_coords) != neuron_count or 
            len(potentials) != neuron_count):
            raise ValueError(f"Inconsistent array lengths for cortical area {cortical_id}")
        
        # Convert lists to numpy arrays for efficiency if they aren't already
        if not isinstance(x_coords, np.ndarray):
            x_coords = np.array(x_coords, dtype=np.int32)
        if not isinstance(y_coords, np.ndarray):
            y_coords = np.array(y_coords, dtype=np.int32)
        if not isinstance(z_coords, np.ndarray):
            z_coords = np.array(z_coords, dtype=np.int32)
        if not isinstance(potentials, np.ndarray):
            potentials = np.array(potentials, dtype=np.float32)
        
        # Store normalized data
        result[normalized_id] = {
            'x': x_coords,
            'y': y_coords,
            'z': z_coords,
            'potentials': potentials
        }
    
    return result


def convert_rgb_to_bgr(image: np.ndarray) -> np.ndarray:
    """
    Convert RGB image to BGR format expected by the byte structure.
    
    Args:
        image: RGB image as numpy array (height, width, 3)
        
    Returns:
        BGR image as numpy array
    """
    if image.shape[2] != 3:
        raise ValueError("Image must have 3 color channels")
    
    # Swap R and B channels
    bgr_image = image.copy()
    bgr_image[:, :, 0] = image[:, :, 2]  # R → B
    bgr_image[:, :, 2] = image[:, :, 0]  # B → R
    
    return bgr_image


def compress_data(data: bytes, level: int = 6) -> bytes:
    """
    Compress data using zlib with specified compression level.
    
    Args:
        data: Data to compress
        level: Compression level (0-9, 9 is highest compression)
        
    Returns:
        Compressed data
    """
    return zlib.compress(data, level=level)


def decompress_data(data: bytes) -> bytes:
    """
    Decompress zlib-compressed data.
    
    Args:
        data: Compressed data
        
    Returns:
        Decompressed data
        
    Raises:
        zlib.error: If decompression fails
    """
    return zlib.decompress(data)


def is_compressed(data: bytes) -> bool:
    """
    Check if data is likely zlib-compressed.
    
    Args:
        data: Data to check
        
    Returns:
        True if data appears to be zlib-compressed, False otherwise
    """
    # Check for zlib header bytes (78 01, 78 9C, or 78 DA are common)
    if len(data) < 2:
        return False
    
    # Check for common zlib magic numbers
    return (data[0] == 0x78 and 
            (data[1] == 0x01 or data[1] == 0x9C or data[1] == 0xDA))


def get_structure_info(data: bytes) -> Tuple[int, int]:
    """
    Extract structure type and version from a byte structure.
    
    Args:
        data: Byte structure data
        
    Returns:
        Tuple of (structure_type, version)
        
    Raises:
        ValueError: If data is too short for a header
    """
    if len(data) < 2:
        raise ValueError("Data too short for byte structure header")
    
    structure_type, version = struct.unpack("!BB", data[:2])
    return structure_type, version


def check_version_compatibility(server_version: int, client_version: int) -> bool:
    """
    Check if a client version is compatible with a server version.
    
    Args:
        server_version: Server structure version
        client_version: Client structure version
        
    Returns:
        True if compatible, False otherwise
    """
    # In our current implementation, version compatibility is simple:
    # Versions must match exactly
    return server_version == client_version
    
    # In a more complex implementation, you might have rules like:
    # - Version 2 is backward compatible with version 1
    # - Version 3 is incompatible with version 1
    # This would require a compatibility matrix or rules 