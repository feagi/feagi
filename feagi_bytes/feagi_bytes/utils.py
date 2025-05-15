"""
Utility functions for FEAGI byte structures.

This module provides helper functions for working with FEAGI byte structures.
"""

import struct
import numpy as np
from typing import Tuple, Dict, Any, List, Union


def get_structure_info(data: bytes) -> Tuple[int, int]:
    """
    Extract structure ID and version from byte structure header.
    
    Args:
        data: Byte data with header
        
    Returns:
        Tuple of (structure_id, version)
        
    Raises:
        ValueError: If data is too short
    """
    if len(data) < 2:
        raise ValueError("Data too short for header")
    
    structure_id, version = struct.unpack("!BB", data[:2])
    return structure_id, version


def is_compressed(data: bytes) -> bool:
    """
    Check if data is compressed.
    
    Args:
        data: Byte data to check
        
    Returns:
        True if data is compressed, False otherwise
        
    Raises:
        ValueError: If data is empty
    """
    if not data:
        raise ValueError("Empty data")
    
    return data[0] == 1


def validate_cortical_id(cortical_id: str) -> str:
    """
    Validate and normalize a cortical ID.
    
    Args:
        cortical_id: Cortical ID to validate
        
    Returns:
        Normalized cortical ID (exactly 6 chars)
        
    Raises:
        ValueError: If cortical ID is too long
    """
    if len(cortical_id) > 6:
        raise ValueError(f"Cortical ID '{cortical_id}' exceeds maximum length of 6 characters")
    
    # Pad to 6 chars if needed
    return cortical_id.ljust(6)[:6]


def convert_raw_to_neuron_data(data: Union[bytes, List[float]], 
                              data_type: str = "image",
                              dimensions: Tuple[int, int] = None,
                              channels: int = 1,
                              cortical_area_id: str = "SENSOR") -> Dict[str, Dict[str, Any]]:
    """
    Convert raw sensory data to FEAGI neuron data format.
    
    Args:
        data: Raw sensory data as bytes or list of floats
        data_type: Type of data ("image" or "array")
        dimensions: Image dimensions as (width, height) or array dimensions
        channels: Number of image channels
        cortical_area_id: Target cortical area ID
        
    Returns:
        Dictionary with cortical neuron data in FEAGI's internal format:
        {
            'cortical_id': {
                'x': [x1, x2, ...],
                'y': [y1, y2, ...],
                'z': [z1, z2, ...],
                'potentials': [p1, p2, ...],
            },
        }
    """
    cortical_id = validate_cortical_id(cortical_area_id)
    result = {
        cortical_id: {
            'x': [],
            'y': [],
            'z': [],
            'potentials': []
        }
    }
    
    # For array data (direct neuron potentials)
    if data_type == "array":
        # Convert to numpy array for processing
        if isinstance(data, bytes):
            values = np.frombuffer(data, dtype=np.float32)
        else:
            values = np.array(data, dtype=np.float32)
        
        # For a 1D array, create a square-ish 2D grid
        if dimensions is None:
            size = len(values)
            width = int(np.sqrt(size))
            height = (size + width - 1) // width  # Ceiling division
        else:
            width, height = dimensions
            
        # Create neuron coordinates and potentials
        for i in range(len(values)):
            if i >= width * height:
                break
                
            x = i % width
            y = i // width
            z = 0  # Using z=0 for all neurons in this simple example
            
            # Add to the result
            result[cortical_id]['x'].append(x)
            result[cortical_id]['y'].append(y)
            result[cortical_id]['z'].append(z)
            result[cortical_id]['potentials'].append(float(values[i]))
    
    # For image data
    elif data_type == "image":
        # Convert bytes to numpy array
        if isinstance(data, bytes):
            if dimensions is None:
                raise ValueError("Must provide dimensions for image data")
                
            width, height = dimensions
            if channels == 1:
                values = np.frombuffer(data, dtype=np.uint8).reshape(height, width)
            else:
                values = np.frombuffer(data, dtype=np.uint8).reshape(height, width, channels)
        else:
            # Already a list, convert to numpy array
            values = np.array(data, dtype=np.uint8)
            
        # Process each pixel
        for y in range(height):
            for x in range(width):
                # For RGB, average the channels or process separately
                if channels > 1:
                    if isinstance(values[y, x], np.ndarray):
                        # RGB image, use average intensity
                        intensity = float(np.mean(values[y, x]) / 255.0)
                    else:
                        intensity = float(values[y, x] / 255.0)
                else:
                    intensity = float(values[y, x] / 255.0)
                
                # Only add neurons with non-zero potential
                if intensity > 0.01:  # Small threshold to reduce data size
                    result[cortical_id]['x'].append(x)
                    result[cortical_id]['y'].append(y)
                    result[cortical_id]['z'].append(0)  # Using z=0 for all neurons in this example
                    result[cortical_id]['potentials'].append(intensity)
    
    else:
        raise ValueError(f"Unsupported data type: {data_type}")
        
    return result 