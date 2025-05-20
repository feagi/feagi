"""
FEAGI Byte Processing Utilities - Python Implementation

This module provides Python implementations of functions for processing FEAGI byte structures.
"""

import struct
from typing import List, Dict, Tuple, Any, Optional, Union, Set

# Constants for byte structure types
MULTI_STRUCT_HOLDER = 0 
NEURON_POTENTIAL_CATEGORICAL_XYZ = 11


def infer_byte_structure_type_python(bytes_data: bytes) -> int:
    """
    Infer the type of FEAGI byte structure from raw bytes (Python implementation).
    
    Args:
        bytes_data: Raw bytes data
        
    Returns:
        Integer representing the byte structure type
        
    Raises:
        ValueError: If the byte structure type cannot be inferred
    """
    if not bytes_data:
        raise ValueError("Empty byte array")
    
    byte_type = bytes_data[0]
    if byte_type == MULTI_STRUCT_HOLDER:
        return MULTI_STRUCT_HOLDER
    elif byte_type == NEURON_POTENTIAL_CATEGORICAL_XYZ:
        return NEURON_POTENTIAL_CATEGORICAL_XYZ
    else:
        raise ValueError(f"Unknown byte structure type: {byte_type}")


def extract_sub_structures_python(bytes_data: bytes) -> List[bytes]:
    """
    Extract sub-structures from a multi-structure holder (Python implementation).
    
    Args:
        bytes_data: Raw bytes data of a multi-structure holder
        
    Returns:
        List of byte arrays representing the sub-structures
        
    Raises:
        ValueError: If the bytes data is not a valid multi-structure holder
    """
    if not bytes_data:
        raise ValueError("Empty byte array")
    
    # Basic validation
    if bytes_data[0] != MULTI_STRUCT_HOLDER:
        raise ValueError("Not a multi-structure holder")
    
    # Extract number of contained structures (using little-endian format)
    # Byte 0: structure type
    # Byte 1: version
    # Byte 2-5: number of contained structures (uint32)
    num_structures = struct.unpack("<I", bytes_data[2:6])[0]
    
    # Parse the sub-structures
    result = []
    offset = 6  # Start after the header
    
    for i in range(num_structures):
        if offset + 4 > len(bytes_data):
            raise ValueError("Invalid multi-structure format: unexpected end of data")
        
        # Get size of the next structure (uint32)
        struct_size = struct.unpack("<I", bytes_data[offset:offset+4])[0]
        offset += 4
        
        if offset + struct_size > len(bytes_data):
            raise ValueError(f"Invalid multi-structure format: structure {i} exceeds data bounds")
        
        # Extract the structure
        structure = bytes_data[offset:offset+struct_size]
        result.append(structure)
        offset += struct_size
    
    return result


def decode_neuron_potential_xyz_python(bytes_data: bytes) -> Dict[Tuple[int, int, int], float]:
    """
    Decode neuron potential data from XYZ categorical format (Python implementation).
    
    Args:
        bytes_data: Raw bytes in NEURON_POTENTIAL_CATEGORICAL_XYZ format
        
    Returns:
        Dictionary mapping (x, y, z) coordinates to activation values (0.0-1.0)
        
    Raises:
        ValueError: If the bytes data is not in the expected format
    """
    if not bytes_data:
        raise ValueError("Empty byte array")
    
    # Basic validation
    if bytes_data[0] != NEURON_POTENTIAL_CATEGORICAL_XYZ:
        raise ValueError("Not a neuron potential XYZ structure")
    
    # Extract relevant data
    # Byte 0: structure type
    # Byte 1: version
    # Byte 2: min potential value (uint8)
    # Byte 3: max potential value (uint8)
    # Byte 4-7: number of neurons (uint32)
    
    min_val = bytes_data[2] / 255.0
    max_val = bytes_data[3] / 255.0
    num_neurons = struct.unpack("<I", bytes_data[4:8])[0]
    
    # Each neuron entry is represented by:
    # - x, y, z coordinates (3 * uint16 = 6 bytes)
    # - potential category (uint8 = 1 byte)
    entry_size = 7
    result = {}
    
    for i in range(num_neurons):
        offset = 8 + (i * entry_size)
        if offset + entry_size > len(bytes_data):
            raise ValueError(f"Invalid neuron potential format: neuron {i} exceeds data bounds")
        
        # Extract coordinates and potential
        x, y, z = struct.unpack("<HHH", bytes_data[offset:offset+6])
        potential_category = bytes_data[offset+6]
        
        # Map category to actual potential value
        if max_val == min_val:
            potential = max_val
        else:
            potential = min_val + (potential_category / 255.0) * (max_val - min_val)
        
        result[(x, y, z)] = potential
    
    return result


def encode_neuron_potential_xyz_python(neuron_data: Dict[Tuple[int, int, int], float]) -> bytes:
    """
    Encode neuron potential data into XYZ categorical format (Python implementation).
    
    Args:
        neuron_data: Dictionary mapping (x, y, z) coordinates to activation values (0.0-1.0)
        
    Returns:
        Bytes in NEURON_POTENTIAL_CATEGORICAL_XYZ format
        
    Raises:
        ValueError: If the input data is invalid
    """
    if not neuron_data:
        # Empty data, create minimal valid structure
        return bytes([NEURON_POTENTIAL_CATEGORICAL_XYZ, 1, 0, 0]) + struct.pack("<I", 0)
    
    # Find min and max values
    values = list(neuron_data.values())
    min_val = max(0.0, min(values))
    max_val = min(1.0, max(values))
    
    # Create header
    # Byte 0: structure type
    # Byte 1: version (use 1)
    # Byte 2: min potential value (uint8)
    # Byte 3: max potential value (uint8)
    # Byte 4-7: number of neurons (uint32)
    header = bytes([
        NEURON_POTENTIAL_CATEGORICAL_XYZ,
        1,
        int(min_val * 255),
        int(max_val * 255)
    ]) + struct.pack("<I", len(neuron_data))
    
    # Build neuron entries
    entries = bytearray()
    for (x, y, z), potential in neuron_data.items():
        # Validate coordinates
        if not (0 <= x <= 65535 and 0 <= y <= 65535 and 0 <= z <= 65535):
            raise ValueError(f"Coordinate out of range: ({x}, {y}, {z})")
        
        # Normalize potential to category
        if max_val == min_val:
            category = 255
        else:
            category = int(((potential - min_val) / (max_val - min_val)) * 255)
        category = max(0, min(255, category))
        
        # Add entry
        entries.extend(struct.pack("<HHH", x, y, z))
        entries.append(category)
    
    return header + entries 