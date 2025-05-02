"""
Serialization utilities for FEAGI API.

This module provides serialization and deserialization functions for
different data formats used in the API.
"""

import json
import struct
import zlib
import logging
import numpy as np
from typing import Any, Dict, List, Union, Optional, Tuple

logger = logging.getLogger(__name__)

def serialize_data(
    data: Any,
    format_type: str = "json",
    compression_level: int = 0
) -> bytes:
    """
    Serialize data to binary format.
    
    Args:
        data: Data to serialize.
        format_type: Serialization format ("json", "binary", "numpy").
        compression_level: Compression level (0-9, 0 = no compression).
        
    Returns:
        Serialized data as bytes.
    """
    try:
        # Serialize based on format type
        if format_type == "json":
            binary_data = json.dumps(data).encode()
        elif format_type == "binary":
            if isinstance(data, bytes):
                binary_data = data
            else:
                raise TypeError(f"Expected bytes for binary format, got {type(data)}")
        elif format_type == "numpy":
            if isinstance(data, np.ndarray):
                # Encode numpy array with shape and dtype
                shape = data.shape
                dtype = str(data.dtype)
                
                # Create header with shape and dtype
                header = struct.pack(">B", len(shape))  # Number of dimensions
                for dim in shape:
                    header += struct.pack(">I", dim)  # Size of each dimension
                
                # Add dtype length and dtype string
                dtype_bytes = dtype.encode()
                header += struct.pack(">B", len(dtype_bytes))
                header += dtype_bytes
                
                # Flatten array and convert to bytes
                data_bytes = data.tobytes()
                
                binary_data = header + data_bytes
            else:
                raise TypeError(f"Expected numpy array for numpy format, got {type(data)}")
        else:
            raise ValueError(f"Unsupported format type: {format_type}")
            
        # Compress if needed
        if compression_level > 0:
            binary_data = zlib.compress(binary_data, compression_level)
            
        # Add format type and compression level to header
        format_byte = {
            "json": 0,
            "binary": 1,
            "numpy": 2
        }.get(format_type, 0)
        
        header = struct.pack(">BB", format_byte, compression_level)
        
        return header + binary_data
    except Exception as e:
        logger.exception(f"Error serializing data: {e}")
        # Fallback to simple JSON serialization
        return json.dumps({"error": str(e)}).encode()

def deserialize_data(binary_data: bytes) -> Tuple[Any, str]:
    """
    Deserialize binary data.
    
    Args:
        binary_data: Binary data to deserialize.
        
    Returns:
        Tuple of (deserialized data, format type).
    """
    try:
        # Extract header
        if len(binary_data) < 2:
            raise ValueError("Invalid binary data: too short")
            
        format_byte, compression_level = struct.unpack(">BB", binary_data[:2])
        data = binary_data[2:]
        
        # Map format byte to format type
        format_type = {
            0: "json",
            1: "binary",
            2: "numpy"
        }.get(format_byte, "json")
        
        # Decompress if needed
        if compression_level > 0:
            data = zlib.decompress(data)
            
        # Deserialize based on format type
        if format_type == "json":
            return json.loads(data.decode()), format_type
        elif format_type == "binary":
            return data, format_type
        elif format_type == "numpy":
            # Parse numpy header
            offset = 0
            dims = struct.unpack(">B", data[offset:offset+1])[0]
            offset += 1
            
            shape = []
            for i in range(dims):
                shape.append(struct.unpack(">I", data[offset:offset+4])[0])
                offset += 4
                
            dtype_len = struct.unpack(">B", data[offset:offset+1])[0]
            offset += 1
            
            dtype = data[offset:offset+dtype_len].decode()
            offset += dtype_len
            
            # Reconstruct numpy array
            array_data = data[offset:]
            return np.frombuffer(array_data, dtype=dtype).reshape(shape), format_type
        else:
            raise ValueError(f"Unsupported format type: {format_type}")
    except Exception as e:
        logger.exception(f"Error deserializing data: {e}")
        return None, "error" 