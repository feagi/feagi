"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
FEAGI Byte Processing Utilities - Rust Implementation

This module provides direct wrappers to the Rust implementations in feagi-rust-py-libs.
These functions require the feagi-rust-py-libs package to be installed.
"""

import feagi_rust_py_libs as frpl
from typing import List, Dict, Tuple, Any

# Constants for byte structure types (should match Rust implementation)
MULTI_STRUCT_HOLDER = 0
NEURON_POTENTIAL_CATEGORICAL_XYZ = 11


def infer_byte_structure_type_rust(bytes_data: bytes) -> int:
    """
    Infer the type of FEAGI byte structure from raw bytes (Rust implementation).
    
    Args:
        bytes_data: Raw bytes data
        
    Returns:
        Integer representing the byte structure type
        
    Raises:
        ValueError: If the byte structure type cannot be inferred
        ImportError: If feagi-rust-py-libs is not installed
    """
    return frpl.byte_data_functions.infer_byte_structure_type(bytes_data)


def extract_sub_structures_rust(bytes_data: bytes) -> List[bytes]:
    """
    Extract sub-structures from a multi-structure holder (Rust implementation).
    
    Args:
        bytes_data: Raw bytes data of a multi-structure holder
        
    Returns:
        List of byte arrays representing the sub-structures
        
    Raises:
        ValueError: If the bytes data is not a valid multi-structure holder
        ImportError: If feagi-rust-py-libs is not installed
    """
    return frpl.byte_data_functions.extract_sub_structures(bytes_data)


def decode_neuron_potential_xyz_rust(bytes_data: bytes) -> Dict[Tuple[int, int, int], float]:
    """
    Decode neuron potential data from XYZ categorical format (Rust implementation).
    
    Args:
        bytes_data: Raw bytes in NEURON_POTENTIAL_CATEGORICAL_XYZ format
        
    Returns:
        Dictionary mapping (x, y, z) coordinates to activation values (0.0-1.0)
        
    Raises:
        ValueError: If the bytes data is not in the expected format
        ImportError: If feagi-rust-py-libs is not installed
    """
    return frpl.byte_data_functions.decode_neuron_potential_xyz(bytes_data)


def encode_neuron_potential_xyz_rust(neuron_data: Dict[Tuple[int, int, int], float]) -> bytes:
    """
    Encode neuron potential data into XYZ categorical format (Rust implementation).
    
    Args:
        neuron_data: Dictionary mapping (x, y, z) coordinates to activation values (0.0-1.0)
        
    Returns:
        Bytes in NEURON_POTENTIAL_CATEGORICAL_XYZ format
        
    Raises:
        ValueError: If the input data is invalid
        ImportError: If feagi-rust-py-libs is not installed
    """
    return frpl.byte_data_functions.encode_neuron_potential_xyz(neuron_data)