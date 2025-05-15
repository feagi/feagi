"""
Binary serialization for FEAGI protocols

This module provides optimized serialization and deserialization for 
FEAGI communication protocols using custom byte structures.
"""

import struct
import json
import zlib
import logging
from typing import Dict, List, Any, Union, Optional, Tuple

import numpy as np

from .constants import ByteStructureID
from .utils import validate_cortical_id, is_compressed, get_structure_info

# Configure logging
logger = logging.getLogger(__name__)

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
    byte structures optimized for neural data transmission.
    """
    
    def __init__(self):
        """Initialize encoder with default versions."""
        # Default to highest supported version for each structure type
        self.default_versions = {
            structure_id: max(versions)
            for structure_id, versions in SUPPORTED_VERSIONS.items()
        }
    
    def encode_header(self, structure_id: int, version: int = 1) -> bytes:
        """
        Encode the universal FEAGI byte structure header.
        
        Args:
            structure_id: Byte structure ID (see ByteStructureID enum)
            version: Structure version (default: 1)
        
        Returns:
            Encoded header bytes (2 bytes total)
        """
        return struct.pack("!BB", structure_id, version)
    
    def encode_json(self, data: Union[Dict, List, str], version: Optional[int] = None) -> bytes:
        """
        Encode data as a JSON byte structure (ID: 1).
        
        Args:
            data: Data to encode as JSON
            version: Structure version to use (default: use highest supported)
            
        Returns:
            Encoded byte structure
            
        Raises:
            ValueError: If the specified version is not supported
        """
        version = version or self.default_versions.get(ByteStructureID.JSON, 1)
        if version not in SUPPORTED_VERSIONS[ByteStructureID.JSON]:
            raise ValueError(f"Unsupported version {version} for JSON structure")
            
        if version == 1:
            return self._encode_json_v1(data)
        else:
            # Should never reach here due to the check above
            raise ValueError(f"Version {version} implementation missing for JSON structure")
    
    def _encode_json_v1(self, data: Union[Dict, List, str]) -> bytes:
        """Version 1 implementation of JSON structure."""
        # Ensure data is serializable
        if isinstance(data, str):
            json_str = data
        else:
            json_str = json.dumps(data)
        
        # Convert to UTF-8 bytes
        json_bytes = json_str.encode('utf-8')
        
        # Create header and append data
        header = self.encode_header(ByteStructureID.JSON, version=1)
        return header + json_bytes
    
    def encode_raw_image(self, image: np.ndarray, version: Optional[int] = None) -> bytes:
        """
        Encode a raw image as byte structure (ID: 8).
        
        Args:
            image: Image data as numpy array (height, width, 3)
                  Expected format is BGR (uint8)
            version: Structure version to use (default: use highest supported)
                  
        Returns:
            Encoded byte structure
            
        Raises:
            ValueError: If the specified version is not supported
        """
        version = version or self.default_versions.get(ByteStructureID.RAW_IMAGE, 1)
        if version not in SUPPORTED_VERSIONS[ByteStructureID.RAW_IMAGE]:
            raise ValueError(f"Unsupported version {version} for raw image structure")
            
        if version == 1:
            return self._encode_raw_image_v1(image)
        else:
            # Should never reach here due to the check above
            raise ValueError(f"Version {version} implementation missing for raw image structure")
    
    def _encode_raw_image_v1(self, image: np.ndarray) -> bytes:
        """Version 1 implementation of raw image structure."""
        # Validate input
        if not isinstance(image, np.ndarray) or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("Image must be a 3D numpy array with shape (height, width, 3)")
        
        if image.dtype != np.uint8:
            raise ValueError("Image must have dtype uint8")
        
        # Extract dimensions
        height, width, channels = image.shape
        
        # Create header
        header = self.encode_header(ByteStructureID.RAW_IMAGE, version=1)
        
        # Create metadata section
        metadata = struct.pack("!III", width, height, channels)
        
        # Get raw image bytes (ensure C-contiguous memory layout for performance)
        image_bytes = np.ascontiguousarray(image).tobytes()
        
        # Combine all parts
        return header + metadata + image_bytes
    
    def encode_multi_holder(self, byte_structures: List[bytes], version: Optional[int] = None) -> bytes:
        """
        Encode multiple byte structures into a single multi-holder structure (ID: 9).
        
        Args:
            byte_structures: List of encoded byte structures to combine
            version: Structure version to use (default: use highest supported)
            
        Returns:
            Encoded multi-holder byte structure
            
        Raises:
            ValueError: If the specified version is not supported or too many structures
        """
        version = version or self.default_versions.get(ByteStructureID.MULTI_HOLDER, 1)
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
        version: Optional[int] = None
    ) -> bytes:
        """
        Encode neuron data in flat format (ID: 10).
        
        Args:
            cortical_ids: List of cortical area IDs (one per neuron)
            x_coords: List of X coordinates
            y_coords: List of Y coordinates
            z_coords: List of Z coordinates
            potentials: List of activation potentials
            version: Structure version to use (default: use highest supported)
            
        Returns:
            Encoded neuron data byte structure
            
        Raises:
            ValueError: If the specified version is not supported or input data is invalid
        """
        version = version or self.default_versions.get(ByteStructureID.NEURON_FLAT, 1)
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
        version: Optional[int] = None
    ) -> bytes:
        """
        Encode categorized neuron data (ID: 11).
        
        Args:
            cortical_data: Dict mapping cortical IDs to neuron data
                          Each value contains 'x', 'y', 'z', and 'potentials' lists
            version: Structure version to use (default: use highest supported)
            
        Returns:
            Encoded categorized neuron data structure
            
        Raises:
            ValueError: If the specified version is not supported or input data is invalid
        """
        version = version or self.default_versions.get(ByteStructureID.NEURON_CATEGORIES, 1)
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
        
        # Get number of cortical areas
        num_areas = len(cortical_data)
        init_header = struct.pack("!I", num_areas)
        
        # Initialize sections
        secondary_headers = bytearray()
        neuron_data = bytearray()
        
        # Track current index in neuron data section
        current_index = 0
        
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
            area_data.extend(struct.pack(f"!{neuron_count}i", *x_coords))  # 4 bytes per int32
            area_data.extend(struct.pack(f"!{neuron_count}i", *y_coords))  # 4 bytes per int32
            area_data.extend(struct.pack(f"!{neuron_count}i", *z_coords))  # 4 bytes per int32
            area_data.extend(struct.pack(f"!{neuron_count}f", *potentials))  # 4 bytes per float
            
            # Add to neuron data section
            neuron_data.extend(area_data)
            
            # Update index for next area
            current_index += len(area_data)
        
        # Combine all parts
        return header + init_header + secondary_headers + neuron_data
        
    def compress(self, data: bytes) -> bytes:
        """
        Compress byte structure data using zlib.
        
        Args:
            data: Raw byte data to compress
            
        Returns:
            Compressed data with flag byte prepended (1=compressed)
        """
        return b'\x01' + zlib.compress(data)


class ByteStructureDecoder:
    """
    Decoder for FEAGI byte structures.
    
    This class provides methods to decode FEAGI byte structures back into
    usable data structures according to their specifications.
    """
    
    def decode_header(self, data: bytes) -> Tuple[int, int]:
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
    
    def get_structure_type(self, data: bytes) -> int:
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
            Image as numpy array (height, width, 3)
            
        Raises:
            ValueError: If header is invalid or version is unsupported
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
        # Skip header (2 bytes) and read metadata (12 bytes)
        metadata_end = 2 + 12  # header + 3 uint32 values
        if len(data) < metadata_end:
            raise ValueError("Data too short for raw image metadata")
        
        # Extract metadata (width, height, channels)
        width, height, channels = struct.unpack("!III", data[2:metadata_end])
        
        # Validate expected data size
        expected_size = width * height * channels
        if len(data) - metadata_end < expected_size:
            raise ValueError(f"Data too short for image: expected {expected_size} bytes")
        
        # Extract image data and reshape
        image_data = data[metadata_end:metadata_end + expected_size]
        image = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width, channels))
        
        return image
    
    def decode_multi_holder(self, data: bytes) -> List[bytes]:
        """
        Decode a multi-holder byte structure (ID: 9).
        
        Args:
            data: Encoded multi-holder byte structure
            
        Returns:
            List of contained byte structures
            
        Raises:
            ValueError: If header is invalid or version is unsupported
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
        Decode a neuron flat byte structure (ID: 10).
        
        Args:
            data: Encoded neuron flat byte structure
            
        Returns:
            Dictionary with cortical IDs, coordinates, and potentials
            
        Raises:
            ValueError: If header is invalid or version is unsupported
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
            return self._decode_neuron_flat_v1(data)
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
            cortical_id = data[offset:offset+6].decode('ascii').rstrip()
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
        
        # Return all data
        return {
            "cortical_ids": cortical_ids,
            "x": x_coords,
            "y": y_coords,
            "z": z_coords,
            "potentials": potentials
        }
    
    def decode_neuron_categories(self, data: bytes) -> Dict[str, Dict[str, List]]:
        """
        Decode a neuron categories byte structure (ID: 11).
        
        Args:
            data: Encoded neuron categories byte structure
            
        Returns:
            Dictionary mapping cortical IDs to neuron data
            
        Raises:
            ValueError: If header is invalid or version is unsupported
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
            return self._decode_neuron_categories_v1(data)
        else:
            # Should never reach here due to the check above
            raise ValueError(f"Version {version} implementation missing for neuron categories structure")
    
    def _decode_neuron_categories_v1(self, data: bytes) -> Dict[str, Dict[str, List]]:
        """Decode version 1 of neuron categories structure."""
        # Validate minimum size: header (2) + num areas (4)
        if len(data) < 6:
            raise ValueError("Data too short for neuron categories structure")
        
        # Extract number of areas
        num_areas = struct.unpack("!I", data[2:6])[0]
        
        # Parse secondary headers
        result = {}
        secondary_header_offset = 6
        
        for i in range(num_areas):
            # Each secondary header: cortical_id (6) + data_offset (4) + neuron_count (4)
            if secondary_header_offset + 14 > len(data):
                raise ValueError(f"Data too short for secondary header {i}")
            
            # Extract cortical ID
            cortical_id_bytes = data[secondary_header_offset:secondary_header_offset+6]
            cortical_id = cortical_id_bytes.decode('ascii').rstrip()
            
            # Extract data offset and neuron count
            data_offset_bytes = data[secondary_header_offset+6:secondary_header_offset+10]
            neuron_count_bytes = data[secondary_header_offset+10:secondary_header_offset+14]
            
            data_offset = struct.unpack("!I", data_offset_bytes)[0]
            neuron_count = struct.unpack("!I", neuron_count_bytes)[0]
            
            # Advance to next header
            secondary_header_offset += 14
            
            # Calculate absolute offset in the data section
            # Data section starts after all headers (header + num_areas + all secondary headers)
            data_section_start = 6 + (num_areas * 14)
            absolute_offset = data_section_start + data_offset
            
            # Calculate size of this area's data
            bytes_per_neuron = 16  # 4 bytes each for x, y, z, potential
            area_data_size = neuron_count * bytes_per_neuron
            
            # Validate range
            if absolute_offset + area_data_size > len(data):
                raise ValueError(f"Data too short for area {cortical_id}")
            
            # Extract coordinates and potentials
            x_coords_offset = absolute_offset
            y_coords_offset = x_coords_offset + (neuron_count * 4)
            z_coords_offset = y_coords_offset + (neuron_count * 4)
            potentials_offset = z_coords_offset + (neuron_count * 4)
            
            x_coords = list(struct.unpack(f"!{neuron_count}i", 
                                        data[x_coords_offset:y_coords_offset]))
            y_coords = list(struct.unpack(f"!{neuron_count}i", 
                                        data[y_coords_offset:z_coords_offset]))
            z_coords = list(struct.unpack(f"!{neuron_count}i", 
                                        data[z_coords_offset:potentials_offset]))
            potentials = list(struct.unpack(f"!{neuron_count}f", 
                                        data[potentials_offset:potentials_offset+(neuron_count*4)]))
            
            # Store in result dictionary
            result[cortical_id] = {
                "x": x_coords,
                "y": y_coords,
                "z": z_coords,
                "potentials": potentials
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
    
    def decompress(self, data: bytes) -> bytes:
        """
        Decompress data using Deflate algorithm.
        
        Args:
            data: Compressed data bytes (first byte should be compression flag)
            
        Returns:
            Decompressed bytes
        """
        if len(data) == 0:
            raise ValueError("Empty data")
            
        compression_flag = data[0]
        
        if compression_flag == 0:
            # Not compressed, return data without flag
            return data[1:]
        elif compression_flag == 1:
            # Compressed, decompress and return
            return zlib.decompress(data[1:])
        else:
            raise ValueError(f"Unknown compression flag: {compression_flag}")
    
    def detect_compression(self, data: bytes) -> bool:
        """
        Detect if data is compressed.
        
        Args:
            data: Byte data to check (first byte should be compression flag)
            
        Returns:
            True if compressed, False otherwise
            
        Raises:
            ValueError: If data is empty
        """
        if len(data) == 0:
            raise ValueError("Empty data")
            
        return data[0] == 1 