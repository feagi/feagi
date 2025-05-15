"""
Binary serialization for FEAGI protocols

This module provides serialization and deserialization functions for
the binary byte structures used in FEAGI communication protocols.
"""

import struct
import time
import json
import zlib
from typing import Dict, Any, Optional, Tuple, List, Union

from feagi_connector.protocols import ByteStructureID


class ByteStructureEncoder:
    """
    Encoder for FEAGI byte structures.
    
    This class provides methods to encode messages according to the
    binary byte structure specifications with versioning support.
    """
    
    # Byte structure header format: structure_type (1 byte) + version (1 byte)
    HEADER_FORMAT = "!BB"
    HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
    
    # Default supported versions for each structure type
    SUPPORTED_VERSIONS = {
        ByteStructureID.JSON: [1],
        ByteStructureID.RAW_IMAGE: [1],
        ByteStructureID.MULTI_HOLDER: [1],
        ByteStructureID.NEURON_FLAT: [1],
        ByteStructureID.NEURON_CATEGORIES: [1],
    }
    
    def __init__(self):
        # Default to highest supported version for each structure type
        self.default_versions = {
            structure_id: max(versions)
            for structure_id, versions in self.SUPPORTED_VERSIONS.items()
        }
    
    def encode_header(self, structure_id: int, version: int) -> bytes:
        """Encode the byte structure header."""
        return struct.pack(self.HEADER_FORMAT, structure_id, version)
    
    def encode_json(self, data: Dict[str, Any], version: int = None) -> bytes:
        """Encode data as a JSON byte structure."""
        version = version or self.default_versions.get(ByteStructureID.JSON, 1)
        if version not in self.SUPPORTED_VERSIONS[ByteStructureID.JSON]:
            raise ValueError(f"Unsupported JSON structure version: {version}")
            
        if version == 1:
            return self._encode_json_v1(data)
        
        raise ValueError(f"Unknown JSON structure version: {version}")
    
    def _encode_json_v1(self, data: Dict[str, Any]) -> bytes:
        """Encode data as a JSON byte structure (version 1)."""
        header = self.encode_header(ByteStructureID.JSON, 1)
        json_data = json.dumps(data).encode('utf-8')
        return header + json_data
    
    def encode_raw_image(self, image_data: bytes, width: int, height: int, channels: int, version: int = None) -> bytes:
        """Encode image data as a raw image byte structure."""
        version = version or self.default_versions.get(ByteStructureID.RAW_IMAGE, 1)
        if version not in self.SUPPORTED_VERSIONS[ByteStructureID.RAW_IMAGE]:
            raise ValueError(f"Unsupported raw image structure version: {version}")
            
        if version == 1:
            return self._encode_raw_image_v1(image_data, width, height, channels)
        
        raise ValueError(f"Unknown raw image structure version: {version}")
    
    def _encode_raw_image_v1(self, image_data: bytes, width: int, height: int, channels: int) -> bytes:
        """Encode image data as a raw image byte structure (version 1)."""
        header = self.encode_header(ByteStructureID.RAW_IMAGE, 1)
        metadata = struct.pack("!III", width, height, channels)
        return header + metadata + image_data
    
    def encode_neuron_flat(self, neuron_data: bytes, area_id: str, version: int = None) -> bytes:
        """Encode neuron data as a flat neuron byte structure."""
        version = version or self.default_versions.get(ByteStructureID.NEURON_FLAT, 1)
        if version not in self.SUPPORTED_VERSIONS[ByteStructureID.NEURON_FLAT]:
            raise ValueError(f"Unsupported neuron flat structure version: {version}")
            
        if version == 1:
            return self._encode_neuron_flat_v1(neuron_data, area_id)
        
        raise ValueError(f"Unknown neuron flat structure version: {version}")
    
    def _encode_neuron_flat_v1(self, neuron_data: bytes, area_id: str) -> bytes:
        """Encode neuron data as a flat neuron byte structure (version 1)."""
        header = self.encode_header(ByteStructureID.NEURON_FLAT, 1)
        area_id_bytes = area_id.encode('utf-8')
        area_id_length = len(area_id_bytes)
        
        # Structure: header + area_id_length (2 bytes) + area_id + neuron_data
        metadata = struct.pack("!H", area_id_length)
        return header + metadata + area_id_bytes + neuron_data
    
    def encode_neuron_categories(self, category_data: Dict[str, bytes], version: int = None) -> bytes:
        """Encode multi-area neuron data as a categorized neuron byte structure."""
        version = version or self.default_versions.get(ByteStructureID.NEURON_CATEGORIES, 1)
        if version not in self.SUPPORTED_VERSIONS[ByteStructureID.NEURON_CATEGORIES]:
            raise ValueError(f"Unsupported neuron categories structure version: {version}")
            
        if version == 1:
            return self._encode_neuron_categories_v1(category_data)
        
        raise ValueError(f"Unknown neuron categories structure version: {version}")
    
    def _encode_neuron_categories_v1(self, category_data: Dict[str, bytes]) -> bytes:
        """Encode multi-area neuron data as a categorized neuron byte structure (version 1)."""
        header = self.encode_header(ByteStructureID.NEURON_CATEGORIES, 1)
        
        # Count number of areas
        num_areas = len(category_data)
        
        # Create buffer to build the structure
        buffer = bytearray()
        buffer.extend(struct.pack("!I", num_areas))
        
        # Add each area's data
        for area_id, data in category_data.items():
            area_id_bytes = area_id.encode('utf-8')
            area_id_length = len(area_id_bytes)
            data_length = len(data)
            
            # Add area metadata and data
            buffer.extend(struct.pack("!HI", area_id_length, data_length))
            buffer.extend(area_id_bytes)
            buffer.extend(data)
        
        return header + bytes(buffer)
    
    def encode_multi_holder(self, structures: List[bytes], version: int = None) -> bytes:
        """Encode multiple byte structures as a single multi-holder structure."""
        version = version or self.default_versions.get(ByteStructureID.MULTI_HOLDER, 1)
        if version not in self.SUPPORTED_VERSIONS[ByteStructureID.MULTI_HOLDER]:
            raise ValueError(f"Unsupported multi-holder structure version: {version}")
            
        if version == 1:
            return self._encode_multi_holder_v1(structures)
        
        raise ValueError(f"Unknown multi-holder structure version: {version}")
    
    def _encode_multi_holder_v1(self, structures: List[bytes]) -> bytes:
        """Encode multiple byte structures as a single multi-holder structure (version 1)."""
        header = self.encode_header(ByteStructureID.MULTI_HOLDER, 1)
        
        # Number of structures (4 bytes)
        num_structures = len(structures)
        buffer = bytearray(struct.pack("!I", num_structures))
        
        # Add each structure with its length
        for structure in structures:
            structure_length = len(structure)
            buffer.extend(struct.pack("!I", structure_length))
            buffer.extend(structure)
        
        return header + bytes(buffer)

    def compress(self, data: bytes) -> bytes:
        """Compress byte structure data using deflate algorithm."""
        # Indicate compression with the first byte (1=compressed)
        return b'\x01' + zlib.compress(data)


class ByteStructureDecoder:
    """
    Decoder for FEAGI byte structures.
    
    This class provides methods to decode binary byte structures
    into Python objects with versioning support.
    """
    
    # Byte structure header format: structure_type (1 byte) + version (1 byte)
    HEADER_FORMAT = "!BB"
    HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
    
    def decode_header(self, data: bytes) -> Tuple[int, int]:
        """Decode the byte structure header."""
        if len(data) < self.HEADER_SIZE:
            raise ValueError(f"Data too short for header: {len(data)} bytes")
            
        structure_id, version = struct.unpack(self.HEADER_FORMAT, data[:self.HEADER_SIZE])
        return structure_id, version
    
    def decompress(self, data: bytes) -> bytes:
        """Decompress compressed byte structure data."""
        if not data or len(data) < 1:
            raise ValueError("Empty or too short data")
            
        compression_flag = data[0]
        if compression_flag == 1:  # Compressed
            return zlib.decompress(data[1:])
        elif compression_flag == 0:  # Uncompressed
            return data[1:]
        else:
            raise ValueError(f"Unknown compression flag: {compression_flag}")
    
    def detect_compression(self, data: bytes) -> bool:
        """Detect if data is compressed."""
        if not data or len(data) < 1:
            raise ValueError("Empty or too short data")
        return data[0] == 1
    
    def decode(self, data: bytes) -> Dict[str, Any]:
        """
        Decode a byte structure based on its type.
        
        This is the main entry point for decoding byte structures.
        """
        # Check for compression
        if self.detect_compression(data):
            data = self.decompress(data)
        
        # Get structure type and version from header
        if len(data) < self.HEADER_SIZE:
            raise ValueError(f"Data too short for header: {len(data)} bytes")
            
        structure_id, version = self.decode_header(data)
        
        # Dispatch to appropriate decoder based on type
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
            raise ValueError(f"Unknown structure type: {structure_id}")
    
    def decode_json(self, data: bytes) -> Dict[str, Any]:
        """Decode a JSON byte structure."""
        structure_id, version = self.decode_header(data)
        if structure_id != ByteStructureID.JSON:
            raise ValueError(f"Expected JSON structure (1), got {structure_id}")
        
        if version == 1:
            return self._decode_json_v1(data)
            
        raise ValueError(f"Unsupported JSON structure version: {version}")
    
    def _decode_json_v1(self, data: bytes) -> Dict[str, Any]:
        """Decode a JSON byte structure (version 1)."""
        json_data = data[self.HEADER_SIZE:].decode('utf-8')
        return json.loads(json_data)
    
    def decode_raw_image(self, data: bytes) -> Dict[str, Any]:
        """Decode a raw image byte structure."""
        structure_id, version = self.decode_header(data)
        if structure_id != ByteStructureID.RAW_IMAGE:
            raise ValueError(f"Expected raw image structure (8), got {structure_id}")
        
        if version == 1:
            return self._decode_raw_image_v1(data)
            
        raise ValueError(f"Unsupported raw image structure version: {version}")
    
    def _decode_raw_image_v1(self, data: bytes) -> Dict[str, Any]:
        """Decode a raw image byte structure (version 1)."""
        metadata_offset = self.HEADER_SIZE
        metadata_size = struct.calcsize("!III")  # width, height, channels
        
        if len(data) < metadata_offset + metadata_size:
            raise ValueError("Data too short for image metadata")
            
        width, height, channels = struct.unpack(
            "!III",
            data[metadata_offset:metadata_offset + metadata_size]
        )
        
        image_data = data[metadata_offset + metadata_size:]
        
        return {
            "width": width,
            "height": height,
            "channels": channels,
            "data": image_data
        }
    
    def decode_neuron_flat(self, data: bytes) -> Dict[str, Any]:
        """Decode a flat neuron byte structure."""
        structure_id, version = self.decode_header(data)
        if structure_id != ByteStructureID.NEURON_FLAT:
            raise ValueError(f"Expected flat neuron structure (10), got {structure_id}")
        
        if version == 1:
            return self._decode_neuron_flat_v1(data)
            
        raise ValueError(f"Unsupported flat neuron structure version: {version}")
    
    def _decode_neuron_flat_v1(self, data: bytes) -> Dict[str, Any]:
        """Decode a flat neuron byte structure (version 1)."""
        offset = self.HEADER_SIZE
        
        # Extract area ID length
        area_id_length_size = struct.calcsize("!H")
        if len(data) < offset + area_id_length_size:
            raise ValueError("Data too short for area ID length")
            
        area_id_length = struct.unpack("!H", data[offset:offset + area_id_length_size])[0]
        offset += area_id_length_size
        
        # Extract area ID
        if len(data) < offset + area_id_length:
            raise ValueError("Data too short for area ID")
            
        area_id = data[offset:offset + area_id_length].decode('utf-8')
        offset += area_id_length
        
        # Extract neuron data
        neuron_data = data[offset:]
        
        return {
            "area_id": area_id,
            "data": neuron_data
        }
    
    def decode_neuron_categories(self, data: bytes) -> Dict[str, Any]:
        """Decode a categorized neuron byte structure."""
        structure_id, version = self.decode_header(data)
        if structure_id != ByteStructureID.NEURON_CATEGORIES:
            raise ValueError(f"Expected categorized neuron structure (11), got {structure_id}")
        
        if version == 1:
            return self._decode_neuron_categories_v1(data)
            
        raise ValueError(f"Unsupported categorized neuron structure version: {version}")
    
    def _decode_neuron_categories_v1(self, data: bytes) -> Dict[str, Any]:
        """Decode a categorized neuron byte structure (version 1)."""
        offset = self.HEADER_SIZE
        
        # Extract number of areas
        num_areas_size = struct.calcsize("!I")
        if len(data) < offset + num_areas_size:
            raise ValueError("Data too short for number of areas")
            
        num_areas = struct.unpack("!I", data[offset:offset + num_areas_size])[0]
        offset += num_areas_size
        
        # Extract each area's data
        areas = {}
        for _ in range(num_areas):
            # Extract area metadata
            metadata_size = struct.calcsize("!HI")  # area_id_length, data_length
            if len(data) < offset + metadata_size:
                raise ValueError("Data too short for area metadata")
                
            area_id_length, data_length = struct.unpack(
                "!HI",
                data[offset:offset + metadata_size]
            )
            offset += metadata_size
            
            # Extract area ID
            if len(data) < offset + area_id_length:
                raise ValueError("Data too short for area ID")
                
            area_id = data[offset:offset + area_id_length].decode('utf-8')
            offset += area_id_length
            
            # Extract area data
            if len(data) < offset + data_length:
                raise ValueError("Data too short for area data")
                
            area_data = data[offset:offset + data_length]
            offset += data_length
            
            areas[area_id] = area_data
        
        return {
            "areas": areas
        }
    
    def decode_multi_holder(self, data: bytes) -> Dict[str, Any]:
        """Decode a multi-holder byte structure."""
        structure_id, version = self.decode_header(data)
        if structure_id != ByteStructureID.MULTI_HOLDER:
            raise ValueError(f"Expected multi-holder structure (9), got {structure_id}")
        
        if version == 1:
            return self._decode_multi_holder_v1(data)
            
        raise ValueError(f"Unsupported multi-holder structure version: {version}")
    
    def _decode_multi_holder_v1(self, data: bytes) -> Dict[str, Any]:
        """Decode a multi-holder byte structure (version 1)."""
        offset = self.HEADER_SIZE
        
        # Extract number of structures
        num_structures_size = struct.calcsize("!I")
        if len(data) < offset + num_structures_size:
            raise ValueError("Data too short for number of structures")
            
        num_structures = struct.unpack("!I", data[offset:offset + num_structures_size])[0]
        offset += num_structures_size
        
        # Extract each structure
        structures = []
        for _ in range(num_structures):
            # Extract structure length
            length_size = struct.calcsize("!I")
            if len(data) < offset + length_size:
                raise ValueError("Data too short for structure length")
                
            structure_length = struct.unpack("!I", data[offset:offset + length_size])[0]
            offset += length_size
            
            # Extract structure data
            if len(data) < offset + structure_length:
                raise ValueError("Data too short for structure data")
                
            structure_data = data[offset:offset + structure_length]
            structures.append(structure_data)
            offset += structure_length
        
        return {
            "structures": structures
        }


class ByteStructureTranslator:
    """
    Translator between FEAGI byte structures and Python objects.
    
    This class provides a high-level interface for encoding and decoding
    FEAGI byte structures with version negotiation.
    """
    
    def __init__(self):
        self.encoder = ByteStructureEncoder()
        self.decoder = ByteStructureDecoder()
        
        # Client capabilities registry
        self.client_capabilities = {}
    
    def register_client_capabilities(self, client_id: str, capabilities: Dict[int, List[int]]):
        """
        Register a client's supported byte structure versions.
        
        Args:
            client_id: Client identifier
            capabilities: Dictionary mapping structure IDs to lists of supported versions
        """
        self.client_capabilities[client_id] = capabilities
    
    def get_supported_version(self, client_id: str, structure_id: int) -> int:
        """
        Get the highest mutually supported version for a structure type.
        
        Args:
            client_id: Client identifier
            structure_id: Byte structure type ID
            
        Returns:
            Highest mutually supported version
        """
        # If no client ID or not registered, use default
        if not client_id or client_id not in self.client_capabilities:
            return self.encoder.default_versions.get(structure_id, 1)
        
        # Get client's supported versions for this structure
        client_versions = self.client_capabilities[client_id].get(structure_id, [1])
        
        # Get server's supported versions
        server_versions = self.encoder.SUPPORTED_VERSIONS.get(structure_id, [1])
        
        # Find highest mutually supported version
        common_versions = [v for v in client_versions if v in server_versions]
        if not common_versions:
            return 1  # Fallback to version 1
            
        return max(common_versions)
    
    def encode_message(self, data: Dict[str, Any], client_id: str = None) -> bytes:
        """Encode a message as a JSON byte structure with appropriate version."""
        version = self.get_supported_version(client_id, ByteStructureID.JSON)
        return self.encoder.encode_json(data, version)
    
    def decode_message(self, data: bytes) -> Dict[str, Any]:
        """Decode a byte structure message."""
        # Check for compression
        if len(data) > 0 and data[0] in (0, 1):  # Compression flag
            if self.decoder.detect_compression(data):
                data = self.decoder.decompress(data)
                if len(data) > 0:
                    data = data[1:]  # Remove compression flag
        
        # Get structure type from header
        structure_id, version = self.decoder.decode_header(data)
        
        # Decode based on structure type
        if structure_id == ByteStructureID.JSON:
            return self.decoder.decode_json(data)
        elif structure_id == ByteStructureID.RAW_IMAGE:
            return self.decoder.decode_raw_image(data)
        elif structure_id == ByteStructureID.MULTI_HOLDER:
            return self.decoder.decode_multi_holder(data)
        elif structure_id == ByteStructureID.NEURON_FLAT:
            return self.decoder.decode_neuron_flat(data)
        elif structure_id == ByteStructureID.NEURON_CATEGORIES:
            return self.decoder.decode_neuron_categories(data)
        else:
            raise ValueError(f"Unknown structure type: {structure_id}") 