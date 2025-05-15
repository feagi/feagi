"""
Binary serialization for FEAGI protocols

This module provides serialization and deserialization functions for
the binary formats used in FEAGI communication protocols.
"""

import struct
import time
from typing import Dict, Any, Optional, Tuple

from feagi_connector.protocols import ProtocolID


class BinarySerializer:
    """
    Binary serializer for FEAGI protocols.
    
    This class provides methods to serialize and deserialize messages
    according to the binary protocol specifications.
    """
    
    # Protocol header format: protocol_id (1 byte) + version (1 byte)
    HEADER_FORMAT = "!BB"
    HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
    
    # FCP message format: header + command_type (1 byte) + message_length (4 bytes) + payload
    FCP_HEADER_FORMAT = "!BI"
    FCP_HEADER_SIZE = struct.calcsize(FCP_HEADER_FORMAT)
    
    # FVP message format: header + frame_type (1 byte) + timestamp (8 bytes) + data_length (4 bytes) + payload
    FVP_HEADER_FORMAT = "!BQI"
    FVP_HEADER_SIZE = struct.calcsize(FVP_HEADER_FORMAT)
    
    # FSMP message format: header + channel_id (2 bytes) + timestamp (8 bytes) + data_length (4 bytes) + payload
    FSMP_HEADER_FORMAT = "!HQI"
    FSMP_HEADER_SIZE = struct.calcsize(FSMP_HEADER_FORMAT)
    
    @classmethod
    def encode_header(cls, protocol_id: int, version: int) -> bytes:
        """Encode the protocol header."""
        return struct.pack(cls.HEADER_FORMAT, protocol_id, version)
    
    @classmethod
    def decode_header(cls, data: bytes) -> Tuple[int, int]:
        """Decode the protocol header."""
        if len(data) < cls.HEADER_SIZE:
            raise ValueError(f"Data too short for header: {len(data)} bytes")
            
        protocol_id, version = struct.unpack(cls.HEADER_FORMAT, data[:cls.HEADER_SIZE])
        return protocol_id, version
    
    @classmethod
    def encode_fcp(cls, command_type: int, payload: bytes, version: int = 1) -> bytes:
        """Encode an FCP message."""
        header = cls.encode_header(ProtocolID.FCP, version)
        message_length = len(payload)
        fcp_header = struct.pack(cls.FCP_HEADER_FORMAT, command_type, message_length)
        return header + fcp_header + payload
    
    @classmethod
    def decode_fcp(cls, data: bytes) -> Dict[str, Any]:
        """Decode an FCP message."""
        if len(data) < cls.HEADER_SIZE + cls.FCP_HEADER_SIZE:
            raise ValueError(f"Data too short for FCP message: {len(data)} bytes")
            
        # Extract header
        protocol_id, version = cls.decode_header(data)
        if protocol_id != ProtocolID.FCP:
            raise ValueError(f"Expected FCP protocol (1), got {protocol_id}")
            
        # Extract FCP header
        offset = cls.HEADER_SIZE
        command_type, message_length = struct.unpack(
            cls.FCP_HEADER_FORMAT, 
            data[offset:offset + cls.FCP_HEADER_SIZE]
        )
        offset += cls.FCP_HEADER_SIZE
        
        # Extract payload
        if len(data) < offset + message_length:
            raise ValueError(f"Data too short for payload: expected {message_length} bytes")
            
        payload = data[offset:offset + message_length]
        
        return {
            "protocol_id": protocol_id,
            "version": version,
            "command_type": command_type,
            "payload": payload
        }
    
    @classmethod
    def encode_fsmp(cls, channel_id: int, payload: bytes, timestamp: Optional[int] = None, version: int = 1) -> bytes:
        """Encode an FSMP message."""
        header = cls.encode_header(ProtocolID.FSMP, version)
        
        # Use current time if timestamp not provided
        if timestamp is None:
            timestamp = int(time.time() * 1000)
            
        data_length = len(payload)
        fsmp_header = struct.pack(cls.FSMP_HEADER_FORMAT, channel_id, timestamp, data_length)
        return header + fsmp_header + payload
    
    @classmethod
    def decode_fsmp(cls, data: bytes) -> Dict[str, Any]:
        """Decode an FSMP message."""
        if len(data) < cls.HEADER_SIZE + cls.FSMP_HEADER_SIZE:
            raise ValueError(f"Data too short for FSMP message: {len(data)} bytes")
            
        # Extract header
        protocol_id, version = cls.decode_header(data)
        if protocol_id != ProtocolID.FSMP:
            raise ValueError(f"Expected FSMP protocol (3), got {protocol_id}")
            
        # Extract FSMP header
        offset = cls.HEADER_SIZE
        channel_id, timestamp, data_length = struct.unpack(
            cls.FSMP_HEADER_FORMAT, 
            data[offset:offset + cls.FSMP_HEADER_SIZE]
        )
        offset += cls.FSMP_HEADER_SIZE
        
        # Extract payload
        if len(data) < offset + data_length:
            raise ValueError(f"Data too short for payload: expected {data_length} bytes")
            
        payload = data[offset:offset + data_length]
        
        return {
            "protocol_id": protocol_id,
            "version": version,
            "channel_id": channel_id,
            "timestamp": timestamp,
            "payload": payload
        }
    
    @classmethod
    def encode_fvp(cls, frame_type: int, payload: bytes, timestamp: Optional[int] = None, version: int = 1) -> bytes:
        """Encode an FVP message."""
        header = cls.encode_header(ProtocolID.FVP, version)
        
        # Use current time if timestamp not provided
        if timestamp is None:
            timestamp = int(time.time() * 1000)
            
        data_length = len(payload)
        fvp_header = struct.pack(cls.FVP_HEADER_FORMAT, frame_type, timestamp, data_length)
        return header + fvp_header + payload
    
    @classmethod
    def decode_fvp(cls, data: bytes) -> Dict[str, Any]:
        """Decode an FVP message."""
        if len(data) < cls.HEADER_SIZE + cls.FVP_HEADER_SIZE:
            raise ValueError(f"Data too short for FVP message: {len(data)} bytes")
            
        # Extract header
        protocol_id, version = cls.decode_header(data)
        if protocol_id != ProtocolID.FVP:
            raise ValueError(f"Expected FVP protocol (2), got {protocol_id}")
            
        # Extract FVP header
        offset = cls.HEADER_SIZE
        frame_type, timestamp, data_length = struct.unpack(
            cls.FVP_HEADER_FORMAT, 
            data[offset:offset + cls.FVP_HEADER_SIZE]
        )
        offset += cls.FVP_HEADER_SIZE
        
        # Extract payload
        if len(data) < offset + data_length:
            raise ValueError(f"Data too short for payload: expected {data_length} bytes")
            
        payload = data[offset:offset + data_length]
        
        return {
            "protocol_id": protocol_id,
            "version": version,
            "frame_type": frame_type,
            "timestamp": timestamp,
            "payload": payload
        } 