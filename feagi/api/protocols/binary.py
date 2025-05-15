"""
Binary Protocol Serialization for FEAGI

This module provides binary serialization and deserialization functions for
the FEAGI communication protocols (FCP, FVP, FSMP).
"""

import struct
import time
from typing import Dict, Any, Tuple, Union, Optional, List
from enum import IntEnum

from .base import ProtocolID


class BinaryProtocolError(Exception):
    """Exception raised for errors in the binary protocol."""
    pass


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
    def encode_header(cls, protocol_id: ProtocolID, version: int) -> bytes:
        """
        Encode the protocol header.
        
        Args:
            protocol_id: Protocol identifier
            version: Protocol version
            
        Returns:
            Encoded header bytes
        """
        return struct.pack(cls.HEADER_FORMAT, protocol_id.value, version)
    
    @classmethod
    def decode_header(cls, data: bytes) -> Tuple[ProtocolID, int]:
        """
        Decode the protocol header.
        
        Args:
            data: Binary data containing the header
            
        Returns:
            Tuple of (protocol_id, version)
            
        Raises:
            BinaryProtocolError: If the data is too short or invalid
        """
        if len(data) < cls.HEADER_SIZE:
            raise BinaryProtocolError(f"Data too short for header: {len(data)} bytes")
            
        try:
            protocol_id_value, version = struct.unpack(cls.HEADER_FORMAT, data[:cls.HEADER_SIZE])
            protocol_id = ProtocolID(protocol_id_value)
            return protocol_id, version
        except (struct.error, ValueError) as e:
            raise BinaryProtocolError(f"Invalid header: {e}")
    
    @classmethod
    def encode_fcp(cls, command_type: int, payload: bytes, version: int = 1) -> bytes:
        """
        Encode an FCP message.
        
        Args:
            command_type: Command type
            payload: Message payload
            version: Protocol version
            
        Returns:
            Encoded FCP message
        """
        header = cls.encode_header(ProtocolID.FCP, version)
        message_length = len(payload)
        fcp_header = struct.pack(cls.FCP_HEADER_FORMAT, command_type, message_length)
        return header + fcp_header + payload
    
    @classmethod
    def decode_fcp(cls, data: bytes) -> Dict[str, Any]:
        """
        Decode an FCP message.
        
        Args:
            data: Binary FCP message
            
        Returns:
            Decoded message as a dictionary
            
        Raises:
            BinaryProtocolError: If the data is invalid
        """
        if len(data) < cls.HEADER_SIZE + cls.FCP_HEADER_SIZE:
            raise BinaryProtocolError(f"Data too short for FCP message: {len(data)} bytes")
            
        # Extract header
        protocol_id, version = cls.decode_header(data)
        if protocol_id != ProtocolID.FCP:
            raise BinaryProtocolError(f"Expected FCP protocol (1), got {protocol_id.value}")
            
        # Extract FCP header
        offset = cls.HEADER_SIZE
        command_type, message_length = struct.unpack(
            cls.FCP_HEADER_FORMAT, 
            data[offset:offset + cls.FCP_HEADER_SIZE]
        )
        offset += cls.FCP_HEADER_SIZE
        
        # Extract payload
        if len(data) < offset + message_length:
            raise BinaryProtocolError(f"Data too short for payload: expected {message_length} bytes")
            
        payload = data[offset:offset + message_length]
        
        return {
            "protocol_id": protocol_id.value,
            "version": version,
            "command_type": command_type,
            "payload": payload
        }
    
    @classmethod
    def encode_fvp(cls, frame_type: int, payload: bytes, timestamp: Optional[int] = None, version: int = 1) -> bytes:
        """
        Encode an FVP message.
        
        Args:
            frame_type: Frame type
            payload: Message payload
            timestamp: Timestamp in milliseconds (default: current time)
            version: Protocol version
            
        Returns:
            Encoded FVP message
        """
        header = cls.encode_header(ProtocolID.FVP, version)
        
        # Use current time if timestamp not provided
        if timestamp is None:
            timestamp = int(time.time() * 1000)
            
        data_length = len(payload)
        fvp_header = struct.pack(cls.FVP_HEADER_FORMAT, frame_type, timestamp, data_length)
        return header + fvp_header + payload
    
    @classmethod
    def decode_fvp(cls, data: bytes) -> Dict[str, Any]:
        """
        Decode an FVP message.
        
        Args:
            data: Binary FVP message
            
        Returns:
            Decoded message as a dictionary
            
        Raises:
            BinaryProtocolError: If the data is invalid
        """
        if len(data) < cls.HEADER_SIZE + cls.FVP_HEADER_SIZE:
            raise BinaryProtocolError(f"Data too short for FVP message: {len(data)} bytes")
            
        # Extract header
        protocol_id, version = cls.decode_header(data)
        if protocol_id != ProtocolID.FVP:
            raise BinaryProtocolError(f"Expected FVP protocol (2), got {protocol_id.value}")
            
        # Extract FVP header
        offset = cls.HEADER_SIZE
        frame_type, timestamp, data_length = struct.unpack(
            cls.FVP_HEADER_FORMAT, 
            data[offset:offset + cls.FVP_HEADER_SIZE]
        )
        offset += cls.FVP_HEADER_SIZE
        
        # Extract payload
        if len(data) < offset + data_length:
            raise BinaryProtocolError(f"Data too short for payload: expected {data_length} bytes")
            
        payload = data[offset:offset + data_length]
        
        return {
            "protocol_id": protocol_id.value,
            "version": version,
            "frame_type": frame_type,
            "timestamp": timestamp,
            "payload": payload
        }
    
    @classmethod
    def encode_fsmp(cls, channel_id: int, payload: bytes, timestamp: Optional[int] = None, version: int = 1) -> bytes:
        """
        Encode an FSMP message.
        
        Args:
            channel_id: Channel ID (0-65535)
            payload: Message payload
            timestamp: Timestamp in milliseconds (default: current time)
            version: Protocol version
            
        Returns:
            Encoded FSMP message
        """
        header = cls.encode_header(ProtocolID.FSMP, version)
        
        # Use current time if timestamp not provided
        if timestamp is None:
            timestamp = int(time.time() * 1000)
            
        data_length = len(payload)
        fsmp_header = struct.pack(cls.FSMP_HEADER_FORMAT, channel_id, timestamp, data_length)
        return header + fsmp_header + payload
    
    @classmethod
    def decode_fsmp(cls, data: bytes) -> Dict[str, Any]:
        """
        Decode an FSMP message.
        
        Args:
            data: Binary FSMP message
            
        Returns:
            Decoded message as a dictionary
            
        Raises:
            BinaryProtocolError: If the data is invalid
        """
        if len(data) < cls.HEADER_SIZE + cls.FSMP_HEADER_SIZE:
            raise BinaryProtocolError(f"Data too short for FSMP message: {len(data)} bytes")
            
        # Extract header
        protocol_id, version = cls.decode_header(data)
        if protocol_id != ProtocolID.FSMP:
            raise BinaryProtocolError(f"Expected FSMP protocol (3), got {protocol_id.value}")
            
        # Extract FSMP header
        offset = cls.HEADER_SIZE
        channel_id, timestamp, data_length = struct.unpack(
            cls.FSMP_HEADER_FORMAT, 
            data[offset:offset + cls.FSMP_HEADER_SIZE]
        )
        offset += cls.FSMP_HEADER_SIZE
        
        # Extract payload
        if len(data) < offset + data_length:
            raise BinaryProtocolError(f"Data too short for payload: expected {data_length} bytes")
            
        payload = data[offset:offset + data_length]
        
        return {
            "protocol_id": protocol_id.value,
            "version": version,
            "channel_id": channel_id,
            "timestamp": timestamp,
            "payload": payload
        }
    
    @classmethod
    def encode(cls, protocol_id: ProtocolID, message: Dict[str, Any], version: int = 1) -> bytes:
        """
        Encode a message for the specified protocol.
        
        Args:
            protocol_id: Protocol identifier
            message: Message to encode
            version: Protocol version
            
        Returns:
            Encoded binary message
            
        Raises:
            BinaryProtocolError: If the protocol is not supported
        """
        if protocol_id == ProtocolID.FCP:
            command_type = message.get("command_type", 0)
            payload = message.get("payload", b"")
            return cls.encode_fcp(command_type, payload, version)
            
        elif protocol_id == ProtocolID.FVP:
            frame_type = message.get("frame_type", 0)
            payload = message.get("payload", b"")
            timestamp = message.get("timestamp")
            return cls.encode_fvp(frame_type, payload, timestamp, version)
            
        elif protocol_id == ProtocolID.FSMP:
            channel_id = message.get("channel_id", 0)
            payload = message.get("payload", b"")
            timestamp = message.get("timestamp")
            return cls.encode_fsmp(channel_id, payload, timestamp, version)
            
        else:
            raise BinaryProtocolError(f"Unsupported protocol: {protocol_id}")
    
    @classmethod
    def decode(cls, data: bytes) -> Dict[str, Any]:
        """
        Decode a binary message.
        
        Args:
            data: Binary message
            
        Returns:
            Decoded message as a dictionary
            
        Raises:
            BinaryProtocolError: If the data is invalid
        """
        if len(data) < cls.HEADER_SIZE:
            raise BinaryProtocolError(f"Data too short for header: {len(data)} bytes")
            
        protocol_id, _ = cls.decode_header(data)
        
        if protocol_id == ProtocolID.FCP:
            return cls.decode_fcp(data)
            
        elif protocol_id == ProtocolID.FVP:
            return cls.decode_fvp(data)
            
        elif protocol_id == ProtocolID.FSMP:
            return cls.decode_fsmp(data)
            
        else:
            raise BinaryProtocolError(f"Unsupported protocol: {protocol_id}") 