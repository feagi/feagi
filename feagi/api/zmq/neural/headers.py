"""
Fixed-size header structures for neural data transmission.

All headers are designed to be exactly 32 bytes for consistency and alignment.
These structures are compatible with C/Rust FFI for future migration.
"""

import struct
from dataclasses import dataclass
from typing import ClassVar, Optional

from .protocols import (
    NeuralProtocolID, 
    CompressionType, 
    NeuralPrecision,
    NEURAL_HEADER_SIZE,
    NEURAL_MAGIC
)

__all__ = [
    'NeuralDataHeader',
    'NeuralHeaderError',
    'parse_header',
    'create_header',
]


class NeuralHeaderError(Exception):
    """Error parsing or creating neural data headers."""
    pass


@dataclass
class NeuralDataHeader:
    """
    Fixed-size 32-byte header for neural data packets.
    
    Layout (32 bytes total):
    - magic: 4 bytes (b'FEAG')
    - protocol_id: 1 byte
    - version: 1 byte 
    - flags: 2 bytes
    - timestamp: 8 bytes (nanoseconds since epoch)
    - cortical_area_id: 4 bytes
    - neuron_count: 4 bytes
    - payload_size: 4 bytes
    - sequence_number: 4 bytes
    
    Flags (16 bits):
    - bits 0-2: compression type (3 bits)
    - bits 3-5: precision (3 bits) 
    - bits 6-8: priority (3 bits)
    - bit 9: is_delta (1 bit)
    - bit 10: has_coordinates (1 bit)
    - bits 11-15: reserved (5 bits)
    """
    
    # Header fields
    protocol_id: NeuralProtocolID
    version: int
    flags: int
    timestamp: int  # nanoseconds
    cortical_area_id: int
    neuron_count: int
    payload_size: int
    sequence_number: int
    
    # Struct format for packing/unpacking
    STRUCT_FORMAT: ClassVar[str] = '<4sBBHQIIII'  # Little-endian, 32 bytes
    
    def __post_init__(self):
        """Validate header fields."""
        if not (0 <= self.version <= 255):
            raise ValueError(f"Version must be 0-255, got {self.version}")
        if not (0 <= self.flags <= 65535):
            raise ValueError(f"Flags must be 0-65535, got {self.flags}")
        if self.neuron_count < 0:
            raise ValueError(f"Neuron count must be >= 0, got {self.neuron_count}")
        if self.payload_size < 0:
            raise ValueError(f"Payload size must be >= 0, got {self.payload_size}")
    
    @property
    def compression(self) -> CompressionType:
        """Extract compression type from flags."""
        return CompressionType((self.flags >> 0) & 0b111)
    
    @property
    def precision(self) -> NeuralPrecision:
        """Extract precision from flags."""
        return NeuralPrecision((self.flags >> 3) & 0b111)
    
    @property
    def priority(self) -> int:
        """Extract priority from flags (0-7)."""
        return (self.flags >> 6) & 0b111
    
    @property
    def is_delta(self) -> bool:
        """Check if this is a delta update."""
        return bool((self.flags >> 9) & 0b1)
    
    @property
    def has_coordinates(self) -> bool:
        """Check if payload includes neuron coordinates."""
        return bool((self.flags >> 10) & 0b1)
    
    def to_bytes(self) -> bytes:
        """Pack header to 32-byte binary format."""
        return struct.pack(
            self.STRUCT_FORMAT,
            NEURAL_MAGIC,
            self.protocol_id,
            self.version,
            self.flags,
            self.timestamp,
            self.cortical_area_id,
            self.neuron_count,
            self.payload_size,
            self.sequence_number
        )
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'NeuralDataHeader':
        """Unpack header from binary data."""
        if len(data) < NEURAL_HEADER_SIZE:
            raise NeuralHeaderError(
                f"Header must be {NEURAL_HEADER_SIZE} bytes, got {len(data)}"
            )
        
        try:
            (magic, protocol_id, version, flags, timestamp, 
             cortical_area_id, neuron_count, payload_size, 
             sequence_number) = struct.unpack(cls.STRUCT_FORMAT, data[:NEURAL_HEADER_SIZE])
        except struct.error as e:
            raise NeuralHeaderError(f"Failed to unpack header: {e}")
        
        if magic != NEURAL_MAGIC:
            raise NeuralHeaderError(
                f"Invalid magic bytes: expected {NEURAL_MAGIC!r}, got {magic!r}"
            )
        
        return cls(
            protocol_id=NeuralProtocolID(protocol_id),
            version=version,
            flags=flags,
            timestamp=timestamp,
            cortical_area_id=cortical_area_id,
            neuron_count=neuron_count,
            payload_size=payload_size,
            sequence_number=sequence_number
        )


def create_header(
    protocol_id: NeuralProtocolID,
    cortical_area_id: int,
    neuron_count: int,
    payload_size: int,
    *,
    timestamp: Optional[int] = None,
    sequence_number: int = 0,
    compression: CompressionType = CompressionType.NONE,
    precision: NeuralPrecision = NeuralPrecision.FLOAT32,
    priority: int = 2,  # Normal priority
    is_delta: bool = False,
    has_coordinates: bool = True,
    version: int = 1
) -> NeuralDataHeader:
    """
    Create a neural data header with sensible defaults.
    
    Args:
        protocol_id: Protocol type for the neural data
        cortical_area_id: ID of the cortical area
        neuron_count: Number of neurons in payload
        payload_size: Size of payload in bytes
        timestamp: Nanosecond timestamp (auto-generated if None)
        sequence_number: Sequence number for ordering
        compression: Compression type
        precision: Data precision
        priority: Priority level (0-7)
        is_delta: Whether this is a delta update
        has_coordinates: Whether coordinates are included
        version: Protocol version
    
    Returns:
        Configured NeuralDataHeader
    """
    if timestamp is None:
        import time
        timestamp = int(time.time() * 1_000_000_000)  # nanoseconds
    
    # Build flags field
    flags = 0
    flags |= (compression & 0b111) << 0
    flags |= (precision & 0b111) << 3
    flags |= (priority & 0b111) << 6
    flags |= (int(is_delta) & 0b1) << 9
    flags |= (int(has_coordinates) & 0b1) << 10
    
    return NeuralDataHeader(
        protocol_id=protocol_id,
        version=version,
        flags=flags,
        timestamp=timestamp,
        cortical_area_id=cortical_area_id,
        neuron_count=neuron_count,
        payload_size=payload_size,
        sequence_number=sequence_number
    )


def parse_header(data: bytes) -> tuple[NeuralDataHeader, memoryview]:
    """
    Parse header and return header object plus payload view.
    
    Args:
        data: Raw bytes containing header and payload
        
    Returns:
        Tuple of (header, payload_view)
    """
    header = NeuralDataHeader.from_bytes(data)
    
    # Create memory view of payload without copying
    payload_start = NEURAL_HEADER_SIZE
    payload_end = payload_start + header.payload_size
    
    if len(data) < payload_end:
        raise NeuralHeaderError(
            f"Data too short: expected {payload_end} bytes, got {len(data)}"
        )
    
    payload_view = memoryview(data)[payload_start:payload_end]
    
    return header, payload_view 