"""
LZ4 decompression utilities for FEAGI connector.

ARCHITECTURE: Bridge operates in PASSTHROUGH mode - clients handle decompression

This module provides helpers for decompressing LZ4-compressed data received
from FEAGI via the bridge's passthrough architecture.
"""

import logging
from typing import Optional

logger = logging.getLogger(__name__)

# Try to import feagi_rust_py_libs for high-performance LZ4 decompression
try:
    import feagi_rust_py_libs as frpl
    HAS_RUST_DECOMPRESSION = True
    logger.debug("Rust-based LZ4 decompression available")
except ImportError:
    HAS_RUST_DECOMPRESSION = False
    logger.warning("feagi_rust_py_libs not available - LZ4 decompression disabled")
    
    # Try fallback to Python lz4 library
    try:
        import lz4.block
        HAS_PYTHON_LZ4 = True
        logger.info("Using Python lz4 library for decompression (slower than Rust)")
    except ImportError:
        HAS_PYTHON_LZ4 = False
        logger.error("No LZ4 decompression available - install feagi_rust_py_libs or python-lz4")


def decompress_if_needed(data: bytes) -> Optional[bytes]:
    """
    Decompress data if it's LZ4 compressed, otherwise return as-is.
    
    ARCHITECTURE: FEAGI PNS → LZ4 compress → ZMQ → Bridge PASSTHROUGH → Client DECOMPRESS
    
    This function automatically detects LZ4-compressed data (magic header 0x04)
    and decompresses it. Uncompressed data is returned unchanged.
    
    Args:
        data: Bytes that may or may not be compressed
        
    Returns:
        Decompressed bytes, or None if decompression failed
        
    Example:
        >>> raw_data = await viz_client.receive_visualization_data()
        >>> decompressed = decompress_if_needed(raw_data)
        >>> if decompressed:
        ...     container = frpl.data_serialization.FeagiByteContainer()
        ...     container.load_bytes_and_verify(decompressed)
    """
    if not data:
        return data
    
    # Check if data is LZ4 compressed (magic header 0x04)
    if len(data) > 0 and data[0] == 0x04:
        # Data is compressed - decompress it
        if HAS_RUST_DECOMPRESSION:
            try:
                return frpl.compression.decompress_lz4(data)
            except Exception as e:
                logger.error(f"Rust LZ4 decompression failed: {e}")
                return None
        elif HAS_PYTHON_LZ4:
            try:
                return lz4.block.decompress(data)
            except Exception as e:
                logger.error(f"Python LZ4 decompression failed: {e}")
                return None
        else:
            logger.error("Cannot decompress LZ4 data - no decompression library available")
            return None
    else:
        # Data is not compressed - return as-is
        return data


def is_lz4_compressed(data: bytes) -> bool:
    """
    Check if data is LZ4 compressed by examining magic header.
    
    LZ4 compressed data starts with magic number 0x04.
    
    Args:
        data: Bytes to check
        
    Returns:
        True if data appears to be LZ4 compressed
    """
    if HAS_RUST_DECOMPRESSION:
        return frpl.compression.is_lz4_compressed(data)
    else:
        # Fallback: manual check
        return len(data) > 0 and data[0] == 0x04


def decompress_lz4(compressed_data: bytes) -> Optional[bytes]:
    """
    Decompress LZ4-compressed data.
    
    Args:
        compressed_data: LZ4-compressed bytes
        
    Returns:
        Decompressed bytes, or None if decompression failed
    """
    if not compressed_data:
        return None
    
    if HAS_RUST_DECOMPRESSION:
        try:
            return frpl.compression.decompress_lz4(compressed_data)
        except Exception as e:
            logger.error(f"Rust LZ4 decompression failed: {e}")
            return None
    elif HAS_PYTHON_LZ4:
        try:
            return lz4.block.decompress(compressed_data)
        except Exception as e:
            logger.error(f"Python LZ4 decompression failed: {e}")
            return None
    else:
        logger.error("Cannot decompress LZ4 data - no decompression library available")
        return None

