"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.

LatestOnlyWriter - Python writer for LatestOnlySharedSlot format.
Compatible with Rust ShmReader in feagi-burst-engine.
"""

import os
import mmap
import struct
import time
from pathlib import Path
from typing import Optional

class LatestOnlyWriter:
    """Write sensory data to SHM using LatestOnlySharedSlot format.
    
    Compatible with Rust ShmReader (magic: FEAGILAT).
    
    Header format (256 bytes):
        magic: 8 bytes (b"FEAGILAT")
        version: u32 (4 bytes)
        max_payload_size: u32 (4 bytes)
        writer_pid: u32 (4 bytes)
        timestamp_ns: u64 (8 bytes)
        sequence: u64 (8 bytes)
        payload_size: u32 (4 bytes)
        reserved: u32 (4 bytes)
        padding: 212 bytes (unused)
    
    Followed by payload data.
    """
    
    MAGIC = b"FEAGILAT"
    VERSION = 1
    HEADER_SIZE = 256
    HEADER_FMT = "<8sIIIQQII212s"  # Matches Rust ShmHeader
    DEFAULT_MAX_PAYLOAD = 16 * 1024 * 1024  # 16MB default
    
    def __init__(self, path: Path, max_payload_size: int = DEFAULT_MAX_PAYLOAD):
        """Initialize LatestOnlyWriter.
        
        Args:
            path: Path to SHM file
            max_payload_size: Maximum payload size in bytes (default: 16MB)
        """
        self.path = Path(path)
        self.max_payload_size = max_payload_size
        self._fd: Optional[int] = None
        self._mm: Optional[mmap.mmap] = None
        self._sequence = 0
        self._pid = os.getpid()
        self._open()
    
    def _open(self) -> None:
        """Create and initialize SHM file."""
        import logging
        logger = logging.getLogger(__name__)
        
        # Create parent directory if needed
        self.path.parent.mkdir(parents=True, exist_ok=True)
        
        # Check if file already exists with correct size (FEAGI may have pre-created it)
        total_size = self.HEADER_SIZE + self.max_payload_size
        file_exists = self.path.exists()
        existing_size = os.path.getsize(str(self.path)) if file_exists else 0
        
        # Open/create file
        flags = os.O_RDWR | os.O_CREAT
        self._fd = os.open(str(self.path), flags, 0o600)
        
        # Only truncate if file doesn't exist or has wrong size
        # This preserves Rust reader's mmap if FEAGI pre-created the file
        if not file_exists or existing_size != total_size:
            os.ftruncate(self._fd, total_size)
            logger.info(f"📝 [LATEST-ONLY-WRITER] Truncated SHM file to {total_size} bytes (was {existing_size})")
        else:
            logger.info(f"📝 [LATEST-ONLY-WRITER] Reusing existing SHM file ({total_size} bytes) - preserving Rust mmap")
        
        # Memory map
        self._mm = mmap.mmap(self._fd, total_size, access=mmap.ACCESS_WRITE)
        
        # Write initial header (all zeros except magic/version/max_payload/pid)
        header = struct.pack(
            self.HEADER_FMT,
            self.MAGIC,
            self.VERSION,
            self.max_payload_size,
            self._pid,
            0,  # timestamp_ns
            0,  # sequence
            0,  # payload_size
            0,  # reserved
            b'\x00' * 212  # padding
        )
        self._mm[0:self.HEADER_SIZE] = header
        self._mm.flush()
        # Ensure file metadata (size) is visible to other processes
        os.fsync(self._fd)
        
        # Verify file was created properly
        actual_size = os.path.getsize(str(self.path))
        logger.info(f"📝 [LATEST-ONLY-WRITER] Initialized SHM: path={self.path}, size={actual_size} bytes (expected {total_size})")
    
    def write(self, data: bytes) -> bool:
        """Write data to SHM slot.
        
        Args:
            data: Payload bytes to write
            
        Returns:
            True if write succeeded, False if payload too large
        """
        if not self._mm:
            return False
            
        payload_size = len(data)
        if payload_size > self.max_payload_size:
            return False
        
        # Get timestamp
        timestamp_ns = time.time_ns()
        self._sequence += 1
        
        # Write header
        header = struct.pack(
            self.HEADER_FMT,
            self.MAGIC,
            self.VERSION,
            self.max_payload_size,
            self._pid,
            timestamp_ns,
            self._sequence,
            payload_size,
            0,  # reserved
            b'\x00' * 212  # padding
        )
        
        # Write header + payload atomically
        self._mm[0:self.HEADER_SIZE] = header
        self._mm[self.HEADER_SIZE:self.HEADER_SIZE + payload_size] = data
        self._mm.flush()
        
        return True
    
    def close(self) -> None:
        """Close SHM file."""
        if self._mm:
            self._mm.close()
            self._mm = None
        
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
    
    def __del__(self):
        self.close()

