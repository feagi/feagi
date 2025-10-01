"""Latest-Only Shared Slot for Neural Data

Replacement for ring buffers that eliminates temporal pattern replay bugs
by maintaining only the most recent data. Designed specifically for neural
simulation where stale data is harmful rather than useful.

Key Features:
- Zero-copy shared memory access
- Atomic write operations with timestamps  
- Automatic stale data rejection
- Agent lifecycle coupling
- Neural simulation temporal consistency
"""

import mmap
import os
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, NamedTuple
import threading

__all__ = [
    "LatestOnlySharedSlot",
    "LatestOnlyWriter", 
    "LatestOnlyReader",
    "SlotData",
    "LatestOnlyError",
]


class LatestOnlyError(Exception):
    """Latest-only slot operation error."""
    pass


class SlotData(NamedTuple):
    """Data read from a latest-only slot."""
    data: bytes
    timestamp_ns: int
    sequence: int
    age_ms: float


@dataclass
class SlotStats:
    """Statistics for latest-only slot performance monitoring."""
    total_writes: int = 0
    total_reads: int = 0  
    stale_data_rejections: int = 0
    overwrites: int = 0


class LatestOnlySharedSlot:
    """Shared memory slot that maintains only the latest data.
    
    Unlike ring buffers, this structure:
    - Overwrites old data atomically
    - Includes timestamps for stale data detection
    - Eliminates temporal replay bugs
    - Optimized for neural simulation requirements
    
    Memory Layout:
    Header (256 bytes): magic, version, max_payload_size, writer_pid, timestamp_ns, sequence, payload_size, padding
    Payload: Variable length data up to max_payload_size
    """
    
    MAGIC = b"FEAGILAT"  # "FEAGI Latest"
    VERSION = 1
    HEADER_SIZE = 256
    HEADER_FORMAT = "<8sIIIQQII192s"  # magic, version, max_payload_size, writer_pid, timestamp_ns, sequence, payload_size, reserved, padding
    
    def __init__(self, path: Path, max_payload_size: int = 2 * 1024 * 1024):
        """Initialize latest-only shared slot.
        
        Args:
            path: Path to shared memory file
            max_payload_size: Maximum payload size in bytes
        """
        self.path = Path(path)
        self.max_payload_size = max_payload_size
        self.total_size = self.HEADER_SIZE + max_payload_size
        self._fd = None
        self._mm = None
        self._lock = threading.RLock()
        self.stats = SlotStats()
        
    def _ensure_initialized(self) -> None:
        """Ensure the shared memory is initialized."""
        if self._mm is not None:
            return
            
        # Create directory if needed
        self.path.parent.mkdir(parents=True, exist_ok=True)
        
        # Open/create file
        self._fd = os.open(str(self.path), os.O_CREAT | os.O_RDWR, 0o600)
        
        # Resize to required size
        current_size = os.fstat(self._fd).st_size
        if current_size < self.total_size:
            os.ftruncate(self._fd, self.total_size)
            
        # Memory map
        self._mm = mmap.mmap(self._fd, self.total_size)
        
        # Initialize header if needed
        if current_size < self.HEADER_SIZE:
            self._write_header(0, 0, 0, 0)
            
    def _write_header(self, writer_pid: int, timestamp_ns: int, sequence: int, payload_size: int) -> None:
        """Write header atomically."""
        header = struct.pack(
            self.HEADER_FORMAT,
            self.MAGIC,
            self.VERSION, 
            self.max_payload_size,
            writer_pid,
            timestamp_ns,
            sequence,
            payload_size,
            0,  # reserved
            b'\x00' * 192  # padding
        )
        self._mm.seek(0)
        self._mm.write(header)
        self._mm.flush()
        
    def _read_header(self) -> tuple:
        """Read and parse header."""
        self._mm.seek(0)
        header_data = self._mm.read(self.HEADER_SIZE)
        if len(header_data) < self.HEADER_SIZE:
            raise LatestOnlyError("Incomplete header read")
            
        header = struct.unpack(self.HEADER_FORMAT, header_data)
        magic, version, max_payload_size, writer_pid, timestamp_ns, sequence, payload_size = header[:7]
        
        if magic != self.MAGIC:
            raise LatestOnlyError(f"Invalid magic: {magic}")
        if version != self.VERSION:
            raise LatestOnlyError(f"Unsupported version: {version}")
            
        return writer_pid, timestamp_ns, sequence, payload_size
        
    def close(self) -> None:
        """Close and cleanup resources."""
        with self._lock:
            if self._mm:
                try:
                    self._mm.close()
                except Exception:
                    pass
                self._mm = None
                
            if self._fd:
                try:
                    os.close(self._fd)
                except Exception:
                    pass
                self._fd = None
                
    def __enter__(self):
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


class LatestOnlyWriter:
    """Writer for latest-only shared slot."""
    
    def __init__(self, slot: LatestOnlySharedSlot):
        self.slot = slot
        self._sequence = 0
        self._writer_pid = os.getpid()
        
    def write_latest(self, data: bytes) -> bool:
        """Write latest data, overwriting any previous data.
        
        Args:
            data: Payload data to write
            
        Returns:
            True if write successful, False otherwise
        """
        if len(data) > self.slot.max_payload_size:
            return False
            
        try:
            with self.slot._lock:
                self.slot._ensure_initialized()
                
                # Increment sequence for this write
                self._sequence += 1
                timestamp_ns = time.time_ns()
                
                # Write payload first
                self.slot._mm.seek(self.slot.HEADER_SIZE)
                self.slot._mm.write(data)
                
                # Then update header atomically (makes data visible)
                self.slot._write_header(
                    self._writer_pid,
                    timestamp_ns, 
                    self._sequence,
                    len(data)
                )
                
                # Update stats
                if self._sequence > 1:
                    self.slot.stats.overwrites += 1
                self.slot.stats.total_writes += 1
                
                return True
                
        except Exception as e:
            # Don't log exceptions in hot path - just return failure
            return False


class LatestOnlyReader:
    """Reader for latest-only shared slot."""
    
    def __init__(self, slot: LatestOnlySharedSlot, max_age_ms: float = 100.0):
        self.slot = slot
        self.max_age_ms = max_age_ms  # Maximum age before data is considered stale
        self._last_sequence = 0
        
    def read_latest(self) -> Optional[SlotData]:
        """Read latest data if newer than last read and not stale.
        
        Returns:
            SlotData if new data available, None if no new data or stale
        """
        try:
            with self.slot._lock:
                self.slot._ensure_initialized()
                
                # Read header
                writer_pid, timestamp_ns, sequence, payload_size = self.slot._read_header()
                
                # Check if this is newer data
                if sequence <= self._last_sequence:
                    return None  # No new data
                    
                # Check if data is too stale
                current_time_ns = time.time_ns()
                age_ms = (current_time_ns - timestamp_ns) / 1_000_000
                
                if age_ms > self.max_age_ms:
                    self.slot.stats.stale_data_rejections += 1
                    self._last_sequence = sequence  # Mark as processed to avoid re-checking
                    return None  # Too stale
                    
                # Check payload size validity
                if payload_size <= 0 or payload_size > self.slot.max_payload_size:
                    return None  # Invalid payload
                    
                # Read payload
                self.slot._mm.seek(self.slot.HEADER_SIZE)
                payload_data = self.slot._mm.read(payload_size)
                
                if len(payload_data) != payload_size:
                    return None  # Incomplete read
                    
                # Update tracking
                self._last_sequence = sequence
                self.slot.stats.total_reads += 1
                
                return SlotData(
                    data=bytes(payload_data),
                    timestamp_ns=timestamp_ns,
                    sequence=sequence,
                    age_ms=age_ms
                )
                
        except Exception:
            return None  # Don't log exceptions in hot path
            
    def reset_sequence(self) -> None:
        """Reset sequence tracking (use with caution - may cause re-reads)."""
        self._last_sequence = 0


def create_agent_slot_path(base_dir: Path, agent_id: str, slot_type: str) -> Path:
    """Create standardized path for agent shared memory slot.
    
    Args:
        base_dir: Base directory for shared memory files
        agent_id: Agent identifier
        slot_type: Type of slot (sensory, motor, etc.)
        
    Returns:
        Path to shared memory slot file
    """
    return base_dir / f"agent_{agent_id}_{slot_type}.slot"


def cleanup_agent_slots(base_dir: Path, agent_id: str) -> int:
    """Cleanup all shared memory slots for an agent.
    
    Args:
        base_dir: Base directory for shared memory files
        agent_id: Agent identifier to cleanup
        
    Returns:
        Number of slot files removed
    """
    removed_count = 0
    try:
        pattern = f"agent_{agent_id}_*.slot"
        for slot_file in base_dir.glob(pattern):
            try:
                slot_file.unlink()
                removed_count += 1
            except Exception:
                pass  # File may already be gone
    except Exception:
        pass
    return removed_count
