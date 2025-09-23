"""
Shared memory utilities for FEAGI agents.

Provides cross-platform, file-backed ring buffers for:
- RGB frames (uint8, RGB8) with a fixed 256-byte header for metadata
- Generic variable-length byte payloads (length-prefixed slots)

Design goals:
- OS-agnostic, mmap-based (no platform-specific shm APIs)
- Lock-free single-writer design with N slots (ring buffer)
- Stable binary header contracts for downstream readers (e.g., Godot/BV)
- Minimal dependencies; only numpy required for frame writers

@cursor:ffi-safe
"""

from __future__ import annotations

import mmap
import os
import struct
from pathlib import Path
from typing import Optional

import numpy as np


class SharedFrameWriter:
    """Cross-platform shared memory writer for RGB frames.

    Writes frames into a memory-mapped file ring buffer so external processes
    can read the latest frame without locks.

    File layout:
        - 256-byte header (little-endian)
        - N frame slots, each width*height*3 bytes (RGB8)

    Header format (struct pack fmt: '<8sIIIIIIQIQ'):
        magic[8] = b'FEAGIVID'
        version: u32 = 1
        width: u32
        height: u32
        channels: u32 = 3
        pixel_format: u32 = 1 (RGB8)
        num_slots: u32
        frame_stride: u64
        write_index: u32
        frame_seq: u64

    Notes
    -----
    - Single writer is assumed. Multiple readers may read concurrently.
    - Header is rewritten atomically-ish after each frame write to publish index/seq.
    - The mapping is recreated if frame dimensions change.
    """

    MAGIC = b"FEAGIVID"
    VERSION = 1
    HEADER_SIZE = 256
    HEADER_FMT = "<8sIIIIIIQIQ"

    def __init__(self, path: Optional[Path] = None, num_slots: int = 3) -> None:
        """Initialize the shared memory writer.

        Args:
            path: Optional path to the backing file. If None, a temp path is used.
            num_slots: Number of frame slots in the ring buffer.
        """
        self.path: Path = Path(path) if path else Path(os.path.expanduser("~")) / "feagi_video_shm--temp.bin"
        self.num_slots: int = max(1, int(num_slots))
        self._mm: Optional[mmap.mmap] = None
        self._fd: Optional[int] = None
        self._frame_stride: int = 0
        self._write_index: int = -1
        self._frame_seq: int = 0
        self._width: int = 0
        self._height: int = 0

    def open(self, width: int, height: int, channels: int = 3, pixel_format: int = 1) -> None:
        """Create or resize the mapping based on frame dimensions.

        Args:
            width: Frame width in pixels.
            height: Frame height in pixels.
            channels: Number of channels (must be 3 for RGB).
            pixel_format: Pixel format identifier (1 = RGB8).
        """
        self._width, self._height = int(width), int(height)
        self._frame_stride = self._width * self._height * int(channels)
        total_size = self.HEADER_SIZE + self._frame_stride * self.num_slots

        flags = os.O_RDWR | os.O_CREAT
        self._fd = os.open(str(self.path), flags, 0o600)
        os.ftruncate(self._fd, total_size)

        self._mm = mmap.mmap(self._fd, total_size, access=mmap.ACCESS_WRITE)

        header = struct.pack(
            self.HEADER_FMT,
            self.MAGIC,
            self.VERSION,
            self._width,
            self._height,
            3,
            pixel_format,
            self.num_slots,
            self._frame_stride,
            0,
            0,
        )
        self._mm.seek(0)
        self._mm.write(header)
        if len(header) < self.HEADER_SIZE:
            self._mm.write(b"\x00" * (self.HEADER_SIZE - len(header)))
        self._mm.flush()

    def _close_mapping(self) -> None:
        try:
            if self._mm is not None:
                try:
                    self._mm.flush()
                except Exception:
                    pass
                try:
                    self._mm.close()
                except Exception:
                    pass
                self._mm = None
        finally:
            if self._fd is not None:
                try:
                    os.close(self._fd)
                except Exception:
                    pass
                self._fd = None

    def ensure_open(self, width: int, height: int, channels: int = 3, pixel_format: int = 1) -> None:
        """Ensure mapping exists with given dimensions; reopen if dimensions changed."""
        if (
            self._mm is None
            or self._width != int(width)
            or self._height != int(height)
        ):
            # Close existing mapping without deleting the file
            self._close_mapping()
            # Reset indices
            self._write_index = -1
            self._frame_seq = 0
            # Reopen with new size
            self.open(width, height, channels=channels, pixel_format=pixel_format)

    def write_frame(self, rgb_frame: np.ndarray) -> None:
        """Write a single RGB frame into the ring buffer and publish it.

        Args:
            rgb_frame: A numpy array of shape (H, W, 3), dtype uint8, RGB order.
        """
        h, w, c = rgb_frame.shape
        self.ensure_open(w, h, channels=c, pixel_format=1)

        if rgb_frame.dtype != np.uint8 or rgb_frame.ndim != 3 or rgb_frame.shape[2] != 3:
            raise ValueError("SharedFrameWriter expects RGB uint8 frame (H,W,3)")

        self._write_index = (self._write_index + 1) % self.num_slots
        frame_offset = self.HEADER_SIZE + self._write_index * self._frame_stride

        self._mm.seek(frame_offset)
        self._mm.write(rgb_frame.tobytes(order="C"))

        self._frame_seq += 1
        header = struct.pack(
            self.HEADER_FMT,
            self.MAGIC,
            self.VERSION,
            self._width,
            self._height,
            3,
            1,
            self.num_slots,
            self._frame_stride,
            self._write_index,
            self._frame_seq,
        )
        self._mm.seek(0)
        self._mm.write(header)
        self._mm.flush()

    def close(self) -> None:
        """Close the mapping and remove the temp file if applicable."""
        try:
            self._close_mapping()
        finally:
            try:
                self.path.unlink(missing_ok=True)
            except Exception:
                pass


class ShmBytesWriter:
    """Minimal ring writer for variable-length byte payloads.

    Header: '<8sIIIQI' => magic, version, num_slots, slot_size, frame_seq, write_index
    Each slot: u32 length + data
    """

    MAGIC = b"FEAGIBIN"
    VERSION = 1
    HEADER_SIZE = 256
    HEADER_FMT = "<8sIIIQI"

    def __init__(self, path: Path, num_slots: int = 16, slot_size: int = 262144) -> None:
        self.path = Path(path)
        self.num_slots = int(num_slots)
        self.slot_size = int(slot_size)
        self._fd: Optional[int] = None
        self._mm: Optional[mmap.mmap] = None
        self._write_index: int = 0
        self._frame_seq: int = 0
        self._open()

    def _open(self) -> None:
        flags = os.O_RDWR | os.O_CREAT
        self._fd = os.open(str(self.path), flags, 0o600)
        total_size = self.HEADER_SIZE + self.num_slots * self.slot_size
        os.ftruncate(self._fd, total_size)
        self._mm = mmap.mmap(self._fd, total_size, access=mmap.ACCESS_WRITE)
        header = struct.pack(
            self.HEADER_FMT,
            self.MAGIC,
            self.VERSION,
            self.num_slots,
            self.slot_size,
            0,  # frame_seq
            0,  # write_index
        )
        self._mm.seek(0)
        self._mm.write(header)
        rem = self.HEADER_SIZE - len(header)
        if rem > 0:
            self._mm.write(b"\x00" * rem)

    def write(self, data: bytes) -> None:
        if not data:
            return
        if len(data) > (self.slot_size - 4):
            payload = data[: self.slot_size - 4]
        else:
            payload = data
        idx = self._write_index % self.num_slots
        slot_off = self.HEADER_SIZE + idx * self.slot_size
        self._mm.seek(slot_off)
        self._mm.write(struct.pack("<I", len(payload)))
        self._mm.write(payload)
        self._frame_seq += 1
        self._write_index += 1
        self._mm.seek(0)
        header = struct.pack(
            self.HEADER_FMT,
            self.MAGIC,
            self.VERSION,
            self.num_slots,
            self.slot_size,
            self._frame_seq,
            self._write_index,
        )
        self._mm.write(header)

    def close(self) -> None:
        try:
            if self._mm:
                self._mm.flush()
                self._mm.close()
        except Exception:
            pass
        if self._fd is not None:
            try:
                os.close(self._fd)
            except Exception:
                pass
        self._mm = None
        self._fd = None


class ShmBytesReader:
    """Minimal ring reader for variable-length byte payloads (pairs with ShmBytesWriter)."""

    MAGIC = b"FEAGIBIN"
    VERSION = 1
    HEADER_SIZE = 256
    HEADER_FMT = "<8sIIIQI"

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self._fd: Optional[int] = None
        self._mm: Optional[mmap.mmap] = None
        self.num_slots: int = 0
        self.slot_size: int = 0
        self._last_read_index: int = -1
        self._open()

    def _open(self) -> None:
        if not self.path.exists():
            raise FileNotFoundError(str(self.path))
        self._fd = os.open(str(self.path), os.O_RDONLY)
        size = os.path.getsize(self.path)
        if size < self.HEADER_SIZE:
            raise ValueError("SHM file too small for header")
        self._mm = mmap.mmap(self._fd, size, access=mmap.ACCESS_READ)
        self._mm.seek(0)
        header = self._mm.read(self.HEADER_SIZE)
        try:
            magic, version, num_slots, slot_size, frame_seq, write_index = struct.unpack(
                self.HEADER_FMT, header[: struct.calcsize(self.HEADER_FMT)]
            )
        except Exception as e:
            raise ValueError(f"Invalid header: {e}")
        # Accept both generic bytes ring and FEAGI motor ring headers
        if magic not in (self.MAGIC, b"FEAGIMOT"):
            raise ValueError("Invalid MAGIC")
        self.num_slots = int(num_slots)
        self.slot_size = int(slot_size)

    def read_latest(self) -> Optional[bytes]:
        if self._mm is None:
            return None
        self._mm.seek(0)
        header = self._mm.read(self.HEADER_SIZE)
        try:
            magic, version, num_slots, slot_size, frame_seq, write_index = struct.unpack(
                self.HEADER_FMT, header[: struct.calcsize(self.HEADER_FMT)]
            )
        except Exception:
            return None
        if self.num_slots <= 0 or self.slot_size <= 0:
            return None
        if write_index <= 0:
            return None
        idx = (write_index - 1) % self.num_slots
        if idx == self._last_read_index:
            return None
        slot_off = self.HEADER_SIZE + idx * self.slot_size
        if slot_off + 4 > self._mm.size():
            return None
        self._mm.seek(slot_off)
        (length,) = struct.unpack("<I", self._mm.read(4))
        if length <= 0 or length > (self.slot_size - 4):
            return None
        data = self._mm.read(length)
        self._last_read_index = idx
        return data

    def close(self) -> None:
        try:
            if self._mm:
                self._mm.close()
        except Exception:
            pass
        if self._fd is not None:
            try:
                os.close(self._fd)
            except Exception:
                pass
        self._mm = None
        self._fd = None


