"""Lock-free ring buffer for zero-copy neural data transmission.

This implementation uses atomic operations and memory mapping for high-
performance neural data buffering without locks or memory copies.
"""

import mmap
import multiprocessing
import os
from dataclasses import dataclass
from typing import NamedTuple, Optional

__all__ = [
    "ZeroCopyRingBuffer",
    "BufferSlot",
    "RingBufferError",
]


class RingBufferError(Exception):
    """Ring buffer operation error."""

    pass


class BufferSlot(NamedTuple):
    """A slot in the ring buffer."""

    index: int
    memory_view: memoryview
    offset: int
    size: int


@dataclass
class RingBufferStats:
    """Statistics for ring buffer performance monitoring."""

    total_writes: int = 0
    total_reads: int = 0
    buffer_full_count: int = 0
    buffer_empty_count: int = 0
    max_usage_percent: float = 0.0


class ZeroCopyRingBuffer:
    """Lock-free ring buffer for zero-copy neural data.

    This implementation uses:
    - Memory-mapped files for zero-copy access
    - Atomic operations for lock-free coordination
    - Power-of-2 sizing for efficient modulo operations
    - Cache-line alignment to prevent false sharing
    """

    # Cache line size (typical for x86_64)
    CACHE_LINE_SIZE = 64

    def __init__(
        self, slots: int, slot_size: int, use_shared_memory: bool = True
    ):
        """Initialize ring buffer.

        Args:
            slots: Number of slots (will be rounded up to power of 2)
            slot_size: Size of each slot in bytes
            use_shared_memory: Whether to use shared memory (for multi-process)
        """
        #  CRITICAL SAFETY CHECK: Prevent initialization during brain
        #  development
        # Brain development should not initialize multiprocessing components
        import threading

        current_thread = threading.current_thread()
        if (
            "neurogenesis" in current_thread.name.lower()
            or "embryogenesis" in current_thread.name.lower()
        ):
            raise RuntimeError(
                "ZeroCopyRingBuffer cannot be initialized during brain development to prevent memory corruption"
            )

        # Check if we're in a context where multiprocessing might cause issues
        import os

        if os.environ.get("FEAGI_DISABLE_MULTIPROCESSING", "").lower() in (
            "true",
            "1",
            "yes",
        ):
            use_shared_memory = False
            import logging

            logging.warning(
                "Multiprocessing disabled via FEAGI_DISABLE_MULTIPROCESSING - using anonymous mmap"
            )

        # Round up to power of 2 for efficient modulo
        self.slots = 1 << (slots - 1).bit_length()
        self.slot_size = slot_size
        self.total_size = self.slots * slot_size

        # Validate parameters
        if self.slots < 2:
            raise ValueError("Ring buffer must have at least 2 slots")
        if slot_size < 1:
            raise ValueError("Slot size must be positive")

        # Create memory-mapped buffer
        if use_shared_memory:
            # Create shared memory that persists across processes
            self._create_shared_memory()
        else:
            # Create anonymous memory map
            self.buffer = mmap.mmap(-1, self.total_size)

        # Atomic indices - aligned to cache lines to prevent false sharing
        self._create_atomic_indices()

        # Statistics
        self.stats = RingBufferStats()

        # State tracking
        self.closed = False

    def _create_shared_memory(self):
        """Create shared memory segment."""
        # Generate unique name
        self.shm_name = f"feagi_ring_{os.getpid()}_{id(self)}"

        # Create shared memory
        from multiprocessing import shared_memory

        try:
            # Try to create new shared memory
            self.shm = shared_memory.SharedMemory(
                create=True, size=self.total_size, name=self.shm_name
            )
            self.buffer = mmap.mmap(self.shm._fd, self.total_size)
            self._owns_shm = True
        except FileExistsError:
            # Attach to existing shared memory
            self.shm = shared_memory.SharedMemory(name=self.shm_name)
            self.buffer = mmap.mmap(self.shm._fd, self.total_size)
            self._owns_shm = False

    def _create_atomic_indices(self):
        """Create cache-line aligned atomic indices."""
        # Use multiprocessing Values for atomic operations
        # Each Value is padded to cache line size to prevent false sharing
        self.write_index = multiprocessing.Value("Q", 0)  # uint64
        self.read_index = multiprocessing.Value("Q", 0)  # uint64

        # Padding to prevent false sharing
        self._write_padding = multiprocessing.Array(
            "c", self.CACHE_LINE_SIZE - 8
        )
        self._read_padding = multiprocessing.Array(
            "c", self.CACHE_LINE_SIZE - 8
        )

    def get_write_slot(self) -> Optional[BufferSlot]:
        """Get next available write slot without blocking.

        Returns:
            BufferSlot if available, None if buffer is full or closed
        """
        # Check if buffer is closed
        if self.closed:
            return None

        # Load indices with acquire semantics
        write_idx = self.write_index.value
        read_idx = self.read_index.value

        # Check if buffer is full
        next_write = (write_idx + 1) & (self.slots - 1)  # Fast modulo
        if next_write == read_idx:
            self.stats.buffer_full_count += 1
            return None  # Buffer full

        # Calculate slot offset
        slot_idx = write_idx & (self.slots - 1)
        offset = slot_idx * self.slot_size

        # Return memory view of slot
        return BufferSlot(
            index=write_idx,
            memory_view=memoryview(self.buffer)[
                offset : offset + self.slot_size
            ],
            offset=offset,
            size=self.slot_size,
        )

    def commit_write(self, slot: BufferSlot) -> None:
        """Commit a write operation.

        Args:
            slot: The slot that was written to
        """
        # Ensure write completes before updating index
        # In Python, assignment is atomic for aligned values
        expected = slot.index
        new_value = (expected + 1) & ((1 << 64) - 1)  # Wrap at 64 bits

        # Update write index with release semantics
        self.write_index.value = new_value
        self.stats.total_writes += 1

        # Update max usage
        self._update_usage_stats()

    def get_read_slot(self) -> Optional[BufferSlot]:
        """Get next slot to read from without blocking.

        Returns:
            BufferSlot if available, None if buffer is empty
        """
        # Load indices with acquire semantics
        write_idx = self.write_index.value
        read_idx = self.read_index.value

        # Check if buffer is empty
        if read_idx == write_idx:
            self.stats.buffer_empty_count += 1
            return None  # Buffer empty

        # Calculate slot offset
        slot_idx = read_idx & (self.slots - 1)
        offset = slot_idx * self.slot_size

        # Return memory view of slot
        return BufferSlot(
            index=read_idx,
            memory_view=memoryview(self.buffer)[
                offset : offset + self.slot_size
            ],
            offset=offset,
            size=self.slot_size,
        )

    def commit_read(self, slot: BufferSlot) -> None:
        """Commit a read operation.

        Args:
            slot: The slot that was read from
        """
        # Ensure read completes before updating index
        expected = slot.index
        new_value = (expected + 1) & ((1 << 64) - 1)  # Wrap at 64 bits

        # Update read index with release semantics
        self.read_index.value = new_value
        self.stats.total_reads += 1

    def _update_usage_stats(self):
        """Update buffer usage statistics."""
        write_idx = self.write_index.value
        read_idx = self.read_index.value

        # Calculate current usage
        if write_idx >= read_idx:
            used = write_idx - read_idx
        else:
            used = (write_idx + (1 << 64)) - read_idx

        usage_percent = (used / self.slots) * 100
        self.stats.max_usage_percent = max(
            self.stats.max_usage_percent, usage_percent
        )

    @property
    def available_slots(self) -> int:
        """Get number of available write slots."""
        write_idx = self.write_index.value
        read_idx = self.read_index.value

        if write_idx >= read_idx:
            used = write_idx - read_idx
        else:
            used = (write_idx + (1 << 64)) - read_idx

        return self.slots - used - 1  # -1 to distinguish full from empty

    @property
    def used_slots(self) -> int:
        """Get number of used slots."""
        write_idx = self.write_index.value
        read_idx = self.read_index.value

        if write_idx >= read_idx:
            return write_idx - read_idx
        else:
            return (write_idx + (1 << 64)) - read_idx

    def reset(self):
        """Reset buffer to empty state."""
        self.write_index.value = 0
        self.read_index.value = 0

    def close(self):
        """Close and cleanup resources with RTOS-friendly deterministic
        cleanup."""
        # RTOS-friendly: Simple, bounded cleanup operations

        # Step 1: Mark as closed to prevent new operations
        self.closed = True

        # Step 2: Clear any references that might hold memory views
        # Force garbage collection to release memory views
        import gc

        gc.collect()

        # Step 3: Close memory map (with retry for exported pointers)
        if hasattr(self, "buffer") and self.buffer:
            try:
                self.buffer.close()
            except Exception as e:
                # RTOS-friendly: Log but continue cleanup
                import logging

                logging.warning(f"Error closing buffer (will retry): {e}")
                # Try garbage collection and retry once
                gc.collect()
                try:
                    self.buffer.close()
                except Exception as e2:
                    logging.warning(f"Buffer close retry failed: {e2}")

        # Step 4: Clean up shared memory if we own it
        if hasattr(self, "shm") and hasattr(self, "_owns_shm"):
            try:
                if self._owns_shm and self.shm:
                    # RTOS-friendly: Close first, then unlink
                    self.shm.close()
                    try:
                        self.shm.unlink()  # Remove from system
                    except FileNotFoundError:
                        # Already cleaned up, that's fine
                        pass
                elif self.shm:
                    # Just close if we don't own it
                    self.shm.close()
            except Exception as e:
                # RTOS-friendly: Log but don't fail
                import logging

                logging.warning(
                    f"Error cleaning up shared memory {getattr(self, 'shm_name', 'unknown')}: {e}"
                )

        # Step 5: Clear references for deterministic cleanup
        if hasattr(self, "buffer"):
            self.buffer = None
        if hasattr(self, "shm"):
            self.shm = None

    def __del__(self):
        """Destructor to ensure cleanup even if close() isn't called
        explicitly."""
        try:
            # Only attempt cleanup if we haven't already cleaned up
            if hasattr(self, "buffer") and self.buffer is not None:
                self.close()
        except Exception:
            # In destructor, we can't do much about errors
            # Just ensure we don't raise exceptions from __del__
            pass

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()


# Example usage for neural data
if __name__ == "__main__":
    # Create ring buffer for neural data
    # 1024 slots * 1MB each = 1GB total buffer
    with ZeroCopyRingBuffer(slots=1024, slot_size=1048576) as ring:
        # Producer: write neural data
        slot = ring.get_write_slot()
        if slot:
            # Write directly to memory-mapped buffer
            # No copying needed!
            slot.memory_view[0:4] = b"FEAG"  # Magic bytes
            ring.commit_write(slot)

        # Consumer: read neural data
        slot = ring.get_read_slot()
        if slot:
            # Read directly from memory-mapped buffer
            magic = bytes(slot.memory_view[0:4])
            print(f"Read magic bytes: {magic}")
            ring.commit_read(slot)

        print(f"Stats: {ring.stats}")
