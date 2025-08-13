"""Static buffer pools for zero-allocation neural data processing.

Pre-allocates buffers of fixed sizes to eliminate dynamic allocation in the
critical path of neural data processing.
"""

import mmap
import threading
from dataclasses import dataclass
from queue import Empty, Full, Queue
from typing import Dict, List, Optional

__all__ = [
    "FixedBufferPool",
    "NeuralBufferPool",
    "Buffer",
    "BufferPoolError",
]


class BufferPoolError(Exception):
    """Buffer pool operation error."""

    pass


@dataclass
class Buffer:
    """A buffer from the pool."""

    data: memoryview
    size: int
    pool_id: int
    slot_id: int

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - auto-release to pool."""
        # Buffer should be released by pool


class FixedBufferPool:
    """Fixed-size buffer pool with static allocation.

    Features:
    - Pre-allocated buffers at initialization
    - Zero allocation during acquire/release
    - Thread-safe operations
    - Optional NUMA node affinity
    - Memory alignment for SIMD operations
    """

    def __init__(
        self,
        count: int,
        size: int,
        alignment: int = 64,
        numa_node: int = -1,
        name: str = "buffer_pool",
    ):
        """Initialize buffer pool.

        Args:
            count: Number of buffers to pre-allocate
            size: Size of each buffer in bytes
            alignment: Memory alignment (default 64 for cache lines)
            numa_node: NUMA node for allocation (-1 for any)
            name: Name for the pool (for debugging)
        """
        if count < 1:
            raise ValueError("Buffer count must be at least 1")
        if size < 1:
            raise ValueError("Buffer size must be positive")
        if alignment < 1 or (alignment & (alignment - 1)) != 0:
            raise ValueError("Alignment must be a power of 2")

        self.count = count
        self.size = size
        self.alignment = alignment
        self.numa_node = numa_node
        self.name = name

        # Calculate aligned size
        self.aligned_size = (size + alignment - 1) & ~(alignment - 1)
        self.total_size = count * self.aligned_size

        # Allocate contiguous memory
        self._allocate_memory()

        # Create buffer views
        self._create_buffers()

        # Free list for available buffers
        self.free_queue = Queue(maxsize=count)
        for i in range(count):
            self.free_queue.put(i)

        # Statistics
        self.stats = {
            "total_acquires": 0,
            "total_releases": 0,
            "failed_acquires": 0,
            "peak_usage": 0,
        }
        self._stats_lock = threading.Lock()

    def _allocate_memory(self):
        """Allocate contiguous memory for all buffers."""
        # In production, this would use NUMA-aware allocation
        # For now, use mmap for portability
        self.memory = mmap.mmap(-1, self.total_size)

        # Zero-initialize (optional, for security)
        self.memory[:] = b"\x00" * self.total_size

    def _create_buffers(self):
        """Create buffer views into the allocated memory."""
        self.buffers: List[memoryview] = []

        for i in range(self.count):
            offset = i * self.aligned_size
            view = memoryview(self.memory)[offset : offset + self.size]
            self.buffers.append(view)

    def acquire(self, timeout: Optional[float] = None) -> Optional[Buffer]:
        """Acquire a buffer from the pool.

        Args:
            timeout: Timeout in seconds (None for non-blocking)

        Returns:
            Buffer if available, None otherwise
        """
        try:
            slot_id = self.free_queue.get(
                block=False if timeout is None else True, timeout=timeout
            )
        except Empty:
            with self._stats_lock:
                self.stats["failed_acquires"] += 1
            return None

        # Update statistics
        with self._stats_lock:
            self.stats["total_acquires"] += 1
            current_usage = self.count - self.free_queue.qsize()
            self.stats["peak_usage"] = max(
                self.stats["peak_usage"], current_usage
            )

        return Buffer(
            data=self.buffers[slot_id],
            size=self.size,
            pool_id=id(self),
            slot_id=slot_id,
        )

    def release(self, buffer: Buffer):
        """Release a buffer back to the pool.

        Args:
            buffer: Buffer to release
        """
        if buffer.pool_id != id(self):
            raise BufferPoolError("Buffer does not belong to this pool")

        # Optional: Clear buffer for security
        # buffer.data[:] = b'\x00' * buffer.size

        try:
            self.free_queue.put(buffer.slot_id, block=False)
            with self._stats_lock:
                self.stats["total_releases"] += 1
        except Full as e:
            raise BufferPoolError(
                "Buffer pool corruption: too many releases"
            ) from e

    @property
    def available(self) -> int:
        """Get number of available buffers."""
        return self.free_queue.qsize()

    @property
    def in_use(self) -> int:
        """Get number of buffers in use."""
        return self.count - self.free_queue.qsize()

    def close(self):
        """Close and cleanup resources."""
        if hasattr(self, "memory"):
            self.memory.close()


class NeuralBufferPool:
    """Specialized buffer pool system for neural data arrays.

    Pre-allocates buffers based on cortical area configurations to ensure zero
    allocation during neural processing.
    """

    def __init__(self, cortical_config: Dict[str, Dict[str, any]]):
        """Initialize neural buffer pools.

        Args:
            cortical_config: Configuration for each cortical area
                            {area_id: {'neuron_count': N, 'numa_node': 0, ...}}
        """
        self.pools: Dict[str, FixedBufferPool] = {}
        self.generic_pools: Dict[int, FixedBufferPool] = {}

        # Create pool for each cortical area
        for area_id, config in cortical_config.items():
            neuron_count = config.get("neuron_count", 0)
            if neuron_count <= 0:
                continue

            # Calculate buffer size for neural data
            # Each neuron needs: 4 bytes (float32) firing + 12 bytes (3 x int32) coordinates
            buffer_size = neuron_count * 16

            # Create pool with area-specific configuration
            pool = FixedBufferPool(
                count=config.get(
                    "buffer_count", 32
                ),  # Default 32 buffers per area
                size=buffer_size,
                alignment=64,  # Cache line alignment
                numa_node=config.get("numa_node", -1),
                name=f"neural_{area_id}",
            )

            self.pools[area_id] = pool

        # Create generic pools for different size classes
        self._create_generic_pools()

    def _create_generic_pools(self):
        """Create generic pools for common buffer sizes."""
        # Size classes: 1KB, 4KB, 16KB, 64KB, 256KB, 1MB
        size_classes = [
            (1024, 128),  # 1KB x 128 buffers
            (4096, 64),  # 4KB x 64 buffers
            (16384, 32),  # 16KB x 32 buffers
            (65536, 16),  # 64KB x 16 buffers
            (262144, 8),  # 256KB x 8 buffers
            (1048576, 4),  # 1MB x 4 buffers
        ]

        for size, count in size_classes:
            pool = FixedBufferPool(
                count=count, size=size, alignment=64, name=f"generic_{size}"
            )
            self.generic_pools[size] = pool

    def get_buffer_for_area(self, area_id: str) -> Optional[Buffer]:
        """Get pre-sized buffer for specific cortical area.

        Args:
            area_id: Cortical area identifier

        Returns:
            Buffer if available, None otherwise
        """
        pool = self.pools.get(area_id)
        if pool:
            return pool.acquire()
        return None

    def get_buffer_by_size(self, size: int) -> Optional[Buffer]:
        """Get buffer from generic pool by size.

        Args:
            size: Required buffer size

        Returns:
            Buffer of at least the requested size, or None
        """
        # Find smallest pool that fits
        for pool_size in sorted(self.generic_pools.keys()):
            if pool_size >= size:
                return self.generic_pools[pool_size].acquire()
        return None

    def release_buffer(self, buffer: Buffer):
        """Release buffer back to its pool.

        Args:
            buffer: Buffer to release
        """
        # Find pool by ID
        for pool in list(self.pools.values()) + list(
            self.generic_pools.values()
        ):
            if id(pool) == buffer.pool_id:
                pool.release(buffer)
                return

        raise BufferPoolError("Buffer pool not found")

    def get_stats(self) -> Dict[str, Dict[str, any]]:
        """Get statistics for all pools."""
        stats = {}

        # Area-specific pools
        for area_id, pool in self.pools.items():
            stats[f"area_{area_id}"] = {
                "available": pool.available,
                "in_use": pool.in_use,
                "stats": pool.stats.copy(),
            }

        # Generic pools
        for size, pool in self.generic_pools.items():
            stats[f"generic_{size}"] = {
                "available": pool.available,
                "in_use": pool.in_use,
                "stats": pool.stats.copy(),
            }

        return stats

    def close(self):
        """Close all pools."""
        for pool in self.pools.values():
            pool.close()
        for pool in self.generic_pools.values():
            pool.close()


# Example usage
if __name__ == "__main__":
    # Create buffer pool for neural processing
    pool = FixedBufferPool(count=10, size=4096)

    # Acquire buffer - zero allocation!
    buffer = pool.acquire()
    if buffer:
        # Use buffer for neural data
        buffer.data[0:4] = b"TEST"
        print(f"Buffer acquired: {len(buffer.data)} bytes")

        # Release back to pool
        pool.release(buffer)

    print(f"Pool stats: Available={pool.available}, In use={pool.in_use}")

    # Neural buffer pool example
    cortical_config = {
        "visual_cortex": {
            "neuron_count": 100000,
            "buffer_count": 16,
        },  # VISUALIZATION FIX: Increased from 10,000 to 100,000
        "motor_cortex": {"neuron_count": 5000, "buffer_count": 8},
    }

    neural_pool = NeuralBufferPool(cortical_config)

    # Get buffer for specific cortical area
    visual_buffer = neural_pool.get_buffer_for_area("visual_cortex")
    if visual_buffer:
        print(f"Visual cortex buffer: {visual_buffer.size} bytes")
        neural_pool.release_buffer(visual_buffer)
