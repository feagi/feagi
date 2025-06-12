"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
Shared Memory Manager for FEAGI IPC.

This module provides a high-level interface for managing shared memory
regions used for inter-process communication in FEAGI.
"""

import logging
import mmap
import multiprocessing as mp
import os
import threading

from feagi.utils.logger import setup_logger

logger = setup_logger()
import tempfile
from pathlib import Path
from typing import Any, Dict, Optional, Tuple, Union

import numpy as np

# Constants
DEFAULT_SHARED_MEM_SIZE = 1024 * 1024 * 10  # 10MB
DEFAULT_TEMP_DIR = tempfile.gettempdir()


class SharedMemoryRegion:
    """
    Represents a single shared memory region that can be accessed by multiple processes.
    """

    def __init__(
        self,
        name: str,
        size: int = DEFAULT_SHARED_MEM_SIZE,
        temp_dir: str = DEFAULT_TEMP_DIR,
        create: bool = True,
    ):
        """
        Initialize a shared memory region.

        Args:
            name: Unique identifier for this shared memory region
            size: Size in bytes for the shared memory region
            temp_dir: Directory to store the memory-mapped file
            create: Whether to create the region (True) or just open existing (False)
        """
        self.name = name
        self.size = size
        self.logger = logging.getLogger(f"feagi.api.shared_memory.{name}")

        # Create the file path
        self.file_path = os.path.join(temp_dir, f"feagi_shared_{name}.dat")
        self.lock_path = os.path.join(temp_dir, f"feagi_shared_{name}.lock")

        # Create or open the file
        if create:
            self._create_memory_region()
        else:
            self._open_memory_region()

        # Initialize lock
        self._lock = threading.RLock()  # For thread safety within a process
        self._file_lock = None  # Will be created when needed for cross-process locking

    def _create_memory_region(self):
        """Create a new shared memory region."""
        if not os.path.exists(self.file_path):
            # Create and initialize the file
            with open(self.file_path, "wb") as f:
                f.write(b"\0" * self.size)

        # Open the file for reading/writing
        self.file = open(self.file_path, "r+b")

        # Create memory map
        self.mmap = mmap.mmap(self.file.fileno(), self.size)
        self.logger.info(
            f"Created shared memory region '{self.name}' at {self.file_path} ({self.size} bytes)"
        )

    def _open_memory_region(self):
        """Open an existing shared memory region."""
        if not os.path.exists(self.file_path):
            raise FileNotFoundError(
                f"Shared memory region '{self.name}' does not exist at {self.file_path}"
            )

        # Open the file for reading/writing
        self.file = open(self.file_path, "r+b")

        # Create memory map
        self.mmap = mmap.mmap(
            self.file.fileno(), 0
        )  # 0 means use the file's actual size
        self.size = self.mmap.size()
        self.logger.info(
            f"Opened shared memory region '{self.name}' at {self.file_path} ({self.size} bytes)"
        )

    def acquire_lock(self, timeout: Optional[float] = None) -> bool:
        """
        Acquire a lock on this memory region.

        Args:
            timeout: Maximum time to wait for the lock (None means wait forever)

        Returns:
            True if lock was acquired, False if timed out
        """
        # First acquire the thread lock
        if not self._lock.acquire(blocking=True, timeout=timeout):
            return False

        # Then acquire the file lock for cross-process synchronization
        try:
            import fcntl  # Unix-only

            self._file_lock = open(self.lock_path, "w+")
            fcntl.flock(
                self._file_lock.fileno(),
                fcntl.LOCK_EX | (fcntl.LOCK_NB if timeout == 0 else 0),
            )
            return True
        except ImportError:
            # On Windows, use a simpler locking mechanism
            # This is less robust but better than nothing
            self.logger.warning("Using simplified file locking on Windows")
            return True
        except (IOError, BlockingIOError):
            # Could not acquire the lock
            self._lock.release()
            return False

    def release_lock(self):
        """Release the lock on this memory region."""
        try:
            if self._file_lock:
                import fcntl

                fcntl.flock(self._file_lock.fileno(), fcntl.LOCK_UN)
                self._file_lock.close()
                self._file_lock = None
        except ImportError:
            # Windows case
            pass
        except Exception as e:
            self.logger.error(f"Error releasing file lock: {e}")

        self._lock.release()

    def read(self, offset: int = 0, size: Optional[int] = None) -> bytes:
        """
        Read data from the shared memory region.

        Args:
            offset: Starting position for the read
            size: Number of bytes to read (None means read until the end)

        Returns:
            Bytes read from the memory region
        """
        with self._lock:
            self.mmap.seek(offset)
            return self.mmap.read(size if size is not None else (self.size - offset))

    def write(self, data: bytes, offset: int = 0) -> int:
        """
        Write data to the shared memory region.

        Args:
            data: Bytes to write
            offset: Starting position for the write

        Returns:
            Number of bytes written
        """
        with self._lock:
            self.mmap.seek(offset)
            written = self.mmap.write(data)
            self.mmap.flush()
            self.file.flush()
            os.fsync(self.file.fileno())
            return written

    def as_array(
        self, shape: Tuple[int, ...], dtype: np.dtype = np.float32
    ) -> np.ndarray:
        """
        Get a numpy array view of the shared memory.

        Args:
            shape: Shape of the numpy array
            dtype: Data type of the numpy array

        Returns:
            NumPy array backed by the shared memory
        """
        # Calculate required size
        required_size = int(np.prod(shape) * np.dtype(dtype).itemsize)
        if required_size > self.size:
            raise ValueError(
                f"Array size {required_size} exceeds shared memory size {self.size}"
            )

        # Create a numpy array that views the shared memory
        return np.frombuffer(self.mmap, dtype=dtype).reshape(shape)

    def close(self, delete_file: bool = False):
        """
        Close the shared memory region. Optionally delete the file.
        Args:
            delete_file: If True, delete the file from disk (default: False)
        """
        try:
            self.mmap.close()
            self.file.close()
            if delete_file and os.path.exists(self.file_path):
                os.remove(self.file_path)
        except Exception as e:
            self.logger.error(f"Error closing shared memory region: {e}")

    def __del__(self):
        """Ensure resources are cleaned up."""
        self.close()

    def reload(self):
        """Reload the memory map from disk (for cross-process consistency)."""
        with self._lock:
            self.mmap.close()
            self.file.close()
            self.file = open(self.file_path, "r+b")
            self.mmap = mmap.mmap(self.file.fileno(), self.size)


class SharedMemoryManager:
    """
    Manages multiple shared memory regions used for IPC in FEAGI.
    """

    def __init__(self, temp_dir: str = DEFAULT_TEMP_DIR):
        """
        Initialize the shared memory manager.

        Args:
            temp_dir: Directory to store the memory-mapped files
        """
        self.temp_dir = temp_dir
        self.regions: Dict[str, SharedMemoryRegion] = {}
        self.logger = logging.getLogger("feagi.api.shared_memory.manager")

        # Make sure the temp directory exists
        os.makedirs(temp_dir, exist_ok=True)

    def create_region(
        self, name: str, size: int = DEFAULT_SHARED_MEM_SIZE
    ) -> SharedMemoryRegion:
        """
        Create a new shared memory region.

        Args:
            name: Unique identifier for the region
            size: Size in bytes for the shared memory region

        Returns:
            The created shared memory region
        """
        if name in self.regions:
            self.logger.warning(
                f"Shared memory region '{name}' already exists, returning existing region"
            )
            return self.regions[name]

        region = SharedMemoryRegion(name, size, self.temp_dir, create=True)
        self.regions[name] = region
        return region

    def get_region(self, name: str) -> SharedMemoryRegion:
        """
        Get an existing shared memory region.

        Args:
            name: Identifier for the region to get

        Returns:
            The shared memory region

        Raises:
            KeyError: If the region doesn't exist
        """
        if name not in self.regions:
            # Try to open an existing region
            try:
                region = SharedMemoryRegion(name, temp_dir=self.temp_dir, create=False)
                self.regions[name] = region
                return region
            except FileNotFoundError:
                raise KeyError(f"Shared memory region '{name}' does not exist")

        return self.regions[name]

    def delete_region(self, name: str):
        """Explicitly delete a shared memory region file."""
        if name in self.regions:
            self.regions[name].close(delete_file=True)
            del self.regions[name]

    def list_regions(self) -> Dict[str, int]:
        """
        List all managed shared memory regions.

        Returns:
            Dictionary of region names mapped to their sizes
        """
        return {name: region.size for name, region in self.regions.items()}

    def cleanup(self):
        """Clean up all managed shared memory regions."""
        for name in list(self.regions.keys()):
            self.delete_region(name)

    def __del__(self):
        """Ensure all regions are properly closed."""
        for region in self.regions.values():
            region.close()

    def region_path(self, name: str) -> str:
        """Return the file path for a given shared memory region name."""
        return os.path.join(self.temp_dir, f"feagi_shared_{name}.dat")
