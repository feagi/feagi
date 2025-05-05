"""
Shared Memory Manager for FEAGI IPC.

This module provides a high-level interface for managing shared memory
regions used for inter-process communication in FEAGI.
"""

import os
import mmap
import multiprocessing as mp
import threading
import logging
import numpy as np
import tempfile
from typing import Dict, Any, Optional, Tuple, Union
from pathlib import Path

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
        create: bool = True
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
                f.write(b'\0' * self.size)
                
        # Open the file for reading/writing
        self.file = open(self.file_path, "r+b")
        
        # Create memory map
        self.mmap = mmap.mmap(self.file.fileno(), self.size)
        self.logger.info(f"Created shared memory region '{self.name}' at {self.file_path} ({self.size} bytes)")
    
    def _open_memory_region(self):
        """Open an existing shared memory region."""
        if not os.path.exists(self.file_path):
            raise FileNotFoundError(f"Shared memory region '{self.name}' does not exist at {self.file_path}")
            
        # Open the file for reading/writing
        self.file = open(self.file_path, "r+b")
        
        # Create memory map
        self.mmap = mmap.mmap(self.file.fileno(), 0)  # 0 means use the file's actual size
        self.size = self.mmap.size()
        self.logger.info(f"Opened shared memory region '{self.name}' at {self.file_path} ({self.size} bytes)")
    
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
            fcntl.flock(self._file_lock.fileno(), fcntl.LOCK_EX | (fcntl.LOCK_NB if timeout == 0 else 0))
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
            return self.mmap.write(data)
    
    def as_array(self, shape: Tuple[int, ...], dtype: np.dtype = np.float32) -> np.ndarray:
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
            raise ValueError(f"Array size {required_size} exceeds shared memory size {self.size}")
            
        # Create a numpy array that views the shared memory
        return np.frombuffer(self.mmap, dtype=dtype).reshape(shape)
    
    def close(self):
        """Close the shared memory region."""
        try:
            if hasattr(self, 'mmap') and self.mmap:
                self.mmap.close()
            if hasattr(self, 'file') and self.file:
                self.file.close()
        except Exception as e:
            self.logger.error(f"Error closing shared memory region: {e}")
    
    def __del__(self):
        """Ensure resources are cleaned up."""
        self.close()


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
    
    def create_region(self, name: str, size: int = DEFAULT_SHARED_MEM_SIZE) -> SharedMemoryRegion:
        """
        Create a new shared memory region.
        
        Args:
            name: Unique identifier for the region
            size: Size in bytes for the shared memory region
            
        Returns:
            The created shared memory region
        """
        if name in self.regions:
            self.logger.warning(f"Shared memory region '{name}' already exists, returning existing region")
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
        """
        Delete a shared memory region.
        
        Args:
            name: Identifier for the region to delete
        """
        if name in self.regions:
            region = self.regions[name]
            region.close()
            
            # Delete the files
            try:
                os.remove(region.file_path)
                if os.path.exists(region.lock_path):
                    os.remove(region.lock_path)
                self.logger.info(f"Deleted shared memory region '{name}'")
            except Exception as e:
                self.logger.error(f"Error deleting shared memory region '{name}': {e}")
                
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