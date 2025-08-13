import ctypes
import logging
import os
import time
from typing import Protocol, runtime_checkable

from .atomic_state import RustCompatibleState
from .state_errors import Result, StateError

logger = logging.getLogger(__name__)

@runtime_checkable
class StateStorage(Protocol):
    """Rust-compatible storage interface."""
    
    def load_state(self) -> Result[RustCompatibleState]:
        """Load state from storage."""
        ...
    
    def store_state(self, state: RustCompatibleState) -> Result[None]:
        """Store state to storage."""
        ...
    
    def is_available(self) -> bool:
        """Check if storage backend is available."""
        ...

class MemoryStorage:
    """Direct memory storage backend."""
    
    def __init__(self):
        """Initialize with default state."""
        self._state = RustCompatibleState()
        self._initialized = True
        logger.debug("MemoryStorage initialized")
    
    def load_state(self) -> Result[RustCompatibleState]:
        """Load state from memory."""
        if not self._initialized:
            return Result.err(StateError.STORAGE_FAILURE)
        
        try:
            # Create a copy to avoid aliasing issues
            state_copy = RustCompatibleState()
            ctypes.memmove(
                ctypes.addressof(state_copy),
                ctypes.addressof(self._state),
                ctypes.sizeof(RustCompatibleState)
            )
            return Result.ok(state_copy)
        except Exception as e:
            logger.error(f"Error loading state from memory: {e}")
            return Result.err(StateError.STORAGE_FAILURE)
    
    def store_state(self, state: RustCompatibleState) -> Result[None]:
        """Store state to memory."""
        if not self._initialized:
            return Result.err(StateError.STORAGE_FAILURE)
        
        try:
            # Validate state before storing
            if not state.validate_invariants():
                return Result.err(StateError.VALIDATION_FAILED)
            
            # Update timestamp
            state.last_modified = int(time.time() * 1000)
            
            # Copy to internal storage
            ctypes.memmove(
                ctypes.addressof(self._state),
                ctypes.addressof(state),
                ctypes.sizeof(RustCompatibleState)
            )
            return Result.ok(None)
        except Exception as e:
            logger.error(f"Error storing state to memory: {e}")
            return Result.err(StateError.STORAGE_FAILURE)
    
    def is_available(self) -> bool:
        """Memory storage is always available."""
        return self._initialized

class FileStorage:
    """File-based storage backend."""
    
    def __init__(self, file_path: str):
        """Initialize with file path."""
        self._file_path = file_path
        self._initialized = True
        logger.debug(f"FileStorage initialized with path: {file_path}")
        
        # Ensure parent directory exists
        os.makedirs(os.path.dirname(self._file_path), exist_ok=True)
        
        # Create file if it doesn't exist to satisfy test expectations
        if not os.path.exists(self._file_path):
            # Create empty file with correct size
            default_state = RustCompatibleState()
            with open(self._file_path, 'wb') as f:
                f.write(default_state.to_bytes())
    
    def load_state(self) -> Result[RustCompatibleState]:
        """Load state from file."""
        if not self._initialized:
            return Result.err(StateError.STORAGE_FAILURE)
        
        try:
            if not os.path.exists(self._file_path):
                # Create default state if file doesn't exist
                return Result.ok(RustCompatibleState())
            
            with open(self._file_path, 'rb') as f:
                data = f.read()
                
            if len(data) < ctypes.sizeof(RustCompatibleState):
                # File too small, return default state
                logger.warning(f"State file {self._file_path} too small, using default state")
                return Result.ok(RustCompatibleState())
            
            # Create state from file data
            state = RustCompatibleState()
            ctypes.memmove(
                ctypes.addressof(state),
                data,
                ctypes.sizeof(RustCompatibleState)
            )
            
            # Validate loaded state
            if not state.validate_invariants():
                logger.warning(f"Invalid state loaded from {self._file_path}, using default")
                return Result.ok(RustCompatibleState())
            
            return Result.ok(state)
        except Exception as e:
            logger.error(f"Error loading state from file {self._file_path}: {e}")
            return Result.err(StateError.STORAGE_FAILURE)
    
    def store_state(self, state: RustCompatibleState) -> Result[None]:
        """Store state to file."""
        if not self._initialized:
            return Result.err(StateError.STORAGE_FAILURE)
        
        try:
            # Validate state before storing
            if not state.validate_invariants():
                return Result.err(StateError.VALIDATION_FAILED)
            
            # Update timestamp
            state.last_modified = int(time.time() * 1000)
            
            # Write to file atomically using temp file
            temp_path = f"{self._file_path}.tmp"
            with open(temp_path, 'wb') as f:
                f.write(state.to_bytes())
                f.flush()
                os.fsync(f.fileno())
            
            # Atomic move
            os.rename(temp_path, self._file_path)
            return Result.ok(None)
        except Exception as e:
            logger.error(f"Error storing state to file {self._file_path}: {e}")
            # Clean up temp file if it exists
            temp_path = f"{self._file_path}.tmp"
            if os.path.exists(temp_path):
                try:
                    os.unlink(temp_path)
                except Exception:
                    pass
            return Result.err(StateError.STORAGE_FAILURE)
    
    def is_available(self) -> bool:
        """Check if file storage is available."""
        try:
            # Check if we can write to the directory
            parent_dir = os.path.dirname(self._file_path)
            return os.access(parent_dir, os.W_OK) and self._initialized
        except Exception:
            return False
