from typing import Protocol; import ctypes; import time; import logging; from .state_errors import Result, StateError; from .atomic_state import RustCompatibleState

logger = logging.getLogger(__name__)

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
