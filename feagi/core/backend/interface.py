"""
Backend interface definitions for FEAGI.

This module defines the common interfaces and types for all backend implementations,
allowing for transparent switching between different computational backends.
"""

import enum
import logging
import threading
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Set, Tuple, Union, Any, Type

import numpy as np

from feagi.config import config

logger = logging.getLogger("feagi.core.backend")


class BackendType(enum.Enum):
    """
    Enumeration of supported backend types.
    """
    CPU = "cpu"
    WEBGPU = "webgpu"
    CUDA = "cuda"
    METAL = "metal"
    AUTO = "auto"  # Automatically select the best available backend


class BackendCapability(enum.Enum):
    """
    Enumeration of backend capabilities.
    """
    MATRIX_MULTIPLICATION = "matrix_multiplication"
    CONVOLUTION = "convolution"
    ELEMENT_WISE_OPERATIONS = "element_wise_operations"
    BITMAP_OPERATIONS = "bitmap_operations"
    RANDOM_NUMBER_GENERATION = "random_number_generation"
    SPARSE_OPERATIONS = "sparse_operations"


class BackendInterface(ABC):
    """
    Abstract base class for all backend implementations.
    
    Each backend must implement this interface to provide consistent
    functionality across different hardware configurations.
    """
    
    def __init__(self, name: str, device: Optional[str] = None):
        """
        Initialize the backend.
        
        Args:
            name: Name of the backend.
            device: Specific device to use (e.g., "cuda:0", "mps").
        """
        self.name = name
        self.device = device
        self._initialized = False
        self._capabilities: Set[BackendCapability] = set()
    
    @abstractmethod
    def initialize(self) -> bool:
        """
        Initialize the backend.
        
        Returns:
            True if initialization succeeded, False otherwise.
        """
        pass
    
    @abstractmethod
    def shutdown(self) -> None:
        """Shutdown the backend and release resources."""
        pass
    
    @abstractmethod
    def supports_capability(self, capability: BackendCapability) -> bool:
        """
        Check if the backend supports a specific capability.
        
        Args:
            capability: The capability to check.
            
        Returns:
            True if the capability is supported, False otherwise.
        """
        pass
    
    @abstractmethod
    def create_tensor(
        self,
        shape: Tuple[int, ...],
        dtype: Any = np.float32,
        data: Optional[Any] = None,
    ) -> Any:
        """
        Create a tensor with the given shape and type.
        
        Args:
            shape: Shape of the tensor.
            dtype: Data type of the tensor.
            data: Optional data to initialize the tensor with.
            
        Returns:
            A tensor object compatible with this backend.
        """
        pass
    
    @abstractmethod
    def to_numpy(self, tensor: Any) -> np.ndarray:
        """
        Convert a backend-specific tensor to a NumPy array.
        
        Args:
            tensor: Backend-specific tensor.
            
        Returns:
            NumPy array with the same data.
        """
        pass
    
    @abstractmethod
    def from_numpy(self, array: np.ndarray) -> Any:
        """
        Convert a NumPy array to a backend-specific tensor.
        
        Args:
            array: NumPy array.
            
        Returns:
            Backend-specific tensor with the same data.
        """
        pass
    
    @abstractmethod
    def synchronize(self) -> None:
        """
        Ensure all pending operations are complete.
        
        This is particularly important for asynchronous backends like CUDA.
        """
        pass
    
    @property
    def is_initialized(self) -> bool:
        """Check if the backend is initialized."""
        return self._initialized


# Global registry of available backends
_BACKENDS: Dict[BackendType, Type[BackendInterface]] = {}

# Global registry of initialized backend instances
_BACKEND_INSTANCES: Dict[BackendType, BackendInterface] = {}

# Lock for thread safety
_backend_lock = threading.RLock()


def register_backend(backend_type: BackendType, backend_class: Type[BackendInterface]) -> None:
    """
    Register a backend implementation.
    
    Args:
        backend_type: Type of the backend.
        backend_class: Backend implementation class.
    """
    with _backend_lock:
        _BACKENDS[backend_type] = backend_class
        logger.debug(f"Registered backend {backend_type.value} with class {backend_class.__name__}")


def get_available_backends() -> List[BackendType]:
    """
    Get a list of available backends.
    
    Returns:
        List of available backend types.
    """
    available = []
    
    # CPU is always available
    available.append(BackendType.CPU)
    
    # Try to import and detect other backends
    try:
        import torch
        if torch.cuda.is_available():
            available.append(BackendType.CUDA)
        
        if hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
            available.append(BackendType.METAL)
    except ImportError:
        pass
    
    # Check for WebGPU support (this is a placeholder - actual detection would be more complex)
    try:
        # This would need to be implemented based on how WebGPU is integrated
        # For now, we just assume it's not available
        pass
    except ImportError:
        pass
    
    return available


def determine_best_backend() -> BackendType:
    """
    Determine the best available backend based on hardware capabilities.
    
    Returns:
        The best backend type to use.
    """
    available = get_available_backends()
    
    # Prefer CUDA > Metal > WebGPU > CPU
    if BackendType.CUDA in available:
        return BackendType.CUDA
    elif BackendType.METAL in available:
        return BackendType.METAL
    elif BackendType.WEBGPU in available:
        return BackendType.WEBGPU
    else:
        return BackendType.CPU


def get_backend(backend_type: Optional[BackendType] = None) -> BackendInterface:
    """
    Get or create a backend instance.
    
    Args:
        backend_type: Type of backend to get or create. If None, use the configured backend.
        
    Returns:
        Backend instance.
        
    Raises:
        ValueError: If the requested backend is not available.
    """
    with _backend_lock:
        # Determine which backend to use
        if backend_type is None:
            # Use configured backend, defaulting to AUTO
            config_backend = config.get("npu.backend", "auto")
            
            if config_backend == "auto":
                backend_type = determine_best_backend()
            else:
                try:
                    backend_type = BackendType(config_backend)
                except ValueError:
                    logger.warning(
                        f"Invalid backend type {config_backend} in configuration. "
                        f"Using auto detection."
                    )
                    backend_type = determine_best_backend()
        
        # If AUTO is explicitly specified, determine the best backend
        if backend_type == BackendType.AUTO:
            backend_type = determine_best_backend()
        
        # Check if we already have an instance
        if backend_type in _BACKEND_INSTANCES:
            return _BACKEND_INSTANCES[backend_type]
        
        # Check if the backend is available
        if backend_type not in _BACKENDS:
            available = get_available_backends()
            if backend_type not in available:
                if backend_type == BackendType.CPU:
                    # This should never happen as CPU is always available
                    raise RuntimeError("CPU backend is not available. This is a bug.")
                else:
                    # For other backends, fall back to CPU with a warning
                    logger.warning(
                        f"Requested backend {backend_type.value} is not available. "
                        f"Falling back to CPU."
                    )
                    backend_type = BackendType.CPU
            else:
                # Backend is available but not registered yet
                # We'll register CPU implementations later in this file
                raise ValueError(
                    f"Backend {backend_type.value} is available but not registered. "
                    f"This is likely a bug in the backend registration."
                )
        
        # Create and initialize the backend
        backend_class = _BACKENDS[backend_type]
        instance = backend_class()
        
        success = instance.initialize()
        if not success:
            if backend_type == BackendType.CPU:
                # This should never happen for CPU
                raise RuntimeError("Failed to initialize CPU backend. This is a bug.")
            else:
                # For other backends, fall back to CPU with a warning
                logger.warning(
                    f"Failed to initialize {backend_type.value} backend. "
                    f"Falling back to CPU."
                )
                return get_backend(BackendType.CPU)
        
        # Cache the instance
        _BACKEND_INSTANCES[backend_type] = instance
        
        logger.info(f"Using {backend_type.value} backend")
        return instance 