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
Backend interface definitions for FEAGI.

This module defines the common interfaces and types for all backend implementations,
allowing for transparent switching between different computational backends.
"""

import enum

from feagi.utils.logger import setup_logger

logger = setup_logger("feagi.core.backend")
import threading
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Set, Tuple, Type, Union

import numpy as np

from feagi.config import config


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


def register_backend(
    backend_type: BackendType, backend_class: Type[BackendInterface]
) -> None:
    """
    Register a backend implementation.

    Args:
        backend_type: Type of the backend.
        backend_class: Backend implementation class.
    """
    with _backend_lock:
        _BACKENDS[backend_type] = backend_class
        logger.debug(
            f"Registered backend {backend_type.value} with class {backend_class.__name__}"
        )


def get_available_backends() -> List[BackendType]:
    """
    Get a list of available backends by querying the Resource Manager.

    Returns:
        List of available backend types.
    """
    from feagi.core.resource_mgr import ResourceManager

    available = []

    # CPU is always available
    available.append(BackendType.CPU)

    # Get resource information from ResourceManager
    try:
        resource_mgr = ResourceManager.get_instance()
        resources = resource_mgr.resources

        # Check for CUDA GPU availability
        if resources.get("gpu_available", False) and resources.get("gpu_count", 0) > 0:
            available.append(BackendType.CUDA)

        # Check for WebGPU availability
        if BackendType.WEBGPU in _BACKENDS:
            available.append(BackendType.WEBGPU)

        # Check for Metal (Apple Silicon) availability
        if resources.get("metal_available", False) and BackendType.METAL in _BACKENDS:
            available.append(BackendType.METAL)

        logger.debug(
            f"Available backends detected via ResourceManager: {[b.value for b in available]}"
        )
    except Exception as e:
        logger.warning(f"Error detecting available backends via ResourceManager: {e}")
        logger.warning("Falling back to CPU backend only")

    return available


def determine_best_backend() -> BackendType:
    """
    Determine the best available backend based on system capabilities.

    This function queries the ResourceManager to get the most appropriate
    backend for the current hardware configuration.

    Returns:
        The best available backend type.
    """
    from feagi.core.resource_mgr import ResourceManager

    # Get system resources from ResourceManager
    resources = ResourceManager.get_instance().resources

    # First preference: CUDA GPU if available
    if (
        resources.get("gpu_available", False)
        and resources.get("gpu_count", 0) > 0
        and BackendType.CUDA in _BACKENDS
    ):
        logger.info("CUDA GPU detected, selecting CUDA backend")
        return BackendType.CUDA

    # Second preference: Metal for Apple Silicon
    if resources.get("metal_available", False) and BackendType.METAL in _BACKENDS:
        logger.info("Apple Metal detected, selecting Metal backend")
        return BackendType.METAL

    # Third preference: WebGPU if available
    if resources.get("webgpu_available", False) and BackendType.WEBGPU in _BACKENDS:
        logger.info("WebGPU detected, selecting WebGPU backend")
        return BackendType.WEBGPU

    # Default: CPU
    logger.info("No GPU acceleration detected, falling back to CPU backend")
    return BackendType.CPU


def get_backend(
    backend_type: Optional[BackendType] = None,
) -> Optional[BackendInterface]:
    """
    Get or create a backend instance.

    Args:
        backend_type: Type of backend to get. If None, the best available backend is chosen.

    Returns:
        A backend instance, or None if the requested backend is not available.
    """
    with _backend_lock:
        # If no specific backend is requested, determine the best one
        if backend_type is None or backend_type == BackendType.AUTO:
            backend_type = determine_best_backend()

        # Check if we already have an initialized instance
        if backend_type in _BACKEND_INSTANCES:
            return _BACKEND_INSTANCES[backend_type]

        # If backend is not registered, return None or try fallback
        if backend_type not in _BACKENDS:
            logger.warning(f"Backend {backend_type.value} is not registered")

            # Check if user specifically requested this backend or it was auto-selected
            if backend_type in [BackendType.AUTO, BackendType.CPU]:
                logger.warning("No fallback available, returning None")
                return None
            else:
                # Try fallback to CPU
                logger.warning(f"Falling back to CPU backend.")
                backend_type = BackendType.CPU
                if backend_type not in _BACKENDS:
                    logger.error(
                        "CPU backend is not registered. This should never happen."
                    )
                    return None

        # Create a new backend instance
        try:
            backend_class = _BACKENDS[backend_type]
            backend = backend_class()

            # Initialize the backend
            init_success = backend.initialize()
            if not init_success:
                logger.warning(
                    f"Failed to initialize {backend_type.value} backend. Falling back to CPU."
                )
                # Try fallback to CPU if requested backend failed to initialize
                if backend_type != BackendType.CPU:
                    return get_backend(BackendType.CPU)
                else:
                    logger.error(
                        "CPU backend initialization failed. This should never happen."
                    )
                    return None

            # Store the initialized instance
            _BACKEND_INSTANCES[backend_type] = backend
            logger.info(f"Successfully initialized {backend_type.value} backend")
            return backend
        except Exception as e:
            logger.error(f"Error initializing {backend_type.value} backend: {e}")
            # Try fallback to CPU if there was an error
            if backend_type != BackendType.CPU:
                logger.warning(f"Falling back to CPU backend.")
                return get_backend(BackendType.CPU)
            else:
                logger.error(
                    "CPU backend initialization error. This should never happen."
                )
                return None
