"""Array backend abstraction for FEAGI.

This module provides a unified interface for different array backends (NumPy, PyTorch, CuPy, WebGPU),
enabling transparent switching between CPU and GPU acceleration.
"""

import logging
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple, Union, Type

import numpy as np

# Try to import optional backends
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

try:
    import cupy as cp
    CUPY_AVAILABLE = True
except ImportError:
    CUPY_AVAILABLE = False

try:
    import wgpu
    WGPU_AVAILABLE = True
except ImportError:
    WGPU_AVAILABLE = False

logger = logging.getLogger(__name__)


class BackendType(Enum):
    """Supported array backends."""
    NUMPY = "numpy"
    PYTORCH = "pytorch"
    CUPY = "cupy"
    WEBGPU = "webgpu"
    AUTO = "auto"  # Automatically select best available backend


class ArrayBackend:
    """Backend-agnostic array operations.
    
    This class provides a unified interface for array operations across different
    backends (NumPy, PyTorch, CuPy, WebGPU), enabling transparent switching
    between CPU and GPU acceleration.
    """
    
    def __init__(self, backend_type: Union[str, BackendType] = BackendType.AUTO):
        """Initialize array backend.
        
        Args:
            backend_type: Backend to use (numpy, pytorch, cupy, webgpu, or auto)
        """
        if isinstance(backend_type, str):
            try:
                backend_type = BackendType(backend_type.lower())
            except ValueError:
                logger.warning(f"Unknown backend type: {backend_type}. Falling back to AUTO.")
                backend_type = BackendType.AUTO
        
        self.backend_type = self._resolve_backend_type(backend_type)
        self._initialize_backend()
        logger.info(f"Using array backend: {self.backend_type.value}")
    
    def _resolve_backend_type(self, backend_type: BackendType) -> BackendType:
        """Resolve AUTO backend type to concrete backend."""
        if backend_type != BackendType.AUTO:
            # Validate availability
            if backend_type == BackendType.PYTORCH and not TORCH_AVAILABLE:
                logger.warning("PyTorch requested but not available. Falling back to AUTO selection.")
                backend_type = BackendType.AUTO
            elif backend_type == BackendType.CUPY and not CUPY_AVAILABLE:
                logger.warning("CuPy requested but not available. Falling back to AUTO selection.")
                backend_type = BackendType.AUTO
            elif backend_type == BackendType.WEBGPU and not WGPU_AVAILABLE:
                logger.warning("WebGPU requested but not available. Falling back to AUTO selection.")
                backend_type = BackendType.AUTO
        
        if backend_type == BackendType.AUTO:
            # Select best available backend
            if TORCH_AVAILABLE and torch.cuda.is_available():
                return BackendType.PYTORCH
            elif CUPY_AVAILABLE:
                return BackendType.CUPY
            elif WGPU_AVAILABLE:
                return BackendType.WEBGPU
            else:
                return BackendType.NUMPY
        
        return backend_type
    
    def _initialize_backend(self):
        """Initialize the selected backend."""
        if self.backend_type == BackendType.PYTORCH:
            # Determine device
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
            if self.device == "cuda":
                logger.info(f"Using PyTorch with CUDA device: {torch.cuda.get_device_name(0)}")
            else:
                logger.info("Using PyTorch with CPU device")
        elif self.backend_type == BackendType.WEBGPU:
            # Initialize WebGPU device
            self.adapter = wgpu.request_adapter()
            self.device = self.adapter.request_device()
            logger.info(f"Using WebGPU device: {self.adapter.request_adapter_info().description}")
        elif self.backend_type == BackendType.CUPY:
            # Use default CUDA device
            logger.info(f"Using CuPy with device: {cp.cuda.runtime.getDeviceProperties(0)['name'].decode()}")
        else:
            # NumPy always uses CPU
            logger.info("Using NumPy backend on CPU")
    
    def zeros(self, shape: Tuple[int, ...], dtype: Any = None) -> Any:
        """Create array of zeros with specified shape and type.
        
        Args:
            shape: Shape of the array
            dtype: Data type (default: float32)
            
        Returns:
            Array of zeros with backend-specific type
        """
        if dtype is None:
            dtype = np.float32
            
        if self.backend_type == BackendType.NUMPY:
            return np.zeros(shape, dtype=dtype)
        elif self.backend_type == BackendType.PYTORCH:
            torch_dtype = self._numpy_to_torch_dtype(dtype)
            return torch.zeros(shape, dtype=torch_dtype, device=self.device)
        elif self.backend_type == BackendType.CUPY:
            return cp.zeros(shape, dtype=dtype)
        elif self.backend_type == BackendType.WEBGPU:
            # For WebGPU, we create a NumPy array first, then transfer it to GPU
            cpu_array = np.zeros(shape, dtype=dtype)
            return self._numpy_to_wgpu(cpu_array)
    
    def ones(self, shape: Tuple[int, ...], dtype: Any = None) -> Any:
        """Create array of ones with specified shape and type.
        
        Args:
            shape: Shape of the array
            dtype: Data type (default: float32)
            
        Returns:
            Array of ones with backend-specific type
        """
        if dtype is None:
            dtype = np.float32
            
        if self.backend_type == BackendType.NUMPY:
            return np.ones(shape, dtype=dtype)
        elif self.backend_type == BackendType.PYTORCH:
            torch_dtype = self._numpy_to_torch_dtype(dtype)
            return torch.ones(shape, dtype=torch_dtype, device=self.device)
        elif self.backend_type == BackendType.CUPY:
            return cp.ones(shape, dtype=dtype)
        elif self.backend_type == BackendType.WEBGPU:
            # For WebGPU, we create a NumPy array first, then transfer it to GPU
            cpu_array = np.ones(shape, dtype=dtype)
            return self._numpy_to_wgpu(cpu_array)
    
    def full(self, shape: Tuple[int, ...], fill_value: Union[float, int], dtype: Any = None) -> Any:
        """Create array filled with specified value.
        
        Args:
            shape: Shape of the array
            fill_value: Value to fill array with
            dtype: Data type (default: float32 for float values, int32 for int values)
            
        Returns:
            Array filled with specified value
        """
        if dtype is None:
            dtype = np.float32 if isinstance(fill_value, float) else np.int32
            
        if self.backend_type == BackendType.NUMPY:
            return np.full(shape, fill_value, dtype=dtype)
        elif self.backend_type == BackendType.PYTORCH:
            torch_dtype = self._numpy_to_torch_dtype(dtype)
            return torch.full(shape, fill_value, dtype=torch_dtype, device=self.device)
        elif self.backend_type == BackendType.CUPY:
            return cp.full(shape, fill_value, dtype=dtype)
        elif self.backend_type == BackendType.WEBGPU:
            # For WebGPU, we create a NumPy array first, then transfer it to GPU
            cpu_array = np.full(shape, fill_value, dtype=dtype)
            return self._numpy_to_wgpu(cpu_array)
    
    def array(self, data: Any, dtype: Any = None) -> Any:
        """Create array from data.
        
        Args:
            data: Data to create array from (list, tuple, NumPy array, etc.)
            dtype: Data type (default: inferred from data)
            
        Returns:
            Array with backend-specific type
        """
        if self.backend_type == BackendType.NUMPY:
            return np.array(data, dtype=dtype)
        elif self.backend_type == BackendType.PYTORCH:
            torch_dtype = self._numpy_to_torch_dtype(dtype) if dtype is not None else None
            if isinstance(data, np.ndarray):
                return torch.from_numpy(data).to(self.device, dtype=torch_dtype)
            else:
                return torch.tensor(data, dtype=torch_dtype, device=self.device)
        elif self.backend_type == BackendType.CUPY:
            if isinstance(data, np.ndarray):
                return cp.array(data, dtype=dtype)
            else:
                return cp.array(data, dtype=dtype)
        elif self.backend_type == BackendType.WEBGPU:
            # For WebGPU, we create a NumPy array first, then transfer it to GPU
            cpu_array = np.array(data, dtype=dtype)
            return self._numpy_to_wgpu(cpu_array)
    
    def to_numpy(self, array: Any) -> np.ndarray:
        """Convert backend-specific array to NumPy array.
        
        Args:
            array: Backend-specific array
            
        Returns:
            NumPy array
        """
        if self.backend_type == BackendType.NUMPY:
            return array
        elif self.backend_type == BackendType.PYTORCH:
            return array.detach().cpu().numpy()
        elif self.backend_type == BackendType.CUPY:
            return cp.asnumpy(array)
        elif self.backend_type == BackendType.WEBGPU:
            return self._wgpu_to_numpy(array)
    
    def sparse_csr(self, data: Any, indices: Any, indptr: Any, shape: Tuple[int, ...]) -> Any:
        """Create sparse CSR matrix.
        
        Args:
            data: Data array (values)
            indices: Column indices array
            indptr: Row pointers array
            shape: Shape of the matrix (rows, cols)
            
        Returns:
            Sparse CSR matrix with backend-specific type
        """
        if self.backend_type == BackendType.NUMPY:
            from scipy import sparse
            return sparse.csr_matrix((data, indices, indptr), shape=shape)
        elif self.backend_type == BackendType.PYTORCH:
            if not isinstance(data, torch.Tensor):
                data = torch.tensor(data, device=self.device)
            if not isinstance(indices, torch.Tensor):
                indices = torch.tensor(indices, dtype=torch.int64, device=self.device)
            if not isinstance(indptr, torch.Tensor):
                indptr = torch.tensor(indptr, dtype=torch.int64, device=self.device)
            return torch.sparse_csr_tensor(indptr, indices, data, size=shape, device=self.device)
        elif self.backend_type == BackendType.CUPY:
            return cp.sparse.csr_matrix((data, indices, indptr), shape=shape)
        elif self.backend_type == BackendType.WEBGPU:
            # WebGPU doesn't have native sparse matrix support
            # Return the components as separate arrays
            from collections import namedtuple
            WGPUSparse = namedtuple('WGPUSparse', ['data', 'indices', 'indptr', 'shape'])
            return WGPUSparse(
                data=self.array(data),
                indices=self.array(indices, dtype=np.int32),
                indptr=self.array(indptr, dtype=np.int32),
                shape=shape
            )
    
    def to_device(self, array: Any) -> Any:
        """Transfer array to the appropriate device (CPU/GPU).
        
        Args:
            array: Array to transfer
            
        Returns:
            Array on the appropriate device
        """
        if self.backend_type == BackendType.NUMPY:
            # NumPy is always on CPU, nothing to do
            return array
        elif self.backend_type == BackendType.PYTORCH:
            if isinstance(array, torch.Tensor) and array.device.type != self.device:
                return array.to(self.device)
            elif isinstance(array, np.ndarray):
                return torch.from_numpy(array).to(self.device)
            else:
                return array
        elif self.backend_type == BackendType.CUPY:
            if isinstance(array, np.ndarray):
                return cp.array(array)
            else:
                return array
        elif self.backend_type == BackendType.WEBGPU:
            if isinstance(array, np.ndarray):
                return self._numpy_to_wgpu(array)
            else:
                return array
    
    def to_cpu(self, array: Any) -> Any:
        """Transfer array to CPU.
        
        Args:
            array: Array to transfer to CPU
            
        Returns:
            Array on CPU (NumPy array for most backends)
        """
        return self.to_numpy(array)
    
    def _numpy_to_torch_dtype(self, dtype: Any) -> torch.dtype:
        """Convert NumPy dtype to PyTorch dtype."""
        if dtype is None:
            return None
            
        dtype_map = {
            np.float32: torch.float32,
            np.float64: torch.float64,
            np.float16: torch.float16,
            np.int32: torch.int32,
            np.int64: torch.int64,
            np.int16: torch.int16,
            np.int8: torch.int8,
            np.uint8: torch.uint8,
            np.bool_: torch.bool
        }
        
        np_dtype = np.dtype(dtype)
        torch_dtype = dtype_map.get(np_dtype.type)
        if torch_dtype is None:
            logger.warning(f"No matching PyTorch dtype for {np_dtype}, using float32")
            torch_dtype = torch.float32
            
        return torch_dtype
    
    def _numpy_to_wgpu(self, array: np.ndarray) -> Any:
        """Convert NumPy array to WebGPU buffer."""
        # This is a placeholder - actual implementation would use wgpu to create buffers
        # Store array shape and dtype for later reconstruction
        buffer = self.device.create_buffer_with_data(
            data=array.tobytes(), 
            usage=wgpu.BufferUsages.STORAGE | wgpu.BufferUsages.COPY_SRC | wgpu.BufferUsages.COPY_DST
        )
        
        # Store metadata for later reconstruction
        buffer.shape = array.shape
        buffer.dtype = array.dtype
        
        return buffer
    
    def _wgpu_to_numpy(self, buffer: Any) -> np.ndarray:
        """Convert WebGPU buffer to NumPy array."""
        # This is a simplified version - actual implementation would be more complex
        staging_buffer = self.device.create_buffer(
            size=buffer.size,
            usage=wgpu.BufferUsages.MAP_READ | wgpu.BufferUsages.COPY_DST
        )
        
        # Copy from the buffer to the staging buffer
        encoder = self.device.create_command_encoder()
        encoder.copy_buffer_to_buffer(buffer, 0, staging_buffer, 0, buffer.size)
        self.device.queue.submit([encoder.finish()])
        
        # Map the staging buffer and read its contents
        staging_buffer.map_read()
        data = staging_buffer.read_mapped()
        staging_buffer.unmap()
        
        # Reconstruct the NumPy array
        return np.frombuffer(data, dtype=buffer.dtype).reshape(buffer.shape) 