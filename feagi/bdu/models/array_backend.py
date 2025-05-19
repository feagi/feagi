"""Array backend abstraction for FEAGI.

This module provides a unified interface for different array backends (NumPy, PyTorch, CuPy, WebGPU),
enabling transparent switching between CPU and GPU acceleration.
"""

import logging
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple, Union, Type, Literal

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


class PrecisionType(Enum):
    """Supported precision types for array operations."""
    FP32 = "fp32"  # 32-bit floating point (standard)
    FP16 = "fp16"  # 16-bit floating point (half precision)
    INT8 = "int8"  # 8-bit integer (quantized)
    MIXED = "mixed"  # Use mixed precision where appropriate


class ArrayBackend:
    """Backend-agnostic array operations.
    
    This class provides a unified interface for array operations across different
    backends (NumPy, PyTorch, CuPy, WebGPU), enabling transparent switching
    between CPU and GPU acceleration.
    """
    
    def __init__(self, 
                backend_type: Union[str, BackendType] = BackendType.AUTO,
                precision: Union[str, PrecisionType] = PrecisionType.FP32):
        """Initialize array backend.
        
        Args:
            backend_type: Backend to use (numpy, pytorch, cupy, webgpu, or auto)
            precision: Precision to use for computations (fp32, fp16, int8, or mixed)
        """
        if isinstance(backend_type, str):
            try:
                backend_type = BackendType(backend_type.lower())
            except ValueError:
                logger.warning(f"Unknown backend type: {backend_type}. Falling back to AUTO.")
                backend_type = BackendType.AUTO
        
        if isinstance(precision, str):
            try:
                precision = PrecisionType(precision.lower())
            except ValueError:
                logger.warning(f"Unknown precision type: {precision}. Falling back to FP32.")
                precision = PrecisionType.FP32
        
        self.backend_type = self._resolve_backend_type(backend_type)
        self.precision = precision
        self._initialize_backend()
        logger.info(f"Using array backend: {self.backend_type.value} with precision: {self.precision.value}")
    
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
    
    @staticmethod
    def _is_backend_available(backend_type: BackendType) -> bool:
        """Check if a specific backend is available.
        
        Args:
            backend_type: Backend to check
            
        Returns:
            True if the backend is available, False otherwise
        """
        if backend_type == BackendType.NUMPY:
            return True
        elif backend_type == BackendType.PYTORCH:
            return TORCH_AVAILABLE and (torch.cuda.is_available() or True)  # CPU fallback
        elif backend_type == BackendType.CUPY:
            return CUPY_AVAILABLE
        elif backend_type == BackendType.WEBGPU:
            return WGPU_AVAILABLE
        else:
            return False
    
    def _initialize_backend(self):
        """Initialize the selected backend."""
        if self.backend_type == BackendType.PYTORCH:
            # Determine device
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
            if self.device == "cuda":
                logger.info(f"Using PyTorch with CUDA device: {torch.cuda.get_device_name(0)}")
                # Set default tensor type for mixed precision if needed
                if self.precision == PrecisionType.FP16:
                    torch.set_default_tensor_type(torch.cuda.HalfTensor)
                    logger.info("Using FP16 precision with PyTorch CUDA")
                elif self.precision == PrecisionType.MIXED:
                    try:
                        # Initialize AMP (Automatic Mixed Precision)
                        from torch.cuda.amp import autocast
                        self.autocast = autocast
                        logger.info("Using Automatic Mixed Precision with PyTorch CUDA")
                    except ImportError:
                        logger.warning("AMP not available in this PyTorch version, falling back to FP32")
                        self.precision = PrecisionType.FP32
            else:
                logger.info("Using PyTorch with CPU device")
        elif self.backend_type == BackendType.WEBGPU:
            # Initialize WebGPU device
            self.adapter = wgpu.request_adapter()
            self.device = self.adapter.request_device()
            logger.info(f"Using WebGPU device: {self.adapter.request_adapter_info().description}")
        elif self.backend_type == BackendType.CUPY:
            # Use default CUDA device
            if self.precision == PrecisionType.FP16:
                try:
                    # Check if FP16 is supported
                    x = cp.array([1.0], dtype=cp.float16)
                    x + x  # Simple test
                    logger.info(f"Using CuPy with device: {cp.cuda.runtime.getDeviceProperties(0)['name'].decode()} (FP16 enabled)")
                except Exception as e:
                    logger.warning(f"FP16 not supported by CuPy: {e}, falling back to FP32")
                    self.precision = PrecisionType.FP32
            else:
                logger.info(f"Using CuPy with device: {cp.cuda.runtime.getDeviceProperties(0)['name'].decode()}")
        else:
            # NumPy always uses CPU
            if self.precision != PrecisionType.FP32:
                logger.warning("Precision settings other than FP32 have minimal effect with NumPy backend.")
            logger.info("Using NumPy backend on CPU")
    
    def _get_dtype_for_precision(self, base_dtype: Any = None) -> Any:
        """Get the appropriate dtype for the current precision setting.
        
        Args:
            base_dtype: Base dtype to convert (default: float32 or int32)
            
        Returns:
            Adjusted dtype based on precision setting
        """
        if base_dtype is None:
            base_dtype = np.float32
        
        # If already a precision-specific type, return as is
        if base_dtype in [np.float16, np.int8]:
            return base_dtype
        
        # For integer types, only convert if explicitly requested
        if np.issubdtype(np.dtype(base_dtype), np.integer):
            if self.precision == PrecisionType.INT8:
                return np.int8
            else:
                return base_dtype
                
        # For float types
        if self.precision == PrecisionType.FP16:
            return np.float16
        elif self.precision == PrecisionType.INT8:
            logger.warning("INT8 precision requested for float data. Using FP16 instead.")
            return np.float16
        else:
            return base_dtype
    
    def zeros(self, shape: Tuple[int, ...], dtype: Any = None) -> Any:
        """Create array of zeros with specified shape and type.
        
        Args:
            shape: Shape of the array
            dtype: Data type (default: adjusted based on precision setting)
            
        Returns:
            Array of zeros with backend-specific type
        """
        if dtype is None:
            dtype = np.float32
            
        # Adjust dtype based on precision setting
        adjusted_dtype = self._get_dtype_for_precision(dtype)
            
        if self.backend_type == BackendType.NUMPY:
            return np.zeros(shape, dtype=adjusted_dtype)
        elif self.backend_type == BackendType.PYTORCH:
            torch_dtype = self._numpy_to_torch_dtype(adjusted_dtype)
            return torch.zeros(shape, dtype=torch_dtype, device=self.device)
        elif self.backend_type == BackendType.CUPY:
            return cp.zeros(shape, dtype=adjusted_dtype)
        elif self.backend_type == BackendType.WEBGPU:
            # For WebGPU, we create a NumPy array first, then transfer it to GPU
            cpu_array = np.zeros(shape, dtype=adjusted_dtype)
            return self._numpy_to_wgpu(cpu_array)
    
    def ones(self, shape: Tuple[int, ...], dtype: Any = None) -> Any:
        """Create array of ones with specified shape and type.
        
        Args:
            shape: Shape of the array
            dtype: Data type (default: adjusted based on precision setting)
            
        Returns:
            Array of ones with backend-specific type
        """
        if dtype is None:
            dtype = np.float32
            
        # Adjust dtype based on precision setting
        adjusted_dtype = self._get_dtype_for_precision(dtype)
            
        if self.backend_type == BackendType.NUMPY:
            return np.ones(shape, dtype=adjusted_dtype)
        elif self.backend_type == BackendType.PYTORCH:
            torch_dtype = self._numpy_to_torch_dtype(adjusted_dtype)
            return torch.ones(shape, dtype=torch_dtype, device=self.device)
        elif self.backend_type == BackendType.CUPY:
            return cp.ones(shape, dtype=adjusted_dtype)
        elif self.backend_type == BackendType.WEBGPU:
            # For WebGPU, we create a NumPy array first, then transfer it to GPU
            cpu_array = np.ones(shape, dtype=adjusted_dtype)
            return self._numpy_to_wgpu(cpu_array)
    
    def full(self, shape: Tuple[int, ...], fill_value: Union[float, int], dtype: Any = None) -> Any:
        """Create array filled with specified value.
        
        Args:
            shape: Shape of the array
            fill_value: Value to fill array with
            dtype: Data type (default: adjusted based on precision setting)
            
        Returns:
            Array filled with specified value
        """
        if dtype is None:
            dtype = np.float32 if isinstance(fill_value, float) else np.int32
            
        # Adjust dtype based on precision setting
        adjusted_dtype = self._get_dtype_for_precision(dtype)
            
        if self.backend_type == BackendType.NUMPY:
            return np.full(shape, fill_value, dtype=adjusted_dtype)
        elif self.backend_type == BackendType.PYTORCH:
            torch_dtype = self._numpy_to_torch_dtype(adjusted_dtype)
            return torch.full(shape, fill_value, dtype=torch_dtype, device=self.device)
        elif self.backend_type == BackendType.CUPY:
            return cp.full(shape, fill_value, dtype=adjusted_dtype)
        elif self.backend_type == BackendType.WEBGPU:
            # For WebGPU, we create a NumPy array first, then transfer it to GPU
            cpu_array = np.full(shape, fill_value, dtype=adjusted_dtype)
            return self._numpy_to_wgpu(cpu_array)
    
    def array(self, data: Any, dtype: Any = None) -> Any:
        """Create array from data.
        
        Args:
            data: Data to create array from (list, tuple, NumPy array, etc.)
            dtype: Data type (default: inferred from data, then adjusted for precision)
            
        Returns:
            Array with backend-specific type
        """
        # Adjust dtype based on precision setting if provided
        adjusted_dtype = None if dtype is None else self._get_dtype_for_precision(dtype)
        
        if self.backend_type == BackendType.NUMPY:
            return np.array(data, dtype=adjusted_dtype)
        elif self.backend_type == BackendType.PYTORCH:
            torch_dtype = self._numpy_to_torch_dtype(adjusted_dtype) if adjusted_dtype is not None else None
            if isinstance(data, np.ndarray):
                # Apply precision conversion before moving to torch
                if adjusted_dtype is not None and data.dtype != adjusted_dtype:
                    data = data.astype(adjusted_dtype)
                return torch.from_numpy(data).to(self.device, dtype=torch_dtype)
            else:
                return torch.tensor(data, dtype=torch_dtype, device=self.device)
        elif self.backend_type == BackendType.CUPY:
            if isinstance(data, np.ndarray):
                # Apply precision conversion before moving to cupy
                if adjusted_dtype is not None and data.dtype != adjusted_dtype:
                    data = data.astype(adjusted_dtype)
                return cp.array(data)
            else:
                return cp.array(data, dtype=adjusted_dtype)
        elif self.backend_type == BackendType.WEBGPU:
            # For WebGPU, we create a NumPy array first, then transfer it to GPU
            cpu_array = np.array(data, dtype=adjusted_dtype)
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
            # Handle half-precision tensors by converting to float32
            if array.dtype == torch.float16:
                array = array.float()  # Convert to float32 for CPU
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
        # Apply precision conversion to data if needed
        if self.precision == PrecisionType.FP16 and isinstance(data, np.ndarray) and data.dtype == np.float32:
            data = data.astype(np.float16)
            
        if self.backend_type == BackendType.NUMPY:
            from scipy import sparse
            return sparse.csr_matrix((data, indices, indptr), shape=shape)
        elif self.backend_type == BackendType.PYTORCH:
            if not isinstance(data, torch.Tensor):
                # Apply precision conversion before moving to torch
                if self.precision == PrecisionType.FP16 and isinstance(data, np.ndarray) and data.dtype == np.float32:
                    data = data.astype(np.float16)
                data = torch.tensor(data, device=self.device)
                if self.precision == PrecisionType.FP16:
                    data = data.half()
            if not isinstance(indices, torch.Tensor):
                indices = torch.tensor(indices, dtype=torch.int64, device=self.device)
            if not isinstance(indptr, torch.Tensor):
                indptr = torch.tensor(indptr, dtype=torch.int64, device=self.device)
            return torch.sparse_csr_tensor(indptr, indices, data, size=shape, device=self.device)
        elif self.backend_type == BackendType.CUPY:
            # Apply precision conversion before moving to cupy
            if self.precision == PrecisionType.FP16 and isinstance(data, np.ndarray) and data.dtype == np.float32:
                data = data.astype(np.float16)
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
                # Apply precision conversion before moving to torch
                if self.precision == PrecisionType.FP16 and array.dtype == np.float32:
                    array = array.astype(np.float16)
                tensor = torch.from_numpy(array).to(self.device)
                if self.precision == PrecisionType.FP16 and tensor.dtype == torch.float32:
                    tensor = tensor.half()
                return tensor
            else:
                return array
        elif self.backend_type == BackendType.CUPY:
            if isinstance(array, np.ndarray):
                # Apply precision conversion before moving to cupy
                if self.precision == PrecisionType.FP16 and array.dtype == np.float32:
                    array = array.astype(np.float16)
                return cp.array(array)
            else:
                return array
        elif self.backend_type == BackendType.WEBGPU:
            if isinstance(array, np.ndarray):
                # Apply precision conversion before moving to WebGPU
                if self.precision == PrecisionType.FP16 and array.dtype == np.float32:
                    array = array.astype(np.float16)
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
    
    def synchronize(self):
        """Synchronize the device to ensure all operations are complete.
        
        This is a no-op for NumPy, but is important for GPU backends.
        """
        if self.backend_type == BackendType.PYTORCH and self.device == "cuda":
            torch.cuda.synchronize()
        elif self.backend_type == BackendType.CUPY:
            cp.cuda.stream.get_current_stream().synchronize()
        elif self.backend_type == BackendType.WEBGPU:
            # WebGPU is asynchronous - we'd need a synchronization primitive
            pass
    
    def matmul(self, a: Any, b: Any) -> Any:
        """Perform matrix multiplication.
        
        This method uses optimized implementations for each backend,
        and applies mixed precision optimizations when appropriate.
        
        Args:
            a: First matrix
            b: Second matrix
            
        Returns:
            Result of matrix multiplication
        """
        if self.backend_type == BackendType.NUMPY:
            return np.matmul(a, b)
        elif self.backend_type == BackendType.PYTORCH:
            if self.precision == PrecisionType.MIXED and hasattr(self, 'autocast'):
                with self.autocast():
                    return torch.matmul(a, b)
            else:
                return torch.matmul(a, b)
        elif self.backend_type == BackendType.CUPY:
            return cp.matmul(a, b)
        elif self.backend_type == BackendType.WEBGPU:
            # For WebGPU, we'd use a compute shader for matrix multiplication
            # This is a placeholder that falls back to NumPy
            a_np = self.to_numpy(a)
            b_np = self.to_numpy(b)
            result_np = np.matmul(a_np, b_np)
            return self._numpy_to_wgpu(result_np)
    
    def get_device_stats(self) -> Dict[str, Any]:
        """Get statistics about the current device.
        
        Returns:
            Dictionary of device statistics
        """
        stats = {
            "backend": self.backend_type.value,
            "precision": self.precision.value,
        }
        
        if self.backend_type == BackendType.PYTORCH and self.device == "cuda":
            stats["device"] = f"cuda:{torch.cuda.current_device()}"
            stats["device_name"] = torch.cuda.get_device_name(torch.cuda.current_device())
            stats["cuda_version"] = torch.version.cuda
            stats["memory_allocated_mb"] = torch.cuda.memory_allocated() / (1024 * 1024)
            stats["memory_cached_mb"] = torch.cuda.memory_reserved() / (1024 * 1024)
            stats["max_memory_mb"] = torch.cuda.get_device_properties(torch.cuda.current_device()).total_memory / (1024 * 1024)
        elif self.backend_type == BackendType.CUPY:
            device_id = cp.cuda.Device().id
            stats["device"] = f"cuda:{device_id}"
            stats["device_name"] = cp.cuda.runtime.getDeviceProperties(device_id)['name'].decode()
            stats["memory_allocated_mb"] = cp.cuda.Device().mem_info[1] / (1024 * 1024)  # Used memory
            stats["max_memory_mb"] = cp.cuda.Device().mem_info[0] / (1024 * 1024)  # Total memory
        elif self.backend_type == BackendType.WEBGPU:
            stats["device"] = "webgpu"
            if hasattr(self.adapter, 'request_adapter_info'):
                adapter_info = self.adapter.request_adapter_info()
                stats["device_name"] = adapter_info.description
        else:
            stats["device"] = "cpu"
            
        return stats 