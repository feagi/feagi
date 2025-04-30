"""
WebGPU backend implementation for FEAGI.

This module provides a WebGPU-based backend for tensor operations in FEAGI,
enabling GPU acceleration across different platforms without vendor lock-in.
"""

import logging
import numpy as np
from typing import Optional, Tuple, Union, List, Dict, Any

try:
    import wgpu
    WEBGPU_AVAILABLE = True
except ImportError:
    WEBGPU_AVAILABLE = False

from feagi.core.backend.interface import BackendInterface, BackendType, register_backend

logger = logging.getLogger(__name__)

class WebGPUTensor:
    """A tensor implementation using WebGPU for storage and computation."""
    
    def __init__(self, shape: Tuple[int, ...], buffer: Any, dtype: np.dtype = np.float32):
        self.shape = shape
        self.buffer = buffer
        self.dtype = dtype
        self.size = int(np.prod(shape))
        
    def __repr__(self):
        return f"WebGPUTensor(shape={self.shape}, dtype={self.dtype})"


class WebGPUBackend(BackendInterface):
    """
    WebGPU backend implementation for FEAGI.
    
    This backend leverages WebGPU for cross-platform GPU acceleration,
    providing efficient tensor operations without vendor lock-in.
    """
    
    def __init__(self):
        self.device = None
        self.adapter = None
        self.initialized = False
        self._shader_modules = {}
        
    def initialize(self) -> bool:
        """Initialize the WebGPU backend and select an appropriate device."""
        if not WEBGPU_AVAILABLE:
            logger.error("WebGPU module is not available. Cannot initialize WebGPU backend.")
            return False
        
        try:
            # Use wgpu.gpu for newer versions of wgpu
            if hasattr(wgpu, 'gpu') and hasattr(wgpu.gpu, 'request_adapter'):
                self.adapter = wgpu.gpu.request_adapter(power_preference="high-performance")
            # Fallback to direct access for older versions
            elif hasattr(wgpu, 'request_adapter'):
                self.adapter = wgpu.request_adapter(power_preference="high-performance")
            else:
                logger.error("WebGPU API not found in installed wgpu package")
                return False
                
            self.device = self.adapter.request_device()
            
            # Log device information
            adapter_info = self.adapter.request_adapter_info()
            logger.info(f"WebGPU adapter: {adapter_info.get('name', 'Unknown')}")
            logger.info(f"WebGPU device: {self.device}")
            
            # Initialize common shader modules
            self._init_shader_modules()
            
            self.initialized = True
            return True
        except Exception as e:
            logger.error(f"Failed to initialize WebGPU backend: {e}")
            return False
    
    def shutdown(self) -> None:
        """Shutdown the WebGPU backend and release resources."""
        if self.device:
            self.device.destroy()
            self.device = None
        self.initialized = False
        logger.info("WebGPU backend shut down")
    
    def supports_capability(self, capability: str) -> bool:
        """Check if this backend supports a specific capability."""
        supported_capabilities = {
            "matrix_multiplication": True,
            "element_wise_operations": True,
            "random_generation": True,
            "bitmap_operations": True,
            "sparse_operations": True,
            "batched_operations": True,
        }
        return supported_capabilities.get(capability, False)
    
    def create_tensor(self, shape: Tuple[int, ...], dtype: np.dtype = np.float32) -> WebGPUTensor:
        """Create a new tensor with the specified shape."""
        if not self.initialized:
            raise RuntimeError("WebGPU backend is not initialized")
        
        size = int(np.prod(shape))
        
        # Map numpy dtypes to WebGPU format
        format_map = {
            np.float32: "float32",
            np.float16: "float16",
            np.int32: "int32",
            np.uint32: "uint32",
            np.int16: "int16",
            np.uint16: "uint16",
            np.int8: "int8",
            np.uint8: "uint8",
        }
        
        buffer_format = format_map.get(np.dtype(dtype), "float32")
        
        # Create a buffer on the GPU
        buffer_size = size * np.dtype(dtype).itemsize
        buffer = self.device.create_buffer(
            size=buffer_size,
            usage=wgpu.BufferUsage.STORAGE | wgpu.BufferUsage.COPY_SRC | wgpu.BufferUsage.COPY_DST,
        )
        
        # Initialize buffer with zeros
        temp_data = np.zeros(size, dtype=dtype)
        self.device.queue.write_buffer(buffer, 0, temp_data)
        
        return WebGPUTensor(shape, buffer, dtype)
    
    def from_numpy(self, array: np.ndarray) -> WebGPUTensor:
        """Create a tensor from a NumPy array."""
        if not self.initialized:
            raise RuntimeError("WebGPU backend is not initialized")
        
        tensor = self.create_tensor(array.shape, array.dtype)
        
        # Copy data from CPU to GPU
        self.device.queue.write_buffer(tensor.buffer, 0, array.ravel())
        
        return tensor
    
    def to_numpy(self, tensor: WebGPUTensor) -> np.ndarray:
        """Convert a WebGPU tensor to a NumPy array."""
        if not self.initialized:
            raise RuntimeError("WebGPU backend is not initialized")
        
        # Create a staging buffer for reading data back to CPU
        staging_buffer = self.device.create_buffer(
            size=tensor.buffer.size,
            usage=wgpu.BufferUsage.MAP_READ | wgpu.BufferUsage.COPY_DST,
        )
        
        # Copy from tensor buffer to staging buffer
        encoder = self.device.create_command_encoder()
        encoder.copy_buffer_to_buffer(
            tensor.buffer, 0, staging_buffer, 0, tensor.buffer.size
        )
        self.device.queue.submit([encoder.finish()])
        
        # Map the staging buffer to CPU memory
        staging_buffer.map()
        mapped_data = staging_buffer.read_data()
        
        # Convert to numpy array
        np_array = np.frombuffer(mapped_data, dtype=tensor.dtype)
        np_array = np_array.reshape(tensor.shape)
        
        # Unmap and clean up
        staging_buffer.unmap()
        
        return np_array
    
    def synchronize(self) -> None:
        """Ensure all pending operations are complete."""
        if self.initialized and self.device:
            self.device.queue.submit([])  # Submit empty command list to synchronize
    
    def _init_shader_modules(self) -> None:
        """Initialize common shader modules for reuse."""
        # Simple elementwise operations shader
        elementwise_shader = """
        @group(0) @binding(0) var<storage, read> input1: array<f32>;
        @group(0) @binding(1) var<storage, read> input2: array<f32>;
        @group(0) @binding(2) var<storage, read_write> output: array<f32>;
        
        @compute @workgroup_size(256)
        fn add(@builtin(global_invocation_id) id: vec3<u32>) {
            let index = id.x;
            if (index >= arrayLength(&output)) {
                return;
            }
            output[index] = input1[index] + input2[index];
        }
        
        @compute @workgroup_size(256)
        fn subtract(@builtin(global_invocation_id) id: vec3<u32>) {
            let index = id.x;
            if (index >= arrayLength(&output)) {
                return;
            }
            output[index] = input1[index] - input2[index];
        }
        
        @compute @workgroup_size(256)
        fn multiply(@builtin(global_invocation_id) id: vec3<u32>) {
            let index = id.x;
            if (index >= arrayLength(&output)) {
                return;
            }
            output[index] = input1[index] * input2[index];
        }
        """
        
        self._shader_modules["elementwise"] = self.device.create_shader_module(code=elementwise_shader)
        
        # Neuron dynamics shader for burst engine
        neuron_shader = """
        struct NeuronParams {
            resting_potential: f32,
            threshold: f32,
            refractory_period: u32,
            current_step: u32,
        }
        
        @group(0) @binding(0) var<storage, read_write> membrane_potentials: array<f32>;
        @group(0) @binding(1) var<storage, read_write> fire_candidates: array<u32>;
        @group(0) @binding(2) var<storage, read> neuron_params: NeuronParams;
        @group(0) @binding(3) var<storage, read> synaptic_inputs: array<f32>;
        @group(0) @binding(4) var<storage, read_write> last_fired: array<u32>;
        
        @compute @workgroup_size(256)
        fn update_neurons(@builtin(global_invocation_id) id: vec3<u32>) {
            let neuron_id = id.x;
            if (neuron_id >= arrayLength(&membrane_potentials)) {
                return;
            }
            
            // Check if neuron is in refractory period
            if (neuron_params.current_step - last_fired[neuron_id] < neuron_params.refractory_period) {
                // In refractory period, decay towards resting potential
                membrane_potentials[neuron_id] = 
                    0.9 * membrane_potentials[neuron_id] + 0.1 * neuron_params.resting_potential;
                fire_candidates[neuron_id] = 0u;
                return;
            }
            
            // Update membrane potential with synaptic input
            membrane_potentials[neuron_id] += synaptic_inputs[neuron_id];
            
            // Check for firing
            if (membrane_potentials[neuron_id] >= neuron_params.threshold) {
                // Neuron fires
                fire_candidates[neuron_id] = 1u;
                last_fired[neuron_id] = neuron_params.current_step;
                // Reset membrane potential
                membrane_potentials[neuron_id] = neuron_params.resting_potential;
            } else {
                // Neuron doesn't fire
                fire_candidates[neuron_id] = 0u;
                // Decay membrane potential slightly towards resting potential
                membrane_potentials[neuron_id] = 
                    0.95 * membrane_potentials[neuron_id] + 0.05 * neuron_params.resting_potential;
            }
        }
        """
        
        self._shader_modules["neuron_dynamics"] = self.device.create_shader_module(code=neuron_shader)


# Register backend if WebGPU is available
if WEBGPU_AVAILABLE:
    try:
        register_backend(BackendType.WEBGPU, WebGPUBackend)
        logger.info("WebGPU backend registered successfully")
    except Exception as e:
        logger.error(f"Failed to register WebGPU backend: {e}")
else:
    logger.warning("WebGPU is not available. WebGPU backend will not be registered.") 