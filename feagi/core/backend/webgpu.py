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
WebGPU backend implementation for FEAGI.

This module provides a WebGPU-based backend for tensor operations in FEAGI,
enabling GPU acceleration across different platforms without vendor lock-in.
"""

from feagi.utils.logger import setup_logger

logger = setup_logger()
from typing import Any, Tuple

import numpy as np

try:
    import wgpu

    WEBGPU_AVAILABLE = True
except ImportError:
    WEBGPU_AVAILABLE = False

from feagi.core.backend.interface import BackendInterface, BackendType, register_backend


class WebGPUTensor:
    """A tensor implementation using WebGPU for storage and computation."""

    def __init__(
        self, shape: Tuple[int, ...], buffer: Any, dtype: np.dtype = np.float32
    ):
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
            logger.error(
                "WebGPU module is not available. Cannot initialize WebGPU backend."
            )
            return False

        try:
            # Use wgpu.gpu for newer versions of wgpu
            if hasattr(wgpu, "gpu") and hasattr(wgpu.gpu, "request_adapter_sync"):
                self.adapter = wgpu.gpu.request_adapter_sync(
                    power_preference="high-performance"
                )
            # Fallback to direct access for older versions
            elif hasattr(wgpu, "request_adapter_sync"):
                self.adapter = wgpu.request_adapter_sync(
                    power_preference="high-performance"
                )
            else:
                logger.error("WebGPU API not found in installed wgpu package")
                return False

            # Use request_device_sync instead of request_device
            if hasattr(self.adapter, "request_device_sync"):
                self.device = self.adapter.request_device_sync()
            else:
                self.device = self.adapter.request_device()

            # Log device information
            logger.info("WebGPU adapter successfully loaded")
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

    def create_tensor(
        self, shape: Tuple[int, ...], dtype: np.dtype = np.float32
    ) -> WebGPUTensor:
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
            usage=wgpu.BufferUsage.STORAGE
            | wgpu.BufferUsage.COPY_SRC
            | wgpu.BufferUsage.COPY_DST,
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

        @compute @workgroup_size(256)
        fn divide(@builtin(global_invocation_id) id: vec3<u32>) {
            let index = id.x;
            if (index >= arrayLength(&output)) {
                return;
            }
            if (input2[index] != 0.0) {
                output[index] = input1[index] / input2[index];
            } else {
                output[index] = 0.0;
            }
        }
        """

        self._shader_modules["elementwise"] = self.device.create_shader_module(
            code=elementwise_shader
        )

        # Bitmap operations shader
        bitmap_shader = """
        @group(0) @binding(0) var<storage, read> bitmap1: array<u32>;
        @group(0) @binding(1) var<storage, read> bitmap2: array<u32>;
        @group(0) @binding(2) var<storage, read_write> output: array<atomic<u32>>;

        @compute @workgroup_size(256)
        fn bitmap_or(@builtin(global_invocation_id) id: vec3<u32>) {
            let index = id.x;
            if (index >= arrayLength(&output)) {
                return;
            }
            output[index] = bitmap1[index] | bitmap2[index];
        }

        @compute @workgroup_size(256)
        fn bitmap_and(@builtin(global_invocation_id) id: vec3<u32>) {
            let index = id.x;
            if (index >= arrayLength(&output)) {
                return;
            }
            output[index] = bitmap1[index] & bitmap2[index];
        }

        @compute @workgroup_size(256)
        fn bitmap_xor(@builtin(global_invocation_id) id: vec3<u32>) {
            let index = id.x;
            if (index >= arrayLength(&output)) {
                return;
            }
            output[index] = bitmap1[index] ^ bitmap2[index];
        }

        @compute @workgroup_size(256)
        fn bitmap_subtract(@builtin(global_invocation_id) id: vec3<u32>) {
            let index = id.x;
            if (index >= arrayLength(&output)) {
                return;
            }
            output[index] = bitmap1[index] & ~bitmap2[index];
        }

        @compute @workgroup_size(256)
        fn clear_bitmap(@builtin(global_invocation_id) id: vec3<u32>) {
            let index = id.x;
            if (index >= arrayLength(&output)) {
                return;
            }
            output[index] = 0u;
        }

        @compute @workgroup_size(256)
        fn bitmap_set_bit(@builtin(global_invocation_id) id: vec3<u32>) {
            let neuron_id = id.x;
            if (neuron_id >= arrayLength(&bitmap1)) {
                return;
            }

            // bitmap1 contains the neuron IDs to set
            // bitmap2 contains the bitmap to update
            let chunk_index = neuron_id / 32u;
            let bit_index = neuron_id % 32u;
            let mask = 1u << bit_index;

            // Atomic OR to set the bit
            atomicOr(&output[chunk_index], mask);
        }
        """

        self._shader_modules["bitmap_ops"] = self.device.create_shader_module(
            code=bitmap_shader
        )

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

        self._shader_modules["neuron_dynamics"] = self.device.create_shader_module(
            code=neuron_shader
        )

    def bitmap_or(self, bitmap1: WebGPUTensor, bitmap2: WebGPUTensor) -> WebGPUTensor:
        """Perform bitwise OR operation between two bitmaps."""
        if not self.initialized:
            raise RuntimeError("WebGPU backend is not initialized")

        if bitmap1.shape != bitmap2.shape:
            raise ValueError("Input bitmaps must have the same shape")

        # Create output tensor
        output = self.create_tensor(bitmap1.shape, bitmap1.dtype)

        # Create bind group layout
        bind_group_layout = self.device.create_bind_group_layout(
            entries=[
                {
                    "binding": 0,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.read_only_storage},
                },
                {
                    "binding": 1,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.read_only_storage},
                },
                {
                    "binding": 2,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.storage},
                },
            ]
        )

        # Create pipeline layout
        pipeline_layout = self.device.create_pipeline_layout(
            bind_group_layouts=[bind_group_layout]
        )

        # Create compute pipeline
        compute_pipeline = self.device.create_compute_pipeline(
            layout=pipeline_layout,
            compute={
                "module": self._shader_modules["bitmap_ops"],
                "entry_point": "bitmap_or",
            },
        )

        # Create bind group
        bind_group = self.device.create_bind_group(
            layout=bind_group_layout,
            entries=[
                {"binding": 0, "resource": {"buffer": bitmap1.buffer}},
                {"binding": 1, "resource": {"buffer": bitmap2.buffer}},
                {"binding": 2, "resource": {"buffer": output.buffer}},
            ],
        )

        # Create command encoder
        encoder = self.device.create_command_encoder()

        # Begin compute pass
        compute_pass = encoder.begin_compute_pass()
        compute_pass.set_pipeline(compute_pipeline)
        compute_pass.set_bind_group(0, bind_group)

        # Dispatch compute shader
        workgroup_count = (bitmap1.size + 255) // 256
        compute_pass.dispatch(workgroup_count)

        # End compute pass
        compute_pass.end()

        # Submit commands
        self.device.queue.submit([encoder.finish()])

        return output

    def bitmap_and(self, bitmap1: WebGPUTensor, bitmap2: WebGPUTensor) -> WebGPUTensor:
        """Perform bitwise AND operation between two bitmaps."""
        if not self.initialized:
            raise RuntimeError("WebGPU backend is not initialized")

        if bitmap1.shape != bitmap2.shape:
            raise ValueError("Input bitmaps must have the same shape")

        # Create output tensor
        output = self.create_tensor(bitmap1.shape, bitmap1.dtype)

        # Create bind group layout
        bind_group_layout = self.device.create_bind_group_layout(
            entries=[
                {
                    "binding": 0,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.read_only_storage},
                },
                {
                    "binding": 1,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.read_only_storage},
                },
                {
                    "binding": 2,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.storage},
                },
            ]
        )

        # Create pipeline layout
        pipeline_layout = self.device.create_pipeline_layout(
            bind_group_layouts=[bind_group_layout]
        )

        # Create compute pipeline
        compute_pipeline = self.device.create_compute_pipeline(
            layout=pipeline_layout,
            compute={
                "module": self._shader_modules["bitmap_ops"],
                "entry_point": "bitmap_and",
            },
        )

        # Create bind group
        bind_group = self.device.create_bind_group(
            layout=bind_group_layout,
            entries=[
                {"binding": 0, "resource": {"buffer": bitmap1.buffer}},
                {"binding": 1, "resource": {"buffer": bitmap2.buffer}},
                {"binding": 2, "resource": {"buffer": output.buffer}},
            ],
        )

        # Create command encoder
        encoder = self.device.create_command_encoder()

        # Begin compute pass
        compute_pass = encoder.begin_compute_pass()
        compute_pass.set_pipeline(compute_pipeline)
        compute_pass.set_bind_group(0, bind_group)

        # Dispatch compute shader
        workgroup_count = (bitmap1.size + 255) // 256
        compute_pass.dispatch(workgroup_count)

        # End compute pass
        compute_pass.end()

        # Submit commands
        self.device.queue.submit([encoder.finish()])

        return output

    def bitmap_xor(self, bitmap1: WebGPUTensor, bitmap2: WebGPUTensor) -> WebGPUTensor:
        """Perform bitwise XOR operation between two bitmaps."""
        if not self.initialized:
            raise RuntimeError("WebGPU backend is not initialized")

        if bitmap1.shape != bitmap2.shape:
            raise ValueError("Input bitmaps must have the same shape")

        # Create output tensor
        output = self.create_tensor(bitmap1.shape, bitmap1.dtype)

        # Create bind group layout
        bind_group_layout = self.device.create_bind_group_layout(
            entries=[
                {
                    "binding": 0,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.read_only_storage},
                },
                {
                    "binding": 1,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.read_only_storage},
                },
                {
                    "binding": 2,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.storage},
                },
            ]
        )

        # Create pipeline layout
        pipeline_layout = self.device.create_pipeline_layout(
            bind_group_layouts=[bind_group_layout]
        )

        # Create compute pipeline
        compute_pipeline = self.device.create_compute_pipeline(
            layout=pipeline_layout,
            compute={
                "module": self._shader_modules["bitmap_ops"],
                "entry_point": "bitmap_xor",
            },
        )

        # Create bind group
        bind_group = self.device.create_bind_group(
            layout=bind_group_layout,
            entries=[
                {"binding": 0, "resource": {"buffer": bitmap1.buffer}},
                {"binding": 1, "resource": {"buffer": bitmap2.buffer}},
                {"binding": 2, "resource": {"buffer": output.buffer}},
            ],
        )

        # Create command encoder
        encoder = self.device.create_command_encoder()

        # Begin compute pass
        compute_pass = encoder.begin_compute_pass()
        compute_pass.set_pipeline(compute_pipeline)
        compute_pass.set_bind_group(0, bind_group)

        # Dispatch compute shader
        workgroup_count = (bitmap1.size + 255) // 256
        compute_pass.dispatch(workgroup_count)

        # End compute pass
        compute_pass.end()

        # Submit commands
        self.device.queue.submit([encoder.finish()])

        return output

    def bitmap_subtract(
        self, bitmap1: WebGPUTensor, bitmap2: WebGPUTensor
    ) -> WebGPUTensor:
        """Perform bitmap subtraction (remove elements in bitmap2 from bitmap1)."""
        if not self.initialized:
            raise RuntimeError("WebGPU backend is not initialized")

        if bitmap1.shape != bitmap2.shape:
            raise ValueError("Input bitmaps must have the same shape")

        # Create output tensor
        output = self.create_tensor(bitmap1.shape, bitmap1.dtype)

        # Create bind group layout
        bind_group_layout = self.device.create_bind_group_layout(
            entries=[
                {
                    "binding": 0,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.read_only_storage},
                },
                {
                    "binding": 1,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.read_only_storage},
                },
                {
                    "binding": 2,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.storage},
                },
            ]
        )

        # Create pipeline layout
        pipeline_layout = self.device.create_pipeline_layout(
            bind_group_layouts=[bind_group_layout]
        )

        # Create compute pipeline
        compute_pipeline = self.device.create_compute_pipeline(
            layout=pipeline_layout,
            compute={
                "module": self._shader_modules["bitmap_ops"],
                "entry_point": "bitmap_subtract",
            },
        )

        # Create bind group
        bind_group = self.device.create_bind_group(
            layout=bind_group_layout,
            entries=[
                {"binding": 0, "resource": {"buffer": bitmap1.buffer}},
                {"binding": 1, "resource": {"buffer": bitmap2.buffer}},
                {"binding": 2, "resource": {"buffer": output.buffer}},
            ],
        )

        # Create command encoder
        encoder = self.device.create_command_encoder()

        # Begin compute pass
        compute_pass = encoder.begin_compute_pass()
        compute_pass.set_pipeline(compute_pipeline)
        compute_pass.set_bind_group(0, bind_group)

        # Dispatch compute shader
        workgroup_count = (bitmap1.size + 255) // 256
        compute_pass.dispatch(workgroup_count)

        # End compute pass
        compute_pass.end()

        # Submit commands
        self.device.queue.submit([encoder.finish()])

        return output

    def bitmap_set_bits(
        self, bitmap: WebGPUTensor, neuron_ids: WebGPUTensor
    ) -> WebGPUTensor:
        """Set bits in the bitmap for each neuron ID."""
        if not self.initialized:
            raise RuntimeError("WebGPU backend is not initialized")

        # Create output tensor (copy of input bitmap)
        output = self.from_numpy(self.to_numpy(bitmap))

        # Create bind group layout
        bind_group_layout = self.device.create_bind_group_layout(
            entries=[
                {
                    "binding": 0,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.read_only_storage},
                },
                {
                    "binding": 1,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.read_only_storage},
                },
                {
                    "binding": 2,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.storage},
                },
            ]
        )

        # Create pipeline layout
        pipeline_layout = self.device.create_pipeline_layout(
            bind_group_layouts=[bind_group_layout]
        )

        # Create compute pipeline
        compute_pipeline = self.device.create_compute_pipeline(
            layout=pipeline_layout,
            compute={
                "module": self._shader_modules["bitmap_ops"],
                "entry_point": "bitmap_set_bit",
            },
        )

        # Create bind group
        bind_group = self.device.create_bind_group(
            layout=bind_group_layout,
            entries=[
                {"binding": 0, "resource": {"buffer": neuron_ids.buffer}},
                {"binding": 1, "resource": {"buffer": bitmap.buffer}},
                {"binding": 2, "resource": {"buffer": output.buffer}},
            ],
        )

        # Create command encoder
        encoder = self.device.create_command_encoder()

        # Begin compute pass
        compute_pass = encoder.begin_compute_pass()
        compute_pass.set_pipeline(compute_pipeline)
        compute_pass.set_bind_group(0, bind_group)

        # Dispatch compute shader - one workgroup per neuron ID
        workgroup_count = (neuron_ids.size + 255) // 256
        compute_pass.dispatch(workgroup_count)

        # End compute pass
        compute_pass.end()

        # Submit commands
        self.device.queue.submit([encoder.finish()])

        return output


# Register backend if WebGPU is available
if WEBGPU_AVAILABLE:
    try:
        register_backend(BackendType.WEBGPU, WebGPUBackend)
        logger.info("WebGPU backend registered successfully")
    except Exception as e:
        logger.error(f"Failed to register WebGPU backend: {e}")
else:
    logger.warning("WebGPU is not available. WebGPU backend will not be registered.")
