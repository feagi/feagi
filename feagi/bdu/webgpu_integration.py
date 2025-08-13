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

import logging
import time

import numpy as np

# Try to import wgpu, but don't fail if it's not available
try:
    import wgpu

    WGPU_AVAILABLE = True
except ImportError:
    WGPU_AVAILABLE = False
    wgpu = None

"""WebGPU integration for the BDU.

This module provides WebGPU-based acceleration for neural processing,
enabling high-performance computation on modern GPUs.
"""

# WebGPU configuration constants
DEFAULT_WORKGROUP_SIZE = 256


class ConnectomeManagerWebGPU:
    """WebGPU-accelerated version of the ConnectomeManagerGPU.

    This class wraps ConnectomeManagerGPU with WebGPU acceleration for browser
    compatibility. It uses WGSL (WebGPU Shading Language) for core operations.
    """

    def __init__(self, connectome_manager_gpu):
        """Initialize WebGPU integration.

        Args:
            connectome_manager_gpu: A ConnectomeManagerGPU instance to accelerate

        Raises:
            ImportError: If wgpu is not available
        """
        if not WGPU_AVAILABLE:
            raise ImportError("wgpu is not available. Install with: pip install wgpu")

        self.connectome = connectome_manager_gpu
        self.device = None
        self.queue = None
        self.buffers = {}
        self.staging_buffers = {}
        self._shader_modules = {}
        self._compute_pipelines = {}

        # Performance tracking
        self.performance_metrics = {
            "last_transfer_time": 0.0,
            "total_transfer_time": 0.0,
            "transfer_count": 0,
            "last_compute_time": 0.0,
            "total_compute_time": 0.0,
            "compute_count": 0,
        }

        # Initialize WebGPU context
        if not self._init_webgpu():
            raise RuntimeError("Failed to initialize WebGPU context")

    def _init_webgpu(self):
        """Initialize WebGPU context.

        Returns:
            True if successful, False otherwise
        """
        try:
            # Request adapter and device
            adapter = wgpu.request_adapter(power_preference="high-performance")
            if not adapter:
                logging.error("Failed to get WebGPU adapter")
                return False

            self.device = adapter.request_device()
            if not self.device:
                logging.error("Failed to get WebGPU device")
                return False

            self.queue = self.device.queue

            # Initialize shaders and buffers
            self._init_shaders()
            self._allocate_buffers()

            logging.info("WebGPU context initialized successfully")
            return True

        except Exception as e:
            logging.error(f"Failed to initialize WebGPU: {e}")
            return False

    def _init_shaders(self):
        """Initialize shader modules for different operations."""
        # Neuron update shader (follows recommendations from the architecture document)
        neuron_shader = """
        """

        # Synapse processing shader (follows architecture document recommendations)
        synapse_shader = """
        """

        # Create shader modules
        self._shader_modules["neuron_operations"] = self.device.create_shader_module(
            code=neuron_shader
        )
        self._shader_modules["synapse_operations"] = self.device.create_shader_module(
            code=synapse_shader
        )

    def _allocate_buffers(self):
        """Allocate GPU buffers for neuron and synapse data with proper alignment."""
        """Update GPU buffers with current neuron data using staging buffers."""
        start_time = time.time()

        # Get neuron array
        neuron_array = self.connectome.neuron_array

        # Convert to NumPy arrays if they're not already
        membrane_potentials = self.connectome.neuron_array.backend.to_numpy(
            neuron_array.membrane_potentials
        )
        resting_potentials = self.connectome.neuron_array.backend.to_numpy(
            neuron_array.resting_potentials
        )
        thresholds = self.connectome.neuron_array.backend.to_numpy(
            neuron_array.thresholds
        )
        refractory_periods = self.connectome.neuron_array.backend.to_numpy(
            neuron_array.refractory_periods
        )
        refractory_counters = self.connectome.neuron_array.backend.to_numpy(
            neuron_array.refractory_counters
        )
        is_active = self.connectome.neuron_array.backend.to_numpy(
            neuron_array.is_active
        ).astype(np.uint32)
        valid_mask = self.connectome.neuron_array.backend.to_numpy(
            neuron_array.valid_mask
        ).astype(np.uint32)

        # Ensure arrays are contiguous and properly aligned
        membrane_potentials = np.ascontiguousarray(
            membrane_potentials, dtype=np.float32
        )
        resting_potentials = np.ascontiguousarray(resting_potentials, dtype=np.float32)
        thresholds = np.ascontiguousarray(thresholds, dtype=np.float32)
        refractory_periods = np.ascontiguousarray(refractory_periods, dtype=np.int32)
        refractory_counters = np.ascontiguousarray(refractory_counters, dtype=np.int32)
        is_active = np.ascontiguousarray(is_active, dtype=np.uint32)
        valid_mask = np.ascontiguousarray(valid_mask, dtype=np.uint32)

        # Use staging buffers for efficient transfer
        # First map the staging buffers
        staging_membrane = self.staging_buffers["membrane_potentials"]
        staging_membrane.map_write()
        staging_membrane.write_mapped_view().reshape(membrane_potentials.shape)[:] = (
            membrane_potentials
        )
        staging_membrane.unmap()

        staging_resting = self.staging_buffers["resting_potentials"]
        staging_resting.map_write()
        staging_resting.write_mapped_view().reshape(resting_potentials.shape)[:] = (
            resting_potentials
        )
        staging_resting.unmap()

        staging_thresholds = self.staging_buffers["thresholds"]
        staging_thresholds.map_write()
        staging_thresholds.write_mapped_view().reshape(thresholds.shape)[:] = thresholds
        staging_thresholds.unmap()

        staging_ref_periods = self.staging_buffers["refractory_periods"]
        staging_ref_periods.map_write()
        staging_ref_periods.write_mapped_view().reshape(refractory_periods.shape)[:] = (
            refractory_periods
        )
        staging_ref_periods.unmap()

        staging_ref_counters = self.staging_buffers["refractory_counters"]
        staging_ref_counters.map_write()
        staging_ref_counters.write_mapped_view().reshape(refractory_counters.shape)[
            :
        ] = refractory_counters
        staging_ref_counters.unmap()

        staging_active = self.staging_buffers["is_active"]
        staging_active.map_write()
        staging_active.write_mapped_view().reshape(is_active.shape)[:] = is_active
        staging_active.unmap()

        staging_valid = self.staging_buffers["valid_mask"]
        staging_valid.map_write()
        staging_valid.write_mapped_view().reshape(valid_mask.shape)[:] = valid_mask
        staging_valid.unmap()

        # Create command encoder to copy from staging buffers to device buffers
        encoder = self.device.create_command_encoder(label="Neuron Data Upload Encoder")

        # Copy all buffer data
        encoder.copy_buffer_to_buffer(
            self.staging_buffers["membrane_potentials"],
            0,
            self.buffers["membrane_potentials"],
            0,
            membrane_potentials.nbytes,
        )
        encoder.copy_buffer_to_buffer(
            self.staging_buffers["resting_potentials"],
            0,
            self.buffers["resting_potentials"],
            0,
            resting_potentials.nbytes,
        )
        encoder.copy_buffer_to_buffer(
            self.staging_buffers["thresholds"],
            0,
            self.buffers["thresholds"],
            0,
            thresholds.nbytes,
        )
        encoder.copy_buffer_to_buffer(
            self.staging_buffers["refractory_periods"],
            0,
            self.buffers["refractory_periods"],
            0,
            refractory_periods.nbytes,
        )
        encoder.copy_buffer_to_buffer(
            self.staging_buffers["refractory_counters"],
            0,
            self.buffers["refractory_counters"],
            0,
            refractory_counters.nbytes,
        )
        encoder.copy_buffer_to_buffer(
            self.staging_buffers["is_active"],
            0,
            self.buffers["is_active"],
            0,
            is_active.nbytes,
        )
        encoder.copy_buffer_to_buffer(
            self.staging_buffers["valid_mask"],
            0,
            self.buffers["valid_mask"],
            0,
            valid_mask.nbytes,
        )

        # Submit commands to GPU queue
        self.device.queue.submit([encoder.finish()])

        # Update performance metrics
        end_time = time.time()
        transfer_time = end_time - start_time
        self.performance_metrics["last_transfer_time"] = transfer_time
        self.performance_metrics["total_transfer_time"] += transfer_time
        self.performance_metrics["transfer_count"] += 1

    def update_synapse_data(self):
        """Update GPU buffers with current synapse data."""
        """Create bind groups for the compute pipelines."""
        # Neuron operations bind group
        neuron_bind_group_layout = self.device.create_bind_group_layout(
            entries=[
                # Simulation parameters
                {
                    "binding": 0,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.UNIFORM},
                },
                # Neuron data
                {
                    "binding": 1,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.STORAGE},
                },
                # Fired neurons
                {
                    "binding": 2,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.STORAGE},
                },
            ],
            label="Neuron Operations Bind Group Layout",
        )

        # Create bind group with actual buffers
        self.bind_groups["neuron_operations"] = self.device.create_bind_group(
            layout=neuron_bind_group_layout,
            entries=[
                {"binding": 0, "resource": {"buffer": self.buffers["params"]}},
                {
                    "binding": 1,
                    "resource": {"buffer": self.buffers["membrane_potentials"]},
                },
                {"binding": 2, "resource": {"buffer": self.buffers["fire_list"]}},
            ],
            label="Neuron Operations Bind Group",
        )

        # Synapse operations bind group layout
        synapse_bind_group_layout = self.device.create_bind_group_layout(
            entries=[
                # Batch size
                {
                    "binding": 0,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.UNIFORM},
                },
                # Fire list
                {
                    "binding": 1,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.STORAGE},
                },
                # Synapse data (CSR format)
                {
                    "binding": 2,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.STORAGE},
                },
                # Neuron data
                {
                    "binding": 3,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.STORAGE},
                },
            ],
            label="Synapse Operations Bind Group Layout",
        )

        # Create bind group with actual buffers
        self.bind_groups["synapse_operations"] = self.device.create_bind_group(
            layout=synapse_bind_group_layout,
            entries=[
                {"binding": 0, "resource": {"buffer": self.buffers["batch_size"]}},
                {"binding": 1, "resource": {"buffer": self.buffers["fire_list"]}},
                {
                    "binding": 2,
                    "resource": {"buffer": self.buffers["row_indices"]},
                },  # CSR indptr
                {
                    "binding": 3,
                    "resource": {"buffer": self.buffers["membrane_potentials"]},
                },
            ],
            label="Synapse Operations Bind Group",
        )

    def _create_compute_pipelines(self):
        """Create compute pipelines for neuron and synapse operations."""
        """Update membrane potentials using WebGPU acceleration."""
        # Update params buffer with current timestep
        self.device.queue.write_buffer(
            self.buffers["params"],
            0,
            np.array(
                [self.connectome.max_neurons, self.connectome.current_timestep],
                dtype=np.uint32,
            ),
        )

        # Reset fired count to 0
        self.device.queue.write_buffer(
            self.buffers["fired_count"], 0, np.array([0], dtype=np.uint32)
        )

        # Update GPU buffers with current CPU data
        self.update_neuron_data()
        self.update_synapse_data()

        # Create command encoder
        encoder = self.device.create_command_encoder(label="Neuron Update Encoder")

        # Update neurons compute pass
        neuron_pass = encoder.begin_compute_pass()
        neuron_pass.set_pipeline(self.pipelines["neuron_update"])
        neuron_pass.set_bind_group(0, self.bind_groups["neuron_operations"])

        # Dispatch compute shader
        # 256 is the workgroup size defined in shader
        workgroup_count = (self.connectome.max_neurons + 255) // 256
        neuron_pass.dispatch_workgroups(workgroup_count)
        neuron_pass.end()

        # Submit command buffer for neuron update
        self.device.queue.submit([encoder.finish()])

        # Now create encoder for signal propagation
        encoder = self.device.create_command_encoder(
            label="Synapse Propagation Encoder"
        )

        # Signal propagation compute pass
        # First read back fired count to determine dispatch size
        fired_count_staging = self.device.create_buffer(
            size=4,
            usage=wgpu.BufferUsages.MAP_READ | wgpu.BufferUsages.COPY_DST,
            label="Fired Count Staging Buffer",
        )

        # Copy fired count to staging buffer
        encoder.copy_buffer_to_buffer(
            self.buffers["fired_count"], 0, fired_count_staging, 0, 4
        )
        self.device.queue.submit([encoder.finish()])

        # Map buffer and read data
        fired_count_staging.map_async()
        fired_count_data = np.frombuffer(
            fired_count_staging.get_mapped_range(), dtype=np.uint32
        )
        fired_count = int(fired_count_data[0])
        fired_count_staging.unmap()

        # Only proceed with signal propagation if there are fired neurons
        if fired_count > 0:
            # Update batch size for synapse propagation
            self.device.queue.write_buffer(
                self.buffers["batch_size"], 0, np.array([fired_count], dtype=np.uint32)
            )

            # Create command encoder for synapse propagation
            encoder = self.device.create_command_encoder(
                label="Synapse Propagation Encoder"
            )

            # Signal propagation compute pass
            propagation_pass = encoder.begin_compute_pass()
            propagation_pass.set_pipeline(self.pipelines["synapse_propagation"])
            propagation_pass.set_bind_group(0, self.bind_groups["synapse_operations"])

            # Dispatch compute shader with appropriate workgroup count
            # Each workgroup handles a batch of synapses
            workgroup_count = (fired_count + 255) // 256
            propagation_pass.dispatch_workgroups(workgroup_count, 1, 1)
            propagation_pass.end()

            # Submit command buffer for synapse propagation
            self.device.queue.submit([encoder.finish()])

        # Read back fired neuron IDs
        fired_neurons_staging = self.device.create_buffer(
            size=min(fired_count, 1_000_000) * 4,
            usage=wgpu.BufferUsages.MAP_READ | wgpu.BufferUsages.COPY_DST,
            label="Fired Neurons Staging Buffer",
        )

        # Copy fired neurons to staging buffer
        encoder = self.device.create_command_encoder()
        encoder.copy_buffer_to_buffer(
            self.buffers["fire_list"],
            0,
            fired_neurons_staging,
            0,
            min(fired_count, 1_000_000) * 4,
        )
        self.device.queue.submit([encoder.finish()])

        # Map buffer and read data
        fired_neurons_staging.map_async()
        fired_neurons_data = np.frombuffer(
            fired_neurons_staging.get_mapped_range(),
            dtype=np.uint32,
            count=min(fired_count, 1_000_000),
        )

        # OPTIMIZED: Use vectorized conversion instead of dictionary comprehension
        fired_neuron_ids = (
            self.connectome.neuron_array.vectorized_indices_to_neuron_ids(
                fired_neurons_data.astype(np.int64), filter_invalid=True
            ).tolist()
        )

        fired_neurons_staging.unmap()

        # Update FCL manager
        self.connectome.fcl_manager.register_event(
            self.connectome.current_timestep, fired_neuron_ids
        )

        # Increment timestep
        self.connectome.current_timestep += 1

        # Record performance metrics
        self.performance_metrics["compute_count"] += 1

        return fired_neuron_ids


def is_webgpu_available():
    """Check if WebGPU is available on this system."""
    return WGPU_AVAILABLE
