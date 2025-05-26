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

"""WebGPU integration for GPU-optimized ConnectomeManager.

This module provides WebGPU-based acceleration for the ConnectomeManagerGPU,
enabling browser-compatible GPU acceleration for neural network operations.
"""

import logging
import numpy as np
from typing import Dict, List, Tuple, Optional, Union, Any
import os
import time

# Try to import wgpu, but don't fail if it's not available
try:
    import wgpu
    WEBGPU_AVAILABLE = True
except ImportError:
    WEBGPU_AVAILABLE = False

logger = logging.getLogger(__name__)

# Memory alignment for optimal GPU transfer (64 bytes for AVX-512 compatibility)
MEMORY_ALIGNMENT = 64

# Recommended workgroup size for WGSL shaders (most GPUs work well with this)
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
        if not WEBGPU_AVAILABLE:
            raise ImportError("WebGPU integration requires the wgpu package")
        
        self.connectome = connectome_manager_gpu
        self.device = None
        self.adapter = None
        
        # Shader modules
        self._shader_modules = {}
        
        # GPU buffers
        self.buffers = {}
        
        # Staging buffers for CPU-GPU transfers
        self.staging_buffers = {}
        
        # Bind groups
        self.bind_groups = {}
        
        # Pipeline cache
        self.pipelines = {}
        
        # Performance metrics
        self.performance_metrics = {
            "last_transfer_time": 0,
            "last_compute_time": 0,
            "total_transfer_time": 0,
            "total_compute_time": 0,
            "transfer_count": 0,
            "compute_count": 0
        }
        
        # Initialize WebGPU
        success = self._initialize_webgpu()
        if not success:
            logger.warning("WebGPU initialization failed. Falling back to CPU implementation.")
    
    def _initialize_webgpu(self):
        """Initialize WebGPU context.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            # Initialize WebGPU adapter
            if hasattr(wgpu, 'gpu') and hasattr(wgpu.gpu, 'request_adapter_async'):
                self.adapter = wgpu.gpu.request_adapter_async(
                    power_preference="high-performance"
                )
            else:
                logger.warning("Using older wgpu interface.")
                self.adapter = wgpu.request_adapter(
                    power_preference="high-performance"
                )
            
            # Create device with required features and limits
            self.device = self.adapter.request_device(
                required_features=[
                    "timestamp-query",  # For performance metrics
                    "pipeline-statistics-query"  # For performance metrics
                ],
                required_limits={
                    "max_storage_buffer_binding_size": 1024 * 1024 * 1024,  # 1GB
                    "max_buffer_size": 1024 * 1024 * 1024,  # 1GB
                    "max_compute_workgroup_size_x": DEFAULT_WORKGROUP_SIZE,
                    "max_compute_invocations_per_workgroup": DEFAULT_WORKGROUP_SIZE
                }
            )
            
            # Initialize shader modules
            self._init_shader_modules()
            
            # Allocate GPU buffers
            self._allocate_buffers()
            
            # Create bind groups for shaders
            self._create_bind_groups()
            
            # Create compute pipelines
            self._create_compute_pipelines()
            
            logger.info(f"WebGPU device initialized: {self.device}")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize WebGPU: {e}")
            return False
    
    def _init_shader_modules(self):
        """Initialize shader modules for different operations."""
        # Neuron update shader (follows recommendations from the architecture document)
        neuron_shader = """
        // WebGPU shader for neuron operations
        // Workgroup size optimized for most GPUs
        
        struct SimParams {
            // Global simulation parameters
            neuron_count: u32,
            timestep: u32,
            decay_factor: f32,
        };
        
        struct NeuronData {
            // SoA layout for optimal memory access
            membrane_potentials: array<f32>,
            resting_potentials: array<f32>,
            thresholds: array<f32>,
            refractory_periods: array<u32>,
            refractory_counters: array<u32>,
            is_active: array<u32>,
            valid_mask: array<u32>,
        };
        
        struct FireCandidateList {
            // Bitmap for fire candidates - each bit represents a neuron
            count: atomic<u32>,
            neuron_ids: array<u32>,
        };
        
        @group(0) @binding(0) var<uniform> params: SimParams;
        @group(0) @binding(1) var<storage, read_write> neurons: NeuronData;
        @group(0) @binding(2) var<storage, read_write> fire_list: FireCandidateList;
        
        // Store frequently used neuron data in workgroup memory for faster access
        var<workgroup> local_potentials: array<f32, 256>;
        var<workgroup> local_thresholds: array<f32, 256>;
        
        @compute @workgroup_size(256)
        fn update_neurons(@builtin(global_invocation_id) global_id: vec3<u32>,
                         @builtin(local_invocation_id) local_id: vec3<u32>,
                         @builtin(workgroup_id) workgroup_id: vec3<u32>) {
            // Calculate global neuron index
            let neuron_idx = global_id.x;
            
            // Skip if beyond neuron count
            if (neuron_idx >= params.neuron_count) {
                return;
            }
            
            // Skip if neuron is not valid
            if (neurons.valid_mask[neuron_idx] == 0u) {
                return;
            }
            
            // Load data into workgroup memory for faster access
            local_potentials[local_id.x] = neurons.membrane_potentials[neuron_idx];
            local_thresholds[local_id.x] = neurons.thresholds[neuron_idx];
            
            // Ensure all threads have loaded their data
            workgroupBarrier();
            
            // Update membrane potential with decay
            let membrane_potential = local_potentials[local_id.x] * params.decay_factor;
            neurons.membrane_potentials[neuron_idx] = membrane_potential;
            
            // Check if neuron should fire (above threshold and not in refractory period)
            let threshold = local_thresholds[local_id.x];
            let should_fire = (membrane_potential >= threshold) && 
                             (neurons.refractory_counters[neuron_idx] == 0u);
            
            if (should_fire) {
                // Reset membrane potential
                neurons.membrane_potentials[neuron_idx] = neurons.resting_potentials[neuron_idx];
                
                // Set refractory counter
                neurons.refractory_counters[neuron_idx] = neurons.refractory_periods[neuron_idx];
                
                // Mark as active
                neurons.is_active[neuron_idx] = 1u;
                
                // Add to fire list using atomic operations
                let fire_idx = atomicAdd(&fire_list.count, 1u);
                if (fire_idx < 1000000u) {  // Prevent buffer overflow
                    fire_list.neuron_ids[fire_idx] = u32(neuron_idx);
                }
            } else {
                // Update refractory counter if it's not zero
                if (neurons.refractory_counters[neuron_idx] > 0u) {
                    neurons.refractory_counters[neuron_idx] = neurons.refractory_counters[neuron_idx] - 1u;
                }
                
                // Reset active flag
                neurons.is_active[neuron_idx] = 0u;
            }
        }
        
        // Kernel for updating refractory periods across all neurons
        @compute @workgroup_size(256)
        fn update_refractory(@builtin(global_invocation_id) global_id: vec3<u32>) {
            let neuron_idx = global_id.x;
            
            // Skip if beyond neuron count or invalid
            if (neuron_idx >= params.neuron_count || neurons.valid_mask[neuron_idx] == 0u) {
                return;
            }
            
            // Decrement refractory counter if not zero
            if (neurons.refractory_counters[neuron_idx] > 0u) {
                neurons.refractory_counters[neuron_idx] = neurons.refractory_counters[neuron_idx] - 1u;
            }
        }
        """
        
        # Synapse processing shader (follows architecture document recommendations)
        synapse_shader = """
        // WebGPU shader for synapse operations
        // Uses CSR format for efficient sparse matrix operations
        
        struct SynapseData {
            // CSR format components
            row_count: u32,
            nnz: u32,  // Number of non-zeros
            indptr: array<u32>,
            indices: array<u32>,
            weights: array<f32>,
        };
        
        struct NeuronData {
            // SoA layout for optimal memory access
            membrane_potentials: array<f32>,
            valid_mask: array<u32>,
        };
        
        struct FireCandidateList {
            count: u32,
            neuron_ids: array<u32>,
        };
        
        @group(0) @binding(0) var<uniform> batch_size: u32;
        @group(0) @binding(1) var<storage, read> fire_list: FireCandidateList;
        @group(0) @binding(2) var<storage, read> synapses: SynapseData;
        @group(0) @binding(3) var<storage, read_write> neurons: NeuronData;
        
        // We'll use atomic operations to safely update membrane potentials
        // from multiple threads targeting the same neuron
        
        @compute @workgroup_size(256)
        fn propagate_signals(@builtin(global_invocation_id) global_id: vec3<u32>) {
            let batch_idx = global_id.x;
            
            // Skip if beyond batch size
            if (batch_idx >= batch_size) {
                return;
            }
            
            // Determine which fired neuron to process based on batch index
            let fired_idx = batch_idx % fire_list.count;
            let fired_neuron = fire_list.neuron_ids[fired_idx];
            
            // Get synaptic connections for this neuron
            let row_start = synapses.indptr[fired_neuron];
            let row_end = synapses.indptr[fired_neuron + 1u];
            
            // Process outgoing connections
            for (var i = row_start; i < row_end; i++) {
                let target = synapses.indices[i];
                let weight = synapses.weights[i];
                
                // Skip invalid targets
                if (neurons.valid_mask[target] == 0u) {
                    continue;
                }
                
                // Update membrane potential atomically to handle concurrent updates
                let current = atomicLoad(&neurons.membrane_potentials[target]);
                atomicStore(&neurons.membrane_potentials[target], current + weight);
            }
        }
        
        // Helper functions for atomic operations on f32
        fn atomicLoad(ptr: ptr<storage, f32, read>) -> f32 {
            var result: f32;
            
            // Cast to u32, perform atomic op, then cast back to f32
            // This is a workaround since WGSL doesn't directly support f32 atomics
            var temp = ptr;
            var int_ptr = bitcast<ptr<storage, atomic<u32>, read>>(temp);
            var int_val = atomicLoad(int_ptr);
            
            // Convert back to f32
            result = bitcast<f32>(int_val);
            return result;
        }
        
        fn atomicStore(ptr: ptr<storage, f32, read_write>, val: f32) {
            // Cast to u32, perform atomic op, then cast back to f32
            var temp = ptr;
            var int_ptr = bitcast<ptr<storage, atomic<u32>, read_write>>(temp);
            var int_val = bitcast<u32>(val);
            atomicStore(int_ptr, int_val);
        }
        """
        
        # Create shader modules
        self._shader_modules["neuron_operations"] = self.device.create_shader_module(code=neuron_shader)
        self._shader_modules["synapse_operations"] = self.device.create_shader_module(code=synapse_shader)
    
    def _allocate_buffers(self):
        """Allocate GPU buffers for neuron and synapse data with proper alignment."""
        # Get dimensions from connectome
        max_neurons = self.connectome.neuron_array.aligned_size
        
        # Simulation parameters buffer (16-byte aligned)
        self.buffers["params"] = self.device.create_buffer(
            size=16,  # 3 values (u32, u32, f32) with padding to 16 bytes
            usage=wgpu.BufferUsages.UNIFORM | wgpu.BufferUsages.COPY_DST,
            label="Simulation Parameters Buffer"
        )
        
        # Neuron data buffers - ensure 64-byte alignment for optimal performance
        buffer_sizes = {
            "membrane_potentials": max_neurons * 4,  # f32 array
            "resting_potentials": max_neurons * 4,   # f32 array
            "thresholds": max_neurons * 4,           # f32 array
            "refractory_periods": max_neurons * 4,   # u32 array
            "refractory_counters": max_neurons * 4,  # u32 array
            "is_active": max_neurons * 4,            # u32 array (using u32 instead of bool for alignment)
            "valid_mask": max_neurons * 4,           # u32 array (using u32 instead of bool for alignment)
            "area_ids": max_neurons * 4              # u32 array
        }
        
        # Create buffers with proper alignment
        for name, size in buffer_sizes.items():
            # Align size to 64 bytes (16 floats)
            aligned_size = ((size + MEMORY_ALIGNMENT - 1) // MEMORY_ALIGNMENT) * MEMORY_ALIGNMENT
            
            # Create buffer
            self.buffers[name] = self.device.create_buffer(
                size=aligned_size,
                usage=wgpu.BufferUsages.STORAGE | wgpu.BufferUsages.COPY_DST | wgpu.BufferUsages.COPY_SRC,
                label=f"{name.capitalize()} Buffer"
            )
            
            # Create staging buffer for efficient transfers
            self.staging_buffers[name] = self.device.create_buffer(
                size=aligned_size,
                usage=wgpu.BufferUsages.MAP_WRITE | wgpu.BufferUsages.COPY_SRC,
                label=f"{name.capitalize()} Staging Buffer"
            )
        
        # Fire candidate list buffer
        # Size for up to 1 million firing neurons
        fcl_size = 1_000_000
        self.buffers["fire_count"] = self.device.create_buffer(
            size=4,  # single u32
            usage=wgpu.BufferUsages.STORAGE | wgpu.BufferUsages.COPY_DST | wgpu.BufferUsages.COPY_SRC,
            label="Fire Count Buffer"
        )
        
        self.buffers["fire_list"] = self.device.create_buffer(
            size=fcl_size * 4,  # u32 array
            usage=wgpu.BufferUsages.STORAGE | wgpu.BufferUsages.COPY_DST | wgpu.BufferUsages.COPY_SRC,
            label="Fire List Buffer"
        )
        
        # Batch size for synapse propagation
        self.buffers["batch_size"] = self.device.create_buffer(
            size=4,  # single u32
            usage=wgpu.BufferUsages.UNIFORM | wgpu.BufferUsages.COPY_DST,
            label="Batch Size Buffer"
        )
    
    def update_neuron_data(self):
        """Update GPU buffers with current neuron data using staging buffers."""
        start_time = time.time()
        
        # Get neuron array
        neuron_array = self.connectome.neuron_array
        
        # Convert to NumPy arrays if they're not already
        membrane_potentials = self.connectome.neuron_array.backend.to_numpy(neuron_array.membrane_potentials)
        resting_potentials = self.connectome.neuron_array.backend.to_numpy(neuron_array.resting_potentials)
        thresholds = self.connectome.neuron_array.backend.to_numpy(neuron_array.thresholds)
        refractory_periods = self.connectome.neuron_array.backend.to_numpy(neuron_array.refractory_periods)
        refractory_counters = self.connectome.neuron_array.backend.to_numpy(neuron_array.refractory_counters)
        is_active = self.connectome.neuron_array.backend.to_numpy(neuron_array.is_active).astype(np.uint32)
        valid_mask = self.connectome.neuron_array.backend.to_numpy(neuron_array.valid_mask).astype(np.uint32)
        
        # Ensure arrays are contiguous and properly aligned
        membrane_potentials = np.ascontiguousarray(membrane_potentials, dtype=np.float32)
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
        staging_membrane.write_mapped_view().reshape(membrane_potentials.shape)[:] = membrane_potentials
        staging_membrane.unmap()
        
        staging_resting = self.staging_buffers["resting_potentials"]
        staging_resting.map_write()
        staging_resting.write_mapped_view().reshape(resting_potentials.shape)[:] = resting_potentials
        staging_resting.unmap()
        
        staging_thresholds = self.staging_buffers["thresholds"]
        staging_thresholds.map_write()
        staging_thresholds.write_mapped_view().reshape(thresholds.shape)[:] = thresholds
        staging_thresholds.unmap()
        
        staging_ref_periods = self.staging_buffers["refractory_periods"]
        staging_ref_periods.map_write()
        staging_ref_periods.write_mapped_view().reshape(refractory_periods.shape)[:] = refractory_periods
        staging_ref_periods.unmap()
        
        staging_ref_counters = self.staging_buffers["refractory_counters"]
        staging_ref_counters.map_write()
        staging_ref_counters.write_mapped_view().reshape(refractory_counters.shape)[:] = refractory_counters
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
        encoder.copy_buffer_to_buffer(self.staging_buffers["membrane_potentials"], 0, 
                                     self.buffers["membrane_potentials"], 0, membrane_potentials.nbytes)
        encoder.copy_buffer_to_buffer(self.staging_buffers["resting_potentials"], 0, 
                                     self.buffers["resting_potentials"], 0, resting_potentials.nbytes)
        encoder.copy_buffer_to_buffer(self.staging_buffers["thresholds"], 0, 
                                     self.buffers["thresholds"], 0, thresholds.nbytes)
        encoder.copy_buffer_to_buffer(self.staging_buffers["refractory_periods"], 0, 
                                     self.buffers["refractory_periods"], 0, refractory_periods.nbytes)
        encoder.copy_buffer_to_buffer(self.staging_buffers["refractory_counters"], 0, 
                                     self.buffers["refractory_counters"], 0, refractory_counters.nbytes)
        encoder.copy_buffer_to_buffer(self.staging_buffers["is_active"], 0, 
                                     self.buffers["is_active"], 0, is_active.nbytes)
        encoder.copy_buffer_to_buffer(self.staging_buffers["valid_mask"], 0, 
                                     self.buffers["valid_mask"], 0, valid_mask.nbytes)
        
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
        # Ensure matrices are in CSR format for efficient processing
        self.connectome._ensure_csr_format_outgoing()
        
        # Get sparse matrix from connectome
        sparse_matrix = self.connectome.outgoing_matrix
        
        # CSR format: indptr (row pointers), indices (column indices), data (weights)
        # Create buffers if needed
        if "row_indices" not in self.buffers:
            self.buffers["row_indices"] = self.device.create_buffer(
                size=(sparse_matrix.shape[0] + 1) * 4,  # u32 array, length = num_rows + 1
                usage=wgpu.BufferUsages.STORAGE | wgpu.BufferUsages.COPY_DST,
                label="Row Indices Buffer"
            )
        
        if "col_indices" not in self.buffers:
            self.buffers["col_indices"] = self.device.create_buffer(
                size=sparse_matrix.nnz * 4,  # u32 array, length = num_nonzeros
                usage=wgpu.BufferUsages.STORAGE | wgpu.BufferUsages.COPY_DST,
                label="Column Indices Buffer"
            )
        
        if "weights" not in self.buffers:
            self.buffers["weights"] = self.device.create_buffer(
                size=sparse_matrix.nnz * 4,  # f32 array, length = num_nonzeros
                usage=wgpu.BufferUsages.STORAGE | wgpu.BufferUsages.COPY_DST,
                label="Weights Buffer"
            )
        
        # Write synapse data to GPU buffers
        self.device.queue.write_buffer(self.buffers["row_indices"], 0, 
                                      sparse_matrix.indptr.astype(np.uint32))
        self.device.queue.write_buffer(self.buffers["col_indices"], 0, 
                                      sparse_matrix.indices.astype(np.uint32))
        self.device.queue.write_buffer(self.buffers["weights"], 0, 
                                      sparse_matrix.data.astype(np.float32))
    
    def _create_bind_groups(self):
        """Create bind groups for the compute pipelines."""
        # Neuron operations bind group
        neuron_bind_group_layout = self.device.create_bind_group_layout(
            entries=[
                # Simulation parameters
                {
                    "binding": 0,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.UNIFORM}
                },
                # Neuron data
                {
                    "binding": 1,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.STORAGE}
                },
                # Fired neurons
                {
                    "binding": 2,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.STORAGE}
                }
            ],
            label="Neuron Operations Bind Group Layout"
        )
        
        # Create bind group with actual buffers
        self.bind_groups["neuron_operations"] = self.device.create_bind_group(
            layout=neuron_bind_group_layout,
            entries=[
                {"binding": 0, "resource": {"buffer": self.buffers["params"]}},
                {"binding": 1, "resource": {"buffer": self.buffers["membrane_potentials"]}},
                {"binding": 2, "resource": {"buffer": self.buffers["fire_list"]}}
            ],
            label="Neuron Operations Bind Group"
        )
        
        # Synapse operations bind group layout
        synapse_bind_group_layout = self.device.create_bind_group_layout(
            entries=[
                # Batch size
                {
                    "binding": 0,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.UNIFORM}
                },
                # Fire list
                {
                    "binding": 1,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.STORAGE}
                },
                # Synapse data (CSR format)
                {
                    "binding": 2,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.STORAGE}
                },
                # Neuron data
                {
                    "binding": 3,
                    "visibility": wgpu.ShaderStage.COMPUTE,
                    "buffer": {"type": wgpu.BufferBindingType.STORAGE}
                }
            ],
            label="Synapse Operations Bind Group Layout"
        )
        
        # Create bind group with actual buffers
        self.bind_groups["synapse_operations"] = self.device.create_bind_group(
            layout=synapse_bind_group_layout,
            entries=[
                {"binding": 0, "resource": {"buffer": self.buffers["batch_size"]}},
                {"binding": 1, "resource": {"buffer": self.buffers["fire_list"]}},
                {"binding": 2, "resource": {"buffer": self.buffers["row_indices"]}}, # CSR indptr
                {"binding": 3, "resource": {"buffer": self.buffers["membrane_potentials"]}}
            ],
            label="Synapse Operations Bind Group"
        )
    
    def _create_compute_pipelines(self):
        """Create compute pipelines for neuron and synapse operations."""
        # Neuron update pipeline
        neuron_pipeline_layout = self.device.create_pipeline_layout(
            bind_group_layouts=[self.bind_groups["neuron_operations"].layout]
        )
        
        self.pipelines["neuron_update"] = self.device.create_compute_pipeline(
            layout=neuron_pipeline_layout,
            compute={"module": self._shader_modules["neuron_operations"],
                    "entry_point": "update_neurons"}
        )
        
        # Synapse propagation pipeline
        synapse_pipeline_layout = self.device.create_pipeline_layout(
            bind_group_layouts=[self.bind_groups["synapse_operations"].layout]
        )
        
        self.pipelines["synapse_propagation"] = self.device.create_compute_pipeline(
            layout=synapse_pipeline_layout,
            compute={"module": self._shader_modules["synapse_operations"],
                    "entry_point": "propagate_signals"}
        )
    
    def update_membrane_potentials(self):
        """Update membrane potentials using WebGPU acceleration."""
        # Update params buffer with current timestep
        self.device.queue.write_buffer(
            self.buffers["params"], 0,
            np.array([self.connectome.max_neurons, self.connectome.current_timestep], 
                    dtype=np.uint32)
        )
        
        # Reset fired count to 0
        self.device.queue.write_buffer(
            self.buffers["fired_count"], 0,
            np.array([0], dtype=np.uint32)
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
        encoder = self.device.create_command_encoder(label="Synapse Propagation Encoder")
        
        # Signal propagation compute pass
        # First read back fired count to determine dispatch size
        fired_count_staging = self.device.create_buffer(
            size=4,
            usage=wgpu.BufferUsages.MAP_READ | wgpu.BufferUsages.COPY_DST,
            label="Fired Count Staging Buffer"
        )
        
        # Copy fired count to staging buffer
        encoder.copy_buffer_to_buffer(self.buffers["fired_count"], 0,
                                     fired_count_staging, 0, 4)
        self.device.queue.submit([encoder.finish()])
        
        # Map buffer and read data
        fired_count_staging.map_async()
        fired_count_data = np.frombuffer(fired_count_staging.get_mapped_range(), dtype=np.uint32)
        fired_count = int(fired_count_data[0])
        fired_count_staging.unmap()
        
        # Only proceed with signal propagation if there are fired neurons
        if fired_count > 0:
            # Update batch size for synapse propagation
            self.device.queue.write_buffer(
                self.buffers["batch_size"], 0,
                np.array([fired_count], dtype=np.uint32)
            )
            
            # Create command encoder for synapse propagation
            encoder = self.device.create_command_encoder(label="Synapse Propagation Encoder")
            
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
            label="Fired Neurons Staging Buffer"
        )
        
        # Copy fired neurons to staging buffer
        encoder = self.device.create_command_encoder()
        encoder.copy_buffer_to_buffer(self.buffers["fire_list"], 0,
                                     fired_neurons_staging, 0, 
                                     min(fired_count, 1_000_000) * 4)
        self.device.queue.submit([encoder.finish()])
        
        # Map buffer and read data
        fired_neurons_staging.map_async()
        fired_neurons_data = np.frombuffer(fired_neurons_staging.get_mapped_range(), 
                                          dtype=np.uint32, 
                                          count=min(fired_count, 1_000_000))
        fired_neuron_ids = [
            self.connectome.index_to_neuron_id.get(int(idx), int(idx)) 
            for idx in fired_neurons_data
        ]
        fired_neurons_staging.unmap()
        
        # Update FCL manager
        self.connectome.fcl_manager.register_event(self.connectome.current_timestep, fired_neuron_ids)
        
        # Increment timestep
        self.connectome.current_timestep += 1
        
        # Record performance metrics
        self.performance_metrics["compute_count"] += 1
        
        return fired_neuron_ids

def is_webgpu_available():
    """Check if WebGPU is available on this system."""
    return WEBGPU_AVAILABLE 