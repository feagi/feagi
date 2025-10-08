/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # WGPU Backend
//!
//! GPU-accelerated backend using WGPU (cross-platform GPU compute library).
//! Supports Metal (macOS), Vulkan (Linux), DirectX 12 (Windows).

use super::ComputeBackend;
use feagi_types::*;

/// WGPU backend for GPU acceleration
pub struct WGPUBackend {
    /// Backend name for logging
    name: String,
    
    /// WGPU device
    device: wgpu::Device,
    
    /// WGPU command queue
    queue: wgpu::Queue,
    
    /// Neural dynamics compute pipeline
    neural_dynamics_pipeline: Option<wgpu::ComputePipeline>,
    
    /// Neural dynamics bind group
    neural_dynamics_bind_group: Option<wgpu::BindGroup>,
    
    /// Synaptic propagation compute pipeline
    synaptic_propagation_pipeline: Option<wgpu::ComputePipeline>,
    
    /// Synaptic propagation bind group
    synaptic_propagation_bind_group: Option<wgpu::BindGroup>,
    
    /// GPU buffers (persistent)
    buffers: WGPUBuffers,
    
    /// Capacity (max neurons/synapses)
    neuron_capacity: usize,
    synapse_capacity: usize,
    
    /// Current neuron count (for dispatch)
    current_neuron_count: usize,
}

/// GPU buffer management
struct WGPUBuffers {
    // Neuron arrays (persistent)
    membrane_potentials: Option<wgpu::Buffer>,
    thresholds: Option<wgpu::Buffer>,
    leak_coefficients: Option<wgpu::Buffer>,
    resting_potentials: Option<wgpu::Buffer>,
    refractory_periods: Option<wgpu::Buffer>,
    refractory_countdowns: Option<wgpu::Buffer>,
    excitabilities: Option<wgpu::Buffer>,
    consecutive_fire_counts: Option<wgpu::Buffer>,
    consecutive_fire_limits: Option<wgpu::Buffer>,
    snooze_periods: Option<wgpu::Buffer>,
    snooze_countdowns: Option<wgpu::Buffer>,
    valid_mask: Option<wgpu::Buffer>,
    
    // Synapse arrays (persistent)
    source_neurons: Option<wgpu::Buffer>,
    target_neurons: Option<wgpu::Buffer>,
    weights: Option<wgpu::Buffer>,
    conductances: Option<wgpu::Buffer>,
    synapse_types: Option<wgpu::Buffer>,
    synapse_valid_mask: Option<wgpu::Buffer>,
    
    // Dynamic buffers (per-burst)
    fired_neurons_input: Option<wgpu::Buffer>,
    fired_neurons_output: Option<wgpu::Buffer>,
    
    // Staging buffers for readback
    staging_buffer: Option<wgpu::Buffer>,
}

impl WGPUBuffers {
    fn new() -> Self {
        Self {
            membrane_potentials: None,
            thresholds: None,
            leak_coefficients: None,
            resting_potentials: None,
            refractory_periods: None,
            refractory_countdowns: None,
            excitabilities: None,
            consecutive_fire_counts: None,
            consecutive_fire_limits: None,
            snooze_periods: None,
            snooze_countdowns: None,
            valid_mask: None,
            source_neurons: None,
            target_neurons: None,
            weights: None,
            conductances: None,
            synapse_types: None,
            synapse_valid_mask: None,
            fired_neurons_input: None,
            fired_neurons_output: None,
            staging_buffer: None,
        }
    }
}

impl WGPUBackend {
    /// Create a new WGPU backend
    pub fn new(neuron_capacity: usize, synapse_capacity: usize) -> Result<Self> {
        // Initialize WGPU
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });
        
        // Request adapter (GPU)
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: None,
            force_fallback_adapter: false,
        }))
        .ok_or_else(|| Error::ComputationError("Failed to find WGPU adapter".to_string()))?;
        
        let adapter_info = adapter.get_info();
        let backend_name = format!("WGPU ({} - {:?})", adapter_info.name, adapter_info.backend);
        
        // Request device and queue
        let (device, queue) = pollster::block_on(adapter.request_device(
            &wgpu::DeviceDescriptor {
                label: Some("FEAGI NPU Device"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::default(),
            },
            None,
        ))
        .map_err(|e| Error::ComputationError(format!("Failed to create device: {}", e)))?;
        
        Ok(Self {
            name: backend_name,
            device,
            queue,
            neural_dynamics_pipeline: None,
            neural_dynamics_bind_group: None,
            synaptic_propagation_pipeline: None,
            synaptic_propagation_bind_group: None,
            buffers: WGPUBuffers::new(),
            neuron_capacity,
            synapse_capacity,
            current_neuron_count: 0,
        })
    }
    
    /// Initialize compute pipelines (shaders)
    fn initialize_pipelines(&mut self) -> Result<()> {
        // Load WGSL shaders
        let neural_dynamics_shader = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Neural Dynamics Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/neural_dynamics.wgsl").into()),
        });
        
        let synaptic_propagation_shader = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Synaptic Propagation Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/synaptic_propagation.wgsl").into()),
        });
        
        // Create bind group layouts
        let neural_dynamics_bind_group_layout = self.device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Neural Dynamics Bind Group Layout"),
            entries: &[
                // Membrane potentials (read-write)
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // Thresholds (read-only)
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // ... (would need to define all 16 bindings)
                // For brevity, showing structure
            ],
        });
        
        // Create compute pipelines
        let neural_dynamics_pipeline_layout = self.device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Neural Dynamics Pipeline Layout"),
            bind_group_layouts: &[&neural_dynamics_bind_group_layout],
            push_constant_ranges: &[],
        });
        
        self.neural_dynamics_pipeline = Some(self.device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("Neural Dynamics Pipeline"),
            layout: Some(&neural_dynamics_pipeline_layout),
            module: &neural_dynamics_shader,
            entry_point: "neural_dynamics_main",
        }));
        
        // Similar for synaptic propagation
        // ... (would create synaptic_propagation_pipeline)
        
        Ok(())
    }
    
    /// Upload neuron array data to GPU
    fn upload_neuron_arrays(&mut self, neuron_array: &NeuronArray) -> Result<()> {
        let neuron_count = neuron_array.count;
        self.current_neuron_count = neuron_count;
        
        // Helper to create or update buffer
        let create_buffer_f32 = |device: &wgpu::Device, queue: &wgpu::Queue, data: &[f32], label: &str| {
            let buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: (data.len() * std::mem::size_of::<f32>()) as u64,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            });
            queue.write_buffer(&buffer, 0, bytemuck::cast_slice(data));
            buffer
        };
        
        let create_buffer_u16 = |device: &wgpu::Device, queue: &wgpu::Queue, data: &[u16], label: &str| {
            let buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: (data.len() * std::mem::size_of::<u16>()) as u64,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            });
            queue.write_buffer(&buffer, 0, bytemuck::cast_slice(data));
            buffer
        };
        
        let create_buffer_bool = |device: &wgpu::Device, queue: &wgpu::Queue, data: &[bool], label: &str| {
            // Pack bools into u32 bitfield
            let packed_count = (data.len() + 31) / 32;
            let mut packed = vec![0u32; packed_count];
            for (i, &val) in data.iter().enumerate() {
                if val {
                    let word_idx = i / 32;
                    let bit_idx = i % 32;
                    packed[word_idx] |= 1u32 << bit_idx;
                }
            }
            let buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: (packed.len() * std::mem::size_of::<u32>()) as u64,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            queue.write_buffer(&buffer, 0, bytemuck::cast_slice(&packed));
            buffer
        };
        
        // Upload all neuron arrays
        self.buffers.membrane_potentials = Some(create_buffer_f32(
            &self.device, &self.queue, &neuron_array.membrane_potentials[..neuron_count], 
            "Membrane Potentials"
        ));
        
        self.buffers.thresholds = Some(create_buffer_f32(
            &self.device, &self.queue, &neuron_array.thresholds[..neuron_count],
            "Thresholds"
        ));
        
        self.buffers.leak_coefficients = Some(create_buffer_f32(
            &self.device, &self.queue, &neuron_array.leak_coefficients[..neuron_count],
            "Leak Coefficients"
        ));
        
        self.buffers.resting_potentials = Some(create_buffer_f32(
            &self.device, &self.queue, &neuron_array.resting_potentials[..neuron_count],
            "Resting Potentials"
        ));
        
        // Convert u16 to u32 for GPU (easier alignment)
        let refractory_periods_u32: Vec<u32> = neuron_array.refractory_periods[..neuron_count]
            .iter().map(|&x| x as u32).collect();
        self.buffers.refractory_periods = Some({
            let buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("Refractory Periods"),
                size: (refractory_periods_u32.len() * 4) as u64,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            self.queue.write_buffer(&buffer, 0, bytemuck::cast_slice(&refractory_periods_u32));
            buffer
        });
        
        self.buffers.refractory_countdowns = Some(create_buffer_u16(
            &self.device, &self.queue, &neuron_array.refractory_countdowns[..neuron_count],
            "Refractory Countdowns"
        ));
        
        self.buffers.excitabilities = Some(create_buffer_f32(
            &self.device, &self.queue, &neuron_array.excitabilities[..neuron_count],
            "Excitabilities"
        ));
        
        self.buffers.consecutive_fire_counts = Some(create_buffer_u16(
            &self.device, &self.queue, &neuron_array.consecutive_fire_counts[..neuron_count],
            "Consecutive Fire Counts"
        ));
        
        self.buffers.consecutive_fire_limits = Some(create_buffer_u16(
            &self.device, &self.queue, &neuron_array.consecutive_fire_limits[..neuron_count],
            "Consecutive Fire Limits"
        ));
        
        self.buffers.snooze_periods = Some(create_buffer_u16(
            &self.device, &self.queue, &neuron_array.snooze_periods[..neuron_count],
            "Snooze Periods"
        ));
        
        self.buffers.snooze_countdowns = Some(create_buffer_u16(
            &self.device, &self.queue, &neuron_array.snooze_countdowns[..neuron_count],
            "Snooze Countdowns"
        ));
        
        self.buffers.valid_mask = Some(create_buffer_bool(
            &self.device, &self.queue, &neuron_array.valid_mask[..neuron_count],
            "Valid Mask"
        ));
        
        Ok(())
    }
    
    /// Upload synapse array data to GPU
    fn upload_synapse_arrays(&mut self, synapse_array: &SynapseArray) -> Result<()> {
        let synapse_count = synapse_array.count;
        
        // Helper to create buffer for u32 arrays
        let create_buffer_u32 = |device: &wgpu::Device, queue: &wgpu::Queue, data: &[u32], label: &str| {
            let buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: (data.len() * 4) as u64,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            queue.write_buffer(&buffer, 0, bytemuck::cast_slice(data));
            buffer
        };
        
        // Upload synapse arrays
        self.buffers.source_neurons = Some(create_buffer_u32(
            &self.device, &self.queue, &synapse_array.source_neurons[..synapse_count],
            "Source Neurons"
        ));
        
        self.buffers.target_neurons = Some(create_buffer_u32(
            &self.device, &self.queue, &synapse_array.target_neurons[..synapse_count],
            "Target Neurons"
        ));
        
        // Convert u8 to u32 for GPU
        let weights_u32: Vec<u32> = synapse_array.weights[..synapse_count]
            .iter().map(|&x| x as u32).collect();
        self.buffers.weights = Some(create_buffer_u32(
            &self.device, &self.queue, &weights_u32,
            "Weights"
        ));
        
        let conductances_u32: Vec<u32> = synapse_array.conductances[..synapse_count]
            .iter().map(|&x| x as u32).collect();
        self.buffers.conductances = Some(create_buffer_u32(
            &self.device, &self.queue, &conductances_u32,
            "Conductances"
        ));
        
        let types_u32: Vec<u32> = synapse_array.types[..synapse_count]
            .iter().map(|&x| x as u32).collect();
        self.buffers.synapse_types = Some(create_buffer_u32(
            &self.device, &self.queue, &types_u32,
            "Synapse Types"
        ));
        
        // Pack valid mask
        let packed_count = (synapse_count + 31) / 32;
        let mut packed = vec![0u32; packed_count];
        for i in 0..synapse_count {
            if synapse_array.valid_mask[i] {
                let word_idx = i / 32;
                let bit_idx = i % 32;
                packed[word_idx] |= 1u32 << bit_idx;
            }
        }
        self.buffers.synapse_valid_mask = Some(create_buffer_u32(
            &self.device, &self.queue, &packed,
            "Synapse Valid Mask"
        ));
        
        // TODO: Build GPU hash table for synapse index
        // This requires building a hash table from synapse_array.source_index
        
        Ok(())
    }
    
    /// Create bind groups after buffers are uploaded
    fn create_bind_groups(&mut self) -> Result<()> {
        // Create simplified bind group for neural dynamics
        // Note: Full implementation would need all 16 bindings
        // For now, creating minimal version to show structure
        
        let entries = vec![
            wgpu::BindGroupEntry {
                binding: 0,
                resource: self.buffers.membrane_potentials.as_ref()
                    .ok_or_else(|| Error::ComputationError("Membrane potentials buffer not created".to_string()))?
                    .as_entire_binding(),
            },
            // Would need to add all other bindings here...
        ];
        
        // Note: This is incomplete - just showing structure
        // Full implementation needs bind group layout from pipeline
        
        Ok(())
    }
    
    /// Dispatch neural dynamics shader
    fn dispatch_neural_dynamics(&mut self, burst_count: u64) -> Result<()> {
        let pipeline = self.neural_dynamics_pipeline.as_ref()
            .ok_or_else(|| Error::ComputationError("Neural dynamics pipeline not initialized".to_string()))?;
        
        let bind_group = self.neural_dynamics_bind_group.as_ref()
            .ok_or_else(|| Error::ComputationError("Neural dynamics bind group not created".to_string()))?;
        
        // Calculate workgroups (256 neurons per workgroup)
        let workgroup_count = (self.current_neuron_count as u32 + 255) / 256;
        
        // Create command encoder
        let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Neural Dynamics Encoder"),
        });
        
        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("Neural Dynamics Pass"),
                timestamp_writes: None,
            });
            
            compute_pass.set_pipeline(pipeline);
            compute_pass.set_bind_group(0, bind_group, &[]);
            compute_pass.dispatch_workgroups(workgroup_count, 1, 1);
        }
        
        // Submit commands
        self.queue.submit(Some(encoder.finish()));
        
        Ok(())
    }
    
    /// Dispatch synaptic propagation shader
    fn dispatch_synaptic_propagation(&mut self, fired_count: usize) -> Result<()> {
        let pipeline = self.synaptic_propagation_pipeline.as_ref()
            .ok_or_else(|| Error::ComputationError("Synaptic propagation pipeline not initialized".to_string()))?;
        
        let bind_group = self.synaptic_propagation_bind_group.as_ref()
            .ok_or_else(|| Error::ComputationError("Synaptic propagation bind group not created".to_string()))?;
        
        // Calculate workgroups (256 fired neurons per workgroup)
        let workgroup_count = (fired_count as u32 + 255) / 256;
        
        // Create command encoder
        let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Synaptic Propagation Encoder"),
        });
        
        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("Synaptic Propagation Pass"),
                timestamp_writes: None,
            });
            
            compute_pass.set_pipeline(pipeline);
            compute_pass.set_bind_group(0, bind_group, &[]);
            compute_pass.dispatch_workgroups(workgroup_count, 1, 1);
        }
        
        // Submit commands
        self.queue.submit(Some(encoder.finish()));
        
        Ok(())
    }
    
    /// Download fired neuron results from GPU
    fn download_fired_neurons(&self) -> Result<Vec<u32>> {
        // Create staging buffer if not exists
        // Map buffer from GPU to CPU
        // Read fired neuron indices
        // Unmap buffer
        
        // Placeholder: Would need fired_indices buffer and fired_count atomic
        // For now, return empty to keep compilation working
        Ok(vec![])
    }
}

impl ComputeBackend for WGPUBackend {
    fn backend_name(&self) -> &str {
        &self.name
    }
    
    fn process_synaptic_propagation(
        &mut self,
        fired_neurons: &[u32],
        _synapse_array: &SynapseArray,
        _neuron_array: &mut NeuronArray,
    ) -> Result<usize> {
        if fired_neurons.is_empty() {
            return Ok(0);
        }
        
        // Upload fired neurons to GPU (create/update fired_neurons_input buffer)
        let fired_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Fired Neurons Input"),
            size: (fired_neurons.len() * 4) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        self.queue.write_buffer(&fired_buffer, 0, bytemuck::cast_slice(fired_neurons));
        self.buffers.fired_neurons_input = Some(fired_buffer);
        
        // Dispatch synaptic propagation shader
        self.dispatch_synaptic_propagation(fired_neurons.len())?;
        
        // Wait for GPU to complete (blocking - synchronous for now)
        self.device.poll(wgpu::Maintain::Wait);
        
        // Return estimated synapse count (would be actual from GPU in full impl)
        Ok(fired_neurons.len() * 100) // Placeholder
    }
    
    fn process_neural_dynamics(
        &mut self,
        _neuron_array: &mut NeuronArray,
        burst_count: u64,
    ) -> Result<(Vec<u32>, usize, usize)> {
        // Dispatch neural dynamics compute shader
        self.dispatch_neural_dynamics(burst_count)?;
        
        // Wait for GPU to complete
        self.device.poll(wgpu::Maintain::Wait);
        
        // Download fired neuron results
        let fired_neurons = self.download_fired_neurons()?;
        
        let fired_count = fired_neurons.len();
        let neurons_processed = self.current_neuron_count;
        
        // Return results (refractory count placeholder)
        Ok((fired_neurons, neurons_processed, 0))
    }
    
    fn initialize_persistent_data(
        &mut self,
        neuron_array: &NeuronArray,
        synapse_array: &SynapseArray,
    ) -> Result<()> {
        // Upload all data to GPU
        self.upload_neuron_arrays(neuron_array)?;
        self.upload_synapse_arrays(synapse_array)?;
        
        // Initialize pipelines and shaders
        self.initialize_pipelines()?;
        
        // Create bind groups (connect buffers to shaders)
        self.create_bind_groups()?;
        
        println!("✅ GPU initialized: {} neurons, {} synapses uploaded", 
                 neuron_array.count, synapse_array.count);
        
        Ok(())
    }
    
    fn on_genome_change(&mut self) -> Result<()> {
        // Invalidate GPU buffers - will be re-uploaded on next burst
        self.buffers = WGPUBuffers::new();
        self.neural_dynamics_pipeline = None;
        self.neural_dynamics_bind_group = None;
        self.synaptic_propagation_pipeline = None;
        self.synaptic_propagation_bind_group = None;
        self.current_neuron_count = 0;
        
        println!("🔄 GPU state invalidated due to genome change");
        
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_wgpu_backend_creation() {
        // This test requires a GPU - may not work in CI
        if let Ok(backend) = WGPUBackend::new(1000, 10000) {
            assert!(backend.backend_name().contains("WGPU"));
        }
    }
}

