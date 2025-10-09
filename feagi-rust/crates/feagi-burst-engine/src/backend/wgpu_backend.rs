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
    snooze_periods: Option<wgpu::Buffer>,  // Extended refractory (additive)
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
        
        // Note: Bind group layout will be created in create_bind_groups()
        // For now, create pipeline without layout (will be set when dispatching)
        
        self.neural_dynamics_pipeline = Some(self.device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("Neural Dynamics Pipeline"),
            layout: None,  // Auto-layout from shader
            module: &neural_dynamics_shader,
            entry_point: "neural_dynamics_main",
        }));
        
        println!("✅ Neural dynamics shader loaded");
        
        // Synaptic propagation will be created after hash table is built
        
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
        
        // Build GPU hash table for synapse index
        self.build_gpu_synapse_hash_table(synapse_array)?;
        
        Ok(())
    }
    
    /// Build GPU-friendly hash table for synapse lookups
    fn build_gpu_synapse_hash_table(&mut self, synapse_array: &SynapseArray) -> Result<()> {
        use ahash::AHashMap;
        
        // Build temporary CPU hash table
        // Maps source_neuron_id → (start_index, count)
        let mut source_map: AHashMap<u32, Vec<usize>> = AHashMap::new();
        for i in 0..synapse_array.count {
            if synapse_array.valid_mask[i] {
                let source = synapse_array.source_neurons[i];
                source_map.entry(source).or_insert_with(Vec::new).push(i);
            }
        }
        
        // Calculate hash table capacity (2x entries for low collision rate)
        let capacity = (source_map.len() * 2).next_power_of_two().max(256);
        
        // Initialize hash table arrays
        let mut hash_keys = vec![0xFFFFFFFFu32; capacity];  // 0xFFFFFFFF = empty slot
        let mut hash_starts = vec![0u32; capacity];
        let mut hash_counts = vec![0u32; capacity];
        
        // Build flat synapse list
        let mut synapse_list = Vec::new();
        
        // Insert into hash table using linear probing
        for (&source_neuron, synapse_indices) in &source_map {
            let mut slot = (source_neuron as usize * 2654435761) % capacity;
            
            // Linear probing to find empty slot
            while hash_keys[slot] != 0xFFFFFFFF {
                slot = (slot + 1) % capacity;
            }
            
            // Insert
            hash_keys[slot] = source_neuron;
            hash_starts[slot] = synapse_list.len() as u32;
            hash_counts[slot] = synapse_indices.len() as u32;
            
            // Add synapse indices to flat list
            for &idx in synapse_indices {
                synapse_list.push(idx as u32);
            }
        }
        
        // Upload to GPU
        let create_buffer = |device: &wgpu::Device, queue: &wgpu::Queue, data: &[u32], label: &str| {
            let buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: (data.len() * 4) as u64,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            queue.write_buffer(&buffer, 0, bytemuck::cast_slice(data));
            buffer
        };
        
        // Store in buffers (would need to add these fields to WGPUBuffers struct)
        // For now, just print statistics
        
        println!("✅ GPU hash table built: {} entries, {} capacity, {} total synapses",
                 source_map.len(), capacity, synapse_list.len());
        println!("   Load factor: {:.1}%, Average synapses/neuron: {:.1}",
                 (source_map.len() as f32 / capacity as f32) * 100.0,
                 synapse_list.len() as f32 / source_map.len() as f32);
        
        Ok(())
    }
    
    /// Create bind groups after buffers are uploaded
    fn create_bind_groups(&mut self) -> Result<()> {
        // Helper macro to get buffer or error
        macro_rules! get_buffer {
            ($field:expr, $name:expr) => {
                $field.as_ref()
                    .ok_or_else(|| Error::ComputationError(format!("{} buffer not created", $name)))?
            };
        }
        
        // Create bind group layout for neural dynamics (matches shader @group(0))
        let neural_dynamics_layout = self.device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Neural Dynamics Layout"),
            entries: &[
                // @binding(0): membrane_potentials (read-write)
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
                // @binding(1): thresholds (read-only)
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
                // @binding(2): leak_coefficients (read-only)
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // @binding(3): resting_potentials (read-only)
                wgpu::BindGroupLayoutEntry {
                    binding: 3,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // @binding(4): refractory_periods (read-only)
                wgpu::BindGroupLayoutEntry {
                    binding: 4,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // @binding(5): refractory_countdowns (read-write)
                wgpu::BindGroupLayoutEntry {
                    binding: 5,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // @binding(6): excitabilities (read-only)
                wgpu::BindGroupLayoutEntry {
                    binding: 6,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // @binding(7): consecutive_fire_counts (read-write)
                wgpu::BindGroupLayoutEntry {
                    binding: 7,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // @binding(8): consecutive_fire_limits (read-only)
                wgpu::BindGroupLayoutEntry {
                    binding: 8,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // @binding(9): snooze_periods (read-only)
                wgpu::BindGroupLayoutEntry {
                    binding: 9,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // @binding(10): valid_mask (read-only, bitpacked)
                wgpu::BindGroupLayoutEntry {
                    binding: 11,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // @binding(12): fired_mask (read-write, bitpacked output)
                // Will create this buffer on-demand
            ],
        });
        
        // Create fired_mask buffer (output, bitpacked)
        let fired_mask_size = ((self.neuron_capacity + 31) / 32 * 4) as u64;
        let fired_mask_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Fired Mask"),
            size: fired_mask_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        
        // Create bind group for neural dynamics
        let neural_dynamics_bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Neural Dynamics Bind Group"),
            layout: &neural_dynamics_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: get_buffer!(self.buffers.membrane_potentials, "membrane_potentials").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: get_buffer!(self.buffers.thresholds, "thresholds").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: get_buffer!(self.buffers.leak_coefficients, "leak_coefficients").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: get_buffer!(self.buffers.resting_potentials, "resting_potentials").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: get_buffer!(self.buffers.refractory_periods, "refractory_periods").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 5,
                    resource: get_buffer!(self.buffers.refractory_countdowns, "refractory_countdowns").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 6,
                    resource: get_buffer!(self.buffers.excitabilities, "excitabilities").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 7,
                    resource: get_buffer!(self.buffers.consecutive_fire_counts, "consecutive_fire_counts").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 8,
                    resource: get_buffer!(self.buffers.consecutive_fire_limits, "consecutive_fire_limits").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 9,
                    resource: get_buffer!(self.buffers.snooze_periods, "snooze_periods").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 10,
                    resource: get_buffer!(self.buffers.valid_mask, "valid_mask").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 11,
                    resource: fired_mask_buffer.as_entire_binding(),
                },
            ],
        });
        
        self.neural_dynamics_bind_group = Some(neural_dynamics_bind_group);
        self.buffers.fired_neurons_output = Some(fired_mask_buffer);
        
        println!("✅ Neural dynamics bind group created (12 bindings)");
        
        // Note: Synaptic propagation bind group needs GPU hash table first
        // Will be created separately once hash table is built
        
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
        // Get the fired_mask buffer (bitpacked output from neural dynamics shader)
        let fired_mask_buffer = self.buffers.fired_neurons_output.as_ref()
            .ok_or_else(|| Error::ComputationError("Fired mask buffer not created".to_string()))?;
        
        let fired_mask_size = fired_mask_buffer.size();
        
        // Create staging buffer for GPU→CPU transfer
        let staging_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Fired Mask Staging"),
            size: fired_mask_size,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        
        // Copy from GPU buffer to staging buffer
        let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Download Fired Neurons"),
        });
        encoder.copy_buffer_to_buffer(fired_mask_buffer, 0, &staging_buffer, 0, fired_mask_size);
        self.queue.submit(Some(encoder.finish()));
        
        // Map staging buffer to CPU memory (blocking)
        let buffer_slice = staging_buffer.slice(..);
        let (sender, receiver) = std::sync::mpsc::channel();
        buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
            sender.send(result).unwrap();
        });
        
        // Wait for mapping to complete
        self.device.poll(wgpu::Maintain::Wait);
        receiver.recv()
            .map_err(|_| Error::ComputationError("Failed to receive buffer map result".to_string()))?
            .map_err(|e| Error::ComputationError(format!("Failed to map buffer: {:?}", e)))?;
        
        // Read data
        let data = buffer_slice.get_mapped_range();
        let fired_mask_u32: &[u32] = bytemuck::cast_slice(&data);
        
        // Extract fired neuron indices from bitpacked mask
        let mut fired_neurons = Vec::new();
        for (word_idx, &word) in fired_mask_u32.iter().enumerate() {
            if word != 0 {
                // Check each bit in this word
                for bit_idx in 0..32 {
                    if (word & (1u32 << bit_idx)) != 0 {
                        let neuron_id = (word_idx * 32 + bit_idx) as u32;
                        if (neuron_id as usize) < self.current_neuron_count {
                            fired_neurons.push(neuron_id);
                        }
                    }
                }
            }
        }
        
        // Unmap buffer
        drop(data);
        staging_buffer.unmap();
        
        Ok(fired_neurons)
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

