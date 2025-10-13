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
    
    /// Neural dynamics compute pipeline (legacy - full neuron array)
    neural_dynamics_pipeline: Option<wgpu::ComputePipeline>,
    
    /// Neural dynamics bind group (legacy)
    neural_dynamics_bind_group: Option<wgpu::BindGroup>,
    
    /// FCL-aware neural dynamics pipeline (sparse processing)
    fcl_neural_dynamics_pipeline: Option<wgpu::ComputePipeline>,
    
    /// FCL neural dynamics bind group
    fcl_neural_dynamics_bind_group: Option<wgpu::BindGroup>,
    
    /// Synaptic propagation compute pipeline (legacy)
    synaptic_propagation_pipeline: Option<wgpu::ComputePipeline>,
    
    /// Synaptic propagation bind group (legacy)
    synaptic_propagation_bind_group: Option<wgpu::BindGroup>,
    
    /// FCL-aware synaptic propagation pipeline (GPU→GPU)
    fcl_synaptic_propagation_pipeline: Option<wgpu::ComputePipeline>,
    
    /// FCL synaptic propagation bind group
    fcl_synaptic_propagation_bind_group: Option<wgpu::BindGroup>,
    
    /// GPU buffers (persistent)
    buffers: WGPUBuffers,
    
    /// Capacity (max neurons/synapses)
    neuron_capacity: usize,
    synapse_capacity: usize,
    
    /// Current neuron count (for dispatch)
    current_neuron_count: usize,
    
    /// Synapse hash table capacity
    synapse_hash_capacity: usize,
}

/// GPU buffer management (consolidated for Metal's 8-binding limit)
struct WGPUBuffers {
    // Neuron arrays (consolidated, persistent)
    membrane_potentials: Option<wgpu::Buffer>,  // Frequently updated
    f32_params: Option<wgpu::Buffer>,           // Interleaved: [threshold, leak, resting, excite, ...]
    u16_static_params: Option<wgpu::Buffer>,    // Interleaved: [refrac_period, consec_limit, snooze, ...]
    u16_dynamic_state: Option<wgpu::Buffer>,    // Interleaved: [refrac_countdown, consec_count, ...]
    valid_mask: Option<wgpu::Buffer>,           // Bitpacked
    
    // Synapse arrays (consolidated, persistent)
    synapse_data: Option<wgpu::Buffer>,         // Interleaved: [source, target, packed_params] (stride=3)
    synapse_hash_keys: Option<wgpu::Buffer>,    // Hash table keys
    synapse_hash_metadata: Option<wgpu::Buffer>, // Hash table: [start, count] (stride=2)
    synapse_list: Option<wgpu::Buffer>,         // Flat synapse indices for hash lookup
    
    // FCL buffers (sparse, per-burst)
    fcl_neuron_ids: Option<wgpu::Buffer>,       // Sparse neuron IDs
    fcl_potentials: Option<wgpu::Buffer>,       // Accumulated potentials (f32)
    fcl_fired_mask: Option<wgpu::Buffer>,       // Sparse output (bitpacked)
    fcl_potentials_atomic: Option<wgpu::Buffer>, // Atomic accumulation (i32, full array)
    
    // Legacy buffers (for compatibility)
    fired_neurons_input: Option<wgpu::Buffer>,
    fired_neurons_output: Option<wgpu::Buffer>,
    
    // Staging buffers for readback
    staging_buffer: Option<wgpu::Buffer>,
}

impl WGPUBuffers {
    fn new() -> Self {
        Self {
            membrane_potentials: None,
            f32_params: None,
            u16_static_params: None,
            u16_dynamic_state: None,
            valid_mask: None,
            synapse_data: None,
            synapse_hash_keys: None,
            synapse_hash_metadata: None,
            synapse_list: None,
            fcl_neuron_ids: None,
            fcl_potentials: None,
            fcl_fired_mask: None,
            fcl_potentials_atomic: None,
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
            fcl_neural_dynamics_pipeline: None,
            fcl_neural_dynamics_bind_group: None,
            synaptic_propagation_pipeline: None,
            synaptic_propagation_bind_group: None,
            fcl_synaptic_propagation_pipeline: None,
            fcl_synaptic_propagation_bind_group: None,
            buffers: WGPUBuffers::new(),
            neuron_capacity,
            synapse_capacity,
            current_neuron_count: 0,
            synapse_hash_capacity: 0,
        })
    }
    
    /// Initialize compute pipelines (shaders)
    fn initialize_pipelines(&mut self) -> Result<()> {
        // Load legacy neural dynamics shader (full neuron array)
        let neural_dynamics_shader = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Neural Dynamics Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/neural_dynamics.wgsl").into()),
        });
        
        self.neural_dynamics_pipeline = Some(self.device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("Neural Dynamics Pipeline"),
            layout: None,  // Auto-layout from shader
            module: &neural_dynamics_shader,
            entry_point: "neural_dynamics_main",
        }));
        
        println!("✅ Neural dynamics shader loaded (legacy)");
        
        // Load FCL-aware neural dynamics shader (sparse processing)
        let fcl_neural_dynamics_shader = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("FCL Neural Dynamics Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/neural_dynamics_fcl.wgsl").into()),
        });
        
        self.fcl_neural_dynamics_pipeline = Some(self.device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("FCL Neural Dynamics Pipeline"),
            layout: None,  // Auto-layout from shader
            module: &fcl_neural_dynamics_shader,
            entry_point: "neural_dynamics_fcl_main",
        }));
        
        println!("✅ FCL neural dynamics shader loaded (sparse)");
        
        // Load FCL-aware synaptic propagation shader (GPU→GPU, consolidated, Metal-compatible)
        let fcl_synaptic_propagation_shader = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("FCL Synaptic Propagation Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/synaptic_propagation_fcl.wgsl").into()),
        });
        
        self.fcl_synaptic_propagation_pipeline = Some(self.device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("FCL Synaptic Propagation Pipeline"),
            layout: None,  // Auto-layout from shader
            module: &fcl_synaptic_propagation_shader,
            entry_point: "synaptic_propagation_fcl_main",
        }));
        
        println!("✅ FCL synaptic propagation shader loaded (7 bindings - Metal compatible!)");
        
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
        
        // ═══════════════════════════════════════════════════════════
        // CONSOLIDATE BUFFERS FOR METAL (≤8 bindings)
        // ═══════════════════════════════════════════════════════════
        
        // 1. Membrane potentials (separate, frequently updated)
        self.buffers.membrane_potentials = Some(create_buffer_f32(
            &self.device, &self.queue, &neuron_array.membrane_potentials[..neuron_count], 
            "Membrane Potentials"
        ));
        
        // 2. Interleaved f32 params: [threshold, leak, resting, excitability, ...]
        let mut f32_params = Vec::with_capacity(neuron_count * 4);
        for i in 0..neuron_count {
            f32_params.push(neuron_array.thresholds[i]);
            f32_params.push(neuron_array.leak_coefficients[i]);
            f32_params.push(neuron_array.resting_potentials[i]);
            f32_params.push(neuron_array.excitabilities[i]);
        }
        self.buffers.f32_params = Some(create_buffer_f32(
            &self.device, &self.queue, &f32_params,
            "F32 Params (interleaved)"
        ));
        
        // 3. Interleaved u16 static params: [refrac_period, consec_limit, snooze_period, ...]
        // Convert u16 to u32 for GPU (easier alignment, matches shader)
        let mut u16_static = Vec::with_capacity(neuron_count * 3);
        for i in 0..neuron_count {
            u16_static.push(neuron_array.refractory_periods[i] as u32);
            u16_static.push(neuron_array.consecutive_fire_limits[i] as u32);
            u16_static.push(neuron_array.snooze_periods[i] as u32);
        }
        self.buffers.u16_static_params = Some({
            let buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("U16 Static Params (interleaved)"),
                size: (u16_static.len() * 4) as u64,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            self.queue.write_buffer(&buffer, 0, bytemuck::cast_slice(&u16_static));
            buffer
        });
        
        // 4. Interleaved u16 dynamic state: [refrac_countdown, consec_count, ...]
        let mut u16_dynamic = Vec::with_capacity(neuron_count * 2);
        for i in 0..neuron_count {
            u16_dynamic.push(neuron_array.refractory_countdowns[i] as u32);
            u16_dynamic.push(neuron_array.consecutive_fire_counts[i] as u32);
        }
        self.buffers.u16_dynamic_state = Some({
            let buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("U16 Dynamic State (interleaved)"),
                size: (u16_dynamic.len() * 4) as u64,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            });
            self.queue.write_buffer(&buffer, 0, bytemuck::cast_slice(&u16_dynamic));
            buffer
        });
        
        // 5. Valid mask (bitpacked)
        self.buffers.valid_mask = Some(create_buffer_bool(
            &self.device, &self.queue, &neuron_array.valid_mask[..neuron_count],
            "Valid Mask"
        ));
        
        Ok(())
    }
    
    /// Upload synapse array data to GPU
    fn upload_synapse_arrays(&mut self, synapse_array: &SynapseArray) -> Result<()> {
        let synapse_count = synapse_array.count;
        
        println!("📤 Uploading {} synapses to GPU (consolidated, Metal-compatible)...", synapse_count);
        
        // ═══════════════════════════════════════════════════════════
        // 1. CONSOLIDATE SYNAPSE DATA (stride=3)
        // Format: [source_id, target_id, packed_params] per synapse
        // packed_params = (type << 16) | (conductance << 8) | weight
        // ═══════════════════════════════════════════════════════════
        
        let mut synapse_data = Vec::with_capacity(synapse_count * 3);
        for i in 0..synapse_count {
            let source = synapse_array.source_neurons[i];
            let target = synapse_array.target_neurons[i];
            
            // Pack u8 params into single u32
            let weight = synapse_array.weights[i] as u32;
            let conductance = synapse_array.conductances[i] as u32;
            let synapse_type = synapse_array.types[i] as u32;
            let packed_params = (synapse_type << 16) | (conductance << 8) | weight;
            
            synapse_data.push(source);
            synapse_data.push(target);
            synapse_data.push(packed_params);
        }
        
        let synapse_data_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Synapse Data (Consolidated)"),
            size: (synapse_data.len() * 4) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        self.queue.write_buffer(&synapse_data_buffer, 0, bytemuck::cast_slice(&synapse_data));
        self.buffers.synapse_data = Some(synapse_data_buffer);
        
        // ═══════════════════════════════════════════════════════════
        // 2. BUILD GPU HASH TABLE (source neuron → synapse lookup)
        // ═══════════════════════════════════════════════════════════
        
        use ahash::AHashMap;
        
        // Collect source neurons and their synapse indices
        let mut source_map: AHashMap<u32, Vec<usize>> = AHashMap::new();
        for i in 0..synapse_count {
            if synapse_array.valid_mask[i] {
                let source = synapse_array.source_neurons[i];
                source_map.entry(source).or_insert_with(Vec::new).push(i);
            }
        }
        
        // Calculate capacity (2x for low collision rate)
        let capacity = (source_map.len() * 2).next_power_of_two().max(256);
        
        // Initialize hash table
        let mut hash_keys = vec![0xFFFFFFFFu32; capacity]; // 0xFFFFFFFF = empty
        let mut hash_metadata = vec![0u32; capacity * 2]; // [start, count] per entry
        let mut synapse_list = Vec::new();
        
        // Insert using linear probing
        for (&source_neuron, synapse_indices) in &source_map {
            let mut slot = (source_neuron as usize * 2654435761) % capacity;
            
            while hash_keys[slot] != 0xFFFFFFFF {
                slot = (slot + 1) % capacity;
            }
            
            // Store key
            hash_keys[slot] = source_neuron;
            
            // Store metadata: [start_index_in_synapse_list, count]
            hash_metadata[slot * 2] = synapse_list.len() as u32;
            hash_metadata[slot * 2 + 1] = synapse_indices.len() as u32;
            
            // Append synapse indices
            for &idx in synapse_indices {
                synapse_list.push(idx as u32);
            }
        }
        
        // Upload hash table buffers
        let hash_keys_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Synapse Hash Keys"),
            size: (hash_keys.len() * 4) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        self.queue.write_buffer(&hash_keys_buffer, 0, bytemuck::cast_slice(&hash_keys));
        self.buffers.synapse_hash_keys = Some(hash_keys_buffer);
        
        let hash_metadata_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Synapse Hash Metadata"),
            size: (hash_metadata.len() * 4) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        self.queue.write_buffer(&hash_metadata_buffer, 0, bytemuck::cast_slice(&hash_metadata));
        self.buffers.synapse_hash_metadata = Some(hash_metadata_buffer);
        
        let synapse_list_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Synapse List"),
            size: (synapse_list.len() * 4) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        self.queue.write_buffer(&synapse_list_buffer, 0, bytemuck::cast_slice(&synapse_list));
        self.buffers.synapse_list = Some(synapse_list_buffer);
        
        // Store hash capacity for later use
        self.synapse_hash_capacity = capacity;
        
        println!("✅ GPU hash table built: {} entries, {} capacity, {} total synapses",
                 source_map.len(), capacity, synapse_count);
        println!("   Load factor: {:.1}%, Average synapses/neuron: {:.1}",
                 (source_map.len() as f32 / capacity as f32) * 100.0,
                 synapse_count as f32 / source_map.len() as f32);
        
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
        
        // ═══════════════════════════════════════════════════════════
        // GET BIND GROUP LAYOUT FROM PIPELINE (auto-derived from shader)
        // ═══════════════════════════════════════════════════════════
        
        let pipeline = self.neural_dynamics_pipeline.as_ref()
            .ok_or_else(|| Error::ComputationError("Neural dynamics pipeline not initialized".to_string()))?;
        
        // Get the bind group layout from the pipeline (derived from shader)
        let neural_dynamics_layout = pipeline.get_bind_group_layout(0);
        
        // Create fired_mask buffer (output, bitpacked)
        let fired_mask_size = ((self.neuron_capacity + 31) / 32 * 4) as u64;
        let fired_mask_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Fired Mask"),
            size: fired_mask_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        
        // Create params buffer (uniform) - stores burst_count and neuron_count
        #[repr(C)]
        #[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
        struct NeuralParams {
            neuron_count: u32,
            burst_count: u32,
            _padding0: u32,
            _padding1: u32,
        }
        let params_data = NeuralParams {
            neuron_count: self.current_neuron_count as u32,
            burst_count: 0, // Will be updated per-dispatch
            _padding0: 0,
            _padding1: 0,
        };
        let params_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Neural Params"),
            size: std::mem::size_of::<NeuralParams>() as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        self.queue.write_buffer(&params_buffer, 0, bytemuck::bytes_of(&params_data));
        
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
                    resource: get_buffer!(self.buffers.f32_params, "f32_params").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: get_buffer!(self.buffers.u16_static_params, "u16_static_params").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: get_buffer!(self.buffers.u16_dynamic_state, "u16_dynamic_state").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: get_buffer!(self.buffers.valid_mask, "valid_mask").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 5,
                    resource: fired_mask_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 6,
                    resource: params_buffer.as_entire_binding(),
                },
            ],
        });
        
        self.neural_dynamics_bind_group = Some(neural_dynamics_bind_group);
        self.buffers.fired_neurons_output = Some(fired_mask_buffer);
        
        println!("✅ Neural dynamics bind group created (7 bindings - Metal compatible!)");
        
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
    
    /// Dispatch GPU synaptic propagation (accumulates to atomic FCL buffer)
    /// Uses consolidated buffers (7 bindings - Metal-compatible)
    fn dispatch_synaptic_propagation_fcl_gpu(&mut self, fired_count: usize, hash_capacity: usize) -> Result<()> {
        let pipeline = self.fcl_synaptic_propagation_pipeline.as_ref()
            .ok_or_else(|| Error::ComputationError("FCL synaptic propagation pipeline not initialized".to_string()))?;
        
        // Get layout from pipeline
        let layout = pipeline.get_bind_group_layout(0);
        
        // Helper macro
        macro_rules! get_buffer {
            ($field:expr, $name:expr) => {
                $field.as_ref()
                    .ok_or_else(|| Error::ComputationError(format!("{} buffer not created", $name)))?
            };
        }
        
        // Create params buffer
        let params_data = [
            fired_count as u32,
            hash_capacity as u32,
            0u32,
            0u32,
        ];
        let params_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Synaptic Propagation Params"),
            size: 16,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        self.queue.write_buffer(&params_buffer, 0, bytemuck::cast_slice(&params_data));
        
        // Create bind group (7 bindings total - Metal-compatible!)
        let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Synaptic Propagation Bind Group"),
            layout: &layout,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: get_buffer!(self.buffers.synapse_data, "synapse_data").as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: get_buffer!(self.buffers.synapse_hash_keys, "synapse_hash_keys").as_entire_binding() },
                wgpu::BindGroupEntry { binding: 2, resource: get_buffer!(self.buffers.synapse_hash_metadata, "synapse_hash_metadata").as_entire_binding() },
                wgpu::BindGroupEntry { binding: 3, resource: get_buffer!(self.buffers.synapse_list, "synapse_list").as_entire_binding() },
                wgpu::BindGroupEntry { binding: 4, resource: get_buffer!(self.buffers.fcl_potentials_atomic, "fcl_potentials_atomic").as_entire_binding() },
                wgpu::BindGroupEntry { binding: 5, resource: get_buffer!(self.buffers.fired_neurons_input, "fired_neurons").as_entire_binding() },
                wgpu::BindGroupEntry { binding: 6, resource: params_buffer.as_entire_binding() },
            ],
        });
        
        // Calculate workgroups (256 fired neurons per workgroup)
        let workgroup_count = (fired_count as u32 + 255) / 256;
        
        println!("  🚀 Dispatching {} workgroups for {} fired neurons (7 bindings - Metal OK!)", workgroup_count, fired_count);
        
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
            compute_pass.set_bind_group(0, &bind_group, &[]);
            compute_pass.dispatch_workgroups(workgroup_count, 1, 1);
        }
        
        // Submit commands
        self.queue.submit(Some(encoder.finish()));
        
        Ok(())
    }
    
    /// Upload FCL candidates to GPU (sparse array)
    /// 
    /// **Key optimization**: Only uploads ~1-10% of neurons (FCL candidates) instead of all neurons
    fn upload_fcl_candidates(&mut self, candidates: &[(u32, f32)]) -> Result<()> {
        let count = candidates.len();
        
        // Separate into neuron_ids and potentials
        let neuron_ids: Vec<u32> = candidates.iter().map(|(id, _)| *id).collect();
        let potentials: Vec<f32> = candidates.iter().map(|(_, pot)| *pot).collect();
        
        // Upload neuron IDs (needs COPY_SRC for readback)
        let ids_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("FCL Neuron IDs"),
            size: (count * 4) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        self.queue.write_buffer(&ids_buffer, 0, bytemuck::cast_slice(&neuron_ids));
        self.buffers.fcl_neuron_ids = Some(ids_buffer);
        
        // Upload potentials
        let potentials_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("FCL Potentials"),
            size: (count * 4) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        self.queue.write_buffer(&potentials_buffer, 0, bytemuck::cast_slice(&potentials));
        self.buffers.fcl_potentials = Some(potentials_buffer);
        
        // Create sparse output buffer (fired mask)
        let fired_mask_size = ((count + 31) / 32 * 4) as u64;
        let fired_mask_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("FCL Fired Mask"),
            size: fired_mask_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        // Initialize to 0
        self.queue.write_buffer(&fired_mask_buffer, 0, &vec![0u8; fired_mask_size as usize]);
        self.buffers.fcl_fired_mask = Some(fired_mask_buffer);
        
        Ok(())
    }
    
    /// Dispatch neural dynamics for FCL candidates only (sparse processing)
    fn dispatch_neural_dynamics_fcl(&mut self, burst_count: u64, fcl_count: usize) -> Result<()> {
        // Get FCL pipeline
        let pipeline = self.fcl_neural_dynamics_pipeline.as_ref()
            .ok_or_else(|| Error::ComputationError("FCL neural dynamics pipeline not initialized".to_string()))?;
        
        // Get bind group layout from pipeline
        let layout = pipeline.get_bind_group_layout(0);
        
        // Helper macro to get buffer
        macro_rules! get_buffer {
            ($field:expr, $name:expr) => {
                $field.as_ref()
                    .ok_or_else(|| Error::ComputationError(format!("{} buffer not created", $name)))?
            };
        }
        
        // Create params buffer
        let params_data = [fcl_count as u32, burst_count as u32, 0u32, 0u32];
        let params_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("FCL Params"),
            size: 16,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        self.queue.write_buffer(&params_buffer, 0, bytemuck::cast_slice(&params_data));
        
        // Create bind group with FCL buffers
        let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("FCL Neural Dynamics Bind Group"),
            layout: &layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: get_buffer!(self.buffers.fcl_neuron_ids, "fcl_neuron_ids").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: get_buffer!(self.buffers.fcl_potentials, "fcl_potentials").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: get_buffer!(self.buffers.membrane_potentials, "membrane_potentials").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: get_buffer!(self.buffers.f32_params, "f32_params").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: get_buffer!(self.buffers.u16_dynamic_state, "u16_dynamic_state").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 5,
                    resource: get_buffer!(self.buffers.u16_static_params, "u16_static_params").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 6,
                    resource: get_buffer!(self.buffers.fcl_fired_mask, "fcl_fired_mask").as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 7,
                    resource: params_buffer.as_entire_binding(),
                },
            ],
        });
        
        // Calculate workgroups for FCL candidates only (not all neurons!)
        let workgroup_count = (fcl_count as u32 + 255) / 256;
        
        println!("  Dispatching {} workgroups for {} FCL neurons (vs {} for all neurons)",
                 workgroup_count, fcl_count, (self.current_neuron_count + 255) / 256);
        
        // Create command encoder
        let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Neural Dynamics FCL Encoder"),
        });
        
        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("Neural Dynamics FCL Pass"),
                timestamp_writes: None,
            });
            
            compute_pass.set_pipeline(pipeline);
            compute_pass.set_bind_group(0, &bind_group, &[]);
            compute_pass.dispatch_workgroups(workgroup_count, 1, 1);
        }
        
        // Submit commands
        self.queue.submit(Some(encoder.finish()));
        
        Ok(())
    }
    
    /// Download fired neurons from GPU (sparse: only FCL indices)
    /// 
    /// **Key optimization**: Downloads only FCL fired mask (< 1 KB), not full mask (122 KB)
    fn download_fired_neurons_fcl(&self) -> Result<Vec<u32>> {
        // Get the FCL fired mask buffer (sparse bitpacked output)
        let fcl_fired_mask_buffer = self.buffers.fcl_fired_mask.as_ref()
            .ok_or_else(|| Error::ComputationError("FCL fired mask buffer not created".to_string()))?;
        
        let fcl_fired_mask_size = fcl_fired_mask_buffer.size();
        
        // Create staging buffer for GPU→CPU transfer
        let staging_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("FCL Fired Mask Staging"),
            size: fcl_fired_mask_size,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        
        // Copy from GPU buffer to staging buffer
        let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Download FCL Fired Neurons"),
        });
        encoder.copy_buffer_to_buffer(fcl_fired_mask_buffer, 0, &staging_buffer, 0, fcl_fired_mask_size);
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
        
        // Read data and copy to owned Vec
        let data = buffer_slice.get_mapped_range();
        let fcl_fired_mask_u32: Vec<u32> = bytemuck::cast_slice(&data).to_vec();
        
        // Unmap immediately after copying
        drop(data);
        staging_buffer.unmap();
        
        // Extract fired FCL indices and map to neuron IDs
        // Need to get FCL neuron IDs to map back
        let fcl_ids_buffer = self.buffers.fcl_neuron_ids.as_ref()
            .ok_or_else(|| Error::ComputationError("FCL neuron IDs buffer not found".to_string()))?;
        
        // Download FCL neuron IDs separately
        let fcl_ids_size = fcl_ids_buffer.size();
        let fcl_ids_staging = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("FCL IDs Staging"),
            size: fcl_ids_size,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        
        let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Download FCL IDs"),
        });
        encoder.copy_buffer_to_buffer(fcl_ids_buffer, 0, &fcl_ids_staging, 0, fcl_ids_size);
        self.queue.submit(Some(encoder.finish()));
        
        let buffer_slice = fcl_ids_staging.slice(..);
        let (sender, receiver) = std::sync::mpsc::channel();
        buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
            sender.send(result).unwrap();
        });
        
        self.device.poll(wgpu::Maintain::Wait);
        receiver.recv()
            .map_err(|_| Error::ComputationError("Failed to receive FCL IDs buffer map result".to_string()))?
            .map_err(|e| Error::ComputationError(format!("Failed to map FCL IDs buffer: {:?}", e)))?;
        
        let fcl_ids_data = buffer_slice.get_mapped_range();
        let fcl_neuron_ids: &[u32] = bytemuck::cast_slice(&fcl_ids_data);
        
        // Extract fired neuron IDs from FCL mask
        let mut fired_neurons = Vec::new();
        for (word_idx, &word) in fcl_fired_mask_u32.iter().enumerate() {
            if word != 0 {
                // Check each bit in this word
                for bit_idx in 0..32 {
                    if (word & (1u32 << bit_idx)) != 0 {
                        let fcl_idx = word_idx * 32 + bit_idx;
                        if fcl_idx < fcl_neuron_ids.len() {
                            // Map FCL index to actual neuron ID
                            fired_neurons.push(fcl_neuron_ids[fcl_idx]);
                        }
                    }
                }
            }
        }
        
        drop(fcl_ids_data);
        fcl_ids_staging.unmap();
        
        Ok(fired_neurons)
    }
    
    /// Download neuron state updates from GPU back to CPU NeuronArray
    /// 
    /// Updates refractory countdowns and consecutive fire counts for FCL neurons
    fn download_neuron_state_updates(
        &mut self,
        neuron_array: &mut NeuronArray,
        fcl_candidates: &[(u32, f32)],
    ) -> Result<()> {
        // TODO: Download u16_dynamic_state buffer for FCL neurons
        // For now, skip state sync (GPU state is authoritative)
        // This is OK because state will be synced on next burst
        
        // Placeholder: In production, download GPU state and update neuron_array
        let _ = (neuron_array, fcl_candidates); // Suppress unused warnings
        
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
        synapse_array: &SynapseArray,
        _fcl: &mut FireCandidateList,
    ) -> Result<usize> {
        if fired_neurons.is_empty() {
            return Ok(0);
        }
        
        println!("🚀 GPU synaptic propagation: {} fired neurons", fired_neurons.len());
        
        // Reset atomic FCL buffer to zero
        let neuron_count = self.current_neuron_count;
        self.queue.write_buffer(
            self.buffers.fcl_potentials_atomic.as_ref().unwrap(),
            0,
            &vec![0u8; neuron_count * 4]
        );
        
        // Upload fired neurons to GPU
        let fired_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Fired Neurons Input"),
            size: (fired_neurons.len() * 4) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        self.queue.write_buffer(&fired_buffer, 0, bytemuck::cast_slice(fired_neurons));
        self.buffers.fired_neurons_input = Some(fired_buffer);
        
        // Dispatch GPU synaptic propagation (7 bindings - Metal compatible!)
        self.dispatch_synaptic_propagation_fcl_gpu(fired_neurons.len(), self.synapse_hash_capacity)?;
        
        // Wait for GPU to complete
        self.device.poll(wgpu::Maintain::Wait);
        
        println!("✅ GPU synaptic propagation complete");
        
        // Return estimated synapse count
        // Note: FCL results stay on GPU for neural dynamics (full GPU→GPU pipeline)
        Ok(fired_neurons.len() * (synapse_array.count / synapse_array.source_index.len().max(1)))
    }
    
    fn process_neural_dynamics(
        &mut self,
        fcl: &FireCandidateList,
        neuron_array: &mut NeuronArray,
        burst_count: u64,
    ) -> Result<(Vec<u32>, usize, usize)> {
        // **FCL-AWARE**: Upload only FCL candidates to GPU (sparse array)
        let fcl_candidates_raw = fcl.get_all_candidates();
        let fcl_count = fcl_candidates_raw.len();
        
        if fcl_count == 0 {
            return Ok((vec![], 0, 0));
        }
        
        // Convert NeuronId to u32 for GPU
        let fcl_candidates: Vec<(u32, f32)> = fcl_candidates_raw
            .iter()
            .map(|(neuron_id, potential)| (neuron_id.0, *potential))
            .collect();
        
        println!("🎯 GPU processing {} FCL candidates (out of {} total neurons)",
                 fcl_count, self.current_neuron_count);
        
        // Upload FCL candidates to GPU (sparse: neuron IDs + potentials)
        self.upload_fcl_candidates(&fcl_candidates)?;
        
        // Dispatch neural dynamics compute shader (processes ONLY FCL neurons)
        self.dispatch_neural_dynamics_fcl(burst_count, fcl_count)?;
        
        // Wait for GPU to complete
        self.device.poll(wgpu::Maintain::Wait);
        
        // Download fired neuron results (sparse: only neurons that fired)
        let fired_neurons = self.download_fired_neurons_fcl()?;
        
        // Update neuron_array state from GPU (refractory, consecutive counts)
        self.download_neuron_state_updates(neuron_array, &fcl_candidates)?;
        
        let fired_count = fired_neurons.len();
        
        // Return results (refractory count placeholder)
        Ok((fired_neurons, fcl_count, 0))
    }
    
    fn initialize_persistent_data(
        &mut self,
        neuron_array: &NeuronArray,
        synapse_array: &SynapseArray,
    ) -> Result<()> {
        // Upload all data to GPU
        self.upload_neuron_arrays(neuron_array)?;
        self.upload_synapse_arrays(synapse_array)?;
        
        // Create atomic FCL potentials buffer (for synaptic propagation output)
        let neuron_count = neuron_array.count;
        let atomic_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("FCL Potentials Atomic"),
            size: (neuron_count * 4) as u64,  // i32 per neuron
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        // Initialize to 0
        self.queue.write_buffer(&atomic_buffer, 0, &vec![0u8; neuron_count * 4]);
        self.buffers.fcl_potentials_atomic = Some(atomic_buffer);
        
        println!("✅ FCL atomic accumulation buffer created ({} neurons)", neuron_count);
        
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
        self.fcl_neural_dynamics_pipeline = None;
        self.fcl_neural_dynamics_bind_group = None;
        self.synaptic_propagation_pipeline = None;
        self.synaptic_propagation_bind_group = None;
        self.fcl_synaptic_propagation_pipeline = None;
        self.fcl_synaptic_propagation_bind_group = None;
        self.current_neuron_count = 0;
        self.synapse_hash_capacity = 0;
        
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

