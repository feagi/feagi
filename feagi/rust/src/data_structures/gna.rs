use std::alloc::{alloc, Layout};

/// Global Neuron Array (GNA) optimized for both SIMD and GPU processing
/// 
/// Uses Structure of Arrays (SoA) layout with memory alignment for:
/// - Efficient SIMD operations on CPU (aligned to 64-byte boundaries)
/// - Coalesced memory access on GPU
#[repr(C, align(64))]
pub struct GlobalNeuronArray {
    /// Number of neurons in the array
    pub neuron_count: usize,
    
    /// Membrane potentials of all neurons
    pub membrane_potentials: Vec<f32>,
    
    /// Firing thresholds for all neurons 
    pub thresholds: Vec<f32>,
    
    /// Refractory periods (in timesteps)
    pub refractory_periods: Vec<u32>,
    
    /// Timestamp of last firing for each neuron
    pub last_fired: Vec<u32>,
    
    /// Current refractory counters
    pub refractory_counters: Vec<u32>,
    
    /// Neuron types (excitatory/inhibitory/etc)
    pub neuron_types: Vec<u8>,
    
    /// Whether each neuron is enabled
    pub enabled_flags: Vec<u8>,
    
    /// Cortical area ID for each neuron
    pub cortical_area_ids: Vec<u32>,
    
    /// 3D coordinates of neurons (x,y,z packed into sequential memory)
    pub coordinates_x: Vec<u16>,
    pub coordinates_y: Vec<u16>,
    pub coordinates_z: Vec<u16>,
}

impl GlobalNeuronArray {
    /// Create a new GNA with specified capacity
    pub fn new(capacity: usize) -> Self {
        // Ensure capacity is aligned to 64-byte boundaries (16 f32 values)
        let aligned_capacity = (capacity + 15) & !15;
        
        Self {
            neuron_count: capacity,
            membrane_potentials: vec![0.0; aligned_capacity],
            thresholds: vec![1.0; aligned_capacity],
            refractory_periods: vec![0; aligned_capacity],
            last_fired: vec![0; aligned_capacity],
            refractory_counters: vec![0; aligned_capacity],
            neuron_types: vec![0; aligned_capacity],
            enabled_flags: vec![1; aligned_capacity], // Default to enabled
            cortical_area_ids: vec![0; aligned_capacity],
            coordinates_x: vec![0; aligned_capacity],
            coordinates_y: vec![0; aligned_capacity],
            coordinates_z: vec![0; aligned_capacity],
        }
    }
    
    /// Update membrane potentials using SIMD operations
    #[cfg(target_feature = "avx2")]
    pub fn update_membrane_potentials_simd(&mut self, decay_factor: f32) {
        use std::arch::x86_64::{__m256, _mm256_loadu_ps, _mm256_mul_ps, _mm256_storeu_ps};
        
        let decay_vector = unsafe { _mm256_set1_ps(decay_factor) };
        
        for i in (0..self.membrane_potentials.len()).step_by(8) {
            unsafe {
                let potentials = _mm256_loadu_ps(self.membrane_potentials[i..].as_ptr());
                let decayed = _mm256_mul_ps(potentials, decay_vector);
                _mm256_storeu_ps(self.membrane_potentials[i..].as_mut_ptr(), decayed);
            }
        }
    }
    
    /// Scalar fallback for membrane potential updates
    #[cfg(not(target_feature = "avx2"))]
    pub fn update_membrane_potentials_simd(&mut self, decay_factor: f32) {
        for potential in &mut self.membrane_potentials {
            *potential *= decay_factor;
        }
    }
    
    /// Prepare data for GPU processing
    pub fn to_gpu_buffer(&self) -> Vec<u8> {
        // Simplified for now - in production would use proper GPU bindings
        let mut buffer = Vec::new();
        
        // Add membrane potentials
        let potential_bytes = bytemuck::cast_slice(&self.membrane_potentials);
        buffer.extend_from_slice(potential_bytes);
        
        // Add thresholds
        let threshold_bytes = bytemuck::cast_slice(&self.thresholds);
        buffer.extend_from_slice(threshold_bytes);
        
        // Add other fields as needed
        // ...
        
        buffer
    }
    
    /// Check which neurons are ready to fire (above threshold)
    pub fn find_fire_candidates(&self, timestep: u32) -> Vec<u32> {
        let mut candidates = Vec::new();
        
        for i in 0..self.neuron_count {
            // Skip neurons in refractory period
            if self.refractory_counters[i] > 0 {
                continue;
            }
            
            // Skip disabled neurons
            if self.enabled_flags[i] == 0 {
                continue;
            }
            
            // Check if above threshold
            if self.membrane_potentials[i] >= self.thresholds[i] {
                candidates.push(i as u32);
            }
        }
        
        candidates
    }
    
    /// Reset membrane potential and update refractory state for fired neurons
    pub fn process_fired_neurons(&mut self, fired_neurons: &[u32], current_timestep: u32) {
        for &neuron_id in fired_neurons {
            let id = neuron_id as usize;
            if id < self.neuron_count {
                // Reset membrane potential
                self.membrane_potentials[id] = 0.0;
                
                // Set refractory counter
                self.refractory_counters[id] = self.refractory_periods[id];
                
                // Update last fired timestamp
                self.last_fired[id] = current_timestep;
            }
        }
    }
    
    /// Update refractory counters for all neurons
    pub fn update_refractory_counters(&mut self) {
        for counter in &mut self.refractory_counters {
            if *counter > 0 {
                *counter -= 1;
            }
        }
    }
}

// WebGPU-compatible representation for the GNA
#[repr(C)]
pub struct GpuNeuronData {
    pub membrane_potentials_offset: u32,
    pub thresholds_offset: u32,
    pub refractory_counters_offset: u32,
    pub neuron_count: u32,
    pub current_timestep: u32,
} 