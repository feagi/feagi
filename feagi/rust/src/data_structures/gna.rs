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

/// Fire Queue - Structure of Arrays (SoA) for neurons being evaluated for firing
/// 
/// This maintains a queue of neurons that are potential firing candidates,
/// along with all their relevant parameters for efficient processing.
#[repr(C, align(64))]
pub struct FireQueue {
    /// IDs of neurons in the queue
    pub neuron_ids: Vec<u32>,
    
    /// Current membrane potentials
    pub membrane_potentials: Vec<f32>,
    
    /// Firing thresholds
    pub thresholds: Vec<f32>,
    
    /// Number of consecutive times each neuron has fired
    pub consecutive_fire_counts: Vec<u32>,
    
    /// Current refractory counters
    pub refractory_counters: Vec<u32>,
    
    /// Current count of neurons in the queue
    pub count: usize,
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
    
    /// Get the consecutive fire count for a neuron
    pub fn get_consecutive_fire_count(&self, neuron_id: usize) -> u32 {
        // This would typically be stored in the GNA
        // For this implementation, we'll use a dummy value
        0 // Placeholder value
    }
    
    /// Update the consecutive fire count for a neuron
    pub fn update_consecutive_fire_count(&mut self, neuron_id: usize, fired: bool) {
        // This would update the consecutive fire count based on whether the neuron fired
        // Implementation would be added in a production system
    }
}

impl FireQueue {
    /// Creates a new empty FireQueue with pre-allocated capacity
    pub fn new(capacity: usize) -> Self {
        Self {
            neuron_ids: Vec::with_capacity(capacity),
            membrane_potentials: Vec::with_capacity(capacity),
            thresholds: Vec::with_capacity(capacity),
            consecutive_fire_counts: Vec::with_capacity(capacity),
            refractory_counters: Vec::with_capacity(capacity),
            count: 0,
        }
    }
    
    /// Add a neuron to the fire queue
    pub fn add(&mut self, neuron_id: u32, membrane_potential: f32, threshold: f32, 
               consecutive_fire_count: u32, refractory_counter: u32) {
        self.neuron_ids.push(neuron_id);
        self.membrane_potentials.push(membrane_potential);
        self.thresholds.push(threshold);
        self.consecutive_fire_counts.push(consecutive_fire_count);
        self.refractory_counters.push(refractory_counter);
        self.count += 1;
    }
    
    /// Clear the fire queue
    pub fn clear(&mut self) {
        self.neuron_ids.clear();
        self.membrane_potentials.clear();
        self.thresholds.clear();
        self.consecutive_fire_counts.clear();
        self.refractory_counters.clear();
        self.count = 0;
    }
    
    /// Extract neurons that satisfy the activation function criteria
    pub fn extract_fire_candidates(&self, max_consecutive_fires: u32) -> Vec<u32> {
        let mut candidates = Vec::new();
        
        for i in 0..self.count {
            // Skip neurons in refractory period
            if self.refractory_counters[i] > 0 {
                continue;
            }
            
            // Skip neurons exceeding consecutive fire limit (if max_consecutive_fires > 0)
            if max_consecutive_fires > 0 && self.consecutive_fire_counts[i] >= max_consecutive_fires {
                continue;
            }
            
            // Check if above threshold
            if self.membrane_potentials[i] >= self.thresholds[i] {
                candidates.push(self.neuron_ids[i]);
            }
        }
        
        candidates
    }
    
    /// SIMD-optimized extraction of fire candidates (when AVX2 is available)
    #[cfg(target_feature = "avx2")]
    pub fn extract_fire_candidates_simd(&self, max_consecutive_fires: u32) -> Vec<u32> {
        use std::arch::x86_64::{__m256, __m256i, _mm256_loadu_ps, _mm256_cmp_ps_mask, _mm256_set1_ps};
        
        let mut candidates = Vec::new();
        let mut i = 0;
        
        // Process in blocks of 8 (AVX2 width)
        while i + 8 <= self.count {
            unsafe {
                // Load vectors of 8 values
                let membranes = _mm256_loadu_ps(&self.membrane_potentials[i] as *const f32);
                let thresholds = _mm256_loadu_ps(&self.thresholds[i] as *const f32);
                
                // Compare membrane potentials >= thresholds
                let mask = _mm256_cmp_ps_mask(membranes, thresholds, 0x0D); // _CMP_GE_OQ
                
                // Check each bit of the mask
                for j in 0..8 {
                    let idx = i + j;
                    if (mask & (1 << j)) != 0 && 
                       self.refractory_counters[idx] == 0 && 
                       (max_consecutive_fires == 0 || self.consecutive_fire_counts[idx] < max_consecutive_fires) {
                        candidates.push(self.neuron_ids[idx]);
                    }
                }
            }
            
            i += 8;
        }
        
        // Handle remaining elements
        for idx in i..self.count {
            if self.refractory_counters[idx] == 0 && 
               (max_consecutive_fires == 0 || self.consecutive_fire_counts[idx] < max_consecutive_fires) && 
               self.membrane_potentials[idx] >= self.thresholds[idx] {
                candidates.push(self.neuron_ids[idx]);
            }
        }
        
        candidates
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