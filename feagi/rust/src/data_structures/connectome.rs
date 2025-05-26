/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/// Connectome implementation using Compressed Sparse Row (CSR) format
/// 
/// This implementation is optimized for both:
/// - SIMD operations on CPU (with 64-byte cache line alignment)
/// - GPU compute shader execution (with coalesced memory access)

use crate::data_structures::gna::{GlobalNeuronArray, FireQueue};

#[repr(C, align(64))]
pub struct Connectome {
    /// Number of neurons (rows in the matrix)
    pub neuron_count: usize,
    
    /// Row pointers - Starting index in the column array for each neuron's connections
    /// Length is neuron_count + 1, with the last entry marking the end
    pub row_ptr: Vec<u32>,
    
    /// Column indices - Target neuron IDs for each connection
    pub col_idx: Vec<u32>,
    
    /// Synaptic weights for each connection
    pub weights: Vec<f32>,
    
    /// Synaptic delays for each connection (in timesteps)
    pub delays: Vec<u8>,
    
    /// Connection types for each synapse (excitatory/inhibitory/modulatory)
    pub types: Vec<u8>,
    
    /// Parent cortical area ID for source neurons
    pub source_area_ids: Vec<u32>,
    
    /// Parent cortical area ID for target neurons
    pub target_area_ids: Vec<u32>,
}

impl Connectome {
    /// Creates a new empty connectome with capacity for the specified number of neurons
    pub fn new(neuron_count: usize, estimated_edges: usize) -> Self {
        let mut row_ptr = Vec::with_capacity(neuron_count + 1);
        row_ptr.resize(neuron_count + 1, 0);
        
        // Preallocate with estimated capacity
        let col_idx = Vec::with_capacity(estimated_edges);
        let weights = Vec::with_capacity(estimated_edges);
        let delays = Vec::with_capacity(estimated_edges);
        let types = Vec::with_capacity(estimated_edges);
        let source_area_ids = Vec::with_capacity(estimated_edges);
        let target_area_ids = Vec::with_capacity(estimated_edges);
        
        Self {
            neuron_count,
            row_ptr,
            col_idx,
            weights,
            delays,
            types,
            source_area_ids,
            target_area_ids,
        }
    }
    
    /// Adds a connection from source_id to target_id with the given weight
    pub fn add_connection(
        &mut self, 
        source_id: u32, 
        target_id: u32, 
        weight: f32,
        delay: u8,
        connection_type: u8,
        source_area_id: u32,
        target_area_id: u32
    ) -> Result<(), &'static str> {
        if source_id as usize >= self.neuron_count {
            return Err("Source neuron ID out of range");
        }
        
        // Insert the new connection
        let insert_pos = self.row_ptr[source_id as usize + 1] as usize;
        
        // Update all row pointers after the insertion point
        for i in (source_id as usize + 1)..=self.neuron_count {
            self.row_ptr[i] += 1;
        }
        
        // Insert the new connection at the correct position
        self.col_idx.insert(insert_pos, target_id);
        self.weights.insert(insert_pos, weight);
        self.delays.insert(insert_pos, delay);
        self.types.insert(insert_pos, connection_type);
        self.source_area_ids.insert(insert_pos, source_area_id);
        self.target_area_ids.insert(insert_pos, target_area_id);
        
        Ok(())
    }
    
    /// Propagates activation from source neurons to target neurons using SIMD
    #[cfg(target_feature = "avx2")]
    pub fn propagate_activations_simd(
        &self, 
        source_activations: &[f32], 
        target_buffer: &mut [f32]
    ) {
        use std::arch::x86_64::{__m256, _mm256_loadu_ps, _mm256_mul_ps, _mm256_add_ps, _mm256_storeu_ps, _mm256_set1_ps};
        
        // Process each source neuron
        for source_id in 0..self.neuron_count {
            let activation = source_activations[source_id];
            
            // Skip if not activated
            if activation <= 0.0 {
                continue;
            }
            
            // Get SIMD vector of the activation
            let activation_vector = unsafe { _mm256_set1_ps(activation) };
            
            // Get the range of connections for this neuron
            let start = self.row_ptr[source_id] as usize;
            let end = self.row_ptr[source_id + 1] as usize;
            
            // Process connections in blocks of 8 (SIMD width)
            let mut i = start;
            while i + 8 <= end {
                // Load 8 weights
                let weights = unsafe { _mm256_loadu_ps(&self.weights[i] as *const f32) };
                
                // Multiply by activation
                let contributions = unsafe { _mm256_mul_ps(weights, activation_vector) };
                
                // For each of the 8 connections, add the contribution to the target
                for j in 0..8 {
                    unsafe {
                        let target_id = self.col_idx[i + j] as usize;
                        // We could do better SIMD here but need to handle scatter properly
                        // For simplicity, just extract and add one by one
                        let contribution = *(&contributions as *const __m256 as *const f32).add(j);
                        target_buffer[target_id] += contribution;
                    }
                }
                
                i += 8;
            }
            
            // Handle remaining connections (less than 8)
            for i in i..end {
                let target_id = self.col_idx[i] as usize;
                let weight = self.weights[i];
                target_buffer[target_id] += activation * weight;
            }
        }
    }
    
    /// Scalar fallback for activation propagation
    #[cfg(not(target_feature = "avx2"))]
    pub fn propagate_activations_simd(
        &self, 
        source_activations: &[f32], 
        target_buffer: &mut [f32]
    ) {
        self.propagate_activations_scalar(source_activations, target_buffer);
    }
    
    /// Basic scalar implementation of activation propagation
    pub fn propagate_activations_scalar(
        &self, 
        source_activations: &[f32], 
        target_buffer: &mut [f32]
    ) {
        // For each source neuron
        for source_id in 0..self.neuron_count {
            let activation = source_activations[source_id];
            
            // Skip if not activated
            if activation <= 0.0 {
                continue;
            }
            
            // Get the range of connections for this neuron
            let start = self.row_ptr[source_id] as usize;
            let end = self.row_ptr[source_id + 1] as usize;
            
            // Process each connection
            for i in start..end {
                let target_id = self.col_idx[i] as usize;
                let weight = self.weights[i];
                target_buffer[target_id] += activation * weight;
            }
        }
    }
    
    /// Prepare data for GPU processing
    pub fn to_gpu_buffers(&self) -> (Vec<u32>, Vec<u32>, Vec<f32>) {
        // Return copies of the three main arrays for GPU
        (
            self.row_ptr.clone(),
            self.col_idx.clone(),
            self.weights.clone()
        )
    }
    
    /// Get all connections for a specific neuron
    pub fn get_connections_for_neuron(&self, neuron_id: u32) -> ConnectionsView {
        let start = self.row_ptr[neuron_id as usize] as usize;
        let end = self.row_ptr[neuron_id as usize + 1] as usize;
        
        ConnectionsView {
            connectome: self,
            start,
            end,
        }
    }
    
    /// Get the total number of connections in the connectome
    pub fn connection_count(&self) -> usize {
        self.col_idx.len()
    }
    
    /// Calculate Post-Synaptic Potential using the specified formula:
    /// FNPSP = [MPF * FNMP + !MPF * FNPSP] / [(!PUF * (FNSC-1)) + 1] * Synapse Conductance
    /// 
    /// Where:
    /// - FNMP: Firing Neuron Membrane Potential
    /// - FNSC: Firing Neuron Synapse Count
    /// - FNPSP: Firing Neuron Post Synaptic Potential
    /// - MPF: Membrane Potential Driven PSP Flag
    /// - PUF: Post Synaptic Potential Uniformity Flag
    /// - SC: Synapse Conductance
    pub fn calculate_psp(
        &self,
        firing_neuron_mp: f32,
        firing_neuron_psp: f32,
        synapse_count: u32,
        synapse_conductance: f32,
        mpf: bool,   // Membrane Potential Driven PSP Flag
        puf: bool,   // PSP Uniformity Flag
    ) -> f32 {
        // FNPSP = [MPF * FNMP + !MPF * FNPSP] / [(!PUF * (FNSC-1)) + 1] * Synapse Conductance
        
        // Calculate the numerator: either use membrane potential or existing PSP
        let numerator = if mpf { firing_neuron_mp } else { firing_neuron_psp };
        
        // Calculate the denominator: either normalize by synapse count or use 1
        let denominator = if !puf && synapse_count > 0 { 
            (synapse_count - 1) as f32 + 1.0 
        } else { 
            1.0 
        };
        
        // Calculate the final PSP
        (numerator / denominator) * synapse_conductance
    }
    
    /// Propagate activations with PSP calculation to the fire queue
    pub fn propagate_with_psp(
        &self,
        source_neurons: &[u32],
        gna: &GlobalNeuronArray,
        fire_queue: &mut FireQueue,
        mpf: bool,
        puf: bool,
    ) {
        // For each firing neuron
        for &source_id in source_neurons {
            let source_id = source_id as usize;
            if source_id >= self.neuron_count {
                continue;
            }
            
            // Get outgoing connections
            let start = self.row_ptr[source_id] as usize;
            let end = self.row_ptr[source_id + 1] as usize;
            let synapse_count = end - start;
            
            // Skip if no connections
            if synapse_count == 0 {
                continue;
            }
            
            // Get firing neuron membrane potential
            let firing_neuron_mp = gna.membrane_potentials[source_id];
            
            // Default PSP value if not using previous PSP (would need tracking in real impl)
            let firing_neuron_psp = 1.0; 
            
            // Process each outgoing connection from this firing neuron
            for i in start..end {
                let target_id = self.col_idx[i] as usize;
                if target_id >= gna.neuron_count {
                    continue;
                }
                
                let synapse_conductance = self.weights[i];
                
                // Calculate PSP contribution
                let psp = self.calculate_psp(
                    firing_neuron_mp,
                    firing_neuron_psp,
                    synapse_count as u32,
                    synapse_conductance,
                    mpf,
                    puf
                );
                
                // Update target neuron's membrane potential
                let current_mp = gna.membrane_potentials[target_id];
                let updated_mp = current_mp + psp;
                
                // Get consecutive fire count (in production, would be stored)
                let consecutive_fire_count = gna.get_consecutive_fire_count(target_id);
                
                // Add to fire queue
                fire_queue.add(
                    target_id as u32,
                    updated_mp,
                    gna.thresholds[target_id],
                    consecutive_fire_count,
                    gna.refractory_counters[target_id]
                );
            }
        }
    }
    
    /// SIMD-optimized propagation with PSP calculation (when AVX2 is available)
    #[cfg(target_feature = "avx2")]
    pub fn propagate_with_psp_simd(
        &self,
        source_neurons: &[u32],
        gna: &GlobalNeuronArray,
        fire_queue: &mut FireQueue,
        mpf: bool,
        puf: bool,
    ) {
        use std::arch::x86_64::{__m256, _mm256_set1_ps, _mm256_loadu_ps, _mm256_mul_ps, _mm256_div_ps};
        
        // For each firing neuron
        for &source_id in source_neurons {
            let source_id = source_id as usize;
            if source_id >= self.neuron_count {
                continue;
            }
            
            // Get outgoing connections
            let start = self.row_ptr[source_id] as usize;
            let end = self.row_ptr[source_id + 1] as usize;
            let synapse_count = end - start;
            
            // Skip if no connections
            if synapse_count == 0 {
                continue;
            }
            
            // Get firing neuron membrane potential
            let firing_neuron_mp = gna.membrane_potentials[source_id];
            
            // Default PSP value if not using previous PSP
            let firing_neuron_psp = 1.0;
            
            // Choose numerator based on MPF flag
            let numerator = if mpf { firing_neuron_mp } else { firing_neuron_psp };
            
            // Calculate denominator based on PUF flag
            let denominator = if !puf && synapse_count > 0 { 
                (synapse_count - 1) as f32 + 1.0 
            } else { 
                1.0 
            };
            
            // Set up SIMD vectors for the calculation
            let numerator_vec = unsafe { _mm256_set1_ps(numerator) };
            let denominator_vec = unsafe { _mm256_set1_ps(denominator) };
            
            // Process connections in blocks of 8 (SIMD width)
            let mut i = start;
            while i + 8 <= end {
                unsafe {
                    // Load 8 synapse conductance values
                    let conductance_vec = _mm256_loadu_ps(&self.weights[i] as *const f32);
                    
                    // Calculate PSP (numerator / denominator) * conductance
                    let div_result = _mm256_div_ps(numerator_vec, denominator_vec);
                    let psp_vec = _mm256_mul_ps(div_result, conductance_vec);
                    
                    // Now apply to each target neuron (can't easily vectorize this part
                    // due to scattered memory access patterns)
                    for j in 0..8 {
                        let idx = i + j;
                        let target_id = self.col_idx[idx] as usize;
                        if target_id >= gna.neuron_count {
                            continue;
                        }
                        
                        // Extract PSP value from vector
                        let psp = *(&psp_vec as *const __m256 as *const f32).add(j);
                        
                        // Update target neuron's membrane potential
                        let current_mp = gna.membrane_potentials[target_id];
                        let updated_mp = current_mp + psp;
                        
                        // Get consecutive fire count 
                        let consecutive_fire_count = gna.get_consecutive_fire_count(target_id);
                        
                        // Add to fire queue
                        fire_queue.add(
                            target_id as u32,
                            updated_mp,
                            gna.thresholds[target_id],
                            consecutive_fire_count,
                            gna.refractory_counters[target_id]
                        );
                    }
                }
                
                i += 8;
            }
            
            // Handle remaining connections
            for idx in i..end {
                let target_id = self.col_idx[idx] as usize;
                if target_id >= gna.neuron_count {
                    continue;
                }
                
                let synapse_conductance = self.weights[idx];
                let psp = (numerator / denominator) * synapse_conductance;
                
                // Update target neuron's membrane potential
                let current_mp = gna.membrane_potentials[target_id];
                let updated_mp = current_mp + psp;
                
                // Get consecutive fire count
                let consecutive_fire_count = gna.get_consecutive_fire_count(target_id);
                
                // Add to fire queue
                fire_queue.add(
                    target_id as u32,
                    updated_mp,
                    gna.thresholds[target_id],
                    consecutive_fire_count,
                    gna.refractory_counters[target_id]
                );
            }
        }
    }
}

/// View of connections for a specific neuron
pub struct ConnectionsView<'a> {
    connectome: &'a Connectome,
    start: usize,
    end: usize,
}

impl<'a> ConnectionsView<'a> {
    /// Iterator over the connections in this view
    pub fn iter(&self) -> impl Iterator<Item = (u32, f32, u8, u8, u32, u32)> + '_ {
        (self.start..self.end).map(move |i| {
            (
                self.connectome.col_idx[i],
                self.connectome.weights[i],
                self.connectome.delays[i],
                self.connectome.types[i],
                self.connectome.source_area_ids[i],
                self.connectome.target_area_ids[i],
            )
        })
    }
    
    /// Get the number of connections in this view
    pub fn len(&self) -> usize {
        self.end - self.start
    }
    
    /// Check if this view is empty
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// WebGPU-compatible representation of the connectome for use in compute shaders
#[repr(C)]
pub struct GpuConnectomeData {
    pub row_ptr_offset: u32,
    pub col_idx_offset: u32,
    pub weights_offset: u32,
    pub neuron_count: u32,
} 