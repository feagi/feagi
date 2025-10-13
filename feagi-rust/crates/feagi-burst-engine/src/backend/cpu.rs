/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # CPU Backend
//!
//! SIMD-optimized CPU backend using existing neural_dynamics and synaptic_propagation modules.
//! This wraps the current high-performance Rust implementation.

use super::ComputeBackend;
use crate::neural_dynamics;
use feagi_types::*;

/// CPU backend with SIMD optimization (current implementation)
pub struct CPUBackend {
    /// Backend name for logging
    name: String,
}

impl CPUBackend {
    /// Create a new CPU backend
    pub fn new() -> Self {
        Self {
            name: "CPU (SIMD)".to_string(),
        }
    }
}

impl Default for CPUBackend {
    fn default() -> Self {
        Self::new()
    }
}

impl ComputeBackend for CPUBackend {
    fn backend_name(&self) -> &str {
        &self.name
    }
    
    fn process_synaptic_propagation(
        &mut self,
        fired_neurons: &[u32],
        synapse_array: &SynapseArray,
        fcl: &mut FireCandidateList,
    ) -> Result<usize> {
        // FCL-aware: Accumulate synaptic contributions into FCL
        let mut synapse_count = 0;
        
        for &source_id in fired_neurons {
            if let Some(synapse_indices) = synapse_array.source_index.get(&source_id) {
                for &syn_idx in synapse_indices {
                    if !synapse_array.valid_mask[syn_idx] {
                        continue;
                    }
                    
                    let target_id = synapse_array.target_neurons[syn_idx];
                    let weight = synapse_array.weights[syn_idx] as f32 / 255.0; // Normalize to [0,1]
                    let conductance = synapse_array.conductances[syn_idx] as f32 / 255.0;
                    let synapse_type = synapse_array.types[syn_idx];
                    
                    // Calculate synaptic contribution
                    let sign = if synapse_type == 0 { 1.0 } else { -1.0 }; // 0=excitatory, 1=inhibitory
                    let contribution = sign * weight * conductance * 10.0; // Scale factor
                    
                    // Accumulate into FCL
                    fcl.add_candidate(NeuronId(target_id), contribution);
                    synapse_count += 1;
                }
            }
        }
        
        Ok(synapse_count)
    }
    
    fn process_neural_dynamics(
        &mut self,
        fcl: &FireCandidateList,
        neuron_array: &mut NeuronArray,
        burst_count: u64,
    ) -> Result<(Vec<u32>, usize, usize)> {
        // FCL-aware: Process only FCL neurons (existing neural_dynamics already supports this!)
        let result = neural_dynamics::process_neural_dynamics(
            fcl,
            neuron_array,
            burst_count,
        )?;
        
        // Extract neuron IDs from fire queue
        let fired_neurons: Vec<u32> = result.fire_queue.get_all_neuron_ids()
            .iter()
            .map(|n| n.0)
            .collect();
        
        Ok((
            fired_neurons,
            result.neurons_processed,
            result.neurons_in_refractory,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_cpu_backend_creation() {
        let backend = CPUBackend::new();
        assert_eq!(backend.backend_name(), "CPU (SIMD)");
    }
    
    #[test]
    fn test_cpu_backend_synaptic_propagation() {
        let mut backend = CPUBackend::new();
        
        // Create minimal test data
        let fired_neurons = vec![0, 1];
        let synapse_array = SynapseArray::new(100);
        let mut fcl = FireCandidateList::new();
        
        // Should not panic
        let result = backend.process_synaptic_propagation(
            &fired_neurons,
            &synapse_array,
            &mut fcl,
        );
        
        assert!(result.is_ok());
    }
}

