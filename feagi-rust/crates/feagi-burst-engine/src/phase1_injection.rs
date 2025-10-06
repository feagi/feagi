/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # Phase 1: Injection
//!
//! Injects potential into the Fire Candidate List (FCL) from:
//! 1. Power neurons (continuous input)
//! 2. Synaptic propagation (from previous burst)
//! 3. Sensory input (future feature)

use feagi_types::*;
use crate::synaptic_propagation::SynapticPropagationEngine;

/// Phase 1 result
#[derive(Debug)]
pub struct InjectionResult {
    pub power_injections: usize,
    pub synaptic_injections: usize,
    pub sensory_injections: usize,
}

/// Phase 1: Injection
///
/// Builds the Fire Candidate List (FCL) from all input sources
pub fn phase1_injection(
    fcl: &mut FireCandidateList,
    neuron_array: &NeuronArray,
    propagation_engine: &mut SynapticPropagationEngine,
    previous_fire_queue: &FireQueue,
    power_neurons: &[NeuronId],
    power_amount: f32,
) -> Result<InjectionResult> {
    // Clear FCL from previous burst
    fcl.clear();
    
    let mut power_count = 0;
    let mut synaptic_count = 0;
    
    // 1. Power Injection - continuous input to specific neurons
    for &neuron_id in power_neurons {
        let idx = neuron_id.0 as usize;
        if idx < neuron_array.count {
            fcl.add_candidate(neuron_id, power_amount);
            power_count += 1;
        }
    }
    
    // 2. Synaptic Propagation - compute from previous burst's firing
    if !previous_fire_queue.is_empty() {
        let fired_ids = previous_fire_queue.get_all_neuron_ids();
        let fired_array: Vec<u32> = fired_ids.iter().map(|id| id.0).collect();
        
        // Call synaptic propagation engine - NOTE: This stub version doesn't use synapses
        // Real implementation will be in the npu.rs version that has access to synapses
        // For standalone testing, just skip propagation
        synaptic_count = 0;
    }
    
    // 3. Sensory Injection (future feature - placeholder)
    let sensory_count = 0;
    
    Ok(InjectionResult {
        power_injections: power_count,
        synaptic_injections: synaptic_count,
        sensory_injections: sensory_count,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_power_injection() {
        let mut fcl = FireCandidateList::new();
        let mut neurons = NeuronArray::new(100);
        
        // Add neurons first!
        for i in 0..5 {
            neurons.add_neuron(1.0, 0.1, 5, 1.0, 1, i, 0, 0).unwrap();
        }
        
        let mut propagation_engine = SynapticPropagationEngine::new();
        let prev_fq = FireQueue::new();
        
        let power_neurons = vec![NeuronId(1), NeuronId(2), NeuronId(3)];
        
        let result = phase1_injection(
            &mut fcl,
            &neurons,
            &mut propagation_engine,
            &prev_fq,
            &power_neurons,
            1.0,
        ).unwrap();
        
        assert_eq!(result.power_injections, 3);
        assert_eq!(fcl.len(), 3);
        assert_eq!(fcl.get_potential(NeuronId(1)), 1.0);
    }

    #[test]
    fn test_synaptic_propagation_injection() {
        let mut fcl = FireCandidateList::new();
        
        // Create neurons
        let mut neurons = NeuronArray::new(100);
        for i in 1..=10 {
            neurons.add_neuron(1.0, 0.1, 5, 1.0, 1, i, 0, 0).unwrap();
        }
        
        // NOTE: Testing synaptic propagation requires the full NPU context
        // This test is a stub - real propagation testing is done in npu.rs tests
        let mut propagation_engine = SynapticPropagationEngine::new();
        
        // NOTE: Synaptic propagation is not tested here
        // See npu.rs for full integration tests
        let prev_fq = FireQueue::new();
        
        // Run injection (no power, no previous firing)
        let result = phase1_injection(
            &mut fcl,
            &neurons,
            &mut propagation_engine,
            &prev_fq,
            &[],
            1.0,
        ).unwrap();
        
        assert_eq!(result.power_injections, 0);
        assert_eq!(result.synaptic_injections, 0);
    }
}
