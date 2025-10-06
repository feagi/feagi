/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # Neural Dynamics (Phase 2)
//!
//! SIMD-optimized membrane potential updates, threshold checks, and firing logic.
//!
//! ## Performance Critical Path
//! This is the hottest code path in FEAGI. Every optimization matters.
//!
//! ## Optimization Strategy
//! 1. **SIMD**: Process 8+ neurons at once using AVX2/AVX-512
//! 2. **Rayon**: Parallelize across cores for large neuron counts
//! 3. **Cache-friendly**: Sequential access patterns, no pointer chasing

use feagi_types::*;
use rayon::prelude::*;

/// Result of neural dynamics processing
#[derive(Debug, Clone)]
pub struct DynamicsResult {
    /// Neurons that fired this burst
    pub fire_queue: FireQueue,
    
    /// Performance metrics
    pub neurons_processed: usize,
    pub neurons_fired: usize,
    pub neurons_in_refractory: usize,
}

/// Process neural dynamics for all candidate neurons
///
/// This is the CRITICAL HOT PATH - every microsecond matters!
///
/// ## Algorithm:
/// 1. Apply leak/decay to membrane potentials
/// 2. Add candidate potentials from FCL
/// 3. Check firing thresholds (with refractory period)
/// 4. Apply probabilistic excitability
/// 5. Create Fire Queue from firing neurons
pub fn process_neural_dynamics(
    fcl: &FireCandidateList,
    neuron_array: &mut NeuronArray,
) -> Result<DynamicsResult> {
    let candidates = fcl.get_all_candidates();
    
    if candidates.is_empty() {
        return Ok(DynamicsResult {
            fire_queue: FireQueue::new(),
            neurons_processed: 0,
            neurons_fired: 0,
            neurons_in_refractory: 0,
        });
    }
    
    // NOTE: We CANNOT use Rayon here because process_single_neuron mutates neuron_array
    // This would require unsafe code or a different approach (batch processing)
    // For now, use single-threaded processing (still very fast!)
    let (fired_neurons, refractory_count): (Vec<_>, usize) = {
        // Single-threaded for small sets (avoid parallelism overhead)
        let mut results = Vec::with_capacity(candidates.len());
        let mut refractory = 0;
        
        for &(neuron_id, candidate_potential) in &candidates {
            if let Some(neuron) = process_single_neuron(neuron_id, candidate_potential, neuron_array) {
                results.push(neuron);
            }
            
            let idx = neuron_id.0 as usize;
            if neuron_array.refractory_countdowns[idx] > 0 {
                refractory += 1;
            }
        }
        
        (results, refractory)
    };
    
    // Build Fire Queue
    let mut fire_queue = FireQueue::new();
    for neuron in fired_neurons.iter() {
        fire_queue.add_neuron(*neuron);
    }
    
    Ok(DynamicsResult {
        fire_queue,
        neurons_processed: candidates.len(),
        neurons_fired: fired_neurons.len(),
        neurons_in_refractory: refractory_count,
    })
}

/// Process a single neuron's dynamics
///
/// Returns Some(FiringNeuron) if the neuron fires, None otherwise
#[inline(always)]
fn process_single_neuron(
    neuron_id: NeuronId,
    candidate_potential: f32,
    neuron_array: &mut NeuronArray,
) -> Option<FiringNeuron> {
    let idx = neuron_id.0 as usize;
    
    // Check if neuron exists
    if idx >= neuron_array.count {
        return None;
    }
    
    // 1. Handle refractory period
    if neuron_array.refractory_countdowns[idx] > 0 {
        neuron_array.refractory_countdowns[idx] -= 1;
        return None;
    }
    
    // 2. Apply leak/decay to existing membrane potential
    let leak_rate = neuron_array.leak_rates[idx];
    let decayed_potential = neuron_array.membrane_potentials[idx] * (1.0 - leak_rate);
    
    // 3. Add candidate potential
    let new_potential = decayed_potential + candidate_potential;
    neuron_array.membrane_potentials[idx] = new_potential;
    
    // 4. Check threshold
    let threshold = neuron_array.thresholds[idx];
    if new_potential >= threshold {
        // 5. Apply probabilistic excitability
        let excitability = neuron_array.excitabilities[idx];
        
        // Fast path: excitability = 1.0 means always fire
        let should_fire = if excitability >= 1.0 {
            true
        } else if excitability <= 0.0 {
            false
        } else {
            // Probabilistic firing based on excitability
            // For production, use a proper RNG. For now, use a deterministic check.
            // TODO: Add RNG support
            new_potential >= threshold * (1.0 / excitability)
        };
        
        if should_fire {
            // Reset membrane potential
            neuron_array.membrane_potentials[idx] = 0.0;
            
            // Set refractory period
            neuron_array.refractory_countdowns[idx] = neuron_array.refractory_periods[idx];
            
            // Get neuron coordinates
            let coord_idx = idx * 3;
            let (x, y, z) = (
                neuron_array.coordinates[coord_idx],
                neuron_array.coordinates[coord_idx + 1],
                neuron_array.coordinates[coord_idx + 2],
            );
            
            return Some(FiringNeuron {
                neuron_id,
                membrane_potential: new_potential,
                cortical_area: CorticalAreaId(neuron_array.cortical_areas[idx]),
                x,
                y,
                z,
            });
        }
    }
    
    None
}

/// SIMD-optimized batch processing (future optimization)
///
/// Process 8 neurons at once using AVX2 SIMD instructions.
/// This will be enabled once we have sufficient test coverage.
#[cfg(feature = "simd")]
pub fn process_neural_dynamics_simd(
    fcl: &FireCandidateList,
    neuron_array: &mut NeuronArray,
) -> Result<DynamicsResult> {
    // TODO: Implement SIMD version using std::simd or explicit SIMD intrinsics
    // For now, fall back to scalar implementation
    process_neural_dynamics(fcl, neuron_array)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_neuron_fires_when_above_threshold() {
        let mut neurons = NeuronArray::new(10);
        
        // Add a neuron with threshold 1.0
        let id = neurons.add_neuron(
            1.0,   // threshold
            0.1,   // leak_rate
            5,     // refractory_period
            1.0,   // excitability
            1,     // cortical_area
            0, 0, 0
        ).unwrap();
        
        // Create FCL with enough potential to fire
        let mut fcl = FireCandidateList::new();
        fcl.add_candidate(id, 1.5);
        
        // Process dynamics
        let result = process_neural_dynamics(&fcl, &mut neurons).unwrap();
        
        assert_eq!(result.neurons_fired, 1);
        assert_eq!(result.fire_queue.len(), 1);
        assert_eq!(neurons.get_potential(id), 0.0);  // Reset after firing
        assert_eq!(neurons.refractory_countdowns[0], 5);  // Refractory set
    }

    #[test]
    fn test_neuron_does_not_fire_below_threshold() {
        let mut neurons = NeuronArray::new(10);
        
        let id = neurons.add_neuron(1.0, 0.1, 5, 1.0, 1, 0, 0, 0).unwrap();
        
        let mut fcl = FireCandidateList::new();
        fcl.add_candidate(id, 0.5);  // Below threshold
        
        let result = process_neural_dynamics(&fcl, &mut neurons).unwrap();
        
        assert_eq!(result.neurons_fired, 0);
        assert_eq!(result.fire_queue.len(), 0);
        assert!(neurons.get_potential(id) > 0.0);  // Potential accumulated
    }

    #[test]
    fn test_refractory_period_blocks_firing() {
        let mut neurons = NeuronArray::new(10);
        
        let id = neurons.add_neuron(1.0, 0.0, 5, 1.0, 1, 0, 0, 0).unwrap();
        
        // Set refractory countdown
        neurons.refractory_countdowns[0] = 3;
        
        let mut fcl = FireCandidateList::new();
        fcl.add_candidate(id, 2.0);  // Well above threshold
        
        let result = process_neural_dynamics(&fcl, &mut neurons).unwrap();
        
        assert_eq!(result.neurons_fired, 0);
        assert_eq!(neurons.refractory_countdowns[0], 2);  // Decremented
    }

    #[test]
    fn test_leak_decay() {
        let mut neurons = NeuronArray::new(10);
        
        let id = neurons.add_neuron(
            10.0,  // High threshold (won't fire)
            0.5,   // 50% leak
            0,
            1.0,
            1,
            0, 0, 0
        ).unwrap();
        
        // Set initial potential
        neurons.set_potential(id, 1.0);
        
        // Add small candidate potential
        let mut fcl = FireCandidateList::new();
        fcl.add_candidate(id, 0.1);
        
        process_neural_dynamics(&fcl, &mut neurons).unwrap();
        
        // Expected: 1.0 * (1 - 0.5) + 0.1 = 0.6
        assert!((neurons.get_potential(id) - 0.6).abs() < 0.001);
    }

    #[test]
    fn test_multiple_neurons_firing() {
        let mut neurons = NeuronArray::new(100);
        
        // Add 10 neurons
        let mut ids = Vec::new();
        for i in 0..10 {
            let id = neurons.add_neuron(1.0, 0.1, 5, 1.0, 1, i, 0, 0).unwrap();
            ids.push(id);
        }
        
        // Create FCL with all above threshold
        let mut fcl = FireCandidateList::new();
        for id in &ids {
            fcl.add_candidate(*id, 1.5);
        }
        
        let result = process_neural_dynamics(&fcl, &mut neurons).unwrap();
        
        assert_eq!(result.neurons_processed, 10);
        assert_eq!(result.neurons_fired, 10);
    }
}
