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
use std::sync::atomic::{AtomicUsize, Ordering};

/// Fast PCG hash for deterministic pseudo-random number generation
/// Based on PCG family of PRNGs: https://www.pcg-random.org/
/// 
/// This provides fast, high-quality pseudo-random numbers suitable for
/// excitability checks without requiring a mutable RNG state.
#[inline(always)]
fn pcg_hash(input: u32) -> u32 {
    let mut state = input.wrapping_mul(747796405).wrapping_add(2891336453);
    let word = ((state >> ((state >> 28) + 4)) ^ state).wrapping_mul(277803737);
    (word >> 22) ^ word
}

/// Convert PCG hash to floating point in range [0, 1)
#[inline(always)]
fn pcg_hash_to_float(input: u32) -> f32 {
    (pcg_hash(input) as f32) / 4294967296.0
}

/// Generate pseudo-random value for excitability check
/// 
/// CRITICAL: Combines neuron_id AND burst_count to ensure different random values each burst.
/// This ensures probabilistic firing works correctly (e.g., 20% excitability = 20% chance per burst).
#[inline(always)]
fn excitability_random(neuron_id: u32, burst_count: u64) -> f32 {
    // Combine neuron_id and burst_count to get different random values each burst
    // Use wrapping operations to prevent overflow
    let seed = neuron_id.wrapping_mul(2654435761).wrapping_add((burst_count as u32).wrapping_mul(1597334677));
    pcg_hash_to_float(seed)
}

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
    burst_count: u64,
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
            if let Some(neuron) = process_single_neuron(neuron_id, candidate_potential, neuron_array, burst_count) {
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
    burst_count: u64,
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
    
    // 1b. Handle snooze period (rest after consecutive fires)
    // Snooze blocks firing even if neuron is above threshold
    if neuron_array.snooze_countdowns[idx] > 0 {
        neuron_array.snooze_countdowns[idx] -= 1;
        
        // Apply leak during snooze to prevent potential buildup
        let leak_coefficient = neuron_array.leak_coefficients[idx];
        if leak_coefficient > 0.0 {
            let resting_potential = neuron_array.resting_potentials[idx];
            let current_potential = neuron_array.membrane_potentials[idx];
            let leaked_potential = current_potential + leak_coefficient * (resting_potential - current_potential);
            neuron_array.membrane_potentials[idx] = leaked_potential;
        }
        
        return None;
    }
    
    // 2. Add candidate potential (matches Python: add BEFORE checking threshold)
    let old_potential = neuron_array.membrane_potentials[idx];
    let current_potential = old_potential + candidate_potential;
    neuron_array.membrane_potentials[idx] = current_potential;
    
    // 3. Check threshold (matches Python: "Check firing conditions BEFORE decay")
    let threshold = neuron_array.thresholds[idx];
    if current_potential >= threshold {
        // 5. Check consecutive fire limit (matches Python SIMD implementation)
        // Skip constraint if consecutive_fire_limit is 0 (unlimited firing)
        let consecutive_fire_limit = neuron_array.consecutive_fire_limits[idx];
        let consecutive_fire_count = neuron_array.consecutive_fire_counts[idx];
        
        let consecutive_fire_constraint = if consecutive_fire_limit > 0 {
            consecutive_fire_count < consecutive_fire_limit
        } else {
            true  // No limit (limit == 0 means unlimited)
        };
        
        if !consecutive_fire_constraint {
            // Neuron exceeded consecutive fire limit - prevent firing
            // CRITICAL: Reset count to 0 (matches Python SIMD: all non-firing neurons get count reset)
            // This allows the neuron to fire again after being blocked for one burst
            neuron_array.consecutive_fire_counts[idx] = 0;
            
            // Apply leak to prevent potential from growing unbounded
            let leak_coefficient = neuron_array.leak_coefficients[idx];
            if leak_coefficient > 0.0 {
                let resting_potential = neuron_array.resting_potentials[idx];
                let leaked_potential = current_potential + leak_coefficient * (resting_potential - current_potential);
                neuron_array.membrane_potentials[idx] = leaked_potential;
            }
            
            return None;
        }
        
        // 6. Apply probabilistic excitability
        let excitability = neuron_array.excitabilities[idx];
        
        // Fast path: excitability >= 0.999 means always fire (matches Python)
        let should_fire = if excitability >= 0.999 {
            true
        } else if excitability <= 0.0 {
            false
        } else {
            // Probabilistic firing based on excitability (matches Python RNG logic)
            // CRITICAL: Use excitability_random() which combines neuron_id AND burst_count
            // This ensures different random values each burst (e.g., 20% excitability = 20% chance per burst)
            let random_val = excitability_random(neuron_id.0, burst_count);
            random_val < excitability
        };
        
        if should_fire {
            // Increment consecutive fire count (matches Python SIMD implementation)
            if consecutive_fire_limit > 0 {
                neuron_array.consecutive_fire_counts[idx] += 1;
                
                // Check if neuron hit consecutive fire limit → trigger snooze
                let new_count = neuron_array.consecutive_fire_counts[idx];
                if new_count >= consecutive_fire_limit {
                    let snooze_period = neuron_array.snooze_periods[idx];
                    if snooze_period > 0 {
                        // Reset consecutive count and enter snooze
                        neuron_array.consecutive_fire_counts[idx] = 0;
                        neuron_array.snooze_countdowns[idx] = snooze_period;
                    }
                }
            }
            
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
                membrane_potential: current_potential,
                cortical_area: CorticalAreaId(neuron_array.cortical_areas[idx]),
                x,
                y,
                z,
            });
        }
    }
    
    // Neuron did not fire - apply leak and reset consecutive fire count
    // (matches Python: "Apply membrane decay to remaining neurons (leak behavior)")
    
    // Reset consecutive fire count (matches Python SIMD implementation)
    let consecutive_fire_limit = neuron_array.consecutive_fire_limits[idx];
    if consecutive_fire_limit > 0 {
        neuron_array.consecutive_fire_counts[idx] = 0;
    }
    
    // Apply LIF leak toward resting potential (only for non-firing neurons)
    // Formula: V_new = V_current + leak_coeff * (V_rest - V_current)
    // This naturally pulls the potential toward resting potential
    let leak_coefficient = neuron_array.leak_coefficients[idx];
    if leak_coefficient > 0.0 {
        let resting_potential = neuron_array.resting_potentials[idx];
        let leaked_potential = current_potential + leak_coefficient * (resting_potential - current_potential);
        neuron_array.membrane_potentials[idx] = leaked_potential;
    }
    // If leak_coefficient == 0, potential remains unchanged (e.g., power neurons)
    
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
    burst_count: u64,
) -> Result<DynamicsResult> {
    // TODO: Implement SIMD version using std::simd or explicit SIMD intrinsics
    // For now, fall back to scalar implementation
    process_neural_dynamics(fcl, neuron_array, burst_count)
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
            0.0,   // leak_coefficient
            0.0,   // resting_potential
            0,     // neuron_type
            5,     // refractory_period
            1.0,   // excitability
            0,     // consecutive_fire_limit
            0,     // snooze_period
            1,     // cortical_area
            0, 0, 0
        ).unwrap();
        
        // Create FCL with enough potential to fire
        let mut fcl = FireCandidateList::new();
        fcl.add_candidate(id, 1.5);
        
        // Process dynamics
        let result = process_neural_dynamics(&fcl, &mut neurons, 0).unwrap();
        
        assert_eq!(result.neurons_fired, 1);
        assert_eq!(result.fire_queue.len(), 1);
        assert_eq!(neurons.get_potential(id), 0.0);  // Reset after firing
        assert_eq!(neurons.refractory_countdowns[0], 5);  // Refractory set
    }

    #[test]
    fn test_neuron_does_not_fire_below_threshold() {
        let mut neurons = NeuronArray::new(10);
        
        let id = neurons.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 0, 1, 0, 0, 0).unwrap();
        
        let mut fcl = FireCandidateList::new();
        fcl.add_candidate(id, 0.5);  // Below threshold
        
        let result = process_neural_dynamics(&fcl, &mut neurons, 0).unwrap();
        
        assert_eq!(result.neurons_fired, 0);
        assert_eq!(result.fire_queue.len(), 0);
        assert!(neurons.get_potential(id) > 0.0);  // Potential accumulated
    }

    #[test]
    fn test_refractory_period_blocks_firing() {
        let mut neurons = NeuronArray::new(10);
        
        let id = neurons.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, 1, 0, 0, 0).unwrap();
        
        // Set refractory countdown
        neurons.refractory_countdowns[0] = 3;
        
        let mut fcl = FireCandidateList::new();
        fcl.add_candidate(id, 2.0);  // Well above threshold
        
        let result = process_neural_dynamics(&fcl, &mut neurons, 0).unwrap();
        
        assert_eq!(result.neurons_fired, 0);
        assert_eq!(neurons.refractory_countdowns[0], 2);  // Decremented
    }

    #[test]
    fn test_leak_decay() {
        let mut neurons = NeuronArray::new(10);
        
        let id = neurons.add_neuron(
            10.0,  // High threshold (won't fire)
            0.5,   // leak_coefficient (50% leak toward resting)
            0.0,   // resting_potential
            0,     // neuron_type
            0,     // refractory_period
            1.0,   // excitability
            0,     // consecutive_fire_limit
            0,     // snooze_period
            1,     // cortical_area
            0, 0, 0
        ).unwrap();
        
        // Set initial potential
        neurons.set_potential(id, 1.0);
        
        // Add small candidate potential
        let mut fcl = FireCandidateList::new();
        fcl.add_candidate(id, 0.1);
        
        process_neural_dynamics(&fcl, &mut neurons, 0).unwrap();
        
        // Expected LIF: (1.0 + 0.1) + 0.5 * (0.0 - 1.1) = 1.1 - 0.55 = 0.55
        assert!((neurons.get_potential(id) - 0.55).abs() < 0.001);
    }

    #[test]
    fn test_multiple_neurons_firing() {
        let mut neurons = NeuronArray::new(100);
        
        // Add 10 neurons
        let mut ids = Vec::new();
        for i in 0..10 {
            let id = neurons.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 0, 1, i, 0, 0).unwrap();
            ids.push(id);
        }
        
        // Create FCL with all above threshold
        let mut fcl = FireCandidateList::new();
        for id in &ids {
            fcl.add_candidate(*id, 1.5);
        }
        
        let result = process_neural_dynamics(&fcl, &mut neurons, 0).unwrap();
        
        assert_eq!(result.neurons_processed, 10);
        assert_eq!(result.neurons_fired, 10);
    }
}
