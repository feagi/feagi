/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! STDP (Spike-Timing-Dependent Plasticity) computation
//!
//! Pure functions for computing synaptic weight changes based on spike timing.
//! RTOS-friendly with deterministic execution and no dynamic allocation in hot paths.

use std::collections::HashMap;
use feagi_types::*;

/// STDP configuration parameters
#[derive(Debug, Clone, Copy)]
pub struct STDPConfig {
    /// Number of timesteps to look back for spike history
    pub lookback_steps: u32,
    
    /// Pre-synaptic time constant (τ_pre)
    pub tau_pre: f32,
    
    /// Post-synaptic time constant (τ_post)
    pub tau_post: f32,
    
    /// Potentiation learning rate (A+)
    pub a_plus: f32,
    
    /// Depression learning rate (A-)
    pub a_minus: f32,
    
    /// Maximum spike pairs to consider per synapse
    pub max_pairs_per_synapse: usize,
}

impl Default for STDPConfig {
    fn default() -> Self {
        Self {
            lookback_steps: 20,
            tau_pre: 20.0,
            tau_post: 20.0,
            a_plus: 0.01,
            a_minus: 0.012,
            max_pairs_per_synapse: 8,
        }
    }
}

/// Compute activity-based STDP factors for synapses
///
/// This is a simplified Hebbian approximation that uses presence/absence
/// of spikes in the lookback window rather than precise timing.
pub fn compute_activity_factors(
    syn_source_ids: &[NeuronId],
    syn_target_ids: &[NeuronId],
    source_history: &[NeuronId],
    target_history: &[NeuronId],
) -> (Vec<f32>, Vec<f32>) {
    let count = syn_source_ids.len();
    if count == 0 {
        return (Vec::new(), Vec::new());
    }
    
    // Build fast lookup sets for pre and post activity
    let pre_active: std::collections::HashSet<u32> = 
        source_history.iter().map(|n| n.0).collect();
    let post_active: std::collections::HashSet<u32> = 
        target_history.iter().map(|n| n.0).collect();
    
    // Compute activity factors
    let mut pre_activity = Vec::with_capacity(count);
    let mut post_activity = Vec::with_capacity(count);
    
    for (src, tgt) in syn_source_ids.iter().zip(syn_target_ids.iter()) {
        let pre = if pre_active.contains(&src.0) { 1.0 } else { 0.0 };
        let post = if post_active.contains(&tgt.0) { 1.0 } else { 0.0 };
        pre_activity.push(pre);
        post_activity.push(post);
    }
    
    (pre_activity, post_activity)
}

/// Compute timing-based STDP factors for synapses
///
/// Uses exponential STDP rule:
/// - Δw = A+ * exp(-Δt/τ_pre) if pre before post (potentiation)
/// - Δw = -A- * exp(Δt/τ_post) if post before pre (depression)
pub fn compute_timing_factors(
    syn_source_ids: &[NeuronId],
    syn_target_ids: &[NeuronId],
    source_history: &[(u64, NeuronId)],  // (timestep, neuron_id)
    target_history: &[(u64, NeuronId)],  // (timestep, neuron_id)
    config: &STDPConfig,
) -> Vec<f32> {
    let count = syn_source_ids.len();
    if count == 0 {
        return Vec::new();
    }
    
    // Build last-spike maps (neuron_id -> timestep)
    let mut pre_last: HashMap<u32, u64> = HashMap::new();
    let mut post_last: HashMap<u32, u64> = HashMap::new();
    
    for &(ts, nid) in source_history.iter() {
        pre_last.entry(nid.0).or_insert(ts);
    }
    
    for &(ts, nid) in target_history.iter() {
        post_last.entry(nid.0).or_insert(ts);
    }
    
    // Compute timing factors
    let mut factors = vec![0.0f32; count];
    
    for (i, (src, tgt)) in syn_source_ids.iter().zip(syn_target_ids.iter()).enumerate() {
        if let (Some(&pre_ts), Some(&post_ts)) = (pre_last.get(&src.0), post_last.get(&tgt.0)) {
            // Compute time difference (positive if pre before post)
            let dt = post_ts as i64 - pre_ts as i64;
            
            if dt > 0 {
                // Pre fired before post -> potentiation
                let dt_f = dt as f32;
                factors[i] = config.a_plus * (-dt_f / config.tau_pre.max(1e-6)).exp();
            } else if dt < 0 {
                // Post fired before pre -> depression
                let dt_f = (-dt) as f32;
                factors[i] = -config.a_minus * (-dt_f / config.tau_post.max(1e-6)).exp();
            } else {
                // Same timestep -> strong potentiation
                factors[i] = config.a_plus;
            }
        }
    }
    
    factors
}

/// Group synapses by (source_area, target_area) pairs for batch processing
pub fn group_synapses_by_area_pairs(
    syn_source_ids: &[NeuronId],
    syn_target_ids: &[NeuronId],
    neuron_to_area: &HashMap<u32, u32>,
) -> HashMap<(u32, u32), Vec<usize>> {
    let count = syn_source_ids.len();
    let mut groups: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    
    for i in 0..count {
        let src = syn_source_ids[i].0;
        let tgt = syn_target_ids[i].0;
        
        if let (Some(&s_area), Some(&t_area)) = (neuron_to_area.get(&src), neuron_to_area.get(&tgt)) {
            groups.entry((s_area, t_area))
                .or_insert_with(Vec::new)
                .push(i);
        }
    }
    
    groups
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_stdp_config_default() {
        let config = STDPConfig::default();
        assert_eq!(config.lookback_steps, 20);
        assert_eq!(config.tau_pre, 20.0);
    }
    
    #[test]
    fn test_activity_factors() {
        let sources = vec![NeuronId(1), NeuronId(2)];
        let targets = vec![NeuronId(10), NeuronId(11)];
        let source_history = vec![NeuronId(1)];  // Only neuron 1 fired
        let target_history = vec![NeuronId(10)]; // Only neuron 10 fired
        
        let (pre, post) = compute_activity_factors(&sources, &targets, &source_history, &target_history);
        
        assert_eq!(pre[0], 1.0); // Neuron 1 was active
        assert_eq!(pre[1], 0.0); // Neuron 2 was not active
        assert_eq!(post[0], 1.0); // Neuron 10 was active
        assert_eq!(post[1], 0.0); // Neuron 11 was not active
    }
    
    #[test]
    fn test_timing_factors_potentiation() {
        let config = STDPConfig::default();
        let sources = vec![NeuronId(1)];
        let targets = vec![NeuronId(10)];
        
        // Pre fired at t=5, post fired at t=10 (pre before post)
        let source_history = vec![(5, NeuronId(1))];
        let target_history = vec![(10, NeuronId(10))];
        
        let factors = compute_timing_factors(&sources, &targets, &source_history, &target_history, &config);
        
        assert!(factors[0] > 0.0); // Should be positive (potentiation)
    }
    
    #[test]
    fn test_timing_factors_depression() {
        let config = STDPConfig::default();
        let sources = vec![NeuronId(1)];
        let targets = vec![NeuronId(10)];
        
        // Post fired at t=5, pre fired at t=10 (post before pre)
        let source_history = vec![(10, NeuronId(1))];
        let target_history = vec![(5, NeuronId(10))];
        
        let factors = compute_timing_factors(&sources, &targets, &source_history, &target_history, &config);
        
        assert!(factors[0] < 0.0); // Should be negative (depression)
    }
}

