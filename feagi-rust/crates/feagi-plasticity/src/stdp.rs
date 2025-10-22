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
        assert_eq!(config.tau_post, 20.0);
        assert_eq!(config.a_plus, 0.01);
        assert_eq!(config.a_minus, 0.012);
    }
    
    #[test]
    fn test_stdp_config_custom() {
        let config = STDPConfig {
            lookback_steps: 50,
            tau_pre: 30.0,
            tau_post: 25.0,
            a_plus: 0.02,
            a_minus: 0.015,
            max_pairs_per_synapse: 10,
        };
        
        assert_eq!(config.lookback_steps, 50);
        assert_eq!(config.tau_pre, 30.0);
    }
    
    #[test]
    fn test_activity_factors_empty() {
        let sources = vec![];
        let targets = vec![];
        let source_history = vec![];
        let target_history = vec![];
        
        let (pre, post) = compute_activity_factors(&sources, &targets, &source_history, &target_history);
        
        assert_eq!(pre.len(), 0);
        assert_eq!(post.len(), 0);
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
    fn test_activity_factors_all_active() {
        let sources = vec![NeuronId(1), NeuronId(2), NeuronId(3)];
        let targets = vec![NeuronId(10), NeuronId(11), NeuronId(12)];
        let source_history = vec![NeuronId(1), NeuronId(2), NeuronId(3)];
        let target_history = vec![NeuronId(10), NeuronId(11), NeuronId(12)];
        
        let (pre, post) = compute_activity_factors(&sources, &targets, &source_history, &target_history);
        
        assert_eq!(pre.len(), 3);
        assert_eq!(post.len(), 3);
        assert!(pre.iter().all(|&x| x == 1.0));
        assert!(post.iter().all(|&x| x == 1.0));
    }
    
    #[test]
    fn test_timing_factors_empty() {
        let config = STDPConfig::default();
        let sources = vec![];
        let targets = vec![];
        let source_history = vec![];
        let target_history = vec![];
        
        let factors = compute_timing_factors(&sources, &targets, &source_history, &target_history, &config);
        
        assert_eq!(factors.len(), 0);
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
        assert!(factors[0] <= config.a_plus); // Should be bounded by a_plus
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
        assert!(factors[0] >= -config.a_minus); // Should be bounded by -a_minus
    }
    
    #[test]
    fn test_timing_factors_same_timestep() {
        let config = STDPConfig::default();
        let sources = vec![NeuronId(1)];
        let targets = vec![NeuronId(10)];
        
        // Both fired at same timestep
        let source_history = vec![(5, NeuronId(1))];
        let target_history = vec![(5, NeuronId(10))];
        
        let factors = compute_timing_factors(&sources, &targets, &source_history, &target_history, &config);
        
        assert_eq!(factors[0], config.a_plus); // Should be strong potentiation
    }
    
    #[test]
    fn test_timing_factors_no_history() {
        let config = STDPConfig::default();
        let sources = vec![NeuronId(1), NeuronId(2)];
        let targets = vec![NeuronId(10), NeuronId(11)];
        
        // No firing history for these neurons
        let source_history = vec![(5, NeuronId(99))];
        let target_history = vec![(10, NeuronId(99))];
        
        let factors = compute_timing_factors(&sources, &targets, &source_history, &target_history, &config);
        
        assert_eq!(factors[0], 0.0); // No change if neurons didn't fire
        assert_eq!(factors[1], 0.0);
    }
    
    #[test]
    fn test_timing_factors_exponential_decay() {
        let config = STDPConfig::default();
        let sources = vec![NeuronId(1)];
        let targets = vec![NeuronId(10)];
        
        // Test that closer spikes have stronger effects
        let source_history1 = vec![(5, NeuronId(1))];
        let target_history1 = vec![(6, NeuronId(10))]; // dt = 1
        
        let factors1 = compute_timing_factors(&sources, &targets, &source_history1, &target_history1, &config);
        
        let source_history2 = vec![(5, NeuronId(1))];
        let target_history2 = vec![(15, NeuronId(10))]; // dt = 10
        
        let factors2 = compute_timing_factors(&sources, &targets, &source_history2, &target_history2, &config);
        
        assert!(factors1[0] > factors2[0]); // Closer spikes should have stronger effect
    }
    
    #[test]
    fn test_group_synapses_by_area_pairs() {
        let mut neuron_to_area = HashMap::new();
        neuron_to_area.insert(1, 10);
        neuron_to_area.insert(2, 10);
        neuron_to_area.insert(3, 20);
        neuron_to_area.insert(11, 30);
        neuron_to_area.insert(12, 30);
        neuron_to_area.insert(13, 40);
        
        let sources = vec![NeuronId(1), NeuronId(2), NeuronId(3)];
        let targets = vec![NeuronId(11), NeuronId(12), NeuronId(13)];
        
        let groups = group_synapses_by_area_pairs(&sources, &targets, &neuron_to_area);
        
        // Should have 3 groups: (10,30), (10,30), (20,40)
        assert!(groups.contains_key(&(10, 30)));
        assert!(groups.contains_key(&(20, 40)));
        
        let group_10_30 = &groups[&(10, 30)];
        assert_eq!(group_10_30.len(), 2); // Two synapses from area 10 to area 30
    }
    
    #[test]
    fn test_group_synapses_missing_neurons() {
        let neuron_to_area = HashMap::new(); // Empty mapping
        
        let sources = vec![NeuronId(1), NeuronId(2)];
        let targets = vec![NeuronId(10), NeuronId(11)];
        
        let groups = group_synapses_by_area_pairs(&sources, &targets, &neuron_to_area);
        
        assert_eq!(groups.len(), 0); // Should have no groups if neurons not found
    }
}

