/*
 * Copyright 2025 Neuraville Inc.
 *
 * PROPRIETARY AND CONFIDENTIAL
 * 
 * This module contains proprietary plasticity algorithms.
 * Separate crate for IP protection.
 */

//! # FEAGI Plasticity Module
//!
//! This crate implements synaptic plasticity algorithms for FEAGI.
//! It is intentionally kept as a separate crate for intellectual property protection.
//!
//! ## Future Implementation
//! - Short-term plasticity (STP)
//! - Long-term potentiation/depression (LTP/LTD)
//! - Spike-timing-dependent plasticity (STDP)
//! - Homeostatic plasticity
//!
//! ## Architecture
//! - Trait-based plugin system
//! - No direct dependency on burst-engine (only on feagi-types)
//! - Can be compiled separately and linked at runtime

use feagi_types::*;

/// Trait for plasticity rules
/// This allows different plasticity algorithms to be plugged in
pub trait PlasticityRule: Send + Sync {
    /// Apply plasticity to a synapse
    fn apply(&self, synapse: &mut Synapse, pre_fire: bool, post_fire: bool, dt: f32);
    
    /// Get the name of this plasticity rule
    fn name(&self) -> &str;
}

/// Placeholder plasticity implementation (to be filled in later)
pub struct NoPlasticity;

impl PlasticityRule for NoPlasticity {
    fn apply(&self, _synapse: &mut Synapse, _pre_fire: bool, _post_fire: bool, _dt: f32) {
        // No-op: no plasticity applied
    }

    fn name(&self) -> &str {
        "NoPlasticity"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_no_plasticity() {
        let rule = NoPlasticity;
        let mut synapse = Synapse {
            source_neuron: NeuronId(1),
            target_neuron: NeuronId(2),
            weight: SynapticWeight(128),
            conductance: SynapticConductance(255),
            synapse_type: SynapseType::Excitatory,
            valid: true,
        };
        
        let original_weight = synapse.weight;
        rule.apply(&mut synapse, true, true, 0.1);
        assert_eq!(synapse.weight, original_weight);
    }
}




