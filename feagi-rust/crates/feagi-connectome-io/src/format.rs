/*
 * Copyright 2025 Neuraville Inc.
 */

//! Connectome format utilities and helpers

use crate::ConnectomeSnapshot;

impl ConnectomeSnapshot {
    /// Get human-readable summary of the connectome
    pub fn summary(&self) -> String {
        format!(
            "Connectome v{}: {} neurons, {} synapses, {} cortical areas (burst: {})",
            self.version,
            self.neurons.count,
            self.synapses.count,
            self.cortical_area_names.len(),
            self.burst_count
        )
    }

    /// Validate the connectome structure
    pub fn validate(&self) -> Result<(), String> {
        // Check neuron array consistency
        if self.neurons.membrane_potentials.len() != self.neurons.capacity {
            return Err(format!(
                "Neuron array size mismatch: membrane_potentials.len()={}, capacity={}",
                self.neurons.membrane_potentials.len(),
                self.neurons.capacity
            ));
        }

        // Check synapse array consistency
        if self.synapses.source_neurons.len() != self.synapses.capacity {
            return Err(format!(
                "Synapse array size mismatch: source_neurons.len()={}, capacity={}",
                self.synapses.source_neurons.len(),
                self.synapses.capacity
            ));
        }

        // Check synapse references are valid
        for i in 0..self.synapses.count {
            if !self.synapses.valid_mask[i] {
                continue;
            }

            let source = self.synapses.source_neurons[i] as usize;
            let target = self.synapses.target_neurons[i] as usize;

            if source >= self.neurons.count {
                return Err(format!(
                    "Synapse {} has invalid source neuron: {}",
                    i, source
                ));
            }

            if target >= self.neurons.count {
                return Err(format!(
                    "Synapse {} has invalid target neuron: {}",
                    i, target
                ));
            }
        }

        Ok(())
    }

    /// Get statistics about the connectome
    pub fn statistics(&self) -> ConnectomeStatistics {
        let mut stats = ConnectomeStatistics::default();

        stats.neuron_count = self.neurons.count;
        stats.synapse_count = self.synapses.count;
        stats.cortical_area_count = self.cortical_area_names.len();

        // Count active synapses
        stats.active_synapse_count = self.synapses.valid_mask[..self.synapses.count]
            .iter()
            .filter(|&&v| v)
            .count();

        // Calculate average synaptic weight
        let total_weight: u32 = self.synapses.weights[..self.synapses.count]
            .iter()
            .map(|&w| w as u32)
            .sum();
        stats.avg_synaptic_weight = if stats.active_synapse_count > 0 {
            total_weight as f32 / stats.active_synapse_count as f32
        } else {
            0.0
        };

        stats
    }
}

/// Statistics about a connectome
#[derive(Debug, Clone, Default)]
pub struct ConnectomeStatistics {
    pub neuron_count: usize,
    pub synapse_count: usize,
    pub active_synapse_count: usize,
    pub cortical_area_count: usize,
    pub avg_synaptic_weight: f32,
}

impl std::fmt::Display for ConnectomeStatistics {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Neurons: {}, Synapses: {} ({} active), Cortical Areas: {}, Avg Weight: {:.2}",
            self.neuron_count,
            self.synapse_count,
            self.active_synapse_count,
            self.cortical_area_count,
            self.avg_synaptic_weight
        )
    }
}

