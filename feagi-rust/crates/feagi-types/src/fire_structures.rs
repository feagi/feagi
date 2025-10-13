/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # Fire Structures
//!
//! Fire Candidate List (FCL), Fire Queue (FQ), and Fire Ledger for tracking neural activity.
//!
//! ## Data Flow
//! ```text
//! FCL (candidates) → Fire Queue (current firing) → Fire Ledger (history)
//! ```

use crate::*;
use std::collections::{HashMap, VecDeque};

/// Fire Candidate List (FCL) - neurons that might fire this burst
///
/// Holds candidates from:
/// - Synaptic input (from previous burst's propagation)
/// - Power injection
/// - Sensory input
#[derive(Debug, Clone)]
pub struct FireCandidateList {
    /// Candidate neurons: neuron_id -> accumulated potential
    candidates: HashMap<u32, f32>,
}

impl FireCandidateList {
    /// Create a new FCL
    pub fn new() -> Self {
        Self {
            candidates: HashMap::new(),
        }
    }
    
    /// Add a candidate (or accumulate if already exists)
    #[inline]
    pub fn add_candidate(&mut self, neuron_id: NeuronId, potential: f32) {
        *self.candidates.entry(neuron_id.0).or_insert(0.0) += potential;
    }
    
    /// Get accumulated potential for a neuron
    #[inline]
    pub fn get_potential(&self, neuron_id: NeuronId) -> f32 {
        self.candidates.get(&neuron_id.0).copied().unwrap_or(0.0)
    }
    
    /// Get all candidates
    pub fn get_all_candidates(&self) -> Vec<(NeuronId, f32)> {
        self.candidates
            .iter()
            .map(|(&id, &potential)| (NeuronId(id), potential))
            .collect()
    }
    
    /// Get number of candidates
    pub fn len(&self) -> usize {
        self.candidates.len()
    }
    
    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.candidates.is_empty()
    }
    
    /// Clear all candidates
    pub fn clear(&mut self) {
        self.candidates.clear();
    }
}

impl Default for FireCandidateList {
    fn default() -> Self {
        Self::new()
    }
}

/// Firing neuron record (for Fire Queue)
#[derive(Debug, Clone, Copy)]
pub struct FiringNeuron {
    pub neuron_id: NeuronId,
    pub membrane_potential: f32,
    pub cortical_area: CorticalAreaId,
    pub x: u32,
    pub y: u32,
    pub z: u32,
}

/// Fire Queue (FQ) - neurons that ARE firing this burst
///
/// Maintains current burst's firing neurons with their properties
#[derive(Debug, Clone)]
pub struct FireQueue {
    /// Firing neurons in this burst
    neurons: Vec<FiringNeuron>,
    
    /// Quick lookup: neuron_id -> index in neurons vector
    neuron_index: HashMap<u32, usize>,
}

impl FireQueue {
    /// Create a new Fire Queue
    pub fn new() -> Self {
        Self {
            neurons: Vec::new(),
            neuron_index: HashMap::new(),
        }
    }
    
    /// Add a firing neuron
    pub fn add_neuron(&mut self, neuron: FiringNeuron) {
        let idx = self.neurons.len();
        self.neuron_index.insert(neuron.neuron_id.0, idx);
        self.neurons.push(neuron);
    }
    
    /// Get all firing neuron IDs
    pub fn get_all_neuron_ids(&self) -> Vec<NeuronId> {
        self.neurons.iter().map(|n| n.neuron_id).collect()
    }
    
    /// Get all firing neurons
    pub fn get_all_neurons(&self) -> &[FiringNeuron] {
        &self.neurons
    }
    
    /// Get firing neuron by ID
    pub fn get_neuron(&self, neuron_id: NeuronId) -> Option<&FiringNeuron> {
        self.neuron_index
            .get(&neuron_id.0)
            .and_then(|&idx| self.neurons.get(idx))
    }
    
    /// Get number of firing neurons
    pub fn len(&self) -> usize {
        self.neurons.len()
    }
    
    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.neurons.is_empty()
    }
    
    /// Clear all firing neurons
    pub fn clear(&mut self) {
        self.neurons.clear();
        self.neuron_index.clear();
    }
}

impl Default for FireQueue {
    fn default() -> Self {
        Self::new()
    }
}

/// Fire history entry (for Fire Ledger)
#[derive(Debug, Clone)]
pub struct FireHistory {
    /// Burst timestep when this occurred
    pub burst: u64,
    
    /// Firing neurons in that burst
    pub neurons: Vec<NeuronId>,
}

/// Fire Ledger - historical record of neural firing
///
/// Maintains a sliding window of past firing history
#[derive(Debug, Clone)]
pub struct FireLedger {
    /// Maximum number of bursts to keep in history
    window_size: usize,
    
    /// Historical fire records (oldest to newest)
    history: VecDeque<FireHistory>,
}

impl FireLedger {
    /// Create a new Fire Ledger with specified window size
    pub fn new(window_size: usize) -> Self {
        Self {
            window_size,
            history: VecDeque::with_capacity(window_size),
        }
    }
    
    /// Record a burst's firing
    pub fn record_burst(&mut self, burst: u64, neurons: Vec<NeuronId>) {
        // Add new record
        self.history.push_back(FireHistory { burst, neurons });
        
        // Remove old records if exceeding window size
        while self.history.len() > self.window_size {
            self.history.pop_front();
        }
    }
    
    /// Get history for a specific burst
    pub fn get_burst(&self, burst: u64) -> Option<&FireHistory> {
        self.history.iter().find(|h| h.burst == burst)
    }
    
    /// Get recent history (last N bursts)
    pub fn get_recent_history(&self, count: usize) -> Vec<&FireHistory> {
        self.history
            .iter()
            .rev()
            .take(count)
            .collect()
    }
    
    /// Get all history
    pub fn get_all_history(&self) -> &VecDeque<FireHistory> {
        &self.history
    }
    
    /// Clear all history
    pub fn clear(&mut self) {
        self.history.clear();
    }
    
    /// Get window size
    pub fn window_size(&self) -> usize {
        self.window_size
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fcl() {
        let mut fcl = FireCandidateList::new();
        
        fcl.add_candidate(NeuronId(1), 0.5);
        fcl.add_candidate(NeuronId(1), 0.3);  // Accumulates
        fcl.add_candidate(NeuronId(2), 1.0);
        
        assert_eq!(fcl.len(), 2);
        assert_eq!(fcl.get_potential(NeuronId(1)), 0.8);
        assert_eq!(fcl.get_potential(NeuronId(2)), 1.0);
    }

    #[test]
    fn test_fire_queue() {
        let mut fq = FireQueue::new();
        
        fq.add_neuron(FiringNeuron {
            neuron_id: NeuronId(1),
            membrane_potential: 1.5,
            cortical_area: CorticalAreaId(1),
            x: 0, y: 0, z: 0,
        });
        
        fq.add_neuron(FiringNeuron {
            neuron_id: NeuronId(2),
            membrane_potential: 2.0,
            cortical_area: CorticalAreaId(1),
            x: 1, y: 0, z: 0,
        });
        
        assert_eq!(fq.len(), 2);
        assert!(fq.get_neuron(NeuronId(1)).is_some());
        
        let ids = fq.get_all_neuron_ids();
        assert_eq!(ids.len(), 2);
    }

    #[test]
    fn test_fire_ledger() {
        let mut ledger = FireLedger::new(5);
        
        ledger.record_burst(1, vec![NeuronId(1), NeuronId(2)]);
        ledger.record_burst(2, vec![NeuronId(3)]);
        ledger.record_burst(3, vec![NeuronId(1), NeuronId(4)]);
        
        assert_eq!(ledger.history.len(), 3);
        
        let burst1 = ledger.get_burst(1).unwrap();
        assert_eq!(burst1.neurons.len(), 2);
        
        let recent = ledger.get_recent_history(2);
        assert_eq!(recent.len(), 2);
        assert_eq!(recent[0].burst, 3);  // Most recent
        assert_eq!(recent[1].burst, 2);
    }

    #[test]
    fn test_fire_ledger_window() {
        let mut ledger = FireLedger::new(3);
        
        for i in 1..=5 {
            ledger.record_burst(i, vec![NeuronId(i as u32)]);
        }
        
        // Should only keep last 3
        assert_eq!(ledger.history.len(), 3);
        assert_eq!(ledger.history.front().unwrap().burst, 3);
        assert_eq!(ledger.history.back().unwrap().burst, 5);
    }
}
