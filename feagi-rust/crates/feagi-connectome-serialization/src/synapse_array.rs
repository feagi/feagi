/*
 * Copyright 2025 Neuraville Inc.
 */

//! Serializable synapse array structures

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Serializable version of SynapseArray
///
/// This captures all synapse data from the RustNPU.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SerializableSynapseArray {
    /// Number of valid synapses
    pub count: usize,

    /// Capacity (pre-allocated size)
    pub capacity: usize,

    /// Source neuron IDs (u32)
    pub source_neurons: Vec<u32>,

    /// Target neuron IDs (u32)
    pub target_neurons: Vec<u32>,

    /// Synaptic weights (u8, 0-255)
    pub weights: Vec<u8>,

    /// Synaptic conductances (u8, 0-255)
    pub conductances: Vec<u8>,

    /// Synapse types (u8: 0=excitatory, 1=inhibitory)
    pub types: Vec<u8>,

    /// Valid mask (bool)
    pub valid_mask: Vec<bool>,

    /// Source neuron index (for fast lookup)
    /// Maps source_neuron_id -> Vec<synapse_index>
    pub source_index: HashMap<u32, Vec<usize>>,
}

impl Default for SerializableSynapseArray {
    fn default() -> Self {
        Self {
            count: 0,
            capacity: 0,
            source_neurons: Vec::new(),
            target_neurons: Vec::new(),
            weights: Vec::new(),
            conductances: Vec::new(),
            types: Vec::new(),
            valid_mask: Vec::new(),
            source_index: HashMap::new(),
        }
    }
}

impl SerializableSynapseArray {
    /// Create a new empty synapse array
    pub fn new(capacity: usize) -> Self {
        Self {
            count: 0,
            capacity,
            source_neurons: vec![0; capacity],
            target_neurons: vec![0; capacity],
            weights: vec![0; capacity],
            conductances: vec![0; capacity],
            types: vec![0; capacity],
            valid_mask: vec![false; capacity],
            source_index: HashMap::new(),
        }
    }
}

