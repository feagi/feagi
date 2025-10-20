/*
 * Copyright 2025 Neuraville Inc.
 */

//! Serializable neuron array structures

use serde::{Deserialize, Serialize};

/// Serializable version of NeuronArray
///
/// This captures all neuron data from the RustNPU in a format
/// that can be efficiently serialized to disk.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SerializableNeuronArray {
    /// Number of valid neurons
    pub count: usize,

    /// Capacity (pre-allocated size)
    pub capacity: usize,

    /// Membrane potentials (f32)
    pub membrane_potentials: Vec<f32>,

    /// Firing thresholds (f32)
    pub thresholds: Vec<f32>,

    /// Leak coefficients (f32, 0-1 range for exponential decay)
    pub leak_coefficients: Vec<f32>,

    /// Resting potentials (f32)
    pub resting_potentials: Vec<f32>,

    /// Neuron types (i32)
    pub neuron_types: Vec<i32>,

    /// Refractory periods (u16)
    pub refractory_periods: Vec<u16>,

    /// Current refractory countdowns (u16)
    pub refractory_countdowns: Vec<u16>,

    /// Excitability multipliers (f32)
    pub excitabilities: Vec<f32>,

    /// Cortical area IDs (u32)
    pub cortical_areas: Vec<u32>,

    /// 3D coordinates (flat array: [x0, y0, z0, x1, y1, z1, ...])
    pub coordinates: Vec<u32>,

    /// Valid mask (bool)
    pub valid_mask: Vec<bool>,
}

impl Default for SerializableNeuronArray {
    fn default() -> Self {
        Self {
            count: 0,
            capacity: 0,
            membrane_potentials: Vec::new(),
            thresholds: Vec::new(),
            leak_coefficients: Vec::new(),
            resting_potentials: Vec::new(),
            neuron_types: Vec::new(),
            refractory_periods: Vec::new(),
            refractory_countdowns: Vec::new(),
            excitabilities: Vec::new(),
            cortical_areas: Vec::new(),
            coordinates: Vec::new(),
            valid_mask: Vec::new(),
        }
    }
}

impl SerializableNeuronArray {
    /// Create a new empty neuron array
    pub fn new(capacity: usize) -> Self {
        Self {
            count: 0,
            capacity,
            membrane_potentials: vec![0.0; capacity],
            thresholds: vec![0.0; capacity],
            leak_coefficients: vec![0.0; capacity],
            resting_potentials: vec![0.0; capacity],
            neuron_types: vec![0; capacity],
            refractory_periods: vec![0; capacity],
            refractory_countdowns: vec![0; capacity],
            excitabilities: vec![1.0; capacity],
            cortical_areas: vec![0; capacity],
            coordinates: vec![0; capacity * 3], // x, y, z for each neuron
            valid_mask: vec![false; capacity],
        }
    }
}

