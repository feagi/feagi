/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

//! # FEAGI Core Types
//!
//! Shared types for the FEAGI neural processing framework.
//! 
//! ## Design Philosophy
//! - **RTOS-Compatible**: No allocations in hot paths
//! - **Zero-copy**: Use references and slices where possible
//! - **Type-safe**: Use strong types instead of primitives
//! - **Cache-friendly**: Struct layouts optimized for CPU cache

use std::fmt;

pub mod npu;
pub mod fire_structures;

pub use npu::*;
pub use fire_structures::*;

/// Neuron ID (globally unique across the entire brain)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct NeuronId(pub u32);

impl fmt::Display for NeuronId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Neuron({})", self.0)
    }
}

/// Cortical Area ID (brain region identifier)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CorticalAreaId(pub u32);

impl fmt::Display for CorticalAreaId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Area({})", self.0)
    }
}

/// Synapse ID (unique identifier for a synaptic connection)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SynapseId(pub u32);

/// Synaptic weight (0-255, stored as u8 for memory efficiency)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SynapticWeight(pub u8);

impl SynapticWeight {
    /// Convert to float (direct cast, NO normalization - matches Python behavior)
    #[inline(always)]
    pub fn to_float(self) -> f32 {
        self.0 as f32  // Direct cast: 1 → 1.0 (same as Python's .astype(np.float32))
    }

    /// Create from float (direct cast)
    #[inline(always)]
    pub fn from_float(value: f32) -> Self {
        Self(value as u8)
    }
}

/// Synaptic conductance (0-255, stored as u8)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SynapticConductance(pub u8);

impl SynapticConductance {
    /// Convert to float (direct cast, NO normalization - matches Python behavior)
    #[inline(always)]
    pub fn to_float(self) -> f32 {
        self.0 as f32  // Direct cast: 1 → 1.0 (same as Python's .astype(np.float32))
    }

    /// Create from float (direct cast)
    #[inline(always)]
    pub fn from_float(value: f32) -> Self {
        Self(value as u8)
    }
}

/// Synaptic contribution (weight × conductance × sign)
/// This is the actual "power" injected into the target neuron
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SynapticContribution(pub f32);

/// Synapse type (excitatory or inhibitory)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SynapseType {
    Excitatory = 0,
    Inhibitory = 1,
}

impl SynapseType {
    /// Get the sign for contribution calculation (+1 or -1)
    #[inline(always)]
    pub fn sign(self) -> f32 {
        match self {
            SynapseType::Excitatory => 1.0,
            SynapseType::Inhibitory => -1.0,
        }
    }

    /// Create from integer (0=excitatory, 1=inhibitory)
    #[inline(always)]
    pub fn from_int(value: u8) -> Self {
        match value {
            0 => SynapseType::Excitatory,
            _ => SynapseType::Inhibitory,
        }
    }
}

/// A single synapse (compact representation)
#[repr(C)]  // C layout for predictable memory layout
#[derive(Debug, Clone, Copy)]
pub struct Synapse {
    pub source_neuron: NeuronId,
    pub target_neuron: NeuronId,
    pub weight: SynapticWeight,
    pub conductance: SynapticConductance,
    pub synapse_type: SynapseType,
    pub valid: bool,  // For soft deletion
}

impl Synapse {
    /// Calculate the synaptic contribution (weight × conductance × sign)
    #[inline(always)]
    pub fn calculate_contribution(&self) -> SynapticContribution {
        if !self.valid {
            return SynapticContribution(0.0);
        }
        let weight = self.weight.to_float();
        let conductance = self.conductance.to_float();
        let sign = self.synapse_type.sign();
        SynapticContribution(weight * conductance * sign)
    }
}

/// Membrane potential (in arbitrary units)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MembranePotential(pub f32);

/// Neuron firing threshold
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FiringThreshold(pub f32);

/// Error types for FEAGI operations
#[derive(Debug, thiserror::Error)]
pub enum FeagiError {
    #[error("Invalid neuron ID: {0}")]
    InvalidNeuronId(u32),

    #[error("Invalid cortical area ID: {0}")]
    InvalidCorticalAreaId(u32),

    #[error("Invalid synapse ID: {0}")]
    InvalidSynapseId(u32),

    #[error("Neuron not found: {0}")]
    NeuronNotFound(NeuronId),

    #[error("Cortical area not found: {0}")]
    CorticalAreaNotFound(CorticalAreaId),

    #[error("Array size mismatch: expected {expected}, got {actual}")]
    ArraySizeMismatch { expected: usize, actual: usize },

    #[error("Computation error: {0}")]
    ComputationError(String),

    #[error("Memory allocation error: {0}")]
    MemoryAllocationError(String),

    #[error("Invalid backend: {0}")]
    InvalidBackend(String),
}

pub type Result<T> = std::result::Result<T, FeagiError>;
pub type Error = FeagiError;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_synaptic_weight_conversion() {
        let weight = SynapticWeight::from_float(0.5);
        assert_eq!(weight.0, 127);
        assert!((weight.to_float() - 0.498).abs() < 0.01);
    }

    #[test]
    fn test_synapse_contribution() {
        let synapse = Synapse {
            source_neuron: NeuronId(1),
            target_neuron: NeuronId(2),
            weight: SynapticWeight(255),  // Max weight
            conductance: SynapticConductance(255),  // Max conductance
            synapse_type: SynapseType::Excitatory,
            valid: true,
        };
        let contribution = synapse.calculate_contribution();
        assert!((contribution.0 - 1.0).abs() < 0.01);  // Should be ~1.0

        let inhibitory = Synapse {
            synapse_type: SynapseType::Inhibitory,
            ..synapse
        };
        let contribution = inhibitory.calculate_contribution();
        assert!((contribution.0 + 1.0).abs() < 0.01);  // Should be ~-1.0
    }
}
