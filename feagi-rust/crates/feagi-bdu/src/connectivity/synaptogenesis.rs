/*!
Main synaptogenesis entry point - finding candidate neurons for connections.
*/

use crate::types::{BduError, BduResult, NeuronId, Position};
use serde::{Deserialize, Serialize};

/// Candidate neuron with synaptic weight
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[repr(C)]
pub struct CandidateNeuron {
    pub neuron_id: NeuronId,
    pub weight: f32,
}

/// Morphology parameters for synaptogenesis
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MorphologyParams {
    pub morphology_id: String,
    pub morphology_scalar: [f32; 3],
    pub psc_multiplier: f32,
}

/// Request for synaptogenesis operation
#[derive(Debug, Clone)]
pub struct SynaptogenesisRequest {
    pub src_area_id: String,
    pub dst_area_id: String,
    pub src_neuron_id: NeuronId,
    pub src_dimensions: (usize, usize, usize),
    pub dst_dimensions: (usize, usize, usize),
    pub neuron_location: Position,
    pub morphology: MorphologyParams,
}

/// Result of synaptogenesis operation
#[derive(Debug, Clone)]
pub struct SynaptogenesisResult {
    pub candidate_neurons: Vec<CandidateNeuron>,
    pub candidate_positions: Vec<Position>,
}

/// Find candidate neurons for synapse formation.
///
/// This is the main entry point that will eventually replace the Python
/// `find_candidate_neurons` function.
///
/// Phase 1: Implements projector morphology only
/// Phase 2+: Add remaining morphology types
pub fn find_candidate_neurons(
    request: &SynaptogenesisRequest,
) -> BduResult<SynaptogenesisResult> {
    // Phase 1: Only implement PROJECTOR morphology
    let morphology_id = request.morphology.morphology_id.as_str();

    let candidate_positions = match morphology_id {
        "PROJECTOR" | "projector" => {
            super::rules::syn_projector(
                &request.src_area_id,
                &request.dst_area_id,
                request.src_neuron_id,
                request.src_dimensions,
                request.dst_dimensions,
                request.neuron_location,
                None, // No transpose for basic projector
                None, // No project_last_layer
            )?
        }
        _ => {
            return Err(BduError::InvalidMorphology(format!(
                "Morphology '{}' not yet implemented in Rust. Use Python fallback.",
                morphology_id
            )));
        }
    };

    // Convert positions to candidate neurons
    // Note: In Phase 1, we return positions. Python will do the position->neuron lookup.
    // Phase 2: Integrate spatial hash for full Rust pipeline
    let candidate_neurons = Vec::new(); // Populated by Python for now

    Ok(SynaptogenesisResult {
        candidate_neurons,
        candidate_positions,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_projector_morphology() {
        let request = SynaptogenesisRequest {
            src_area_id: "src001".to_string(),
            dst_area_id: "dst001".to_string(),
            src_neuron_id: 42,
            src_dimensions: (128, 128, 3),
            dst_dimensions: (128, 128, 1),
            neuron_location: (64, 64, 1),
            morphology: MorphologyParams {
                morphology_id: "PROJECTOR".to_string(),
                morphology_scalar: [1.0, 1.0, 1.0],
                psc_multiplier: 1.0,
            },
        };

        let result = find_candidate_neurons(&request);
        assert!(result.is_ok());

        let syn_result = result.unwrap();
        assert!(!syn_result.candidate_positions.is_empty());
    }

    #[test]
    fn test_unsupported_morphology() {
        let request = SynaptogenesisRequest {
            src_area_id: "src001".to_string(),
            dst_area_id: "dst001".to_string(),
            src_neuron_id: 42,
            src_dimensions: (128, 128, 3),
            dst_dimensions: (128, 128, 1),
            neuron_location: (64, 64, 1),
            morphology: MorphologyParams {
                morphology_id: "UNSUPPORTED".to_string(),
                morphology_scalar: [1.0, 1.0, 1.0],
                psc_multiplier: 1.0,
            },
        };

        let result = find_candidate_neurons(&request);
        assert!(result.is_err());
    }
}

