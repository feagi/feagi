/*!
Connectivity and synaptogenesis operations.

This module implements high-performance synapse creation based on morphology rules.
*/

pub mod rules;
pub mod synaptogenesis;

pub use synaptogenesis::{
    find_candidate_neurons, CandidateNeuron, MorphologyParams, SynaptogenesisRequest,
    SynaptogenesisResult,
};

pub use rules::{syn_projector, ProjectorParams};

