/*!
# FEAGI BDU (Brain Development Utilities)

This crate implements high-performance brain development operations including:
- Synaptogenesis (synapse creation based on morphology rules)
- Connectivity rules (projection, topology, patterns)
- Spatial hashing and coordinate transformations

## Architecture

Mirrors the Python structure:
- `feagi/bdu/connectivity/` → `feagi_bdu::connectivity`
- `feagi/bdu/morton_spatial_hash.py` → `feagi_bdu::spatial`

## Performance Goals

- 40x-100x faster than Python implementation
- Sub-second projection mappings for 128×128×20 areas
- SIMD-optimized coordinate transformations
- Parallel processing for large mappings

## FFI / Python Integration

All public functions are exposed via PyO3 for seamless Python integration.
See `ffi` module for C-compatible API.

Copyright 2025 Neuraville Inc.
Licensed under the Apache License, Version 2.0
*/

pub mod connectivity;
pub mod ffi;
pub mod spatial;
pub mod types;

// Re-export commonly used types
pub use connectivity::{
    find_candidate_neurons, CandidateNeuron, MorphologyParams, SynaptogenesisRequest,
    SynaptogenesisResult,
};

pub use spatial::{MortonHash, SpatialLookup};

pub use types::{AreaId, NeuronId, Position, Weight};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_projection() {
        // Smoke test to ensure modules compile
        let result = connectivity::rules::syn_projector(
            "src_area",
            "dst_area",
            42,
            (128, 128, 3),
            (128, 128, 1),
            (0, 0, 0),
            None,
            None,
        );
        assert!(result.is_ok());
    }
}

