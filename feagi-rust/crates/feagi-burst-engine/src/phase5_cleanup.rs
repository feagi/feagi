/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # Phase 5: Cleanup
//!
//! Cleanup operations after burst processing.

use feagi_types::*;

/// Phase 5: Cleanup
///
/// Clears temporary structures for next burst
pub fn phase5_cleanup(
    fcl: &mut FireCandidateList,
) -> Result<()> {
    fcl.clear();
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cleanup() {
        let mut fcl = FireCandidateList::new();
        fcl.add_candidate(NeuronId(1), 1.0);
        fcl.add_candidate(NeuronId(2), 0.5);
        
        assert_eq!(fcl.len(), 2);
        
        phase5_cleanup(&mut fcl).unwrap();
        
        assert_eq!(fcl.len(), 0);
    }
}
