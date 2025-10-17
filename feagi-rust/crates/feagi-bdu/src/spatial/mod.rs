/*!
Spatial hashing and coordinate lookups.

Phase 1: Stub module (position->neuron lookup done in Python)
Phase 2: Implement full Morton spatial hash integration
*/

use crate::types::{BduResult, NeuronId, Position};
use std::collections::HashMap;

/// Spatial hash for O(1) position lookups
pub struct MortonHash {
    position_to_neurons: HashMap<Position, Vec<NeuronId>>,
}

impl MortonHash {
    pub fn new() -> Self {
        Self {
            position_to_neurons: HashMap::new(),
        }
    }

    pub fn insert(&mut self, position: Position, neuron_id: NeuronId) {
        self.position_to_neurons
            .entry(position)
            .or_insert_with(Vec::new)
            .push(neuron_id);
    }

    pub fn lookup(&self, position: &Position) -> Option<&[NeuronId]> {
        self.position_to_neurons.get(position).map(|v| v.as_slice())
    }
}

impl Default for MortonHash {
    fn default() -> Self {
        Self::new()
    }
}

/// Spatial lookup trait (for future optimization)
pub trait SpatialLookup {
    fn lookup_neurons(&self, position: &Position) -> BduResult<Vec<NeuronId>>;
    fn lookup_batch(&self, positions: &[Position]) -> BduResult<Vec<Vec<NeuronId>>>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_morton_hash_basic() {
        let mut hash = MortonHash::new();
        hash.insert((0, 0, 0), 1);
        hash.insert((0, 0, 0), 2);
        hash.insert((1, 1, 1), 3);

        let neurons = hash.lookup(&(0, 0, 0));
        assert!(neurons.is_some());
        assert_eq!(neurons.unwrap().len(), 2);

        let neurons = hash.lookup(&(1, 1, 1));
        assert!(neurons.is_some());
        assert_eq!(neurons.unwrap().len(), 1);

        let neurons = hash.lookup(&(2, 2, 2));
        assert!(neurons.is_none());
    }
}

