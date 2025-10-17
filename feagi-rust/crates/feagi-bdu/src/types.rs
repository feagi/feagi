/*!
Core types for BDU operations.

These types match the Python API for seamless integration.
*/

use serde::{Deserialize, Serialize};

/// Cortical area identifier (6-character string in Python)
pub type AreaId = String;

/// Unique neuron identifier
pub type NeuronId = u64;

/// 3D position (x, y, z)
pub type Position = (i32, i32, i32);

/// Synaptic weight (0-255 in u8, converted from Python float)
pub type Weight = u8;

/// Result type for BDU operations
pub type BduResult<T> = Result<T, BduError>;

/// Errors that can occur during BDU operations
#[derive(Debug, thiserror::Error)]
pub enum BduError {
    #[error("Invalid area: {0}")]
    InvalidArea(String),

    #[error("Invalid morphology: {0}")]
    InvalidMorphology(String),

    #[error("Invalid position: {0:?}")]
    InvalidPosition(Position),

    #[error("Dimension mismatch: expected {expected:?}, got {actual:?}")]
    DimensionMismatch {
        expected: (usize, usize, usize),
        actual: (usize, usize, usize),
    },

    #[error("Out of bounds: position {pos:?} not in dimensions {dims:?}")]
    OutOfBounds {
        pos: Position,
        dims: (usize, usize, usize),
    },

    #[error("Internal error: {0}")]
    Internal(String),
}

/// 3D dimensions (width, height, depth)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Dimensions {
    pub width: usize,
    pub height: usize,
    pub depth: usize,
}

impl Dimensions {
    pub fn new(width: usize, height: usize, depth: usize) -> Self {
        Self {
            width,
            height,
            depth,
        }
    }

    pub fn from_tuple(tuple: (usize, usize, usize)) -> Self {
        Self::new(tuple.0, tuple.1, tuple.2)
    }

    pub fn to_tuple(&self) -> (usize, usize, usize) {
        (self.width, self.height, self.depth)
    }

    pub fn contains(&self, pos: Position) -> bool {
        pos.0 >= 0
            && pos.1 >= 0
            && pos.2 >= 0
            && (pos.0 as usize) < self.width
            && (pos.1 as usize) < self.height
            && (pos.2 as usize) < self.depth
    }

    pub fn total_voxels(&self) -> usize {
        self.width * self.height * self.depth
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dimensions_contains() {
        let dims = Dimensions::new(128, 128, 20);
        assert!(dims.contains((0, 0, 0)));
        assert!(dims.contains((127, 127, 19)));
        assert!(!dims.contains((128, 0, 0)));
        assert!(!dims.contains((-1, 0, 0)));
    }

    #[test]
    fn test_dimensions_total_voxels() {
        let dims = Dimensions::new(128, 128, 20);
        assert_eq!(dims.total_voxels(), 128 * 128 * 20);
    }
}

