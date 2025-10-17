/*!
Spatial indexing utilities for efficient position-based queries.

Implements Morton encoding (Z-order curve) + Roaring bitmaps for:
- Fast neuron position lookups
- Region queries
- Spatial locality preservation
*/

pub mod morton;
pub mod hash;

pub use morton::{morton_encode_3d, morton_decode_3d, morton_encode_region_3d};
pub use hash::{MortonSpatialHash, SpatialHashStats};
