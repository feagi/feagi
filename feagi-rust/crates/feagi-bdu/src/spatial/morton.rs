/*!
Morton encoding utilities for 3D spatial hashing.

Implements Z-order curve encoding to preserve spatial locality.
*/

/// Morton encode 3D coordinates into a single u64.
///
/// Interleaves bits of x, y, z coordinates to preserve spatial locality.
/// Each dimension limited to 21 bits (0-2,097,151).
#[inline]
pub fn morton_encode_3d(x: u32, y: u32, z: u32) -> u64 {
    // Limit to 21 bits per dimension (63 bits total)
    debug_assert!(x < (1 << 21), "x coordinate exceeds 21-bit limit");
    debug_assert!(y < (1 << 21), "y coordinate exceeds 21-bit limit");
    debug_assert!(z < (1 << 21), "z coordinate exceeds 21-bit limit");
    
    let mut result = 0u64;
    
    // Interleave bits: ...z2y2x2z1y1x1z0y0x0
    for i in 0..21 {
        result |= ((x as u64 & (1 << i)) << (2 * i))
                | ((y as u64 & (1 << i)) << (2 * i + 1))
                | ((z as u64 & (1 << i)) << (2 * i + 2));
    }
    
    result
}

/// Morton decode a u64 back to 3D coordinates.
#[inline]
pub fn morton_decode_3d(morton_code: u64) -> (u32, u32, u32) {
    let mut x = 0u32;
    let mut y = 0u32;
    let mut z = 0u32;
    
    // Extract interleaved bits
    for i in 0..21 {
        x |= ((morton_code & (1 << (3 * i))) >> (2 * i)) as u32;
        y |= ((morton_code & (1 << (3 * i + 1))) >> (2 * i + 1)) as u32;
        z |= ((morton_code & (1 << (3 * i + 2))) >> (2 * i + 2)) as u32;
    }
    
    (x, y, z)
}

/// Encode a 3D region into a vector of Morton codes.
///
/// Returns all Morton codes for coordinates in the inclusive range
/// [x1..=x2, y1..=y2, z1..=z2].
pub fn morton_encode_region_3d(
    x1: u32, y1: u32, z1: u32,
    x2: u32, y2: u32, z2: u32,
) -> Vec<u64> {
    let mut codes = Vec::new();
    
    for z in z1..=z2 {
        for y in y1..=y2 {
            for x in x1..=x2 {
                codes.push(morton_encode_3d(x, y, z));
            }
        }
    }
    
    codes
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_morton_encode_decode() {
        let coords = [(0, 0, 0), (1, 2, 3), (100, 200, 300), (1000, 2000, 3000)];
        
        for (x, y, z) in coords {
            let encoded = morton_encode_3d(x, y, z);
            let (dx, dy, dz) = morton_decode_3d(encoded);
            assert_eq!((x, y, z), (dx, dy, dz), "Round-trip failed for ({}, {}, {})", x, y, z);
        }
    }

    #[test]
    fn test_spatial_locality() {
        // Nearby points should have nearby Morton codes
        let c1 = morton_encode_3d(10, 10, 10);
        let c2 = morton_encode_3d(11, 10, 10);
        let c3 = morton_encode_3d(100, 100, 100);
        
        // c1 and c2 should be closer than c1 and c3
        let diff_near = c1.abs_diff(c2);
        let diff_far = c1.abs_diff(c3);
        assert!(diff_near < diff_far, "Spatial locality not preserved");
    }

    #[test]
    fn test_region_encoding() {
        let codes = morton_encode_region_3d(0, 0, 0, 1, 1, 1);
        assert_eq!(codes.len(), 8, "2x2x2 region should have 8 points");
        
        // All codes should be unique
        let unique: std::collections::HashSet<_> = codes.iter().collect();
        assert_eq!(unique.len(), 8, "All Morton codes should be unique");
    }
}

