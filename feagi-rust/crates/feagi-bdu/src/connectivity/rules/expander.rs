/*!
Expander morphology - scales coordinates from source to destination.
*/

use crate::types::{BduResult, Position};

/// Expander mapping - scales coordinates by dimension ratios.
pub fn syn_expander(
    _src_area_id: &str,
    _dst_area_id: &str,
    neuron_location: Position,
    src_dimensions: (usize, usize, usize),
    dst_dimensions: (usize, usize, usize),
) -> BduResult<Position> {
    let (src_x, src_y, src_z) = src_dimensions;
    let (dst_x, dst_y, dst_z) = dst_dimensions;
    
    // Calculate expansion ratios
    let ratio_x = dst_x as f32 / src_x as f32;
    let ratio_y = dst_y as f32 / src_y as f32;
    let ratio_z = dst_z as f32 / src_z as f32;
    
    // Scale neuron position
    let (x, y, z) = neuron_location;
    let scaled_x = ((x as f32 * ratio_x) as usize).min(dst_x - 1) as i32;
    let scaled_y = ((y as f32 * ratio_y) as usize).min(dst_y - 1) as i32;
    let scaled_z = ((z as f32 * ratio_z) as usize).min(dst_z - 1) as i32;
    
    Ok((scaled_x, scaled_y, scaled_z))
}

/// Batch expander for parallel processing
pub fn syn_expander_batch(
    src_area_id: &str,
    dst_area_id: &str,
    neuron_locations: &[Position],
    src_dimensions: (usize, usize, usize),
    dst_dimensions: (usize, usize, usize),
) -> BduResult<Vec<Position>> {
    use rayon::prelude::*;
    
    neuron_locations.par_iter()
        .map(|&loc| syn_expander(src_area_id, dst_area_id, loc, src_dimensions, dst_dimensions))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_expander_scale_up() {
        let result = syn_expander(
            "src", "dst",
            (5, 5, 5),
            (10, 10, 10),
            (20, 20, 20)
        );
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), (10, 10, 10));
    }

    #[test]
    fn test_expander_scale_down() {
        let result = syn_expander(
            "src", "dst",
            (10, 10, 10),
            (20, 20, 20),
            (10, 10, 10)
        );
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), (5, 5, 5));
    }
}

