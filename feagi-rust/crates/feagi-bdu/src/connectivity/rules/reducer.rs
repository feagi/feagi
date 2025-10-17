/*!
Reducer morphology - binary encoding/decoding for position mapping.
*/

use crate::types::{BduResult, Position};

/// Reducer mapping - maps source x position to multiple destination positions
/// based on binary representation.
pub fn syn_reducer_x(
    _src_area_id: &str,
    _dst_area_id: &str,
    neuron_location: Position,
    _src_dimensions: (usize, usize, usize),
    dst_dimensions: (usize, usize, usize),
    dst_y_index: i32,
    dst_z_index: i32,
) -> BduResult<Vec<Position>> {
    let (x, _y, _z) = neuron_location;
    let dst_x_dim = dst_dimensions.0;
    
    let mut positions = Vec::new();
    
    // Convert x to binary and map to destination positions
    for bit_pos in 0..dst_x_dim {
        if bit_pos < 32 {  // Safety check for bit shift
            let mask = 1 << bit_pos;
            if (x & mask) != 0 {
                positions.push((bit_pos as i32, dst_y_index, dst_z_index));
            }
        }
    }
    
    Ok(positions)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_reducer() {
        // Binary 5 = 101, should map to positions 0 and 2
        let result = syn_reducer_x(
            "src", "dst",
            (5, 0, 0),
            (10, 10, 10),
            (8, 1, 1),
            0, 0
        );
        assert!(result.is_ok());
        let positions = result.unwrap();
        assert!(positions.contains(&(0, 0, 0)));
        assert!(positions.contains(&(2, 0, 0)));
    }
}

