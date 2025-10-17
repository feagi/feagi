/*!
Pattern-based connectivity - wildcard matching and transformations.
*/

use crate::types::Position;

type Dimensions = (usize, usize, usize);

/// Pattern element types
#[derive(Debug, Clone, PartialEq)]
pub enum PatternElement {
    Wildcard,        // "*" - matches any coordinate
    Skip,            // "?" - pass through source coordinate
    Exclude,         // "!" - exclude source coordinate
    Exact(i32),      // Exact integer match
}

impl PatternElement {
    pub fn from_value(value: &str) -> Self {
        match value {
            "*" => PatternElement::Wildcard,
            "?" => PatternElement::Skip,
            "!" => PatternElement::Exclude,
            _ => {
                if let Ok(num) = value.parse::<i32>() {
                    PatternElement::Exact(num)
                } else {
                    PatternElement::Wildcard // Default to wildcard for invalid
                }
            }
        }
    }
    
    pub fn from_int(value: i32) -> Self {
        if value == -1 {
            PatternElement::Wildcard
        } else if value == -2 {
            PatternElement::Skip
        } else if value == -3 {
            PatternElement::Exclude
        } else {
            PatternElement::Exact(value)
        }
    }
}

/// 3D pattern (x, y, z)
pub type Pattern3D = (PatternElement, PatternElement, PatternElement);

/// Match a coordinate against a pattern element
pub fn match_pattern_element(element: &PatternElement, coordinate: i32, src_coord: i32) -> bool {
    match element {
        PatternElement::Wildcard => true,
        PatternElement::Skip => coordinate == src_coord,
        PatternElement::Exclude => coordinate != src_coord,
        PatternElement::Exact(val) => coordinate == *val,
    }
}

/// Generate destination coordinates from pattern matching
pub fn find_destination_coordinates(
    dst_dimensions: Dimensions,
    src_coordinate: Position,
    _src_pattern: &Pattern3D,
    dst_pattern: &Pattern3D,
) -> Vec<Position> {
    let mut results = Vec::new();
    
    let (dst_width, dst_height, dst_depth) = dst_dimensions;
    let (src_x, src_y, src_z) = src_coordinate;
    
    // Generate ranges based on destination pattern
    let x_range: Vec<i32> = match &dst_pattern.0 {
        PatternElement::Wildcard => (0..dst_width as i32).collect(),
        PatternElement::Skip => {
            if src_x < dst_width as i32 {
                vec![src_x]
            } else {
                vec![]
            }
        }
        PatternElement::Exclude => {
            (0..dst_width as i32).filter(|&x| x != src_x).collect()
        }
        PatternElement::Exact(val) => {
            if *val >= 0 && (*val as usize) < dst_width {
                vec![*val]
            } else {
                vec![]
            }
        }
    };
    
    let y_range: Vec<i32> = match &dst_pattern.1 {
        PatternElement::Wildcard => (0..dst_height as i32).collect(),
        PatternElement::Skip => {
            if src_y < dst_height as i32 {
                vec![src_y]
            } else {
                vec![]
            }
        }
        PatternElement::Exclude => {
            (0..dst_height as i32).filter(|&y| y != src_y).collect()
        }
        PatternElement::Exact(val) => {
            if *val >= 0 && (*val as usize) < dst_height {
                vec![*val]
            } else {
                vec![]
            }
        }
    };
    
    let z_range: Vec<i32> = match &dst_pattern.2 {
        PatternElement::Wildcard => (0..dst_depth as i32).collect(),
        PatternElement::Skip => {
            if src_z < dst_depth as i32 {
                vec![src_z]
            } else {
                vec![]
            }
        }
        PatternElement::Exclude => {
            (0..dst_depth as i32).filter(|&z| z != src_z).collect()
        }
        PatternElement::Exact(val) => {
            if *val >= 0 && (*val as usize) < dst_depth {
                vec![*val]
            } else {
                vec![]
            }
        }
    };
    
    // Generate all combinations
    for x in &x_range {
        for y in &y_range {
            for z in &z_range {
                results.push((*x, *y, *z));
            }
        }
    }
    
    results
}

/// Find source coordinates that match a pattern
pub fn find_source_coordinates(
    src_pattern: &Pattern3D,
    src_dimensions: Dimensions,
) -> Vec<Position> {
    let mut results = Vec::new();
    
    let (src_width, src_height, src_depth) = src_dimensions;
    
    // Generate ranges based on source pattern
    let x_range: Vec<i32> = match &src_pattern.0 {
        PatternElement::Wildcard => (0..src_width as i32).collect(),
        PatternElement::Exact(val) => {
            if *val >= 0 && (*val as usize) < src_width {
                vec![*val]
            } else {
                vec![]
            }
        }
        _ => (0..src_width as i32).collect(), // Skip/Exclude treated as wildcard for source
    };
    
    let y_range: Vec<i32> = match &src_pattern.1 {
        PatternElement::Wildcard => (0..src_height as i32).collect(),
        PatternElement::Exact(val) => {
            if *val >= 0 && (*val as usize) < src_height {
                vec![*val]
            } else {
                vec![]
            }
        }
        _ => (0..src_height as i32).collect(),
    };
    
    let z_range: Vec<i32> = match &src_pattern.2 {
        PatternElement::Wildcard => (0..src_depth as i32).collect(),
        PatternElement::Exact(val) => {
            if *val >= 0 && (*val as usize) < src_depth {
                vec![*val]
            } else {
                vec![]
            }
        }
        _ => (0..src_depth as i32).collect(),
    };
    
    // Generate all combinations
    for x in &x_range {
        for y in &y_range {
            for z in &z_range {
                results.push((*x, *y, *z));
            }
        }
    }
    
    results
}

/// Batch process pattern matching for multiple patterns
pub fn match_patterns_batch(
    src_coordinate: Position,
    patterns: &[(Pattern3D, Pattern3D)], // (src_pattern, dst_pattern) pairs
    _src_dimensions: Dimensions,
    dst_dimensions: Dimensions,
) -> Vec<Position> {
    let mut all_results = Vec::new();
    
    for (src_pattern, dst_pattern) in patterns {
        // Check if source coordinate matches source pattern
        let (src_x, src_y, src_z) = src_coordinate;
        
        let x_match = match &src_pattern.0 {
            PatternElement::Wildcard => true,
            PatternElement::Exact(val) => src_x == *val,
            _ => true, // Skip/Exclude don't filter source
        };
        
        let y_match = match &src_pattern.1 {
            PatternElement::Wildcard => true,
            PatternElement::Exact(val) => src_y == *val,
            _ => true,
        };
        
        let z_match = match &src_pattern.2 {
            PatternElement::Wildcard => true,
            PatternElement::Exact(val) => src_z == *val,
            _ => true,
        };
        
        if x_match && y_match && z_match {
            let mut results = find_destination_coordinates(
                dst_dimensions,
                src_coordinate,
                src_pattern,
                dst_pattern,
            );
            all_results.append(&mut results);
        }
    }
    
    // Remove duplicates
    all_results.sort_unstable();
    all_results.dedup();
    
    all_results
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wildcard_pattern() {
        let src_pattern = (
            PatternElement::Wildcard,
            PatternElement::Wildcard,
            PatternElement::Exact(0),
        );
        let dst_pattern = (
            PatternElement::Skip,
            PatternElement::Skip,
            PatternElement::Exact(1),
        );
        
        let results = find_destination_coordinates(
            (10, 10, 10),
            (5, 5, 0),
            &src_pattern,
            &dst_pattern,
        );
        
        assert_eq!(results.len(), 1);
        assert_eq!(results[0], (5, 5, 1)); // Pass through x,y, set z=1
    }

    #[test]
    fn test_exact_pattern() {
        let src_pattern = (
            PatternElement::Exact(0),
            PatternElement::Exact(0),
            PatternElement::Exact(0),
        );
        let dst_pattern = (
            PatternElement::Exact(1),
            PatternElement::Exact(2),
            PatternElement::Exact(3),
        );
        
        let results = find_destination_coordinates(
            (10, 10, 10),
            (0, 0, 0),
            &src_pattern,
            &dst_pattern,
        );
        
        assert_eq!(results.len(), 1);
        assert_eq!(results[0], (1, 2, 3));
    }

    #[test]
    fn test_exclude_pattern() {
        let src_pattern = (
            PatternElement::Wildcard,
            PatternElement::Wildcard,
            PatternElement::Wildcard,
        );
        let dst_pattern = (
            PatternElement::Exclude,
            PatternElement::Exact(0),
            PatternElement::Exact(0),
        );
        
        let results = find_destination_coordinates(
            (3, 1, 1),
            (1, 0, 0),
            &src_pattern,
            &dst_pattern,
        );
        
        // Should match x=0 and x=2 (excluding x=1)
        assert_eq!(results.len(), 2);
        assert!(results.contains(&(0, 0, 0)));
        assert!(results.contains(&(2, 0, 0)));
    }
}

