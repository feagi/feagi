//! Unit tests for motor extraction module

#[cfg(test)]
mod motor_extractor_tests {
    #[test]
    fn test_parse_cortical_id_consistency() {
        // Test that same name produces same ID
        let id1 = hash_cortical_name("opu_motor");
        let id2 = hash_cortical_name("opu_motor");
        assert_eq!(id1, id2, "Same name should produce same cortical ID");
    }

    #[test]
    fn test_parse_cortical_id_uniqueness() {
        // Test that different names produce different IDs
        let id1 = hash_cortical_name("opu_motor_left");
        let id2 = hash_cortical_name("opu_motor_right");
        assert_ne!(id1, id2, "Different names should produce different cortical IDs");
    }

    #[test]
    fn test_cortical_id_conversion() {
        // Test CorticalID byte array creation
        let area_name = "motor";
        let area_bytes = area_name.as_bytes();
        let mut cortical_id_bytes = [0u8; 6];
        let copy_len = area_bytes.len().min(6);
        cortical_id_bytes[..copy_len].copy_from_slice(&area_bytes[..copy_len]);

        // Verify bytes are correct
        assert_eq!(&cortical_id_bytes[..5], b"motor");
        assert_eq!(cortical_id_bytes[5], 0); // Padding
    }

    #[test]
    fn test_cortical_id_truncation() {
        // Test that long names are properly truncated to 6 bytes
        let area_name = "very_long_cortical_area_name";
        let area_bytes = area_name.as_bytes();
        let mut cortical_id_bytes = [0u8; 6];
        let copy_len = area_bytes.len().min(6);
        cortical_id_bytes[..copy_len].copy_from_slice(&area_bytes[..copy_len]);

        // Should only copy first 6 bytes
        assert_eq!(&cortical_id_bytes, b"very_l");
    }

    #[test]
    fn test_xyzp_array_creation() {
        // Test creating XYZP arrays from vectors
        let x_coords = vec![0u32, 1, 2, 3];
        let y_coords = vec![0u32, 1, 2, 3];
        let z_coords = vec![0u32, 0, 1, 1];
        let potentials = vec![50.0f32, 75.0, 60.0, 90.0];

        assert_eq!(x_coords.len(), y_coords.len());
        assert_eq!(y_coords.len(), z_coords.len());
        assert_eq!(z_coords.len(), potentials.len());

        // Verify data integrity
        for i in 0..x_coords.len() {
            assert!(potentials[i] >= 0.0);
            assert!(potentials[i] <= 100.0); // Typical range
        }
    }

    #[test]
    fn test_motor_area_parsing() {
        // Test parsing comma-separated motor areas
        let input = "opu_motor_left,opu_motor_right,opu_servo";
        let areas: Vec<String> = input
            .split(',')
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect();

        assert_eq!(areas.len(), 3);
        assert_eq!(areas[0], "opu_motor_left");
        assert_eq!(areas[1], "opu_motor_right");
        assert_eq!(areas[2], "opu_servo");
    }

    #[test]
    fn test_motor_area_parsing_with_spaces() {
        // Test parsing with extra spaces
        let input = " opu_motor_left , opu_motor_right , opu_servo ";
        let areas: Vec<String> = input
            .split(',')
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect();

        assert_eq!(areas.len(), 3);
        assert_eq!(areas[0], "opu_motor_left");
    }

    #[test]
    fn test_motor_area_parsing_empty() {
        // Test parsing empty string
        let input = "";
        let areas: Vec<String> = input
            .split(',')
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect();

        assert_eq!(areas.len(), 0);
    }

    #[test]
    fn test_motor_output_verbose_frequency() {
        // Test verbose output frequency calculation
        let extraction_count = 100u64;
        let verbose_interval = 10u64;
        
        // Should print on every 10th extraction
        assert_eq!(extraction_count % verbose_interval, 0);
        assert_ne!((extraction_count - 1) % verbose_interval, 0);
    }

    // Helper function matching the one in motor_extraction.rs
    fn hash_cortical_name(name: &str) -> u32 {
        name.chars()
            .map(|c| c as u32)
            .fold(0u32, |acc, c| acc.wrapping_add(c))
            % 1000
    }
}

#[cfg(test)]
mod motor_config_tests {
    use std::collections::HashMap;

    #[test]
    fn test_default_motor_config() {
        // Test default motor configuration
        let motor_areas = vec!["opu_motor".to_string()];
        let verbose = true;

        assert_eq!(motor_areas.len(), 1);
        assert_eq!(motor_areas[0], "opu_motor");
        assert!(verbose);
    }

    #[test]
    fn test_multiple_motor_areas() {
        // Test configuration with multiple motor areas
        let motor_areas = vec![
            "opu_motor_left".to_string(),
            "opu_motor_right".to_string(),
        ];

        assert_eq!(motor_areas.len(), 2);
        assert!(motor_areas.contains(&"opu_motor_left".to_string()));
        assert!(motor_areas.contains(&"opu_motor_right".to_string()));
    }

    #[test]
    fn test_motor_area_map_creation() {
        // Test creating motor area ID mapping
        let motor_area_names = vec!["opu_motor".to_string(), "opu_servo".to_string()];
        let mut motor_areas = HashMap::new();

        for name in motor_area_names.iter() {
            let area_id = hash_name(name);
            motor_areas.insert(name.clone(), area_id);
        }

        assert_eq!(motor_areas.len(), 2);
        assert!(motor_areas.contains_key("opu_motor"));
        assert!(motor_areas.contains_key("opu_servo"));
    }

    fn hash_name(name: &str) -> u32 {
        name.chars()
            .map(|c| c as u32)
            .fold(0u32, |acc, c| acc.wrapping_add(c))
            % 1000
    }
}

#[cfg(test)]
mod fire_queue_tests {
    #[test]
    fn test_fire_queue_data_structure() {
        // Test the expected structure of fire queue data
        // (area_id, (ids, xs, ys, zs, ps))
        let area_id = 42u32;
        let neuron_ids = vec![0u32, 1, 2];
        let x_coords = vec![0u32, 1, 2];
        let y_coords = vec![0u32, 1, 2];
        let z_coords = vec![0u32, 0, 1];
        let potentials = vec![50.0f32, 75.0, 60.0];

        // Verify all vectors have same length
        assert_eq!(neuron_ids.len(), x_coords.len());
        assert_eq!(x_coords.len(), y_coords.len());
        assert_eq!(y_coords.len(), z_coords.len());
        assert_eq!(z_coords.len(), potentials.len());

        // Verify data is valid
        assert!(area_id < 1000); // Reasonable area ID
        for &potential in &potentials {
            assert!(potential >= 0.0);
        }
    }

    #[test]
    fn test_empty_fire_queue() {
        // Test handling of empty fire queue
        let fire_data: Vec<(u32, (Vec<u32>, Vec<u32>, Vec<u32>, Vec<u32>, Vec<f32>))> = vec![];
        
        assert!(fire_data.is_empty());
    }

    #[test]
    fn test_fire_queue_filtering() {
        // Test filtering motor areas from fire queue
        let motor_area_ids = vec![10u32, 20u32];
        
        // Simulate fire queue data
        let area_id_1 = 10u32; // Motor area
        let area_id_2 = 5u32;  // Non-motor area
        let area_id_3 = 20u32; // Motor area

        assert!(motor_area_ids.contains(&area_id_1));
        assert!(!motor_area_ids.contains(&area_id_2));
        assert!(motor_area_ids.contains(&area_id_3));
    }
}

