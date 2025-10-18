/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # Type 11 Cortical Format Decoder
//!
//! Decodes sensory data encoded by feagi_data_serialization crate.
//!
//! Format: Type 11 = Neuron Categories (cortical areas with coordinates)
//!
//! Binary structure (little-endian):
//! ```text
//! [type:u8=11][area_count:u16][
//!   [area_id:6bytes][neuron_count:u32][
//!     x_coords:[u32; neuron_count]
//!     y_coords:[u32; neuron_count]
//!     z_coords:[u32; neuron_count]
//!     potentials:[f32; neuron_count]  // Optional, may be zeros
//!   ]
//!   ... more areas ...
//! ]]
//! ```

use std::io::{self, Cursor, Read};

/// Decoded cortical area data
#[derive(Debug, Clone)]
pub struct CorticalAreaData {
    /// Cortical area ID (e.g., "iic300")
    pub area_id: String,
    /// X coordinates
    pub coords_x: Vec<u32>,
    /// Y coordinates
    pub coords_y: Vec<u32>,
    /// Z coordinates
    pub coords_z: Vec<u32>,
    /// Membrane potentials (may be empty if not provided)
    pub potentials: Vec<f32>,
}

/// Decoded sensory data (Type 11 format)
#[derive(Debug, Clone)]
pub struct SensoryData {
    /// Cortical areas with their neuron coordinates
    pub areas: Vec<CorticalAreaData>,
}

/// Decode Type 11 cortical format from bytes
pub fn decode_type11(data: &[u8]) -> io::Result<SensoryData> {
    let mut cursor = Cursor::new(data);
    
    // Read structure type
    let mut type_byte = [0u8; 1];
    cursor.read_exact(&mut type_byte)?;
    if type_byte[0] != 11 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("Expected Type 11, got Type {}", type_byte[0]),
        ));
    }
    
    // Read area count (u16 little-endian)
    let mut area_count_bytes = [0u8; 2];
    cursor.read_exact(&mut area_count_bytes)?;
    let area_count = u16::from_le_bytes(area_count_bytes) as usize;
    
    let mut areas = Vec::with_capacity(area_count);
    
    for _ in 0..area_count {
        // Read area ID (6 bytes ASCII)
        let mut area_id_bytes = [0u8; 6];
        cursor.read_exact(&mut area_id_bytes)?;
        let area_id = String::from_utf8_lossy(&area_id_bytes).trim_end_matches('\0').to_string();
        
        // Read neuron count (u32 little-endian)
        let mut neuron_count_bytes = [0u8; 4];
        cursor.read_exact(&mut neuron_count_bytes)?;
        let neuron_count = u32::from_le_bytes(neuron_count_bytes) as usize;
        
        // Read X coordinates
        let mut coords_x = Vec::with_capacity(neuron_count);
        for _ in 0..neuron_count {
            let mut coord_bytes = [0u8; 4];
            cursor.read_exact(&mut coord_bytes)?;
            coords_x.push(u32::from_le_bytes(coord_bytes));
        }
        
        // Read Y coordinates
        let mut coords_y = Vec::with_capacity(neuron_count);
        for _ in 0..neuron_count {
            let mut coord_bytes = [0u8; 4];
            cursor.read_exact(&mut coord_bytes)?;
            coords_y.push(u32::from_le_bytes(coord_bytes));
        }
        
        // Read Z coordinates
        let mut coords_z = Vec::with_capacity(neuron_count);
        for _ in 0..neuron_count {
            let mut coord_bytes = [0u8; 4];
            cursor.read_exact(&mut coord_bytes)?;
            coords_z.push(u32::from_le_bytes(coord_bytes));
        }
        
        // Read potentials (f32 little-endian) - may be zeros or missing
        let mut potentials = Vec::with_capacity(neuron_count);
        for _ in 0..neuron_count {
            let mut potential_bytes = [0u8; 4];
            // If EOF, assume potentials not provided (older format)
            if cursor.read_exact(&mut potential_bytes).is_ok() {
                potentials.push(f32::from_le_bytes(potential_bytes));
            } else {
                // Fill remaining with zeros
                potentials.push(0.0);
            }
        }
        
        areas.push(CorticalAreaData {
            area_id,
            coords_x,
            coords_y,
            coords_z,
            potentials,
        });
    }
    
    Ok(SensoryData { areas })
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_decode_type11_single_area() {
        // Construct a minimal Type 11 payload:
        // type=11, area_count=1, area_id="iic300", neuron_count=2,
        // x=[10,20], y=[30,40], z=[0,1], potentials=[0.5,0.8]
        let mut data = Vec::new();
        data.push(11u8);  // Type
        data.extend_from_slice(&1u16.to_le_bytes());  // area_count
        data.extend_from_slice(b"iic300");  // area_id (6 bytes)
        data.extend_from_slice(&2u32.to_le_bytes());  // neuron_count
        data.extend_from_slice(&10u32.to_le_bytes());  // x[0]
        data.extend_from_slice(&20u32.to_le_bytes());  // x[1]
        data.extend_from_slice(&30u32.to_le_bytes());  // y[0]
        data.extend_from_slice(&40u32.to_le_bytes());  // y[1]
        data.extend_from_slice(&0u32.to_le_bytes());   // z[0]
        data.extend_from_slice(&1u32.to_le_bytes());   // z[1]
        data.extend_from_slice(&0.5f32.to_le_bytes()); // potentials[0]
        data.extend_from_slice(&0.8f32.to_le_bytes()); // potentials[1]
        
        let decoded = decode_type11(&data).unwrap();
        assert_eq!(decoded.areas.len(), 1);
        
        let area = &decoded.areas[0];
        assert_eq!(area.area_id, "iic300");
        assert_eq!(area.coords_x, vec![10, 20]);
        assert_eq!(area.coords_y, vec![30, 40]);
        assert_eq!(area.coords_z, vec![0, 1]);
        assert_eq!(area.potentials, vec![0.5, 0.8]);
    }
    
    #[test]
    fn test_decode_type11_wrong_type() {
        let data = vec![12u8];  // Wrong type
        let result = decode_type11(&data);
        assert!(result.is_err());
    }
}

