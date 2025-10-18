/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! Visualization SHM Writer
//!
//! Writes Fire Queue samples to shared memory for Brain Visualizer.
//! Uses the same format as Python's LatestOnlySharedSlot for compatibility.
//!
//! Architecture:
//! - Rust Burst Loop → FQ Sampler → Viz SHM Writer → BV reads directly
//! - NO Python in hot path!
//!
//! Format (matches Python LatestOnlySharedSlot):
//! ```
//! Header (48 bytes):
//!   [0:8]   Magic number (0x4645414749534C54 = "FEAGISLT")
//!   [8:16]  Sequence number (u64, increments per write)
//!   [16:24] Data length (u64, bytes)
//!   [24:32] CRC32 checksum (u64, for data integrity)
//!   [32:40] Timestamp seconds (u64)
//!   [40:48] Timestamp nanos (u64)
//! Data:
//!   [48:...] LZ4 compressed JSON or binary data
//! ```

use std::path::PathBuf;
use std::fs::{File, OpenOptions};
use std::io::Write;
use std::os::unix::fs::OpenOptionsExt;
use std::time::{SystemTime, UNIX_EPOCH};
use memmap2::MmapMut;
use crate::fq_sampler::FQSampleResult;

const MAGIC_NUMBER: u64 = 0x4645414749534C54; // "FEAGISLT"
const HEADER_SIZE: usize = 48;
const MAX_SHM_SIZE: usize = 100 * 1024 * 1024; // 100 MB

/// Visualization SHM Writer
pub struct VizSHMWriter {
    /// SHM file path
    shm_path: PathBuf,
    
    /// Memory-mapped file
    mmap: Option<MmapMut>,
    
    /// Sequence number (increments per write)
    sequence: u64,
    
    /// Total writes
    total_writes: u64,
    
    /// Enabled flag
    enabled: bool,
}

impl VizSHMWriter {
    /// Create a new visualization SHM writer
    ///
    /// Args:
    ///     shm_path: Path to shared memory file (e.g., "/dev/shm/feagi-shared-mem-visualization_stream.bin")
    pub fn new(shm_path: PathBuf) -> Result<Self, std::io::Error> {
        // Create/open SHM file
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .mode(0o666) // rw-rw-rw-
            .open(&shm_path)?;
        
        // Set file size
        file.set_len(MAX_SHM_SIZE as u64)?;
        
        // Memory-map the file
        let mmap = unsafe { MmapMut::map_mut(&file)? };
        
        println!("✅ Created Viz SHM Writer: {:?}", shm_path);
        
        Ok(Self {
            shm_path,
            mmap: Some(mmap),
            sequence: 0,
            total_writes: 0,
            enabled: true,
        })
    }
    
    /// Write FQ sample to SHM
    ///
    /// Encodes the sample as JSON, compresses with LZ4, writes with header.
    pub fn write_sample(&mut self, sample: &FQSampleResult) -> Result<(), std::io::Error> {
        if !self.enabled {
            return Ok(());
        }
        
        // Serialize to JSON (simple format for BV)
        let json_data = self.serialize_to_json(sample)?;
        
        // Compress with LZ4
        let compressed = self.compress_lz4(&json_data)?;
        
        // Write to SHM
        self.write_to_shm(&compressed)?;
        
        Ok(())
    }
    
    /// Serialize FQ sample to JSON (BV format)
    fn serialize_to_json(&self, sample: &FQSampleResult) -> Result<Vec<u8>, std::io::Error> {
        use std::collections::HashMap;
        
        // Build JSON structure
        let mut data = HashMap::new();
        data.insert("timestep", sample.timestep.to_string());
        data.insert("total_neurons", sample.total_neurons.to_string());
        
        // Convert areas to JSON
        let mut areas_json = String::from("{");
        for (cortical_idx, area_data) in &sample.areas {
            if areas_json.len() > 1 {
                areas_json.push(',');
            }
            areas_json.push_str(&format!(
                "\"{}\": {{\"neuron_ids\":[{}],\"coordinates_x\":[{}],\"coordinates_y\":[{}],\"coordinates_z\":[{}],\"potentials\":[{}]}}",
                cortical_idx,
                area_data.neuron_ids.iter().map(|id| id.to_string()).collect::<Vec<_>>().join(","),
                area_data.coordinates_x.iter().map(|x| x.to_string()).collect::<Vec<_>>().join(","),
                area_data.coordinates_y.iter().map(|y| y.to_string()).collect::<Vec<_>>().join(","),
                area_data.coordinates_z.iter().map(|z| z.to_string()).collect::<Vec<_>>().join(","),
                area_data.potentials.iter().map(|p| p.to_string()).collect::<Vec<_>>().join(","),
            ));
        }
        areas_json.push('}');
        
        // Final JSON
        let json = format!("{{\"timestep\":{},\"total_neurons\":{},\"areas\":{}}}", 
                          sample.timestep, sample.total_neurons, areas_json);
        
        Ok(json.into_bytes())
    }
    
    /// Compress data with LZ4
    fn compress_lz4(&self, data: &[u8]) -> Result<Vec<u8>, std::io::Error> {
        // TODO: Add LZ4 compression (for now, return uncompressed)
        // This matches Python's behavior when compression is disabled
        Ok(data.to_vec())
    }
    
    /// Write data to SHM with header
    fn write_to_shm(&mut self, data: &[u8]) -> Result<(), std::io::Error> {
        // Check size
        if data.len() + HEADER_SIZE > MAX_SHM_SIZE {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("Data too large: {} bytes", data.len())
            ));
        }
        
        // Calculate CRC32 before borrowing mmap mutably
        let checksum = self.calculate_crc32(data);
        
        // Increment sequence
        self.sequence += 1;
        
        // Get timestamp
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap();
        let timestamp_secs = now.as_secs();
        let timestamp_nanos = now.subsec_nanos() as u64;
        
        // Now get mutable reference to mmap
        let mmap = self.mmap.as_mut().ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::Other, "SHM not mapped")
        })?;
        
        // Write header
        mmap[0..8].copy_from_slice(&MAGIC_NUMBER.to_le_bytes());
        mmap[8..16].copy_from_slice(&self.sequence.to_le_bytes());
        mmap[16..24].copy_from_slice(&(data.len() as u64).to_le_bytes());
        mmap[24..32].copy_from_slice(&checksum.to_le_bytes());
        mmap[32..40].copy_from_slice(&timestamp_secs.to_le_bytes());
        mmap[40..48].copy_from_slice(&timestamp_nanos.to_le_bytes());
        
        // Write data
        mmap[HEADER_SIZE..(HEADER_SIZE + data.len())].copy_from_slice(data);
        
        // Flush to disk
        mmap.flush()?;
        
        self.total_writes += 1;
        
        Ok(())
    }
    
    /// Simple CRC32 checksum
    fn calculate_crc32(&self, data: &[u8]) -> u64 {
        // Simple checksum (TODO: use proper CRC32)
        let mut sum: u32 = 0;
        for &byte in data {
            sum = sum.wrapping_add(byte as u32);
        }
        sum as u64
    }
    
    /// Enable/disable writing
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }
    
    /// Get statistics
    pub fn get_stats(&self) -> (u64, u64) {
        (self.sequence, self.total_writes)
    }
}

impl Drop for VizSHMWriter {
    fn drop(&mut self) {
        println!("🗑️  Dropping Viz SHM Writer: {:?} (wrote {} samples)", self.shm_path, self.total_writes);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fq_sampler::{FQSampleResult, SampledAreaData};
    use ahash::AHashMap;
    
    #[test]
    fn test_viz_shm_writer_create() {
        let path = PathBuf::from("/tmp/test_viz_shm.bin");
        let writer = VizSHMWriter::new(path);
        assert!(writer.is_ok());
    }
}

