/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! Visualization SHM Writer
//!
//! Writes Fire Queue samples to shared memory for Brain Visualizer.
//! Uses ring buffer format matching Python's _ShmRingWriter for compatibility.
//!
//! Architecture:
//! - Rust Burst Loop → FQ Sampler → Viz SHM Writer → BV reads directly
//! - NO Python in hot path!
//!
//! Format (matches Python _ShmRingWriter / FEAGIVIS):
//! ```
//! Header (256 bytes):
//!   [0:8]    Magic number "FEAGIVIS" (8 bytes ASCII)
//!   [8:12]   Version (u32)
//!   [12:16]  Num slots (u32)
//!   [16:20]  Slot size (u32)
//!   [20:28]  Frame sequence (u64, increments per write)
//!   [28:32]  Write index (u32, current slot)
//!   [32:256] Padding (zeros)
//! 
//! Then N slots (default 64), each slot_size bytes (default 1MB):
//!   [0:4]    Payload length (u32)
//!   [4:...]  Payload data (binary neuron data)
//!   [...end] Padding (zeros to fill slot)
//! ```

use std::path::PathBuf;
use std::fs::OpenOptions;
use std::os::unix::fs::OpenOptionsExt;
use memmap2::MmapMut;
use crate::fq_sampler::FQSampleResult;

const MAGIC: &[u8; 8] = b"FEAGIVIS"; // Ring buffer magic (BV expects this!)
const VERSION: u32 = 1;
const HEADER_SIZE: usize = 256;
const DEFAULT_NUM_SLOTS: u32 = 64;
const DEFAULT_SLOT_SIZE: usize = 1 * 1024 * 1024; // 1 MB per slot

/// Visualization SHM Writer (Ring Buffer Format)
pub struct VizSHMWriter {
    /// SHM file path
    shm_path: PathBuf,
    
    /// Memory-mapped file
    mmap: Option<MmapMut>,
    
    /// Number of ring buffer slots
    num_slots: u32,
    
    /// Size of each slot (bytes)
    slot_size: usize,
    
    /// Frame sequence number (increments per write)
    frame_seq: u64,
    
    /// Current write index (0..num_slots-1)
    write_index: u32,
    
    /// Total writes
    total_writes: u64,
    
    /// Enabled flag
    enabled: bool,
}

impl VizSHMWriter {
    /// Create a new visualization SHM writer with ring buffer format
    ///
    /// Args:
    ///     shm_path: Path to shared memory file (e.g., "/tmp/feagi-shared-mem-visualization_stream.bin")
    ///     num_slots: Number of ring buffer slots (default: 64)
    ///     slot_size: Size of each slot in bytes (default: 1MB)
    pub fn new(shm_path: PathBuf, num_slots: Option<u32>, slot_size: Option<usize>) -> Result<Self, std::io::Error> {
        let num_slots = num_slots.unwrap_or(DEFAULT_NUM_SLOTS);
        let slot_size = slot_size.unwrap_or(DEFAULT_SLOT_SIZE);
        
        // Calculate total size: header + (num_slots * slot_size)
        let total_size = HEADER_SIZE + (num_slots as usize * slot_size);
        
        // Create/open SHM file
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .mode(0o666) // rw-rw-rw-
            .open(&shm_path)?;
        
        // Set file size
        file.set_len(total_size as u64)?;
        
        // Memory-map the file
        let mut mmap = unsafe { MmapMut::map_mut(&file)? };
        
        // Initialize header
        mmap[0..8].copy_from_slice(MAGIC);
        mmap[8..12].copy_from_slice(&VERSION.to_le_bytes());
        mmap[12..16].copy_from_slice(&num_slots.to_le_bytes());
        mmap[16..20].copy_from_slice(&(slot_size as u32).to_le_bytes());
        mmap[20..28].copy_from_slice(&0u64.to_le_bytes()); // frame_seq
        mmap[28..32].copy_from_slice(&0u32.to_le_bytes()); // write_index
        // Padding (32..256) is already zeroed by file.set_len()
        
        mmap.flush()?;
        
        println!("✅ Created Viz SHM Writer: {:?} (FEAGIVIS ring buffer: {} slots x {} bytes = {} MB)", 
            shm_path, num_slots, slot_size, total_size / 1024 / 1024);
        
        Ok(Self {
            shm_path,
            mmap: Some(mmap),
            num_slots,
            slot_size,
            frame_seq: 0,
            write_index: 0,
            total_writes: 0,
            enabled: true,
        })
    }
    
    /// Write FQ sample to SHM ring buffer
    ///
    /// Encodes the sample as binary (Type 11), writes to next slot, updates header.
    pub fn write_sample(&mut self, sample: &FQSampleResult, binary_data: &[u8]) -> Result<(), std::io::Error> {
        if !self.enabled {
            return Ok(());
        }
        
        // Check payload size
        if binary_data.len() + 4 > self.slot_size {
            // Truncate if too large (should not happen with proper encoding)
            eprintln!("[VIZ-SHM] Warning: payload {} bytes exceeds slot size {} bytes, truncating", 
                binary_data.len(), self.slot_size);
            let truncated = &binary_data[0..(self.slot_size - 4)];
            self.write_to_ring_slot(truncated)?;
        } else {
            self.write_to_ring_slot(binary_data)?;
        }
        
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

