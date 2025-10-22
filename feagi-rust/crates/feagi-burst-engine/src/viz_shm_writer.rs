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
//!
//! Note: SHM functionality is Unix-only. Windows uses ZMQ for visualization streaming.

use std::path::PathBuf;
use std::fs::OpenOptions;
#[cfg(unix)]
use std::os::unix::fs::OpenOptionsExt;
use memmap2::MmapMut;

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
        #[cfg(unix)]
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .mode(0o666) // rw-rw-rw-
            .open(&shm_path)?;
        
        #[cfg(not(unix))]
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
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
    
    /// Write binary neuron data to SHM ring buffer
    ///
    /// Takes pre-encoded binary data (Type 11 format) and writes to next slot.
    pub fn write_payload(&mut self, payload: &[u8]) -> Result<(), std::io::Error> {
        if !self.enabled {
            return Ok(());
        }
        
        // Check payload size
        if payload.len() + 4 > self.slot_size {
            // Truncate if too large (should not happen with proper encoding)
            eprintln!("[VIZ-SHM] Warning: payload {} bytes exceeds slot size {} bytes, truncating", 
                payload.len(), self.slot_size);
            let truncated = &payload[0..(self.slot_size - 4)];
            self.write_to_ring_slot(truncated)?;
        } else {
            self.write_to_ring_slot(payload)?;
        }
        
        Ok(())
    }
    
    /// Write payload to current ring buffer slot
    fn write_to_ring_slot(&mut self, payload: &[u8]) -> Result<(), std::io::Error> {
        let mmap = self.mmap.as_mut().ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::Other, "SHM not mapped")
        })?;
        
        // Calculate slot offset: HEADER_SIZE + (write_index * slot_size)
        let slot_offset = HEADER_SIZE + (self.write_index as usize * self.slot_size);
        
        // Write slot: u32 length + payload + padding
        let length = payload.len() as u32;
        mmap[slot_offset..(slot_offset + 4)].copy_from_slice(&length.to_le_bytes());
        mmap[(slot_offset + 4)..(slot_offset + 4 + payload.len())].copy_from_slice(payload);
        
        // Pad remainder with zeros
        let rem = self.slot_size - 4 - payload.len();
        if rem > 0 {
            mmap[(slot_offset + 4 + payload.len())..(slot_offset + self.slot_size)].fill(0);
        }
        
        // Update counters
        self.frame_seq += 1;
        self.write_index = (self.write_index + 1) % self.num_slots;
        
        // Update header with new frame_seq and write_index
        mmap[20..28].copy_from_slice(&self.frame_seq.to_le_bytes());
        mmap[28..32].copy_from_slice(&self.write_index.to_le_bytes());
        
        // Flush (ensures BV sees the update)
        mmap.flush()?;
        
        self.total_writes += 1;
        
        Ok(())
    }
    
    /// Enable/disable writing
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }
    
    /// Get statistics
    pub fn get_stats(&self) -> (u64, u64) {
        (self.frame_seq, self.total_writes)
    }
}

impl Drop for VizSHMWriter {
    fn drop(&mut self) {
        println!("🗑️  Dropping Viz SHM Writer: {:?} (wrote {} frames)", self.shm_path, self.total_writes);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_viz_shm_writer_create() {
        let path = PathBuf::from("/tmp/test_viz_shm_ring.bin");
        let writer = VizSHMWriter::new(path, None, None);
        assert!(writer.is_ok());
        
        // Clean up
        std::fs::remove_file("/tmp/test_viz_shm_ring.bin").ok();
    }
    
    #[test]
    fn test_viz_shm_writer_write() {
        let path = PathBuf::from("/tmp/test_viz_shm_ring_write.bin");
        let mut writer = VizSHMWriter::new(path, Some(4), Some(1024)).unwrap();
        
        // Write some data
        let test_data = b"test neuron data";
        writer.write_payload(test_data).unwrap();
        
        assert_eq!(writer.frame_seq, 1);
        assert_eq!(writer.write_index, 1);
        assert_eq!(writer.total_writes, 1);
        
        // Clean up
        std::fs::remove_file("/tmp/test_viz_shm_ring_write.bin").ok();
    }
}
