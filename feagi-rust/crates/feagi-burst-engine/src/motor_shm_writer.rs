/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! Motor SHM Writer
//!
//! Writes motor output data to shared memory for motor agents.
//! Uses ring buffer format matching Python's _ShmRingWriter for compatibility.
//!
//! Architecture:
//! - Rust Burst Loop → Motor Data → Motor SHM Writer → Agents read directly
//! - NO Python in hot path!
//!
//! Format (matches Python _ShmRingWriter / FEAGIMOT):
//! ```
//! Header (256 bytes):
//!   [0:8]    Magic number "FEAGIMOT" (8 bytes ASCII)
//!   [8:12]   Version (u32)
//!   [12:16]  Num slots (u32)
//!   [16:20]  Slot size (u32)
//!   [20:28]  Frame sequence (u64, increments per write)
//!   [28:32]  Write index (u32, current slot)
//!   [32:256] Padding (zeros)
//! 
//! Then N slots (default 64), each slot_size bytes (default 1MB):
//!   [0:4]    Payload length (u32)
//!   [4:...]  Payload data (binary motor data)
//!   [...end] Padding (zeros to fill slot)
//! ```

use std::path::PathBuf;
use std::fs::OpenOptions;
#[cfg(unix)]
use std::os::unix::fs::OpenOptionsExt;
use memmap2::MmapMut;

const MAGIC: &[u8; 8] = b"FEAGIMOT"; // Ring buffer magic (motor agents expect this!)
const VERSION: u32 = 1;
const HEADER_SIZE: usize = 256;
const DEFAULT_NUM_SLOTS: u32 = 64;
const DEFAULT_SLOT_SIZE: usize = 1 * 1024 * 1024; // 1 MB per slot

/// Motor SHM Writer (Ring Buffer Format)
pub struct MotorSHMWriter {
    /// SHM file path (kept for future debugging/logging during migration)
    /// Warning about unused field is expected - will be used for error messages
    shm_path: PathBuf,
    
    /// Memory-mapped file
    mmap: Option<MmapMut>,
    
    /// Ring buffer configuration
    num_slots: u32,
    slot_size: usize,
    
    /// Current write state
    frame_seq: u64,
    write_index: u32,
    
    /// Statistics
    total_writes: u64,
    
    /// Enable/disable writes
    enabled: bool,
}

impl MotorSHMWriter {
    /// Create a new motor SHM writer with ring buffer format
    ///
    /// # Arguments
    /// * `shm_path` - Path to the SHM file
    /// * `num_slots` - Number of ring buffer slots (default: 64)
    /// * `slot_size` - Size of each slot in bytes (default: 1MB)
    pub fn new(shm_path: PathBuf, num_slots: Option<u32>, slot_size: Option<usize>) -> Result<Self, std::io::Error> {
        let num_slots = num_slots.unwrap_or(DEFAULT_NUM_SLOTS);
        let slot_size = slot_size.unwrap_or(DEFAULT_SLOT_SIZE);
        let total_size = HEADER_SIZE + (num_slots as usize * slot_size);
        
        // Ensure parent directory exists
        if let Some(parent) = shm_path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        
        // Open/create SHM file with proper permissions
        #[cfg(unix)]
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .mode(0o600) // Unix permissions
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
        
        // Zero out rest of header
        mmap[32..HEADER_SIZE].fill(0);
        
        // Flush to disk
        mmap.flush()?;
        
        println!("✅ Created Motor SHM Writer: {:?} (FEAGIMOT ring buffer: {} slots x {} bytes = {} MB)",
            shm_path, num_slots, slot_size, total_size / (1024 * 1024));
        
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
    
    /// Write a payload to the ring buffer
    ///
    /// The payload will be truncated if it exceeds slot_size - 4 bytes.
    pub fn write_payload(&mut self, payload: &[u8]) -> Result<(), std::io::Error> {
        if !self.enabled {
            return Ok(());
        }
        
        // Truncate if needed (4 bytes reserved for length prefix)
        if payload.len() + 4 > self.slot_size {
            eprintln!("[MOTOR-SHM] Warning: payload {} bytes exceeds slot size {} bytes, truncating", 
                payload.len(), self.slot_size);
            let truncated = &payload[0..(self.slot_size - 4)];
            self.write_to_ring_slot(truncated)?;
        } else {
            self.write_to_ring_slot(payload)?;
        }
        
        Ok(())
    }
    
    /// Internal: Write payload to the current ring slot
    fn write_to_ring_slot(&mut self, payload: &[u8]) -> Result<(), std::io::Error> {
        let mmap = self.mmap.as_mut().ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::Other, "SHM not mapped")
        })?;
        
        // Calculate slot offset
        let slot_offset = HEADER_SIZE + (self.write_index as usize * self.slot_size);
        
        // Write length prefix (u32 LE)
        let length = payload.len() as u32;
        mmap[slot_offset..(slot_offset + 4)].copy_from_slice(&length.to_le_bytes());
        
        // Write payload
        mmap[(slot_offset + 4)..(slot_offset + 4 + payload.len())].copy_from_slice(payload);
        
        // Zero out remaining slot space
        let rem = self.slot_size - 4 - payload.len();
        if rem > 0 {
            mmap[(slot_offset + 4 + payload.len())..(slot_offset + self.slot_size)].fill(0);
        }
        
        // Update frame sequence and write index
        self.frame_seq += 1;
        self.write_index = (self.write_index + 1) % self.num_slots;
        
        // Update header
        mmap[20..28].copy_from_slice(&self.frame_seq.to_le_bytes());
        mmap[28..32].copy_from_slice(&self.write_index.to_le_bytes());
        
        // Flush changes
        mmap.flush()?;
        
        self.total_writes += 1;
        Ok(())
    }
    
    /// Get writer statistics
    pub fn get_stats(&self) -> (u64, u64) {
        (self.frame_seq, self.total_writes)
    }
    
    /// Enable or disable writing
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }
    
    /// Check if writer is enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
}

impl Drop for MotorSHMWriter {
    fn drop(&mut self) {
        if let Some(mmap) = self.mmap.take() {
            let _ = mmap.flush();
        }
        println!("[MOTOR-SHM] Writer closed after {} frames written", self.frame_seq);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    
    #[test]
    fn test_motor_shm_writer_creation() {
        let temp_path = PathBuf::from("/tmp/test_motor_shm_writer.bin");
        let _ = fs::remove_file(&temp_path);
        
        let writer = MotorSHMWriter::new(temp_path.clone(), Some(4), Some(1024)).unwrap();
        
        // Verify file exists and has correct size
        let metadata = fs::metadata(&temp_path).unwrap();
        assert_eq!(metadata.len(), 256 + 4 * 1024); // header + 4 slots of 1024 bytes
        
        // Cleanup
        drop(writer);
        let _ = fs::remove_file(&temp_path);
    }
    
    #[test]
    fn test_motor_shm_writer_write() {
        let temp_path = PathBuf::from("/tmp/test_motor_shm_write.bin");
        let _ = fs::remove_file(&temp_path);
        
        let mut writer = MotorSHMWriter::new(temp_path.clone(), Some(4), Some(1024)).unwrap();
        
        // Write test data
        let test_data = b"Hello, Motor Agent!";
        writer.write_payload(test_data).unwrap();
        
        // Verify stats
        let (frame_seq, total_writes) = writer.get_stats();
        assert_eq!(frame_seq, 1);
        assert_eq!(total_writes, 1);
        
        // Cleanup
        drop(writer);
        let _ = fs::remove_file(&temp_path);
    }
}

