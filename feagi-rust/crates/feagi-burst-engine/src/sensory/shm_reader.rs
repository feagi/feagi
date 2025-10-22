/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # Shared Memory Reader for Latest-Only Slots
//!
//! Reads sensory data from SHM using the LatestOnlySharedSlot protocol.
//! Compatible with Python feagi/api/zmq/neural/latest_only_slot.py
//!
//! Note: SHM functionality is Unix-only. Windows uses ZMQ for agent communication.

#[cfg(unix)]
use std::fs::File;
#[cfg(unix)]
use std::os::unix::io::AsRawFd;
use std::path::{Path, PathBuf};
#[cfg(unix)]
use memmap2::MmapMut;
use std::io;

/// Latest-only slot protocol constants
const MAGIC: &[u8; 8] = b"FEAGILAT";
const VERSION: u32 = 1;
const HEADER_SIZE: usize = 256;

/// Shared memory header (matches Python struct format "<8sIIIQQII212s")
/// 
/// Migration status: SHM reader partially migrated from Python. This struct will be used
/// once the full SHM protocol implementation is completed.
/// Warning about unused struct is expected during migration.
#[repr(C, packed)]
struct ShmHeader {
    magic: [u8; 8],           // 8 bytes
    version: u32,             // 4 bytes  
    max_payload_size: u32,    // 4 bytes
    writer_pid: u32,          // 4 bytes
    timestamp_ns: u64,        // 8 bytes
    sequence: u64,            // 8 bytes
    payload_size: u32,        // 4 bytes
    reserved: u32,            // 4 bytes
    _padding: [u8; 212],      // 212 bytes (unused)
}

/// Data read from SHM slot
#[derive(Debug, Clone)]
pub struct SlotData {
    pub data: Vec<u8>,
    pub timestamp_ns: u64,
    pub sequence: u64,
    pub age_ms: f64,
}

/// Shared memory reader for latest-only sensory data
/// Only available on Unix systems (Linux, macOS)
#[cfg(unix)]
pub struct ShmReader {
    path: PathBuf,
    _file: File,
    mmap: MmapMut,
    last_sequence: u64,
}

/// Stub for Windows (SHM not supported)
#[cfg(not(unix))]
pub struct ShmReader {
    #[allow(dead_code)]
    path: PathBuf,
}

/// Windows stub implementation
#[cfg(not(unix))]
impl ShmReader {
    /// Open SHM - not supported on Windows
    pub fn open<P: AsRef<Path>>(_path: P) -> io::Result<Self> {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "Shared memory is not supported on Windows. Use ZMQ for agent communication."
        ))
    }
    
    /// Read latest - stub for Windows
    pub fn read_latest(&mut self) -> Option<SlotData> {
        None
    }
    
    /// Get path - stub for Windows
    pub fn path(&self) -> &Path {
        &self.path
    }
}

/// Unix implementation
#[cfg(unix)]
impl ShmReader {
    /// Open an existing SHM slot for reading
    pub fn open<P: AsRef<Path>>(path: P) -> io::Result<Self> {
        let path = path.as_ref().to_path_buf();
        
        // Open file (must already exist - created by writer)
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)  // Needed for mmap with PROT_READ|PROT_WRITE
            .open(&path)?;
        
        // Memory map
        let mmap = unsafe { MmapMut::map_mut(file.as_raw_fd())? };
        
        // Verify header
        if mmap.len() < HEADER_SIZE {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("SHM slot too small: {} bytes (need at least {})", mmap.len(), HEADER_SIZE),
            ));
        }
        
        // Read magic
        let magic_bytes = &mmap[0..8];
        if magic_bytes != MAGIC {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("Invalid magic: expected {:?}, got {:?}", MAGIC, magic_bytes),
            ));
        }
        
        Ok(Self {
            path,
            _file: file,
            mmap,
            last_sequence: 0,
        })
    }
    
    /// Read the latest data from the slot (non-blocking)
    /// Returns None if no new data since last read
    pub fn read_latest(&mut self) -> Option<SlotData> {
        // Read header (all at once for atomicity)
        let header_bytes = &self.mmap[0..HEADER_SIZE];
        
        // Parse header fields (little-endian)
        let magic = &header_bytes[0..8];
        if magic != MAGIC {
            return None;  // Invalid magic, slot not initialized
        }
        
        let version = u32::from_le_bytes([header_bytes[8], header_bytes[9], header_bytes[10], header_bytes[11]]);
        if version != VERSION {
            return None;  // Version mismatch
        }
        
        let max_payload_size = u32::from_le_bytes([header_bytes[12], header_bytes[13], header_bytes[14], header_bytes[15]]);
        let _writer_pid = u32::from_le_bytes([header_bytes[16], header_bytes[17], header_bytes[18], header_bytes[19]]);
        let timestamp_ns = u64::from_le_bytes([
            header_bytes[20], header_bytes[21], header_bytes[22], header_bytes[23],
            header_bytes[24], header_bytes[25], header_bytes[26], header_bytes[27],
        ]);
        let sequence = u64::from_le_bytes([
            header_bytes[28], header_bytes[29], header_bytes[30], header_bytes[31],
            header_bytes[32], header_bytes[33], header_bytes[34], header_bytes[35],
        ]);
        let payload_size = u32::from_le_bytes([header_bytes[36], header_bytes[37], header_bytes[38], header_bytes[39]]);
        
        // Check if this is new data
        if sequence <= self.last_sequence {
            return None;  // No new data
        }
        
        // Validate payload size
        if payload_size > max_payload_size {
            return None;  // Invalid payload size
        }
        
        if payload_size == 0 {
            return None;  // No payload
        }
        
        // Check if we have enough space
        let total_size = HEADER_SIZE + max_payload_size as usize;
        if self.mmap.len() < total_size {
            return None;  // Slot too small
        }
        
        // Read payload (copy to avoid holding lock)
        let payload_start = HEADER_SIZE;
        let payload_end = payload_start + payload_size as usize;
        let data = self.mmap[payload_start..payload_end].to_vec();
        
        // Calculate age
        let now_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
        let age_ms = (now_ns.saturating_sub(timestamp_ns)) as f64 / 1_000_000.0;
        
        // Update last sequence
        self.last_sequence = sequence;
        
        Some(SlotData {
            data,
            timestamp_ns,
            sequence,
            age_ms,
        })
    }
    
    /// Get the path to the SHM slot
    pub fn path(&self) -> &Path {
        &self.path
    }
}

impl Drop for ShmReader {
    fn drop(&mut self) {
        // mmap automatically unmaps on drop
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    fn test_shm_reader_invalid_magic() {
        // Create temporary file with invalid magic
        let tmp_file = std::env::temp_dir().join("test_shm_invalid_magic.bin");
        let mut file = File::create(&tmp_file).unwrap();
        
        // Write 256 bytes of zeros (invalid header)
        file.write_all(&[0u8; 256]).unwrap();
        file.sync_all().unwrap();
        drop(file);
        
        // Attempt to open should fail or read_latest should return None
        if let Ok(mut reader) = ShmReader::open(&tmp_file) {
            assert!(reader.read_latest().is_none());
        }
        
        // Cleanup
        let _ = std::fs::remove_file(&tmp_file);
    }
}

