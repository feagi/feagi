/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # FEAGI Connectome Serialization
//!
//! Serialization and deserialization of complete brain connectomes for the FEAGI inference engine.
//!
//! ## Design Goals
//! - **Fast**: Binary format optimized for speed
//! - **Compact**: Optional compression for storage/network transfer
//! - **Complete**: Captures entire NPU state including runtime data
//! - **Version-safe**: Format versioning for backward compatibility
//!
//! ## Usage
//! ```ignore
//! use feagi_connectome_serialization::{save_connectome, load_connectome};
//! use feagi_burst_engine::RustNPU;
//!
//! // Save connectome
//! let npu = RustNPU::new(10000, 100000, 20);
//! save_connectome(&npu, "brain.connectome")?;
//!
//! // Load connectome
//! let npu = load_connectome("brain.connectome")?;
//! ```

use serde::{Deserialize, Serialize};
use ahash::AHashMap;
use std::fs::File;
use std::io::{Read, Write};
use std::path::Path;
use thiserror::Error;

mod format;
mod neuron_array;
mod synapse_array;

pub use format::*;
pub use neuron_array::*;
pub use synapse_array::*;

/// Connectome I/O errors
#[derive(Error, Debug)]
pub enum ConnectomeError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Serialization error: {0}")]
    Serialization(String),

    #[error("Deserialization error: {0}")]
    Deserialization(String),

    #[error("Version mismatch: file version {file_version}, expected {expected_version}")]
    VersionMismatch {
        file_version: u32,
        expected_version: u32,
    },

    #[error("Invalid magic number: expected FEAGI, got {0:?}")]
    InvalidMagic([u8; 5]),

    #[error("Checksum mismatch: file may be corrupted")]
    ChecksumMismatch,

    #[error("Compression error: {0}")]
    Compression(String),
}

pub type Result<T> = std::result::Result<T, ConnectomeError>;

/// Magic number for connectome files: "FEAGI"
const MAGIC: &[u8; 5] = b"FEAGI";

/// Current format version (increment when format changes)
const FORMAT_VERSION: u32 = 1;

/// Complete connectome snapshot
///
/// This structure captures the entire state of a RustNPU, including:
/// - All neurons and their properties
/// - All synapses and their weights
/// - Cortical area metadata
/// - Runtime state (burst count, etc.)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConnectomeSnapshot {
    /// Format version (for backward compatibility)
    pub version: u32,

    /// Neuron data
    pub neurons: SerializableNeuronArray,

    /// Synapse data
    pub synapses: SerializableSynapseArray,

    /// Cortical area ID to name mapping (for visualization)
    pub cortical_area_names: AHashMap<u32, String>,

    /// Burst count (runtime state)
    pub burst_count: u64,

    /// Power injection amount
    pub power_amount: f32,

    /// Fire ledger window size
    pub fire_ledger_window: usize,

    /// Metadata (optional, for debugging/tracking)
    pub metadata: ConnectomeMetadata,
}

/// Connectome metadata (for tracking and debugging)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConnectomeMetadata {
    /// When this connectome was saved
    pub timestamp: u64,

    /// Human-readable description
    pub description: String,

    /// Source (e.g., "genome: essential_genome.json", "checkpoint: burst_12345")
    pub source: String,

    /// Custom tags for organization
    pub tags: AHashMap<String, String>,
}

impl Default for ConnectomeMetadata {
    fn default() -> Self {
        Self {
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs(),
            description: String::new(),
            source: String::from("unknown"),
            tags: AHashMap::new(),
        }
    }
}

/// Save a connectome to a file
///
/// # Arguments
/// * `snapshot` - The connectome snapshot to save
/// * `path` - File path to write to
///
/// # Format
/// ```text
/// [Header]
/// - Magic: "FEAGI" (5 bytes)
/// - Version: u32 (4 bytes)
/// - Checksum: u64 (8 bytes, CRC64 of data)
/// [Data]
/// - Bincode-serialized ConnectomeSnapshot
/// ```
pub fn save_connectome<P: AsRef<Path>>(
    snapshot: &ConnectomeSnapshot,
    path: P,
) -> Result<()> {
    let mut file = File::create(path)?;

    // Write header
    file.write_all(MAGIC)?;
    file.write_all(&FORMAT_VERSION.to_le_bytes())?;

    // Serialize data
    let data = bincode::serialize(snapshot)
        .map_err(|e| ConnectomeError::Serialization(e.to_string()))?;

    // Calculate checksum (simple CRC64)
    let checksum = calculate_checksum(&data);
    file.write_all(&checksum.to_le_bytes())?;

    // Write data
    file.write_all(&data)?;

    Ok(())
}

/// Load a connectome from a file
///
/// # Arguments
/// * `path` - File path to read from
///
/// # Returns
/// The deserialized connectome snapshot
pub fn load_connectome<P: AsRef<Path>>(path: P) -> Result<ConnectomeSnapshot> {
    let mut file = File::open(path)?;

    // Read and verify magic number
    let mut magic = [0u8; 5];
    file.read_exact(&mut magic)?;
    if &magic != MAGIC {
        return Err(ConnectomeError::InvalidMagic(magic));
    }

    // Read and verify version
    let mut version_bytes = [0u8; 4];
    file.read_exact(&mut version_bytes)?;
    let version = u32::from_le_bytes(version_bytes);

    if version != FORMAT_VERSION {
        return Err(ConnectomeError::VersionMismatch {
            file_version: version,
            expected_version: FORMAT_VERSION,
        });
    }

    // Read checksum
    let mut checksum_bytes = [0u8; 8];
    file.read_exact(&mut checksum_bytes)?;
    let expected_checksum = u64::from_le_bytes(checksum_bytes);

    // Read data
    let mut data = Vec::new();
    file.read_to_end(&mut data)?;

    // Verify checksum
    let actual_checksum = calculate_checksum(&data);
    if actual_checksum != expected_checksum {
        return Err(ConnectomeError::ChecksumMismatch);
    }

    // Deserialize
    let snapshot: ConnectomeSnapshot = bincode::deserialize(&data)
        .map_err(|e| ConnectomeError::Deserialization(e.to_string()))?;

    Ok(snapshot)
}

/// Calculate a simple checksum (CRC64-like)
fn calculate_checksum(data: &[u8]) -> u64 {
    // Simple FNV-1a hash for now (can upgrade to proper CRC64 later)
    const FNV_OFFSET: u64 = 14695981039346656037;
    const FNV_PRIME: u64 = 1099511628211;

    let mut hash = FNV_OFFSET;
    for &byte in data {
        hash ^= byte as u64;
        hash = hash.wrapping_mul(FNV_PRIME);
    }
    hash
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::NamedTempFile;

    #[test]
    fn test_save_load_roundtrip() {
        // Create a minimal snapshot
        let snapshot = ConnectomeSnapshot {
            version: FORMAT_VERSION,
            neurons: SerializableNeuronArray::default(),
            synapses: SerializableSynapseArray::default(),
            cortical_area_names: AHashMap::new(),
            burst_count: 42,
            power_amount: 1.0,
            fire_ledger_window: 20,
            metadata: ConnectomeMetadata::default(),
        };

        // Save to temp file
        let temp_file = NamedTempFile::new().unwrap();
        save_connectome(&snapshot, temp_file.path()).unwrap();

        // Load back
        let loaded = load_connectome(temp_file.path()).unwrap();

        // Verify
        assert_eq!(loaded.version, snapshot.version);
        assert_eq!(loaded.burst_count, snapshot.burst_count);
        assert_eq!(loaded.power_amount, snapshot.power_amount);
    }

    #[test]
    fn test_invalid_magic() {
        let temp_file = NamedTempFile::new().unwrap();
        let mut file = File::create(temp_file.path()).unwrap();
        file.write_all(b"WRONG").unwrap();

        let result = load_connectome(temp_file.path());
        assert!(matches!(result, Err(ConnectomeError::InvalidMagic(_))));
    }

    #[test]
    fn test_checksum() {
        let data1 = b"hello world";
        let data2 = b"hello world";
        let data3 = b"hello worlD";

        assert_eq!(calculate_checksum(data1), calculate_checksum(data2));
        assert_ne!(calculate_checksum(data1), calculate_checksum(data3));
    }
}

