/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # Rust Sensory Injection System
//!
//! High-performance sensory data ingestion directly into the Rust NPU.
//!
//! ## Architecture
//! ```text
//! ┌──────────────────────────────────────────────────────────────┐
//! │ Video Agent (Python)                                          │
//! │  - Writes to SHM: /tmp/feagi-shm-{agent_id}-sensory.bin      │
//! └────────────────────┬─────────────────────────────────────────┘
//!                      │
//!                      ▼
//! ┌──────────────────────────────────────────────────────────────┐
//! │ Rust Sensory Engine (THIS MODULE)                            │
//! │                                                               │
//! │  Per-Agent Thread:                                            │
//! │  ┌────────────────────────────────────────────────────────┐  │
//! │  │ 1. SHM Reader (shm_reader.rs)                          │  │
//! │  │    - Polls /tmp/feagi-shm-* at agent's rate_hz         │  │
//! │  │    - Reads LatestOnlySharedSlot header + payload       │  │
//! │  ├────────────────────────────────────────────────────────┤  │
//! │  │ 2. Decoder (decoder.rs)                                │  │
//! │  │    - Decodes Type 11 cortical format                   │  │
//! │  │    - Extracts (area_id, coordinates)                   │  │
//! │  ├────────────────────────────────────────────────────────┤  │
//! │  │ 3. Coordinate → NeuronID lookup (NPU spatial hash)     │  │
//! │  │    - Batch lookup for efficiency                       │  │
//! │  ├────────────────────────────────────────────────────────┤  │
//! │  │ 4. FCL Injection                                       │  │
//! │  │    - Directly writes to Fire Candidate List            │  │
//! │  │    - Thread-safe via Arc<Mutex<FCL>>                   │  │
//! │  └────────────────────────────────────────────────────────┘  │
//! │                                                               │
//! │  Agent Manager (agent_manager.rs):                            │
//! │    - Spawns/stops threads on agent registration/deregistration│
//! │    - Manages thread pool (one thread per active agent)       │
//! │    - Rate limiting per agent (capability_rate_hz)            │
//! └───────────────────────────────────────────────────────────────┘
//!                      │
//!                      ▼
//! ┌──────────────────────────────────────────────────────────────┐
//! │ Rust NPU (npu.rs)                                             │
//! │  - FCL receives injected neurons                             │
//! │  - process_burst() processes them                            │
//! └──────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Design Principles
//! - **Zero Python involvement**: Sensory data never touches Python
//! - **Lock-free where possible**: Use Arc for shared state, minimize contention
//! - **RTOS-ready**: No allocations in hot path
//! - **Capability-aware**: Respects agent-requested rate_hz
//! - **Graceful lifecycle**: Clean shutdown on agent deregistration

mod shm_reader;
mod decoder;
mod agent_manager;
mod rate_limiter;

pub use shm_reader::*;
pub use decoder::*;
pub use agent_manager::*;
pub use rate_limiter::*;

