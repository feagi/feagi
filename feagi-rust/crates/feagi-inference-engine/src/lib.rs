//! FEAGI Inference Engine Library
//!
//! This module exports the core functionality of the inference engine
//! for testing and potential library use.

// Note: Video/image processing modules disabled - now handled by external agents via ZMQ
// pub mod sensory_injection;
pub mod motor_extraction;
// Note: zmq_transport module removed - functionality integrated into main.rs

// Re-export key types for convenience
// pub use sensory_injection::{SensoryInjector, SensoryConfig};
pub use motor_extraction::{MotorExtractor, MotorConfig};

