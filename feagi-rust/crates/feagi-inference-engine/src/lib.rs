//! FEAGI Inference Engine Library
//!
//! This module exports the core functionality of the inference engine
//! for testing and potential library use.

pub mod video_reader;
pub mod sensory_injection;
pub mod motor_extraction;

// Re-export key types for convenience
pub use video_reader::{VideoReader, VideoLoopConfig};
pub use sensory_injection::{SensoryInjector, SensoryConfig};
pub use motor_extraction::{MotorExtractor, MotorConfig};

