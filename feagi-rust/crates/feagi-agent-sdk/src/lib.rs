//! FEAGI Agent SDK - Rust client library for building FEAGI agents
//!
//! This SDK provides a production-ready client for building agents that connect to FEAGI.
//!
//! # Features
//! - Automatic registration with retry/backoff
//! - Background heartbeat for keepalive
//! - Reconnection logic
//! - Sensory data sending (ZMQ PUSH)
//! - Motor data receiving (ZMQ SUB)
//! - Thread-safe operations
//! - Graceful shutdown with deregistration
//!
//! # Quick Start
//!
//! ```ignore
//! use feagi_agent_sdk::{AgentClient, AgentConfig, AgentType};
//!
//! // Create configuration
//! let config = AgentConfig::new("my_camera", AgentType::Sensory)
//!     .with_feagi_host("localhost")
//!     .with_vision_capability("camera", (640, 480), 3, "i_vision")
//!     .with_heartbeat_interval(5.0);
//!
//! // Create and connect client
//! let mut client = AgentClient::new(config)?;
//! client.connect()?;
//!
//! // Send sensory data
//! client.send_sensory_data(vec![
//!     (0, 50.0),   // neuron_id, potential
//!     (1, 75.0),
//!     (2, 30.0),
//! ])?;
//!
//! // For motor agents - receive motor commands
//! if let Some(motor_data) = client.receive_motor_data()? {
//!     println!("Motor command: {:?}", motor_data);
//! }
//!
//! // Client automatically deregisters on drop
//! ```
//!
//! # Architecture
//!
//! The SDK uses ZMQ for communication with FEAGI:
//! - **Registration**: ZMQ REQ/REP socket (also used for heartbeat)
//! - **Sensory Data**: ZMQ PUSH socket (agent → FEAGI)
//! - **Motor Data**: ZMQ SUB socket (FEAGI → agent)
//!
//! Heartbeat runs in a background thread and automatically keeps the agent alive.
//!
//! # Error Handling
//!
//! All operations return `Result<T, SdkError>`. The SDK distinguishes between:
//! - **Retryable errors**: Network issues, timeouts (handled by retry logic)
//! - **Non-retryable errors**: Configuration errors, invalid data
//!
//! # Thread Safety
//!
//! `AgentClient` uses `Arc<Mutex<>>` internally for socket sharing between
//! the main thread and heartbeat thread. All public methods are safe to call
//! from multiple threads.

pub mod client;
pub mod config;
pub mod error;
pub mod heartbeat;
pub mod reconnect;

// Re-export main types for convenience
pub use client::AgentClient;
pub use config::AgentConfig;
pub use error::{Result, SdkError};

// Re-export types from feagi-pns
pub use feagi_pns::agent_registry::{
    AgentType,
    AgentCapabilities,
    VisionCapability,
    MotorCapability,
};

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_sdk_imports() {
        // Verify all main types are accessible
        let _config = AgentConfig::new("test", AgentType::Sensory);
    }
}

