//! Transport-agnostic agent registration and management for FEAGI
//!
//! This crate provides core agent registry logic without coupling to any specific
//! transport mechanism (ZMQ, REST, gRPC, shared memory, etc.). Transports implement
//! the `AgentTransport` trait to adapt the registry to their communication method.

pub mod types;
pub mod registry;
pub mod transport;

pub use types::{AgentInfo, AgentType, AgentCapabilities, VisionCapability, MotorCapability, VisualizationCapability};
pub use registry::AgentRegistry;
pub use transport::{AgentTransport, TransportEndpoints};

#[derive(Debug, thiserror::Error)]
pub enum RegistryError {
    #[error("Agent already registered: {0}")]
    AgentAlreadyExists(String),
    
    #[error("Agent not found: {0}")]
    AgentNotFound(String),
    
    #[error("Invalid agent configuration: {0}")]
    InvalidConfiguration(String),
    
    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),
    
    #[error("Transport error: {0}")]
    TransportError(String),
}

pub type Result<T> = std::result::Result<T, RegistryError>;

