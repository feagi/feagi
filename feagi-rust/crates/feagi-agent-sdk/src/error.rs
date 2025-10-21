//! Error types for FEAGI Agent SDK

/// Result type alias using SdkError
pub type Result<T> = std::result::Result<T, SdkError>;

/// Error types for the FEAGI Agent SDK
#[derive(Debug, thiserror::Error)]
pub enum SdkError {
    /// ZMQ communication error
    #[error("ZMQ error: {0}")]
    Zmq(#[from] zmq::Error),
    
    /// JSON serialization/deserialization error
    #[error("JSON error: {0}")]
    Json(#[from] serde_json::Error),
    
    /// Registration failed
    #[error("Registration failed: {0}")]
    RegistrationFailed(String),
    
    /// Agent not registered
    #[error("Agent not registered - call connect() first")]
    NotRegistered,
    
    /// Connection timeout
    #[error("Connection timeout: {0}")]
    Timeout(String),
    
    /// Invalid configuration
    #[error("Invalid configuration: {0}")]
    InvalidConfig(String),
    
    /// Agent already connected
    #[error("Agent already connected")]
    AlreadyConnected,
    
    /// Heartbeat failure
    #[error("Heartbeat failed: {0}")]
    HeartbeatFailed(String),
    
    /// Thread communication error
    #[error("Thread communication error: {0}")]
    ThreadError(String),
    
    /// Generic SDK error
    #[error("SDK error: {0}")]
    Other(String),
}

impl SdkError {
    /// Check if error is retryable (for reconnection logic)
    pub fn is_retryable(&self) -> bool {
        matches!(
            self,
            SdkError::Zmq(_)
                | SdkError::Timeout(_)
                | SdkError::HeartbeatFailed(_)
        )
    }
}

