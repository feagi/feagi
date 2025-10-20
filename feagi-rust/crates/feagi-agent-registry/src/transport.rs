//! Transport abstraction for agent communication

use crate::Result;

/// Transport-agnostic interface for agent communication
///
/// Different transports (ZMQ, REST, gRPC, etc.) implement this trait to
/// adapt the AgentRegistry to their specific communication protocol.
pub trait AgentTransport: Send + Sync {
    /// Send registration confirmation to agent
    ///
    /// # Arguments
    /// * `agent_id` - ID of the agent being confirmed
    /// * `endpoints` - Connection endpoints for sensory/motor data
    fn send_registration_confirmation(
        &self,
        agent_id: &str,
        endpoints: &TransportEndpoints,
    ) -> Result<()>;
    
    /// Send registration rejection to agent
    ///
    /// # Arguments
    /// * `agent_id` - ID of the agent being rejected
    /// * `reason` - Human-readable rejection reason
    fn send_registration_rejection(&self, agent_id: &str, reason: &str) -> Result<()>;
    
    /// Notify agent of deregistration
    ///
    /// # Arguments
    /// * `agent_id` - ID of the agent being deregistered
    /// * `reason` - Reason for deregistration (timeout, manual, etc.)
    fn send_deregistration_notice(&self, agent_id: &str, reason: &str) -> Result<()>;
}

/// Transport endpoint information provided during registration
#[derive(Debug, Clone)]
pub struct TransportEndpoints {
    /// Endpoint for sensory data input (agents send to this)
    pub sensory_input: String,
    
    /// Endpoint for motor data output (agents receive from this)
    pub motor_output: String,
    
    /// Additional transport-specific endpoints
    pub custom: std::collections::HashMap<String, String>,
}

impl TransportEndpoints {
    /// Create basic endpoints
    pub fn new(sensory_input: String, motor_output: String) -> Self {
        Self {
            sensory_input,
            motor_output,
            custom: std::collections::HashMap::new(),
        }
    }
    
    /// Add custom endpoint
    pub fn with_custom(mut self, key: String, value: String) -> Self {
        self.custom.insert(key, value);
        self
    }
}

