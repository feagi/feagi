//! ZMQ transport implementation for agent communication

use feagi_agent_registry::{AgentTransport, TransportEndpoints, Result, RegistryError};
use log::{info, warn, error};
use serde::{Deserialize, Serialize};
use std::sync::Arc;

/// ZMQ transport adapter for agent registry
pub struct ZmqTransport {
    /// Registration response socket (REP)
    /// Note: ZMQ sockets are not thread-safe, so this is not wrapped in Arc
    response_socket: zmq::Socket,
}

// Manual Send/Sync implementation - ZmqTransport owns the socket exclusively
unsafe impl Send for ZmqTransport {}
unsafe impl Sync for ZmqTransport {}

impl ZmqTransport {
    /// Create a new ZMQ transport
    ///
    /// # Arguments
    /// * `bind_address` - Address to bind registration endpoint (e.g., "tcp://*:5000")
    pub fn new(bind_address: &str) -> Result<Self> {
        let context = zmq::Context::new();
        let socket = context.socket(zmq::REP)
            .map_err(|e| RegistryError::TransportError(format!("Failed to create ZMQ socket: {}", e)))?;
        
        socket.bind(bind_address)
            .map_err(|e| RegistryError::TransportError(format!("Failed to bind to {}: {}", bind_address, e)))?;
        
        info!("✓ ZMQ registration endpoint bound to {}", bind_address);
        
        Ok(Self {
            response_socket: socket,
        })
    }
    
    /// Receive registration request (with timeout)
    ///
    /// Returns (agent_id, agent_type, capabilities) parsed from JSON
    /// Timeout in milliseconds (default: 1000ms)
    pub fn receive_registration_request(&self) -> Result<RegistrationRequest> {
        // Poll with 1 second timeout
        match self.response_socket.poll(zmq::POLLIN, 1000) {
            Ok(rc) if rc > 0 => {
                // Message available
                let msg = self.response_socket.recv_msg(0)
                    .map_err(|e| RegistryError::TransportError(format!("Failed to receive message: {}", e)))?;
                
                let json_str = msg.as_str()
                    .ok_or_else(|| RegistryError::TransportError("Invalid UTF-8 in message".to_string()))?;
                
                serde_json::from_str(json_str)
                    .map_err(|e| RegistryError::SerializationError(e))
            }
            Ok(_) => {
                // Timeout - no message available
                Err(RegistryError::TransportError("timeout".to_string()))
            }
            Err(e) => {
                Err(RegistryError::TransportError(format!("Poll error: {}", e)))
            }
        }
    }
}

impl AgentTransport for ZmqTransport {
    fn send_registration_confirmation(
        &self,
        agent_id: &str,
        endpoints: &TransportEndpoints,
    ) -> Result<()> {
        let response = RegistrationResponse {
            status: "success".to_string(),
            agent_id: agent_id.to_string(),
            message: Some("Agent registered successfully".to_string()),
            endpoints: Some(EndpointInfo {
                sensory_input: endpoints.sensory_input.clone(),
                motor_output: endpoints.motor_output.clone(),
            }),
        };
        
        let json = serde_json::to_string(&response)?;
        
        self.response_socket.send(&json, 0)
            .map_err(|e| RegistryError::TransportError(format!("Failed to send confirmation: {}", e)))?;
        
        info!("✓ Sent registration confirmation to {}", agent_id);
        Ok(())
    }
    
    fn send_registration_rejection(&self, agent_id: &str, reason: &str) -> Result<()> {
        let response = RegistrationResponse {
            status: "error".to_string(),
            agent_id: agent_id.to_string(),
            message: Some(reason.to_string()),
            endpoints: None,
        };
        
        let json = serde_json::to_string(&response)?;
        
        self.response_socket.send(&json, 0)
            .map_err(|e| RegistryError::TransportError(format!("Failed to send rejection: {}", e)))?;
        
        warn!("✗ Sent registration rejection to {}: {}", agent_id, reason);
        Ok(())
    }
    
    fn send_deregistration_notice(&self, agent_id: &str, reason: &str) -> Result<()> {
        // For ZMQ, deregistration notices could be sent via a separate PUB socket
        // For now, just log it (agents will know they're deregistered when their
        // messages get rejected)
        info!("Agent {} deregistered: {}", agent_id, reason);
        Ok(())
    }
}

/// Registration request from agent
#[derive(Debug, Deserialize)]
pub struct RegistrationRequest {
    #[serde(rename = "type")]
    pub request_type: String,
    pub agent_id: String,
    pub agent_type: String, // Will be parsed to AgentType enum
    pub capabilities: serde_json::Value,
}

/// Registration response to agent
#[derive(Debug, Serialize)]
struct RegistrationResponse {
    status: String,
    agent_id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    message: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    endpoints: Option<EndpointInfo>,
}

/// Endpoint information in response
#[derive(Debug, Serialize)]
struct EndpointInfo {
    sensory_input: String,
    motor_output: String,
}

