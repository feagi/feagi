//! Agent registry implementation

use crate::{AgentInfo, AgentCapabilities, AgentType, AgentTransport, TransportEndpoints, Result, RegistryError};
use std::collections::HashMap;
use std::sync::{Arc, RwLock};
use log::{info, warn, debug};

/// Agent registry managing lifecycle and state
///
/// This is the core transport-agnostic registry. It maintains agent state
/// and enforces registration rules without knowledge of the underlying
/// transport mechanism.
pub struct AgentRegistry {
    /// Registered agents (agent_id -> AgentInfo)
    agents: Arc<RwLock<HashMap<String, AgentInfo>>>,
    
    /// Maximum number of agents allowed
    max_agents: usize,
    
    /// Agent inactivity timeout in milliseconds
    timeout_ms: u64,
}

impl AgentRegistry {
    /// Create a new agent registry
    ///
    /// # Arguments
    /// * `max_agents` - Maximum number of concurrent agents
    /// * `timeout_ms` - Inactivity timeout in milliseconds
    pub fn new(max_agents: usize, timeout_ms: u64) -> Self {
        info!("Initializing agent registry (max_agents={}, timeout_ms={})", max_agents, timeout_ms);
        Self {
            agents: Arc::new(RwLock::new(HashMap::new())),
            max_agents,
            timeout_ms,
        }
    }
    
    /// Register a new agent
    ///
    /// # Arguments
    /// * `agent_id` - Unique agent identifier
    /// * `agent_type` - Type of agent (sensory/motor/both)
    /// * `capabilities` - Agent capabilities
    /// * `transport` - Transport implementation for sending responses
    /// * `endpoints` - Connection endpoints to provide to agent
    pub fn register_agent(
        &self,
        agent_id: String,
        agent_type: AgentType,
        capabilities: AgentCapabilities,
        transport: &dyn AgentTransport,
        endpoints: &TransportEndpoints,
    ) -> Result<()> {
        debug!("Registration request from agent: {}", agent_id);
        
        // Validate configuration
        self.validate_agent_config(&agent_id, &agent_type, &capabilities)?;
        
        let mut agents = self.agents.write().unwrap();
        
        // Check if already registered
        if agents.contains_key(&agent_id) {
            warn!("Agent already registered: {}", agent_id);
            transport.send_registration_rejection(&agent_id, "Agent already registered")?;
            return Err(RegistryError::AgentAlreadyExists(agent_id));
        }
        
        // Check capacity
        if agents.len() >= self.max_agents {
            warn!("Agent registry full ({}/{})", agents.len(), self.max_agents);
            transport.send_registration_rejection(&agent_id, "Registry capacity reached")?;
            return Err(RegistryError::InvalidConfiguration("Registry full".to_string()));
        }
        
        // Create and store agent info
        let agent_info = AgentInfo::new(agent_id.clone(), agent_type, capabilities);
        agents.insert(agent_id.clone(), agent_info);
        
        info!("✓ Agent registered: {} (type: {:?}, total agents: {})", 
              agent_id, agent_type, agents.len());
        
        // Send confirmation via transport
        transport.send_registration_confirmation(&agent_id, endpoints)?;
        
        Ok(())
    }
    
    /// Deregister an agent
    ///
    /// # Arguments
    /// * `agent_id` - ID of agent to deregister
    /// * `transport` - Transport for sending deregistration notice
    /// * `reason` - Reason for deregistration
    pub fn deregister_agent(
        &self,
        agent_id: &str,
        transport: Option<&dyn AgentTransport>,
        reason: &str,
    ) -> Result<()> {
        let mut agents = self.agents.write().unwrap();
        
        if agents.remove(agent_id).is_some() {
            info!("✓ Agent deregistered: {} (reason: {})", agent_id, reason);
            
            if let Some(t) = transport {
                let _ = t.send_deregistration_notice(agent_id, reason);
            }
            
            Ok(())
        } else {
            Err(RegistryError::AgentNotFound(agent_id.to_string()))
        }
    }
    
    /// Update agent activity timestamp
    pub fn update_agent_activity(&self, agent_id: &str) -> Result<()> {
        let mut agents = self.agents.write().unwrap();
        
        if let Some(agent) = agents.get_mut(agent_id) {
            agent.update_activity();
            Ok(())
        } else {
            Err(RegistryError::AgentNotFound(agent_id.to_string()))
        }
    }
    
    /// Get agent information
    pub fn get_agent(&self, agent_id: &str) -> Result<AgentInfo> {
        let agents = self.agents.read().unwrap();
        agents.get(agent_id)
            .cloned()
            .ok_or_else(|| RegistryError::AgentNotFound(agent_id.to_string()))
    }
    
    /// Get all registered agents
    pub fn get_all_agents(&self) -> Vec<AgentInfo> {
        let agents = self.agents.read().unwrap();
        agents.values().cloned().collect()
    }
    
    /// Get count of registered agents
    pub fn agent_count(&self) -> usize {
        self.agents.read().unwrap().len()
    }
    
    /// Prune inactive agents
    ///
    /// # Arguments
    /// * `transport` - Transport for sending deregistration notices
    ///
    /// # Returns
    /// Number of agents pruned
    pub fn prune_inactive_agents(&self, transport: Option<&dyn AgentTransport>) -> usize {
        let mut agents = self.agents.write().unwrap();
        let initial_count = agents.len();
        
        let inactive: Vec<String> = agents
            .iter()
            .filter(|(_, info)| info.is_inactive(self.timeout_ms))
            .map(|(id, _)| id.clone())
            .collect();
        
        for agent_id in &inactive {
            agents.remove(agent_id);
            info!("Pruned inactive agent: {}", agent_id);
            
            if let Some(t) = transport {
                let _ = t.send_deregistration_notice(agent_id, "Inactivity timeout");
            }
        }
        
        let pruned = initial_count - agents.len();
        if pruned > 0 {
            info!("Pruned {} inactive agents", pruned);
        }
        
        pruned
    }
    
    /// Validate agent configuration
    fn validate_agent_config(
        &self,
        agent_id: &str,
        agent_type: &AgentType,
        capabilities: &AgentCapabilities,
    ) -> Result<()> {
        // Agent ID must not be empty
        if agent_id.is_empty() {
            return Err(RegistryError::InvalidConfiguration(
                "Agent ID cannot be empty".to_string()
            ));
        }
        
        // Validate capabilities match agent type
        match agent_type {
            AgentType::Sensory => {
                if capabilities.vision.is_none() && capabilities.custom.is_empty() {
                    return Err(RegistryError::InvalidConfiguration(
                        "Sensory agent must have at least one input capability".to_string()
                    ));
                }
            }
            AgentType::Motor => {
                if capabilities.motor.is_none() {
                    return Err(RegistryError::InvalidConfiguration(
                        "Motor agent must have motor capability".to_string()
                    ));
                }
            }
            AgentType::Both => {
                // Both requires at least one capability of each type
                if (capabilities.vision.is_none() && capabilities.custom.is_empty()) 
                    || capabilities.motor.is_none() {
                    return Err(RegistryError::InvalidConfiguration(
                        "Bidirectional agent must have both input and output capabilities".to_string()
                    ));
                }
            }
        }
        
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::VisionCapability;
    
    struct MockTransport;
    
    impl AgentTransport for MockTransport {
        fn send_registration_confirmation(&self, _: &str, _: &TransportEndpoints) -> Result<()> {
            Ok(())
        }
        
        fn send_registration_rejection(&self, _: &str, _: &str) -> Result<()> {
            Ok(())
        }
        
        fn send_deregistration_notice(&self, _: &str, _: &str) -> Result<()> {
            Ok(())
        }
    }
    
    #[test]
    fn test_agent_registration() {
        let registry = AgentRegistry::new(10, 60000);
        let transport = MockTransport;
        let endpoints = TransportEndpoints::new(
            "tcp://localhost:5555".to_string(),
            "tcp://localhost:5556".to_string(),
        );
        
        let mut capabilities = AgentCapabilities::default();
        capabilities.vision = Some(VisionCapability {
            modality: "camera".to_string(),
            dimensions: (640, 480),
            channels: 3,
            target_cortical_area: "i_vision".to_string(),
        });
        
        let result = registry.register_agent(
            "test_agent".to_string(),
            AgentType::Sensory,
            capabilities,
            &transport,
            &endpoints,
        );
        
        assert!(result.is_ok());
        assert_eq!(registry.agent_count(), 1);
    }
    
    #[test]
    fn test_duplicate_registration() {
        let registry = AgentRegistry::new(10, 60000);
        let transport = MockTransport;
        let endpoints = TransportEndpoints::new(
            "tcp://localhost:5555".to_string(),
            "tcp://localhost:5556".to_string(),
        );
        
        let mut capabilities = AgentCapabilities::default();
        capabilities.vision = Some(VisionCapability {
            modality: "camera".to_string(),
            dimensions: (640, 480),
            channels: 3,
            target_cortical_area: "i_vision".to_string(),
        });
        
        // First registration
        registry.register_agent(
            "test_agent".to_string(),
            AgentType::Sensory,
            capabilities.clone(),
            &transport,
            &endpoints,
        ).unwrap();
        
        // Second registration should fail
        let result = registry.register_agent(
            "test_agent".to_string(),
            AgentType::Sensory,
            capabilities,
            &transport,
            &endpoints,
        );
        
        assert!(result.is_err());
        assert_eq!(registry.agent_count(), 1);
    }
}

