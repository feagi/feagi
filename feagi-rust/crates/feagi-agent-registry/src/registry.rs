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
    
    #[derive(Clone, Copy)]
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
    
    #[test]
    fn test_capacity_limit() {
        let registry = AgentRegistry::new(2, 60000);
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
        
        // Register first agent
        registry.register_agent(
            "agent1".to_string(),
            AgentType::Sensory,
            capabilities.clone(),
            &transport,
            &endpoints,
        ).unwrap();
        
        // Register second agent
        registry.register_agent(
            "agent2".to_string(),
            AgentType::Sensory,
            capabilities.clone(),
            &transport,
            &endpoints,
        ).unwrap();
        
        // Third registration should fail (capacity reached)
        let result = registry.register_agent(
            "agent3".to_string(),
            AgentType::Sensory,
            capabilities,
            &transport,
            &endpoints,
        );
        
        assert!(result.is_err());
        assert_eq!(registry.agent_count(), 2);
    }
    
    #[test]
    fn test_deregister_agent() {
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
        
        registry.register_agent(
            "test_agent".to_string(),
            AgentType::Sensory,
            capabilities,
            &transport,
            &endpoints,
        ).unwrap();
        
        assert_eq!(registry.agent_count(), 1);
        
        // Deregister
        let result = registry.deregister_agent("test_agent", Some(&transport), "test");
        assert!(result.is_ok());
        assert_eq!(registry.agent_count(), 0);
    }
    
    #[test]
    fn test_deregister_nonexistent_agent() {
        let registry = AgentRegistry::new(10, 60000);
        let transport = MockTransport;
        
        let result = registry.deregister_agent("nonexistent", Some(&transport), "test");
        assert!(result.is_err());
    }
    
    #[test]
    fn test_get_agent() {
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
        
        registry.register_agent(
            "test_agent".to_string(),
            AgentType::Sensory,
            capabilities,
            &transport,
            &endpoints,
        ).unwrap();
        
        let agent = registry.get_agent("test_agent").unwrap();
        assert_eq!(agent.agent_id, "test_agent");
        assert_eq!(agent.agent_type, AgentType::Sensory);
    }
    
    #[test]
    fn test_get_nonexistent_agent() {
        let registry = AgentRegistry::new(10, 60000);
        let result = registry.get_agent("nonexistent");
        assert!(result.is_err());
    }
    
    #[test]
    fn test_get_all_agents() {
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
        
        registry.register_agent(
            "agent1".to_string(),
            AgentType::Sensory,
            capabilities.clone(),
            &transport,
            &endpoints,
        ).unwrap();
        
        registry.register_agent(
            "agent2".to_string(),
            AgentType::Sensory,
            capabilities,
            &transport,
            &endpoints,
        ).unwrap();
        
        let agents = registry.get_all_agents();
        assert_eq!(agents.len(), 2);
    }
    
    #[test]
    fn test_update_agent_activity() {
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
        
        registry.register_agent(
            "test_agent".to_string(),
            AgentType::Sensory,
            capabilities,
            &transport,
            &endpoints,
        ).unwrap();
        
        let agent_before = registry.get_agent("test_agent").unwrap();
        let last_seen_before = agent_before.last_seen;
        
        std::thread::sleep(std::time::Duration::from_millis(10));
        
        registry.update_agent_activity("test_agent").unwrap();
        
        let agent_after = registry.get_agent("test_agent").unwrap();
        assert!(agent_after.last_seen > last_seen_before);
    }
    
    #[test]
    fn test_prune_inactive_agents() {
        let registry = AgentRegistry::new(10, 100); // 100ms timeout
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
        
        // Register two agents
        registry.register_agent(
            "agent1".to_string(),
            AgentType::Sensory,
            capabilities.clone(),
            &transport,
            &endpoints,
        ).unwrap();
        
        registry.register_agent(
            "agent2".to_string(),
            AgentType::Sensory,
            capabilities,
            &transport,
            &endpoints,
        ).unwrap();
        
        assert_eq!(registry.agent_count(), 2);
        
        // Wait for timeout
        std::thread::sleep(std::time::Duration::from_millis(150));
        
        // Prune inactive agents
        let pruned = registry.prune_inactive_agents(Some(&transport));
        assert_eq!(pruned, 2);
        assert_eq!(registry.agent_count(), 0);
    }
    
    #[test]
    fn test_validation_empty_agent_id() {
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
            "".to_string(),
            AgentType::Sensory,
            capabilities,
            &transport,
            &endpoints,
        );
        
        assert!(result.is_err());
    }
    
    #[test]
    fn test_validation_sensory_without_capabilities() {
        let registry = AgentRegistry::new(10, 60000);
        let transport = MockTransport;
        let endpoints = TransportEndpoints::new(
            "tcp://localhost:5555".to_string(),
            "tcp://localhost:5556".to_string(),
        );
        
        let capabilities = AgentCapabilities::default();
        
        let result = registry.register_agent(
            "test_agent".to_string(),
            AgentType::Sensory,
            capabilities,
            &transport,
            &endpoints,
        );
        
        assert!(result.is_err());
    }
    
    #[test]
    fn test_validation_motor_without_motor_capability() {
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
            AgentType::Motor,
            capabilities,
            &transport,
            &endpoints,
        );
        
        assert!(result.is_err());
    }
    
    #[test]
    fn test_thread_safety() {
        use std::sync::Arc;
        use std::thread;
        
        let registry = Arc::new(AgentRegistry::new(100, 60000));
        let transport = MockTransport;
        let endpoints = TransportEndpoints::new(
            "tcp://localhost:5555".to_string(),
            "tcp://localhost:5556".to_string(),
        );
        
        let mut handles = vec![];
        
        // Spawn 10 threads, each registering 5 agents
        for thread_id in 0..10 {
            let registry_clone = Arc::clone(&registry);
            let endpoints_clone = endpoints.clone();
            
            let handle = thread::spawn(move || {
                for agent_id in 0..5 {
                    let agent_name = format!("agent_t{}_a{}", thread_id, agent_id);
                    
                    let mut capabilities = AgentCapabilities::default();
                    capabilities.vision = Some(VisionCapability {
                        modality: "camera".to_string(),
                        dimensions: (640, 480),
                        channels: 3,
                        target_cortical_area: "i_vision".to_string(),
                    });
                    
                    let _ = registry_clone.register_agent(
                        agent_name,
                        AgentType::Sensory,
                        capabilities,
                        &transport,
                        &endpoints_clone,
                    );
                }
            });
            
            handles.push(handle);
        }
        
        // Wait for all threads to complete
        for handle in handles {
            handle.join().unwrap();
        }
        
        // Should have 50 registered agents
        assert_eq!(registry.agent_count(), 50);
    }
}

