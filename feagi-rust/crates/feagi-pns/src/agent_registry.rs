// Agent Registry - tracks all registered agents and their state

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::{Duration, Instant};

/// Agent capabilities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgentCapabilities {
    pub sensory: Option<SensoryCapability>,
    pub motor: Option<MotorCapability>,
    pub visualization: Option<VizCapability>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensoryCapability {
    pub rate_hz: f64,
    pub shm_path: Option<String>,
    pub cortical_mappings: HashMap<String, u32>, // cortical_id -> cortical_idx
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorCapability {
    pub rate_hz: f64,
    pub shm_path: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VizCapability {
    pub rate_hz: f64,
    pub shm_path: Option<String>,
}

/// Agent information
#[derive(Debug, Clone)]
pub struct AgentInfo {
    pub agent_id: String,
    pub agent_type: String,
    pub capabilities: AgentCapabilities,
    pub registered_at: Instant,
    pub last_heartbeat: Instant,
    pub transport: AgentTransport,
}

#[derive(Debug, Clone, PartialEq)]
pub enum AgentTransport {
    Zmq,
    Shm,
    Hybrid, // Uses both ZMQ and SHM
}

/// Agent Registry
pub struct AgentRegistry {
    agents: HashMap<String, AgentInfo>,
}

impl AgentRegistry {
    pub fn new() -> Self {
        Self {
            agents: HashMap::new(),
        }
    }

    /// Register a new agent
    pub fn register(&mut self, agent_info: AgentInfo) -> Result<(), String> {
        let agent_id = agent_info.agent_id.clone();
        
        if self.agents.contains_key(&agent_id) {
            return Err(format!("Agent {} already registered", agent_id));
        }

        println!("🦀 [REGISTRY] Registered agent: {} (type: {})", agent_id, agent_info.agent_type);
        self.agents.insert(agent_id, agent_info);
        Ok(())
    }

    /// Deregister an agent
    pub fn deregister(&mut self, agent_id: &str) -> Result<(), String> {
        if self.agents.remove(agent_id).is_some() {
            println!("🦀 [REGISTRY] Deregistered agent: {}", agent_id);
            Ok(())
        } else {
            Err(format!("Agent {} not found", agent_id))
        }
    }

    /// Update heartbeat for an agent
    pub fn heartbeat(&mut self, agent_id: &str) -> Result<(), String> {
        if let Some(agent) = self.agents.get_mut(agent_id) {
            agent.last_heartbeat = Instant::now();
            Ok(())
        } else {
            Err(format!("Agent {} not found", agent_id))
        }
    }

    /// Get agent info
    pub fn get(&self, agent_id: &str) -> Option<&AgentInfo> {
        self.agents.get(agent_id)
    }

    /// Get all agents
    pub fn get_all(&self) -> Vec<&AgentInfo> {
        self.agents.values().collect()
    }

    /// Get agents with stale heartbeats (older than timeout)
    pub fn get_stale_agents(&self, timeout: Duration) -> Vec<String> {
        let now = Instant::now();
        self.agents
            .iter()
            .filter(|(_, info)| now.duration_since(info.last_heartbeat) > timeout)
            .map(|(id, _)| id.clone())
            .collect()
    }

    /// Get number of registered agents
    pub fn count(&self) -> usize {
        self.agents.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_agent_registry() {
        let mut registry = AgentRegistry::new();
        assert_eq!(registry.count(), 0);

        let agent_info = AgentInfo {
            agent_id: "test-agent-1".to_string(),
            agent_type: "external".to_string(),
            capabilities: AgentCapabilities {
                sensory: Some(SensoryCapability {
                    rate_hz: 30.0,
                    shm_path: Some("/tmp/test".to_string()),
                    cortical_mappings: HashMap::new(),
                }),
                motor: None,
                visualization: None,
            },
            registered_at: Instant::now(),
            last_heartbeat: Instant::now(),
            transport: AgentTransport::Shm,
        };

        registry.register(agent_info).unwrap();
        assert_eq!(registry.count(), 1);

        registry.heartbeat("test-agent-1").unwrap();
        assert!(registry.get("test-agent-1").is_some());

        registry.deregister("test-agent-1").unwrap();
        assert_eq!(registry.count(), 0);
    }
}

