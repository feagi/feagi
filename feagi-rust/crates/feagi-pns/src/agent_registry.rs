// Agent Registry - tracks all registered agents and their state
//
// This is the single source of truth for agent registration in FEAGI 2.0.
// It replaces the deprecated feagi-agent-registry crate.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};

/// Type of agent based on I/O direction and purpose
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum AgentType {
    /// Agent provides sensory input to FEAGI
    Sensory,
    /// Agent receives motor output from FEAGI
    Motor,
    /// Agent both sends and receives data
    Both,
    /// Agent consumes visualization stream only (e.g., Brain Visualizer clients)
    Visualization,
    /// Infrastructure agent (e.g., bridges, proxies) - needs viz + control streams
    Infrastructure,
}

impl std::fmt::Display for AgentType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AgentType::Sensory => write!(f, "sensory"),
            AgentType::Motor => write!(f, "motor"),
            AgentType::Both => write!(f, "both"),
            AgentType::Visualization => write!(f, "visualization"),
            AgentType::Infrastructure => write!(f, "infrastructure"),
        }
    }
}

impl std::str::FromStr for AgentType {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "sensory" => Ok(AgentType::Sensory),
            "motor" => Ok(AgentType::Motor),
            "both" => Ok(AgentType::Both),
            "visualization" => Ok(AgentType::Visualization),
            "infrastructure" => Ok(AgentType::Infrastructure),
            _ => Err(format!("Invalid agent type: {}", s)),
        }
    }
}

/// Vision input capability
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisionCapability {
    /// Type of vision sensor (camera, lidar, depth, etc.)
    pub modality: String,
    /// Frame dimensions [width, height]
    pub dimensions: (usize, usize),
    /// Number of channels (1=grayscale, 3=RGB, 4=RGBA)
    pub channels: usize,
    /// Target cortical area ID
    pub target_cortical_area: String,
}

/// Motor output capability
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorCapability {
    /// Type of motor (servo, stepper, dc, etc.)
    pub modality: String,
    /// Number of motor outputs
    pub output_count: usize,
    /// Source cortical area IDs
    pub source_cortical_areas: Vec<String>,
}

/// Visualization capability for agents that consume neural activity stream
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisualizationCapability {
    /// Type of visualization (3d_brain, 2d_plot, neural_graph, etc.)
    pub visualization_type: String,
    /// Display resolution if applicable
    #[serde(skip_serializing_if = "Option::is_none")]
    pub resolution: Option<(usize, usize)>,
    /// Refresh rate in Hz if applicable
    #[serde(skip_serializing_if = "Option::is_none")]
    pub refresh_rate: Option<f64>,
    /// Whether this is a bridge/proxy agent (vs direct consumer)
    #[serde(default)]
    pub bridge_proxy: bool,
}

/// Legacy sensory capability (for backward compatibility with existing code)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensoryCapability {
    pub rate_hz: f64,
    pub shm_path: Option<String>,
    pub cortical_mappings: HashMap<String, u32>, // cortical_id -> cortical_idx
}

/// Agent capabilities describing what data it can provide/consume
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AgentCapabilities {
    /// Vision input capability
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vision: Option<VisionCapability>,
    
    /// Motor output capability
    #[serde(skip_serializing_if = "Option::is_none")]
    pub motor: Option<MotorCapability>,
    
    /// Visualization capability
    #[serde(skip_serializing_if = "Option::is_none")]
    pub visualization: Option<VisualizationCapability>,
    
    /// Legacy sensory capability (for backward compatibility)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sensory: Option<SensoryCapability>,
    
    /// Custom capabilities (extensible for audio, tactile, etc.)
    #[serde(flatten)]
    pub custom: serde_json::Map<String, serde_json::Value>,
}

/// Transport mechanism for agent communication
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum AgentTransport {
    Zmq,
    Shm,
    Hybrid, // Uses both ZMQ and SHM
}

/// Complete agent information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgentInfo {
    /// Unique agent identifier
    pub agent_id: String,
    
    /// Agent type (sensory, motor, or both)
    pub agent_type: AgentType,
    
    /// Agent capabilities
    pub capabilities: AgentCapabilities,
    
    /// Registration timestamp (Unix epoch milliseconds)
    pub registered_at: u64,
    
    /// Last activity timestamp (Unix epoch milliseconds)
    pub last_seen: u64,
    
    /// Transport mechanism
    pub transport: AgentTransport,
    
    /// Metadata (client version, hostname, etc.)
    #[serde(default)]
    pub metadata: serde_json::Map<String, serde_json::Value>,
}

impl AgentInfo {
    /// Create a new agent info with current timestamp
    pub fn new(agent_id: String, agent_type: AgentType, capabilities: AgentCapabilities, transport: AgentTransport) -> Self {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
        
        Self {
            agent_id,
            agent_type,
            capabilities,
            registered_at: now,
            last_seen: now,
            transport,
            metadata: serde_json::Map::new(),
        }
    }
    
    /// Update last_seen timestamp to current time
    pub fn update_activity(&mut self) {
        self.last_seen = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
    }
    
    /// Check if agent has been inactive for more than timeout_ms
    pub fn is_inactive(&self, timeout_ms: u64) -> bool {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
        
        now - self.last_seen > timeout_ms
    }
}

/// Agent Registry - single source of truth for agent management
pub struct AgentRegistry {
    agents: HashMap<String, AgentInfo>,
    max_agents: usize,
    timeout_ms: u64,
}

impl AgentRegistry {
    /// Create a new agent registry
    ///
    /// # Arguments
    /// * `max_agents` - Maximum number of concurrent agents (default: 100)
    /// * `timeout_ms` - Inactivity timeout in milliseconds (default: 60000)
    pub fn new(max_agents: usize, timeout_ms: u64) -> Self {
        println!("🦀 [REGISTRY] Initialized (max_agents={}, timeout_ms={})", max_agents, timeout_ms);
        Self {
            agents: HashMap::new(),
            max_agents,
            timeout_ms,
        }
    }
    
    /// Create registry with default settings (100 agents, 60s timeout)
    pub fn with_defaults() -> Self {
        Self::new(100, 60000)
    }

    /// Register a new agent
    pub fn register(&mut self, agent_info: AgentInfo) -> Result<(), String> {
        let agent_id = agent_info.agent_id.clone();
        
        // Validate configuration
        self.validate_agent_config(&agent_id, &agent_info.agent_type, &agent_info.capabilities)?;
        
        // Check if already registered (allow re-registration)
        let is_reregistration = self.agents.contains_key(&agent_id);
        if is_reregistration {
            println!("⚠️ [REGISTRY] Agent re-registering (updating existing entry): {}", agent_id);
        } else {
            // Check capacity only for new registrations
            if self.agents.len() >= self.max_agents {
                return Err(format!("Registry full ({}/{})", self.agents.len(), self.max_agents));
            }
        }

        println!("🦀 [REGISTRY] Registered agent: {} (type: {}, total: {})", 
                 agent_id, agent_info.agent_type, self.agents.len() + if is_reregistration { 0 } else { 1 });
        self.agents.insert(agent_id, agent_info);
        Ok(())
    }

    /// Deregister an agent
    pub fn deregister(&mut self, agent_id: &str) -> Result<(), String> {
        if self.agents.remove(agent_id).is_some() {
            println!("🦀 [REGISTRY] Deregistered agent: {} (total: {})", agent_id, self.agents.len());
            Ok(())
        } else {
            Err(format!("Agent {} not found", agent_id))
        }
    }

    /// Update heartbeat for an agent
    pub fn heartbeat(&mut self, agent_id: &str) -> Result<(), String> {
        if let Some(agent) = self.agents.get_mut(agent_id) {
            agent.update_activity();
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

    /// Get agents with stale heartbeats (older than configured timeout)
    pub fn get_stale_agents(&self) -> Vec<String> {
        self.agents
            .iter()
            .filter(|(_, info)| info.is_inactive(self.timeout_ms))
            .map(|(id, _)| id.clone())
            .collect()
    }

    /// Prune inactive agents
    ///
    /// # Returns
    /// Number of agents pruned
    pub fn prune_inactive_agents(&mut self) -> usize {
        let inactive: Vec<String> = self.agents
            .iter()
            .filter(|(_, info)| info.is_inactive(self.timeout_ms))
            .map(|(id, _)| id.clone())
            .collect();
        
        let count = inactive.len();
        for agent_id in &inactive {
            self.agents.remove(agent_id);
            println!("🦀 [REGISTRY] Pruned inactive agent: {}", agent_id);
        }
        
        if count > 0 {
            println!("🦀 [REGISTRY] Pruned {} inactive agents (total: {})", count, self.agents.len());
        }
        
        count
    }

    /// Get number of registered agents
    pub fn count(&self) -> usize {
        self.agents.len()
    }
    
    /// Validate agent configuration
    fn validate_agent_config(
        &self,
        agent_id: &str,
        agent_type: &AgentType,
        capabilities: &AgentCapabilities,
    ) -> Result<(), String> {
        // Agent ID must not be empty
        if agent_id.is_empty() {
            return Err("Agent ID cannot be empty".to_string());
        }
        
        // Validate capabilities match agent type
        match agent_type {
            AgentType::Sensory => {
                if capabilities.vision.is_none() 
                    && capabilities.sensory.is_none() 
                    && capabilities.custom.is_empty() {
                    return Err("Sensory agent must have at least one input capability".to_string());
                }
            }
            AgentType::Motor => {
                if capabilities.motor.is_none() {
                    return Err("Motor agent must have motor capability".to_string());
                }
            }
            AgentType::Both => {
                // Both requires at least one capability of each type
                if (capabilities.vision.is_none() && capabilities.sensory.is_none() && capabilities.custom.is_empty()) 
                    || capabilities.motor.is_none() {
                    return Err("Bidirectional agent must have both input and output capabilities".to_string());
                }
            }
            AgentType::Visualization => {
                if capabilities.visualization.is_none() {
                    return Err("Visualization agent must have visualization capability".to_string());
                }
            }
            AgentType::Infrastructure => {
                // Infrastructure agents are flexible - they can proxy any type
                // Just require at least one capability to be declared
                if capabilities.vision.is_none()
                    && capabilities.sensory.is_none()
                    && capabilities.motor.is_none()
                    && capabilities.visualization.is_none()
                    && capabilities.custom.is_empty() {
                    return Err("Infrastructure agent must declare at least one capability".to_string());
                }
            }
        }
        
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_agent_type_serialization() {
        let sensory = AgentType::Sensory;
        let json = serde_json::to_string(&sensory).unwrap();
        assert_eq!(json, "\"sensory\"");
        
        let infrastructure = AgentType::Infrastructure;
        let json = serde_json::to_string(&infrastructure).unwrap();
        assert_eq!(json, "\"infrastructure\"");
    }

    #[test]
    fn test_agent_registry() {
        let mut registry = AgentRegistry::new(10, 60000);
        assert_eq!(registry.count(), 0);

        let agent_info = AgentInfo::new(
            "test-agent-1".to_string(),
            AgentType::Sensory,
            AgentCapabilities {
                sensory: Some(SensoryCapability {
                    rate_hz: 30.0,
                    shm_path: Some("/tmp/test".to_string()),
                    cortical_mappings: HashMap::new(),
                }),
                ..Default::default()
            },
            AgentTransport::Shm,
        );

        registry.register(agent_info).unwrap();
        assert_eq!(registry.count(), 1);

        registry.heartbeat("test-agent-1").unwrap();
        assert!(registry.get("test-agent-1").is_some());

        registry.deregister("test-agent-1").unwrap();
        assert_eq!(registry.count(), 0);
    }
    
    #[test]
    fn test_capacity_limit() {
        let mut registry = AgentRegistry::new(2, 60000);
        
        let agent1 = AgentInfo::new(
            "agent1".to_string(),
            AgentType::Sensory,
            AgentCapabilities {
                sensory: Some(SensoryCapability {
                    rate_hz: 30.0,
                    shm_path: None,
                    cortical_mappings: HashMap::new(),
                }),
                ..Default::default()
            },
            AgentTransport::Zmq,
        );
        
        let agent2 = AgentInfo::new(
            "agent2".to_string(),
            AgentType::Sensory,
            AgentCapabilities {
                sensory: Some(SensoryCapability {
                    rate_hz: 30.0,
                    shm_path: None,
                    cortical_mappings: HashMap::new(),
                }),
                ..Default::default()
            },
            AgentTransport::Zmq,
        );
        
        let agent3 = AgentInfo::new(
            "agent3".to_string(),
            AgentType::Sensory,
            AgentCapabilities {
                sensory: Some(SensoryCapability {
                    rate_hz: 30.0,
                    shm_path: None,
                    cortical_mappings: HashMap::new(),
                }),
                ..Default::default()
            },
            AgentTransport::Zmq,
        );
        
        registry.register(agent1).unwrap();
        registry.register(agent2).unwrap();
        
        // Third registration should fail (capacity reached)
        let result = registry.register(agent3);
        assert!(result.is_err());
        assert_eq!(registry.count(), 2);
    }
    
    #[test]
    fn test_validation_sensory_without_capabilities() {
        let mut registry = AgentRegistry::new(10, 60000);
        
        let agent = AgentInfo::new(
            "test-agent".to_string(),
            AgentType::Sensory,
            AgentCapabilities::default(),
            AgentTransport::Zmq,
        );
        
        let result = registry.register(agent);
        assert!(result.is_err());
    }
    
    #[test]
    fn test_infrastructure_agent() {
        let mut registry = AgentRegistry::new(10, 60000);
        
        let agent = AgentInfo::new(
            "feagi_bridge".to_string(),
            AgentType::Infrastructure,
            AgentCapabilities {
                visualization: Some(VisualizationCapability {
                    visualization_type: "bridge".to_string(),
                    resolution: None,
                    refresh_rate: None,
                    bridge_proxy: true,
                }),
                ..Default::default()
            },
            AgentTransport::Zmq,
        );
        
        registry.register(agent).unwrap();
        assert_eq!(registry.count(), 1);
    }
}
