//! Core types for agent registration

use serde::{Deserialize, Serialize};
use std::time::{SystemTime, UNIX_EPOCH};

/// Type of agent based on I/O direction
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum AgentType {
    /// Agent provides sensory input to FEAGI
    Sensory,
    /// Agent receives motor output from FEAGI
    Motor,
    /// Agent both sends and receives data
    Both,
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

/// Agent capabilities describing what data it can provide/consume
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AgentCapabilities {
    /// Vision input capability
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vision: Option<VisionCapability>,
    
    /// Motor output capability
    #[serde(skip_serializing_if = "Option::is_none")]
    pub motor: Option<MotorCapability>,
    
    /// Custom capabilities (extensible for audio, tactile, etc.)
    #[serde(flatten)]
    pub custom: serde_json::Map<String, serde_json::Value>,
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
    
    /// Metadata (client version, hostname, etc.)
    #[serde(default)]
    pub metadata: serde_json::Map<String, serde_json::Value>,
}

impl AgentInfo {
    /// Create a new agent info with current timestamp
    pub fn new(agent_id: String, agent_type: AgentType, capabilities: AgentCapabilities) -> Self {
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

