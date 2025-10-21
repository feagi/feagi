//! Core types for agent registration

use serde::{Deserialize, Serialize};
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_agent_type_serialization() {
        let sensory = AgentType::Sensory;
        let json = serde_json::to_string(&sensory).unwrap();
        assert_eq!(json, "\"sensory\"");
        
        let motor = AgentType::Motor;
        let json = serde_json::to_string(&motor).unwrap();
        assert_eq!(json, "\"motor\"");
        
        let both = AgentType::Both;
        let json = serde_json::to_string(&both).unwrap();
        assert_eq!(json, "\"both\"");
        
        let visualization = AgentType::Visualization;
        let json = serde_json::to_string(&visualization).unwrap();
        assert_eq!(json, "\"visualization\"");
        
        let infrastructure = AgentType::Infrastructure;
        let json = serde_json::to_string(&infrastructure).unwrap();
        assert_eq!(json, "\"infrastructure\"");
    }

    #[test]
    fn test_agent_type_deserialization() {
        let sensory: AgentType = serde_json::from_str("\"sensory\"").unwrap();
        assert_eq!(sensory, AgentType::Sensory);
        
        let motor: AgentType = serde_json::from_str("\"motor\"").unwrap();
        assert_eq!(motor, AgentType::Motor);
        
        let both: AgentType = serde_json::from_str("\"both\"").unwrap();
        assert_eq!(both, AgentType::Both);
        
        let visualization: AgentType = serde_json::from_str("\"visualization\"").unwrap();
        assert_eq!(visualization, AgentType::Visualization);
        
        let infrastructure: AgentType = serde_json::from_str("\"infrastructure\"").unwrap();
        assert_eq!(infrastructure, AgentType::Infrastructure);
    }

    #[test]
    fn test_vision_capability() {
        let cap = VisionCapability {
            modality: "camera".to_string(),
            dimensions: (640, 480),
            channels: 3,
            target_cortical_area: "vision_1".to_string(),
        };
        
        let json = serde_json::to_string(&cap).unwrap();
        let parsed: VisionCapability = serde_json::from_str(&json).unwrap();
        
        assert_eq!(parsed.modality, "camera");
        assert_eq!(parsed.dimensions, (640, 480));
        assert_eq!(parsed.channels, 3);
        assert_eq!(parsed.target_cortical_area, "vision_1");
    }

    #[test]
    fn test_motor_capability() {
        let cap = MotorCapability {
            modality: "servo".to_string(),
            output_count: 4,
            source_cortical_areas: vec!["motor_1".to_string(), "motor_2".to_string()],
        };
        
        let json = serde_json::to_string(&cap).unwrap();
        let parsed: MotorCapability = serde_json::from_str(&json).unwrap();
        
        assert_eq!(parsed.modality, "servo");
        assert_eq!(parsed.output_count, 4);
        assert_eq!(parsed.source_cortical_areas.len(), 2);
    }

    #[test]
    fn test_agent_capabilities_vision_only() {
        let mut cap = AgentCapabilities::default();
        cap.vision = Some(VisionCapability {
            modality: "camera".to_string(),
            dimensions: (1920, 1080),
            channels: 4,
            target_cortical_area: "vision_hd".to_string(),
        });
        
        let json = serde_json::to_string(&cap).unwrap();
        let parsed: AgentCapabilities = serde_json::from_str(&json).unwrap();
        
        assert!(parsed.vision.is_some());
        assert!(parsed.motor.is_none());
        assert_eq!(parsed.vision.unwrap().dimensions, (1920, 1080));
    }

    #[test]
    fn test_agent_capabilities_with_custom() {
        let mut cap = AgentCapabilities::default();
        cap.custom.insert(
            "audio".to_string(),
            serde_json::json!({"sample_rate": 44100, "channels": 2}),
        );
        
        let json = serde_json::to_string(&cap).unwrap();
        let parsed: AgentCapabilities = serde_json::from_str(&json).unwrap();
        
        assert!(parsed.custom.contains_key("audio"));
    }

    #[test]
    fn test_agent_info_creation() {
        let caps = AgentCapabilities::default();
        
        let info = AgentInfo::new(
            "test-agent-1".to_string(),
            AgentType::Sensory,
            caps,
        );
        
        assert_eq!(info.agent_id, "test-agent-1");
        assert_eq!(info.agent_type, AgentType::Sensory);
        assert_eq!(info.registered_at, info.last_seen);
    }

    #[test]
    fn test_agent_info_activity_update() {
        let caps = AgentCapabilities::default();
        let mut info = AgentInfo::new(
            "test-agent-1".to_string(),
            AgentType::Sensory,
            caps,
        );
        
        let original_last_seen = info.last_seen;
        std::thread::sleep(std::time::Duration::from_millis(10));
        info.update_activity();
        
        assert!(info.last_seen > original_last_seen);
    }

    #[test]
    fn test_agent_info_is_inactive() {
        let caps = AgentCapabilities::default();
        let info = AgentInfo::new(
            "test-agent-1".to_string(),
            AgentType::Sensory,
            caps,
        );
        
        // Should not be inactive with 1 hour timeout
        assert!(!info.is_inactive(3600000));
        
        // Simulate old agent by manually setting last_seen to 0
        let mut old_info = info.clone();
        old_info.last_seen = 0;
        
        // Should be inactive with 1ms timeout
        assert!(old_info.is_inactive(1));
    }

    #[test]
    fn test_agent_info_serialization() {
        let mut caps = AgentCapabilities::default();
        caps.vision = Some(VisionCapability {
            modality: "camera".to_string(),
            dimensions: (640, 480),
            channels: 3,
            target_cortical_area: "vision_1".to_string(),
        });
        
        let info = AgentInfo::new(
            "test-agent-1".to_string(),
            AgentType::Sensory,
            caps,
        );
        
        let json = serde_json::to_string(&info).unwrap();
        let parsed: AgentInfo = serde_json::from_str(&json).unwrap();
        
        assert_eq!(parsed.agent_id, info.agent_id);
        assert_eq!(parsed.agent_type, info.agent_type);
        assert!(parsed.capabilities.vision.is_some());
    }
}

