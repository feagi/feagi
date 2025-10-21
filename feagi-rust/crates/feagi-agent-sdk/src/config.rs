//! Configuration for FEAGI Agent SDK

use feagi_agent_registry::{AgentCapabilities, AgentType, VisionCapability, MotorCapability, VisualizationCapability};
use crate::error::{Result, SdkError};

/// Agent configuration builder
#[derive(Debug, Clone)]
pub struct AgentConfig {
    /// Unique agent identifier
    pub agent_id: String,
    
    /// Agent type (sensory, motor, both, visualization, or infrastructure)
    pub agent_type: AgentType,
    
    /// Agent capabilities
    pub capabilities: AgentCapabilities,
    
    /// FEAGI registration endpoint (ZMQ REQ)
    pub registration_endpoint: String,
    
    /// FEAGI sensory input endpoint (ZMQ PUSH)
    pub sensory_endpoint: String,
    
    /// FEAGI motor output endpoint (ZMQ SUB)
    pub motor_endpoint: String,
    
    /// FEAGI visualization stream endpoint (ZMQ SUB)
    pub visualization_endpoint: String,
    
    /// FEAGI control/API endpoint (ZMQ REQ - REST over ZMQ)
    pub control_endpoint: String,
    
    /// Heartbeat interval in seconds (0 = disabled)
    pub heartbeat_interval: f64,
    
    /// Connection timeout in milliseconds
    pub connection_timeout_ms: u64,
    
    /// Registration retry attempts
    pub registration_retries: u32,
    
    /// Retry backoff base in milliseconds
    pub retry_backoff_ms: u64,
}

impl AgentConfig {
    /// Create a new agent configuration
    ///
    /// # Arguments
    /// * `agent_id` - Unique identifier for this agent
    /// * `agent_type` - Type of agent (Sensory, Motor, or Both)
    ///
    /// # Example
    /// ```
    /// use feagi_agent_sdk::{AgentConfig, AgentType};
    ///
    /// let config = AgentConfig::new("my_camera", AgentType::Sensory);
    /// ```
    pub fn new(agent_id: impl Into<String>, agent_type: AgentType) -> Self {
        Self {
            agent_id: agent_id.into(),
            agent_type,
            capabilities: AgentCapabilities::default(),
            registration_endpoint: "tcp://localhost:30001".to_string(),
            sensory_endpoint: "tcp://localhost:5555".to_string(),
            motor_endpoint: "tcp://localhost:30005".to_string(),
            visualization_endpoint: "tcp://localhost:5562".to_string(),
            control_endpoint: "tcp://localhost:5563".to_string(),
            heartbeat_interval: 5.0,
            connection_timeout_ms: 5000,
            registration_retries: 3,
            retry_backoff_ms: 1000,
        }
    }
    
    /// Set FEAGI host and derive all endpoints
    ///
    /// # Example
    /// ```
    /// # use feagi_agent_sdk::{AgentConfig, AgentType};
    /// let config = AgentConfig::new("camera", AgentType::Sensory)
    ///     .with_feagi_host("192.168.1.100");
    /// ```
    pub fn with_feagi_host(mut self, host: impl Into<String>) -> Self {
        let host = host.into();
        self.registration_endpoint = format!("tcp://{}:30001", host);
        self.sensory_endpoint = format!("tcp://{}:5555", host);
        self.motor_endpoint = format!("tcp://{}:30005", host);
        self.visualization_endpoint = format!("tcp://{}:5562", host);
        self.control_endpoint = format!("tcp://{}:5563", host);
        self
    }
    
    /// Set registration endpoint
    pub fn with_registration_endpoint(mut self, endpoint: impl Into<String>) -> Self {
        self.registration_endpoint = endpoint.into();
        self
    }
    
    /// Set sensory input endpoint
    pub fn with_sensory_endpoint(mut self, endpoint: impl Into<String>) -> Self {
        self.sensory_endpoint = endpoint.into();
        self
    }
    
    /// Set motor output endpoint
    pub fn with_motor_endpoint(mut self, endpoint: impl Into<String>) -> Self {
        self.motor_endpoint = endpoint.into();
        self
    }
    
    /// Set visualization stream endpoint
    pub fn with_visualization_endpoint(mut self, endpoint: impl Into<String>) -> Self {
        self.visualization_endpoint = endpoint.into();
        self
    }
    
    /// Set control/API endpoint
    pub fn with_control_endpoint(mut self, endpoint: impl Into<String>) -> Self {
        self.control_endpoint = endpoint.into();
        self
    }
    
    /// Set heartbeat interval in seconds (0 to disable)
    pub fn with_heartbeat_interval(mut self, interval: f64) -> Self {
        self.heartbeat_interval = interval;
        self
    }
    
    /// Set connection timeout in milliseconds
    pub fn with_connection_timeout_ms(mut self, timeout_ms: u64) -> Self {
        self.connection_timeout_ms = timeout_ms;
        self
    }
    
    /// Set registration retry attempts
    pub fn with_registration_retries(mut self, retries: u32) -> Self {
        self.registration_retries = retries;
        self
    }
    
    /// Add vision capability
    ///
    /// # Example
    /// ```
    /// # use feagi_agent_sdk::{AgentConfig, AgentType};
    /// let config = AgentConfig::new("camera", AgentType::Sensory)
    ///     .with_vision_capability("camera", (640, 480), 3, "i_vision");
    /// ```
    pub fn with_vision_capability(
        mut self,
        modality: impl Into<String>,
        dimensions: (usize, usize),
        channels: usize,
        target_cortical_area: impl Into<String>,
    ) -> Self {
        self.capabilities.vision = Some(VisionCapability {
            modality: modality.into(),
            dimensions,
            channels,
            target_cortical_area: target_cortical_area.into(),
        });
        self
    }
    
    /// Add motor capability
    ///
    /// # Example
    /// ```
    /// # use feagi_agent_sdk::{AgentConfig, AgentType};
    /// let config = AgentConfig::new("arm", AgentType::Motor)
    ///     .with_motor_capability("servo", 4, vec!["o_motor".to_string()]);
    /// ```
    pub fn with_motor_capability(
        mut self,
        modality: impl Into<String>,
        output_count: usize,
        source_cortical_areas: Vec<String>,
    ) -> Self {
        self.capabilities.motor = Some(MotorCapability {
            modality: modality.into(),
            output_count,
            source_cortical_areas,
        });
        self
    }
    
    /// Add visualization capability
    ///
    /// # Example
    /// ```
    /// # use feagi_agent_sdk::{AgentConfig, AgentType};
    /// let config = AgentConfig::new("brain_viz", AgentType::Visualization)
    ///     .with_visualization_capability("3d_brain", Some((1920, 1080)), Some(30.0), false);
    /// ```
    pub fn with_visualization_capability(
        mut self,
        visualization_type: impl Into<String>,
        resolution: Option<(usize, usize)>,
        refresh_rate: Option<f64>,
        bridge_proxy: bool,
    ) -> Self {
        self.capabilities.visualization = Some(VisualizationCapability {
            visualization_type: visualization_type.into(),
            resolution,
            refresh_rate,
            bridge_proxy,
        });
        self
    }
    
    /// Add custom capability
    ///
    /// # Example
    /// ```
    /// # use feagi_agent_sdk::{AgentConfig, AgentType};
    /// use serde_json::json;
    /// 
    /// let config = AgentConfig::new("audio", AgentType::Sensory)
    ///     .with_custom_capability("audio", json!({
    ///         "sample_rate": 44100,
    ///         "channels": 2
    ///     }));
    /// ```
    pub fn with_custom_capability(
        mut self,
        key: impl Into<String>,
        value: serde_json::Value,
    ) -> Self {
        self.capabilities.custom.insert(key.into(), value);
        self
    }
    
    /// Validate configuration
    pub fn validate(&self) -> Result<()> {
        // Agent ID must not be empty
        if self.agent_id.is_empty() {
            return Err(SdkError::InvalidConfig("agent_id cannot be empty".to_string()));
        }
        
        // Must have at least one capability
        if self.capabilities.vision.is_none()
            && self.capabilities.motor.is_none()
            && self.capabilities.visualization.is_none()
            && self.capabilities.custom.is_empty()
        {
            return Err(SdkError::InvalidConfig(
                "Agent must have at least one capability".to_string()
            ));
        }
        
        // Validate agent type matches capabilities
        match self.agent_type {
            AgentType::Sensory => {
                if self.capabilities.vision.is_none() && self.capabilities.custom.is_empty() {
                    return Err(SdkError::InvalidConfig(
                        "Sensory agent must have vision or custom input capability".to_string()
                    ));
                }
            }
            AgentType::Motor => {
                if self.capabilities.motor.is_none() {
                    return Err(SdkError::InvalidConfig(
                        "Motor agent must have motor capability".to_string()
                    ));
                }
            }
            AgentType::Both => {
                if (self.capabilities.vision.is_none() && self.capabilities.custom.is_empty())
                    || self.capabilities.motor.is_none()
                {
                    return Err(SdkError::InvalidConfig(
                        "Bidirectional agent must have both input and output capabilities".to_string()
                    ));
                }
            }
            AgentType::Visualization => {
                if self.capabilities.visualization.is_none() {
                    return Err(SdkError::InvalidConfig(
                        "Visualization agent must have visualization capability".to_string()
                    ));
                }
            }
            AgentType::Infrastructure => {
                // Infrastructure agents can have any combination of capabilities
                // No strict requirements as they may proxy multiple types
                if self.capabilities.vision.is_none()
                    && self.capabilities.motor.is_none()
                    && self.capabilities.visualization.is_none()
                    && self.capabilities.custom.is_empty()
                {
                    return Err(SdkError::InvalidConfig(
                        "Infrastructure agent must declare at least one capability".to_string()
                    ));
                }
            }
        }
        
        // Validate endpoints
        if !self.registration_endpoint.starts_with("tcp://") {
            return Err(SdkError::InvalidConfig(
                "registration_endpoint must start with tcp://".to_string()
            ));
        }
        
        if !self.sensory_endpoint.starts_with("tcp://") {
            return Err(SdkError::InvalidConfig(
                "sensory_endpoint must start with tcp://".to_string()
            ));
        }
        
        if !self.motor_endpoint.starts_with("tcp://") {
            return Err(SdkError::InvalidConfig(
                "motor_endpoint must start with tcp://".to_string()
            ));
        }
        
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_config_builder() {
        let config = AgentConfig::new("test_agent", AgentType::Sensory)
            .with_feagi_host("192.168.1.100")
            .with_vision_capability("camera", (640, 480), 3, "i_vision")
            .with_heartbeat_interval(10.0);
        
        assert_eq!(config.agent_id, "test_agent");
        assert_eq!(config.heartbeat_interval, 10.0);
        assert_eq!(config.registration_endpoint, "tcp://192.168.1.100:30001");
        assert!(config.capabilities.vision.is_some());
    }
    
    #[test]
    fn test_config_validation_empty_agent_id() {
        let config = AgentConfig::new("", AgentType::Sensory);
        assert!(config.validate().is_err());
    }
    
    #[test]
    fn test_config_validation_no_capabilities() {
        let config = AgentConfig::new("test", AgentType::Sensory);
        assert!(config.validate().is_err());
    }
    
    #[test]
    fn test_config_validation_sensory_without_input() {
        let mut config = AgentConfig::new("test", AgentType::Sensory);
        config.capabilities.motor = Some(MotorCapability {
            modality: "servo".to_string(),
            output_count: 1,
            source_cortical_areas: vec!["motor".to_string()],
        });
        assert!(config.validate().is_err());
    }
    
    #[test]
    fn test_config_validation_valid() {
        let config = AgentConfig::new("test", AgentType::Sensory)
            .with_vision_capability("camera", (640, 480), 3, "vision");
        assert!(config.validate().is_ok());
    }
}

