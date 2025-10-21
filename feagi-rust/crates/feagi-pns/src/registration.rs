// Registration Handler - processes agent registration requests

use crate::agent_registry::{
    AgentCapabilities, AgentInfo, AgentRegistry, AgentTransport, MotorCapability,
    SensoryCapability, VizCapability,
};
use parking_lot::RwLock;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::Instant;

/// Registration request from agent
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegistrationRequest {
    pub agent_id: String,
    pub agent_type: String,
    pub capabilities: serde_json::Value, // Flexible JSON for different formats
}

/// Registration response to agent
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegistrationResponse {
    pub status: String,
    pub message: Option<String>,
    pub shm_paths: Option<HashMap<String, String>>, // capability_type -> shm_path
    pub zmq_ports: Option<HashMap<String, u16>>,
}

/// Type alias for registration callbacks
pub type RegistrationCallback = Arc<parking_lot::Mutex<Option<Box<dyn Fn(String, String, String) + Send + Sync>>>>;
pub type DeregistrationCallback = Arc<parking_lot::Mutex<Option<Box<dyn Fn(String) + Send + Sync>>>>;

/// Registration Handler
pub struct RegistrationHandler {
    agent_registry: Arc<RwLock<AgentRegistry>>,
    shm_base_path: String,
    /// Optional reference to burst engine's sensory agent manager for SHM I/O
    sensory_agent_manager: Arc<parking_lot::Mutex<Option<Arc<std::sync::Mutex<feagi_burst_engine::AgentManager>>>>>,
    /// Callbacks for Python integration
    on_agent_registered: RegistrationCallback,
    on_agent_deregistered: DeregistrationCallback,
}

impl RegistrationHandler {
    pub fn new(agent_registry: Arc<RwLock<AgentRegistry>>) -> Self {
        Self {
            agent_registry,
            shm_base_path: "/tmp".to_string(),
            sensory_agent_manager: Arc::new(parking_lot::Mutex::new(None)),
            on_agent_registered: Arc::new(parking_lot::Mutex::new(None)),
            on_agent_deregistered: Arc::new(parking_lot::Mutex::new(None)),
        }
    }

    /// Set the sensory agent manager (for SHM I/O coordination)
    pub fn set_sensory_agent_manager(&self, manager: Arc<std::sync::Mutex<feagi_burst_engine::AgentManager>>) {
        *self.sensory_agent_manager.lock() = Some(manager);
        println!("🦀 [REGISTRATION] Sensory agent manager connected");
    }

    /// Set callback for agent registration events (for Python integration)
    pub fn set_on_agent_registered<F>(&self, callback: F)
    where
        F: Fn(String, String, String) + Send + Sync + 'static,
    {
        *self.on_agent_registered.lock() = Some(Box::new(callback));
        println!("🦀 [REGISTRATION] Agent registration callback set");
    }

    /// Set callback for agent deregistration events (for Python integration)
    pub fn set_on_agent_deregistered<F>(&self, callback: F)
    where
        F: Fn(String) + Send + Sync + 'static,
    {
        *self.on_agent_deregistered.lock() = Some(Box::new(callback));
        println!("🦀 [REGISTRATION] Agent deregistration callback set");
    }

    /// Process a registration request
    pub fn process_registration(
        &self,
        request: RegistrationRequest,
    ) -> Result<RegistrationResponse, String> {
        println!(
            "🦀 [REGISTRATION] Processing registration for agent: {} (type: {})",
            request.agent_id, request.agent_type
        );

        // Parse capabilities
        let capabilities = self.parse_capabilities(&request.capabilities)?;

        // Allocate SHM paths if needed
        let mut shm_paths = HashMap::new();
        let mut allocated_capabilities = capabilities.clone();

        if let Some(ref mut sensory) = allocated_capabilities.sensory {
            let shm_path = format!(
                "{}/feagi-shm-{}-sensory.bin",
                self.shm_base_path, request.agent_id
            );
            sensory.shm_path = Some(shm_path.clone());
            shm_paths.insert("sensory".to_string(), shm_path);
        }

        if let Some(ref mut motor) = allocated_capabilities.motor {
            let shm_path = format!(
                "{}/feagi-shm-{}-motor.bin",
                self.shm_base_path, request.agent_id
            );
            motor.shm_path = Some(shm_path.clone());
            shm_paths.insert("motor".to_string(), shm_path);
        }

        if let Some(ref mut viz) = allocated_capabilities.visualization {
            let shm_path = format!("{}/feagi-shared-mem-visualization_stream.bin", self.shm_base_path);
            viz.shm_path = Some(shm_path.clone());
            shm_paths.insert("visualization".to_string(), shm_path);
        }

        // Determine transport
        let transport = if !shm_paths.is_empty() {
            AgentTransport::Hybrid
        } else {
            AgentTransport::Zmq
        };

        // Create agent info
        let agent_info = AgentInfo {
            agent_id: request.agent_id.clone(),
            agent_type: request.agent_type.clone(),
            capabilities: allocated_capabilities,
            registered_at: Instant::now(),
            last_heartbeat: Instant::now(),
            transport,
        };

        // Register in registry
        self.agent_registry
            .write()
            .register(agent_info.clone())
            .map_err(|e| format!("Failed to register agent: {}", e))?;

        // Register with burst engine's sensory agent manager (if sensory capability exists)
        if let Some(ref sensory) = agent_info.capabilities.sensory {
            if let Some(sensory_mgr_lock) = self.sensory_agent_manager.lock().as_ref() {
                if let Some(shm_path) = &sensory.shm_path {
                    println!(
                        "🦀 [REGISTRATION] Registering {} with burst engine: {} @ {}Hz",
                        request.agent_id, shm_path, sensory.rate_hz
                    );
                    
                    let sensory_mgr = sensory_mgr_lock.lock().unwrap();
                    let config = feagi_burst_engine::AgentConfig {
                        agent_id: request.agent_id.clone(),
                        shm_path: std::path::PathBuf::from(shm_path),
                        rate_hz: sensory.rate_hz,
                        area_mapping: sensory.cortical_mappings.clone(),
                    };
                    sensory_mgr.register_agent(config)
                        .map_err(|e| format!("Failed to register with burst engine: {}", e))?;
                    
                    println!("🦀 [REGISTRATION] ✅ Agent {} registered with burst engine", request.agent_id);
                } else {
                    println!("🦀 [REGISTRATION] ⚠️  Sensory capability exists but no SHM path");
                }
            } else {
                println!("🦀 [REGISTRATION] ⚠️  Sensory agent manager not connected - skipping burst engine registration");
            }
        }

        // Invoke Python callback if set
        if let Some(ref callback) = *self.on_agent_registered.lock() {
            // Serialize capabilities to JSON string for Python
            let caps_json = serde_json::to_string(&request.capabilities)
                .unwrap_or_else(|_| "{}".to_string());
            
            println!("🦀 [REGISTRATION] Invoking Python callback for agent: {}", request.agent_id);
            callback(request.agent_id.clone(), request.agent_type.clone(), caps_json);
        }

        // Return success response
        Ok(RegistrationResponse {
            status: "success".to_string(),
            message: Some(format!("Agent {} registered successfully", request.agent_id)),
            shm_paths: if shm_paths.is_empty() {
                None
            } else {
                Some(shm_paths)
            },
            zmq_ports: Some(HashMap::from([
                ("motor".to_string(), 30005),
                ("visualization".to_string(), 30000),
            ])),
        })
    }

    /// Parse capabilities from JSON
    fn parse_capabilities(
        &self,
        caps_json: &serde_json::Value,
    ) -> Result<AgentCapabilities, String> {
        let mut capabilities = AgentCapabilities {
            sensory: None,
            motor: None,
            visualization: None,
        };

        // Parse sensory capability
        if let Some(sensory) = caps_json.get("sensory") {
            if let Some(rate_hz) = sensory.get("rate_hz").and_then(|v| v.as_f64()) {
                capabilities.sensory = Some(SensoryCapability {
                    rate_hz,
                    shm_path: None, // Will be allocated
                    cortical_mappings: HashMap::new(), // TODO: Extract from genome
                });
            }
        }

        // Parse motor capability
        if let Some(motor) = caps_json.get("motor") {
            if motor.get("enabled").and_then(|v| v.as_bool()).unwrap_or(false) {
                let rate_hz = motor.get("rate_hz").and_then(|v| v.as_f64()).unwrap_or(20.0);
                capabilities.motor = Some(MotorCapability {
                    rate_hz,
                    shm_path: None, // Will be allocated
                });
            }
        }

        // Parse visualization capability
        if let Some(viz) = caps_json.get("visualization") {
            if viz.get("enabled").and_then(|v| v.as_bool()).unwrap_or(false) {
                let rate_hz = viz.get("rate_hz").and_then(|v| v.as_f64()).unwrap_or(30.0);
                capabilities.visualization = Some(VizCapability {
                    rate_hz,
                    shm_path: None, // Will be allocated
                });
            }
        }

        Ok(capabilities)
    }

    /// Process deregistration request
    pub fn process_deregistration(&self, agent_id: &str) -> Result<String, String> {
        // Deregister from burst engine first
        if let Some(sensory_mgr_lock) = self.sensory_agent_manager.lock().as_ref() {
            let sensory_mgr = sensory_mgr_lock.lock().unwrap();
            if let Err(e) = sensory_mgr.deregister_agent(agent_id) {
                println!("🦀 [REGISTRATION] ⚠️  Failed to deregister {} from burst engine: {}", agent_id, e);
            } else {
                println!("🦀 [REGISTRATION] ✅ Agent {} deregistered from burst engine", agent_id);
            }
        }
        
        // Deregister from registry
        let result = self.agent_registry
            .write()
            .deregister(agent_id)
            .map(|_| format!("Agent {} deregistered", agent_id));
        
        // Invoke Python callback if deregistration was successful
        if result.is_ok() {
            if let Some(ref callback) = *self.on_agent_deregistered.lock() {
                println!("🦀 [REGISTRATION] Invoking Python deregistration callback for agent: {}", agent_id);
                callback(agent_id.to_string());
            }
        }
        
        result
    }

    /// Process heartbeat
    pub fn process_heartbeat(&self, agent_id: &str) -> Result<String, String> {
        self.agent_registry
            .write()
            .heartbeat(agent_id)
            .map(|_| format!("Heartbeat recorded for {}", agent_id))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_registration_handler() {
        let registry = Arc::new(RwLock::new(AgentRegistry::new()));
        let handler = RegistrationHandler::new(registry.clone());

        let request = RegistrationRequest {
            agent_id: "test-agent".to_string(),
            agent_type: "external".to_string(),
            capabilities: serde_json::json!({
                "sensory": {"rate_hz": 30.0},
                "motor": {"enabled": true, "rate_hz": 20.0}
            }),
        };

        let response = handler.process_registration(request).unwrap();
        assert_eq!(response.status, "success");
        assert!(response.shm_paths.is_some());

        assert_eq!(registry.read().count(), 1);
    }
}

