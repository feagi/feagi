// FEAGI Peripheral Nervous System (PNS)
// Handles all agent I/O: registration, ZMQ, SHM, heartbeat tracking

use parking_lot::{Mutex, RwLock};
use std::sync::Arc;
use std::thread;
use thiserror::Error;

pub mod agent_registry;
pub mod registration;
pub mod heartbeat;
pub mod zmq;
pub mod shm;

pub use agent_registry::{AgentRegistry, AgentInfo, AgentCapabilities};
pub use registration::RegistrationHandler;
pub use heartbeat::HeartbeatTracker;
pub use zmq::{ZmqStreams, RestStream, MotorStream, VisualizationStream};

#[derive(Error, Debug)]
pub enum PNSError {
    #[error("ZMQ error: {0}")]
    Zmq(String),
    #[error("SHM error: {0}")]
    Shm(String),
    #[error("Agent error: {0}")]
    Agent(String),
    #[error("Registration error: {0}")]
    Registration(String),
    #[error("Not running: {0}")]
    NotRunning(String),
}

pub type Result<T> = std::result::Result<T, PNSError>;

/// Configuration for PNS
#[derive(Debug, Clone)]
pub struct PNSConfig {
    pub zmq_rest_address: String,
    pub zmq_motor_address: String,
    pub zmq_viz_address: String,
    pub shm_base_path: String,
}

impl Default for PNSConfig {
    fn default() -> Self {
        Self {
            zmq_rest_address: "tcp://0.0.0.0:5555".to_string(),
            zmq_motor_address: "tcp://0.0.0.0:30005".to_string(),
            zmq_viz_address: "tcp://0.0.0.0:30000".to_string(),
            shm_base_path: "/tmp".to_string(),
        }
    }
}

/// Main PNS - manages all agent I/O
pub struct PNS {
    config: PNSConfig,
    agent_registry: Arc<RwLock<AgentRegistry>>,
    registration_handler: Arc<Mutex<RegistrationHandler>>,
    heartbeat_tracker: Arc<Mutex<HeartbeatTracker>>,
    zmq_streams: Arc<Mutex<Option<ZmqStreams>>>,
    running: Arc<RwLock<bool>>,
    /// Optional reference to burst engine's sensory agent manager for SHM I/O
    sensory_agent_manager: Arc<Mutex<Option<Arc<std::sync::Mutex<feagi_burst_engine::AgentManager>>>>>,
}

impl PNS {
    /// Create a new PNS with default configuration
    pub fn new() -> Result<Self> {
        Self::with_config(PNSConfig::default())
    }

    /// Create a new PNS with custom configuration
    pub fn with_config(config: PNSConfig) -> Result<Self> {
        let agent_registry = Arc::new(RwLock::new(AgentRegistry::new()));
        let heartbeat_tracker = Arc::new(Mutex::new(HeartbeatTracker::new()));
        let registration_handler = Arc::new(Mutex::new(
            RegistrationHandler::new(Arc::clone(&agent_registry))
        ));
        
        Ok(Self {
            config,
            agent_registry,
            registration_handler,
            heartbeat_tracker,
            zmq_streams: Arc::new(Mutex::new(None)),
            running: Arc::new(RwLock::new(false)),
            sensory_agent_manager: Arc::new(Mutex::new(None)),
        })
    }

    /// Set the sensory agent manager (for SHM I/O coordination)
    /// Should be called before starting the PNS
    pub fn set_sensory_agent_manager(&self, manager: Arc<std::sync::Mutex<feagi_burst_engine::AgentManager>>) {
        *self.sensory_agent_manager.lock() = Some(manager.clone());
        // Also propagate to registration handler
        self.registration_handler.lock().set_sensory_agent_manager(manager);
        println!("🦀 [PNS] Sensory agent manager connected for SHM I/O");
    }

    /// Start all PNS services
    pub fn start(&self) -> Result<()> {
        if *self.running.read() {
            return Err(PNSError::Agent("PNS already running".to_string()));
        }

        println!("🦀 [PNS] Starting FEAGI Peripheral Nervous System...");

        // Start ZMQ streams
        let zmq_streams = ZmqStreams::new(
            &self.config.zmq_rest_address,
            &self.config.zmq_motor_address,
            &self.config.zmq_viz_address,
            Arc::clone(&self.registration_handler),
        )?;

        zmq_streams.start()?;
        *self.zmq_streams.lock() = Some(zmq_streams);

        // Start heartbeat monitoring
        self.heartbeat_tracker.lock().start(Arc::clone(&self.agent_registry));

        *self.running.write() = true;
        println!("🦀 [PNS] ✅ All services started successfully");

        Ok(())
    }

    /// Stop all PNS services
    pub fn stop(&self) -> Result<()> {
        if !*self.running.read() {
            return Ok(());
        }

        println!("🦀 [PNS] Stopping all services...");
        *self.running.write() = false;

        // Stop ZMQ streams
        if let Some(streams) = self.zmq_streams.lock().take() {
            streams.stop()?;
        }

        // Stop heartbeat monitoring
        self.heartbeat_tracker.lock().stop();

        println!("🦀 [PNS] ✅ All services stopped");
        Ok(())
    }

    /// Check if PNS is running
    pub fn is_running(&self) -> bool {
        *self.running.read()
    }

    /// Get agent registry (for external access)
    pub fn get_agent_registry(&self) -> Arc<RwLock<AgentRegistry>> {
        Arc::clone(&self.agent_registry)
    }
}

impl Drop for PNS {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pns_creation() {
        let pns = PNS::new();
        assert!(pns.is_ok());
    }

    #[test]
    fn test_pns_lifecycle() {
        let pns = PNS::new().unwrap();
        assert!(!pns.is_running());
        
        // Note: Can't actually start without conflicting with running FEAGI
        // Real tests require integration testing with Docker
    }
}
