//! Python bindings for FEAGI Agent SDK
//!
//! This crate provides Python bindings for the Rust-based FEAGI Agent SDK.
//! It wraps the core Rust functionality and exposes it as a Python module.

use pyo3::prelude::*;
use pyo3::exceptions::{PyException, PyValueError, PyRuntimeError};
use feagi_agent_sdk::{AgentClient, AgentConfig, AgentType as RustAgentType};
use std::sync::{Arc, Mutex};

/// Python wrapper for AgentType
#[pyclass(eq, eq_int)]
#[derive(Clone, Copy, PartialEq)]
pub enum AgentType {
    Sensory,
    Motor,
    Both,
}

impl From<AgentType> for RustAgentType {
    fn from(py_type: AgentType) -> Self {
        match py_type {
            AgentType::Sensory => RustAgentType::Sensory,
            AgentType::Motor => RustAgentType::Motor,
            AgentType::Both => RustAgentType::Both,
        }
    }
}

/// Python wrapper for AgentConfig
///
/// This provides a builder-style API for configuring agents from Python.
///
/// # Example (Python)
/// ```python
/// from feagi_agent_sdk_py import AgentConfig, AgentType
///
/// config = AgentConfig("my_camera", AgentType.Sensory)
/// config.with_feagi_host("localhost")
/// config.with_vision_capability("camera", 640, 480, 3, "i_vision")
/// config.with_heartbeat_interval(5.0)
/// ```
#[pyclass]
#[derive(Clone)]
pub struct PyAgentConfig {
    config: AgentConfig,
}

#[pymethods]
impl PyAgentConfig {
    /// Create a new agent configuration
    ///
    /// # Arguments
    /// * `agent_id` - Unique identifier for this agent
    /// * `agent_type` - Type of agent (Sensory, Motor, or Both)
    #[new]
    fn new(agent_id: String, agent_type: AgentType) -> Self {
        Self {
            config: AgentConfig::new(agent_id, agent_type.into()),
        }
    }
    
    /// Set FEAGI host (derives all endpoints from host)
    ///
    /// # Example
    /// ```python
    /// config.with_feagi_host("192.168.1.100")
    /// ```
    fn with_feagi_host(&mut self, host: String) {
        // Clone, apply transform, replace
        let new_config = self.config.clone().with_feagi_host(host);
        self.config = new_config;
    }
    
    /// Set registration endpoint
    fn with_registration_endpoint(&mut self, endpoint: String) {
        let new_config = self.config.clone().with_registration_endpoint(endpoint);
        self.config = new_config;
    }
    
    /// Set sensory input endpoint
    fn with_sensory_endpoint(&mut self, endpoint: String) {
        let new_config = self.config.clone().with_sensory_endpoint(endpoint);
        self.config = new_config;
    }
    
    /// Set motor output endpoint
    fn with_motor_endpoint(&mut self, endpoint: String) {
        let new_config = self.config.clone().with_motor_endpoint(endpoint);
        self.config = new_config;
    }
    
    /// Set heartbeat interval in seconds (0 to disable)
    fn with_heartbeat_interval(&mut self, interval: f64) {
        let new_config = self.config.clone().with_heartbeat_interval(interval);
        self.config = new_config;
    }
    
    /// Set connection timeout in milliseconds
    fn with_connection_timeout_ms(&mut self, timeout_ms: u64) {
        let new_config = self.config.clone().with_connection_timeout_ms(timeout_ms);
        self.config = new_config;
    }
    
    /// Set registration retry attempts
    fn with_registration_retries(&mut self, retries: u32) {
        let new_config = self.config.clone().with_registration_retries(retries);
        self.config = new_config;
    }
    
    /// Add vision capability
    ///
    /// # Arguments
    /// * `modality` - Type of vision sensor (e.g., "camera", "lidar")
    /// * `width` - Frame width
    /// * `height` - Frame height
    /// * `channels` - Number of channels (1=grayscale, 3=RGB)
    /// * `target_cortical_area` - Target cortical area ID
    fn with_vision_capability(
        &mut self,
        modality: String,
        width: usize,
        height: usize,
        channels: usize,
        target_cortical_area: String,
    ) {
        let new_config = self.config.clone().with_vision_capability(
            modality,
            (width, height),
            channels,
            target_cortical_area,
        );
        self.config = new_config;
    }
    
    /// Add motor capability
    ///
    /// # Arguments
    /// * `modality` - Type of motor (e.g., "servo", "stepper")
    /// * `output_count` - Number of motor outputs
    /// * `source_cortical_areas` - List of source cortical area IDs
    fn with_motor_capability(
        &mut self,
        modality: String,
        output_count: usize,
        source_cortical_areas: Vec<String>,
    ) {
        let new_config = self.config.clone().with_motor_capability(
            modality,
            output_count,
            source_cortical_areas,
        );
        self.config = new_config;
    }
    
    /// Add custom capability
    ///
    /// # Arguments
    /// * `key` - Capability key
    /// * `value_json` - JSON string with capability data
    fn with_custom_capability(&mut self, key: String, value_json: String) -> PyResult<()> {
        let value: serde_json::Value = serde_json::from_str(&value_json)
            .map_err(|e| PyValueError::new_err(format!("Invalid JSON: {}", e)))?;
        let new_config = self.config.clone().with_custom_capability(key, value);
        self.config = new_config;
        Ok(())
    }
    
    /// Validate configuration
    fn validate(&self) -> PyResult<()> {
        self.config.validate()
            .map_err(|e| PyValueError::new_err(format!("Configuration validation failed: {}", e)))
    }
}

/// Python wrapper for AgentClient
///
/// This is the main interface for building FEAGI agents in Python.
///
/// # Example (Python)
/// ```python
/// from feagi_agent_sdk_py import PyAgentClient, PyAgentConfig, AgentType
///
/// # Create configuration
/// config = PyAgentConfig("my_camera", AgentType.Sensory)
/// config.with_feagi_host("localhost")
/// config.with_vision_capability("camera", 640, 480, 3, "i_vision")
///
/// # Create and connect client
/// client = PyAgentClient(config)
/// client.connect()
///
/// # Send sensory data
/// client.send_sensory_data([(0, 50.0), (1, 75.0)])
///
/// # Client auto-deregisters when garbage collected
/// ```
#[pyclass]
pub struct PyAgentClient {
    client: Arc<Mutex<AgentClient>>,
}

#[pymethods]
impl PyAgentClient {
    /// Create a new FEAGI agent client
    ///
    /// # Arguments
    /// * `config` - Agent configuration (PyAgentConfig)
    #[new]
    fn new(config: PyAgentConfig) -> PyResult<Self> {
        let client = AgentClient::new(config.config)
            .map_err(|e| PyException::new_err(format!("Failed to create agent client: {}", e)))?;
        
        Ok(Self {
            client: Arc::new(Mutex::new(client)),
        })
    }
    
    /// Connect to FEAGI and register the agent
    ///
    /// This will:
    /// 1. Create ZMQ sockets
    /// 2. Register with FEAGI
    /// 3. Start heartbeat service
    fn connect(&self) -> PyResult<()> {
        let mut client = self.client.lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Lock error: {}", e)))?;
        
        client.connect()
            .map_err(|e| PyException::new_err(format!("Connection failed: {}", e)))
    }
    
    /// Send sensory data to FEAGI
    ///
    /// # Arguments
    /// * `neuron_pairs` - List of (neuron_id, potential) tuples
    ///
    /// # Example
    /// ```python
    /// client.send_sensory_data([
    ///     (0, 50.0),
    ///     (1, 75.0),
    ///     (2, 30.0),
    /// ])
    /// ```
    fn send_sensory_data(&self, neuron_pairs: Vec<(i32, f64)>) -> PyResult<()> {
        let client = self.client.lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Lock error: {}", e)))?;
        
        client.send_sensory_data(neuron_pairs)
            .map_err(|e| PyException::new_err(format!("Failed to send sensory data: {}", e)))
    }
    
    /// Receive motor data from FEAGI (non-blocking)
    ///
    /// Returns None if no data is available.
    ///
    /// # Example
    /// ```python
    /// motor_data = client.receive_motor_data()
    /// if motor_data is not None:
    ///     print(f"Motor data: {motor_data}")
    /// ```
    fn receive_motor_data(&self) -> PyResult<Option<String>> {
        let client = self.client.lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Lock error: {}", e)))?;
        
        match client.receive_motor_data() {
            Ok(Some(data)) => {
                let json_str = serde_json::to_string(&data)
                    .map_err(|e| PyException::new_err(format!("Failed to serialize motor data: {}", e)))?;
                Ok(Some(json_str))
            }
            Ok(None) => Ok(None),
            Err(e) => Err(PyException::new_err(format!("Failed to receive motor data: {}", e))),
        }
    }
    
    /// Check if agent is registered
    fn is_registered(&self) -> PyResult<bool> {
        let client = self.client.lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Lock error: {}", e)))?;
        Ok(client.is_registered())
    }
    
    /// Get agent ID
    fn agent_id(&self) -> PyResult<String> {
        let client = self.client.lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Lock error: {}", e)))?;
        Ok(client.agent_id().to_string())
    }
}

/// Python module definition
#[pymodule]
fn feagi_agent_sdk_py(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Add classes
    m.add_class::<AgentType>()?;
    m.add_class::<PyAgentConfig>()?;
    m.add_class::<PyAgentClient>()?;
    
    // Add version
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    
    Ok(())
}

