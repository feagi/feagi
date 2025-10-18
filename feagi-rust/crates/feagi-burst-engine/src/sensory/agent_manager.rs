/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # Agent Manager for Sensory Polling
//!
//! Spawns and manages per-agent polling threads that:
//! 1. Read from SHM at agent-requested rate
//! 2. Decode Type 11 cortical format
//! 3. Convert coordinates to neuron IDs
//! 4. Inject directly into FCL

use super::{ShmReader, decode_type11, RateLimiter};
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::sync::atomic::{AtomicBool, Ordering};

/// Agent registration info
#[derive(Debug, Clone)]
pub struct AgentConfig {
    pub agent_id: String,
    pub shm_path: PathBuf,
    pub rate_hz: f64,
    /// Cortical area ID → cortical_idx mapping (for coordinate lookup)
    pub area_mapping: HashMap<String, u32>,
}

/// Callback for injecting decoded sensory data into FCL
/// Parameters: cortical_idx, neuron_ids
pub type FclInjectionCallback = Arc<dyn Fn(u32, Vec<u32>) + Send + Sync>;

/// Thread handle for an agent's polling thread
struct AgentThread {
    agent_id: String,
    handle: JoinHandle<()>,
    stop_flag: Arc<AtomicBool>,
}

/// Manages all agent sensory polling threads
pub struct AgentManager {
    agents: Arc<Mutex<HashMap<String, AgentThread>>>,
    injection_callback: FclInjectionCallback,
}

impl AgentManager {
    /// Create a new agent manager with an FCL injection callback
    pub fn new(injection_callback: FclInjectionCallback) -> Self {
        Self {
            agents: Arc::new(Mutex::new(HashMap::new())),
            injection_callback,
        }
    }
    
    /// Register a new agent and spawn its polling thread
    pub fn register_agent(&self, config: AgentConfig) -> Result<(), String> {
        let agent_id = config.agent_id.clone();
        
        // Check if agent already registered
        {
            let agents = self.agents.lock().unwrap();
            if agents.contains_key(&agent_id) {
                return Err(format!("Agent '{}' already registered", agent_id));
            }
        }
        
        // Create stop flag for this thread
        let stop_flag = Arc::new(AtomicBool::new(false));
        let stop_flag_clone = stop_flag.clone();
        
        // Clone callback
        let injection_callback = self.injection_callback.clone();
        
        // Spawn polling thread
        let handle = thread::Builder::new()
            .name(format!("sensory-{}", agent_id))
            .spawn(move || {
                agent_polling_loop(config, stop_flag_clone, injection_callback);
            })
            .map_err(|e| format!("Failed to spawn thread for '{}': {}", agent_id, e))?;
        
        // Store thread handle
        let thread = AgentThread {
            agent_id: agent_id.clone(),
            handle,
            stop_flag,
        };
        
        {
            let mut agents = self.agents.lock().unwrap();
            agents.insert(agent_id, thread);
        }
        
        Ok(())
    }
    
    /// Deregister an agent and stop its polling thread
    pub fn deregister_agent(&self, agent_id: &str) -> Result<(), String> {
        let thread = {
            let mut agents = self.agents.lock().unwrap();
            agents.remove(agent_id)
        };
        
        match thread {
            Some(mut thread) => {
                // Signal thread to stop
                thread.stop_flag.store(true, Ordering::Release);
                
                // Wait for thread to finish (with timeout)
                // Note: join() consumes the handle, so we can't reuse it
                if thread.handle.join().is_err() {
                    return Err(format!("Thread for '{}' panicked during shutdown", agent_id));
                }
                
                Ok(())
            }
            None => Err(format!("Agent '{}' not registered", agent_id)),
        }
    }
    
    /// Get list of currently registered agents
    pub fn list_agents(&self) -> Vec<String> {
        let agents = self.agents.lock().unwrap();
        agents.keys().cloned().collect()
    }
    
    /// Get count of active agents
    pub fn agent_count(&self) -> usize {
        let agents = self.agents.lock().unwrap();
        agents.len()
    }
}

impl Drop for AgentManager {
    fn drop(&mut self) {
        // Stop all threads on drop
        let agent_ids: Vec<String> = {
            let agents = self.agents.lock().unwrap();
            agents.keys().cloned().collect()
        };
        
        for agent_id in agent_ids {
            let _ = self.deregister_agent(&agent_id);
        }
    }
}

/// Per-agent polling loop (runs in dedicated thread)
fn agent_polling_loop(
    config: AgentConfig,
    stop_flag: Arc<AtomicBool>,
    injection_callback: FclInjectionCallback,
) {
    // Open SHM reader
    let mut reader = match ShmReader::open(&config.shm_path) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("[SENSORY-{}] Failed to open SHM: {}", config.agent_id, e);
            return;
        }
    };
    
    // Create rate limiter
    let mut rate_limiter = RateLimiter::new(config.rate_hz);
    
    println!("[SENSORY-{}] Polling started at {:.1} Hz from {:?}", 
             config.agent_id, config.rate_hz, config.shm_path);
    
    // Polling loop
    while !stop_flag.load(Ordering::Acquire) {
        // Rate limiting
        if !rate_limiter.should_poll_now() {
            // Sleep until next poll time
            if let Some(sleep_duration) = rate_limiter.time_until_next_poll() {
                // Cap sleep to 100ms to ensure responsive shutdown
                let sleep_ms = sleep_duration.as_millis().min(100);
                thread::sleep(std::time::Duration::from_millis(sleep_ms as u64));
            }
            continue;
        }
        
        // Read from SHM
        let slot_data = match reader.read_latest() {
            Some(data) => data,
            None => {
                // No new data, brief sleep
                thread::sleep(std::time::Duration::from_millis(1));
                continue;
            }
        };
        
        // Decode Type 11 format
        let sensory_data = match decode_type11(&slot_data.data) {
            Ok(data) => data,
            Err(e) => {
                eprintln!("[SENSORY-{}] Decode error: {}", config.agent_id, e);
                continue;
            }
        };
        
        // Process each cortical area
        for area in sensory_data.areas {
            // Get cortical_idx for this area
            let cortical_idx = match config.area_mapping.get(&area.area_id) {
                Some(&idx) => idx,
                None => {
                    eprintln!("[SENSORY-{}] Unknown area '{}'", config.agent_id, area.area_id);
                    continue;
                }
            };
            
            // For now, we'll inject coordinates as-is
            // In Phase 2, we'll add coordinate→neuronID lookup via NPU spatial hash
            // For MVP, assume coords are already neuron IDs (or do batch lookup externally)
            
            // Convert coordinates to neuron IDs (placeholder - needs NPU integration)
            // TODO: Add batch coordinate lookup via NPU.get_neurons_at_coordinates_batch()
            let neuron_ids: Vec<u32> = area.coords_x.iter().copied().collect();
            
            if !neuron_ids.is_empty() {
                // Inject into FCL via callback
                injection_callback(cortical_idx, neuron_ids);
            }
        }
    }
    
    println!("[SENSORY-{}] Polling stopped", config.agent_id);
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::time::Duration;
    
    #[test]
    fn test_agent_manager_lifecycle() {
        // Create a dummy injection callback
        let callback = Arc::new(|_cortical_idx: u32, _neuron_ids: Vec<u32>| {
            // No-op for testing
        });
        
        let manager = AgentManager::new(callback);
        
        // Initially no agents
        assert_eq!(manager.agent_count(), 0);
        
        // Note: Can't fully test registration without creating actual SHM files
        // This would require integration tests
    }
}

