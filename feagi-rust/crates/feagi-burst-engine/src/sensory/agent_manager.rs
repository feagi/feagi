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
//! 2. Decode using feagi_data_serialization
//! 3. Extract neuron IDs
//! 4. Inject directly into FCL

use super::{ShmReader, RateLimiter};
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
/// Parameters: cortical_area, coordinates with potentials (x,y,z,p tuples)
pub type FclInjectionCallback = Arc<dyn Fn(u32, Vec<(u32, u32, u32, f32)>) + Send + Sync>;

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
    println!("[SENSORY-{}] Thread started, attempting to open SHM at {:?}", config.agent_id, config.shm_path);
    
    // Open SHM reader with retry logic (agent may not have written data yet)
    let mut reader = {
        let max_retries = 50;  // ~5 seconds total wait time (agent needs time to create file after registration)
        let mut last_error = None;
        let mut reader_opt = None;
        
        for attempt in 1..=max_retries {
            match ShmReader::open(&config.shm_path) {
                Ok(r) => {
                    println!("[SENSORY-{}] Successfully opened SHM on attempt {}", config.agent_id, attempt);
                    reader_opt = Some(r);
                    break;
                }
                Err(e) => {
                    last_error = Some(e.to_string());
                    if attempt == 1 || attempt % 10 == 0 {
                        println!("[SENSORY-{}] Retry {}/{}: {}", config.agent_id, attempt, max_retries, e);
                    }
                    if attempt < max_retries {
                        // Sleep 100ms before retry
                        thread::sleep(std::time::Duration::from_millis(100));
                    }
                }
            }
        }
        
        match reader_opt {
            Some(r) => r,
            None => {
                eprintln!("[SENSORY-{}] Failed to open SHM after {} retries: {}", 
                    config.agent_id, max_retries, last_error.unwrap_or_else(|| "unknown error".to_string()));
                return;
            }
        }
    };
    
    // Create rate limiter
    let mut rate_limiter = RateLimiter::new(config.rate_hz);
    
    println!("[SENSORY-{}] Polling started at {:.1} Hz from {:?}", 
             config.agent_id, config.rate_hz, config.shm_path);
    
    // Polling loop
    let mut poll_count = 0;
    while !stop_flag.load(Ordering::Acquire) {
        poll_count += 1;
        if poll_count == 1 || poll_count % 100 == 0 {
            println!("[SENSORY-{}] Poll #{}: attempting read...", config.agent_id, poll_count);
        }
        
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
            Some(data) => {
                // Log first successful read only (reduce spam)
                static FIRST_READ_LOGGED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
                if !FIRST_READ_LOGGED.load(Ordering::Relaxed) {
                    println!("[SENSORY-{}] ✅ First SHM read: {} bytes", config.agent_id, data.data.len());
                    FIRST_READ_LOGGED.store(true, Ordering::Relaxed);
                }
                data
            }
            None => {
                // No new data
                static NO_DATA_COUNT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
                let count = NO_DATA_COUNT.fetch_add(1, Ordering::Relaxed);
                if count < 5 || count % 1000 == 0 {
                    println!("[SENSORY-{}] read_latest() returned None (count={})", config.agent_id, count);
                }
                thread::sleep(std::time::Duration::from_millis(1));
                continue;
            }
        };
        
        // Decode using feagi_data_serialization
        let mut byte_container = feagi_data_serialization::FeagiByteContainer::new_empty();
        let mut data_vec = slot_data.data.to_vec();
        
        // 🔍 DEBUG: Log first 20 bytes to diagnose format mismatch
        static FIRST_BYTES_LOGGED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
        if !FIRST_BYTES_LOGGED.load(Ordering::Relaxed) && data_vec.len() >= 20 {
            eprintln!("[SENSORY-{}] 🔍 First 20 bytes: {:?}", config.agent_id, &data_vec[0..20]);
            eprintln!("[SENSORY-{}]    byte[0] (version): {}", config.agent_id, data_vec[0]);
            eprintln!("[SENSORY-{}]    byte[1-2] (increment): {:?}", config.agent_id, &data_vec[1..3]);
            eprintln!("[SENSORY-{}]    byte[3] (struct_count): {}", config.agent_id, data_vec[3]);
            FIRST_BYTES_LOGGED.store(true, Ordering::Relaxed);
        }
        
        if let Err(e) = byte_container.try_write_data_to_container_and_verify(&mut |bytes| {
            std::mem::swap(bytes, &mut data_vec);
            Ok(())
        }) {
            eprintln!("[SENSORY-{}] ❌ Failed to load bytes: {:?}", config.agent_id, e);
            eprintln!("[SENSORY-{}]    Total bytes loaded: {}", config.agent_id, byte_container.get_number_of_bytes_used());
            eprintln!("[SENSORY-{}]    Container valid: {}", config.agent_id, byte_container.is_valid());
            continue;
        }
        
        eprintln!("[SENSORY-{}] ✅ Bytes loaded successfully, container is valid", config.agent_id);
        eprintln!("[SENSORY-{}] 🔍 Pre-check: is_valid()={}, bytes.len()={}", 
            config.agent_id, byte_container.is_valid(), byte_container.get_number_of_bytes_used());
        
        let num_structures = match byte_container.try_get_number_contained_structures() {
            Ok(n) => {
                eprintln!("[SENSORY-{}] ✅ Got struct count: {}", config.agent_id, n);
                n
            },
            Err(e) => {
                eprintln!("[SENSORY-{}] ❌ Failed to get structure count: {:?}", config.agent_id, e);
                eprintln!("[SENSORY-{}]    Post-error: is_valid()={}, bytes.len()={}", 
                    config.agent_id, byte_container.is_valid(), byte_container.get_number_of_bytes_used());
                continue;
            }
        };
        
        // Log first successful decode
        static FIRST_DECODE_LOGGED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
        if !FIRST_DECODE_LOGGED.load(Ordering::Relaxed) {
            println!("[SENSORY-{}] ✅ First decode: {} structures", config.agent_id, num_structures);
            FIRST_DECODE_LOGGED.store(true, Ordering::Relaxed);
        }
        
        // Extract neuron data from each structure
        for struct_idx in 0..num_structures {
            let boxed_struct = match byte_container.try_create_new_struct_from_index(struct_idx as u8) {
                Ok(s) => s,
                Err(e) => {
                    eprintln!("[SENSORY-{}] Failed to extract structure {}: {:?}", config.agent_id, struct_idx, e);
                    continue;
                }
            };
            
            // Downcast to CorticalMappedXYZPNeuronData
            use feagi_data_structures::neuron_voxels::xyzp::CorticalMappedXYZPNeuronVoxels;
            let cortical_mapped = match boxed_struct.as_any().downcast_ref::<CorticalMappedXYZPNeuronVoxels>() {
                Some(cm) => cm,
                None => {
                    eprintln!("[SENSORY-{}] Structure {} is not CorticalMappedXYZPNeuronData", config.agent_id, struct_idx);
                    continue;
                }
            };
            
            // Iterate over cortical areas
            for (cortical_id, neuron_arrays) in &cortical_mapped.mappings {
                let area_id = cortical_id.as_ascii_string();
                
                // Get cortical_idx for this area
                let cortical_idx = match config.area_mapping.get(&area_id) {
                    Some(&idx) => idx,
                    None => {
                        eprintln!("[SENSORY-{}] Unknown area '{}'", config.agent_id, area_id);
                        continue;
                    }
                };
                
                // Extract (x,y,z,p) coordinates with potentials and pass to callback
                // Callback will use NPU's spatial hash to convert → neuron_id
                let (x_coords, y_coords, z_coords, potentials) = neuron_arrays.borrow_xyzp_vectors();
                
                // Build coordinate+potential tuples (XYZP)
                let xyzp_data: Vec<(u32, u32, u32, f32)> = x_coords.iter()
                    .zip(y_coords.iter())
                    .zip(z_coords.iter())
                    .zip(potentials.iter())
                    .map(|(((x, y), z), p)| (*x, *y, *z, *p))
                    .collect();
                
                // 🔍 DEBUG: Log coordinate and potential distribution
                static FIRST_COORD_DEBUG: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
                if !FIRST_COORD_DEBUG.load(Ordering::Relaxed) && xyzp_data.len() >= 5 {
                    eprintln!("[SENSORY-{}] 🔍 First 5 XYZP: {:?}", config.agent_id, &xyzp_data[0..5]);
                    FIRST_COORD_DEBUG.store(true, Ordering::Relaxed);
                }
                
                // Log first injection
                static FIRST_INJECTION_LOGGED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
                if !FIRST_INJECTION_LOGGED.load(Ordering::Relaxed) && !xyzp_data.is_empty() {
                    println!("[SENSORY-{}] ✅ First injection: area='{}', cortical_area={}, neuron_count={}", 
                        config.agent_id, area_id, cortical_idx, xyzp_data.len());
                    println!("[SENSORY-{}]    First 3 XYZP: {:?}",
                        config.agent_id,
                        &xyzp_data[0..xyzp_data.len().min(3)]);
                    FIRST_INJECTION_LOGGED.store(true, Ordering::Relaxed);
                }
                
                if !xyzp_data.is_empty() {
                    // Inject into FCL via callback (callback will do coordinate → neuron_id conversion)
                    injection_callback(cortical_idx, xyzp_data);
                }
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
        let callback = Arc::new(|_cortical_area: u32, _xyzp_data: Vec<(u32, u32, u32, f32)>| {
            // No-op for testing
        });
        
        let manager = AgentManager::new(callback);
        
        // Initially no agents
        assert_eq!(manager.agent_count(), 0);
        
        // Note: Can't fully test registration without creating actual SHM files
        // This would require integration tests
    }
}

