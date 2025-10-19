/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # Pure Rust Burst Loop Runner
//!
//! Runs the burst processing loop in a dedicated thread with NO Python overhead.
//!
//! ## Design
//! - Runs in native Rust thread (no GIL contention)
//! - Zero FFI crossings in hot path
//! - Adaptive timing for RTOS-like precision
//! - Power neurons injected every burst
//! - Sensory neurons injected by separate threads directly into FCL

use crate::RustNPU;
use crate::sensory::AgentManager;
use feagi_types::NeuronId;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Duration, Instant};
use std::thread;

/// Burst loop runner - manages the main neural processing loop
/// 
/// 🦀 Power neurons are stored in RustNPU, not here - 100% Rust!
pub struct BurstLoopRunner {
    /// Shared NPU instance (holds power neurons internally)
    npu: Arc<Mutex<RustNPU>>,
    /// Target frequency in Hz
    frequency_hz: f64,
    /// Running flag (atomic for thread-safe stop)
    running: Arc<AtomicBool>,
    /// Burst counter (atomic for thread-safe read)
    burst_count: Arc<AtomicU64>,
    /// Thread handle (for graceful shutdown)
    thread_handle: Option<thread::JoinHandle<()>>,
    /// Sensory agent manager (per-agent injection threads)
    pub sensory_manager: Arc<Mutex<AgentManager>>,
    /// Visualization SHM writer (optional, None if not configured)
    pub viz_shm_writer: Arc<Mutex<Option<crate::viz_shm_writer::VizSHMWriter>>>,
    /// Motor SHM writer (optional, None if not configured)
    pub motor_shm_writer: Arc<Mutex<Option<crate::motor_shm_writer::MotorSHMWriter>>>,
}

impl BurstLoopRunner {
    /// Create a new burst loop runner
    pub fn new(npu: Arc<Mutex<RustNPU>>, frequency_hz: f64) -> Self {
        // Create FCL injection callback for sensory data
        let npu_for_callback = npu.clone();
        let injection_callback = Arc::new(move |cortical_area: u32, xyzp_data: Vec<(u32, u32, u32, f32)>| {
            // 🔍 DEBUG: Log first injection
            static FIRST_CALLBACK_LOGGED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
            if !FIRST_CALLBACK_LOGGED.load(std::sync::atomic::Ordering::Relaxed) && !xyzp_data.is_empty() {
                println!("[FCL-INJECT] 🔍 First callback: cortical_area={}, neuron_count={}", cortical_area, xyzp_data.len());
                println!("[FCL-INJECT]    First 3 XYZP: {:?}", &xyzp_data[0..xyzp_data.len().min(3)]);
                FIRST_CALLBACK_LOGGED.store(true, std::sync::atomic::Ordering::Relaxed);
            }
            
            // Convert (x,y,z) to neuron IDs and inject with actual P values
            if let Ok(mut npu_lock) = npu_for_callback.lock() {
                // Extract coordinates for batch lookup
                let coords: Vec<(u32, u32, u32)> = xyzp_data.iter()
                    .map(|(x, y, z, _)| (*x, *y, *z))
                    .collect();
                
                // Batch coordinate lookup
                let neuron_ids = npu_lock.neuron_array.batch_coordinate_lookup(cortical_area, &coords);
                
                // 🔍 DEBUG: Log conversion result
                static FIRST_CONVERSION_LOGGED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
                if !FIRST_CONVERSION_LOGGED.load(std::sync::atomic::Ordering::Relaxed) && !neuron_ids.is_empty() {
                    println!("[FCL-INJECT]    Converted {} coords → {} valid neurons", xyzp_data.len(), neuron_ids.len());
                    println!("[FCL-INJECT]    First 5 neuron IDs: {:?}", &neuron_ids[0..neuron_ids.len().min(5)]);
                    FIRST_CONVERSION_LOGGED.store(true, std::sync::atomic::Ordering::Relaxed);
                }
                
                // Build (NeuronId, potential) pairs from XYZP data
                let mut neuron_potential_pairs: Vec<(NeuronId, f32)> = Vec::with_capacity(xyzp_data.len());
                for (x, y, z, p) in xyzp_data.iter() {
                    if let Some(neuron_id) = npu_lock.neuron_array.get_neuron_at_coordinate(cortical_area, *x, *y, *z) {
                        neuron_potential_pairs.push((neuron_id, *p));
                    }
                }
                
                // 🔍 DEBUG: Log first few potentials
                static FIRST_POTENTIALS_LOGGED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
                if !FIRST_POTENTIALS_LOGGED.load(std::sync::atomic::Ordering::Relaxed) && !neuron_potential_pairs.is_empty() {
                    println!("[FCL-INJECT]    First 5 potentials from data:");
                    for (idx, (neuron_id, p)) in neuron_potential_pairs.iter().take(5).enumerate() {
                        println!("[FCL-INJECT]      [{:?}] p={:.3}", neuron_id, p);
                    }
                    FIRST_POTENTIALS_LOGGED.store(true, std::sync::atomic::Ordering::Relaxed);
                }
                
                // Inject with individual potentials
                npu_lock.inject_sensory_with_potentials(&neuron_potential_pairs);
                
                // 🔍 DEBUG: Log injection summary
                static FIRST_SUMMARY_LOGGED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
                if !FIRST_SUMMARY_LOGGED.load(std::sync::atomic::Ordering::Relaxed) {
                    println!("[FCL-INJECT]    ✅ Injected {} neurons with actual P values from data", neuron_potential_pairs.len());
                    FIRST_SUMMARY_LOGGED.store(true, std::sync::atomic::Ordering::Relaxed);
                }
            }
        });
        
        let sensory_manager = AgentManager::new(injection_callback);
        Self {
            npu,
            frequency_hz,
            running: Arc::new(AtomicBool::new(false)),
            burst_count: Arc::new(AtomicU64::new(0)),
            thread_handle: None,
            sensory_manager: Arc::new(Mutex::new(sensory_manager)),
            viz_shm_writer: Arc::new(Mutex::new(None)), // Initialized later via attach_viz_shm_writer
            motor_shm_writer: Arc::new(Mutex::new(None)), // Initialized later via attach_motor_shm_writer
        }
    }
    
    /// Attach visualization SHM writer (called from Python after registration)
    pub fn attach_viz_shm_writer(&mut self, shm_path: std::path::PathBuf) -> Result<(), std::io::Error> {
        let writer = crate::viz_shm_writer::VizSHMWriter::new(shm_path, None, None)?;
        let mut guard = self.viz_shm_writer.lock().unwrap();
        *guard = Some(writer);
        Ok(())
    }
    
    /// Attach motor SHM writer (called from Python after registration)
    pub fn attach_motor_shm_writer(&mut self, shm_path: std::path::PathBuf) -> Result<(), std::io::Error> {
        let writer = crate::motor_shm_writer::MotorSHMWriter::new(shm_path, None, None)?;
        let mut guard = self.motor_shm_writer.lock().unwrap();
        *guard = Some(writer);
        Ok(())
    }
    
    /// Set burst frequency (can be called while running)
    pub fn set_frequency(&mut self, frequency_hz: f64) {
        self.frequency_hz = frequency_hz;
        println!("[BURST-RUNNER] Frequency set to {:.2} Hz", frequency_hz);
    }
    
    /// Start the burst loop in a background thread
    /// 
    /// 🦀 Power neurons are read from RustNPU internally - 100% Rust!
    pub fn start(&mut self) -> Result<(), String> {
        if self.running.load(Ordering::Acquire) {
            return Err("Burst loop already running".to_string());
        }
        
        println!("[BURST-RUNNER] Starting burst loop at {:.2} Hz (power neurons auto-discovered from cortical_idx=1)", 
                 self.frequency_hz);
        
        self.running.store(true, Ordering::Release);
        self.burst_count.store(0, Ordering::Release);
        
        let npu = self.npu.clone();
        let frequency = self.frequency_hz;
        let running = self.running.clone();
        let burst_count = self.burst_count.clone();
        let viz_writer = self.viz_shm_writer.clone();
        
        self.thread_handle = Some(thread::Builder::new()
            .name("feagi-burst-loop".to_string())
            .spawn(move || {
                burst_loop(npu, frequency, running, burst_count, viz_writer);
            })
            .map_err(|e| format!("Failed to spawn burst loop thread: {}", e))?);
        
        println!("[BURST-RUNNER] ✅ Burst loop started successfully");
        Ok(())
    }
    
    /// Stop the burst loop gracefully
    pub fn stop(&mut self) {
        if !self.running.load(Ordering::Acquire) {
            return; // Already stopped
        }
        
        println!("[BURST-RUNNER] Stopping burst loop...");
        self.running.store(false, Ordering::Release);
        
        if let Some(handle) = self.thread_handle.take() {
            if handle.join().is_err() {
                eprintln!("[BURST-RUNNER] ⚠️ Burst loop thread panicked during shutdown");
            } else {
                println!("[BURST-RUNNER] ✅ Burst loop stopped cleanly");
            }
        }
    }
    
    /// Check if the burst loop is running
    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::Acquire)
    }
    
    /// Get current burst count
    pub fn get_burst_count(&self) -> u64 {
        self.burst_count.load(Ordering::Acquire)
    }
}

impl Drop for BurstLoopRunner {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Main burst processing loop (runs in dedicated thread)
/// 
/// This is the HOT PATH - zero Python involvement!
/// Power neurons are read directly from RustNPU's internal state.
fn burst_loop(
    npu: Arc<Mutex<RustNPU>>,
    frequency_hz: f64,
    running: Arc<AtomicBool>,
    burst_count: Arc<AtomicU64>,
    viz_shm_writer: Arc<Mutex<Option<crate::viz_shm_writer::VizSHMWriter>>>,
) {
    println!("[BURST-LOOP] 🚀 Starting main loop at {:.2} Hz", frequency_hz);
    
    let mut burst_num = 0u64;
    let mut last_stats_time = Instant::now();
    let mut total_neurons_fired = 0usize;
    let mut burst_times = Vec::with_capacity(100);
    let mut last_burst_time = None;
    
    while running.load(Ordering::Acquire) {
        let burst_start = Instant::now();
        
        // Track actual burst interval
        if let Some(last) = last_burst_time {
            let interval = burst_start.duration_since(last);
            burst_times.push(interval);
            if burst_times.len() > 100 {
                burst_times.remove(0);
            }
        }
        last_burst_time = Some(burst_start);
        
        // Process burst (THE HOT PATH!)
        // 🔋 Power neurons auto-discovered from neuron array - 100% Rust!
        let _fired_count = {
            let mut npu_lock = npu.lock().unwrap();
            match npu_lock.process_burst() {
                Ok(result) => {
                    total_neurons_fired += result.neuron_count;
                    result.neuron_count
                }
                Err(e) => {
                    eprintln!("[BURST-LOOP] ❌ Burst processing error: {}", e);
                    0
                }
            }
        }; // Drop lock immediately
        
        burst_num += 1;
        burst_count.store(burst_num, Ordering::Release);
        
        // Write visualization data to SHM (if attached)
        {
            static FIRST_CHECK_LOGGED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
            
            let mut viz_writer_lock = viz_shm_writer.lock().unwrap();
            if let Some(writer) = viz_writer_lock.as_mut() {
                // Get latest FQ sample from NPU (non-consuming read)
                let fire_data_opt = npu.lock().unwrap().get_latest_fire_queue_sample();
                
                if !FIRST_CHECK_LOGGED.load(std::sync::atomic::Ordering::Relaxed) {
                    println!("[BURST-LOOP] 🔍 Viz SHM writer is attached, checking FQ sample...");
                    if fire_data_opt.is_some() {
                        println!("[BURST-LOOP] 🔍 FQ sample available!");
                    } else {
                        println!("[BURST-LOOP] ⚠️ FQ sample is None (no neurons have fired yet)");
                    }
                    FIRST_CHECK_LOGGED.store(true, std::sync::atomic::Ordering::Relaxed);
                }
                
                if let Some(fire_data) = fire_data_opt {
                    // Debug: Log first successful viz write
                    static FIRST_VIZ_WRITE_LOGGED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
                    
                    // Encode as Type 11 ByteContainer using feagi_data_serialization
                    use feagi_data_structures::neurons::xyzp::{CorticalMappedXYZPNeuronData, NeuronXYZPArrays, NeuronXYZP};
                    use feagi_data_structures::genomic::CorticalID;
                    use feagi_data_serialization::FeagiByteContainer;
                    
                    // Convert FQ data to CorticalMappedXYZPNeuronData
                    let mut cortical_mapped = CorticalMappedXYZPNeuronData::new();
                    let mut total_neurons = 0;
                    
                    for (area_id, (x_vec, y_vec, z_vec, _id_vec, p_vec)) in fire_data.iter() {
                        total_neurons += x_vec.len();
                        
                        // Get cortical area name from NPU mapping (clone to avoid lifetime issues)
                        let cortical_name_opt = npu.lock().unwrap().get_cortical_area_name(*area_id).map(|s| s.to_string());
                        let cortical_id = match cortical_name_opt {
                            Some(name) => {
                                match CorticalID::new_custom_cortical_area_id(name.clone()) {
                                    Ok(id) => id,
                                    Err(e) => {
                                        eprintln!("[BURST-LOOP] ❌ Failed to create CorticalID for '{}': {:?}", name, e);
                                        continue;
                                    }
                                }
                            },
                            None => {
                                eprintln!("[BURST-LOOP] ❌ No cortical area name registered for area_id {}", area_id);
                                continue;
                            }
                        };
                        
                        let mut neuron_arrays = NeuronXYZPArrays::with_capacity(x_vec.len());
                        
                        for i in 0..x_vec.len() {
                            let neuron = NeuronXYZP::new(x_vec[i], y_vec[i], z_vec[i], p_vec[i]);
                            neuron_arrays.push(&neuron);
                        }
                        
                        cortical_mapped.insert(cortical_id, neuron_arrays);
                    }
                    
                    // Skip encoding if no valid cortical areas
                    if cortical_mapped.len() == 0 {
                        static SKIP_LOGGED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
                        if !SKIP_LOGGED.load(std::sync::atomic::Ordering::Relaxed) {
                            eprintln!("[BURST-LOOP] ⚠️ Skipping viz encoding - no cortical areas with registered names (neuroembryogenesis may not be complete yet)");
                            SKIP_LOGGED.store(true, std::sync::atomic::Ordering::Relaxed);
                        }
                        return;
                    }
                    
                    // Use official feagi_data_serialization API to encode
                    let mut byte_container = FeagiByteContainer::new_empty();
                    if let Err(e) = byte_container.overwrite_byte_data_with_single_struct_data(&cortical_mapped, 0) {
                        eprintln!("[BURST-LOOP] ❌ Failed to encode viz data: {:?}", e);
                        return;
                    }
                    
                    let encoded = byte_container.get_byte_ref();
                    
                    // Write to SHM
                    match writer.write_payload(encoded) {
                        Ok(_) => {
                            if !FIRST_VIZ_WRITE_LOGGED.load(std::sync::atomic::Ordering::Relaxed) {
                                println!("[BURST-LOOP] 🎨 ✅ First viz SHM write (Type 11): {} bytes ({} neurons across {} areas)", 
                                    encoded.len(), total_neurons, fire_data.len());
                                FIRST_VIZ_WRITE_LOGGED.store(true, std::sync::atomic::Ordering::Relaxed);
                            }
                        }
                        Err(e) => {
                            eprintln!("[BURST-LOOP] ❌ Failed to write viz SHM: {}", e);
                        }
                    }
                }
            }
        }
        
        // Performance logging every 5 seconds
        let now = Instant::now();
        if now.duration_since(last_stats_time).as_secs() >= 5 {
            if !burst_times.is_empty() {
                let avg_interval: Duration = burst_times.iter().sum::<Duration>() / burst_times.len() as u32;
                let actual_hz = 1.0 / avg_interval.as_secs_f64();
                let avg_neurons = total_neurons_fired / burst_times.len();
                
                println!(
                    "[BURST-LOOP] 📊 Stats: Burst #{} | Desired: {:.2} Hz | Actual: {:.2} Hz ({:.1}%) | Avg neurons: {}",
                    burst_num,
                    frequency_hz,
                    actual_hz,
                    (actual_hz / frequency_hz * 100.0),
                    avg_neurons
                );
            }
            
            last_stats_time = now;
            total_neurons_fired = 0;
        }
        
        // Adaptive sleep (RTOS-friendly timing)
        // Strategy: <5Hz = simple sleep, 5-100Hz = hybrid, >100Hz = busy-wait
        let interval_sec = 1.0 / frequency_hz;
        let target_time = burst_start + Duration::from_secs_f64(interval_sec);
        let now = Instant::now();
        
        if now < target_time {
            let remaining = target_time - now;
            
            if frequency_hz < 5.0 {
                // Low frequency: simple sleep
                thread::sleep(remaining);
            } else if frequency_hz > 100.0 {
                // High frequency: pure busy-wait
                while Instant::now() < target_time && running.load(Ordering::Relaxed) {}
            } else {
                // Medium frequency: hybrid (sleep 80%, busy-wait 20%)
                let sleep_duration = remaining.mul_f64(0.8);
                if sleep_duration.as_micros() > 100 {
                    thread::sleep(sleep_duration);
                }
                while Instant::now() < target_time && running.load(Ordering::Relaxed) {}
            }
        }
    }
    
    println!("[BURST-LOOP] 🛑 Main loop stopped after {} bursts", burst_num);
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    
    #[test]
    fn test_burst_loop_lifecycle() {
        let npu = Arc::new(Mutex::new(RustNPU::new(1000, 10000, 20)));
        let mut runner = BurstLoopRunner::new(npu, 10.0);
        
        assert!(!runner.is_running());
        
        runner.set_power_neurons(vec![1, 2, 3]);
        runner.start().unwrap();
        
        assert!(runner.is_running());
        
        // Let it run for 100ms
        thread::sleep(Duration::from_millis(100));
        
        // Should have processed ~1 burst at 10Hz
        assert!(runner.get_burst_count() >= 1);
        
        runner.stop();
        assert!(!runner.is_running());
    }
}

