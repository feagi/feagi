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
}

impl BurstLoopRunner {
    /// Create a new burst loop runner
    pub fn new(npu: Arc<Mutex<RustNPU>>, frequency_hz: f64) -> Self {
        // Create FCL injection callback for sensory data
        let npu_for_callback = npu.clone();
        let injection_callback = Arc::new(move |_cortical_idx: u32, neuron_ids: Vec<u32>| {
            // Inject neurons into FCL (thread-safe)
            // Convert u32 to NeuronId and inject with full potential (1.0)
            let neurons: Vec<NeuronId> = neuron_ids.into_iter().map(NeuronId).collect();
            if let Ok(mut npu_lock) = npu_for_callback.lock() {
                npu_lock.inject_sensory_batch(&neurons, 1.0);
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
        }
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
        
        self.thread_handle = Some(thread::Builder::new()
            .name("feagi-burst-loop".to_string())
            .spawn(move || {
                burst_loop(npu, frequency, running, burst_count);
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

