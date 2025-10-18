/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # Rate Limiting for Sensory Polling
//!
//! Ensures sensory data is polled at the agent-requested frequency,
//! matching Python's CapabilityRateManager behavior.

use std::time::{Duration, Instant};

/// Rate limiter for per-agent sensory polling
pub struct RateLimiter {
    /// Target polling interval in nanoseconds
    interval_ns: u64,
    /// Last poll time (monotonic clock)
    last_poll: Option<Instant>,
}

impl RateLimiter {
    /// Create a new rate limiter with target frequency in Hz
    pub fn new(rate_hz: f64) -> Self {
        let interval_ns = (1_000_000_000.0 / rate_hz) as u64;
        Self {
            interval_ns,
            last_poll: None,
        }
    }
    
    /// Check if enough time has elapsed to poll again
    /// Returns true if we should poll now, false if we should wait
    pub fn should_poll_now(&mut self) -> bool {
        let now = Instant::now();
        
        match self.last_poll {
            None => {
                // First poll
                self.last_poll = Some(now);
                true
            }
            Some(last) => {
                let elapsed = now.duration_since(last);
                let elapsed_ns = elapsed.as_nanos() as u64;
                
                if elapsed_ns >= self.interval_ns {
                    self.last_poll = Some(now);
                    true
                } else {
                    false
                }
            }
        }
    }
    
    /// Get the time until next poll
    /// Returns None if we should poll immediately
    pub fn time_until_next_poll(&self) -> Option<Duration> {
        match self.last_poll {
            None => None,  // Poll immediately
            Some(last) => {
                let now = Instant::now();
                let elapsed = now.duration_since(last);
                let elapsed_ns = elapsed.as_nanos() as u64;
                
                if elapsed_ns >= self.interval_ns {
                    None  // Ready to poll
                } else {
                    let remaining_ns = self.interval_ns - elapsed_ns;
                    Some(Duration::from_nanos(remaining_ns))
                }
            }
        }
    }
    
    /// Update the rate (useful for dynamic rate changes)
    pub fn set_rate(&mut self, rate_hz: f64) {
        self.interval_ns = (1_000_000_000.0 / rate_hz) as u64;
    }
    
    /// Get the current rate in Hz
    pub fn rate_hz(&self) -> f64 {
        1_000_000_000.0 / self.interval_ns as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;
    
    #[test]
    fn test_rate_limiter_first_poll() {
        let mut limiter = RateLimiter::new(10.0);  // 10 Hz = 100ms interval
        assert!(limiter.should_poll_now());  // First poll should always be true
    }
    
    #[test]
    fn test_rate_limiter_blocks_too_soon() {
        let mut limiter = RateLimiter::new(10.0);  // 10 Hz = 100ms interval
        assert!(limiter.should_poll_now());  // First poll
        assert!(!limiter.should_poll_now());  // Immediate second poll should be blocked
    }
    
    #[test]
    fn test_rate_limiter_allows_after_interval() {
        let mut limiter = RateLimiter::new(100.0);  // 100 Hz = 10ms interval
        assert!(limiter.should_poll_now());  // First poll
        thread::sleep(Duration::from_millis(11));  // Wait slightly longer than interval
        assert!(limiter.should_poll_now());  // Should allow now
    }
    
    #[test]
    fn test_rate_limiter_time_until_next() {
        let mut limiter = RateLimiter::new(10.0);  // 10 Hz = 100ms interval
        
        // Before first poll
        assert!(limiter.time_until_next_poll().is_none());
        
        // After first poll
        limiter.should_poll_now();
        let time_remaining = limiter.time_until_next_poll();
        assert!(time_remaining.is_some());
        assert!(time_remaining.unwrap().as_millis() > 90);  // Should be close to 100ms
    }
}

