//! Reconnection logic with exponential backoff

use crate::error::Result;
use log::{info, warn};
use std::time::Duration;

/// Reconnection strategy with exponential backoff
pub struct ReconnectionStrategy {
    /// Base backoff in milliseconds
    base_backoff_ms: u64,
    
    /// Maximum backoff in milliseconds
    max_backoff_ms: u64,
    
    /// Current attempt number
    current_attempt: u32,
    
    /// Maximum retry attempts (0 = infinite)
    max_attempts: u32,
}

impl ReconnectionStrategy {
    /// Create a new reconnection strategy
    ///
    /// # Arguments
    /// * `base_backoff_ms` - Initial backoff duration in milliseconds
    /// * `max_attempts` - Maximum retry attempts (0 = infinite)
    pub fn new(base_backoff_ms: u64, max_attempts: u32) -> Self {
        Self {
            base_backoff_ms,
            max_backoff_ms: 60_000, // Cap at 60 seconds
            current_attempt: 0,
            max_attempts,
        }
    }
    
    /// Get next backoff duration with exponential increase
    pub fn next_backoff(&mut self) -> Option<Duration> {
        // Check if max attempts reached
        if self.max_attempts > 0 && self.current_attempt >= self.max_attempts {
            return None;
        }
        
        self.current_attempt += 1;
        
        // Calculate exponential backoff: base * 2^(attempt - 1)
        let backoff_ms = if self.current_attempt == 1 {
            self.base_backoff_ms
        } else {
            let exp = 2u64.saturating_pow(self.current_attempt - 1);
            (self.base_backoff_ms * exp).min(self.max_backoff_ms)
        };
        
        Some(Duration::from_millis(backoff_ms))
    }
    
    /// Reset the strategy (after successful connection)
    pub fn reset(&mut self) {
        self.current_attempt = 0;
    }
    
    /// Get current attempt number
    pub fn attempt_number(&self) -> u32 {
        self.current_attempt
    }
    
    /// Check if attempts exhausted
    pub fn is_exhausted(&self) -> bool {
        self.max_attempts > 0 && self.current_attempt >= self.max_attempts
    }
}

/// Execute a retryable operation with exponential backoff
///
/// # Arguments
/// * `operation` - Closure to execute (returns Result)
/// * `strategy` - Reconnection strategy
/// * `operation_name` - Name of operation for logging
///
/// # Example
/// ```ignore
/// let mut strategy = ReconnectionStrategy::new(1000, 3);
/// retry_with_backoff(
///     || connect_to_feagi(),
///     &mut strategy,
///     "FEAGI connection"
/// )?;
/// ```
pub fn retry_with_backoff<F, T>(
    mut operation: F,
    strategy: &mut ReconnectionStrategy,
    operation_name: &str,
) -> Result<T>
where
    F: FnMut() -> Result<T>,
{
    loop {
        match operation() {
            Ok(result) => {
                if strategy.attempt_number() > 0 {
                    info!(
                        "[RECONNECT] ✓ {} succeeded after {} attempts",
                        operation_name,
                        strategy.attempt_number()
                    );
                }
                strategy.reset();
                return Ok(result);
            }
            Err(e) if e.is_retryable() => {
                if let Some(backoff) = strategy.next_backoff() {
                    warn!(
                        "[RECONNECT] ⚠ {} failed (attempt {}): {} - retrying in {:?}",
                        operation_name,
                        strategy.attempt_number(),
                        e,
                        backoff
                    );
                    std::thread::sleep(backoff);
                } else {
                    warn!(
                        "[RECONNECT] ✗ {} failed after {} attempts - giving up",
                        operation_name,
                        strategy.attempt_number()
                    );
                    return Err(e);
                }
            }
            Err(e) => {
                // Non-retryable error
                return Err(e);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_exponential_backoff() {
        let mut strategy = ReconnectionStrategy::new(100, 5);
        
        assert_eq!(strategy.next_backoff(), Some(Duration::from_millis(100))); // 100 * 2^0
        assert_eq!(strategy.next_backoff(), Some(Duration::from_millis(200))); // 100 * 2^1
        assert_eq!(strategy.next_backoff(), Some(Duration::from_millis(400))); // 100 * 2^2
        assert_eq!(strategy.next_backoff(), Some(Duration::from_millis(800))); // 100 * 2^3
        assert_eq!(strategy.next_backoff(), Some(Duration::from_millis(1600))); // 100 * 2^4
        assert_eq!(strategy.next_backoff(), None); // Max attempts reached
    }
    
    #[test]
    fn test_backoff_capped() {
        let mut strategy = ReconnectionStrategy::new(1000, 20);
        
        // Keep calling next_backoff until we hit the cap
        for _ in 0..10 {
            strategy.next_backoff();
        }
        
        // Next backoff should be capped at max_backoff_ms (60000)
        let backoff = strategy.next_backoff().unwrap();
        assert_eq!(backoff, Duration::from_millis(60_000));
    }
    
    #[test]
    fn test_reset() {
        let mut strategy = ReconnectionStrategy::new(100, 5);
        
        strategy.next_backoff();
        strategy.next_backoff();
        assert_eq!(strategy.attempt_number(), 2);
        
        strategy.reset();
        assert_eq!(strategy.attempt_number(), 0);
    }
    
    #[test]
    fn test_is_exhausted() {
        let mut strategy = ReconnectionStrategy::new(100, 2);
        
        assert!(!strategy.is_exhausted());
        strategy.next_backoff();
        assert!(!strategy.is_exhausted());
        strategy.next_backoff();
        assert!(strategy.is_exhausted());
    }
    
    #[test]
    fn test_infinite_retries() {
        let mut strategy = ReconnectionStrategy::new(100, 0);
        
        // Should never be exhausted with max_attempts = 0
        // Test a reasonable number of retries (not too many to avoid overflow)
        for _ in 0..20 {
            assert!(strategy.next_backoff().is_some());
            assert!(!strategy.is_exhausted());
        }
    }
}

