// Heartbeat Tracker - monitors agent health and deregisters stale agents

use crate::agent_registry::AgentRegistry;
use parking_lot::RwLock;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

/// Heartbeat Tracker
pub struct HeartbeatTracker {
    running: Arc<RwLock<bool>>,
    thread_handle: Option<thread::JoinHandle<()>>,
    timeout: Duration,
}

impl HeartbeatTracker {
    pub fn new() -> Self {
        Self {
            running: Arc::new(RwLock::new(false)),
            thread_handle: None,
            timeout: Duration::from_secs(45), // Default 45s timeout
        }
    }

    /// Start heartbeat monitoring
    pub fn start(&mut self, agent_registry: Arc<RwLock<AgentRegistry>>) {
        if *self.running.read() {
            return;
        }

        *self.running.write() = true;
        let running = Arc::clone(&self.running);
        let timeout = self.timeout;

        let handle = thread::spawn(move || {
            println!("🦀 [HEARTBEAT] Monitoring started (timeout: {:?})", timeout);

            while *running.read() {
                thread::sleep(Duration::from_secs(10)); // Check every 10s

                if !*running.read() {
                    break;
                }

                // Check for stale agents
                let stale_agents = agent_registry.read().get_stale_agents(timeout);

                if !stale_agents.is_empty() {
                    println!(
                        "🦀 [HEARTBEAT] Found {} stale agent(s)",
                        stale_agents.len()
                    );

                    // Deregister stale agents
                    for agent_id in stale_agents {
                        println!("🦀 [HEARTBEAT] Deregistering stale agent: {}", agent_id);
                        if let Err(e) = agent_registry.write().deregister(&agent_id) {
                            eprintln!("🦀 [HEARTBEAT] [ERR] Failed to deregister {}: {}", agent_id, e);
                        }
                    }
                }
            }

            println!("🦀 [HEARTBEAT] Monitoring stopped");
        });

        self.thread_handle = Some(handle);
    }

    /// Stop heartbeat monitoring
    pub fn stop(&mut self) {
        *self.running.write() = false;

        if let Some(handle) = self.thread_handle.take() {
            let _ = handle.join();
        }
    }

    /// Set heartbeat timeout
    pub fn set_timeout(&mut self, timeout: Duration) {
        self.timeout = timeout;
    }
}

impl Drop for HeartbeatTracker {
    fn drop(&mut self) {
        self.stop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::agent_registry::{AgentCapabilities, AgentInfo, AgentTransport};
    use std::time::Instant;

    #[test]
    fn test_heartbeat_tracker() {
        let registry = Arc::new(RwLock::new(AgentRegistry::new()));
        let mut tracker = HeartbeatTracker::new();
        tracker.set_timeout(Duration::from_millis(100));

        // Register a test agent
        let agent_info = AgentInfo {
            agent_id: "test-agent".to_string(),
            agent_type: "external".to_string(),
            capabilities: AgentCapabilities {
                sensory: None,
                motor: None,
                visualization: None,
            },
            registered_at: Instant::now(),
            last_heartbeat: Instant::now() - Duration::from_secs(60), // Stale
            transport: AgentTransport::Zmq,
        };

        registry.write().register(agent_info).unwrap();
        assert_eq!(registry.read().count(), 1);

        // Start tracker (should deregister stale agent)
        tracker.start(Arc::clone(&registry));
        thread::sleep(Duration::from_millis(500)); // Wait for check
        tracker.stop();

        // Agent should be deregistered
        assert_eq!(registry.read().count(), 0);
    }
}

