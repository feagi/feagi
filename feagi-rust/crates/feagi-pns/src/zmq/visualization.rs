// Visualization stream for sending neuron activity to Brain Visualizer (ZMQ fallback for remote clients)
// Uses PUB socket pattern for one-to-many distribution

use parking_lot::Mutex;
use std::sync::Arc;

/// Visualization stream for publishing neuron activity
#[derive(Clone)]
pub struct VisualizationStream {
    context: Arc<zmq::Context>,
    bind_address: String,
    socket: Arc<Mutex<Option<zmq::Socket>>>,
    running: Arc<Mutex<bool>>,
}

impl VisualizationStream {
    /// Create a new visualization stream
    pub fn new(context: Arc<zmq::Context>, bind_address: &str) -> Result<Self, String> {
        Ok(Self {
            context,
            bind_address: bind_address.to_string(),
            socket: Arc::new(Mutex::new(None)),
            running: Arc::new(Mutex::new(false)),
        })
    }

    /// Start the visualization stream
    pub fn start(&self) -> Result<(), String> {
        if *self.running.lock() {
            return Err("Visualization stream already running".to_string());
        }

        // Create PUB socket for broadcasting visualization data
        let socket = self
            .context
            .socket(zmq::PUB)
            .map_err(|e| e.to_string())?;

        // Set socket options for optimal performance
        socket
            .set_linger(0) // Don't wait on close
            .map_err(|e| e.to_string())?;
        socket
            .set_sndhwm(1000) // High water mark for send buffer
            .map_err(|e| e.to_string())?;
        socket
            .set_conflate(true) // Keep only latest message (real-time data)
            .map_err(|e| e.to_string())?;

        // Bind socket
        socket
            .bind(&self.bind_address)
            .map_err(|e| e.to_string())?;

        *self.socket.lock() = Some(socket);
        *self.running.lock() = true;

        println!("🦀 [ZMQ-VIZ] Listening on {}", self.bind_address);

        Ok(())
    }

    /// Stop the visualization stream
    pub fn stop(&self) -> Result<(), String> {
        *self.running.lock() = false;
        *self.socket.lock() = None;
        Ok(())
    }

    /// Publish visualization data to all subscribers
    pub fn publish(&self, data: &[u8]) -> Result<(), String> {
        let sock_guard = self.socket.lock();
        let sock = match sock_guard.as_ref() {
            Some(s) => s,
            None => {
                return Err("Visualization stream not started".to_string())
            }
        };

        // 🔍 DEBUG: Log first 10 ZMQ sends
        static SEND_COUNT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let count = SEND_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        if count < 10 {
            println!("🦀 [ZMQ-VIZ] SEND #{}: {} bytes to {}", count + 1, data.len(), self.bind_address);
            println!("🦀 [ZMQ-VIZ] First 32 bytes: {:?}", &data[0..data.len().min(32)]);
        }

        sock.send(data, 0)
            .map_err(|e| e.to_string())?;

        Ok(())
    }

    /// Check if stream is running
    pub fn is_running(&self) -> bool {
        *self.running.lock()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_viz_stream_creation() {
        let ctx = Arc::new(zmq::Context::new());
        let stream = VisualizationStream::new(ctx, "tcp://127.0.0.1:30010");
        assert!(stream.is_ok());
    }
}

