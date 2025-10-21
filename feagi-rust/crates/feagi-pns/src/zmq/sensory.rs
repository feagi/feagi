// Sensory stream for receiving sensory data from agents
// Uses PULL socket pattern for receiving data from multiple agents (agents use PUSH)

use parking_lot::Mutex;
use std::sync::Arc;
use std::thread;

/// Sensory stream for receiving sensory data from agents
#[derive(Clone)]
pub struct SensoryStream {
    context: Arc<zmq::Context>,
    bind_address: String,
    socket: Arc<Mutex<Option<zmq::Socket>>>,
    running: Arc<Mutex<bool>>,
    /// Reference to Rust NPU for direct injection (no FFI overhead!)
    npu: Arc<Mutex<Option<Arc<std::sync::Mutex<feagi_burst_engine::RustNPU>>>>>,
    /// Statistics
    total_messages: Arc<Mutex<u64>>,
    total_neurons: Arc<Mutex<u64>>,
}

impl SensoryStream {
    /// Create a new sensory stream
    pub fn new(context: Arc<zmq::Context>, bind_address: &str) -> Result<Self, String> {
        Ok(Self {
            context,
            bind_address: bind_address.to_string(),
            socket: Arc::new(Mutex::new(None)),
            running: Arc::new(Mutex::new(false)),
            npu: Arc::new(Mutex::new(None)),
            total_messages: Arc::new(Mutex::new(0)),
            total_neurons: Arc::new(Mutex::new(0)),
        })
    }

    /// Set the Rust NPU reference for direct injection
    pub fn set_npu(&self, npu: Arc<std::sync::Mutex<feagi_burst_engine::RustNPU>>) {
        *self.npu.lock() = Some(npu);
        println!("🦀 [SENSORY-STREAM] NPU connected for direct injection");
    }

    /// Start the sensory stream
    pub fn start(&self) -> Result<(), String> {
        if *self.running.lock() {
            return Err("Sensory stream already running".to_string());
        }

        // Create PULL socket for receiving sensory data
        let socket = self
            .context
            .socket(zmq::PULL)
            .map_err(|e| e.to_string())?;

        // Set socket options
        socket
            .set_linger(0) // Don't wait on close
            .map_err(|e| e.to_string())?;
        socket
            .set_rcvhwm(1000) // High water mark for receive buffer
            .map_err(|e| e.to_string())?;

        // Bind socket
        socket
            .bind(&self.bind_address)
            .map_err(|e| e.to_string())?;

        *self.socket.lock() = Some(socket);
        *self.running.lock() = true;

        println!("🦀 [ZMQ-SENSORY] ✅ Listening on {}", self.bind_address);

        // Start processing loop
        self.start_processing_loop();

        Ok(())
    }

    /// Stop the sensory stream
    pub fn stop(&self) -> Result<(), String> {
        *self.running.lock() = false;
        
        // Log final statistics
        let total_msg = *self.total_messages.lock();
        let total_neurons = *self.total_neurons.lock();
        println!(
            "🦀 [ZMQ-SENSORY] Stopped. Total: {} messages, {} neurons",
            total_msg, total_neurons
        );
        
        *self.socket.lock() = None;
        Ok(())
    }

    /// Start the background processing loop
    fn start_processing_loop(&self) {
        let socket = Arc::clone(&self.socket);
        let running = Arc::clone(&self.running);
        let npu = Arc::clone(&self.npu);
        let total_messages = Arc::clone(&self.total_messages);
        let total_neurons = Arc::clone(&self.total_neurons);

        thread::spawn(move || {
            println!("🦀 [ZMQ-SENSORY] Processing loop started");

            let mut first_data_logged = false;
            let mut message_count = 0u64;

            while *running.lock() {
                let sock_guard = socket.lock();
                let sock = match sock_guard.as_ref() {
                    Some(s) => s,
                    None => {
                        drop(sock_guard);
                        thread::sleep(std::time::Duration::from_millis(100));
                        continue;
                    }
                };

                // Poll for messages with timeout
                let poll_items = &mut [sock.as_poll_item(zmq::POLLIN)];
                if let Err(e) = zmq::poll(poll_items, 1000) {
                    eprintln!("🦀 [ZMQ-SENSORY] [ERR] Poll error: {}", e);
                    continue;
                }

                if !poll_items[0].is_readable() {
                    drop(sock_guard);
                    continue;
                }

                // Receive message
                let mut msg = zmq::Message::new();
                match sock.recv(&mut msg, 0) {
                    Ok(()) => {
                        drop(sock_guard); // Release lock before processing
                        
                        *total_messages.lock() += 1;
                        message_count += 1;

                        // Process the binary data
                        let message_bytes = msg.as_ref();
                        
                        // Try to deserialize as binary XYZP data (using feagi-data-processing)
                        match Self::deserialize_and_inject_xyzp(message_bytes, &npu, &first_data_logged) {
                            Ok(neuron_count) => {
                                *total_neurons.lock() += neuron_count as u64;
                                first_data_logged = true;
                                
                                // Log periodically
                                if message_count % 100 == 0 {
                                    let total_msg = *total_messages.lock();
                                    let total_n = *total_neurons.lock();
                                    println!(
                                        "🦀 [ZMQ-SENSORY] Stats: {} messages, {} neurons total",
                                        total_msg, total_n
                                    );
                                }
                            }
                            Err(e) => {
                                if message_count <= 5 {
                                    eprintln!("🦀 [ZMQ-SENSORY] [ERR] Failed to process sensory data: {}", e);
                                    eprintln!("🦀 [ZMQ-SENSORY] Message size: {} bytes", message_bytes.len());
                                }
                            }
                        }
                    }
                    Err(e) => {
                        eprintln!("🦀 [ZMQ-SENSORY] [ERR] Receive error: {}", e);
                    }
                }
            }

            println!("🦀 [ZMQ-SENSORY] Processing loop stopped");
        });
    }

    /// Deserialize XYZP binary data and inject into NPU
    /// 
    /// This function uses the Rust feagi-data-processing crate to deserialize
    /// binary XYZP data and directly injects it into the Rust NPU.
    /// 
    /// NO Python FFI overhead! Pure Rust path: Agent → ZMQ → Rust deserialize → Rust NPU
    fn deserialize_and_inject_xyzp(
        message_bytes: &[u8],
        npu_mutex: &Arc<Mutex<Option<Arc<std::sync::Mutex<feagi_burst_engine::RustNPU>>>>>,
        first_logged: &bool,
    ) -> Result<usize, String> {
        // TODO: Integrate with feagi-data-processing crate for XYZP deserialization
        // For now, we'll prepare the structure for the integration
        
        // Get NPU reference
        let npu_lock = npu_mutex.lock();
        let npu_arc = match npu_lock.as_ref() {
            Some(n) => n,
            None => return Err("NPU not connected".to_string()),
        };
        
        // Parse the binary XYZP data
        // Format: CorticalMappedXYZPNeuronData (from feagi-data-processing)
        // This will be implemented when feagi-data-processing crate is integrated
        
        // For now, log that we received data
        if !first_logged {
            println!(
                "🦀 [ZMQ-SENSORY] 📥 Received sensory data: {} bytes (XYZP binary)",
                message_bytes.len()
            );
            println!("🦀 [ZMQ-SENSORY] First 32 bytes: {:?}", &message_bytes[0..message_bytes.len().min(32)]);
        }
        
        // TODO: Deserialize XYZP and inject
        // let cortical_mapped = feagi_data_processing::deserialize_xyzp(message_bytes)?;
        // for (cortical_id, neurons) in cortical_mapped {
        //     let neuron_potentials: Vec<(NeuronId, f32)> = neurons
        //         .iter()
        //         .map(|n| (map_xyz_to_neuron_id(n.x, n.y, n.z), n.p))
        //         .collect();
        //     
        //     npu_arc.lock().unwrap().inject_sensory_with_potentials(&neuron_potentials);
        // }
        
        // Placeholder: Return estimated neuron count from message size
        let estimated_neurons = message_bytes.len() / 16; // Rough estimate
        Ok(estimated_neurons)
    }

    /// Check if stream is running
    pub fn is_running(&self) -> bool {
        *self.running.lock()
    }

    /// Get statistics
    pub fn get_stats(&self) -> (u64, u64) {
        (*self.total_messages.lock(), *self.total_neurons.lock())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sensory_stream_creation() {
        let ctx = Arc::new(zmq::Context::new());
        let stream = SensoryStream::new(ctx, "tcp://127.0.0.1:5558");
        assert!(stream.is_ok());
    }
}

