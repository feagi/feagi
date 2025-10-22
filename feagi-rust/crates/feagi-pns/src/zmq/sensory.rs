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
                    // Log waiting every 5 seconds
                    static POLL_COUNT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
                    let count = POLL_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    if count % 5 == 0 {
                        println!("🦀 [ZMQ-SENSORY] ⏳ Waiting for sensory data on port 5558... (poll #{})", count);
                    }
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
    /// This function uses the feagi_data_serialization crate to deserialize
    /// binary XYZP data and directly injects it into the Rust NPU.
    /// 
    /// NO Python FFI overhead! Pure Rust path: Agent → ZMQ → Rust deserialize → Rust NPU
    /// 
    /// TODO: This is currently a stub. Full implementation requires:
    /// 1. Agent registration to work (provides cortical area name → ID mapping)
    /// 2. Coordination with burst engine's sensory manager
    fn deserialize_and_inject_xyzp(
        message_bytes: &[u8],
        npu_mutex: &Arc<Mutex<Option<Arc<std::sync::Mutex<feagi_burst_engine::RustNPU>>>>>,
        first_logged: &bool,
    ) -> Result<usize, String> {
        use feagi_data_structures::neuron_voxels::xyzp::CorticalMappedXYZPNeuronVoxels;
        
        // Get NPU reference
        let npu_lock = npu_mutex.lock();
        let npu_arc = match npu_lock.as_ref() {
            Some(n) => Arc::clone(n),
            None => return Err("NPU not connected".to_string()),
        };
        drop(npu_lock); // Release early
        
        // Deserialize binary XYZP data using FeagiByteContainer (version 2 container format)
        let mut byte_container = feagi_data_serialization::FeagiByteContainer::new_empty();
        let mut data_vec = message_bytes.to_vec();
        
        // Load bytes into container
        byte_container.try_write_data_to_container_and_verify(&mut |bytes| {
            std::mem::swap(bytes, &mut data_vec);
            Ok(())
        }).map_err(|e| format!("Failed to load bytes: {:?}", e))?;
        
        // Get number of structures
        let num_structures = byte_container.try_get_number_contained_structures()
            .map_err(|e| format!("Failed to get structure count: {:?}", e))?;
        
        if num_structures == 0 {
            return Err("No structures in message".to_string());
        }
        
        // Extract first structure
        let boxed_struct = byte_container.try_create_new_struct_from_index(0)
            .map_err(|e| format!("Failed to extract structure: {:?}", e))?;
        
        // Downcast to CorticalMappedXYZPNeuronVoxels
        let cortical_mapped = boxed_struct.as_any().downcast_ref::<CorticalMappedXYZPNeuronVoxels>()
            .ok_or_else(|| "Structure is not CorticalMappedXYZPNeuronVoxels".to_string())?;
        
        // Log first data
        if !first_logged {
            println!(
                "🦀 [ZMQ-SENSORY] 📥 First sensory data: {} bytes, {} cortical areas",
                message_bytes.len(), cortical_mapped.len()
            );
        }
        
        // Inject XYZP data into NPU using cortical area mapping
        let mut total_injected = 0;
        let mut npu = npu_arc.lock().unwrap();
        
        for (cortical_id, neuron_arrays) in &cortical_mapped.mappings {
            // Convert CorticalID to string name
            let cortical_name = cortical_id.to_string();
            
            // Use borrow_xyzp_vectors to access the coordinate vectors
            let (x_coords, y_coords, z_coords, potentials) = neuron_arrays.borrow_xyzp_vectors();
            
            // Build XYZP tuples for this cortical area
            let num_neurons = neuron_arrays.len();
            let mut xyzp_data = Vec::with_capacity(num_neurons);
            
            for i in 0..num_neurons {
                xyzp_data.push((
                    x_coords[i],
                    y_coords[i],
                    z_coords[i],
                    potentials[i],
                ));
            }
            
            // 🔍 DEBUG: Log first few coordinates to verify data
            if !first_logged && num_neurons > 0 {
                println!("🦀 [ZMQ-SENSORY] 📍 Cortical area '{}': {} neurons", cortical_name, num_neurons);
                println!("🦀 [ZMQ-SENSORY]    First neuron: x={}, y={}, z={}, p={}", 
                         x_coords[0], y_coords[0], z_coords[0], potentials[0]);
                if num_neurons > 1 {
                    println!("🦀 [ZMQ-SENSORY]    Last neuron: x={}, y={}, z={}, p={}", 
                             x_coords[num_neurons-1], y_coords[num_neurons-1], 
                             z_coords[num_neurons-1], potentials[num_neurons-1]);
                }
            }
            
            // Inject into NPU (will map cortical name → area_id → neuron_ids)
            let injected = npu.inject_sensory_xyzp(&cortical_name, &xyzp_data);
            total_injected += injected;
            
            if !first_logged {
                if injected == 0 {
                    println!("🦀 [ZMQ-SENSORY] ❌ ZERO neurons found for cortical area '{}'", cortical_name);
                    println!("🦀 [ZMQ-SENSORY] ❌ This means:");
                    println!("🦀 [ZMQ-SENSORY] ❌   1. Cortical area '{}' not in genome, OR", cortical_name);
                    println!("🦀 [ZMQ-SENSORY] ❌   2. No neurons exist at those X,Y,Z coordinates");
                } else if injected < num_neurons {
                    println!("🦀 [ZMQ-SENSORY] ⚠️  Only {}/{} neurons found for cortical area '{}'", 
                             injected, num_neurons, cortical_name);
                } else {
                    println!("🦀 [ZMQ-SENSORY] ✅ Injected {}/{} neurons for '{}'", 
                             injected, num_neurons, cortical_name);
                }
            }
        }
        
        if !first_logged {
            if total_injected == 0 {
                println!("🦀 [ZMQ-SENSORY] ❌ FATAL: NO NEURONS INJECTED INTO NPU!");
                println!("🦀 [ZMQ-SENSORY] ❌ Check genome: does it have the cortical areas being sent?");
            } else {
                println!("🦀 [ZMQ-SENSORY] ✅ Injected {} neurons into NPU → FCL", total_injected);
            }
        }
        
        Ok(total_injected)
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

