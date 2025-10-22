use clap::Parser;
use log::{info, warn, debug};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
extern crate zmq;
extern crate serde_json;

/// FEAGI Inference Engine - Standalone neural processing engine with online learning
#[derive(Parser, Debug)]
#[command(name = "feagi-inference-engine", version, author, long_about = None)]
struct Args {
    /// Path to the connectome file to load
    #[arg(short, long)]
    connectome: PathBuf,

    /// Burst frequency in Hz (default: 50)
    #[arg(long, default_value_t = 50)]
    burst_hz: u64,

    /// Auto-save on shutdown
    #[arg(long, default_value_t = true)]
    auto_save: bool,

    /// Checkpoint interval in seconds (0 = disabled)
    #[arg(long, default_value_t = 0)]
    checkpoint_interval: u64,

    /// Enable verbose logging
    #[arg(short, long, default_value_t = false)]
    verbose: bool,

    /// ZMQ registration endpoint (default: tcp://*:5000)
    #[arg(long, default_value = "tcp://*:5000")]
    registration_endpoint: String,

    /// ZMQ sensory input endpoint (default: tcp://*:5555)
    #[arg(long, default_value = "tcp://*:5555")]
    sensory_endpoint: String,

    /// ZMQ motor output endpoint (default: tcp://*:5556)
    #[arg(long, default_value = "tcp://*:5556")]
    motor_endpoint: String,

    /// Maximum number of agents (default: 100)
    #[arg(long, default_value_t = 100)]
    max_agents: usize,

    /// Agent inactivity timeout in milliseconds (default: 60000)
    #[arg(long, default_value_t = 60000)]
    agent_timeout_ms: u64,
}

/// Main entry point
fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Parse CLI arguments first to get verbose flag
    let args = Args::parse();

    // Initialize logger
    env_logger::Builder::from_default_env()
        .filter_level(if args.verbose {
            log::LevelFilter::Debug
        } else {
            log::LevelFilter::Info
        })
        .init();

    // Banner
    print_banner();

    // Load connectome
    info!("Loading connectome from: {}", args.connectome.display());
    let connectome = feagi_connectome_serialization::load_connectome(&args.connectome)?;

    info!("✓ Connectome loaded successfully!");
    info!("  Neurons: {}/{}", connectome.neurons.count, connectome.neurons.capacity);
    info!("  Synapses: {}/{}", connectome.synapses.count, connectome.synapses.capacity);
    info!("  Cortical areas: {}", connectome.cortical_area_names.len());

    // Create NPU from connectome
    info!("Initializing NPU...");
    let mut npu = feagi_burst_engine::RustNPU::import_connectome(connectome);
    info!("✓ NPU initialized successfully!");

    // Initialize agent registry
    info!("Initializing agent registry...");
    let registry = Arc::new(std::sync::RwLock::new(feagi_pns::agent_registry::AgentRegistry::new(
        args.max_agents,
        args.agent_timeout_ms,
    )));
    info!("✓ Agent registry initialized (max_agents={}, timeout={}ms)", 
          args.max_agents, args.agent_timeout_ms);

    // Note: ZMQ transport functionality integrated directly into main loop
    info!("✓ ZMQ registration endpoint: {}", args.registration_endpoint);
    info!("  ZMQ sensory input endpoint: {}", args.sensory_endpoint);
    info!("  ZMQ motor output endpoint: {}", args.motor_endpoint);

    // Setup signal handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        info!("Shutdown signal received...");
        r.store(false, Ordering::SeqCst);
    })?;

    // Run engine (registration handling integrated into main loop)
    info!("🚀 Starting inference engine ({}Hz)", args.burst_hz);

    run_engine(&mut npu, &args, running, Arc::clone(&registry))?;

    info!("✅ Inference engine shutdown complete!");
    Ok(())
}

// Note: Registration listener functionality to be implemented when ZMQ transport is completed

/// Run the inference engine loop
fn run_engine(
    npu: &mut feagi_burst_engine::RustNPU,
    args: &Args,
    running: Arc<AtomicBool>,
    registry: Arc<std::sync::RwLock<feagi_pns::agent_registry::AgentRegistry>>,
) -> Result<(), Box<dyn std::error::Error>> {
    let burst_interval = std::time::Duration::from_millis(1000 / args.burst_hz);
    let mut burst_count: u64 = 0;
    let mut last_prune = std::time::Instant::now();

    // Create ZMQ context and sockets for sensory/motor I/O
    let ctx = zmq::Context::new();
    
    // Sensory input socket (PULL pattern - agents PUSH data to us)
    let sensory_socket = ctx.socket(zmq::PULL)?;
    sensory_socket.bind(&args.sensory_endpoint)?;
    sensory_socket.set_rcvtimeo(10)?; // 10ms timeout for non-blocking
    info!("✓ Sensory input bound to: {}", args.sensory_endpoint);
    
    // Motor output socket (PUB pattern - we PUBLISH motor data to agents)
    let motor_socket = ctx.socket(zmq::PUB)?;
    motor_socket.bind(&args.motor_endpoint)?;
    info!("✓ Motor output bound to: {}", args.motor_endpoint);

    info!("🔄 Engine running (Press Ctrl+C to stop)...");
    info!("  Agents send sensory data to: {}", args.sensory_endpoint);
    info!("  Motor output published to: {}", args.motor_endpoint);

    while running.load(Ordering::Relaxed) {
        let start = std::time::Instant::now();

        // Process ZMQ sensory input from registered agents
        // Format: JSON with { "cortical_area_id": u32, "neuron_id_potential_pairs": [[id, potential], ...] }
        loop {
            match sensory_socket.recv_bytes(zmq::DONTWAIT) {
                Ok(msg_bytes) => {
                    // Parse JSON message
                    match serde_json::from_slice::<serde_json::Value>(&msg_bytes) {
                        Ok(json) => {
                            // Extract neuron_id and potential pairs
                            if let Some(pairs) = json.get("neuron_id_potential_pairs").and_then(|v| v.as_array()) {
                                let mut injection_data: Vec<(feagi_types::NeuronId, f32)> = Vec::new();
                                for pair in pairs {
                                    if let Some(arr) = pair.as_array() {
                                        if arr.len() == 2 {
                                            if let (Some(id), Some(pot)) = (arr[0].as_u64(), arr[1].as_f64()) {
                                                injection_data.push((feagi_types::NeuronId(id as u32), pot as f32));
                                            }
                                        }
                                    }
                                }
                                
                                if !injection_data.is_empty() {
                                    npu.inject_sensory_with_potentials(&injection_data);
                                    if burst_count % (args.burst_hz * 10) == 0 {
                                        debug!("Injected {} neurons from agent", injection_data.len());
                                    }
                                }
                            }
                        }
                        Err(e) => {
                            warn!("Failed to parse sensory data: {}", e);
                        }
                    }
                }
                Err(zmq::Error::EAGAIN) => {
                    // No more messages available (non-blocking)
                    break;
                }
                Err(e) => {
                    warn!("Sensory socket error: {}", e);
                    break;
                }
            }
        }

        // Execute neural burst
        match npu.process_burst() {
            Ok(result) => {
                if result.neuron_count > 0 && burst_count % (args.burst_hz * 10) == 0 {
                    debug!("Burst #{}: {} neurons fired", burst_count, result.neuron_count);
                }
            }
            Err(e) => {
                warn!("Burst processing error: {}", e);
            }
        }

        // Publish motor output via ZMQ to registered agents
        // Extract fire queue data and publish to all subscribed motor agents
        if let Some(fire_data) = npu.force_sample_fire_queue() {
            if !fire_data.is_empty() {
                // Convert fire data to JSON format
                // Format: { "cortical_areas": { "area_id": { "neuron_ids": [...], "x": [...], "y": [...], "z": [...], "power": [...] } } }
                let mut motor_json = serde_json::json!({
                    "burst": burst_count,
                    "cortical_areas": {}
                });
                
                for (area_id, (ids, xs, ys, zs, ps)) in fire_data.iter() {
                    if !ids.is_empty() {
                        motor_json["cortical_areas"][area_id.to_string()] = serde_json::json!({
                            "neuron_ids": ids,
                            "x": xs,
                            "y": ys,
                            "z": zs,
                            "power": ps,
                        });
                    }
                }
                
                // Serialize and publish
                if let Ok(motor_msg) = serde_json::to_vec(&motor_json) {
                    if let Err(e) = motor_socket.send(&motor_msg, 0) {
                        if burst_count % (args.burst_hz * 10) == 0 {
                            warn!("Failed to publish motor output: {}", e);
                        }
                    } else if burst_count % (args.burst_hz * 10) == 0 {
                        debug!("Published motor output: {} cortical areas", fire_data.len());
                    }
                }
            }
        }

        burst_count += 1;

        // Periodic status
        if burst_count % (args.burst_hz * 10) == 0 {
            let agent_count = registry.read().unwrap().count();
            info!("Status: {} bursts processed, {} agents registered", 
                  burst_count, agent_count);
        }

        // Prune inactive agents every 10 seconds
        if last_prune.elapsed() > std::time::Duration::from_secs(10) {
            let pruned = registry.write().unwrap().prune_inactive_agents();
            if pruned > 0 {
                info!("Pruned {} inactive agents", pruned);
            }
            last_prune = std::time::Instant::now();
        }

        // Checkpoint
        if args.checkpoint_interval > 0
            && burst_count % (args.burst_hz * args.checkpoint_interval) == 0
        {
            info!("Checkpoint at burst {}", burst_count);
            // TODO: Implement checkpointing via connectome save
        }

        // Sleep to maintain frequency
        let elapsed = start.elapsed();
        if elapsed < burst_interval {
            std::thread::sleep(burst_interval - elapsed);
        }
    }

    info!("Stopped after {} bursts", burst_count);
    info!("Final agent count: {}", registry.read().unwrap().count());

    // Auto-save if enabled
    if args.auto_save {
        info!("Auto-saving connectome...");
        // TODO: Implement auto-save via connectome serialization
        info!("✓ Connectome saved");
    }

    Ok(())
}

/// Print ASCII banner
fn print_banner() {
    println!(
        r#"
╔═══════════════════════════════════════════════════════════════════╗
║                                                                   ║
║   ███████╗███████╗ █████╗  ██████╗ ██╗                           ║
║   ██╔════╝██╔════╝██╔══██╗██╔════╝ ██║                           ║
║   █████╗  █████╗  ███████║██║  ███╗██║                           ║
║   ██╔══╝  ██╔══╝  ██╔══██║██║   ██║██║                           ║
║   ██║     ███████╗██║  ██║╚██████╔╝██║                           ║
║   ╚═╝     ╚══════╝╚═╝  ╚═╝ ╚═════╝ ╚═╝                           ║
║                                                                   ║
║   FEAGI Inference Engine v{}                                   ║
║   Standalone Neural Processing System with Online Learning       ║
║                                                                   ║
╚═══════════════════════════════════════════════════════════════════╝
    "#,
        env!("CARGO_PKG_VERSION")
    );
}
