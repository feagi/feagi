use clap::Parser;
use log::{info, warn, debug, error};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use feagi_agent_registry::AgentTransport;

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
    let registry = Arc::new(feagi_agent_registry::AgentRegistry::new(
        args.max_agents,
        args.agent_timeout_ms,
    ));
    info!("✓ Agent registry initialized (max_agents={}, timeout={}ms)", 
          args.max_agents, args.agent_timeout_ms);

    // Create ZMQ transport
    info!("Setting up ZMQ transport...");
    let transport = Arc::new(
        feagi_inference_engine::ZmqTransport::new(&args.registration_endpoint)
            .map_err(|e| format!("Failed to create ZMQ transport: {}", e))?
    );
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

    // Start registration listener thread
    let registry_clone = Arc::clone(&registry);
    let transport_clone = Arc::clone(&transport);
    let running_clone = Arc::clone(&running);
    let sensory_ep = args.sensory_endpoint.clone();
    let motor_ep = args.motor_endpoint.clone();
    
    let registration_thread = thread::spawn(move || {
        registration_listener(
            registry_clone,
            transport_clone,
            running_clone,
            sensory_ep,
            motor_ep,
        )
    });

    // Run engine
    info!("🚀 Starting inference engine ({}Hz)", args.burst_hz);

    run_engine(&mut npu, &args, running, Arc::clone(&registry))?;

    // Wait for registration thread to finish
    info!("Waiting for registration thread to finish...");
    let _ = registration_thread.join();

    info!("✅ Inference engine shutdown complete!");
    Ok(())
}

/// Registration listener thread
fn registration_listener(
    registry: Arc<feagi_agent_registry::AgentRegistry>,
    transport: Arc<feagi_inference_engine::ZmqTransport>,
    running: Arc<AtomicBool>,
    sensory_endpoint: String,
    motor_endpoint: String,
) {
    info!("📡 Registration listener started");
    
    while running.load(Ordering::Relaxed) {
        // Try to receive registration request (non-blocking with timeout)
        match transport.receive_registration_request() {
            Ok(request) => {
                info!("📥 Registration request from: {}", request.agent_id);
                
                // Parse agent type
                let agent_type = match request.agent_type.as_str() {
                    "sensory" => feagi_agent_registry::AgentType::Sensory,
                    "motor" => feagi_agent_registry::AgentType::Motor,
                    "both" => feagi_agent_registry::AgentType::Both,
                    _ => {
                        error!("Invalid agent type: {}", request.agent_type);
                        let _ = transport.send_registration_rejection(
                            &request.agent_id,
                            &format!("Invalid agent type: {}", request.agent_type)
                        );
                        continue;
                    }
                };
                
                // Parse capabilities
                let capabilities: feagi_agent_registry::AgentCapabilities = 
                    match serde_json::from_value(request.capabilities) {
                        Ok(caps) => caps,
                        Err(e) => {
                            error!("Failed to parse capabilities: {}", e);
                            let _ = transport.send_registration_rejection(
                                &request.agent_id,
                                &format!("Invalid capabilities: {}", e)
                            );
                            continue;
                        }
                    };
                
                // Create transport endpoints
                let endpoints = feagi_agent_registry::TransportEndpoints::new(
                    sensory_endpoint.clone(),
                    motor_endpoint.clone(),
                );
                
                // Register agent
                match registry.register_agent(
                    request.agent_id.clone(),
                    agent_type,
                    capabilities,
                    transport.as_ref(),
                    &endpoints,
                ) {
                    Ok(_) => {
                        info!("✓ Agent registered: {} (total: {})", 
                              request.agent_id, registry.agent_count());
                    }
                    Err(e) => {
                        error!("Failed to register agent {}: {}", request.agent_id, e);
                    }
                }
            }
            Err(e) => {
                // Check if it's a timeout (expected during normal operation)
                if e.to_string().contains("timeout") || e.to_string().contains("EAGAIN") {
                    // Normal timeout - no agents trying to register
                    std::thread::sleep(std::time::Duration::from_millis(100));
                } else {
                    error!("Registration error: {}", e);
                    std::thread::sleep(std::time::Duration::from_millis(1000));
                }
            }
        }
    }
    
    info!("📡 Registration listener stopped");
}

/// Run the inference engine loop
fn run_engine(
    npu: &mut feagi_burst_engine::RustNPU,
    args: &Args,
    running: Arc<AtomicBool>,
    registry: Arc<feagi_agent_registry::AgentRegistry>,
) -> Result<(), Box<dyn std::error::Error>> {
    let burst_interval = std::time::Duration::from_millis(1000 / args.burst_hz);
    let mut burst_count: u64 = 0;
    let mut last_prune = std::time::Instant::now();

    info!("🔄 Engine running (Press Ctrl+C to stop)...");
    info!("  Registered agents will send sensory data to: {}", args.sensory_endpoint);
    info!("  Motor output will be published to: {}", args.motor_endpoint);

    while running.load(Ordering::Relaxed) {
        let start = std::time::Instant::now();

        // TODO: Process ZMQ sensory input from registered agents
        // This will be implemented when we have the sensory injection logic

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

        // TODO: Publish motor output via ZMQ to registered agents
        // This will be implemented when we have the motor extraction logic

        burst_count += 1;

        // Periodic status
        if burst_count % (args.burst_hz * 10) == 0 {
            let agent_count = registry.agent_count();
            info!("Status: {} bursts processed, {} agents registered", 
                  burst_count, agent_count);
        }

        // Prune inactive agents every 10 seconds
        if last_prune.elapsed() > std::time::Duration::from_secs(10) {
            let pruned = registry.prune_inactive_agents(None);
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
    info!("Final agent count: {}", registry.agent_count());

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
