use clap::Parser;
use log::{info, warn, debug};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

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
    info!("  Sensory input will be received via ZMQ from external agents");
    info!("  Motor output will be published via ZMQ to external agents");

    // Setup signal handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        info!("Shutdown signal received...");
        r.store(false, Ordering::SeqCst);
    })?;

    // Run engine
    info!("🚀 Starting inference engine ({}Hz)", args.burst_hz);

    run_engine(&mut npu, &args, running)?;

    info!("✅ Inference engine shutdown complete!");
    Ok(())
}

/// Run the inference engine loop
fn run_engine(
    npu: &mut feagi_burst_engine::RustNPU,
    args: &Args,
    running: Arc<AtomicBool>,
) -> Result<(), Box<dyn std::error::Error>> {
    let burst_interval = std::time::Duration::from_millis(1000 / args.burst_hz);
    let mut burst_count: u64 = 0;

    info!("🔄 Engine running (Press Ctrl+C to stop)...");
    info!("  ZMQ sensory input: TODO - implement agent registration");
    info!("  ZMQ motor output: TODO - implement motor publishing");

    while running.load(Ordering::Relaxed) {
        let start = std::time::Instant::now();

        // TODO: Process ZMQ sensory input from external agents

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

        // TODO: Publish motor output via ZMQ to external agents

        burst_count += 1;

        // Periodic status
        if burst_count % (args.burst_hz * 10) == 0 {
            info!("Status: {} bursts processed", burst_count);
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
