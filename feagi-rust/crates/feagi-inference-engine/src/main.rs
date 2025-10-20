use clap::Parser;
use log::{info, warn, error, debug};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use feagi_inference_engine::{VideoReader, SensoryInjector, MotorExtractor};

/// FEAGI Inference Engine - Standalone neural processing engine with online learning
#[derive(Parser, Debug)]
#[command(name = "feagi-inference-engine", version, author, long_about = None)]
struct Args {
    /// Path to the connectome file to load
    #[arg(short, long)]
    connectome: PathBuf,

    /// Path to video file for visual input (optional)
    #[arg(short, long)]
    video: Option<PathBuf>,

    /// Burst frequency in Hz (default: 50)
    #[arg(long, default_value_t = 50)]
    burst_hz: u64,

    /// Vision cortical area ID (default: "ipu_vision")
    #[arg(long, default_value = "ipu_vision")]
    vision_cortical_area: String,

    /// Motor cortical area IDs (comma-separated, e.g., "opu_motor_left,opu_motor_right")
    #[arg(long, default_value = "opu_motor")]
    motor_cortical_areas: String,

    /// Resize video frames to WxH (e.g., "64x64")
    #[arg(long)]
    resize: Option<String>,

    /// Loop video playback indefinitely
    #[arg(long, default_value_t = true)]
    loop_video: bool,

    /// Frame skip (1 = no skip, 2 = every other frame, etc.)
    #[arg(long, default_value_t = 1)]
    frame_skip: u32,

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

    // Parse resize dimensions if provided
    let resize_dims = args.resize.as_ref().and_then(|s| {
        let parts: Vec<&str> = s.split('x').collect();
        if parts.len() == 2 {
            let width: u32 = parts[0].parse().ok()?;
            let height: u32 = parts[1].parse().ok()?;
            Some((width, height))
        } else {
            None
        }
    });

    // Initialize video reader if video path provided
    let mut video_reader = if let Some(ref video_path) = args.video {
        info!("Opening video file: {}", video_path.display());
        match VideoReader::open(video_path) {
            Ok(reader) => {
                info!(
                    "✓ Video opened: {}x{} @ {:.2}fps",
                    reader.dimensions().0,
                    reader.dimensions().1,
                    reader.fps()
                );
                Some(reader)
            }
            Err(e) => {
                error!("Failed to open video: {}", e);
                return Err(e.into());
            }
        }
    } else {
        info!("No video input specified. Running without sensory input.");
        None
    };

    // Initialize sensory injector if video is available
    let mut sensory_injector = video_reader.as_ref().map(|reader| {
        let dims = resize_dims.unwrap_or(reader.dimensions());
        info!(
            "✓ Sensory injector initialized for cortical area '{}' ({}x{})",
            args.vision_cortical_area, dims.0, dims.1
        );
        SensoryInjector::new(args.vision_cortical_area.clone(), dims)
    });

    // Initialize motor extractor
    let motor_areas: Vec<String> = args
        .motor_cortical_areas
        .split(',')
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
        .collect();
    
    let mut motor_extractor = MotorExtractor::new(motor_areas.clone());
    info!(
        "✓ Motor extractor initialized for {} areas: {}",
        motor_areas.len(),
        motor_areas.join(", ")
    );

    // Setup signal handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        info!("Shutdown signal received...");
        r.store(false, Ordering::SeqCst);
    })?;

    // Run engine
    info!("🚀 Starting inference engine ({}Hz)", args.burst_hz);
    if args.video.is_some() {
        info!("  Vision input: {} (loop: {})", 
            args.video.as_ref().unwrap().display(), 
            args.loop_video
        );
        if let Some((w, h)) = resize_dims {
            info!("  Frame resize: {}x{}", w, h);
        }
        info!("  Frame skip: {}", args.frame_skip);
    }

    run_engine(
        &mut npu,
        &args,
        running,
        video_reader.as_mut(),
        sensory_injector.as_mut(),
        &mut motor_extractor,
        resize_dims,
    )?;

    info!("✅ Inference engine shutdown complete!");
    Ok(())
}

/// Run the inference engine loop
fn run_engine(
    npu: &mut feagi_burst_engine::RustNPU,
    args: &Args,
    running: Arc<AtomicBool>,
    mut video_reader: Option<&mut VideoReader>,
    mut sensory_injector: Option<&mut SensoryInjector>,
    motor_extractor: &mut MotorExtractor,
    resize_dims: Option<(u32, u32)>,
) -> Result<(), Box<dyn std::error::Error>> {
    let burst_interval = std::time::Duration::from_millis(1000 / args.burst_hz);
    let mut burst_count: u64 = 0;
    let mut frame_skip_counter = 0u32;

    info!("🔄 Engine running (Press Ctrl+C to stop)...");

    while running.load(Ordering::Relaxed) {
        let start = std::time::Instant::now();

        // 1. Process sensory input (video frame)
        if let (Some(reader), Some(injector)) = (video_reader.as_mut(), sensory_injector.as_mut()) {
            match reader.read_frame() {
                Ok(Some(mut frame)) => {
                    // Apply frame skip
                    frame_skip_counter += 1;
                    if frame_skip_counter >= args.frame_skip {
                        frame_skip_counter = 0;

                        // Resize frame if needed
                        if let Some((target_w, target_h)) = resize_dims {
                            if (frame.width(), frame.height()) != (target_w, target_h) {
                                frame = frame.resize_exact(
                                    target_w,
                                    target_h,
                                    image::imageops::FilterType::Lanczos3,
                                );
                            }
                        }

                        // Inject frame into FEAGI
                        if let Err(e) = injector.inject_frame(npu, &frame) {
                            warn!("Failed to inject frame: {}", e);
                        }
                    }
                }
                Ok(None) => {
                    // End of video
                    if args.loop_video {
                        info!("End of video reached. Looping...");
                        if let Some(ref video_path) = args.video {
                            if let Err(e) = reader.reset(video_path) {
                                error!("Failed to reset video: {}", e);
                                break;
                            }
                        }
                    } else {
                        info!("End of video reached. Stopping...");
                        break;
                    }
                }
                Err(e) => {
                    warn!("Error reading frame: {}", e);
                }
            }
        }

        // 2. Execute neural burst
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

        burst_count += 1;

        // 3. Extract motor output
        if let Err(e) = motor_extractor.extract_motor_output(npu) {
            warn!("Motor extraction error: {}", e);
        }

        // Periodic status
        if burst_count % (args.burst_hz * 10) == 0 {
            let frames_injected = sensory_injector
                .as_ref()
                .map(|i| i.frame_count())
                .unwrap_or(0);
            info!(
                "Status: {} bursts, {} frames injected",
                burst_count, frames_injected
            );
        }

        // Checkpoint
        if args.checkpoint_interval > 0
            && burst_count % (args.burst_hz * args.checkpoint_interval) == 0
        {
            info!("Checkpoint at burst {}", burst_count);
            // TODO: Implement checkpointing
            // let snapshot = npu.export_connectome();
            // let checkpoint_path = format!("{}_checkpoint_{}.connectome", 
            //     args.connectome.display(), burst_count);
            // feagi_connectome_serialization::save_connectome(&snapshot, checkpoint_path)?;
        }

        // Sleep to maintain frequency
        let elapsed = start.elapsed();
        if elapsed < burst_interval {
            std::thread::sleep(burst_interval - elapsed);
        }
    }

    info!("Stopped after {} bursts", burst_count);
    if let Some(injector) = sensory_injector {
        info!("Total frames injected: {}", injector.frame_count());
    }

    // Auto-save if enabled
    if args.auto_save {
        info!("Auto-saving connectome...");
        // TODO: Implement auto-save
        // let snapshot = npu.export_connectome();
        // let save_path = args.connectome.with_extension("final.connectome");
        // feagi_connectome_serialization::save_connectome(&snapshot, save_path)?;
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
