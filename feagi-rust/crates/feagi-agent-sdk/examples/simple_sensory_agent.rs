//! Simple sensory agent example
//!
//! This example demonstrates the basic usage of the FEAGI Agent SDK
//! by creating a simple agent that sends random sensory data.
//!
//! Run with:
//! ```bash
//! cargo run --example simple_sensory_agent
//! ```

use feagi_agent_sdk::{AgentClient, AgentConfig, AgentType};
use std::time::Duration;
use std::thread;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize logging
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .init();
    
    println!("🤖 FEAGI Simple Sensory Agent Example");
    println!("=====================================\n");
    
    // Create agent configuration
    let config = AgentConfig::new("simple_sensory_agent", AgentType::Sensory)
        .with_feagi_host("localhost")
        .with_vision_capability("camera", (10, 10), 1, "i_vision")
        .with_heartbeat_interval(5.0)
        .with_connection_timeout_ms(5000)
        .with_registration_retries(3);
    
    println!("📝 Configuration:");
    println!("   Agent ID: {}", config.agent_id);
    println!("   Registration: {}", config.registration_endpoint);
    println!("   Sensory Input: {}", config.sensory_endpoint);
    println!("   Heartbeat: {}s\n", config.heartbeat_interval);
    
    // Create and connect client
    println!("🔌 Connecting to FEAGI...");
    let mut client = AgentClient::new(config)?;
    client.connect()?;
    
    println!("✅ Connected successfully!\n");
    
    // Send sensory data periodically
    println!("📤 Sending sensory data (press Ctrl+C to stop)...\n");
    
    let mut frame_count = 0;
    loop {
        // Generate sample sensory data (simulating 10x10 grayscale image)
        let mut neuron_pairs = Vec::new();
        
        for neuron_id in 0..100 {
            // Generate pseudo-random potential based on frame and neuron_id
            let potential = ((neuron_id + frame_count) % 100) as f64;
            neuron_pairs.push((neuron_id, potential));
        }
        
        // Send to FEAGI
        client.send_sensory_data(neuron_pairs)?;
        
        frame_count += 1;
        
        if frame_count % 10 == 0 {
            println!("   📊 Sent {} frames", frame_count);
        }
        
        // Wait ~10 FPS
        thread::sleep(Duration::from_millis(100));
    }
    
    // Client automatically deregisters on drop
}

