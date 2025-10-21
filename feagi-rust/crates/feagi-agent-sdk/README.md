# FEAGI Agent SDK (Rust)

Production-ready Rust client library for building FEAGI agents.

## Features

✅ **Automatic Registration** - Register with FEAGI with retry/exponential backoff  
✅ **Background Heartbeat** - Automatic keepalive to prevent agent pruning  
✅ **Reconnection Logic** - Handle network issues gracefully  
✅ **Sensory Data** - Send neuron activation data to FEAGI (ZMQ PUSH)  
✅ **Motor Data** - Receive motor commands from FEAGI (ZMQ SUB)  
✅ **Thread-Safe** - Safe concurrent access across threads  
✅ **Graceful Shutdown** - Automatic deregistration on drop  

## Quick Start

```rust
use feagi_agent_sdk::{AgentClient, AgentConfig, AgentType};

// Create configuration
let config = AgentConfig::new("my_camera", AgentType::Sensory)
    .with_feagi_host("localhost")
    .with_vision_capability("camera", (640, 480), 3, "i_vision")
    .with_heartbeat_interval(5.0);

// Create and connect client
let mut client = AgentClient::new(config)?;
client.connect()?;

// Send sensory data
client.send_sensory_data(vec![
    (0, 50.0),   // neuron_id, potential
    (1, 75.0),
    (2, 30.0),
])?;

// Client automatically deregisters on drop
```

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
feagi-agent-sdk = { path = "../feagi-agent-sdk" }
```

## Configuration

### Agent Types

- `AgentType::Sensory` - Sends sensory data to FEAGI (camera, sensors, etc.)
- `AgentType::Motor` - Receives motor commands from FEAGI (servos, actuators, etc.)
- `AgentType::Both` - Bidirectional agent (both sensory input and motor output)

### Capabilities

#### Vision Capability
```rust
config.with_vision_capability(
    "camera",       // modality
    (640, 480),     // dimensions (width, height)
    3,              // channels (1=grayscale, 3=RGB)
    "i_vision"      // target cortical area
);
```

#### Motor Capability
```rust
config.with_motor_capability(
    "servo",                          // modality
    4,                                // output count
    vec!["o_motor".to_string()]       // source cortical areas
);
```

#### Custom Capability
```rust
use serde_json::json;

config.with_custom_capability("audio", json!({
    "sample_rate": 44100,
    "channels": 2
}));
```

### Network Configuration

```rust
let config = AgentConfig::new("agent_id", AgentType::Sensory)
    // Option 1: Set FEAGI host (uses default ports)
    .with_feagi_host("192.168.1.100")
    
    // Option 2: Set endpoints individually
    .with_registration_endpoint("tcp://192.168.1.100:30001")
    .with_sensory_endpoint("tcp://192.168.1.100:5555")
    .with_motor_endpoint("tcp://192.168.1.100:30005");
```

### Reliability Configuration

```rust
let config = AgentConfig::new("agent_id", AgentType::Sensory)
    .with_heartbeat_interval(5.0)           // heartbeat every 5 seconds
    .with_connection_timeout_ms(5000)       // 5 second timeout
    .with_registration_retries(3);          // retry 3 times before giving up
```

## Examples

### Simple Sensory Agent
```bash
cargo run --example simple_sensory_agent
```

See [`examples/simple_sensory_agent.rs`](examples/simple_sensory_agent.rs) for full code.

### Video Camera Agent

```rust
use feagi_agent_sdk::{AgentClient, AgentConfig, AgentType};

let config = AgentConfig::new("video_camera_01", AgentType::Sensory)
    .with_feagi_host("localhost")
    .with_vision_capability("camera", (640, 480), 3, "i_vision");

let mut client = AgentClient::new(config)?;
client.connect()?;

// In your video processing loop:
for frame in video_frames {
    let neuron_pairs = convert_frame_to_neurons(frame);
    client.send_sensory_data(neuron_pairs)?;
}
```

### Motor Agent

```rust
use feagi_agent_sdk::{AgentClient, AgentConfig, AgentType};

let config = AgentConfig::new("robotic_arm", AgentType::Motor)
    .with_feagi_host("localhost")
    .with_motor_capability("servo", 6, vec!["o_motor".to_string()]);

let mut client = AgentClient::new(config)?;
client.connect()?;

// Receive motor commands
loop {
    if let Some(motor_data) = client.receive_motor_data()? {
        apply_motor_commands(motor_data);
    }
    std::thread::sleep(Duration::from_millis(10));
}
```

## Architecture

### ZMQ Communication

The SDK uses ZeroMQ for all communication with FEAGI:

| Socket Type | Direction | Purpose |
|-------------|-----------|---------|
| REQ/REP | Agent → FEAGI | Registration & Heartbeat |
| PUSH | Agent → FEAGI | Sensory Data |
| SUB | FEAGI → Agent | Motor Commands |

### Heartbeat Service

The heartbeat service runs in a background thread and automatically sends keepalive messages to FEAGI:

- Configured via `heartbeat_interval` (default: 5 seconds)
- Prevents agent from being pruned due to inactivity
- Automatically stops on client drop
- Set interval to 0 to disable

### Reconnection Strategy

The SDK uses exponential backoff for connection retries:

1. Initial backoff: `retry_backoff_ms` (default: 1000ms)
2. Each retry doubles the backoff: 1s → 2s → 4s → 8s → ...
3. Maximum backoff: 60 seconds
4. Maximum retries: `registration_retries` (default: 3)

## Error Handling

All operations return `Result<T, SdkError>`:

```rust
use feagi_agent_sdk::SdkError;

match client.connect() {
    Ok(_) => println!("Connected!"),
    Err(SdkError::Timeout(msg)) => eprintln!("Connection timeout: {}", msg),
    Err(SdkError::RegistrationFailed(msg)) => eprintln!("Registration failed: {}", msg),
    Err(e) => eprintln!("Error: {}", e),
}
```

### Error Types

- `SdkError::Zmq` - ZMQ communication error (retryable)
- `SdkError::Timeout` - Connection timeout (retryable)
- `SdkError::RegistrationFailed` - FEAGI rejected registration
- `SdkError::NotRegistered` - Attempted operation before registration
- `SdkError::InvalidConfig` - Configuration validation failed
- `SdkError::HeartbeatFailed` - Heartbeat not acknowledged

## Thread Safety

`AgentClient` is safe to share across threads:

```rust
use std::sync::Arc;

let client = Arc::new(client);
let client_clone = Arc::clone(&client);

std::thread::spawn(move || {
    // Send data from another thread
    client_clone.send_sensory_data(data)?;
});
```

## Logging

The SDK uses the `log` crate. Initialize with `env_logger`:

```rust
env_logger::Builder::from_default_env()
    .filter_level(log::LevelFilter::Info)
    .init();
```

Set log level via environment variable:
```bash
RUST_LOG=feagi_agent_sdk=debug cargo run
```

## Testing

Run unit tests:
```bash
cargo test
```

Integration tests (requires running FEAGI):
```bash
cargo test --test integration_tests
```

## License

Apache-2.0

## Contributing

See [CONTRIBUTING.md](../../CONTRIBUTING.md) for guidelines.

