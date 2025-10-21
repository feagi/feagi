# FEAGI Agent SDK - Python Bindings

Python bindings for the Rust-based FEAGI Agent SDK. This package provides a high-performance, production-ready client library for building FEAGI agents in Python.

## Features

✅ **Rust-Backed Performance** - Core logic runs in Rust for maximum speed  
✅ **Pythonic API** - Simple, idiomatic Python interface  
✅ **Automatic Registration** - Handles connection and registration automatically  
✅ **Background Heartbeat** - Keeps agent alive with automatic keepalive  
✅ **Reconnection Logic** - Handles network issues gracefully  
✅ **Thread-Safe** - Safe to use from multiple Python threads  

## Installation

### From Source

```bash
# Install maturin (build tool for PyO3)
pip install maturin

# Build and install in development mode
cd feagi_core/feagi-rust/crates/feagi-agent-sdk-py
maturin develop --release
```

### From Wheel (when available)

```bash
pip install feagi-agent-sdk-py
```

## Quick Start

```python
from feagi_agent_sdk_py import PyAgentClient, PyAgentConfig, AgentType

# Create configuration
config = PyAgentConfig("my_camera", AgentType.Sensory)
config.with_feagi_host("localhost")
config.with_vision_capability("camera", 640, 480, 3, "i_vision")
config.with_heartbeat_interval(5.0)

# Create and connect client
client = PyAgentClient(config)
client.connect()

# Send sensory data
client.send_sensory_data([
    (0, 50.0),   # (neuron_id, potential)
    (1, 75.0),
    (2, 30.0),
])

# For motor agents - receive motor commands
motor_data = client.receive_motor_data()
if motor_data is not None:
    print(f"Motor command: {motor_data}")

# Client automatically deregisters when garbage collected
```

## Complete Example

```python
import time
from feagi_agent_sdk_py import PyAgentClient, PyAgentConfig, AgentType

# Configure agent
config = PyAgentConfig("video_camera_01", AgentType.Sensory)
config.with_feagi_host("localhost")
config.with_vision_capability("camera", 640, 480, 3, "i_vision")
config.with_heartbeat_interval(5.0)
config.with_connection_timeout_ms(5000)
config.with_registration_retries(3)

# Create client
client = PyAgentClient(config)

# Connect (with automatic retry)
print("Connecting to FEAGI...")
client.connect()
print(f"✅ Connected as: {client.agent_id()}")

# Send data in a loop
frame_count = 0
while True:
    # Generate sample data (simulating video frames)
    neuron_pairs = []
    for i in range(100):
        neuron_pairs.append((i, (i + frame_count) % 100))
    
    # Send to FEAGI
    client.send_sensory_data(neuron_pairs)
    
    frame_count += 1
    if frame_count % 100 == 0:
        print(f"📊 Sent {frame_count} frames")
    
    time.sleep(0.1)  # ~10 FPS
```

## API Reference

### AgentType

```python
class AgentType(Enum):
    Sensory  # Sends sensory data to FEAGI
    Motor    # Receives motor commands from FEAGI
    Both     # Bidirectional agent
```

### PyAgentConfig

```python
config = PyAgentConfig(agent_id: str, agent_type: AgentType)

# Network configuration
config.with_feagi_host(host: str)  # Sets all endpoints
config.with_registration_endpoint(endpoint: str)
config.with_sensory_endpoint(endpoint: str)
config.with_motor_endpoint(endpoint: str)

# Capabilities
config.with_vision_capability(
    modality: str,
    width: int,
    height: int,
    channels: int,
    target_cortical_area: str
)

config.with_motor_capability(
    modality: str,
    output_count: int,
    source_cortical_areas: List[str]
)

config.with_custom_capability(key: str, value_json: str)

# Reliability
config.with_heartbeat_interval(interval: float)  # seconds
config.with_connection_timeout_ms(timeout_ms: int)
config.with_registration_retries(retries: int)

# Validation
config.validate()  # Raises ValueError if invalid
```

### PyAgentClient

```python
client = PyAgentClient(config: PyAgentConfig)

# Connection
client.connect()  # Connect and register with FEAGI

# Sensory data
client.send_sensory_data(neuron_pairs: List[Tuple[int, float]])

# Motor data
motor_data: Optional[str] = client.receive_motor_data()  # Non-blocking

# Status
is_registered: bool = client.is_registered()
agent_id: str = client.agent_id()
```

## Error Handling

```python
from feagi_agent_sdk_py import PyAgentClient, PyAgentConfig, AgentType

try:
    config = PyAgentConfig("test", AgentType.Sensory)
    config.with_feagi_host("localhost")
    config.validate()  # Validate configuration
    
    client = PyAgentClient(config)
    client.connect()  # May raise connection errors
    
except ValueError as e:
    print(f"Configuration error: {e}")
except Exception as e:
    print(f"Connection error: {e}")
```

## Performance

The Rust-backed implementation provides significant performance benefits:

- **10-100x faster** than pure Python for message serialization
- **Near-zero overhead** for heartbeat (runs in separate Rust thread)
- **Efficient memory usage** with Rust's ownership model
- **Thread-safe** without GIL contention for internal operations

## Architecture

```
┌──────────────────────────────────┐
│  Python Application              │
│  (your agent code)               │
└──────────────┬───────────────────┘
               │ Python API
┌──────────────▼───────────────────┐
│  feagi-agent-sdk-py (PyO3)       │
│  - Thin Python wrapper           │
│  - Type conversions              │
└──────────────┬───────────────────┘
               │ Rust FFI
┌──────────────▼───────────────────┐
│  feagi-agent-sdk (Rust)          │
│  - Core logic                    │
│  - ZMQ communication             │
│  - Heartbeat service             │
│  - Reconnection handling         │
└──────────────────────────────────┘
```

## Comparison with Pure Python

### feagi-agent-sdk-py (This Package)
✅ Rust-backed, high performance  
✅ Automatic heartbeat in separate thread  
✅ Built-in reconnection logic  
✅ Thread-safe by design  
✅ Type-safe configuration  

### Pure Python Connector
❌ Slower message processing  
❌ Manual heartbeat management  
❌ Custom error handling needed  
❌ GIL contention on threads  

## Development

### Building from Source

```bash
# Install Rust (if not already installed)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Install maturin
pip install maturin

# Build in debug mode
maturin develop

# Build in release mode (faster)
maturin develop --release

# Run tests
cargo test
```

### Creating Wheels

```bash
# Build wheel for current platform
maturin build --release

# Build wheels for multiple platforms (requires cross-compilation setup)
maturin build --release --target x86_64-unknown-linux-gnu
maturin build --release --target aarch64-apple-darwin
```

## License

Apache-2.0

## Contributing

See [CONTRIBUTING.md](../../../../CONTRIBUTING.md) for guidelines.

