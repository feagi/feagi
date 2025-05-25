# FEAGI Usage Guide

*Last Updated: January 15, 2025*

This document provides detailed instructions on how to use FEAGI with the current **Rust/RTOS compatible architecture** that implements singleton patterns and direct task spawning.

## Overview

FEAGI 2.0 employs a **unified process architecture** with multiple ZMQ streams and an integrated REST API service. All components run within a single process space using direct task spawning for optimal performance and reliability.

### Key Architecture Features
- **Singleton ConnectomeManager**: Single source of truth for brain state
- **Direct Task Spawning**: No subprocess boundaries for better performance
- **Multi-Stream ZMQ**: Dedicated streams for different protocols
- **Memory-Mapped State**: Zero-copy data synchronization
- **Mission-Critical Reliability**: Clockwork precision for performance-critical applications

## Running FEAGI

### Standard Launch

The recommended way to run FEAGI:

```bash
python -m feagi.main
```

This starts the unified FEAGI system with:
- **FastAPI REST Server**: Port 8001 (direct dependency injection)
- **ZMQ Control Stream**: Port 5561 (legacy agent management)
- **ZMQ REST Stream**: Port 5563 (modern REST-over-ZMQ)
- **ZMQ Visualization Stream**: Port 5562 (neural data broadcasting - all areas)
- **ZMQ Motor Stream**: Port 5564 (real-time motor control - OPU areas only)
- **ZMQ REQ/REP Stream**: Port 5555 (legacy commands)

### Command-Line Options

```bash
python -m feagi.main [OPTIONS]
```

#### Core Options

| Option | Description | Default |
|--------|-------------|---------|
| `--config CONFIG` | Path to configuration file | None |
| `--genome GENOME` | Genome file to load | None |
| `--host HOST` | Server host for all services | 127.0.0.1 |

#### API Configuration

| Option | Description | Default |
|--------|-------------|---------|
| `--api-port PORT` | FastAPI REST server port | 8001 |
| `--api-host HOST` | FastAPI server host | 127.0.0.1 |

#### ZMQ Stream Configuration

| Option | Description | Default |
|--------|-------------|---------|
| `--zmq-control-port PORT` | Control stream port | 5561 |
| `--zmq-rest-port PORT` | REST stream port | 5563 |
| `--zmq-vis-port PORT` | Visualization stream port | 5562 |
| `--zmq-motor-port PORT` | Motor stream port | 5564 |
| `--zmq-req-port PORT` | REQ/REP stream port | 5555 |

#### System Configuration

| Option | Description | Default |
|--------|-------------|---------|
| `--burst-rate RATE` | Neural burst processing rate | 60 |
| `--log-level LEVEL` | Logging level (DEBUG/INFO/WARNING/ERROR) | INFO |

#### Debug Options

| Option | Description | Default |
|--------|-------------|---------|
| `--debug-api` | Enable detailed API request/response logging | Disabled |
| `--debug-npu` | Enable fire queue debugging - shows neuron firing data every burst | Disabled |

For comprehensive debugging information, see the [FEAGI Debugging Guide](guide-how-to-debug.md).

### Usage Examples

```bash
# Standard launch with default settings
python -m feagi.main

# Launch with custom genome
python -m feagi.main --genome /path/to/my_genome.json

# Custom port configuration
python -m feagi.main --api-port 9000 --zmq-rest-port 6000

# Development with debug logging
python -m feagi.main --log-level DEBUG

# Remote access configuration
python -m feagi.main --host 0.0.0.0 --api-port 8001

# Custom burst rate for high-frequency simulation
python -m feagi.main --burst-rate 120

# Debug NPU with detailed fire queue output
python -m feagi.main --debug-npu

# Debug both API and NPU for comprehensive debugging
python -m feagi.main --debug-api --debug-npu

# Test mode with NPU debugging for validation
python -m feagi.main --test --debug-npu --test-duration 30
```

## Architecture Components

### Process Manager
The **Process Manager** coordinates all FEAGI tasks with priority-based resource allocation:

```python
from feagi.process_manager import get_process_manager

# Get singleton process manager
process_manager = get_process_manager()

# Access core components
core_api = process_manager.get_core_api()
connectome_manager = process_manager.get_connectome_manager()
```

### ConnectomeManager (Singleton)
Mission-critical brain state management:

```python
from feagi.bdu.connectome_manager import ConnectomeManager

# Always use singleton instance
connectome = ConnectomeManager.instance()

# Access brain state
cortical_areas = connectome.get_cortical_areas()
neuron_count = connectome.get_total_neuron_count()
```

### State Manager (Memory-Mapped)
Zero-copy state synchronization:

```python
from feagi.core.state_manager import FeagiStateManager, GenomeState

# Get singleton state manager
state_manager = FeagiStateManager.instance()

# Check system state
if state_manager.get_genome_state() == GenomeState.LOADED:
    print("Genome is ready for processing")
```

## ZMQ Multi-Stream Architecture

FEAGI uses **dedicated streams** for different protocols and purposes:

### REST Stream (Port 5563)
Modern REST API operations over ZMQ:

```python
import zmq
import json

# Connect to REST stream
context = zmq.Context()
socket = context.socket(zmq.DEALER)
socket.connect("tcp://localhost:5563")

# Send REST request
request = {
    "method": "GET",
    "route": "/v1/system/health_check",
    "timestamp": int(time.time() * 1000)
}
socket.send_multipart([b"", json.dumps(request).encode()])

# Receive response
response_parts = socket.recv_multipart()
response = json.loads(response_parts[1])
print(f"Status: {response['status']}")
```

### Control Stream (Port 5561)
Legacy agent management and heartbeats:

```python
# Agent registration
control_message = {
    "message_type": "hello",
    "agent_id": "my_agent",
    "agent_type": "godot_bridge",
    "timestamp": int(time.time() * 1000)
}
```

### Visualization Stream (Port 5562)
Real-time neural activity data (publish-subscribe) - **All cortical areas**:

```python
import zmq
from feagi_bytes import ByteStructureDecoder

# Subscribe to neural data (comprehensive brain state)
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.setsockopt(zmq.SUBSCRIBE, b"activity")  # Subscribe to neural activity
socket.connect("tcp://localhost:5562")

# Decode comprehensive neural activity
decoder = ByteStructureDecoder()
while True:
    topic, data = socket.recv_multipart()
    neural_activity = decoder.decode_neuron_flat(data)
    # Process all neural data for visualization...
```

### Motor Stream (Port 5564)
Real-time motor control data (publish-subscribe) - **OPU areas only**:

```python
import zmq
from feagi_bytes import ByteStructureDecoder

# Subscribe to motor control data (OPU areas only)
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.setsockopt(zmq.SUBSCRIBE, b"motor")  # Subscribe to motor commands
socket.connect("tcp://localhost:5564")

# Decode motor control data (optimized for real-time control)
decoder = ByteStructureDecoder()
while True:
    topic, data = socket.recv_multipart()
    motor_data = decoder.decode_neuron_flat(data)
    # Process motor control data for robotic actuation...
```

## API Access

### FastAPI REST Server (Port 8001)
Traditional HTTP REST API:

```python
import requests

# System health check
response = requests.get("http://localhost:8001/v1/system/health_check")
health_data = response.json()

# Get cortical areas
response = requests.get("http://localhost:8001/v1/connectome/cortical_areas")
cortical_areas = response.json()
```

### Interactive Documentation
- **Swagger UI**: http://localhost:8001/docs
- **ReDoc**: http://localhost:8001/redoc

## Configuration

### Configuration File Format

```json
{
  "system": {
    "burst_rate": 60,
    "log_level": "INFO"
  },
  "api": {
    "host": "127.0.0.1",
    "port": 8001
  },
  "zmq": {
    "host": "127.0.0.1",
    "control_port": 5561,
    "rest_port": 5563,
    "vis_port": 5562,
    "motor_port": 5564,
    "req_port": 5555
  },
  "genome": {
    "file_path": "/path/to/genome.json",
    "auto_load": true
  }
}
```

### Environment Variables

```bash
# Core configuration
export FEAGI_CONFIG_FILE="/path/to/config.json"
export FEAGI_GENOME_FILE="/path/to/genome.json"
export FEAGI_LOG_LEVEL="INFO"

# API configuration
export FEAGI_API_HOST="127.0.0.1"
export FEAGI_API_PORT="8001"

# ZMQ configuration
export FEAGI_ZMQ_HOST="127.0.0.1"
export FEAGI_ZMQ_CONTROL_PORT="5561"
export FEAGI_ZMQ_REST_PORT="5563"
export FEAGI_ZMQ_VIS_PORT="5562"
export FEAGI_ZMQ_MOTOR_PORT="5564"
```

## Development Usage

### Programmatic Usage

```python
from feagi.process_manager import get_process_manager
from feagi.bdu.connectome_manager import ConnectomeManager

# Initialize FEAGI programmatically
config = {
    "api": {"host": "127.0.0.1", "port": 8001},
    "zmq": {"host": "127.0.0.1", "rest_port": 5563}
}

# Get process manager and start services
process_manager = get_process_manager()
success = process_manager.start(config)

if success:
    # Access singleton components
    connectome = ConnectomeManager.instance()
    print(f"FEAGI ready with {connectome.get_total_neuron_count()} neurons")
```

### Testing with Singleton Architecture

```python
# In tests, ensure singleton cleanup
def test_feagi_functionality():
    # Test setup
    connectome = ConnectomeManager.instance()
    
    # Test operations
    cortical_area = connectome.create_cortical_area("test_area")
    assert cortical_area is not None
    
    # Cleanup for next test
    connectome.clear()  # Reset singleton state
```

## Performance Considerations

### Mission-Critical Requirements
- **Singleton consistency**: All components use single source of truth
- **Memory-mapped state**: Zero-copy data access for performance
- **Direct task spawning**: No subprocess overhead
- **Priority-based scheduling**: Critical tasks get guaranteed resources

### Monitoring System Health

```python
from feagi.core.state_manager import FeagiStateManager

state_manager = FeagiStateManager.instance()

# Check critical system states
health_check = {
    "genome_loaded": state_manager.get_genome_state(),
    "brain_ready": state_manager.get_brain_state(),
    "burst_engine": state_manager.get_burst_engine_state(),
    "api_ready": state_manager.get_api_state()
}

print(f"System Health: {health_check}")
```

## Troubleshooting

### Common Issues

**Genome not loading**:
```bash
# Check genome file path and format
python -c "from feagi.bdu.connectome_manager import ConnectomeManager; cm = ConnectomeManager.instance(); print(cm.get_genome_info())"
```

**Port conflicts**:
```bash
# Check port availability
ss -tuln | grep -E "8001|5561|5562|5563|5555"
```

**Singleton state issues**:
```python
# Reset singleton state if needed
from feagi.bdu.connectome_manager import ConnectomeManager
ConnectomeManager._instance = None  # Force singleton reset
```

## Related Documentation

- [System Architecture](arch-system-overview.md)
- [ZMQ Architecture](arch-zmq.md)
- [Installation Guide](guide-installation.md)
- [Contribution Guide](guide-contribution.md)
- [Rust/RTOS Migration](arch-rust-rtos-migration.md) 