# FEAGI ZMQ Architecture

This directory contains the high-performance ZeroMQ (ZMQ) implementation for FEAGI 2.0, featuring zero-copy neural data paths, real-time sensor data support, and complete Rust/RTOS migration readiness.

## Architecture Overview

The ZMQ implementation follows a multi-stream architecture optimized for neural data transmission:

```
FEAGI_Connector → [Neural Data] → FEAGI (via ZMQ)
```

## Key Components

### Core Infrastructure

- `neural/`: Neural data structures and protocols
  - `protocols.py`: Protocol identifiers (NEURON_FLAT, NEURON_SPARSE, etc.)
  - `headers.py`: Fixed-size 32-byte headers for all neural data
  - `ring_buffer.py`: Lock-free ring buffer for zero-copy processing

- `memory/`: Memory management for zero allocation
  - `buffer_pool.py`: Pre-allocated buffer pools for neural data

- `platform/`: Platform-specific optimizations
  - `optimizer.py`: Socket optimizations for Linux/macOS/Windows

### Streams

- `streams/`: Specialized communication streams
  - `sensory_neural.py`: Zero-copy neural data ingestion from FEAGI_Connector
  - `motor.py`: Real-time motor command broadcasting
  - `visualization.py`: Brain state visualization with adaptive quality
  - `control.py`: REST/JSON control operations
  - `rest.py`: Pure REST API operations

### Server Components

- `server.py`: Main ZMQ server with all stream management
- `connection_manager.py`: Connection lifecycle management
- `rest_adapter.py`: REST API adapter for control stream

## Performance Features

1. **Zero-Copy Neural Data Paths**
   - Ring buffers with memory-mapped I/O
   - Direct numpy array views into received data
   - No intermediate copies during processing

2. **Static Memory Allocation**
   - Pre-allocated buffer pools per cortical area
   - Fixed-size headers (32 bytes)
   - No dynamic allocation in critical paths

3. **Platform Optimizations**
   - Linux: SO_ZEROCOPY, CPU affinity, NUMA awareness
   - macOS: SO_NOSIGPIPE, optimized buffer sizes
   - Windows: Enhanced I/O threading, larger buffers

## ZMQ Traffic Debugging

FEAGI 2.0 includes a comprehensive, high-performance debugging system for ZMQ traffic analysis with zero overhead when disabled.

### ⚡ Quick Start

**Enable debugging with console output:**
```bash
# Option 1: Command line flags
python3 feagi/main.py --debug-zmq-outbound --debug-zmq-inbound

# Option 2: Environment variables
export FEAGI_DEBUG_ZMQ_OUTBOUND=1
export FEAGI_DEBUG_ZMQ_INBOUND=1
export FEAGI_DEBUG_ZMQ_CONSOLE=1  # Show in console vs log files
python3 feagi/main.py

# Option 3: Runtime control via REST API
curl -X POST http://localhost:8000/v1/debug/zmq/configure \
  -H "Content-Type: application/json" \
  -d '{"outbound_enabled": true, "console_output": true, "debug_level": "summary"}'
```

### 🎛️ Debug Levels

Control the verbosity of debug output:

| Level | Description | Use Case |
|-------|-------------|----------|
| `off` | No debugging (zero overhead) | Production |
| `minimal` | Just endpoint and message counts | Basic monitoring |
| `headers` | Add topics, sizes, timestamps | Connection debugging |
| `summary` | Add data previews (200 chars) | **Default - most useful** |
| `full` | Complete data dumps | Deep protocol analysis |

**Example output at `summary` level:**
```
📤 ZMQ OUTBOUND [13:45:23.123]
   [TARGET] tcp://0.0.0.0:5562
   [TYPE] visualization
   [STATS] Frames: 2, Size: 1024b
   [TAG] Topic: 'brain_activity'
   📄 Frame 0: TEXT: brain_activity
   📄 Frame 1: JSON: {"cortical_areas": {"v1": {"neurons": [1,0,1,0]}, "v2": {...}}}
────────────────────────────────────────
```

### 🎯 Message Type Filtering

Filter debug output by message type:

```bash
# Only show visualization traffic
curl -X POST http://localhost:8000/v1/debug/zmq/filter/messages \
  -H "Content-Type: application/json" \
  -d '["visualization"]'

# Show multiple types
curl -X POST http://localhost:8000/v1/debug/zmq/filter/messages \
  -H "Content-Type: application/json" \
  -d '["sensory", "motor", "visualization"]'

# Clear filters (show all)
curl -X POST http://localhost:8000/v1/debug/zmq/filter/messages \
  -H "Content-Type: application/json" \
  -d '[]'
```

**Available message types:**
- `sensory`: Neural data from sensors/agents to FEAGI
- `motor`: Motor commands from FEAGI to agents/robots
- `visualization`: Brain activity data for visualization clients
- `control`: Control messages for system management
- `rest`: REST API requests over ZMQ
- `heartbeat`: Client heartbeat messages

### 🌐 Endpoint Filtering

Filter by specific ZMQ endpoints:

```bash
# Only debug specific ports
curl -X POST http://localhost:8000/v1/debug/zmq/filter/endpoints \
  -H "Content-Type: application/json" \
  -d '["tcp://0.0.0.0:5562", "tcp://localhost:5558"]'

# Clear endpoint filters
curl -X POST http://localhost:8000/v1/debug/zmq/filter/endpoints \
  -H "Content-Type: application/json" \
  -d '[]'
```

### 🚦 Rate Limiting

Prevent console spam with rate limiting:

```bash
# Limit to 10 messages per second
curl -X POST http://localhost:8000/v1/debug/zmq/rate-limit/10

# High verbosity for development (100 msg/sec)
curl -X POST http://localhost:8000/v1/debug/zmq/rate-limit/100

# Minimal for production monitoring (1 msg/sec)
curl -X POST http://localhost:8000/v1/debug/zmq/rate-limit/1
```

### 📊 Performance Monitoring

Monitor debug system performance and statistics:

```bash
# Get current status and performance metrics
curl -s http://localhost:8000/v1/debug/zmq/status | jq

# Example response:
{
  "inbound_enabled": true,
  "outbound_enabled": true,
  "debug_level": "SUMMARY",
  "console_output": true,
  "rate_limit_per_second": 10,
  "stats": {
    "messages_logged": 1847,
    "messages_filtered": 23,
    "total_bytes": 195992,
    "debug_overhead_ms": 12.34,
    "rate_limited_messages": 5,
    "uptime_seconds": 145.67
  }
}

# Get per-endpoint statistics
curl -s http://localhost:8000/v1/debug/zmq/endpoints | jq

# Reset statistics
curl -X POST http://localhost:8000/v1/debug/zmq/reset-stats
```

### 🔧 Advanced Configuration

**Complete configuration example:**
```bash
curl -X POST http://localhost:8000/v1/debug/zmq/configure \
  -H "Content-Type: application/json" \
  -d '{
    "inbound_enabled": true,
    "outbound_enabled": true,
    "debug_level": "summary",
    "message_filters": ["visualization", "motor"],
    "endpoint_filters": ["tcp://0.0.0.0:5562"],
    "rate_limit_per_second": 20,
    "console_output": true
  }'
```

**Environment variable configuration:**
```bash
export FEAGI_DEBUG_ZMQ_INBOUND=1
export FEAGI_DEBUG_ZMQ_OUTBOUND=1
export FEAGI_DEBUG_ZMQ_CONSOLE=1
export FEAGI_DEBUG_ZMQ_LEVEL=summary
export FEAGI_DEBUG_ZMQ_FILTER_TYPES=visualization,motor
```

### 💡 Debugging Use Cases

**1. Troubleshooting Connection Issues:**
```bash
# Enable minimal debugging to see connection attempts
curl -X POST http://localhost:8000/v1/debug/zmq/configure \
  -d '{"outbound_enabled": true, "debug_level": "minimal", "console_output": true}'
```

**2. Analyzing Data Flow:**
```bash
# Show detailed visualization stream data
curl -X POST http://localhost:8000/v1/debug/zmq/configure \
  -d '{"outbound_enabled": true, "debug_level": "summary", "message_filters": ["visualization"], "console_output": true}'
```

**3. Performance Analysis:**
```bash
# Monitor all traffic with rate limiting
curl -X POST http://localhost:8000/v1/debug/zmq/configure \
  -d '{"inbound_enabled": true, "outbound_enabled": true, "debug_level": "headers", "rate_limit_per_second": 5, "console_output": true}'
```

**4. Deep Protocol Analysis:**
```bash
# Full data dumps for specific endpoint (use carefully!)
curl -X POST http://localhost:8000/v1/debug/zmq/configure \
  -d '{"outbound_enabled": true, "debug_level": "full", "endpoint_filters": ["tcp://0.0.0.0:5562"], "rate_limit_per_second": 1, "console_output": true}'
```

### ⚠️ Performance Impact

- **Disabled**: Absolute zero overhead (no function calls)
- **Enabled (minimal)**: <0.1ms per 1000 messages
- **Enabled (summary)**: <1ms per 100 messages
- **Console output**: ~2x overhead vs file logging
- **Rate limiting**: Prevents system overload

**Best practices:**
- Use `summary` level for most debugging needs
- Enable rate limiting in high-traffic scenarios
- Use message/endpoint filters to reduce noise
- Disable debugging when not needed for zero overhead

### 🆘 Help and Documentation

```bash
# Get comprehensive help
curl -s http://localhost:8000/v1/debug/zmq/help | jq

# Get general debug info
curl -s http://localhost:8000/v1/debug/info | jq
```

## Neural Data Flow

1. **Sensory Data** (FEAGI_Connector → FEAGI):
   ```
   Camera/Sensors → FEAGI_Connector → [Neural Encoding] → ZMQ → FEAGI
   ```

2. **Motor Commands** (FEAGI → Actuators):
   ```
   FEAGI → [Motor Stream] → ZMQ → Motor Controllers
   ```

3. **Visualization** (FEAGI → Brain Visualizer):
   ```
   FEAGI → [Adaptive Quality] → ZMQ → Visualization Clients
   ```

## Protocol Specifications

### Neural Data Header (32 bytes)
```
- Magic: 4 bytes (b'FEAG')
- Protocol ID: 1 byte
- Version: 1 byte
- Flags: 2 bytes (compression, precision, priority)
- Timestamp: 8 bytes (nanoseconds)
- Cortical Area ID: 4 bytes
- Neuron Count: 4 bytes
- Payload Size: 4 bytes
- Sequence Number: 4 bytes
```

## Performance Targets

- Neural data latency: <1ms
- Throughput: 100M neurons/sec
- Zero allocation in data paths
- CPU usage: <5%

## Migration Status

### ✅ Implemented
- Basic ZMQ streams
- Neural data protocols
- Ring buffer implementation
- Platform optimizations framework
- **High-performance debug system with console output**

### 🚧 In Progress
- Zero-copy sensory stream
- Lock-free motor stream
- Adaptive visualization

### ❌ TODO
- Sparse neural protocols
- Multi-modal data support
- Full Rust migration

## Usage

See `arch-zmq.md` for complete architecture documentation and usage examples. 