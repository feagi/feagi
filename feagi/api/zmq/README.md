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