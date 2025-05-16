---
sidebar_position: 1
---

# Configuration Guide

This guide covers the key configuration options in FEAGI, helping you customize the platform for your specific needs.

## Configuration File Structure

FEAGI uses a hierarchical configuration system with settings organized in YAML files. The main configuration file is `config.yaml` in the root directory.

### Example Configuration

```yaml
feagi:
  version: 2.1.0
  runtime:
    backend: "cpu"
    debug_mode: false
    visualization: true
    
  network:
    host: "0.0.0.0"
    api_port: 8000
    websocket_port: 9000
    
  performance:
    burst_duration_ms: 50
    max_firing_rate: 100
    max_synaptic_connections: 10000
```

## Core Configuration Options

### Backend Selection

FEAGI supports multiple computational backends:

```yaml
runtime:
  backend: "cpu"  # Options: cpu, cuda, tensorrt, wgpu
```

- **cpu**: Standard CPU processing, works on all systems
- **cuda**: NVIDIA GPU acceleration using CUDA
- **tensorrt**: Optimized inference using NVIDIA TensorRT
- **wgpu**: WebGPU interface for cross-platform GPU acceleration

### Network Configuration

Control how FEAGI exposes its services:

```yaml
network:
  host: "0.0.0.0"  # Listen on all interfaces
  api_port: 8000    # REST API port
  websocket_port: 9000  # WebSocket for real-time communication
  zmq_port: 5570    # ZeroMQ for high-performance messaging
```

### Performance Tuning

Fine-tune the neural simulation parameters:

```yaml
performance:
  burst_duration_ms: 50  # Duration of each simulation cycle
  max_firing_rate: 100   # Maximum firing rate per neuron
  threads: 4             # Number of processing threads
  batch_size: 32         # Batch size for GPU processing
```

## Environment Variables

You can override configuration settings using environment variables:

- `FEAGI_BACKEND`: Set the computational backend
- `FEAGI_API_PORT`: Set the REST API port
- `FEAGI_DEBUG`: Enable debug mode (set to 1)

Example:
```bash
export FEAGI_BACKEND=cuda
export FEAGI_API_PORT=9090
python run_feagi.py
```

## Advanced Configuration

### Custom Neuron Models

FEAGI supports defining custom neuron models:

```yaml
neuron_models:
  lif:  # Leaky Integrate and Fire
    threshold: 0.5
    leak_factor: 0.1
    refractory_period_ms: 5
  adaptive:
    threshold: 0.6
    adaptation_factor: 0.2
    recovery_time_ms: 10
```

### Logging Configuration

Configure logging behavior:

```yaml
logging:
  level: "INFO"  # DEBUG, INFO, WARNING, ERROR
  file: "feagi.log"
  rotation: "10MB"
  backup_count: 5
```

## Configuration Profiles

FEAGI supports multiple configuration profiles for different use cases:

```bash
python run_feagi.py --config profiles/high_performance.yaml
```

## Next Steps

- Learn about [Visualization](/user-guide/visualization) to monitor your neural networks
- Explore [Agent Connections](/user-guide/agents) to integrate external systems 