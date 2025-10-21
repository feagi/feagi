# FEAGI Core

FEAGI (Framework for Evolutionary Artificial General Intelligence) Core is the main neural simulation engine that provides brain-like processing capabilities with support for CPU and GPU acceleration.

## System Requirements

### Minimum Requirements
- **OS**: Linux (Ubuntu 20.04+), macOS (11+), or Windows (10/11)
- **Python**: 3.9 or higher (3.10+ recommended)
- **RAM**: 8 GB minimum, 16 GB recommended
- **CPU**: Multi-core processor (4+ cores recommended)
- **Disk**: 2 GB for installation + space for brain states

### Optional GPU Acceleration
FEAGI supports multiple GPU backends for 5-50x performance improvement:

| Platform | Recommended GPU Backend | Requirements |
|----------|------------------------|--------------|
| **macOS (Apple Silicon)** | wgpu (Metal) | M1/M2/M3/M4 chip |
| **Linux (NVIDIA)** | PyTorch (CUDA) or CuPy | CUDA-capable GPU, drivers |
| **Linux (AMD)** | wgpu (Vulkan) | Vulkan-capable GPU, drivers |
| **Windows (NVIDIA)** | PyTorch (CUDA) | CUDA-capable GPU, drivers |
| **Windows (AMD/Intel)** | wgpu (DirectX 12) | DirectX 12-capable GPU |

**Note**: GPU acceleration is optional. FEAGI runs efficiently on CPU-only systems.

## Deployment

### Prerequisites
- Python 3.9+ 
- Rust toolchain (install from https://rustup.rs/)

### Build Rust NPU (Required)
FEAGI requires the Rust NPU extension for high-performance neural processing:

```bash
# Linux/macOS
cd feagi_core/feagi-rust
cargo build --release --workspace
cp target/release/libfeagi_rust.* ../../feagi_rust.so

# Windows
cd feagi_core\feagi-rust
cargo build --release --workspace
copy target\release\feagi_rust.dll ..\..\feagi_rust.pyd
```
### Platform-Specific Setup

#### Linux (Ubuntu/Debian)

```bash
# Install system dependencies
sudo apt update
sudo apt install -y python3 python3-pip python3-venv build-essential curl

# Install Rust toolchain (for Rust extensions)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env

# Optional: NVIDIA GPU support
# sudo apt install -y nvidia-cuda-toolkit  # For CUDA support

# Deploy FEAGI
cd feagi_core
python3 -m venv venv
source venv/bin/activate
pip install -e .
python -m feagi.main
```

#### macOS

```bash
# Install Homebrew (if not already installed)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install dependencies
brew install python@3.11 rust

# Deploy FEAGI
cd feagi_core
python3 -m venv venv
source venv/bin/activate
pip install -e .
python -m feagi.main
```

**Apple Silicon GPU Note**: FEAGI automatically uses Metal GPU acceleration on M1/M2/M3/M4 chips via the wgpu backend.

#### Windows

```powershell
# Install Python 3.11+ from https://www.python.org/downloads/
# During installation, check "Add Python to PATH"

# Install Rust toolchain from https://rustup.rs/
# Download and run rustup-init.exe, choose default installation

# Optional: NVIDIA GPU support
# Install CUDA Toolkit from https://developer.nvidia.com/cuda-downloads

# Deploy FEAGI (in PowerShell)
cd feagi_core
python -m venv venv
.\venv\Scripts\Activate.ps1
pip install -e .
python -m feagi.main
```

**Windows GPU Note**: For NVIDIA GPUs, install CUDA Toolkit before running FEAGI for automatic GPU acceleration.

### Quick Start (All Platforms)

Once platform dependencies are installed:

```bash
# 1. Create and activate virtual environment
cd feagi_core
python3 -m venv venv
source venv/bin/activate  # Windows: .\venv\Scripts\Activate.ps1

# 2. Install FEAGI with dependencies
pip install -e .

# 3. Start FEAGI
python -m feagi.main
```

### Verify Deployment

Once FEAGI starts, verify it's running:
```bash
# Check health endpoint
curl http://localhost:8000/v1/system/health_check

# Expected response: {"status":"healthy",...}
```

FEAGI is now ready to accept connections on:
- **REST API**: `http://localhost:8000` (web interface)
- **ZMQ Sensory**: `tcp://localhost:5558` (agent input)
- **ZMQ Motor**: `tcp://localhost:5564` (agent output)

### Communication Options: ZMQ vs Shared Memory (SHM)

FEAGI supports two communication methods:

**ZMQ (Default)**: Network-based communication
- Works across machines (remote agents)
- Automatic serialization/deserialization
- Lower throughput (~100-200 MB/s)
- Works out-of-the-box, no setup required

**Shared Memory (Recommended for local agents)**: Memory-mapped communication
- Zero-copy, ultra-low latency (<1ms)
- High throughput (500+ MB/s)
- Ideal for video agents and visualizers
- Requires SHM directory setup (one-time)

### Enable Shared Memory (Optional but Recommended)

For high-performance local communication, configure SHM storage:

**Linux (Optimal - uses RAM-backed tmpfs):**
```bash
# 1. Build Rust NPU (see "Build Rust NPU" section above)

# 2. Install FEAGI core with dependencies
pip install -e .

# 3. Start FEAGI with default configuration
python -m feagi.main
# Already configured by default - /dev/shm is RAM-backed
# Verify tmpfs is available:
df -h /dev/shm

# Optional: Set explicitly
export FEAGI_SHM_DIR=/dev/shm
```

**macOS (uses /tmp):**
```bash
# Uses /tmp by default (often RAM-backed)
# Optional: Create dedicated RAM disk for best performance
diskutil erasevolume HFS+ "RAMDisk" `hdiutil attach -nomount ram://8388608`
export FEAGI_SHM_DIR=/Volumes/RAMDisk

# Make permanent (add to ~/.zshrc):
echo 'export FEAGI_SHM_DIR=/Volumes/RAMDisk' >> ~/.zshrc
```

**Windows (requires RAM disk):**
```powershell
# 1. Install ImDisk Toolkit (free): https://sourceforge.net/projects/imdisk-toolkit/
# 2. Create RAM disk:
imdisk -a -s 4G -m R: -p "/fs:ntfs /q /y"

# 3. Set environment variable (persists across reboots):
[Environment]::SetEnvironmentVariable("FEAGI_SHM_DIR", "R:\feagi_shm", "User")
```

**Benefits of SHM:**
- ✅ 5-10x faster than ZMQ for video streaming
- ✅ Zero SSD wear (data stays in RAM)
- ✅ Lower CPU overhead (<1% vs 5-10% for ZMQ)
- ✅ Perfect for high-resolution video agents

**When to use each:**
- **Local video agents**: Use SHM (configure as above)
- **Remote agents**: Use ZMQ (default, no setup)
- **Simple sensors**: Either works (ZMQ is easier)

## GPU Acceleration

### Automatic GPU Detection

FEAGI automatically detects and uses available GPUs. On first start, you'll see:

```
INFO: GPU Backend: PyTorch CUDA (NVIDIA GeForce RTX 3080)
INFO: Hybrid CPU/GPU mode enabled (threshold: 1M synapses)
```

### GPU Configuration

GPU behavior is controlled in `feagi_configuration.toml`:

```toml
[neural.hybrid]
enabled = true                # Enable intelligent CPU/GPU hybrid processing
gpu_threshold = 1000000       # Use GPU for workloads ≥ 1M synapses
keepalive_enabled = true      # Keep GPU warm to prevent cold-start overhead
keepalive_interval = 30.0     # GPU keep-alive interval (seconds)

[resources]
use_gpu = true                # Enable GPU acceleration if available
gpu_memory_fraction = 0.8     # Use 80% of available GPU memory
```

### Performance Expectations

| Brain Size | CPU Only | GPU (NVIDIA/Apple Silicon) | Speedup |
|------------|----------|---------------------------|---------|
| Small (<100K neurons) | 50-100 ms/burst | 40-80 ms/burst | 1.2-1.5x |
| Medium (100K-1M neurons) | 200-500 ms/burst | 50-100 ms/burst | 4-5x |
| Large (1M-10M neurons) | 2-5 sec/burst | 100-300 ms/burst | 10-50x |

**Note**: Actual performance depends on genome complexity, connectivity, and hardware.

### GPU Backend Selection

FEAGI automatically selects the best available GPU backend:

| Platform | Priority Order |
|----------|---------------|
| **macOS (Apple Silicon)** | 1. wgpu (Metal) → 2. PyTorch (MPS) → 3. CPU |
| **Linux (NVIDIA)** | 1. PyTorch (CUDA) → 2. CuPy (CUDA) → 3. wgpu (Vulkan) → 4. CPU |
| **Linux (AMD)** | 1. wgpu (Vulkan) → 2. PyTorch (CPU) |
| **Windows (NVIDIA)** | 1. PyTorch (CUDA) → 2. wgpu (DirectX 12) → 3. CPU |
| **Windows (AMD/Intel)** | 1. wgpu (DirectX 12) → 2. CPU |

For detailed GPU architecture information, see [docs/arch-gpu.md](docs/arch-gpu.md).

## Quick Start

### Default Ports
- **FastAPI REST API**: Port 8000 (HTTP)
- **ZMQ REST API**: Port 5563 (primary API interface)
- **ZMQ Visualization**: Port 5562 (neural data)
- **ZMQ Motor**: Port 5564 (motor output)
- **ZMQ Sensory**: Port 5558 (sensory input)

## Architecture

FEAGI Core implements a **Rust/RTOS compatible architecture** with:

- **Hybrid CPU/GPU Processing**: Intelligent workload distribution between CPU and GPU
- **Singleton ConnectomeManager**: Centralized brain state management
- **Direct Task Spawning**: No subprocess boundaries for optimal performance
- **Multi-Stream ZMQ**: Dedicated communication channels for different protocols
- **Memory-Mapped State**: Zero-copy data synchronization
- **Configuration-Driven**: All hosts and ports from TOML configuration

### Key Components

**Neural Processing Unit (NPU)**:
- Rust-accelerated burst engine for neural simulation
- Hybrid CPU/GPU processing with automatic workload distribution
- Sparse computation for efficiency (only processes active neurons)
- Support for plasticity and learning mechanisms (STDP, memory formation)

**Communication Protocols**:

**REST API over ZMQ (Port 5563)**: Primary interface for all API operations including:
- Agent registration and heartbeat
- Brain state queries and modifications
- System status and control
- Configuration management

**Specialized Data Streams**:
- **Visualization Stream (5562)**: Real-time neural activity broadcasting with LZ4 compression
- **Motor Stream (5564)**: High-frequency motor command output
- **Sensory Stream (5558)**: Sensory data input processing with binary neuron encoding

## Configuration

FEAGI uses `feagi_configuration.toml` for all settings:

```toml
[api]
host = "0.0.0.0"
port = 8000

[ports]
zmq_rest_port = 5563        # Primary API interface
zmq_visualization_port = 5562
zmq_motor_port = 5564
zmq_sensory_port = 5558

[zmq]
host = "0.0.0.0"

[neural.hybrid]
enabled = true              # Enable GPU acceleration
gpu_threshold = 1000000     # Use GPU for workloads ≥ 1M synapses
keepalive_enabled = true    # Keep GPU warm
keepalive_interval = 30.0   # GPU keep-alive interval

[resources]
use_gpu = true              # Enable GPU if available
gpu_memory_fraction = 0.8   # Use 80% of GPU memory

[burst_engine]
mode = "inference"          # "inference" or "design" mode
max_supported_neurons = 10000000  # Maximum neurons (10M)

[compression]
enabled = true              # Enable LZ4 compression for ZMQ streams
compress_visualization = true
compress_motor = true
compress_sensory = false    # Real-time, keep uncompressed
```

### Environment Variables

Override configuration via environment variables:

| Variable | Description | Default |
|----------|-------------|---------|
| `FEAGI_API_HOST` | REST API host | `0.0.0.0` |
| `FEAGI_API_PORT` | REST API port | `8000` |
| `FEAGI_ZMQ_HOST` | ZMQ communication host | `0.0.0.0` |
| `FEAGI_SHM_DIR` | Shared memory directory | OS-specific |
| `FEAGI_AGENT_DEFAULT_HOST` | Default agent host | `127.0.0.1` |

Complete configuration reference: [docs/configuration.md](docs/configuration.md)

## Troubleshooting

### Platform-Specific Issues

#### Linux

**Issue**: `ImportError: libcudart.so.11.0: cannot open shared object file`
```bash
# Solution: Install CUDA toolkit or disable GPU
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
# Or disable GPU in feagi_configuration.toml: use_gpu = false
```

**Issue**: Permission denied on `/dev/shm`
```bash
# Solution: Check tmpfs permissions
sudo chmod 1777 /dev/shm
```

#### macOS

**Issue**: `xcrun: error: invalid active developer path`
```bash
# Solution: Install Xcode Command Line Tools
xcode-select --install
```

**Issue**: Python not found after Homebrew install
```bash
# Solution: Add Homebrew Python to PATH
echo 'export PATH="/opt/homebrew/opt/python@3.11/bin:$PATH"' >> ~/.zshrc
source ~/.zshrc
```

#### Windows

**Issue**: `error: Microsoft Visual C++ 14.0 or greater is required`
```powershell
# Solution: Install Visual Studio Build Tools
# Download from: https://visualstudio.microsoft.com/downloads/
# Select "Desktop development with C++" workload
```

**Issue**: PowerShell execution policy prevents running scripts
```powershell
# Solution: Allow script execution (run as Administrator)
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

**Issue**: `pip` command not found
```powershell
# Solution: Use python -m pip instead
python -m pip install -e .
```

### GPU Issues

**GPU not detected**:
```bash
# Check GPU status
python -c "import torch; print(f'CUDA: {torch.cuda.is_available()}')"

# Check installed backends
python -c "from feagi.bdu.models.array_backend import detect_available_backends; print(detect_available_backends())"
```

**Out of GPU memory**:
- Reduce `gpu_memory_fraction` in `feagi_configuration.toml` (try 0.6 or 0.5)
- Reduce brain size or increase `gpu_threshold`

**Performance issues**:
- Enable debug mode to see GPU utilization: `python -m feagi.main --debug-npu`
- Check GPU is actually being used (not falling back to CPU)

## Development

### Running Tests
```bash
# Install development dependencies
pip install -e ".[dev]"

# Architecture compliance tests
pytest tests/system/test_architecture_compliance.py -v

# Unit tests
pytest tests/unit/ -v

# Integration tests
pytest tests/integration/ -v

# GPU-specific tests
pytest tests/integration/test_gpu_acceleration.py -v
```

### Debug Mode
```bash
# Enable API debugging
python -m feagi.main --debug-api

# Enable neural processing debugging
python -m feagi.main --debug-npu

# Full debug mode with verbose logging
python -m feagi.main --debug-api --debug-npu --log-level DEBUG

# Check GPU backend selection
python -m feagi.main --log-level DEBUG 2>&1 | grep -i gpu
```

## Performance Optimization

### Best Practices

**For Large Brains (>1M neurons)**:
1. Enable GPU acceleration (see GPU Acceleration section)
2. Use Shared Memory (SHM) for local agents
3. Set `mode = "inference"` in `[burst_engine]` configuration
4. Enable compression for visualization streams
5. Increase `gpu_threshold` if GPU memory is limited

**For Real-Time Applications**:
1. Reduce `burst_engine_timestep` for faster response
2. Use SHM instead of ZMQ for local communication
3. Disable visualization during training (`enabled = false` in `[visualization]`)
4. Enable GPU keep-alive to prevent cold-start delays

**For Memory-Constrained Systems**:
1. Set `mode = "inference"` to minimize memory allocation
2. Reduce `fcl_capacity_multiplier` and `fire_queue_capacity_multiplier`
3. Disable GPU if CPU has more available RAM
4. Use smaller genome designs

### Benchmarking

```bash
# Run performance benchmarks
python -m feagi.tools.benchmark --genome your_genome.json

# Compare CPU vs GPU performance
python -m feagi.tools.benchmark --compare-backends

# Profile specific brain operations
python -m feagi.main --profile --genome your_genome.json
```

## Documentation

### User Guides
- [Usage Guide](docs/guide-usage.md) - Complete usage instructions
- [Configuration Reference](docs/configuration.md) - Complete configuration options

### Architecture
- [ZMQ Communication](docs/arch-zmq.md) - Multi-stream ZMQ architecture
- [GPU Acceleration](docs/arch-gpu.md) - GPU backend architecture and performance
- [Embedded Mode](docs/arch-embedded-mode.md) - Resource-constrained deployments
- [Shared Memory Specification](docs/spec-shared-memory-streaming.md) - SHM protocol details

### Development
- [Burst Engine](docs/burst_engine.md) - Neural processing unit internals
- [Rust NPU Integration](docs/RUST_NPU_INTEGRATION_SUMMARY.md) - Rust acceleration
- [Contributing](CONTRIBUTING.md) - Development guidelines

## Support

### Getting Help

- **Issues**: [GitHub Issues](https://github.com/feagi/feagi-2.0/issues)
- **Documentation**: Full documentation in `/docs` directory
- **Examples**: See `examples/` directory for sample agents

### Common Questions

**Q: Does FEAGI require a GPU?**  
A: No, FEAGI runs efficiently on CPU-only systems. GPU acceleration is optional and provides 5-50x speedup for large brains.

**Q: Which GPU should I use?**  
A: NVIDIA GPUs (CUDA) have the most mature support. Apple Silicon (M1/M2/M3/M4) and AMD GPUs (via Vulkan) are also well-supported.

**Q: Can I run FEAGI on a Raspberry Pi?**  
A: Yes! FEAGI is designed for embedded systems. Use CPU mode and smaller genome designs. See [Embedded Mode](docs/arch-embedded-mode.md).

**Q: How do I deploy FEAGI in the cloud?**  
A: FEAGI is cloud-ready with Docker support and TOML configuration. Mount `feagi_configuration.toml` as a ConfigMap in Kubernetes.

## License

Copyright 2016-2025 Neuraville Inc. Licensed under the Apache License, Version 2.0.
