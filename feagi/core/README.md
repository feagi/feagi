# FEAGI Core Module

The Core module provides fundamental infrastructure for FEAGI, including resource management, state tracking, and computational backends.

## Overview

This module serves as the foundation for FEAGI, handling:
- Process orchestration and resource management
- System state tracking and synchronization
- Computational backend abstraction (CPU/GPU)
- Transaction management for the genome-connectome bridge

## Components

### State Manager

The `FeagiStateManager` provides a high-performance, memory-mapped state management system that tracks FEAGI's internal states with near-zero overhead access. It exposes methods to:

- Track and update genome and connectome states
- Monitor service states for components
- Provide simulation control
- Support genome transaction management

Example usage:
```python
# Get the state manager instance
state_manager = FeagiStateManager.instance()

# Check system state
if state_manager.is_genome_loaded() and state_manager.is_connectome_ready():
    # Start simulation
    state_manager.set_simulation_state(SimulationState.RUNNING)
```

### Resource Manager

The `ResourceManager` orchestrates processes and allocates hardware resources. It handles:

- Process creation, monitoring, and termination
- CPU/GPU resource detection and allocation
- Thread management
- Memory monitoring
- Cross-process data structure initialization

Example usage:
```python
# Get or create the resource manager
resource_mgr = ResourceManager.get_instance()

# Start a process with specific resource needs
success = resource_mgr.start_process(
    name="burst_engine",
    target=run_burst_engine,
    cpu_allocation=4  # Request 4 CPU cores
)
```

### Backend Framework

The backend framework provides a unified interface for different computational backends. It allows FEAGI to run optimally on various hardware configurations including:

- **CPU Backend**: Optimized for systems without dedicated GPU
- **WebGPU Backend**: Cross-platform GPU acceleration

Example usage:
```python
from feagi.core.backend import get_backend, BackendType

# Get the best available backend
backend = get_backend()

# Or request a specific backend
gpu_backend = get_backend(BackendType.WEBGPU)

# Run computation
result = backend.compute_neuron_updates(neurons, synapses)
```

### Genome Transaction System

The `GenomeTransaction` class provides an atomic way to modify the genome and synchronize those changes with the connectome:

```python
# Create a transaction
with state_manager.begin_genome_transaction_context() as transaction:
    # Make changes
    transaction.record_change("update_cortical_area", area_id, old_props, new_props)
    # Automatically commits at the end of the block
```

## Security Components

The `sec` submodule handles authentication, authorization, and secure communication:

- API authentication and token validation
- ZeroMQ encryption (optional)
- Permission control for sensitive operations

## Hardware Management

The Core module detects available hardware resources and optimizes FEAGI accordingly:

- CPU core detection and allocation
- GPU identification and capability assessment
- Memory monitoring and allocation
- Cross-platform compatibility for different GPU types 