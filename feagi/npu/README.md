# FEAGI Neural Processing Unit (NPU)

*Last Updated: May 15, 2025*

## Overview

The Neural Processing Unit (NPU) is responsible for simulating neuron dynamics, managing burst execution, and handling all neural activity within FEAGI. As the computational core of the brain simulation, the NPU provides high-performance, deterministic neural processing suitable for both research and real-time applications.

## Key Components

### Burst Engine

The Burst Engine manages the neuron firing dynamics and coordinates simulation timing. Key features:

- **Standby Mode**: Initializes without requiring a genome, breaking circular dependencies
- **State Management**: Uses explicit state transitions with emoji-based logging
- **Dependency Injection**: No global state, all dependencies are passed explicitly
- **RTOS-Friendly**: Deterministic timing and no dynamic allocations in the main loop
- **Load Shedding**: Areas marked with `__shed` have their FCL content dropped when the system can't maintain target frequency

```python
# Create and initialize
engine = BurstEngine(connectome_manager, fcl_manager)

# Start the engine
engine.run()

# When genome is loaded
engine.update_with_genome()

# Graceful shutdown
engine.stop()
```

### FCL Manager

The FCL (Fire Candidate List) Manager maintains the temporal history of neuron firing patterns:

- **Roaring Bitmaps**: Efficient sparse data structure for representing neuron activations
- **Historical Tracking**: Maintains multiple timesteps of neuron firing history
- **Area-Based FCLs**: Maintains separate FCLs for each cortical area
- **Membrane Potential**: Handles membrane potential updates from firing neurons

```python
# Get all currently firing neurons
active_neurons = fcl_manager.get_global_fcl()

# Get neurons that fired in the last 3 timesteps
recent_activity = fcl_manager.get_neurons_fired_in_last_n_steps(3)

# Get neurons firing in a specific cortical area
area_activity = fcl_manager.get_area_fcl(area_id)
```

### FCL Sampler

The FCL Sampler provides configurable sampling of the FCL for visualization and motor output:

- **Configurable Rate**: Global and per-area sampling frequencies
- **Non-blocking**: Uses queues to avoid blocking the main simulation
- **Multiple Consumers**: Supports multiple downstream visualization and processing services
- **Runtime Configuration**: Supports live updating of sampling rates for specific areas

```python
# Create a sampler with a global rate of 10Hz
sampler = FCLSampler(fcl_manager, 10, output_queue, connectome_manager)

# Start sampling
sampler.run()

# Update sample rate for a specific area
sampler.update_area_sample_rate(area_id, 20)  # Set to 20Hz
```

## State Transitions

The Burst Engine follows these state transitions:

1. **UNAVAILABLE** → **INITIALIZING**: During initial creation
2. **INITIALIZING** → **READY**: Once basic initialization is complete
3. **READY** → **RUNNING**: When simulation starts
4. **RUNNING** → **READY**: When simulation pauses
5. **ANY_STATE** → **UNAVAILABLE**: During shutdown

## GPU Integration

For high-performance applications, the NPU provides GPU acceleration through:

- **GPU FCL Adapter**: Transfers FCL data to/from GPU
- **Vectorized Operations**: Optimized matrix operations for neuron dynamics
- **Parallelized Processing**: Cortical areas can be processed in parallel

## Genome Integration

When a genome is loaded, the Burst Engine is updated via the `update_with_genome()` method:

```python
# After loading a genome:
burst_engine = core_api_service.get_burst_engine()
if burst_engine:
    burst_engine.update_with_genome()
```

This design allows the system to initialize completely before a genome is loaded.

## Performance Considerations

- **Target Frequency**: The burst engine attempts to maintain a target burst frequency in Hz
- **Actual Frequency**: The achieved frequency is monitored and reported
- **Load Shedding**: When actual frequency falls below target, areas marked as `__shed` have their FCL content dropped

## Related Documentation

- [NPU Architecture](arch-npu.md)
- [FCL Example](fcl_example.md)
- [Burst Engine Details](burst_engine.md)
- [System Architecture](../../docs/arch-system-overview.md) 