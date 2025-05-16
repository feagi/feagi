# NPU Architecture

*Last Updated: May 15, 2025*

## Overview

The Neural Processing Unit (NPU) is the computational core of FEAGI, responsible for simulating neural dynamics. The architecture is designed for performance, determinism, and platform portability, with a focus on supporting real-time applications.

## Architecture Principles

1. **Real-Time Compatibility**: Deterministic execution with bounded time complexity
2. **Platform Independence**: Designed to work across desktop, embedded, and RTOS environments
3. **Parallel Processing**: Support for multi-core CPU and GPU acceleration
4. **No Dynamic Allocation**: Pre-allocated data structures in main simulation loops
5. **Explicit State Management**: Clear state transitions with proper handling

## Component Architecture

```
┌───────────────────────────────────────────────────────┐
│                     BurstEngine                       │
│                                                       │
│  ┌───────────────┐  ┌───────────────┐  ┌───────────┐  │
│  │Simulation Loop│  │State Manager  │  │ Scheduler │  │
│  └───────┬───────┘  └───────────────┘  └───────────┘  │
│          │                                            │
└──────────┼────────────────────────────────────────────┘
           │
           ▼
┌──────────────────┐     ┌──────────────────────────────┐
│                  │     │                              │
│   FCL Manager    │◄────┤      Connectome Manager      │
│                  │     │                              │
└──────┬───────────┘     └──────────────────────────────┘
       │
       ├────────────────┐
       │                │
       ▼                ▼
┌─────────────┐  ┌─────────────────┐  ┌────────────────┐
│ FCL Sampler │  │ GPU FCL Adapter │  │ Visualization  │
│             │  │                 │  │ Interface      │
└─────────────┘  └─────────────────┘  └────────────────┘
```

## Key Components

### 1. Burst Engine

The Burst Engine controls the main simulation loop and coordinates all neuron processing. It implements a deterministic burst cycle with the following key features:

#### 1.1 Simulation Loop

- **Fixed-frequency execution**: Runs at a configurable burst frequency (Hz)
- **Time measurement**: Tracks actual vs. desired burst frequency
- **Load shedding**: When the system cannot keep up with the target frequency, sheds load by clearing Fire Candidate Lists for designated cortical areas
- **Sleep management**: Sleeps for precise intervals to maintain consistent timing

```python
def run(self):
    while self._running:
        start = time.perf_counter()
        # 1. Process neuron firing (update membrane potentials and FCL)
        fired_neurons = self.connectome_manager.update_membrane_potentials()
        # 2. Measure actual frequency
        end = time.perf_counter()
        elapsed = end - start
        actual_freq = 1.0 / elapsed if elapsed > 0 else 0
        # 3. Load shedding if needed
        if actual_freq < self.desired_frequency:
            for area_id in self.shed_areas:
                self.fcl_manager.area_fcl_history[area_id][self.fcl_manager.current_window_index].clear()
        # 4. Sleep for the remainder of the interval
        if elapsed < self.burst_interval:
            time.sleep(self.burst_interval - elapsed)
```

#### 1.2 State Manager

- **Service state tracking**: Maintains the current operational state of the burst engine
- **State transitions**: Handles transitions between INITIALIZING, READY, RUNNING, and UNAVAILABLE states
- **Signaling**: Communicates state changes to other components

#### 1.3 Scheduler

- **Thread management**: Controls execution thread for the simulation loop
- **Signal handling**: Manages OS signals for graceful shutdown
- **Priority setting**: Sets process/thread priority for real-time operation

### 2. FCL Manager

The FCL Manager maintains the Fire Candidate List, which tracks neurons that have fired across timesteps.

#### 2.1 Fire Candidate List Data Structure

- **Global FCL History**: Circular array of Roaring Bitmaps tracking all firing neurons
- **Area-Based FCL History**: Dictionary mapping area IDs to their own circular arrays of bitmaps
- **Indexing**: Efficient integer-based neuron indexing for fast lookups
- **Temporal management**: Sliding window approach for maintaining history

#### 2.2 Membrane Update Processing

- **Update Queue**: Fast queue for processing membrane potential changes
- **Threshold Check**: Efficient vectorized checking of neurons that cross firing thresholds
- **Synapse Processing**: Updates post-synaptic neurons based on firing events
- **Event Generation**: Creates FCL update events for newly firing neurons

#### 2.3 Optimization Techniques

- **Roaring Bitmaps**: Efficiently represents sparse sets of firing neurons
- **Circular Buffers**: Avoids memory allocation during timestep advancement
- **Vectorized Operations**: Where possible, uses NumPy/SIMD for parallel computation
- **Sparse Representation**: Only stores and processes active neurons

### 3. FCL Sampler

The FCL Sampler provides configurable sampling of the FCL for visualization and analysis:

- **Global Sampling Rate**: Default sampling frequency for all areas
- **Per-Area Rates**: Area-specific sampling rates via the `fcl_sample_rate` property
- **Non-blocking Output**: Queue-based output for downstream consumers
- **Runtime Configuration**: Supports dynamic updating of sampling parameters

### 4. GPU FCL Adapter

For high-performance applications, the GPU FCL Adapter provides:

- **Memory Transfer**: Efficient CPU-to-GPU FCL data movement
- **GPU Computation**: Utilizes GPU for parallel neuron updates
- **Device Management**: Handles device selection and computational resource allocation
- **Format Translation**: Converts between bitmap and tensor representations

## Performance Optimization

### 1. Memory Management

- **Pre-allocated Arrays**: Fixed-size arrays for neuron and synapse data
- **Bitmap Compression**: Efficient storage of sparse firing patterns using Roaring Bitmaps
- **Static Allocation**: No dynamic memory allocation during the main simulation loop
- **Shared Memory**: Where appropriate, uses shared memory for inter-process communication

### 2. Computational Efficiency

- **Vectorized Operations**: SIMD instructions for bulk neuron updates
- **Parallelization**: Multi-core processing for independent cortical areas
- **GPU Acceleration**: Optional GPU processing for large-scale simulations
- **Load Shedding**: Intelligent dropping of processing load when system can't keep up

### 3. Real-Time Performance

- **Deterministic Timing**: Predictable execution patterns with bounded time complexity
- **Priority Management**: Process and thread priorities are set for timely execution
- **Frequency Monitoring**: Continuous monitoring of actual burst frequency
- **Adaptive Processing**: Adjusts computational load based on available resources

## Implementation Considerations

### 1. CPU Implementation

The primary CPU implementation uses:
- **Python NumPy**: For vectorized operations
- **PyRoaring**: For efficient bitmap operations
- **Threading**: For parallel processing
- **Time Management**: Precision timing for burst control

### 2. GPU Implementation

The optional GPU implementation uses:
- **CUDA/RAPIDS**: For GPU-accelerated processing
- **Memory Mapping**: For efficient CPU-GPU transfers
- **Stream Processing**: For asynchronous GPU operations

### 3. Embedded/RTOS Implementation

For embedded systems and RTOS environments:
- **Fixed Memory Allocation**: All memory allocated at initialization
- **Bounded Processing**: Guaranteed execution time bounds
- **Priority Inversion Protection**: Proper handling of priority inversions
- **Interrupt Management**: Safe operation within interrupt contexts

## Related Documentation

- [NPU Module](README.md)
- [FCL Example](fcl_example.md)
- [Burst Engine Details](burst_engine.md)
- [System Architecture](../../docs/arch-system-overview.md) 