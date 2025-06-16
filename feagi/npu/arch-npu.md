# NPU Architecture

*Last Updated: May 24, 2025*

## Overview

The Neural Processing Unit (NPU) is the computational core of FEAGI, responsible for simulating neural dynamics. The architecture is designed for performance, determinism, and platform portability, with a focus on supporting real-time applications. A key innovation is the **differentiated FQ sampler** that provides optimized data streams for different consumer types.

## Architecture Principles

1. **Real-Time Compatibility**: Deterministic execution with bounded time complexity
2. **Platform Independence**: Designed to work across desktop, embedded, and RTOS environments
3. **Parallel Processing**: Support for multi-core CPU and GPU acceleration
4. **No Dynamic Allocation**: Pre-allocated data structures in main simulation loops
5. **Explicit State Management**: Clear state transitions with proper handling
6. **Differentiated Data Streams**: Optimized sampling behavior for different subscriber types

## Component Architecture

```
┌───────────────────────────────────────────────────────────────────────────────┐
│                           BurstEngine                                           │
│                                                                                 │
│  ┌───────────────┐  ┌───────────────┐  ┌───────────┐  ┌─────────────────────┐  │
│  │Simulation Loop│  │State Manager  │  │ Scheduler │  │ Subscriber Manager  │  │
│  └───────┬───────┘  └───────────────┘  └───────────┘  └─────────────────────┘  │
│          │                                                                     │
└──────────┼─────────────────────────────────────────────────────────────────────┘
           │
           ▼
┌──────────────────┐     ┌──────────────────────────────┐
│                  │     │                              │
│   FCL Manager    │◄────┤      Connectome Manager      │
│                  │     │                              │
└──────┬───────────┘     └──────────────────────────────┘
       │
       ▼
┌──────────────────┐
│   Fire Queue     │
│                  │
└──────┬───────────┘
       │
       ├────────────────┬─────────────────┬─────────────────┐
       │                │                 │                 │
       ▼                ▼                 ▼                 ▼
┌─────────────┐  ┌─────────────────┐  ┌────────────┐  ┌──────────────┐
│ FQ Sampler  │  │ GPU FCL Adapter │  │CPU/GPU     │  │ Neuron Firing│
│(Enhanced)   │  │                 │  │Operations  │  │ Operations   │
└─────┬───────┘  └─────────────────┘  └────────────┘  └──────────────┘
      │
      ├──────────────────┬─────────────────────────────────────┐
      │                  │                                     │
      ▼                  ▼                                     ▼
┌────────────────┐  ┌──────────────┐                ┌─────────────────┐
│ Visualization  │  │ Motor Stream │                │ Data Analytics  │
│ Stream         │  │              │                │ Stream          │
│                │  │              │                │                 │
│• All Areas     │  │• OPU Only    │                │• Configurable   │
│• Config Rate   │  │• Burst Freq  │                │• Per Area       │
│• Rich Data     │  │• Real-time   │                │• Historical     │
└────────────────┘  └──────────────┘                └─────────────────┘
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

#### 1.4 Subscriber Manager (New)

- **Stream Monitoring**: Tracks active subscribers for visualization and motor streams
- **Automatic FQ Sampler Control**: Enables/disables sampling based on subscriber presence
- **Differentiated Activation**: Separate control for visualization vs. motor subscribers

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

### 3. Enhanced FQ Sampler (New Architecture)

The Enhanced FQ Sampler provides **differentiated sampling behavior** based on subscriber types, optimizing data delivery for different use cases:

#### 3.1 Differentiated Sampling Behavior

**Visualization Subscribers:**
- **Scope**: Samples ALL cortical areas
- **Frequency**: Respects per-area `fq_sample_rate` properties (defaults to global rate)
- **Data Format**: Rich format with coordinates, membrane potentials, and neuron metadata
- **Use Case**: Real-time brain visualization, monitoring dashboards, research analysis

**Motor Subscribers:**
- **Scope**: Samples ONLY OPU (Output Processing Unit) cortical areas
- **Frequency**: Samples at burst frequency (every burst) for minimal latency
- **Data Format**: Optimized for real-time motor control with coordinate data
- **Use Case**: Robotic control, real-time motor output, actuator commands

#### 3.2 Smart Area Detection

**OPU Area Detection Logic:**
```python
def _get_opu_cortical_areas(self) -> List[str]:
    """Automatically detect OPU areas using multiple criteria."""
    opu_areas = []
    for area in self.connectome_manager.cortical_areas.values():
        area_type = area.properties.get('cortical_type', '').upper()

        # Multiple detection methods
        is_opu = (
            area_type == 'OPU' or           # Explicit type
            area_type == 'OUTPUT' or        # Output type
            area_type == 'MOTOR' or         # Motor type
            'OPU' in area_type or           # Contains OPU
            'OUTPUT' in area_type or        # Contains OUTPUT
            'MOTOR' in area_type or         # Contains MOTOR
            area.id.startswith('opu_') or   # Name prefix
            area.id.startswith('motor_') or # Motor prefix
            area.id.startswith('output_')   # Output prefix
        )

        if is_opu:
            opu_areas.append(area.id)

    return opu_areas
```

#### 3.3 Subscriber-Aware Activation

**Automatic Sampler Control:**
- **On-Demand Activation**: FQ sampler only runs when subscribers are present
- **Type-Specific Enabling**: Separate enable/disable for visualization vs. motor
- **Resource Conservation**: No unnecessary sampling when no consumers are connected
- **Connection Monitoring**: Tracks subscriber heartbeats and connection status

```python
# Subscriber detection triggers automatic sampler control
def _control_fq_sampler(self, enable: bool, target_type: str):
    if target_type == 'visualization':
        self.fq_sampler.set_visualization_subscribers(enable)
    elif target_type == 'motor':
        self.fq_sampler.set_motor_subscribers(enable)
```

#### 3.4 Data Flow Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           Fire Queue Provider                                 │
└─────────────────────────┬───────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                      Enhanced FQ Sampler                                     │
│                                                                             │
│  ┌─────────────────────────────┐   ┌─────────────────────────────────────┐  │
│  │   Visualization Path        │   │         Motor Path                  │  │
│  │                             │   │                                     │  │
│  │ • All Cortical Areas        │   │ • OPU Areas Only                   │  │
│  │ • Configured Sample Rates   │   │ • Burst Frequency                  │  │
│  │ • Rich Data Format          │   │ • Optimized for Real-time          │  │
│  │ • Tuple Format Output       │   │ • Tagged Data Format               │  │
│  └─────────────┬───────────────┘   └─────────────┬───────────────────────┘  │
└────────────────┼─────────────────────────────────┼─────────────────────────┘
                 │                                 │
                 ▼                                 ▼
┌────────────────────────────────┐    ┌────────────────────────────────┐
│      Visualization Stream      │    │        Motor Stream            │
│                                │    │                                │
│ • PUB Socket (Port 5562)       │    │ • PUB Socket (Port 5564)       │
│ • feagi_bytes Encoding         │    │ • feagi_bytes Encoding         │
│ • All Neural Activity          │    │ • Motor Commands Only          │
│ • Dashboard/Analysis            │    │ • Robotic Control              │
└────────────────────────────────┘    └────────────────────────────────┘
```

#### 3.5 Performance Optimizations

**Memory Management:**
- **Tagged Data Format**: Eliminates processing overhead for wrong stream types
- **Efficient Queuing**: Separate queues prevent motor data from blocking visualization
- **Selective Processing**: Only processes data relevant to each stream type

**Timing Optimizations:**
- **Burst-Synchronized Motor**: Motor sampling aligns with burst frequency for minimal latency
- **Configurable Visualization**: Per-area rates allow balancing detail vs. performance
- **Adaptive Sampling**: Automatically adjusts based on subscriber presence

### 4. GPU FCL Adapter

For high-performance applications, the GPU FCL Adapter provides:

- **Memory Transfer**: Efficient CPU-to-GPU FCL data movement
- **GPU Computation**: Utilizes GPU for parallel neuron updates
- **Device Management**: Handles device selection and computational resource allocation
- **Format Translation**: Converts between bitmap and tensor representations

## Stream Integration Architecture

### Visualization Stream Integration

**Stream Characteristics:**
- **Socket Type**: PUB (Publisher) on port 5562
- **Protocol**: feagi_bytes binary format
- **Subscriber Detection**: Heartbeat-based client tracking
- **Data Routing**: Processes only visualization-targeted data from FQ sampler

**FQ Sampler Configuration:**
```python
# Per-area sampling configuration
area.properties['fq_sample_rate'] = 30.0  # 30Hz for this area
area.properties['fq_sample_rate'] = 0.0   # Disable sampling for this area

# Global configuration
sampler = FQSampler(
    fire_queue_provider=fire_queue_provider,
    sample_frequency_hz=20.0,  # Default 20Hz for areas without specific rates
    output_queue=viz_queue,
    connectome_manager=connectome_manager
)
```

### Motor Stream Integration

**Stream Characteristics:**
- **Socket Type**: PUB (Publisher) on port 5564
- **Protocol**: feagi_bytes binary format optimized for motor control
- **Subscriber Detection**: Heartbeat-based client tracking with faster timeouts
- **Data Routing**: Processes only motor-targeted data from FQ sampler

**Real-Time Motor Control:**
```python
# Motor sampling configuration
# - Automatically samples at burst frequency (typically 100Hz)
# - Only samples OPU areas
# - Provides minimal latency for real-time control

# Example OPU area configuration
motor_area = {
    "id": "motor_output",
    "properties": {
        "cortical_type": "OPU",
        # No fq_sample_rate needed - uses burst frequency
    }
}
```

## Performance Optimization

### 1. Memory Management

- **Pre-allocated Arrays**: Fixed-size arrays for neuron and synapse data
- **Bitmap Compression**: Efficient storage of sparse firing patterns using Roaring Bitmaps
- **Static Allocation**: No dynamic memory allocation during the main simulation loop
- **Shared Memory**: Where appropriate, uses shared memory for inter-process communication
- **Tagged Data Routing**: Efficient routing prevents unnecessary data processing

### 2. Computational Efficiency

- **Vectorized Operations**: SIMD instructions for bulk neuron updates
- **Parallelization**: Multi-core processing for independent cortical areas
- **GPU Acceleration**: Optional GPU processing for large-scale simulations
- **Load Shedding**: Intelligent dropping of processing load when system can't keep up
- **Selective Sampling**: Only samples when subscribers are present

### 3. Real-Time Performance

- **Deterministic Timing**: Predictable execution patterns with bounded time complexity
- **Priority Management**: Process and thread priorities are set for timely execution
- **Frequency Monitoring**: Continuous monitoring of actual burst frequency
- **Adaptive Processing**: Adjusts computational load based on available resources
- **Stream-Specific Optimization**: Motor streams prioritize latency, visualization streams prioritize completeness

### 4. Differentiated Stream Performance

| Stream Type | Latency Target | Data Volume | Update Frequency | Use Case |
|-------------|---------------|-------------|------------------|----------|
| Motor | less than 10ms | Low (OPU only) | Burst Rate (~100Hz) | Real-time control |
| Visualization | ~50ms | High (All areas) | Configurable (1-60Hz) | Monitoring/Analysis |
| Analytics | ~100ms | Variable | On-demand | Research/Logging |

## Implementation Considerations

### 1. CPU Implementation

The primary CPU implementation uses:
- **Python NumPy**: For vectorized operations
- **PyRoaring**: For efficient bitmap operations
- **Threading**: For parallel processing
- **Time Management**: Precision timing for burst control
- **Tagged Data Queues**: For efficient stream routing

### 2. GPU Implementation

The optional GPU implementation uses:
- **CUDA/RAPIDS**: For GPU-accelerated processing
- **Memory Mapping**: For efficient CPU-GPU transfers
- **Stream Processing**: For asynchronous GPU operations
- **Multi-Stream Support**: Separate GPU streams for different data types

### 3. Embedded/RTOS Implementation

For embedded systems and RTOS environments:
- **Fixed Memory Allocation**: All memory allocated at initialization
- **Bounded Processing**: Guaranteed execution time bounds
- **Priority Inversion Protection**: Proper handling of priority inversions
- **Interrupt Management**: Safe operation within interrupt contexts
- **Deterministic Stream Processing**: Predictable timing for all data streams

## Configuration Examples

### Basic Differentiated Sampling

```python
# Configure FQ sampler with differentiated behavior
config = {
    'sample_frequency_hz': 20.0,  # Default for visualization
    'enable_differentiated_sampling': True,
    'motor_sample_at_burst_frequency': True,
    'auto_detect_opu_areas': True
}

# Per-area configuration
areas = {
    'visual_cortex': {
        'properties': {
            'fq_sample_rate': 30.0,  # High rate for visual analysis
            'cortical_type': 'sensory'
        }
    },
    'motor_cortex': {
        'properties': {
            'cortical_type': 'OPU',  # Automatically detected for motor sampling
            # Uses burst frequency automatically
        }
    },
    'memory_area': {
        'properties': {
            'fq_sample_rate': 5.0,   # Low rate for memory areas
            'cortical_type': 'associative'
        }
    }
}
```

### Stream-Specific Optimization

```python
# Visualization stream configuration
visualization_config = {
    'auto_enable_on_subscribers': True,
    'subscriber_check_interval': 1.0,
    'include_membrane_potentials': True,
    'include_coordinates': True,
    'include_firing_history': True
}

# Motor stream configuration
motor_config = {
    'auto_enable_on_subscribers': True,
    'subscriber_check_interval': 0.5,  # Faster detection
    'motor_timeout_seconds': 10.0,     # Faster timeout
    'optimize_for_latency': True,
    'include_only_coordinates': True   # Minimal data for speed
}
```

## Related Documentation

- [NPU Module](README.md)
- [FCL Example](fcl_example.md)
- [Burst Engine Details](burst_engine.md)
- [ZMQ Streams Architecture](../../docs/arch-zmq.md)
- [System Architecture](../../docs/arch-system-overview.md)
