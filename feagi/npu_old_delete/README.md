# FEAGI Neural Processing Unit (NPU)

*Last Updated: May 15, 2025*

## Overview

The Neural Processing Unit (NPU) is responsible for simulating neuron dynamics, managing burst execution, and handling all neural activity within FEAGI. As the computational core of the brain simulation, the NPU provides high-performance, deterministic neural processing suitable for both research and real-time applications.

## Key Components

### Burst Engine

The Burst Engine manages the neuron firing dynamics and coordinates simulation timing using a **modular mixin architecture**. Key features:

- **Modular Design**: Uses specialized mixins for debug and performance functionality
- **Standby Mode**: Initializes without requiring a genome, breaking circular dependencies
- **State Management**: Uses explicit state transitions with comprehensive logging
- **Dependency Injection**: No global state, all dependencies are passed explicitly
- **RTOS-Friendly**: Deterministic timing and no dynamic allocations in the main loop
- **Load Shedding**: Areas marked with `__shed` have their FCL content dropped when the system can't maintain target frequency
- **Special Area Support**: Handles special cortical areas like power areas with automatic neuron injection
- **SIMD Acceleration**: Automatic vectorization support for high-performance neural processing
- **Debug Integration**: Comprehensive debugging with FQ sampler monitoring and detailed burst analysis

#### Modular Architecture

The burst engine uses a mixin-based architecture for clean separation of concerns:

```python
class BurstEngine(BurstEngineDebugMixin, BurstEnginePerformanceMixin):
    """
    Main burst engine with modular functionality:
    - Core neural simulation logic
    - Debug mixin: NPU debugging, FQ sampler monitoring, state validation
    - Performance mixin: SIMD acceleration, frequency measurement, profiling
    """
```

#### Core Usage

```python
# Create and initialize with clean generic architecture
engine = BurstEngine(connectome_manager, fcl_manager, config={
    'debug_npu': True,               # Enable debug mixin features
    'enable_injection': True,        # Enable FCL injection service for special areas
    'desired_frequency_hz': 10.0,    # Target simulation frequency
    'simd_profiling': True,          # Enable performance profiling
    'performance_monitoring': True   # Enable performance mixin features
})

# Start the engine with automatic mixin initialization
engine.run()

# When genome is loaded, update all components
engine.update_with_genome()

# Graceful shutdown with proper cleanup
engine.stop()
```

#### Debug Features (BurstEngineDebugMixin)

The debug mixin provides comprehensive debugging capabilities:

```python
# Register FQ samplers for monitoring
engine.register_fq_sampler(motor_sampler)
engine.register_fq_sampler(visualization_sampler)

# Enable detailed burst debugging with --debug-npu flag
# Shows per-area breakdowns, sampler status, and performance metrics

# Get debug statistics
debug_stats = engine.get_debug_statistics()
print(f"FQ Samplers: {debug_stats['fq_samplers_registered']}")
print(f"Debug Mode: {debug_stats['debug_mode_enabled']}")

# Validate internal state consistency
validation = engine.debug_validate_state()
print(f"All validations passed: {all(validation.values())}")
```

#### Performance Features (BurstEnginePerformanceMixin)

The performance mixin provides SIMD acceleration and monitoring:

```python
# Automatic SIMD detection and configuration
# Supports AVX512, AVX2, AVX, SSE2, NEON, and scalar fallback

# Real-time frequency measurement
engine.enable_frequency_measurement(True)
results = engine.measure_actual_frequency(duration_seconds=5.0)
print(f"Actual: {results['actual_frequency_hz']:.1f}Hz")
print(f"Potential: {results['potential_frequency_hz']:.1f}Hz")
print(f"Efficiency: {results['efficiency_ratio']:.2f}")

# Get comprehensive performance metrics
metrics = engine.get_performance_metrics()
print(f"SIMD Backend: {metrics['simd_backend']}")
print(f"Vector Width: {metrics['simd_vector_width']}")
print(f"Total Neurons Processed: {metrics['total_neurons_processed']}")
```

### Special Area Handler

The Special Area Handler detects and manages cortical areas with unique behaviors using a clean, extensible architecture:

- **Power Areas**: Areas ending with "_pwr" or named "___pwr" that add all their neurons to the FCL on every burst for foundational activity
- **Modulator Areas**: Areas ending with "_mod" that affect other areas' firing patterns through during-burst injection
- **Enhanced Memory Areas**: Areas with extended temporal windows and specialized processing
- **Sensory Input Areas**: Areas that receive external sensor data injection
- **Custom Special Areas**: Any area type can be defined with custom injection timing and behavior

**Special Area Configuration:**
```python
# Create a power area in your genome
cortical_area = {
    "id": "___pwr",  # Special naming pattern
    "name": "Power Injection Area",
    "dimensions": [10, 10, 1],
    "neurons_per_voxel": 1,
    "properties": {
        "__power_injection": True,  # Alternative property-based detection
        "injection_timing": "pre_burst",  # When to add candidates: pre_burst, during_burst, post_burst
        "injection_probability": 1.0  # Always inject (1.0) or probabilistic (0.0-1.0)
    }
}

# Create a modulator area
modulator_area = {
    "id": "dopamine_mod",
    "properties": {
        "__modulator": True,
        "injection_timing": "during_burst",  # Affect ongoing computation
        "injection_probability": 0.8
    }
}
```

### FCL Injection Service

The FCL Injection Service coordinates the addition of neuron candidates from special areas to the Fire Candidate List using the unified FCL candidate model:

- **Unified Processing**: All candidates (internal synaptic + external special areas) processed together in one sweep
- **Extensible Architecture**: Supports any special area type (power, modulator, sensory, custom)
- **Batch Processing**: Efficiently processes large numbers of neurons in batches
- **Timing Control**: Supports pre-burst, during-burst, and post-burst injection phases
- **Probabilistic Injection**: Supports probabilistic neuron injection for stochastic effects
- **Performance Monitoring**: Tracks injection statistics and performance metrics
- **Area-Agnostic**: Service handles all area-specific logic internally

```python
# Configure injection service (handles all special area types)
config = {
    'batch_injection_size': 1000,
    'enable_probabilistic_injection': True,
    'enable_timing_optimization': True
}

# Service is automatically initialized by BurstEngine when special areas are detected
stats = engine.get_injection_statistics()  # Generic statistics for all special areas
engine.set_injection_enabled("___pwr", True)  # Enable/disable any special area
engine.set_injection_enabled("dopamine_mod", False)  # Works for any area type
```

### Buffered Injection (Burst-Paced Drain)

External activations are buffered and drained at burst boundaries for determinism and performance:

- Bounded global and per-area capacities
- Per-burst drain budgets and round-robin fairness
- Duplicate coalescing for deterministic updates
- Single batched submission to FCL per burst

Configure in `feagi_configuration.toml`:

```toml
[injection.buffer]
capacity_total = 65536
capacity_per_area = 8192
coalesce_duplicates = true

[injection.drain]
per_burst_max_total = 8192
per_burst_max_per_area = 2048
fairness = "round_robin"
drop_policy = "newest"

[injection.metrics]
enabled = true
window_seconds = 5.0
```

Inspect runtime status via:

- `GET /v1/burst_engine/injection/status`

### FCL Manager

The FCL (Fire Candidate List) Manager maintains the temporal history of neuron firing patterns:

- **Roaring Bitmaps**: Efficient sparse data structure for representing neuron activations
- **Historical Tracking**: Maintains multiple timesteps of neuron firing history
- **Area-Based FCLs**: Maintains separate FCLs for each cortical area
- **Membrane Potential**: Handles membrane potential updates from firing neurons
- **Special Area Integration**: Supports injection from power and modulator areas

```python
# Get all currently firing neurons
active_neurons = fcl_manager.get_global_fcl()

# Get neurons that fired in the last 3 timesteps
recent_activity = fcl_manager.get_neurons_fired_in_last_n_steps(3)

# Get neurons firing in a specific cortical area
area_activity = fcl_manager.get_area_fcl(area_id)

# Add neurons to current FCL (used by injection service)
fcl_manager.add_to_current_fcl([neuron_id1, neuron_id2, neuron_id3])
```

### FQ Sampler

The FQ Sampler extracts burst data using a **cortical area-based sampling architecture** with **differentiated behavior** based on subscriber types for optimal performance:

#### Cortical Area-Based Architecture

**Always Organized by Cortical Areas:**
- **Data Format**: All sampling returns data organized by cortical areas as keys
- **Consistent Structure**: `{area_id: [neuron_ids], area_id2: [neuron_ids], ...}`
- **Strategy Pattern**: Extensible architecture for different sampling modes
- **Zero-Copy Operations**: Maintains performance characteristics for real-time processing

#### Differentiated Sampling Modes

**Visualization Mode:**
- **Scope**: Samples ALL cortical areas for comprehensive brain state monitoring
- **Frequency**: Respects per-area `fq_sample_rate` properties (defaults to global rate)
- **Data Format**: Rich format with coordinates, membrane potentials, thresholds, and firing history
- **Use Case**: Real-time brain visualization, research analysis, monitoring dashboards

**OPU (Motor) Mode:**
- **Scope**: Samples ONLY OPU (Output Processing Unit) cortical areas
- **Frequency**: Samples at burst frequency (every burst) for minimal control latency
- **Data Format**: Streamlined format optimized for real-time motor control
- **Use Case**: Robotic control, real-time motor output, actuator commands

**Custom Mode:**
- **Scope**: Samples specific cortical area IDs provided by user
- **Frequency**: Configurable per sampling request
- **Data Format**: Flexible format based on requirements
- **Use Case**: Targeted monitoring, specific area analysis, custom applications

#### Enhanced Configuration

```python
# Create unified sampler with cortical area-based architecture
sampler = UnifiedFQSampler(
    fire_queue_provider=fire_queue_provider,
    connectome_manager=connectome_manager,
    config={
        'sampling_strategy': 'visualization',  # or 'opu' or 'custom'
        'default_sample_rate': 30.0,          # Default sampling frequency
        'enable_zero_copy': True,             # Performance optimization
        'custom_area_ids': ['motor_ctx', 'visual_ctx']  # For custom mode
    }
)

# Strategy-based sampling
viz_data = sampler.sample_visualization_areas()    # All areas, rich format
motor_data = sampler.sample_opu_areas()           # Motor areas only, fast format
custom_data = sampler.sample_custom_areas(['area1', 'area2'])  # Specific areas

# All methods return: {area_id: [neuron_ids], ...}
```

#### Stream Integration

**Visualization Stream (Port 5562):**
- Processes cortical area-organized data from FQ sampler
- Data format: `{area_id: [neuron_ids], area_id2: [neuron_ids], ...}`
- Rich neural activity information for comprehensive brain monitoring
- Automatic heartbeat-based subscriber detection

**Motor Stream (Port 5564):**
- Processes OPU area-organized data from FQ sampler
- Data format: `{motor_area_id: [neuron_ids], ...}` (only OPU areas)
- Optimized format for real-time motor control with minimal latency
- Fast subscriber detection with shorter timeouts

```python
# Visualization client example
viz_socket = zmq_context.socket(zmq.SUB)
viz_socket.connect("tcp://localhost:5562")
viz_socket.setsockopt(zmq.SUBSCRIBE, b"activity")

# Receive cortical area-organized data
data = viz_socket.recv_json()
# data = {
#     "iv00_C": [1, 4, 17, 26, ...],
#     "___pwr": [101, 102, 103, ...],
#     "motor_ctx": [201, 205, 210, ...]
# }

# Motor client example
motor_socket = zmq_context.socket(zmq.SUB)
motor_socket.connect("tcp://localhost:5564")
motor_socket.setsockopt(zmq.SUBSCRIBE, b"motor")

# Receive OPU area-organized data only
motor_data = motor_socket.recv_json()
# motor_data = {
#     "motor_ctx": [201, 205, 210, ...],
#     "actuator_ctrl": [301, 305, ...]
# }

# Both clients should send heartbeats for subscriber detection
heartbeat = {
    "message_type": "heartbeat",
    "agent_id": "client_001",
    "timestamp": time.time() * 1000
}
```

## Core Power Area Implementation

### High-Performance Power Area Injection

FEAGI 2.0 implements ultra-fast power area injection optimized for 100kHz burst frequencies:

1. **Core Power Area**:
   - **cortical_idx=1**: Reserved for the core power area (___pwr)
   - **Guaranteed to exist**: Created during neuroembryogenesis in every genome
   - **Direct access**: No detection or lookup overhead - uses cortical_idx=1 directly
   - **Always inject**: All neurons from the power area are injected into FCL every burst

### Injection Timing

The core power area injects during the **pre_burst** phase:
- Adds candidates before regular neuron firing
- Ensures power neurons are available for synaptic propagation
- Provides foundational neural activity for the connectome

### Configuration Options

Core power area injection can be configured at the burst engine level:

```python
# Burst engine configuration
config = {
    'enable_injection': True,  # Global enable/disable for power area injection
    'desired_frequency_hz': 100000.0,  # Target burst frequency (100kHz supported)
}

engine = BurstEngine(connectome_manager, fcl_manager, config)

# Core power area is automatically configured:
# - cortical_idx=1 (___pwr)
# - Always inject all neurons
# - Pre-burst timing
# - Direct access for maximum performance
```

## Core Area Reservations

During neuroembryogenesis, FEAGI reserves specific cortical_idx values for core areas:

- **cortical_idx=0**: Reserved for `"_death"` area
- **cortical_idx=1**: Reserved for `"___pwr"` area (power area)
- **cortical_idx≥2**: Available for regular cortical areas

This reservation system ensures that critical core areas always have predictable cortical_idx values for:
- Performance optimization (direct access via cortical_idx)
- System reliability (guaranteed existence)
- Inter-component compatibility

### Power Area Injection

The FCL injection service directly accesses the power area using `cortical_idx=1`:

```python
# Direct access to power area (___pwr) at cortical_idx=1
power_neurons = connectome_manager.get_neurons_by_cortical_idx(1)
```

This approach provides:
- **Ultra-fast access**: No string lookups or detection logic
- **100kHz compatibility**: Optimized for high-frequency burst operations
- **Guaranteed availability**: Core areas always exist in every genome

## Neural Processing Architecture

FEAGI 2.0 implements a clean separation between two fundamental neural processing streams:

### 1. Synaptic Propagation (Core Neural Computation)
**Handled by**: `ConnectomeManager.update_membrane_potentials()`
**Purpose**: Standard neural computation with synaptic transmission
**Characteristics**:
- High-performance GPU/SIMD optimized
- Millions of operations per burst
- Membrane potential updates → threshold detection → FCL addition
- This is the **core neural algorithm**

**Process Flow**:
```
Active neurons → Synaptic weights → Membrane potential updates →
Threshold detection → New fired neurons → Added to FCL
```

### 2. Special Area Injection (External Behaviors)
**Handled by**: `FCLInjectionService`
**Purpose**: External/special behaviors that bypass normal neural computation
**Characteristics**:
- Relatively few operations per burst
- Direct FCL candidate addition (bypasses membrane potentials)
- Extensible to multiple special area types
- **Area-agnostic** implementation

**Supported Special Area Types**:
- **Power Areas** (`___pwr`): Inject all neurons every burst for foundational activity
- **Sensory Input**: External sensor data injection
- **Modulator Areas**: Behavioral modification (dopamine, attention, etc.)
- **Memory Areas**: Enhanced temporal processing
- **Future Extensions**: Any special area type

### 3. Unified FCL Candidate Model

Both streams add candidates to the **Fire Candidate List (FCL)**:

```python
def _process_burst(self) -> List[int]:
    # 1. Single-phase external candidates → FCL (atomic accumulation before processing)
    if self.injection_service:
        self.injection_service.inject_candidates(self.burst_count)

    # 2. Process ALL accumulated candidates (internal + external) in one atomic operation
    fired_neurons = self.connectome_manager.update_membrane_potentials()

    return fired_neurons  # All neurons that actually fired
```

### 4. Clean Component Responsibilities

**Burst Engine**:
- **Completely area-agnostic** - no knowledge of specific area types
- Orchestrates the sequence: external injection → synaptic processing → additional injection
- Always calls all injection phases, lets service decide what to do internally
- Returns unified list of fired neurons from all sources

**FCL Injection Service**:
- **Autonomous and extensible** - handles all special area logic internally
- Manages timing decisions (which areas inject when)
- Supports probabilistic injection, batch processing, performance optimization
- Can be extended for new special area types without touching burst engine

**Connectome Manager**:
- **Pure neural computation** - processes FCL candidates through synaptic propagation
- GPU/SIMD optimized for performance
- Determines actual firing based on membrane potentials and thresholds
- Handles the unified processing of all candidates (internal + external)

### 5. Key Architectural Benefits

✅ **Single Source of Truth**: FCL is the unified pool for all firing candidates
✅ **Clean Separation**: External injection vs. synaptic propagation clearly separated
✅ **Performance**: Core synaptic computation remains GPU-optimized
✅ **Extensibility**: New special area types can be added without touching core components
✅ **Area-Agnostic**: Burst engine has no knowledge of specific area types
✅ **Unified Processing**: All candidates processed together in single efficient sweep

### 6. Configuration

**Burst Engine Configuration**:
```python
config = {
    'desired_frequency_hz': 100.0,           # Target burst frequency
    'enable_injection': True,                # Enable special area processing (all types)
    'special_area_config': {
        'batch_injection_threshold': 100      # Threshold for batch processing
    },
    'injection_config': {
        'batch_injection_size': 1000,         # Neurons per injection batch
        'enable_probabilistic_injection': True,# Support probabilistic injection
        'enable_timing_optimization': True     # Enable timing optimizations
    }
}
```

**Injection Service Configuration** (handles area-specific logic internally):
```python
injection_config = {
    'batch_injection_size': 1000,
    'enable_probabilistic_injection': True,
    'enable_timing_optimization': True
}

special_area_config = {
    'batch_injection_threshold': 100
}
```

**Per-Area Configuration** (through area properties):
```python
power_area = {
    "id": "reward_pwr",
    "properties": {
        "__power_injection": True,
        "injection_timing": "pre_burst",      # When to add candidates
        "injection_probability": 1.0          # Probability of injection (0.0-1.0)
    }
}
```

This architecture ensures that the burst engine remains a pure neural processing engine while the injection service handles all special area behaviors autonomously.

## Architecture Guidelines

### Thread Safety

All NPU components are designed to be thread-safe when used correctly:

- FCL operations use atomic bitmap operations
- State transitions are managed through the FeagiStateManager
- Injection services maintain internal consistency during batch operations

### Memory Management

The NPU follows strict memory management principles:

- **No Dynamic Allocation**: Main loops avoid dynamic memory allocation
- **Pre-allocated Buffers**: All working memory allocated during initialization
- **Bounded Structures**: FCL history has configurable but bounded size
- **Cache Optimization**: Frequently accessed data is cached for performance

### RTOS Compatibility

Design patterns support future RTOS/embedded deployment:

- **Deterministic Timing**: All operations have bounded execution time
- **Static Configuration**: Runtime behavior determined by initialization-time configuration
- **Minimal Dependencies**: Core algorithms avoid complex library dependencies
- **Error Handling**: Graceful degradation on resource constraints

## Configuration Reference

### Burst Engine Configuration

```python
{
    'desired_frequency_hz': 100.0,           # Target burst frequency
    'enable_injection': True,                # Enable special area processing (all types)
    'special_area_config': {
        'batch_injection_threshold': 100      # Threshold for batch processing
    },
    'injection_config': {
        'batch_injection_size': 1000,         # Neurons per injection batch
        'enable_probabilistic_injection': True,# Support probabilistic injection
        'enable_timing_optimization': True     # Enable timing optimizations
    }
}
```

### Special Area Properties

```python
{
    '__power_injection': True,               # Mark as power area
    '__modulator': True,                     # Mark as modulator area
    '__shed': False,                         # Enable for load shedding
    'injection_timing': 'pre_burst',         # Injection phase (pre_burst, during_burst, post_burst)
    'injection_probability': 1.0,            # Injection probability (0.0-1.0)
    'fq_sample_rate': 30.0                  # Sampling rate for this area
}
```

## Testing and Validation

### Unit Tests

Power area functionality is covered by comprehensive unit tests:

```bash
# Run NPU tests
cd feagi_core
python -m pytest tests/unit/npu/ -v

# Run specific power area tests
python -m pytest tests/unit/npu/test_special_area_handler.py -v
python -m pytest tests/unit/npu/test_fcl_injection_service.py -v
```

### Integration Tests

End-to-end testing validates the complete power injection pipeline:

```bash
# Run integration tests
python -m pytest tests/integration/npu/ -v

# Test with different genome configurations
python -m pytest tests/integration/npu/test_power_area_integration.py -v
```

### Performance Benchmarks

Benchmark the impact of power injection on burst frequency:

```bash
# Run performance tests
python -m pytest tests/performance/npu/ -v

# Generate performance reports
python -m pytest tests/performance/npu/test_injection_performance.py --benchmark-only
```

## Migration Guide

### From Legacy FEAGI

When migrating from legacy FEAGI systems:

1. **Identify Special Areas**: Look for areas that manually inject neurons into the FCL
2. **Update Naming**: Rename areas to use appropriate suffixes ("_pwr", "_mod") or add special area properties
3. **Configure Timing**: Set appropriate injection timing based on previous behavior
4. **Generic Configuration**: Update burst engine config to use `enable_injection` instead of area-specific flags
5. **Test Performance**: Validate that injection doesn't impact target burst frequency

### Configuration Updates

Update your genome configuration to use the new special area features:

```python
# Before (manual injection)
def manual_power_injection():
    for neuron_id in power_neurons:
        fcl_manager.add_neuron(neuron_id)

# After (automatic injection via unified architecture)
power_area = {
    'id': 'attention_pwr',
    'properties': {'__power_injection': True}
}

modulator_area = {
    'id': 'dopamine_mod',
    'properties': {
        '__modulator': True,
        'injection_timing': 'during_burst'
    }
}

# Burst engine config update
config = {
    'enable_injection': True,  # Generic injection for all special areas
    'injection_config': {
        'batch_injection_size': 1000,
        'enable_probabilistic_injection': True
    }
}
```

## Troubleshooting

### Common Issues

1. **Special Areas Not Detected**: Check naming patterns and property flags for all area types
2. **Performance Impact**: Reduce batch size or adjust probabilistic injection settings
3. **Timing Issues**: Adjust injection timing phase or enable optimizations

### Debug Logging

Enable detailed logging for injection debugging:

```python
import logging
logging.getLogger('feagi.npu.special_area_handler').setLevel(logging.DEBUG)
logging.getLogger('feagi.npu.fcl_injection_service').setLevel(logging.DEBUG)
```

### Statistics and Monitoring

Monitor injection performance for all special area types:

```python
# Get detailed statistics (all special areas)
stats = engine.get_injection_statistics()
print(f"Total injections: {stats['total_injections']}")
print(f"Special areas: {stats['prepared_batches']}")

# Get injection preview
preview = engine.injection_service.get_power_injection_preview() if engine.injection_service else {}
print(f"Neurons to inject: {preview.get('pre_burst_neurons', 0)}")

# Enable/disable specific areas
engine.set_injection_enabled("___pwr", True)      # Enable power area
engine.set_injection_enabled("dopamine_mod", False)  # Disable modulator area
```

### NPU Debug Mode

Launch FEAGI with `--debug-npu` to see detailed fire queue contents every burst:
```bash
python3 -m feagi.main --debug-npu
```
This displays:
- Total firing neurons per burst
- Per-cortical area breakdown with neuron IDs
- Special area injection statistics (all types)
- Recent firing activity trends

### Performance Considerations

Special area injection is optimized for high-frequency operation:

- **Pre-cached Neuron Lists**: Special area neurons are cached on genome load
- **Batch Processing**: Large neuron lists are processed in configurable batches
- **Minimal Runtime Allocation**: All data structures pre-allocated during initialization
- **Statistics Tracking**: Comprehensive performance monitoring

### Use Cases

Special areas enable several advanced neural simulation patterns:

1. **Global Attention Mechanisms**: Power areas can provide persistent background activation
2. **Learning Signals**: Modulator areas inject reward/punishment signals across the brain
3. **Synchronization**: Provide timing signals for coordinated activity
4. **Debugging**: Force specific neurons to fire for testing and validation

## Developer Resources

### Module Dependencies

For information about NPU module dependencies and safe import patterns, see:
- **[DEPENDENCIES.md](DEPENDENCIES.md)** - Internal dependency map and import guidelines
- **[example-fcl.md](example-fcl.md)** - Comprehensive FCL usage examples and data structure explanations

### Architecture Documentation

- **[arch-npu.md](arch-npu.md)** - High-level NPU architecture overview
- **[arch-burst-engine.md](arch-burst-engine.md)** - Detailed burst engine design documentation
- **[burst_engine.md](burst_engine.md)** - Burst engine usage patterns and examples
