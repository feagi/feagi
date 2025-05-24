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
- **Special Area Support**: Handles special cortical areas like power areas with automatic neuron injection

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

### Special Area Handler

The Special Area Handler detects and manages cortical areas with unique behaviors:

- **Power Areas**: Areas ending with "_pwr" or named "___pwr" that inject all their neurons into the FCL on every burst
- **Modulator Areas**: Areas ending with "_mod" that affect other areas' firing patterns
- **Enhanced Memory Areas**: Areas with extended temporal windows and specialized processing

**Power Area Usage:**
```python
# Create a power area in your genome
cortical_area = {
    "id": "___pwr",  # Special naming pattern
    "name": "Power Injection Area",
    "dimensions": [10, 10, 1],
    "neurons_per_voxel": 1,
    "properties": {
        "__power_injection": True,  # Alternative property-based detection
        "injection_timing": "pre_burst",  # When to inject: pre_burst, during_burst, post_burst
        "injection_probability": 1.0  # Always inject (1.0) or probabilistic (0.0-1.0)
    }
}
```

### FCL Injection Service

The FCL Injection Service coordinates the injection of neurons from special areas into the Fire Candidate List:

- **Batch Processing**: Efficiently processes large numbers of neurons in batches
- **Timing Control**: Supports pre-burst, during-burst, and post-burst injection phases
- **Probabilistic Injection**: Supports probabilistic neuron injection for stochastic effects
- **Performance Monitoring**: Tracks injection statistics and performance metrics

```python
# Configure injection service
config = {
    'batch_injection_size': 1000,
    'enable_probabilistic_injection': True,
    'enable_timing_optimization': True
}

# Service is automatically initialized by BurstEngine when power areas are detected
stats = engine.get_power_injection_statistics()
engine.set_power_injection_enabled("___pwr", True)
```

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

### FCL Sampler

The FCL Sampler extracts burst data at configurable rates for visualization and motor output:

- **Per-Area Sampling**: Different sampling rates for different cortical areas
- **Best-Effort Delivery**: Drops samples when consumers can't keep up to maintain real-time performance
- **Subscriber-Aware**: Only samples when there are active visualization or motor consumers
- **RTOS-Compatible**: Deterministic, bounded execution suitable for real-time systems

```python
# Create sampler with 30Hz default rate
sampler = FCLSampler(fcl_manager, sample_frequency_hz=30, output_queue=viz_queue, connectome_manager=connectome_manager)

# Set per-area sample rates
area.properties['fcl_sample_rate'] = 60  # Sample this area at 60Hz

# Control sampling based on subscribers
sampler.set_visualization_subscribers(True)
sampler.set_motor_subscribers(False)
```

## Power Area Implementation

### Naming Conventions

Power areas are automatically detected using these patterns:

1. **Suffix Pattern**: Any cortical area ID ending with "_pwr" (e.g., "motor_pwr", "visual_pwr")
2. **Exact Match**: Area ID exactly equals "___pwr"
3. **Property Flag**: Area has `"__power_injection": true` in its properties

### Injection Timing

Power areas can inject neurons at different phases of the burst cycle:

- **pre_burst** (default): Inject before regular neuron firing - ensures power neurons are available for synaptic propagation
- **during_burst**: Inject during membrane potential updates - for modulation effects
- **post_burst**: Inject after regular processing - for cleanup or special effects

### Configuration Options

Power injection behavior can be configured at multiple levels:

```python
# Burst engine configuration
config = {
    'enable_power_injection': True,  # Global enable/disable
    'power_injection_timing': 'pre_burst',  # Default timing
    'special_area_config': {
        'batch_injection_threshold': 100
    },
    'fcl_injection_config': {
        'batch_injection_size': 1000,
        'enable_probabilistic_injection': True
    }
}

engine = BurstEngine(connectome_manager, fcl_manager, config)

# Per-area configuration through properties
power_area = {
    "id": "reward_pwr",
    "properties": {
        "__power_injection": True,
        "injection_timing": "pre_burst",
        "injection_probability": 1.0  # Always inject (0.0-1.0 for probabilistic)
    }
}
```

### Usage Examples

#### Global Attention System
```python
attention_area = {
    "id": "attn_pwr",
    "dimensions": [10, 10, 1],  # 100 attention neurons
    "properties": {
        "__power_injection": True,
        "injection_timing": "pre_burst",
        "injection_probability": 0.8  # Variable attention level
    }
}
```

#### Learning Signal Injection
```python
reward_area = {
    "id": "reward_pwr", 
    "properties": {
        "__power_injection": True,
        "injection_timing": "during_burst",  # Modulate ongoing activity
        "injection_probability": 1.0
    }
}
```

#### Debug/Test Area
```python
test_area = {
    "id": "test_pwr",
    "properties": {
        "__power_injection": True
    }
}

# Runtime control
engine.set_power_injection_enabled("test_pwr", True)   # Enable
engine.set_power_injection_enabled("test_pwr", False)  # Disable
```

### Monitoring and Statistics

Monitor power injection performance:

```python
# Get comprehensive statistics
stats = engine.get_power_injection_statistics()
print(f"Power areas: {stats['special_areas']['power_areas_count']}")
print(f"Total injections: {stats['injection']['total_injections']}")

# Preview next injection
preview = engine.fcl_injection_service.get_power_injection_preview()
print(f"Pre-burst neurons: {preview['pre_burst_neurons']}")

# Enable debug logging
import logging
logging.getLogger('feagi.npu.special_area_handler').setLevel(logging.DEBUG)
```

### Troubleshooting

**Power Areas Not Detected**:
- Check naming patterns (must end with "_pwr" or be "___pwr")
- Verify `__power_injection: true` property
- Check debug logs for detection messages

**Performance Impact**:
- Reduce power area size or batch injection size
- Use probabilistic injection (`injection_probability < 1.0`)
- Monitor burst frequency and injection statistics

### Performance Considerations

Power area injection is optimized for high-frequency operation:

- **Pre-cached Neuron Lists**: Power area neurons are cached on genome load
- **Batch Processing**: Large neuron lists are processed in configurable batches
- **Minimal Runtime Allocation**: All data structures pre-allocated during initialization
- **Statistics Tracking**: Comprehensive performance monitoring

### Use Cases

Power areas enable several advanced neural simulation patterns:

1. **Global Attention Mechanisms**: A power area can provide persistent background activation
2. **Learning Signals**: Inject reward/punishment signals across the entire brain
3. **Synchronization**: Provide timing signals for coordinated activity
4. **Debugging**: Force specific neurons to fire for testing and validation

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
    'enable_power_injection': True,          # Enable special area processing
    'power_injection_timing': 'pre_burst',   # Default injection timing
    'special_area_config': {
        'batch_injection_threshold': 100      # Threshold for batch processing
    },
    'fcl_injection_config': {
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
    '__modulator': False,                    # Mark as modulator area
    '__shed': False,                         # Enable for load shedding
    'injection_timing': 'pre_burst',         # Injection phase
    'injection_probability': 1.0,            # Injection probability
    'fcl_sample_rate': 30.0                 # Sampling rate for this area
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

1. **Identify Power Areas**: Look for areas that manually inject neurons into the FCL
2. **Update Naming**: Rename areas to use "_pwr" suffix or add `__power_injection` property
3. **Configure Timing**: Set appropriate injection timing based on previous behavior
4. **Test Performance**: Validate that injection doesn't impact target burst frequency

### Configuration Updates

Update your genome configuration to use the new power area features:

```python
# Before (manual injection)
def manual_power_injection():
    for neuron_id in power_neurons:
        fcl_manager.add_neuron(neuron_id)

# After (automatic injection)
power_area = {
    'id': 'attention_pwr',
    'properties': {'__power_injection': True}
}
```

## Troubleshooting

### Common Issues

1. **Power Areas Not Detected**: Check naming patterns and property flags
2. **Performance Impact**: Reduce batch size or disable probabilistic injection
3. **Timing Issues**: Adjust injection timing phase or enable optimizations

### Debug Logging

Enable detailed logging for power injection debugging:

```python
import logging
logging.getLogger('feagi.npu.special_area_handler').setLevel(logging.DEBUG)
logging.getLogger('feagi.npu.fcl_injection_service').setLevel(logging.DEBUG)
```

### Statistics and Monitoring

Monitor power injection performance:

```python
# Get detailed statistics
stats = engine.get_power_injection_statistics()
print(f"Total injections: {stats['injection']['total_injections']}")
print(f"Power areas: {stats['special_areas']['power_areas']}")

# Get injection preview
preview = engine.fcl_injection_service.get_power_injection_preview()
print(f"Neurons to inject: {preview['pre_burst_neurons']}")
``` 