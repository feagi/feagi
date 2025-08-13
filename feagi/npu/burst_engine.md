# Burst Engine - Clean Generic Architecture

The Burst Engine is the central component of the NPU (Neural Processing Unit) in FEAGI. It manages neural dynamics and coordinates the simulation of neural activity using a clean, area-agnostic architecture with specialized mixins.

## Architecture Overview

The Burst Engine uses a **modular mixin architecture** that separates concerns and provides clean, maintainable code:

### Core Class Structure

```python
class BurstEngine(BurstEngineDebugMixin, BurstEnginePerformanceMixin):
    """
    Main burst engine with modular functionality through mixins.
    Completely area-agnostic - handles all special area types through injection service.
    """
```

### Modular Components

1. **BurstEngine** (main class): Core neural simulation logic, area-agnostic
2. **BurstEngineDebugMixin**: Debug and diagnostics functionality
3. **BurstEnginePerformanceMixin**: SIMD acceleration and performance monitoring

## State Management

The Burst Engine follows a state machine architecture with the following states:

1. **UNAVAILABLE**: Initial state before engine creation
2. **INITIALIZING**: Engine is being created and configured
3. **READY**: Engine is initialized and ready to process
4. **RUNNING**: Engine is actively processing neural activity
5. **ERROR**: Engine encountered a fatal error
6. **STOPPED**: Engine has been stopped gracefully

## Core Components

### Main BurstEngine Class

Handles core neural simulation functionality with clean separation of concerns:

```python
class BurstEngine:
    def __init__(self, connectome_manager, fcl_manager, config):
        # Singleton pattern ensures only one instance
        self.connectome_manager = connectome_manager
        self.fcl_manager = fcl_manager

        # Initialize generic injection service (area-agnostic)
        self._initialize_injection_service()

        # Manually initialize mixins (avoids multiple inheritance issues)
        BurstEnginePerformanceMixin.__init__(self)
        BurstEngineDebugMixin.__init__(self)

    def _process_burst(self):
        """Core burst processing with unified FCL candidate model."""
        # 1. External candidates injection (all special area types)
        if self.injection_service:
            self.injection_service.inject_pre_burst(self.burst_count)

        # 2. Unified neural computation (internal + external candidates)
        fired_neurons = self.connectome_manager.update_membrane_potentials()

        # 3. Additional injection phases if needed
        if self.injection_service:
            self.injection_service.inject_during_burst(self.burst_count)
            self.injection_service.inject_post_burst(self.burst_count)

        # 4. Debug output if enabled
        if self.debug_npu:
            self._debug_fire_queue_output()

        return fired_neurons
```

### Debug Mixin (BurstEngineDebugMixin)

Provides comprehensive debugging and diagnostics:

```python
class BurstEngineDebugMixin:
    def _debug_fire_queue_output(self):
        """Detailed fire queue analysis when --debug-npu is enabled."""
        # Global FCL summary
        # Per-cortical area breakdown
        # FQ Sampler status and data
        # Motor/visualization stream debugging

    def register_fq_sampler(self, fq_sampler):
        """Register FQ samplers for debugging and monitoring."""

    def get_debug_statistics(self):
        """Get comprehensive debug statistics."""

    def debug_validate_state(self):
        """Validate internal state consistency."""
```

### Performance Mixin (BurstEnginePerformanceMixin)

Provides SIMD acceleration and performance monitoring:

```python
class BurstEnginePerformanceMixin:
    def _init_simd_support(self):
        """Initialize SIMD detection and configuration."""
        # Uses centralized SIMD configuration from State Manager
        # Supports AVX, AVX2, AVX512, NEON, etc.

    def measure_actual_frequency(self, duration_seconds=5.0):
        """Measure both actual and potential burst frequencies."""
        # Collects detailed timing data
        # Returns comprehensive performance metrics

    def burst(self, burst_size=None, use_gpu=False):
        """Execute neural processing burst with SIMD optimization."""
        # SIMD-optimized fire candidate detection
        # Vectorized membrane potential updates
        # Performance profiling and statistics

    def get_performance_metrics(self):
        """Get comprehensive performance statistics."""
```

## Special Area Support

### Generic Injection Architecture

The burst engine uses a clean, area-agnostic architecture for all special areas:

```python
# All special areas handled through unified injection service
special_areas = {
    "power_areas": ["___pwr", "motor_pwr", "sensory_pwr"],
    "modulator_areas": ["dopamine_mod", "attention_mod"],
    "custom_areas": ["memory_enhanced", "learning_signal"]
}

# Automatic detection and injection service initialization
if any_special_areas_detected:
    self.injection_service = FCLInjectionService(
        fcl_manager=self.fcl_manager,
        special_area_handler=self.special_area_handler
    )
```

### Unified FCL Candidate Model

All special areas add candidates to FCL using the same unified model:

- **External Candidates**: Special areas add candidates to FCL (don't fire directly)
- **Unified Processing**: All candidates (internal synaptic + external special) processed together
- **Clean Separation**: Burst engine remains completely area-agnostic

### Injection Timing

Special area injection supports three timing phases:

- **pre_burst**: Add candidates before membrane potential updates (power areas, sensory input)
- **during_burst**: Add candidates during processing (modulators, attention)
- **post_burst**: Add candidates after standard processing (cleanup, memory consolidation)

## Performance Features

### SIMD Acceleration

The performance mixin provides SIMD acceleration:

```python
# Automatic SIMD backend detection
backends = ["AVX512", "AVX2", "AVX", "SSE2", "NEON", "SCALAR"]

# Vectorized operations for large neuron populations
fired_neurons = self._process_membrane_updates_simd(candidates)

# Performance profiling with detailed metrics
stats = self.get_performance_metrics()
```

### Frequency Measurement

Real-time frequency measurement and analysis:

```python
# Measure actual vs. potential frequency
results = engine.measure_actual_frequency(duration_seconds=5.0)

# Results include:
# - actual_frequency_hz: Achieved frequency including delays
# - potential_frequency_hz: Maximum possible frequency
# - efficiency_ratio: actual/potential performance ratio
# - timing statistics and profiling data
```

## Debug Features

### NPU Debug Mode

Enable comprehensive debugging with `--debug-npu` flag:

```python
# Detailed burst information
[DEBUG] ===== NPU DEBUG - BURST 42 =====
[STATS] Global Fire Summary:
   Total firing neurons: 1,247
   Burst frequency: 10.0Hz target

[BRAIN] Per-Area Breakdown (8 active areas):
   motor_cortex: 423 neurons (33.9%) - [1001, 1002, 1003, 1004, 1005]... (+418 more)
   sensory_input: 312 neurons (25.0%) - [2001, 2002, 2003]... (+309 more)

[TARGET] Sampler Debug Information:
   FQ Samplers Active: 2
      FQSampler-1: RUNNING @ 30.0Hz
         Viz subscribers: YES
         Motor subscribers: NO
```

### FQ Sampler Integration

Debug mixin tracks and monitors FQ samplers:

```python
# Register samplers for debugging
engine.register_fq_sampler(motor_sampler)
engine.register_fq_sampler(viz_sampler)

# Get sampler statistics
debug_stats = engine.get_debug_statistics()
```

## Integration with Other Components

### Connectome Manager

The burst engine works closely with the Connectome Manager:

```python
# When genome changes
engine.update_with_genome()

# During burst processing
fired_neurons = connectome_manager.update_membrane_potentials()
```

### FQ Samplers

Integration with visualization and motor samplers:

```python
# Automatic registration for debugging
process_manager._viz_fq_sampler = UnifiedFQSampler(...)
engine.register_fq_sampler(process_manager._viz_fq_sampler)

# Debug output includes sampler status
```

## Usage Examples

### Basic Usage

```python
# Create burst engine with modular architecture
engine = BurstEngine(connectome_manager, fcl_manager, config={
    'debug_npu': True,           # Enable debug output
    'enable_injection': True,    # Enable special areas (all types)
    'desired_frequency_hz': 10.0     # Target frequency
})

# Start processing
engine.run()
```

### Performance Monitoring

```python
# Enable frequency measurement
engine.enable_frequency_measurement(True)

# Get real-time performance metrics
metrics = engine.get_performance_metrics()
print(f"SIMD Backend: {metrics['simd_backend']}")
print(f"Vector Width: {metrics['simd_vector_width']}")
print(f"Average Processing Time: {metrics['avg_processing_time_ms']:.2f}ms")
```

### Debug Analysis

```python
# Get comprehensive debug information
debug_info = engine.get_debug_statistics()
print(f"Registered FQ Samplers: {debug_info['fq_samplers_registered']}")
print(f"Debug Mode: {debug_info['debug_mode_enabled']}")

# Validate state consistency
validation = engine.debug_validate_state()
print(f"All validations passed: {all(validation.values())}")
```

### Configuration Example

```python
config = {
    'enable_injection': True,  # Enable special areas (all types)
    'desired_frequency_hz': 10.0,
    'special_area_config': {
        'batch_injection_threshold': 100
    },
    'injection_config': {
        'batch_injection_size': 1000,
        'enable_probabilistic_injection': True
    }
}

engine = BurstEngine(connectome_manager, fcl_manager, config)
```

This modular architecture provides clean separation of concerns while maintaining high performance and comprehensive debugging capabilities for neural simulation research and development.
