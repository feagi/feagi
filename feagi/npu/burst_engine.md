# Burst Engine

The Burst Engine is the central component of the NPU (Neural Processing Unit) in FEAGI. It manages neural dynamics and coordinates the simulation of neural activity.

## Architecture

The Burst Engine follows a state machine architecture with the following states:

1. **UNAVAILABLE**: Initial state before engine creation
2. **INITIALIZING**: Engine is being created and configured
3. **READY**: Engine is initialized but not running
4. **RUNNING**: Engine is actively processing neural activity
5. **PAUSED**: Engine has temporarily stopped processing

## Core Components

### Burst Manager

Coordinates the timing and execution of neural simulation bursts:

```python
class BurstManager:
    def __init__(self, target_frequency_hz, max_computational_load):
        self.target_frequency_hz = target_frequency_hz
        self.max_computational_load = max_computational_load
        self.current_load = 0
        self.current_frequency = 0
```

### Neural Activity Processor

Processes neural activity according to neuron models:

```python
class NeuralActivityProcessor:
    def process_burst(self, fcl_manager, connectome_manager):
        # Process firing neurons
        # Update membrane potentials
        # Generate next FCL
```

### Load Monitor

Monitors computational load and adjusts processing if needed:

```python
class LoadMonitor:
    def check_load_shedding(self, areas, current_frequency, target_frequency):
        if current_frequency < 0.8 * target_frequency:
            return self.select_areas_for_load_shedding(areas)
        return []
```

## Integration with Connectome

The Burst Engine works closely with the Connectome Manager:

```python
# When genome changes
burst_engine.update_with_genome(connectome_manager.get_current_genome())

# During burst processing
active_neurons = fcl_manager.get_global_fcl()
connected_neurons = connectome_manager.get_downstream_neurons(active_neurons)
```

## Performance Optimization

The Burst Engine includes several performance optimizations:

- Parallel processing of independent neural regions
- Prioritization of critical neural pathways
- Load shedding for non-critical areas under high computational load
- Batch processing of neural activity 