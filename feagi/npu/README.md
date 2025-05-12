# Neural Processing Unit (NPU)

The NPU is responsible for simulating neuron dynamics and handling all burst-related operations.

## Key Components

### Burst Engine
The Burst Engine manages neuron firing dynamics and is designed for RTOS compatibility. Key features:

- **Standby Mode**: Initializes without requiring a genome, breaking circular dependencies
- **State Management**: Uses explicit state transitions with emoji-based logging
- **Dependency Injection**: No global state, all dependencies are passed explicitly
- **RTOS-Friendly**: Deterministic timing and no dynamic allocations in the main loop

### FCL Manager
Maintains the Fire Candidate List (FCL) which tracks neurons that are firing or scheduled to fire.

### FCL Sampler
Provides configurable sampling of the FCL for visualization and motor output.

## State Transitions

The Burst Engine follows these state transitions:

1. **UNAVAILABLE** → **INITIALIZING**: During initial creation
2. **INITIALIZING** → **READY**: Once basic initialization is complete
3. **READY** → **RUNNING**: When simulation starts
4. **RUNNING** → **READY**: When simulation pauses
5. **ANY_STATE** → **UNAVAILABLE**: During shutdown

## Genome Integration

When a genome is loaded, the Burst Engine is updated via the `update_with_genome()` method:

```python
# After loading a genome:
burst_engine = core_api_service.get_burst_engine()
if burst_engine:
    burst_engine.update_with_genome()
```

This design allows the system to initialize completely before a genome is loaded. 