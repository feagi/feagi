# FEAGI State Management Architecture

*Last Updated: May 15, 2025*

## Overview

FEAGI's state management system provides a high-performance, memory-mapped approach to tracking all internal states with near-zero overhead access. This architecture ensures consistent state tracking across components and enables proper synchronization between different parts of the system.

## State Manager

The `FeagiStateManager` class is responsible for tracking and managing the state of various FEAGI components, including:

- Genome state (loaded, saving, etc.)
- Connectome state (ready, initializing, etc.)
- Service states (API, ZMQ, Burst Engine, etc.)
- Simulation state (running, paused, etc.)
- Brain readiness

### Notification System

The state manager includes a comprehensive notification system that allows components to subscribe to state changes:

```python
# Register for notifications when genome state changes
state_manager.register_notification_callback("genome", my_callback_function)

# Callback function signature
def my_callback_function(old_state, new_state):
    print(f"Genome state changed from {old_state} to {new_state}")
```

Available notification categories:
- `genome`: Notifies when genome state changes
- `connectome`: Notifies when connectome state changes 
- `burst_engine`: Notifies when burst engine state changes
- `simulation`: Notifies when simulation state changes

## Service States

Service components in FEAGI transition through these states:

- `UNINITIALIZED`: Initial state before component is set up
- `INITIALIZING`: Component is in the process of initializing
- `READY`: Component is fully initialized and operational
- `FAILED`: Component failed to initialize or encountered an error
- `STOPPED`: Component was intentionally stopped
- `SYNCING`: Component is synchronizing state with other components
- `SYNC_COMPLETE`: Synchronization has completed successfully
- `SYNC_ERROR`: Synchronization failed, needs intervention

## Genome-Connectome Synchronization

The state manager includes specific states for tracking synchronization between the genome and connectome:

```python
# Get current sync state
sync_state = state_manager.genome_sync_state

# Register for sync notifications
state_manager.register_sync_observer(my_component)

# Begin a transaction for genome modifications
with state_manager.begin_genome_transaction() as transaction:
    transaction.add_change("add_cortical_area", cortical_area_data)
    # Changes are automatically committed at the end of the block
    # or rolled back if an exception occurs
```

All genome modifications should use the transaction system to ensure proper synchronization with the connectome.

## Standardized Logging

State transitions are logged with distinct emoji prefixes for visibility:

### Tracked Systems and Their Emojis

| System | Description | Emoji |
|--------|-------------|-------|
| Burst Engine | Neural firing dynamics | ⚡ |
| Connectome | Neuron and synapse data | 🧠 |
| API Service | REST API availability | 🌐 |
| ZMQ Service | Messaging system | 📡 |
| FCL Sampler | Fire Candidate List sampling | 🔥 |
| Genome | Current genome status | 🧬 |
| Brain Readiness | Overall system readiness | 🟢 |
| Synchronization | System sync processes | 🔄 |

### Logging Implementation

FEAGI uses a standardized approach to logging state changes:

1. **The `_log_state_change` Function**:
   - All state change logs must go through this function
   - Takes an emoji parameter and message parameter
   - Handles compatibility with both custom and standard loggers

2. **Implementation Pattern**:
   ```python
   def set_some_state(self, state):
       old_state = self.get_some_state()
       # Update state storage
       self.storage.value = state
       # Log with emoji
       _log_state_change("🔄", f"State changed: {old_state} → {state}")
   ```

## State-Aware Architecture

The state management system enables several key architectural benefits:

1. **Dependency Checking**: Components can verify system state before operations
2. **Graceful Degradation**: Services can adapt behavior based on component states
3. **Enhanced Debugging**: State transitions provide clear operational insights
4. **Optimistic Concurrency**: Transaction model prevents race conditions
5. **Reactive Architecture**: Components can respond to system state changes

## Best Practices

When working with the state management system:

1. Always check state before operations that depend on specific components
2. Use the transaction system for operations that modify multiple states
3. Register for state change notifications rather than polling
4. Log state transitions consistently using the provided emoji system
5. Check state validity through proper validation methods

## Related Documentation

- [System Overview](arch-system-overview.md)
- [IPC Architecture](arch-ipc.md)
- [API Refactoring](adr-api-refactoring.md) 