# FEAGI State Management

FEAGI's state management system provides a high-performance, memory-mapped approach to tracking all internal states with near-zero overhead access.

## State Manager

The `FeagiStateManager` class is responsible for tracking and managing the state of various FEAGI components, including:

- Genome state (loaded, saving, etc.)
- Connectome state (ready, initializing, etc.)
- Service states (API, ZMQ, Burst Engine, etc.)
- Simulation state (running, paused, etc.)
- Brain readiness

### Notification System

The state manager now includes a comprehensive notification system that allows components to subscribe to state changes:

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

## Overview

FEAGI uses a central state manager (`FeagiStateManager`) to track the state of critical subsystems and ensure proper coordination between components. The state manager is a singleton accessible throughout the application.

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

The state manager now includes specific states for tracking synchronization between the genome and connectome:

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

## Logging

State transitions are logged with distinct emoji prefixes for visibility:
- 🧬 Genome state changes
- 🔄 Synchronization state changes
- ⚡ Burst Engine state changes
- 🧠 Brain readiness state changes

## Tracked Systems and Their Emojis

| System | Description | Emoji |
|--------|-------------|-------|
| Burst Engine | Neural firing dynamics | ⚡ |
| Connectome | Neuron and synapse data | 🧠 |
| API Service | REST API availability | 🌐 |
| ZMQ Service | Messaging system | 📡 |
| FCL Sampler | Fire Candidate List sampling | 🔥 |
| Genome | Current genome status | 🧬 |
| Brain Readiness | Overall system readiness | 🟢 |

## Logging Format 