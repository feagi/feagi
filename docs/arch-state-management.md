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
- **Morton spatial hash integration (NEW)**: Tracks active Morton implementation and coordinate limits

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
- `morton_spatial_hash`: Notifies when Morton spatial hash configuration changes

## Service States

Service components in FEAGI transition through these states:

- `UNAVAILABLE`: Service is not available or not started
- `UNINITIALIZED`: Initial state before component is set up
- `INITIALIZING`: Component is in the process of initializing
- `READY`: Component is fully initialized and operational
- `ON_HOLD`: **NEW** - Component is alive but paused (burst engine only)
- `DEGRADED`: Component is running but with reduced functionality
- `ERROR`: Component encountered an error during operation
- `FAILED`: Component failed to initialize or encountered a critical error

## Morton Spatial Hash Integration (NEW)

The state manager now tracks Morton spatial hash information for system-wide coordination and validation:

### Morton Spatial Hash Tracking

The state manager maintains awareness of the active Morton spatial hash implementation:

- **Morton Class Name**: Tracks the active implementation ("RoaringSpatialHash")
- **Coordinate Limits**: Tracks maximum coordinate values (2,097,152 per dimension for 21-bit encoding)
- **System Validation**: Provides coordinate validation for cortical area creation
- **Integration Points**: ConnectomeManager, genome validation, spatial queries

### State Manager Morton API

```python
from feagi.core.state_manager import get_state_manager

state_manager = get_state_manager()

# Get Morton coordinate limits
limit = state_manager.get_morton_coordinate_limit()  # Returns 2,097,152

# Get Morton class name
morton_class = state_manager.get_morton_class_name()  # Returns "RoaringSpatialHash"

# Check if Morton is registered
has_morton = state_manager.has_morton_class_info()  # Returns True

# Get comprehensive Morton info
info = state_manager.get_morton_spatial_hash_info()
# Returns: {"morton_class": "RoaringSpatialHash", "coordinate_limit": 2097152}

# Register Morton implementation (done automatically by ConnectomeManager)
state_manager.set_morton_class_info("RoaringSpatialHash", 2097152)
```

### Cortical Area Dimension Validation

The ConnectomeManager uses state manager Morton information to validate cortical area dimensions:

```python
from feagi.bdu.connectome_manager import ConnectomeManager

cm = ConnectomeManager(1000)

# Get maximum allowable dimensions based on Morton limits
max_dims = cm.get_max_allowable_cortical_area_dimensions()
# Returns: (2097151, 2097151, 2097151)

# Morton spatial hash info from state manager
morton_info = cm.get_morton_spatial_hash_info()
print(f"Active Morton class: {morton_info['morton_class']}")
print(f"Coordinate limit: {morton_info['coordinate_limit']:,}")

# Validation prevents oversized cortical areas
try:
    area_id = cm.add_cortical_area(
        name="Valid Area", 
        dimensions=(100, 100, 100),  # Within 21-bit limits
        position=(0, 0, 0)
    )
    print(f"✅ Created area: {area_id}")
except ValueError as e:
    print(f"❌ Area blocked: {e}")

# This will fail validation
try:
    cm.add_cortical_area(
        name="Invalid Area", 
        dimensions=(3000000, 100, 100),  # Exceeds 21-bit limit
        position=(0, 0, 0)
    )
except ValueError as e:
    print(f"✅ Correctly blocked oversized area: {e}")
```

### Morton Integration Benefits

1. **Prevents Coordinate Overflow**: Validates cortical area dimensions against Morton encoding limits
2. **System-Wide Awareness**: All components can query Morton capabilities through state manager
3. **Future-Proof Design**: Easy to extend for dynamic bit-width selection (32-bit vs 64-bit Morton)
4. **Consistent Validation**: Single source of truth for coordinate limits across the system
5. **Error Prevention**: Blocks creation of cortical areas that would cause Morton encoding failures
- `STOPPED`: Component was intentionally stopped
- `SYNCING`: Component is synchronizing state with other components
- `SYNC_COMPLETE`: Synchronization has completed successfully
- `SYNC_ERROR`: Synchronization failed, needs intervention

### Burst Engine Specific States

The burst engine uses a strict lifecycle with specific state transitions:

- `UNAVAILABLE` → Default state at FEAGI launch
- `READY` → Engine running and processing neurons
- `ON_HOLD` → Engine alive but neural processing paused
- `FAILED/ERROR` → Engine cannot start or encountered critical errors

**Critical Design Rule**: The burst engine MUST be in `READY` or `ON_HOLD` state when a genome is loaded. If not, the system will automatically start the engine or fail the genome load operation.

See [Burst Engine Lifecycle Architecture](arch-burst-engine-lifecycle.md) for complete details on state transitions and dependencies.

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
| FQ Sampler | Fire Candidate List sampling | 🔥 |
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
