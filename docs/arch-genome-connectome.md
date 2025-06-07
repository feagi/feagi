# Genome-Connectome Architecture

This document describes the synchronization architecture between the genome (blueprint) and connectome (runtime brain) in FEAGI.

## Core Concepts

FEAGI maintains a clear separation between:

- **Genome**: The blueprint that defines the brain's structure and properties
- **Connectome**: The runtime implementation of the brain based on the genome

Changes to the genome must be propagated to the connectome in a consistent, atomic manner.

## Single Source of Truth Architecture

### Architectural Principle

FEAGI implements a **single source of truth** architecture for genome data to eliminate data duplication and ensure consistency:

1. **State Manager**: Holds the authoritative, sanitized genome (THE source of truth)
2. **Connectome Manager**: Stores only structural/runtime data (neurons, synapses, positions)
3. **REST API**: Always reads cortical area properties from State Manager's genome
4. **Auto-Recovery**: Sanitizes genome in State Manager before any brain development

### Genome Loading Flow

The architecture follows this strict sequence:

```
1. Genome Service: Complete sanitization first
2. State Manager: Store sanitized genome as single source of truth  
3. State Manager: Notify Connectome Manager "new genome ready"
4. Connectome Manager: Build brain from State Manager's genome (NOT temp files)
5. REST API: Serve properties from State Manager's genome
```

#### Previous Problematic Pattern:
```
1. Genome Service: Auto-recovery sanitizes genome
2. Genome Service: Immediately calls connectome manager  
3. Connectome Manager: Builds from temp file (race condition!)
4. REST API: Reads from connectome's duplicate property storage
5. Result: Client gets unsanitized null values, crashes
```

#### Current Correct Architecture:
```
1. Genome Service: Complete sanitization first
2. State Manager: Store sanitized genome as single source of truth
3. Genome Service: Notify connectome manager with sanitized data
4. Connectome Manager: Build brain from state manager genome
5. REST API: Always read from state manager's sanitized genome
6. Result: Client always gets sanitized values, no crashes
```

### Code Implementation

#### Genome Service Implementation
```python
# ARCHITECTURE: Stage sanitized genome in state manager FIRST
if self.state_manager:
    self.state_manager.genome = genome_data  # Single source of truth
    self.state_manager.genome_file_name = filename
    self.state_manager.set_genome_state(GenomeState.LOADING)
    
# ARCHITECTURE: Build brain from state manager genome (not temp file)
embry = NeuroEmbryogenesis(connectome_manager=self._connectome_manager)
success = embry.develop_brain_from_genome_data(genome_data)  # Direct data, no files
```

#### Connectome Manager Integration
```python
# Connectome manager reads from state manager's genome for properties
def get_cortical_area_properties(self, cortical_id):
    # Connectome stores structural data only
    # Properties come from state manager's genome
    return state_manager.get_cortical_properties(cortical_id)
```

#### REST API Implementation
```python
# REST API always uses state manager as source of truth
def get_cortical_area_properties(cortical_id):
    state_manager = FeagiStateManager.instance()
    return state_manager.get_cortical_properties(cortical_id)  # Always sanitized
```

### Benefits

1. **No Data Duplication**: Properties stored once in State Manager
2. **Guaranteed Consistency**: All components read from same source
3. **Auto-Recovery Works**: Sanitization happens before brain development
4. **Client Safety**: No null values can reach clients
5. **Race Condition Elimination**: Sequential: sanitize → stage → notify → build
6. **Rust/RTOS Ready**: Clear data ownership, no hidden state

### Live Editing Architecture

User edits through Brain Visualizer follow this flow:

```
1. User Edit → State Manager genome update
2. State Manager → Notify Connectome Manager of change
3. Connectome Manager → Rebuild affected cortical areas live
4. REST API → Serves updated properties from State Manager
```

This ensures the editing loop maintains single source of truth.

### Architecture Requirements

**Current Implementation**:
- Direct data processing with `develop_brain_from_genome_data()` method
- Mandatory sanitization before brain development
- State Manager as authoritative genome source
- Sequential loading: sanitize → stage → notify → build

**Eliminated Patterns**:
- Temp file creation during genome loading
- Duplicate property storage in `area.properties` 
- Race conditions between sanitization and brain development
- Multiple sources of truth for cortical properties

## Transaction-Based Synchronization

FEAGI employs a transaction-based system to ensure synchronization between genome modifications and their effects on the connectome.

### Genome Transactions

The `GenomeTransaction` class provides an atomic way to modify the genome and propagate those changes to the connectome:

```python
# Create a transaction
transaction = core_api_service.begin_transaction()

# Record changes to apply
transaction.record_change("add_cortical_area", area_id, None, area_properties)

# Apply all changes atomically
transaction.commit()
```

Transactions can also be used with a context manager:

```python
with state_manager.begin_genome_transaction_context() as transaction:
    # Make changes
    transaction.record_change("update_cortical_area", area_id, old_props, new_props)
    # Automatically commits at the end of the block
    # Automatically rolls back on exception
```

### Rollback Capability

If a transaction fails, it can be rolled back to restore the genome to its previous state:

```python
try:
    transaction.commit()
except Exception:
    transaction.rollback()
```

## State Management

The state manager tracks synchronization status between genome and connectome:

- `ServiceState.SYNCING`: Changes are being applied
- `ServiceState.SYNC_COMPLETE`: Changes have been successfully applied
- `ServiceState.SYNC_ERROR`: Error occurred during synchronization

## Synchronization Process

The synchronization process follows these steps:

1. **Transaction Creation**: A transaction is initiated to group related changes
2. **Change Recording**: Changes are recorded but not yet applied
3. **Validation**: The transaction validates that all changes are consistent
4. **Genome Update**: Changes are applied to the genome
5. **Connectome Synchronization**: The connectome is updated to reflect the genome changes
6. **State Notification**: Observers are notified of the completed synchronization

## Implementation

### Transaction API

```python
# Via the CoreAPIService
def modify_cortical_area(core_api, cortical_id, properties):
    transaction = core_api.begin_genome_transaction()
    transaction.add_change("update_cortical_area", {
        "cortical_id": cortical_id,
        "properties": properties
    })
    return transaction.commit()

# Direct usage (for internal components)
def complex_genome_operation(state_manager, operations):
    with state_manager.begin_genome_transaction() as transaction:
        for operation in operations:
            transaction.add_change(operation["type"], operation["data"])
        # Transaction is auto-committed at the end of the block
```

### Observer Pattern

Components can register to be notified of genome changes:

```python
class MyComponent:
    def __init__(self, state_manager):
        state_manager.register_sync_observer(self)
        
    def on_sync_state_change(self, old_state, new_state, details):
        if new_state == ServiceState.SYNC_COMPLETE:
            # React to successful synchronization
            self.update_my_state()
```

## Supported Operations

The transaction system supports the following operations:

| Operation | Description | Connectome Effect |
|-----------|-------------|------------------|
| `add_cortical_area` | Create a new cortical area | Creates neuron structures |
| `update_cortical_area` | Modify cortical area properties | Updates neuron properties |
| `delete_cortical_area` | Remove a cortical area | Removes neurons and synapses |
| `add_mapping` | Create a new cortical mapping | Generates synapses |
| `update_mapping` | Modify mapping properties | Updates synaptic properties |
| `delete_mapping` | Remove a cortical mapping | Removes synapses |

## Concurrency Considerations

The synchronization system handles concurrency with these mechanisms:

1. **Transaction Locking**: Only one transaction can be active at a time
2. **State Transitions**: Clear state transitions prevent race conditions
3. **Atomic Operations**: All changes in a transaction succeed or fail together
4. **Notification Ordering**: Observers are notified in a consistent order

## Best Practices

1. **Always use transactions**: Never modify the genome directly
2. **Minimize transaction size**: Keep transactions focused on related changes
3. **Handle sync failures**: Be prepared for transactions to fail
4. **Avoid race conditions**: Don't start new transactions until current ones complete
5. **Validate before commit**: Check that changes are valid before committing

## Rust/RTOS Considerations

This synchronization model is designed to be compatible with Rust and RTOS environments:

- Clear state boundaries with explicit transitions
- Deterministic resource allocation
- Avoidance of hidden side effects
- Well-defined ownership model 