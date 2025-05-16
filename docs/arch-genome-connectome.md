# Genome-Connectome Architecture

This document describes the synchronization architecture between the genome (blueprint) and connectome (runtime brain) in FEAGI.

## Core Concepts

FEAGI maintains a clear separation between:

- **Genome**: The blueprint that defines the brain's structure and properties
- **Connectome**: The runtime implementation of the brain based on the genome

Changes to the genome must be propagated to the connectome in a consistent, atomic manner.

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