# Genome-Connectome Synchronization

FEAGI employs a transaction-based system to ensure synchronization between genome modifications and their effects on the connectome.

## Genome Transactions

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

## Rollback Capability

If a transaction fails, it can be rolled back to restore the genome to its previous state:

```python
try:
    transaction.commit()
except Exception:
    transaction.rollback()
```

## State Synchronization

The state manager tracks synchronization status between genome and connectome:

- `ServiceState.SYNCING`: Changes are being applied
- `ServiceState.SYNC_COMPLETE`: Changes have been successfully applied
- `ServiceState.SYNC_ERROR`: Error occurred during synchronization

## Overview

FEAGI maintains a clear separation between the genome (blueprint) and connectome (runtime brain). This document explains how changes are propagated between them to ensure consistency.

## Synchronization Model

FEAGI uses a transaction-based approach for genome modifications:

1. Begin a transaction
2. Add one or more changes to the transaction
3. Commit the transaction, which:
   - Applies changes to the genome
   - Synchronizes the connectome
   - Notifies observers of the changes

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