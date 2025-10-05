# Debug Commands for Cortical Area Isolation Issue

## Immediate Investigation Commands

### 1. Check Cortical Mapping
```python
# In FEAGI Python shell:
cm = ConnectomeManager.instance()

# Check your isolated area mapping
cortical_id = "cS7aaa"
cortical_idx = cm.cortical_mapping.get_idx(cortical_id)
reverse_id = cm.cortical_mapping.get_id(cortical_idx)

print(f"Mapping: '{cortical_id}' <-> {cortical_idx} <-> '{reverse_id}'")

# Should be: 'cS7aaa' <-> X <-> 'cS7aaa' (where X is some integer)
# If reverse_id != cortical_id, you have mapping corruption!
```

### 2. Check for Phantom Synapses
```python
# Get neurons in isolated area
neurons = cm.get_neurons_by_cortical_area("cS7aaa")
print(f"Isolated area has {len(neurons)} neurons")

# Check for incoming synapses (should be ZERO)
if hasattr(cm, '_npu_interface') and cm._npu_interface:
    synapse_array = cm._npu_interface.synapse_array
    if synapse_array:
        incoming_count = 0
        for src_neuron, synapse_indices in synapse_array.source_neuron_index.items():
            for syn_idx in synapse_indices:
                target_neuron = synapse_array.target_neuron_ids[syn_idx]
                if target_neuron in neurons:
                    incoming_count += 1
                    print(f"PHANTOM SYNAPSE: {src_neuron} -> {target_neuron}")
        
        print(f"Total incoming synapses: {incoming_count} (should be 0!)")
```

### 3. Monitor FCL in Real-Time
```python
# Hook into FCL injection to catch corruption
from feagi.npu.burst_engine import BurstEngine

# Get burst engine from process manager or existing instance
# burst_engine = BurstEngine.get_instance()  # Check if this method exists
# Or get it from the running system - method depends on your setup
target_cortical_idx = cm.cortical_mapping.get_idx("cS7aaa")

# Store original method
original_inject = burst_engine.injection_service.inject_synaptic_propagation

def debug_inject(fcl, propagation_data):
    for cortical_idx, connections in propagation_data.items():
        if cortical_idx == target_cortical_idx:
            print(f"🚨 INJECTION into isolated area! {len(connections)} connections")
            # This should NEVER happen for an isolated area!
    return original_inject(fcl, propagation_data)

# Replace with debug version
burst_engine.injection_service.inject_synaptic_propagation = debug_inject

# Now run your simulation and watch for alerts
```

### 4. Verify Neuron ID Mappings
```python
# Check mapping consistency
for neuron_id in neurons[:10]:  # Check first 10 neurons
    # ConnectomeManager mapping
    cm_index = cm._neuron_id_to_index_map.get(neuron_id)
    
    # NPU NeuronArray mapping
    npu_index = cm._npu_interface.neuron_array.neuron_id_to_index.get(neuron_id)
    
    if cm_index != npu_index:
        print(f"MAPPING INCONSISTENCY: Neuron {neuron_id}: CM={cm_index}, NPU={npu_index}")
        
    # Check cortical assignment
    if cm_index is not None:
        assigned_cortical_idx = cm._npu_interface.neuron_array.cortical_idxs[cm_index]
        if assigned_cortical_idx != target_cortical_idx:
            print(f"WRONG CORTICAL ASSIGNMENT: Neuron {neuron_id} assigned to idx {assigned_cortical_idx}, expected {target_cortical_idx}")
```

## Log Analysis Commands

### Enable Debug Logging
```python
import logging
logging.getLogger('feagi.npu.burst_engine').setLevel(logging.DEBUG)
logging.getLogger('feagi.npu.fcl_injector').setLevel(logging.DEBUG)
logging.getLogger('feagi.bdu.connectome_manager').setLevel(logging.DEBUG)
```

### Check Recent Logs
```bash
# Look for recent FCL injection messages
grep -i "fcl\|inject" feagi_core/*.log | tail -50

# Look for synaptic propagation messages  
grep -i "synaptic\|propagation" feagi_core/*.log | tail -50

# Look for cortical area messages
grep -i "cortical\|cS7aaa" feagi_core/*.log | tail -50
```

## Emergency Mitigation

If you need to stop the spurious activations immediately:

### 1. Temporarily Disable FCL Injection
```python
# Nuclear option: disable FCL injection for the isolated area
original_add_candidates = burst_engine.injection_service.fcl.add_candidates_soa

def safe_add_candidates(cortical_idx, neuron_ids, potential_deltas, excitatory_mask=None):
    if cortical_idx == target_cortical_idx:
        print(f"BLOCKED injection into isolated area {cortical_idx}")
        return 0  # Block injection
    return original_add_candidates(cortical_idx, neuron_ids, potential_deltas, excitatory_mask)

burst_engine.injection_service.fcl.add_candidates_soa = safe_add_candidates
```

### 2. Clear Phantom Synapses
```python
# WARNING: This modifies the connectome! Use with caution!
if hasattr(cm, '_npu_interface') and cm._npu_interface:
    synapse_array = cm._npu_interface.synapse_array
    neurons_set = set(neurons)
    
    # Find and remove synapses targeting the isolated area
    synapses_to_remove = []
    for src_neuron, synapse_indices in synapse_array.source_neuron_index.items():
        for syn_idx in synapse_indices:
            target_neuron = synapse_array.target_neuron_ids[syn_idx]
            if target_neuron in neurons_set:
                synapses_to_remove.append((src_neuron, target_neuron))
    
    print(f"Found {len(synapses_to_remove)} phantom synapses to remove")
    
    # Remove them (implementation depends on synapse array structure)
    for src, tgt in synapses_to_remove:
        # This would need to be implemented based on your synapse removal logic
        print(f"Would remove: {src} -> {tgt}")
```

## Report Generation

After running diagnostics:

```bash
# Run comprehensive diagnostic and save report
python debug_cortical_mapping_isolation.py --area-id cS7aaa --timesteps 10 --output isolation_debug_report.txt

# Run quick check
python quick_isolation_check.py --area-id cS7aaa
```

The exit codes will tell you:
- 0: No immediate issues found  
- 1: Issues found, cause unclear
- 2: Root cause identified (phantom synapses)
- 3: Diagnostic failed
