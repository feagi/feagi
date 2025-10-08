// Synaptic Propagation Compute Shader (WGSL)
//
// Processes synaptic contributions from fired neurons to target neurons.
// This is the BOTTLENECK - 95% of burst time in Python implementation.
//
// Algorithm:
// 1. For each fired neuron, find its outgoing synapses
// 2. Calculate synaptic contribution (weight × conductance × sign)
// 3. Accumulate to target neuron membrane potentials
//
// Challenge: GPU hash table lookup for synapse indices

// Synapse arrays (Structure-of-Arrays, persistent)
@group(0) @binding(0) var<storage, read> source_neurons: array<u32>;
@group(0) @binding(1) var<storage, read> target_neurons: array<u32>;
@group(0) @binding(2) var<storage, read> weights: array<u32>;        // u8 stored as u32
@group(0) @binding(3) var<storage, read> conductances: array<u32>;   // u8 stored as u32
@group(0) @binding(4) var<storage, read> synapse_types: array<u32>;  // u8 stored as u32
@group(0) @binding(5) var<storage, read> synapse_valid_mask: array<u32>;  // Bitpacked

// Synapse index (GPU hash table)
// Maps source_neuron_id → [start_index, end_index] in synapse_list
@group(0) @binding(6) var<storage, read> synapse_index_keys: array<u32>;    // Source neuron IDs
@group(0) @binding(7) var<storage, read> synapse_index_starts: array<u32>;  // Start index in synapse_list
@group(0) @binding(8) var<storage, read> synapse_index_counts: array<u32>;  // Count of synapses
@group(0) @binding(9) var<storage, read> synapse_list: array<u32>;          // Flat array of synapse indices

// Neuron membrane potentials (read-write, accumulated)
@group(0) @binding(10) var<storage, read_write> membrane_potentials: array<atomic<i32>>;  // Atomic for parallel accumulation

// Fired neurons input (dense array)
@group(0) @binding(11) var<storage, read> fired_neurons: array<u32>;

// Parameters
@group(0) @binding(12) var<storage, read> params: SynapticParams;

struct SynapticParams {
    fired_count: u32,          // Number of fired neurons
    synapse_count: u32,        // Total synapses
    index_capacity: u32,       // Hash table capacity
    _padding: u32,
}

// Check if bit is set in bitpacked array
fn is_synapse_valid(index: u32) -> bool {
    let word_idx = index / 32u;
    let bit_idx = index % 32u;
    return (synapse_valid_mask[word_idx] & (1u << bit_idx)) != 0u;
}

// Hash function for GPU hash table lookup
fn hash_neuron_id(neuron_id: u32, capacity: u32) -> u32 {
    // Simple multiplicative hash
    let hash = neuron_id * 2654435761u;
    return hash % capacity;
}

// Find synapse index for source neuron (linear probing)
fn find_synapse_index(source_neuron_id: u32) -> i32 {
    let capacity = params.index_capacity;
    var slot = hash_neuron_id(source_neuron_id, capacity);
    
    // Linear probing (max 16 probes to avoid infinite loops)
    for (var probe = 0u; probe < 16u; probe = probe + 1u) {
        let key = synapse_index_keys[slot];
        
        if (key == source_neuron_id) {
            return i32(slot);  // Found!
        }
        
        if (key == 0xFFFFFFFFu) {
            return -1;  // Empty slot = not found
        }
        
        // Next slot (linear probing)
        slot = (slot + 1u) % capacity;
    }
    
    return -1;  // Not found after max probes
}

// Process one fired neuron (workgroup processes all fired neurons)
@compute @workgroup_size(256)
fn synaptic_propagation_main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let fired_idx = global_id.x;
    
    // Bounds check
    if (fired_idx >= params.fired_count) {
        return;
    }
    
    // Get fired neuron ID
    let source_neuron_id = fired_neurons[fired_idx];
    
    // Find synapse index for this source neuron
    let index_slot = find_synapse_index(source_neuron_id);
    if (index_slot < 0) {
        return;  // No synapses for this neuron
    }
    
    let slot = u32(index_slot);
    let synapse_start = synapse_index_starts[slot];
    let synapse_count = synapse_index_counts[slot];
    
    // Process all synapses from this fired neuron
    for (var i = 0u; i < synapse_count; i = i + 1u) {
        let synapse_idx = synapse_list[synapse_start + i];
        
        // Check if synapse is valid
        if (!is_synapse_valid(synapse_idx)) {
            continue;
        }
        
        // Get synapse properties
        let target_neuron = target_neurons[synapse_idx];
        let weight = weights[synapse_idx];
        let conductance = conductances[synapse_idx];
        let synapse_type = synapse_types[synapse_idx];
        
        // Calculate contribution (weight × conductance × sign)
        var contribution = f32(weight) * f32(conductance);
        
        // Apply sign (0 = excitatory = +1, 1 = inhibitory = -1)
        if (synapse_type != 0u) {
            contribution = -contribution;
        }
        
        // Convert to fixed-point integer (multiply by 1000 for precision)
        let contribution_fixed = i32(contribution * 1000.0);
        
        // Atomic accumulate to target neuron
        // Using atomics allows multiple fired neurons to contribute simultaneously
        atomicAdd(&membrane_potentials[target_neuron], contribution_fixed);
    }
}

// Post-processing: Convert fixed-point back to float
@compute @workgroup_size(256)
fn convert_fixed_to_float(
    @builtin(global_invocation_id) global_id: vec3<u32>,
) {
    let neuron_id = global_id.x;
    
    // Note: This would need neuron_count from a separate uniform
    // For now, this is a placeholder showing the conversion logic
    
    // Load atomic value
    let fixed_value = atomicLoad(&membrane_potentials[neuron_id]);
    
    // Convert back to float (divide by 1000)
    let float_value = f32(fixed_value) / 1000.0;
    
    // Store result (would need a separate f32 buffer)
    // membrane_potentials_float[neuron_id] = float_value;
}

// Alternative approach: Process by synapse (not by fired neuron)
// This can be more efficient for sparse firing patterns
@compute @workgroup_size(256)
fn synaptic_propagation_by_synapse(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let synapse_idx = global_id.x;
    
    // Bounds check
    if (synapse_idx >= params.synapse_count) {
        return;
    }
    
    // Check if synapse is valid
    if (!is_synapse_valid(synapse_idx)) {
        return;
    }
    
    // Get synapse properties
    let source_neuron = source_neurons[synapse_idx];
    let target_neuron = target_neurons[synapse_idx];
    
    // Check if source neuron fired
    // (Would need a fired_mask bitfield for efficient checking)
    // For now, skip this optimization
    
    // Calculate and accumulate contribution
    let weight = weights[synapse_idx];
    let conductance = conductances[synapse_idx];
    let synapse_type = synapse_types[synapse_idx];
    
    var contribution = f32(weight) * f32(conductance);
    if (synapse_type != 0u) {
        contribution = -contribution;
    }
    
    let contribution_fixed = i32(contribution * 1000.0);
    atomicAdd(&membrane_potentials[target_neuron], contribution_fixed);
}

