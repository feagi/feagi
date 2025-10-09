// Synaptic Propagation Compute Shader - FCL Output (GPU→GPU)
// **CONSOLIDATED LAYOUT - Metal-compatible (7 bindings)**
//
// Accumulates synaptic contributions into FCL buffer on GPU (no CPU roundtrip!)
//
// Algorithm:
// 1. For each fired neuron, find its outgoing synapses (hash table lookup)
// 2. Calculate synaptic contribution (weight × conductance × sign)
// 3. Atomically accumulate to FCL potential buffer
//
// Output: FCL candidates ready for neural dynamics (all on GPU)

// ═══════════════════════════════════════════════════════════
// SYNAPSE DATA (Consolidated, stride=3)
// Format: [source_id, target_id, packed_params] per synapse
// packed_params = (type << 16) | (conductance << 8) | weight
// ═══════════════════════════════════════════════════════════

@group(0) @binding(0) var<storage, read> synapse_data: array<u32>;

// ═══════════════════════════════════════════════════════════
// HASH TABLE (Consolidated)
// ═══════════════════════════════════════════════════════════

@group(0) @binding(1) var<storage, read> hash_keys: array<u32>;
@group(0) @binding(2) var<storage, read> hash_metadata: array<u32>; // Interleaved: [start, count] (stride=2)
@group(0) @binding(3) var<storage, read> synapse_list: array<u32>;

// ═══════════════════════════════════════════════════════════
// FCL OUTPUT (Sparse accumulation, per-neuron potential)
// ═══════════════════════════════════════════════════════════

@group(0) @binding(4) var<storage, read_write> fcl_potentials_atomic: array<atomic<i32>>;

// ═══════════════════════════════════════════════════════════
// INPUT (Fired neurons from previous burst)
// ═══════════════════════════════════════════════════════════

@group(0) @binding(5) var<storage, read> fired_neurons: array<u32>;

// ═══════════════════════════════════════════════════════════
// PARAMETERS
// ═══════════════════════════════════════════════════════════

@group(0) @binding(6) var<storage, read> params: SynapticParams;

struct SynapticParams {
    fired_count: u32,          // Number of fired neurons
    hash_capacity: u32,        // Hash table capacity
    _padding0: u32,
    _padding1: u32,
}

// Hash function for GPU hash table lookup
fn hash_neuron_id(neuron_id: u32, capacity: u32) -> u32 {
    let hash = neuron_id * 2654435761u;
    return hash % capacity;
}

// Find synapse index for source neuron (linear probing)
fn find_synapse_metadata(source_neuron_id: u32) -> vec2<u32> {
    let capacity = params.hash_capacity;
    var slot = hash_neuron_id(source_neuron_id, capacity);
    
    // Linear probing (max 16 probes)
    for (var probe = 0u; probe < 16u; probe = probe + 1u) {
        let key = hash_keys[slot];
        
        if (key == source_neuron_id) {
            // Found! Return [start, count] from metadata
            let meta_idx = slot * 2u;
            let start = hash_metadata[meta_idx];
            let count = hash_metadata[meta_idx + 1u];
            return vec2<u32>(start, count);
        }
        
        if (key == 0xFFFFFFFFu) {
            return vec2<u32>(0u, 0u);  // Empty slot = not found
        }
        
        slot = (slot + 1u) % capacity;
    }
    
    return vec2<u32>(0u, 0u);  // Not found after max probes
}

// Process one fired neuron → accumulate to all target neurons
@compute @workgroup_size(256)
fn synaptic_propagation_fcl_main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let fired_idx = global_id.x;
    
    // Bounds check
    if (fired_idx >= params.fired_count) {
        return;
    }
    
    // Get fired neuron ID
    let source_neuron_id = fired_neurons[fired_idx];
    
    // Find synapse metadata for this source neuron
    let metadata = find_synapse_metadata(source_neuron_id);
    let list_start = metadata.x;
    let synapse_count = metadata.y;
    
    if (synapse_count == 0u) {
        return;  // No synapses for this neuron
    }
    
    // Process all synapses from this fired neuron
    for (var i = 0u; i < synapse_count; i = i + 1u) {
        let synapse_idx = synapse_list[list_start + i];
        
        // Read consolidated synapse data (stride=3)
        let data_idx = synapse_idx * 3u;
        let source_id = synapse_data[data_idx + 0u];
        let target_id = synapse_data[data_idx + 1u];
        let packed_params = synapse_data[data_idx + 2u];
        
        // Unpack params: (type << 16) | (conductance << 8) | weight
        let weight_u8 = packed_params & 0xFFu;
        let conductance_u8 = (packed_params >> 8u) & 0xFFu;
        let synapse_type = (packed_params >> 16u) & 0xFFu;
        
        // Calculate synaptic contribution
        let weight_f32 = f32(weight_u8) / 255.0;
        let conductance_f32 = f32(conductance_u8) / 255.0;
        let sign = select(-1.0, 1.0, synapse_type == 0u);  // 0=excitatory, 1=inhibitory
        let contribution = sign * weight_f32 * conductance_f32 * 10.0;  // Scale factor
        
        // Convert to fixed-point i32 (multiply by 1000 for precision)
        let contribution_i32 = i32(contribution * 1000.0);
        
        // Atomically accumulate to FCL potential buffer
        atomicAdd(&fcl_potentials_atomic[target_id], contribution_i32);
    }
}
