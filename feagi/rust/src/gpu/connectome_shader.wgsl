// WebGPU shader for connectome operations
// Optimized for sparse matrix format (CSR) used by our CPU implementation

// Struct definitions for GPU buffers
struct ConnectomeData {
    neuron_count: u32,
    fcl_count: u32,
    padding1: u32,
    padding2: u32,
};

// Buffer bindings
@group(0) @binding(0) var<uniform> control: ConnectomeData;
@group(0) @binding(1) var<storage, read> row_ptr: array<u32>; // CSR row pointers
@group(0) @binding(2) var<storage, read> col_idx: array<u32>; // CSR column indices
@group(0) @binding(3) var<storage, read> weights: array<f32>; // CSR weights
@group(0) @binding(4) var<storage, read> fire_candidates: array<u32>; // FCL
@group(0) @binding(5) var<storage, read_write> membrane_potentials: array<f32>; // Target membrane potentials

// Process a batch of fired neurons and propagate their activations
@compute @workgroup_size(64)
fn propagate_activations(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let fcl_index = global_id.x;
    
    // Early exit if beyond FCL count
    if (fcl_index >= control.fcl_count) {
        return;
    }
    
    // Get the fired neuron ID
    let neuron_id = fire_candidates[fcl_index];
    
    // Skip invalid neuron IDs
    if (neuron_id >= control.neuron_count) {
        return;
    }
    
    // Get the range of connections for this neuron
    let start = row_ptr[neuron_id];
    let end = row_ptr[neuron_id + 1u];
    
    // Process each connection
    for (var i = start; i < end; i++) {
        let target_id = col_idx[i];
        let weight = weights[i];
        
        // Synchronization is needed for concurrent updates
        // Use atomic operations for thread safety
        let old_value = atomicLoad(&membrane_potentials[target_id]);
        let new_value = old_value + weight;
        atomicStore(&membrane_potentials[target_id], new_value);
    }
}

// Second phase: process connectome with sparse activation pattern
@compute @workgroup_size(256)
fn process_sparse_connectome(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let neuron_id = global_id.x;
    
    // Early exit if beyond neuron count
    if (neuron_id >= control.neuron_count) {
        return;
    }
    
    // Check if this neuron is in the FCL
    // This is inefficient in a real shader, used for demonstration only
    var is_fired = false;
    for (var i = 0u; i < control.fcl_count; i++) {
        if (fire_candidates[i] == neuron_id) {
            is_fired = true;
            break;
        }
    }
    
    // Skip if neuron didn't fire
    if (!is_fired) {
        return;
    }
    
    // Process connections (same as above)
    let start = row_ptr[neuron_id];
    let end = row_ptr[neuron_id + 1u];
    
    for (var i = start; i < end; i++) {
        let target_id = col_idx[i];
        let weight = weights[i];
        atomicAdd(&membrane_potentials[target_id], weight);
    }
}

// Utility functions for atomic operations on f32 values
fn atomicLoad(addr: ptr<storage, f32, read>) -> f32 {
    return *addr;
}

fn atomicStore(addr: ptr<storage, f32, read_write>, value: f32) {
    *addr = value;
}

fn atomicAdd(addr: ptr<storage, f32, read_write>, value: f32) -> f32 {
    let old_value = atomicLoad(addr);
    atomicStore(addr, old_value + value);
    return old_value;
} 