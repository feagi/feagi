// WebGPU shader for connectome operations
// Optimized for sparse matrix format (CSR) used by our CPU implementation

// Simulation parameters
struct SimParams {
    // Time parameters
    time_step: u32,
    
    // Neuron activation parameters
    threshold: f32,
    refractory_period: u32,
    max_consecutive_fires: u32,
    
    // PSP calculation flags
    mpf: u32, // Membrane Potential Driven PSP Flag (1 = true, 0 = false)
    puf: u32, // PSP Uniformity Flag (1 = true, 0 = false)
}

// Global Neuron Array (GNA)
struct GNA {
    neuron_count: u32,
    membrane_potentials: array<f32>,
    thresholds: array<f32>,
    refractory_counters: array<u32>,
    consecutive_fire_counts: array<u32>,
    last_fired: array<u32>,
}

// Fire Candidate List (FCL)
struct FCL {
    neuron_count: u32,
    neuron_ids: array<u32>,
}

// Fire Queue
struct FireQueue {
    count: u32,
    neuron_ids: array<u32>,
    membrane_potentials: array<f32>,
    thresholds: array<f32>,
    refractory_counters: array<u32>,
    consecutive_fire_counts: array<u32>,
}

// Connectome (CSR format)
struct Connectome {
    neuron_count: u32,
    edge_count: u32,
    row_ptr: array<u32>,
    col_idx: array<u32>,
    weights: array<f32>,
}

// Original struct definitions for backward compatibility
struct ConnectomeData {
    neuron_count: u32,
    fcl_count: u32,
    padding1: u32,
    padding2: u32,
};

// Original bindings for backward compatibility
@group(0) @binding(0) var<uniform> control: ConnectomeData;
@group(0) @binding(1) var<storage, read> row_ptr: array<u32>; // CSR row pointers
@group(0) @binding(2) var<storage, read> col_idx: array<u32>; // CSR column indices
@group(0) @binding(3) var<storage, read> weights: array<f32>; // CSR weights
@group(0) @binding(4) var<storage, read> fire_candidates: array<u32>; // FCL
@group(0) @binding(5) var<storage, read_write> membrane_potentials: array<f32>; // Target membrane potentials

// New bindings for fire queue approach
@group(1) @binding(0) var<uniform> params: SimParams;
@group(1) @binding(1) var<storage, read_write> gna: GNA;
@group(1) @binding(2) var<storage, read_write> fcl: FCL;
@group(1) @binding(3) var<storage, read> connectome: Connectome;
@group(1) @binding(4) var<storage, read_write> fire_queue: FireQueue;

// Calculate PSP based on the formula
fn calculate_psp(firing_neuron_mp: f32, firing_neuron_psp: f32, synapse_count: u32, 
                synapse_conductance: f32, mpf: bool, puf: bool) -> f32 {
    // FNPSP = [MPF * FNMP + !MPF * FNPSP] / [(!PUF * (FNSC-1)) + 1] * Synapse Conductance
    
    // Calculate the numerator: either use membrane potential or existing PSP
    let numerator = select(firing_neuron_psp, firing_neuron_mp, mpf);
    
    // Calculate the denominator: either normalize by synapse count or use 1
    let denominator = select(1.0, f32(synapse_count - 1u) + 1.0, !puf && synapse_count > 0u);
    
    // Calculate the final PSP
    return (numerator / denominator) * synapse_conductance;
}

// Original compute shader for backward compatibility
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

// Second phase: process connectome with sparse activation pattern (original)
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

// New compute shader for propagation with fire queue approach
@compute @workgroup_size(256)
fn propagate_to_fire_queue(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let fcl_index = global_id.x;
    
    // Skip if out of range
    if (fcl_index >= fcl.neuron_count) {
        return;
    }
    
    // Get neuron ID from FCL
    let neuron_id = fcl.neuron_ids[fcl_index];
    
    // Skip if invalid
    if (neuron_id >= gna.neuron_count) {
        return;
    }
    
    // Reset membrane potential
    gna.membrane_potentials[neuron_id] = 0.0;
    
    // Set refractory counter
    gna.refractory_counters[neuron_id] = params.refractory_period;
    
    // Update last fired timestamp
    gna.last_fired[neuron_id] = params.time_step;
    
    // Increment consecutive fire count
    gna.consecutive_fire_counts[neuron_id] += 1u;
    
    // Get outgoing connections
    let start = connectome.row_ptr[neuron_id];
    let end = connectome.row_ptr[neuron_id + 1u];
    let synapse_count = end - start;
    
    // Skip if no connections
    if (synapse_count == 0u) {
        return;
    }
    
    // Get firing neuron membrane potential (should be 0 now)
    let firing_neuron_mp = gna.membrane_potentials[neuron_id];
    
    // Default PSP value
    let firing_neuron_psp = 1.0;
    
    // MPF and PUF flags from parameters
    let mpf_flag = params.mpf != 0u;
    let puf_flag = params.puf != 0u;
    
    // Process each outgoing connection
    for (var i = start; i < end; i++) {
        let target_id = connectome.col_idx[i];
        
        // Skip if invalid target
        if (target_id >= gna.neuron_count) {
            continue;
        }
        
        let weight = connectome.weights[i];
        
        // Calculate PSP
        let psp = calculate_psp(
            firing_neuron_mp,
            firing_neuron_psp,
            synapse_count,
            weight,
            mpf_flag,
            puf_flag
        );
        
        // Update membrane potential
        let current_mp = gna.membrane_potentials[target_id];
        let updated_mp = current_mp + psp;
        
        // Add to fire queue
        let queue_index = atomicAdd(&fire_queue.count, 1u);
        fire_queue.neuron_ids[queue_index] = target_id;
        fire_queue.membrane_potentials[queue_index] = updated_mp;
        fire_queue.thresholds[queue_index] = gna.thresholds[target_id];
        fire_queue.refractory_counters[queue_index] = gna.refractory_counters[target_id];
        fire_queue.consecutive_fire_counts[queue_index] = gna.consecutive_fire_counts[target_id];
    }
}

// Compute shader for extract firing candidates from fire queue
@compute @workgroup_size(256)
fn process_fire_queue(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let index = global_id.x;
    
    // Skip if out of range
    if (index >= fire_queue.count) {
        return;
    }
    
    // Get neuron details from fire queue
    let neuron_id = fire_queue.neuron_ids[index];
    let membrane_potential = fire_queue.membrane_potentials[index];
    let threshold = fire_queue.thresholds[index];
    let refractory_counter = fire_queue.refractory_counters[index];
    let consecutive_fire_count = fire_queue.consecutive_fire_counts[index];
    
    // Skip neurons in refractory period
    if (refractory_counter > 0u) {
        return;
    }
    
    // Skip neurons exceeding consecutive fire limit (if enabled)
    if (params.max_consecutive_fires > 0u && consecutive_fire_count >= params.max_consecutive_fires) {
        return;
    }
    
    // Check if neuron should fire
    if (membrane_potential >= threshold) {
        // Add to FCL
        let fcl_index = atomicAdd(&fcl.neuron_count, 1u);
        fcl.neuron_ids[fcl_index] = neuron_id;
    } else {
        // Update membrane potential in GNA
        gna.membrane_potentials[neuron_id] = membrane_potential;
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