// WebGPU shader for neuron operations
// Optimized for both performance and compatibility with our CPU SIMD code

// Define a struct for parameters
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

// Define the GNA structure
struct GNA {
    neuron_count: u32,
    membrane_potentials: array<f32>,
    thresholds: array<f32>,
    refractory_counters: array<u32>,
    consecutive_fire_counts: array<u32>,
    last_fired: array<u32>,
    excitability: array<f32>,  // Firing probability (0.0 to 1.0)
}

// Define the FCL structure
struct FCL {
    neuron_count: u32,
    neuron_ids: array<u32>,
}

// Define the Fire Queue structure
struct FireQueue {
    count: u32,
    neuron_ids: array<u32>,
    membrane_potentials: array<f32>,
    thresholds: array<f32>,
    refractory_counters: array<u32>,
    consecutive_fire_counts: array<u32>,
}

// Define the sparse connectome structure
struct Connectome {
    neuron_count: u32,
    edge_count: u32,
    row_ptr: array<u32>,
    col_idx: array<u32>,
    weights: array<f32>,
}

// Parameters binding
@group(0) @binding(0) var<uniform> params: SimParams;

// GNA binding
@group(0) @binding(1) var<storage, read_write> gna: GNA;

// FCL binding
@group(0) @binding(2) var<storage, read_write> fcl: FCL;

// Connectome binding
@group(0) @binding(3) var<storage, read> connectome: Connectome;

// Fire Queue binding
@group(0) @binding(4) var<storage, read_write> fire_queue: FireQueue;

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

// GPU-efficient PCG hash for random number generation
fn pcg_hash(input: u32) -> u32 {
    var state = input * 747796405u + 2891336453u;
    var word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

// Convert PCG hash to floating point [0,1)
fn hash_to_float(hash: u32) -> f32 {
    return f32(hash) / 4294967296.0;
}

// Enhanced firing function with excitability support
fn should_fire_with_excitability(neuron_id: u32, random_seed: u32) -> bool {
    let membrane_potential = gna.membrane_potentials[neuron_id];
    let threshold = gna.thresholds[neuron_id];
    let excitability = gna.excitability[neuron_id];
    
    // Standard threshold check
    if (membrane_potential < threshold) {
        return false;
    }
    
    // Fast path: if excitability is ~1.0, always fire
    if (excitability >= 0.999) {
        return true;
    }
    
    // Probabilistic firing using GPU-efficient PRNG
    let random_val = hash_to_float(pcg_hash(neuron_id + random_seed));
    return random_val < excitability;
}

// Compute shader for processing fired neurons and updating the fire queue
@compute @workgroup_size(256)
fn process_fired_neurons(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let index = global_id.x;

    // Check if this thread should process a neuron
    if (index >= fcl.neuron_count) {
        return;
    }

    // Get the neuron ID from FCL
    let neuron_id = fcl.neuron_ids[index];

    // Skip if invalid ID
    if (neuron_id >= gna.neuron_count) {
        return;
    }

    // Reset membrane potential of the fired neuron
    gna.membrane_potentials[neuron_id] = 0.0;

    // Set refractory counter
    gna.refractory_counters[neuron_id] = params.refractory_period;

    // Update last fired timestamp
    gna.last_fired[neuron_id] = params.time_step;

    // Increment consecutive fire count
    gna.consecutive_fire_counts[neuron_id] += 1u;

    // Process outgoing connections to populate the fire queue
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

        // Store updated values in buffer for atomic addition later
        // Note: In an actual implementation, we would need atomic operations to safely
        // add to the fire queue from multiple threads. This is simplified for now.
        let queue_index = atomicAdd(&fire_queue.count, 1u);
        fire_queue.neuron_ids[queue_index] = target_id;
        fire_queue.membrane_potentials[queue_index] = updated_mp;
        fire_queue.thresholds[queue_index] = gna.thresholds[target_id];
        fire_queue.refractory_counters[queue_index] = gna.refractory_counters[target_id];
        fire_queue.consecutive_fire_counts[queue_index] = gna.consecutive_fire_counts[target_id];
    }
}

// Compute shader for extracting fire candidates from the fire queue
@compute @workgroup_size(256)
fn extract_fire_candidates(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let index = global_id.x;

    // Check if this thread should process a queue entry
    if (index >= fire_queue.count) {
        return;
    }

    // Get the neuron details from the fire queue
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

// Original compute shader for finding firing neurons (kept for backward compatibility)
@compute @workgroup_size(256)
fn find_firing_neurons(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let neuron_id = global_id.x;

    // Check if this thread should process a neuron
    if (neuron_id >= gna.neuron_count) {
        return;
    }

    // Skip neurons in refractory period
    if (gna.refractory_counters[neuron_id] > 0u) {
        return;
    }

    // Check if the neuron has sufficient potential to fire
    let membrane_potential = gna.membrane_potentials[neuron_id];
    let threshold = gna.thresholds[neuron_id];

    if (membrane_potential >= threshold) {
        // Add to FCL
        let fcl_index = atomicAdd(&fcl.neuron_count, 1u);
        fcl.neuron_ids[fcl_index] = neuron_id;
    }
}

// Utility function for atomic additions
fn atomicAdd(a: ptr<storage, atomic<u32>, read_write>, b: u32) -> u32 {
    return atomicAdd(a, b);
}
