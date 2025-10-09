// Neural Dynamics Compute Shader - FCL-Aware (Sparse Processing)
// 
// **Key Difference**: Processes ONLY Fire Candidate List neurons (sparse, ~1-10% of brain)
// 
// Algorithm:
// 1. Get FCL index from workgroup
// 2. Lookup actual neuron_id from FCL array
// 3. Apply FCL accumulated potential to membrane potential
// 4. Process neural dynamics (leak, threshold, firing)
// 5. Write sparse output (only FCL neurons in fired mask)

// ═══════════════════════════════════════════════════════════
// FCL INPUT (Sparse arrays - only candidate neurons)
// ═══════════════════════════════════════════════════════════

@group(0) @binding(0) var<storage, read> fcl_neuron_ids: array<u32>;      // Sparse neuron IDs
@group(0) @binding(1) var<storage, read> fcl_potentials: array<f32>;      // Accumulated potentials

// ═══════════════════════════════════════════════════════════
// NEURON STATE (Dense arrays - full brain, indexed by neuron_id)
// ═══════════════════════════════════════════════════════════

@group(0) @binding(2) var<storage, read_write> membrane_potentials: array<f32>;

// Interleaved f32 parameters: [threshold, leak_coef, resting, excitability, ...]
@group(0) @binding(3) var<storage, read> f32_params: array<f32>;

// Interleaved u16 dynamic state: [refrac_countdown, consec_count, ...]
@group(0) @binding(4) var<storage, read_write> u16_dynamic_state: array<u32>;

// Interleaved u16 static params: [refrac_period, consec_limit, snooze_period, ...]
@group(0) @binding(5) var<storage, read> u16_static_params: array<u32>;

// ═══════════════════════════════════════════════════════════
// OUTPUT (Sparse - only FCL neurons)
// ═══════════════════════════════════════════════════════════

@group(0) @binding(6) var<storage, read_write> fcl_fired_mask: array<u32>;  // Bitpacked sparse output

// Constants
@group(0) @binding(7) var<storage, read> params: NeuralParams;

struct NeuralParams {
    fcl_count: u32,        // Number of FCL candidates
    burst_count: u32,      // Current burst number
    _padding0: u32,
    _padding1: u32,
}

// PCG hash for pseudo-random number generation
fn pcg_hash(input: u32) -> u32 {
    var state = input * 747796405u + 2891336453u;
    let word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

fn pcg_hash_to_float(input: u32) -> f32 {
    return f32(pcg_hash(input)) / 4294967296.0;
}

fn excitability_random(neuron_id: u32, burst_count: u32) -> f32 {
    let seed = neuron_id * 2654435761u + burst_count * 1597334677u;
    return pcg_hash_to_float(seed);
}

@compute @workgroup_size(256)
fn neural_dynamics_fcl_main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let fcl_idx = global_id.x;
    
    // Bounds check: Are we within FCL count?
    if (fcl_idx >= params.fcl_count) {
        return;
    }
    
    // ═══════════════════════════════════════════════════════════
    // SPARSE LOOKUP: Get actual neuron ID from FCL
    // ═══════════════════════════════════════════════════════════
    
    let neuron_id = fcl_neuron_ids[fcl_idx];
    let fcl_potential = fcl_potentials[fcl_idx];
    
    // ═══════════════════════════════════════════════════════════
    // LOAD NEURON STATE (Random access into dense arrays)
    // ═══════════════════════════════════════════════════════════
    
    // f32_params: stride=4 (threshold, leak, resting, excitability)
    let f32_idx = neuron_id * 4u;
    let threshold = f32_params[f32_idx + 0u];
    let leak_coef = f32_params[f32_idx + 1u];
    let resting = f32_params[f32_idx + 2u];
    let excitability = f32_params[f32_idx + 3u];
    
    // u16_static_params: stride=3 (refrac_period, consec_limit, snooze_period)
    let u16_static_idx = neuron_id * 3u;
    let refractory_period = u16_static_params[u16_static_idx + 0u];
    let consecutive_limit = u16_static_params[u16_static_idx + 1u];
    let snooze_period = u16_static_params[u16_static_idx + 2u];
    
    // u16_dynamic_state: stride=2 (refrac_countdown, consec_count)
    let u16_dyn_idx = neuron_id * 2u;
    var refractory_countdown = u16_dynamic_state[u16_dyn_idx + 0u];
    var consecutive_fires = u16_dynamic_state[u16_dyn_idx + 1u];
    
    // ═══════════════════════════════════════════════════════════
    // APPLY FCL POTENTIAL TO MEMBRANE
    // ═══════════════════════════════════════════════════════════
    
    var membrane = membrane_potentials[neuron_id];
    membrane = membrane + fcl_potential;  // Add accumulated synaptic input
    
    // ═══════════════════════════════════════════════════════════
    // NEURAL DYNAMICS
    // ═══════════════════════════════════════════════════════════
    
    // Step 1: Apply leak toward resting potential
    membrane = membrane + (resting - membrane) * leak_coef;
    
    // Step 2: Check unified refractory period
    if (refractory_countdown > 0u) {
        refractory_countdown = refractory_countdown - 1u;
        u16_dynamic_state[u16_dyn_idx + 0u] = refractory_countdown;
        
        // Reset consecutive fire count when extended refractory expires
        if (refractory_countdown == 0u && consecutive_limit > 0u && consecutive_fires >= consecutive_limit) {
            consecutive_fires = 0u;
            u16_dynamic_state[u16_dyn_idx + 1u] = consecutive_fires;
        }
        
        // Update membrane and exit (neuron in refractory)
        membrane_potentials[neuron_id] = membrane;
        return;
    }
    
    // Step 3: Check firing threshold
    if (membrane < threshold) {
        // Not firing - just update membrane
        membrane_potentials[neuron_id] = membrane;
        return;
    }
    
    // Step 4: Apply probabilistic excitability
    let rand_val = excitability_random(neuron_id, params.burst_count);
    if (rand_val > excitability) {
        // Failed excitability check
        membrane_potentials[neuron_id] = membrane;
        return;
    }
    
    // ═══════════════════════════════════════════════════════════
    // NEURON FIRES!
    // ═══════════════════════════════════════════════════════════
    
    // Reset membrane potential
    membrane = resting;
    membrane_potentials[neuron_id] = membrane;
    
    // Update consecutive fire count
    consecutive_fires = consecutive_fires + 1u;
    u16_dynamic_state[u16_dyn_idx + 1u] = consecutive_fires;
    
    // Apply refractory period (additive if hit consecutive fire limit)
    if (consecutive_limit > 0u && consecutive_fires >= consecutive_limit) {
        // Extended refractory: normal + snooze
        refractory_countdown = refractory_period + snooze_period;
    } else {
        // Normal refractory
        refractory_countdown = refractory_period;
    }
    u16_dynamic_state[u16_dyn_idx + 0u] = refractory_countdown;
    
    // ═══════════════════════════════════════════════════════════
    // SPARSE OUTPUT: Mark this FCL neuron as fired
    // ═══════════════════════════════════════════════════════════
    
    // Write to sparse output array (indexed by FCL index, not neuron ID)
    let fired_word_idx = fcl_idx / 32u;
    let fired_bit_idx = fcl_idx % 32u;
    
    // Set bit in sparse fired mask
    fcl_fired_mask[fired_word_idx] = fcl_fired_mask[fired_word_idx] | (1u << fired_bit_idx);
}

