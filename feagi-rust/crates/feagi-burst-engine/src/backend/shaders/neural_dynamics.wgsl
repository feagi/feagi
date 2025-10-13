// Neural Dynamics Compute Shader (WGSL) - Optimized for Metal (≤8 bindings)
// 
// Processes neural dynamics for all neurons in parallel on GPU.
// 
// Algorithm:
// 1. Apply leak toward resting potential
// 2. Check unified refractory period (handles both normal and extended)
// 3. Check firing threshold
// 4. Apply probabilistic excitability
// 5. Update consecutive fire counts
// 6. Apply additive extended refractory when consecutive fire limit hit
//
// Buffer Layout (interleaved for Metal compatibility):
// - f32_params: [threshold, leak, resting, excitability] per neuron (stride=4)
// - u16_static: [refrac_period, consec_limit, snooze_period] per neuron (stride=3)
// - u16_dynamic: [refrac_countdown, consec_count] per neuron (stride=2)

// Neuron state buffers (consolidated for Metal's 8-binding limit)
@group(0) @binding(0) var<storage, read_write> membrane_potentials: array<f32>;

// Interleaved f32 parameters: [threshold, leak_coef, resting, excitability, ...]
@group(0) @binding(1) var<storage, read> f32_params: array<f32>;

// Interleaved u16 static params: [refrac_period, consec_limit, snooze_period, ...]
@group(0) @binding(2) var<storage, read> u16_static_params: array<u32>;

// Interleaved u16 dynamic state: [refrac_countdown, consec_count, ...]
@group(0) @binding(3) var<storage, read_write> u16_dynamic_state: array<u32>;

@group(0) @binding(4) var<storage, read> valid_mask: array<u32>;  // Bitpacked

// Output: Fired neurons (1 = fired, 0 = not fired)
@group(0) @binding(5) var<storage, read_write> fired_mask: array<u32>;  // Bitpacked

// Constants
@group(0) @binding(6) var<storage, read> params: NeuralParams;

struct NeuralParams {
    neuron_count: u32,
    burst_count: u32,
    _padding0: u32,
    _padding1: u32,
}

// PCG hash for pseudo-random number generation
fn pcg_hash(input: u32) -> u32 {
    var state = input * 747796405u + 2891336453u;
    let word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

// Convert hash to float [0, 1)
fn pcg_hash_to_float(input: u32) -> f32 {
    return f32(pcg_hash(input)) / 4294967296.0;
}

// Generate excitability random value (combines neuron_id + burst_count)
fn excitability_random(neuron_id: u32, burst_count: u32) -> f32 {
    let seed = neuron_id * 2654435761u + burst_count * 1597334677u;
    return pcg_hash_to_float(seed);
}

// Note: WGSL cannot pass storage pointers to functions
// Bit operations must be inlined in the main compute function

@compute @workgroup_size(256)
fn neural_dynamics_main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let neuron_id = global_id.x;
    
    // Bounds check
    if (neuron_id >= params.neuron_count) {
        return;
    }
    
    // Check if neuron is valid (inline bit check)
    let valid_word_idx = neuron_id / 32u;
    let valid_bit_idx = neuron_id % 32u;
    if ((valid_mask[valid_word_idx] & (1u << valid_bit_idx)) == 0u) {
        return;
    }
    
    // ═══════════════════════════════════════════════════════════
    // LOAD INTERLEAVED PARAMETERS (optimized for Metal)
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
    // NEURAL DYNAMICS LOGIC
    // ═══════════════════════════════════════════════════════════
    
    // 1. Apply leak toward resting potential
    let current_potential = membrane_potentials[neuron_id];
    var new_potential = current_potential * (1.0 - leak_coef) + resting * leak_coef;
    
    // 2. Update unified refractory countdown (handles both normal and extended)
    if (refractory_countdown > 0u) {
        refractory_countdown = refractory_countdown - 1u;
        u16_dynamic_state[u16_dyn_idx + 0u] = refractory_countdown;
        
        // Check if extended refractory just expired → reset consecutive fire count
        if (refractory_countdown == 0u && consecutive_limit > 0u && consecutive_fires >= consecutive_limit) {
            // Reset happens when countdown expires (Option A logic)
            consecutive_fires = 0u;
            u16_dynamic_state[u16_dyn_idx + 1u] = consecutive_fires;
        }
        
        membrane_potentials[neuron_id] = new_potential;
        return;  // In refractory period, cannot fire
    }
    
    // 3. Check firing threshold
    if (new_potential < threshold) {
        membrane_potentials[neuron_id] = new_potential;
        return;  // Below threshold
    }
    
    // 4. Apply probabilistic excitability
    if (excitability < 1.0) {
        let random_val = excitability_random(neuron_id, params.burst_count);
        if (random_val > excitability) {
            membrane_potentials[neuron_id] = new_potential;
            return;  // Failed excitability check
        }
    }
    
    // ═══════════════════════════════════════════════════════════
    // NEURON FIRES!
    // ═══════════════════════════════════════════════════════════
    
    // 5. Reset membrane potential
    membrane_potentials[neuron_id] = 0.0;
    
    // 6. Update consecutive fire count
    consecutive_fires = consecutive_fires + 1u;
    u16_dynamic_state[u16_dyn_idx + 1u] = consecutive_fires;
    
    // 7. Apply refractory period (additive if hit consecutive fire limit)
    if (consecutive_limit > 0u && consecutive_fires >= consecutive_limit) {
        // Hit burst limit → ADDITIVE extended refractory
        refractory_countdown = refractory_period + snooze_period;
        // Note: consecutive_fire_count will be reset when countdown expires
    } else {
        // Normal fire → normal refractory only
        refractory_countdown = refractory_period;
    }
    u16_dynamic_state[u16_dyn_idx + 0u] = refractory_countdown;
    
    // 8. Mark as fired (inline bit set)
    // Note: No need for atomic as each neuron writes to its own unique bit
    let fired_word_idx = neuron_id / 32u;
    let fired_bit_idx = neuron_id % 32u;
    fired_mask[fired_word_idx] = fired_mask[fired_word_idx] | (1u << fired_bit_idx);
}

// Helper compute shader to extract fired neuron indices
// This compacts the sparse fired_mask into a dense array of indices
@group(0) @binding(14) var<storage, read_write> fired_indices: array<u32>;
@group(0) @binding(15) var<storage, read_write> fired_count: atomic<u32>;

@compute @workgroup_size(256)
fn extract_fired_neurons(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let neuron_id = global_id.x;
    
    // Bounds check
    if (neuron_id >= params.neuron_count) {
        return;
    }
    
    // Check if neuron fired (inline bit check)
    let fired_word_idx = neuron_id / 32u;
    let fired_bit_idx = neuron_id % 32u;
    if ((fired_mask[fired_word_idx] & (1u << fired_bit_idx)) != 0u) {
        // Atomic append to fired_indices
        let index = atomicAdd(&fired_count, 1u);
        fired_indices[index] = neuron_id;
    }
}

