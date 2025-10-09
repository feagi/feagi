// Neural Dynamics Compute Shader (WGSL)
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

// Neuron state buffers (Structure-of-Arrays)
@group(0) @binding(0) var<storage, read_write> membrane_potentials: array<f32>;
@group(0) @binding(1) var<storage, read> thresholds: array<f32>;
@group(0) @binding(2) var<storage, read> leak_coefficients: array<f32>;
@group(0) @binding(3) var<storage, read> resting_potentials: array<f32>;
@group(0) @binding(4) var<storage, read> refractory_periods: array<u32>;
@group(0) @binding(5) var<storage, read_write> refractory_countdowns: array<u32>;
@group(0) @binding(6) var<storage, read> excitabilities: array<f32>;
@group(0) @binding(7) var<storage, read_write> consecutive_fire_counts: array<u32>;
@group(0) @binding(8) var<storage, read> consecutive_fire_limits: array<u32>;
@group(0) @binding(9) var<storage, read> snooze_periods: array<u32>;  // Extended refractory (additive)
@group(0) @binding(10) var<storage, read> valid_mask: array<u32>;  // Bitpacked

// Output: Fired neurons (1 = fired, 0 = not fired)
@group(0) @binding(11) var<storage, read_write> fired_mask: array<u32>;  // Bitpacked

// Constants
@group(0) @binding(12) var<storage, read> params: NeuralParams;

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
    
    // 1. Apply leak toward resting potential
    let current_potential = membrane_potentials[neuron_id];
    let leak_coef = leak_coefficients[neuron_id];
    let resting = resting_potentials[neuron_id];
    
    var new_potential = current_potential * (1.0 - leak_coef) + resting * leak_coef;
    
    // 2. Update unified refractory countdown (handles both normal and extended)
    var refractory_countdown = refractory_countdowns[neuron_id];
    if (refractory_countdown > 0u) {
        refractory_countdowns[neuron_id] = refractory_countdown - 1u;
        
        // Check if extended refractory just expired → reset consecutive fire count
        let consecutive_fires = consecutive_fire_counts[neuron_id];
        let consecutive_limit = consecutive_fire_limits[neuron_id];
        if (refractory_countdown == 1u && consecutive_limit > 0u && consecutive_fires >= consecutive_limit) {
            // Reset happens when countdown expires (Option A logic)
            consecutive_fire_counts[neuron_id] = 0u;
        }
        
        membrane_potentials[neuron_id] = new_potential;
        return;  // In refractory period, cannot fire
    }
    
    // 3. Check firing threshold
    let threshold = thresholds[neuron_id];
    if (new_potential < threshold) {
        membrane_potentials[neuron_id] = new_potential;
        return;  // Below threshold
    }
    
    // 4. Apply probabilistic excitability
    let excitability = excitabilities[neuron_id];
    if (excitability < 1.0) {
        let random_val = excitability_random(neuron_id, params.burst_count);
        if (random_val > excitability) {
            membrane_potentials[neuron_id] = new_potential;
            return;  // Failed excitability check
        }
    }
    
    // NEURON FIRES!
    
    // 5. Reset membrane potential
    membrane_potentials[neuron_id] = 0.0;
    
    // 6. Update consecutive fire count
    var consecutive_fires = consecutive_fire_counts[neuron_id] + 1u;
    consecutive_fire_counts[neuron_id] = consecutive_fires;
    
    // 7. Apply refractory period (additive if hit consecutive fire limit)
    let refractory_period = refractory_periods[neuron_id];
    let consecutive_limit = consecutive_fire_limits[neuron_id];
    
    if (consecutive_limit > 0u && consecutive_fires >= consecutive_limit) {
        // Hit burst limit → ADDITIVE extended refractory
        let snooze_period = snooze_periods[neuron_id];
        refractory_countdowns[neuron_id] = refractory_period + snooze_period;
        // Note: consecutive_fire_count will be reset when countdown expires
    } else {
        // Normal fire → normal refractory only
        refractory_countdowns[neuron_id] = refractory_period;
    }
    
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

