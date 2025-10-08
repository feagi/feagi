// Neural Dynamics Compute Shader (WGSL)
// 
// Processes neural dynamics for all neurons in parallel on GPU.
// 
// Algorithm:
// 1. Apply leak toward resting potential
// 2. Check refractory period
// 3. Check firing threshold
// 4. Apply probabilistic excitability
// 5. Update consecutive fire counts
// 6. Handle snooze periods

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
@group(0) @binding(9) var<storage, read> snooze_periods: array<u32>;
@group(0) @binding(10) var<storage, read_write> snooze_countdowns: array<u32>;
@group(0) @binding(11) var<storage, read> valid_mask: array<u32>;  // Bitpacked

// Output: Fired neurons (1 = fired, 0 = not fired)
@group(0) @binding(12) var<storage, read_write> fired_mask: array<u32>;  // Bitpacked

// Constants
@group(0) @binding(13) var<storage, read> params: NeuralParams;

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

// Check if bit is set in bitpacked array
fn is_bit_set(array: ptr<storage, array<u32>, read>, index: u32) -> bool {
    let word_idx = index / 32u;
    let bit_idx = index % 32u;
    return (array[word_idx] & (1u << bit_idx)) != 0u;
}

// Set bit in bitpacked array
fn set_bit(array: ptr<storage, array<u32>, read_write>, index: u32) {
    let word_idx = index / 32u;
    let bit_idx = index % 32u;
    array[word_idx] = array[word_idx] | (1u << bit_idx);
}

// Clear bit in bitpacked array
fn clear_bit(array: ptr<storage, array<u32>, read_write>, index: u32) {
    let word_idx = index / 32u;
    let bit_idx = index % 32u;
    array[word_idx] = array[word_idx] & ~(1u << bit_idx);
}

@compute @workgroup_size(256)
fn neural_dynamics_main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let neuron_id = global_id.x;
    
    // Bounds check
    if (neuron_id >= params.neuron_count) {
        return;
    }
    
    // Check if neuron is valid
    if (!is_bit_set(&valid_mask, neuron_id)) {
        return;
    }
    
    // Clear fired mask for this neuron
    clear_bit(&fired_mask, neuron_id);
    
    // 1. Apply leak toward resting potential
    let current_potential = membrane_potentials[neuron_id];
    let leak_coef = leak_coefficients[neuron_id];
    let resting = resting_potentials[neuron_id];
    
    var new_potential = current_potential * (1.0 - leak_coef) + resting * leak_coef;
    
    // 2. Update refractory countdown
    var refractory_countdown = refractory_countdowns[neuron_id];
    if (refractory_countdown > 0u) {
        refractory_countdowns[neuron_id] = refractory_countdown - 1u;
        membrane_potentials[neuron_id] = new_potential;
        return;  // In refractory period, cannot fire
    }
    
    // 3. Update snooze countdown
    var snooze_countdown = snooze_countdowns[neuron_id];
    if (snooze_countdown > 0u) {
        snooze_countdowns[neuron_id] = snooze_countdown - 1u;
        membrane_potentials[neuron_id] = new_potential;
        return;  // In snooze period, cannot fire
    }
    
    // 4. Check firing threshold
    let threshold = thresholds[neuron_id];
    if (new_potential < threshold) {
        membrane_potentials[neuron_id] = new_potential;
        return;  // Below threshold
    }
    
    // 5. Apply probabilistic excitability
    let excitability = excitabilities[neuron_id];
    if (excitability < 1.0) {
        let random_val = excitability_random(neuron_id, params.burst_count);
        if (random_val > excitability) {
            membrane_potentials[neuron_id] = new_potential;
            return;  // Failed excitability check
        }
    }
    
    // NEURON FIRES!
    
    // 6. Reset membrane potential
    membrane_potentials[neuron_id] = 0.0;
    
    // 7. Set refractory period
    let refractory_period = refractory_periods[neuron_id];
    refractory_countdowns[neuron_id] = refractory_period;
    
    // 8. Update consecutive fire count
    var consecutive_fires = consecutive_fire_counts[neuron_id] + 1u;
    consecutive_fire_counts[neuron_id] = consecutive_fires;
    
    // 9. Check consecutive fire limit
    let consecutive_limit = consecutive_fire_limits[neuron_id];
    if (consecutive_limit > 0u && consecutive_fires >= consecutive_limit) {
        // Enter snooze period
        let snooze_period = snooze_periods[neuron_id];
        snooze_countdowns[neuron_id] = snooze_period;
        
        // Reset consecutive fire count
        consecutive_fire_counts[neuron_id] = 0u;
    }
    
    // 10. Mark as fired
    set_bit(&fired_mask, neuron_id);
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
    
    // Check if neuron fired
    if (is_bit_set(&fired_mask, neuron_id)) {
        // Atomic append to fired_indices
        let index = atomicAdd(&fired_count, 1u);
        fired_indices[index] = neuron_id;
    }
}

