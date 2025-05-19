// WebGPU shader for neuron operations
// Optimized for both performance and compatibility with our CPU SIMD code

// Struct definitions for GPU buffers
struct NeuronData {
    // Control data
    neuron_count: u32,
    current_timestep: u32,
    decay_factor: f32,
    padding: f32, // Keep aligned to 16-byte boundaries
};

// Buffer bindings
@group(0) @binding(0) var<uniform> control: NeuronData;
@group(0) @binding(1) var<storage, read_write> membrane_potentials: array<f32>;
@group(0) @binding(2) var<storage, read_write> thresholds: array<f32>;
@group(0) @binding(3) var<storage, read_write> refractory_counters: array<u32>;
@group(0) @binding(4) var<storage, read_write> last_fired: array<u32>;
@group(0) @binding(5) var<storage, read_write> fire_candidates: array<atomic<u32>>; // Atomic for parallel writes
@group(0) @binding(6) var<storage, read_write> fcl_counter: array<atomic<u32>>; // Single atomic counter

// Decay membrane potentials and update refractory counters
@compute @workgroup_size(256)
fn decay_potentials(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let neuron_id = global_id.x;
    
    // Early exit if beyond neuron count
    if (neuron_id >= control.neuron_count) {
        return;
    }
    
    // Decay membrane potential
    membrane_potentials[neuron_id] *= control.decay_factor;
    
    // Update refractory counter
    if (refractory_counters[neuron_id] > 0u) {
        refractory_counters[neuron_id] -= 1u;
    }
}

// Find neurons that are ready to fire
@compute @workgroup_size(256)
fn find_fire_candidates(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let neuron_id = global_id.x;
    
    // Early exit if beyond neuron count
    if (neuron_id >= control.neuron_count) {
        return;
    }
    
    // Skip neurons in refractory period
    if (refractory_counters[neuron_id] > 0u) {
        return;
    }
    
    // Check if above threshold
    if (membrane_potentials[neuron_id] >= thresholds[neuron_id]) {
        // Add to fire candidate list
        let fcl_index = atomicAdd(&fcl_counter[0], 1u);
        fire_candidates[fcl_index] = neuron_id;
        
        // Reset membrane potential
        membrane_potentials[neuron_id] = 0.0;
        
        // Update last fired timestamp
        last_fired[neuron_id] = control.current_timestep;
    }
}

// Process fired neurons: propagate activation through connectome
@compute @workgroup_size(256)
fn propagate_activations(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let fcl_index = global_id.x;
    
    // Early exit if beyond FCL count
    if (fcl_index >= fcl_counter[0]) {
        return;
    }
    
    let neuron_id = fire_candidates[fcl_index];
    
    // Process connections for this neuron
    // In a complete implementation, this would access the connectome
    // and update target neurons' membrane potentials
}

// Utility function for atomic additions
fn atomicAdd(a: ptr<storage, atomic<u32>, read_write>, b: u32) -> u32 {
    return atomicAdd(a, b);
} 