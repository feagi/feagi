mod gna;
mod fcl;
mod connectome;

pub use gna::{GlobalNeuronArray, GpuNeuronData};
pub use fcl::{FireCandidateList, FclOperation, GpuFclData};
pub use connectome::{Connectome, ConnectionsView, GpuConnectomeData};

/// Structure containing all the core data structures for FEAGI
/// 
/// Optimized for both SIMD operations on CPU and WebGPU processing
pub struct FeagiCore {
    /// Global neuron array containing all neuron state
    pub gna: GlobalNeuronArray,
    
    /// Current fire candidate list
    pub fcl: FireCandidateList,
    
    /// Connectome containing all synaptic connections
    pub connectome: Connectome,
    
    /// Current simulation timestep
    pub current_timestep: u64,
}

impl FeagiCore {
    /// Creates a new FEAGI core with the specified capacity
    pub fn new(neuron_capacity: usize, estimated_connections: usize) -> Self {
        Self {
            gna: GlobalNeuronArray::new(neuron_capacity),
            fcl: FireCandidateList::new(),
            connectome: Connectome::new(neuron_capacity, estimated_connections),
            current_timestep: 0,
        }
    }
    
    /// Performs a single simulation timestep
    pub fn step(&mut self) {
        // 1. Decay membrane potentials
        self.gna.update_membrane_potentials_simd(0.95); // Example decay factor
        
        // 2. Update refractory counters
        self.gna.update_refractory_counters();
        
        // 3. Find neurons ready to fire
        let fire_candidates = self.gna.find_fire_candidates(self.current_timestep as u32);
        
        // 4. Update the FCL
        self.fcl = FireCandidateList::from_neuron_ids(&fire_candidates);
        
        // 5. Process fired neurons (reset membrane potential, set refractory period)
        self.gna.process_fired_neurons(&fire_candidates, self.current_timestep as u32);
        
        // Increment timestep
        self.current_timestep += 1;
    }
    
    /// Propagates activations from fired neurons through the connectome
    pub fn propagate_activations(&self, target_buffer: &mut [f32]) {
        // Create a temporary buffer of activations (1.0 for fired neurons, 0.0 otherwise)
        let mut activations = vec![0.0f32; self.gna.neuron_count];
        
        // Set activations for fired neurons
        for &neuron_id in self.fcl.iter() {
            if (neuron_id as usize) < activations.len() {
                activations[neuron_id as usize] = 1.0;
            }
        }
        
        // Propagate through the connectome
        self.connectome.propagate_activations_simd(&activations, target_buffer);
    }
} 