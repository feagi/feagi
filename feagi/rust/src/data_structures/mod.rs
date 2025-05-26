/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

mod gna;
mod fcl;
mod connectome;

pub use gna::{GlobalNeuronArray, GpuNeuronData, FireQueue};
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
    
    /// Performs a single simulation timestep using the fire queue process with PSP calculation
    pub fn step_with_fire_queue(&mut self, mpf: bool, puf: bool, max_consecutive_fires: u32) {
        // Get current FCL as vector
        let current_fcl = self.fcl.to_vec();
        
        // Create fire queue for potential firing candidates
        let mut fire_queue = FireQueue::new(self.gna.neuron_count);
        
        // 1. Process each firing neuron
        for &neuron_id in &current_fcl {
            let id = neuron_id as usize;
            if id >= self.gna.neuron_count {
                continue;
            }
            
            // Update source neuron parameters
            
            // a. Set membrane potential to 0
            self.gna.membrane_potentials[id] = 0.0;
            
            // b. Update refractory period
            self.gna.refractory_counters[id] = self.gna.refractory_periods[id];
            
            // c. Update last fired timestamp
            self.gna.last_fired[id] = self.current_timestep as u32;
            
            // d. Update consecutive fire count
            self.gna.update_consecutive_fire_count(id, true);
        }
        
        // 2. Propagate activation through connections with PSP calculation
        #[cfg(target_feature = "avx2")]
        {
            self.connectome.propagate_with_psp_simd(
                &current_fcl,
                &self.gna,
                &mut fire_queue,
                mpf,
                puf
            );
        }
        
        #[cfg(not(target_feature = "avx2"))]
        {
            self.connectome.propagate_with_psp(
                &current_fcl,
                &self.gna,
                &mut fire_queue,
                mpf,
                puf
            );
        }
        
        // 3. Extract firing candidates from queue applying activation function
        #[cfg(target_feature = "avx2")]
        let fire_candidates = fire_queue.extract_fire_candidates_simd(max_consecutive_fires);
        
        #[cfg(not(target_feature = "avx2"))]
        let fire_candidates = fire_queue.extract_fire_candidates(max_consecutive_fires);
        
        // 4. Update the FCL with new fire candidates
        self.fcl = FireCandidateList::from_neuron_ids(&fire_candidates);
        
        // 5. Update membrane potentials for all neurons from fire queue
        for i in 0..fire_queue.count {
            let neuron_id = fire_queue.neuron_ids[i] as usize;
            if neuron_id >= self.gna.neuron_count {
                continue;
            }
            
            // Update only if neuron is not in the fire candidates list
            if !self.fcl.contains(neuron_id as u32) {
                self.gna.membrane_potentials[neuron_id] = fire_queue.membrane_potentials[i];
            }
        }
        
        // 6. Increment timestep
        self.current_timestep += 1;
    }
    
    /// Propagates activations from fired neurons through the connectome
    pub fn propagate_activations(&self, target_buffer: &mut [f32]) {
        // Create a temporary buffer of activations (1.0 for fired neurons, 0.0 otherwise)
        let mut activations = vec![0.0f32; self.gna.neuron_count];
        
        // Set activations for fired neurons
        for neuron_id in self.fcl.iter() {
            if (neuron_id as usize) < activations.len() {
                activations[neuron_id as usize] = 1.0;
            }
        }
        
        // Propagate through the connectome
        self.connectome.propagate_activations_simd(&activations, target_buffer);
    }
} 