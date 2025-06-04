"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
Integration module for optimized FEAGI data structures.

This module provides functions to integrate the optimized data structures
into the existing FEAGI codebase, with automatic fallback to standard Python
implementations when optimized versions are not available.
"""

import logging
from typing import Dict, List, Optional, Set, Tuple, Union, Any

# Try to import optimized structures
try:
    from feagi.npu.optimized_structures import (
        FireCandidateList,
        Connectome,
        OptimizedFeagiCore,
        RUST_AVAILABLE,
    )
    from feagi.bdu.models.neuron import NeuronArray
except ImportError:
    RUST_AVAILABLE = False
    logging.warning("Optimized structures not available. Using standard implementations.")

def create_optimized_core(
    neuron_count: int,
    estimated_connections: int = 1000000,
    use_optimized: bool = True,
) -> Union["OptimizedFeagiCore", Dict[str, Any]]:
    """
    Create an optimized FEAGI core or a compatible dict-based structure.
    
    Args:
        neuron_count: Number of neurons to support
        estimated_connections: Estimated number of synaptic connections
        use_optimized: Whether to use optimized structures if available
        
    Returns:
        Either an OptimizedFeagiCore instance or a dict with compatible structures
    """
    if RUST_AVAILABLE and use_optimized:
        return OptimizedFeagiCore(neuron_count, estimated_connections)
    
    # Fallback to unified NeuronArray with enhanced optimizations
    return {
        "gna": NeuronArray(neuron_count),  # ✅ Use unified enhanced NeuronArray
        "fcl": FireCandidateList(),
        "connectome": Connectome(neuron_count, estimated_connections),
        "current_timestep": 0,
    }

def get_core_property(
    core: Union["OptimizedFeagiCore", Dict[str, Any]], 
    property_name: str
) -> Any:
    """
    Get a property from the core, regardless of implementation.
    
    Args:
        core: The core object (optimized or dict-based)
        property_name: Name of the property to get
        
    Returns:
        The requested property value
    """
    if RUST_AVAILABLE and not isinstance(core, dict):
        if property_name == "current_timestep":
            return core.current_timestep
        elif property_name == "gna":
            return core._rust_core.get_gna()
        elif property_name == "fcl":
            return core._rust_core.get_fcl()
        else:
            raise AttributeError(f"Unknown property: {property_name}")
    else:
        return core[property_name]

def set_core_property(
    core: Union["OptimizedFeagiCore", Dict[str, Any]],
    property_name: str,
    value: Any
) -> None:
    """
    Set a property on the core, regardless of implementation.
    
    Args:
        core: The core object (optimized or dict-based)
        property_name: Name of the property to set
        value: Value to set
    """
    if RUST_AVAILABLE and not isinstance(core, dict):
        if property_name == "current_timestep":
            core.current_timestep = value
        else:
            raise AttributeError(f"Cannot set property: {property_name}")
    else:
        core[property_name] = value

def step_simulation(
    core: Union["OptimizedFeagiCore", Dict[str, Any]]
) -> None:
    """
    Step the simulation forward by one timestep.
    
    Args:
        core: The core object (optimized or dict-based)
    """
    if RUST_AVAILABLE and not isinstance(core, dict):
        core.step()
    else:
        gna = core["gna"]
        fcl = core["fcl"]
        
        # 1. Decay membrane potentials using SIMD-optimized method
        gna.simd_optimized_update_membrane_potentials(0.95)
        
        # 2. Update refractory counters (if available)
        if hasattr(gna, 'update_refractory_counters'):
            gna.update_refractory_counters()
        
        # 3. Find neurons ready to fire using SIMD-optimized method
        fire_candidates = gna.simd_optimized_find_fire_candidates(core["current_timestep"])
        
        # 4. Update FCL
        fcl.clear()
        fcl.add_multiple(fire_candidates)
        
        # 5. Process fired neurons (use decay_and_check_firing if available)
        if hasattr(gna, 'decay_and_check_firing'):
            gna.decay_and_check_firing()
        
        # Increment timestep
        core["current_timestep"] += 1

def step_simulation_with_fire_queue(
    core: Union["OptimizedFeagiCore", Dict[str, Any]],
    mpf: bool = True,
    puf: bool = False,
    max_consecutive_fires: int = 10
) -> None:
    """
    Step the simulation forward using the fire queue process with PSP calculation.
    
    Args:
        core: The core object (optimized or dict-based)
        mpf: Membrane Potential Driven PSP Flag - If True, use membrane potential for PSP calculation
        puf: PSP Uniformity Flag - If True, don't normalize by synapse count
        max_consecutive_fires: Maximum consecutive fire count before inhibiting firing
    """
    if RUST_AVAILABLE and not isinstance(core, dict) and hasattr(core, '_rust_core'):
        core._rust_core.step_with_fire_queue(mpf, puf, max_consecutive_fires)
    else:
        # Fallback Python implementation
        if isinstance(core, dict):
            gna = core["gna"]
            fcl = core["fcl"]
            connectome = core["connectome"]
        else:
            # Handle OptimizedFeagiCore object
            gna = core.gna
            fcl = core.fcl
            connectome = core.connectome
        
        # Get current FCL neurons - all FCL objects support to_list()
        current_fcl = fcl.to_list()
        
        # 1. Process each firing neuron
        for neuron_id in current_fcl:
            # Update source neuron parameters
            # a. Reset membrane potential
            gna.set_membrane_potential(neuron_id, 0.0)
            
            # b. Set refractory counter (handled in process_fired_neurons)
            
            # Note: Consecutive fire count tracking would be added here in a full implementation
        
        # Process fired neurons (handles refractory period)
        if isinstance(core, dict):
            gna.process_fired_neurons(current_fcl, core["current_timestep"])
        else:
            gna.process_fired_neurons(current_fcl, core.current_timestep)
        
        # 2. Create fire queue
        fire_queue = {
            "neuron_ids": [],
            "membrane_potentials": [],
            "thresholds": [],
            "consecutive_fire_counts": [],
            "refractory_counters": []
        }
        
        # 3. For each firing neuron, process outgoing synapses
        for source_id in current_fcl:
            # Get outgoing connections
            connections = connectome.get_connections_for_neuron(source_id)
            
            if not connections:
                continue
                
            # Get firing neuron membrane potential (should be 0 now, but using original logic)
            firing_neuron_mp = gna.get_membrane_potential(source_id)
            
            # Default PSP value
            firing_neuron_psp = 1.0
            
            # Count of synapses
            synapse_count = len(connections)
            
            # Choose numerator based on MPF flag
            numerator = firing_neuron_mp if mpf else firing_neuron_psp
            
            # Calculate denominator based on PUF flag
            denominator = 1.0 if puf else ((synapse_count - 1) + 1.0)
            
            # Process each outgoing connection
            for conn in connections:
                target_id = conn["target_id"] if isinstance(conn, dict) else conn.get("target_id")
                synapse_conductance = conn["weight"] if isinstance(conn, dict) else conn.get("weight")
                
                # Calculate PSP: (numerator / denominator) * synapse_conductance
                psp = (numerator / denominator) * synapse_conductance
                
                # Get current target membrane potential
                current_mp = gna.get_membrane_potential(target_id)
                
                # Update membrane potential
                updated_mp = current_mp + psp
                
                # Add to fire queue
                fire_queue["neuron_ids"].append(target_id)
                fire_queue["membrane_potentials"].append(updated_mp)
                fire_queue["thresholds"].append(1.0)  # Default threshold
                fire_queue["consecutive_fire_counts"].append(0)  # Placeholder value
                fire_queue["refractory_counters"].append(0)  # Placeholder value
        
        # 4. Extract firing candidates from queue
        new_fire_candidates = []
        for i in range(len(fire_queue["neuron_ids"])):
            neuron_id = fire_queue["neuron_ids"][i]
            
            # Skip neurons in refractory period
            if fire_queue["refractory_counters"][i] > 0:
                continue
                
            # Skip neurons exceeding consecutive fire limit
            if max_consecutive_fires > 0 and fire_queue["consecutive_fire_counts"][i] >= max_consecutive_fires:
                continue
                
            # Check if above threshold
            if fire_queue["membrane_potentials"][i] >= fire_queue["thresholds"][i]:
                new_fire_candidates.append(neuron_id)
        
        # 5. Update the FCL
        fcl.clear()
        if isinstance(core, dict):
            fcl.add_multiple(new_fire_candidates)
        else:
            fcl.add_multiple(new_fire_candidates)
        
        # 6. Update membrane potentials for all neurons in fire queue
        for i in range(len(fire_queue["neuron_ids"])):
            neuron_id = fire_queue["neuron_ids"][i]
            
            # Skip neurons that will fire (their potentials will be reset)
            if neuron_id in new_fire_candidates:
                continue
                
            # Update membrane potential
            gna.set_membrane_potential(neuron_id, fire_queue["membrane_potentials"][i])
        
        # 7. Increment timestep
        if isinstance(core, dict):
            core["current_timestep"] += 1
        else:
            core.current_timestep += 1

def propagate_activations(
    core: Union["OptimizedFeagiCore", Dict[str, Any]]
) -> List[float]:
    """
    Propagate activations through the network.
    
    Args:
        core: The core object (optimized or dict-based)
        
    Returns:
        Buffer of activation values for each neuron
    """
    if RUST_AVAILABLE and not isinstance(core, dict):
        return core.propagate_activations()
    else:
        gna = core["gna"]
        fcl = core["fcl"]
        connectome = core["connectome"]
        
        # Create activations array (1.0 for fired neurons)
        activations = [0.0] * gna.capacity
        for neuron_id in fcl:
            activations[neuron_id] = 1.0
        
        # Create target buffer
        target_buffer = [0.0] * gna.capacity
        
        # Propagate activations
        return connectome.propagate_activations(activations, target_buffer)

def add_connection(
    core: Union["OptimizedFeagiCore", Dict[str, Any]],
    source_id: int,
    target_id: int,
    weight: float
) -> None:
    """
    Add a connection between two neurons in the optimized core.
    
    Args:
        core: The core object (optimized or dict-based)
        source_id: ID of the source (pre-synaptic) neuron
        target_id: ID of the target (post-synaptic) neuron
        weight: Synaptic weight
    """
    if RUST_AVAILABLE and not isinstance(core, dict):
        core.connectome.add_connection(source_id, target_id, weight)
    else:
        # Fallback Python implementation
        connectome = core["connectome"]
        connectome.add_connection(source_id, target_id, weight)

def get_membrane_potential(
    core: Union["OptimizedFeagiCore", Dict[str, Any]],
    neuron_id: int,
) -> float:
    """
    Get the membrane potential of a neuron.
    
    Args:
        core: The core object (optimized or dict-based)
        neuron_id: ID of the neuron
        
    Returns:
        The membrane potential value
    """
    if RUST_AVAILABLE and not isinstance(core, dict):
        return core.get_membrane_potential(neuron_id)
    else:
        gna = core["gna"]
        # Use NeuronArray API
        return gna.get_neuron_property(neuron_id, "membrane_potential")

def set_membrane_potential(
    core: Union["OptimizedFeagiCore", Dict[str, Any]],
    neuron_id: int,
    value: float,
) -> None:
    """
    Set the membrane potential of a neuron.
    
    Args:
        core: The core object (optimized or dict-based)
        neuron_id: ID of the neuron
        value: New membrane potential value
    """
    if RUST_AVAILABLE and not isinstance(core, dict):
        core.set_membrane_potential(neuron_id, value)
    else:
        gna = core["gna"]
        # Use NeuronArray API
        gna.set_neuron_property(neuron_id, "membrane_potential", value) 