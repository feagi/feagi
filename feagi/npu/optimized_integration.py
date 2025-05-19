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
        GlobalNeuronArray,
        FireCandidateList,
        Connectome,
        OptimizedFeagiCore,
        RUST_AVAILABLE,
    )
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
    
    # Fallback to standard Python structures that mimic the optimized API
    from feagi.npu.optimized_structures import (
        GlobalNeuronArray,
        FireCandidateList,
        Connectome,
    )
    
    return {
        "gna": GlobalNeuronArray(neuron_count),
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
        
        # 1. Decay membrane potentials
        gna.update_membrane_potentials(0.95)
        
        # 2. Update refractory counters
        gna.update_refractory_counters()
        
        # 3. Find neurons ready to fire
        fire_candidates = gna.find_fire_candidates(core["current_timestep"])
        
        # 4. Update FCL
        fcl.clear()
        fcl.add_multiple(fire_candidates)
        
        # 5. Process fired neurons
        gna.process_fired_neurons(fire_candidates, core["current_timestep"])
        
        # Increment timestep
        core["current_timestep"] += 1

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
    weight: float,
    delay: int = 0,
    connection_type: int = 0,
    source_area_id: int = 0,
    target_area_id: int = 0,
) -> None:
    """
    Add a connection to the connectome.
    
    Args:
        core: The core object
        source_id: ID of the source neuron
        target_id: ID of the target neuron
        weight: Synaptic weight
        delay: Synaptic delay (in timesteps)
        connection_type: Type of connection
        source_area_id: ID of the source cortical area
        target_area_id: ID of the target cortical area
    """
    if RUST_AVAILABLE and not isinstance(core, dict):
        # We don't have direct access to the connectome in the optimized core,
        # so we need to create a temporary connectome object
        connectome = Connectome(core._rust_core.get_neuron_count())
        connectome.add_connection(
            source_id,
            target_id,
            weight,
            delay,
            connection_type,
            source_area_id,
            target_area_id,
        )
    else:
        core["connectome"].add_connection(
            source_id,
            target_id,
            weight,
            delay,
            connection_type,
            source_area_id,
            target_area_id,
        )

def get_membrane_potential(
    core: Union["OptimizedFeagiCore", Dict[str, Any]],
    neuron_id: int,
) -> float:
    """
    Get the membrane potential of a neuron.
    
    Args:
        core: The core object
        neuron_id: ID of the neuron
        
    Returns:
        Membrane potential value
    """
    if RUST_AVAILABLE and not isinstance(core, dict):
        return core._rust_core.get_gna().get_membrane_potential(neuron_id)
    else:
        return core["gna"].get_membrane_potential(neuron_id)

def set_membrane_potential(
    core: Union["OptimizedFeagiCore", Dict[str, Any]],
    neuron_id: int,
    value: float,
) -> None:
    """
    Set the membrane potential of a neuron.
    
    Args:
        core: The core object
        neuron_id: ID of the neuron
        value: Membrane potential value to set
    """
    if RUST_AVAILABLE and not isinstance(core, dict):
        core._rust_core.get_gna().set_membrane_potential(neuron_id, value)
    else:
        core["gna"].set_membrane_potential(neuron_id, value) 