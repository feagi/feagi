#!/usr/bin/env python3
"""
Pytest tests for SynapseManager class.

Tests functionality of the SynapseManager including synapse creation,
retrieval, update, deletion, and plasticity rules.
"""

import pytest
import numpy as np
import scipy.sparse as sp
import sys
import os
from typing import List, Tuple, Dict, Any

# Add the project root to the path if needed for imports
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from feagi.bdu.synapse_manager import SynapseManager


@pytest.fixture
def synapse_manager():
    """Create a SynapseManager instance for testing."""
    max_neurons = 100
    max_synapses_per_neuron = 50
    return SynapseManager(
        max_neurons=max_neurons,
        max_synapses_per_neuron=max_synapses_per_neuron
    )


def test_init(synapse_manager):
    """Test initialization of SynapseManager."""
    max_neurons = 100
    max_synapses_per_neuron = 50
    
    assert synapse_manager.max_neurons == max_neurons
    assert synapse_manager.max_synapses_per_neuron == max_synapses_per_neuron
    assert synapse_manager.total_synapses == 0
    assert synapse_manager.plastic_synapses == 0
    
    # Check matrix dimensions
    assert synapse_manager.weights.shape == (max_neurons, max_neurons)
    assert synapse_manager.is_plastic.shape == (max_neurons, max_neurons)
    
    # Check adjacency lists
    assert len(synapse_manager.outgoing_synapses) == max_neurons
    assert len(synapse_manager.incoming_synapses) == max_neurons


def test_add_synapse_basic(synapse_manager):
    """Test basic synapse addition."""
    # Add a non-plastic synapse
    result = synapse_manager.add_synapse(0, 1, 1.5)
    assert result
    assert synapse_manager.total_synapses == 1
    assert synapse_manager.plastic_synapses == 0
    assert synapse_manager.get_synapse_weight(0, 1) == 1.5
    assert 1 in synapse_manager.outgoing_synapses[0]
    assert 0 in synapse_manager.incoming_synapses[1]


def test_add_synapse_plastic(synapse_manager):
    """Test plastic synapse addition."""
    # Add a plastic synapse
    result = synapse_manager.add_synapse(
        pre_neuron=0, 
        post_neuron=2, 
        weight=2.0, 
        is_plastic=True, 
        plasticity_coeff=1.2, 
        plasticity_decay=0.9,
        plasticity_type=1,
        activity_factor=1.1,
        scaling_exponent=1.05
    )
    assert result
    assert synapse_manager.total_synapses == 1
    assert synapse_manager.plastic_synapses == 1
    assert synapse_manager.get_synapse_weight(0, 2) == 2.0
    assert synapse_manager.is_plastic[0, 2]
    assert synapse_manager.plasticity_coeffs[0, 2] == 1.2
    assert synapse_manager.plasticity_decay[0, 2] == 0.9
    assert synapse_manager.plasticity_type[0, 2] == 1
    assert synapse_manager.activity_factor[0, 2] == 1.1
    assert synapse_manager.scaling_exponent[0, 2] == 1.05


def test_add_synapse_update_existing(synapse_manager):
    """Test updating an existing synapse."""
    # Add a synapse
    synapse_manager.add_synapse(0, 1, 1.5)
    
    # Update the same synapse
    result = synapse_manager.add_synapse(0, 1, 2.5, True, 1.1, 0.8)
    assert result
    assert synapse_manager.total_synapses == 1  # Count should remain the same
    assert synapse_manager.plastic_synapses == 1  # Now it's plastic
    assert synapse_manager.get_synapse_weight(0, 1) == 2.5  # Weight updated
    assert synapse_manager.is_plastic[0, 1]  # Now plastic


def test_add_synapse_invalid(synapse_manager):
    """Test adding synapses with invalid neuron IDs."""
    max_neurons = 100
    max_synapses_per_neuron = 50
    
    # Neuron ID out of range
    result = synapse_manager.add_synapse(max_neurons + 10, 1, 1.0)
    assert not result
    
    result = synapse_manager.add_synapse(0, max_neurons + 10, 1.0)
    assert not result
    
    # Synapse count limit
    for i in range(max_synapses_per_neuron):
        synapse_manager.add_synapse(0, i + 1, 1.0)
        
    # This should fail (exceeds max synapses per neuron)
    result = synapse_manager.add_synapse(0, max_synapses_per_neuron + 1, 1.0)
    assert not result


def test_remove_synapse(synapse_manager):
    """Test synapse removal."""
    # Add two synapses
    synapse_manager.add_synapse(0, 1, 1.5)
    synapse_manager.add_synapse(0, 2, 2.0, True, 1.1, 0.8)
    assert synapse_manager.total_synapses == 2
    
    # Remove one synapse
    result = synapse_manager.remove_synapse(0, 1)
    assert result
    assert synapse_manager.total_synapses == 1
    assert synapse_manager.get_synapse_weight(0, 1) == 0.0
    assert 1 not in synapse_manager.outgoing_synapses[0]
    
    # Remove non-existent synapse
    result = synapse_manager.remove_synapse(1, 2)
    assert not result
    
    # Remove plastic synapse
    result = synapse_manager.remove_synapse(0, 2)
    assert result
    assert synapse_manager.total_synapses == 0
    assert synapse_manager.plastic_synapses == 0


def test_get_synapse_weight(synapse_manager):
    """Test getting synapse weights."""
    max_neurons = 100
    
    # Add a synapse
    synapse_manager.add_synapse(0, 1, 1.5)
    
    # Get weight of existing synapse
    weight = synapse_manager.get_synapse_weight(0, 1)
    assert weight == 1.5
    
    # Get weight of non-existent synapse
    weight = synapse_manager.get_synapse_weight(1, 2)
    assert weight == 0.0
    
    # Get weight with invalid neuron ID
    weight = synapse_manager.get_synapse_weight(max_neurons + 10, 1)
    assert weight == 0.0


def test_get_outgoing_synapses(synapse_manager):
    """Test getting outgoing synapses."""
    max_neurons = 100
    
    # Add synapses
    synapse_manager.add_synapse(0, 1, 1.5)
    synapse_manager.add_synapse(0, 2, 2.0)
    
    # Get outgoing synapses
    synapses = synapse_manager.get_outgoing_synapses(0)
    assert len(synapses) == 2
    assert (1, 1.5) in synapses
    assert (2, 2.0) in synapses
    
    # Get outgoing synapses for neuron with no outgoing connections
    synapses = synapse_manager.get_outgoing_synapses(1)
    assert len(synapses) == 0
    
    # Get outgoing synapses with invalid neuron ID
    synapses = synapse_manager.get_outgoing_synapses(max_neurons + 10)
    assert len(synapses) == 0


def test_get_incoming_synapses(synapse_manager):
    """Test getting incoming synapses."""
    max_neurons = 100
    
    # Add synapses
    synapse_manager.add_synapse(0, 2, 1.5)
    synapse_manager.add_synapse(1, 2, 2.0)
    
    # Get incoming synapses
    synapses = synapse_manager.get_incoming_synapses(2)
    assert len(synapses) == 2
    assert (0, 1.5) in synapses
    assert (1, 2.0) in synapses
    
    # Get incoming synapses for neuron with no incoming connections
    synapses = synapse_manager.get_incoming_synapses(0)
    assert len(synapses) == 0
    
    # Get incoming synapses with invalid neuron ID
    synapses = synapse_manager.get_incoming_synapses(max_neurons + 10)
    assert len(synapses) == 0


def test_update_synapse_weight(synapse_manager):
    """Test updating synapse weights."""
    max_neurons = 100
    
    # Add a synapse
    synapse_manager.add_synapse(0, 1, 1.5)
    
    # Update weight
    result = synapse_manager.update_synapse_weight(0, 1, 3.0)
    assert result
    assert synapse_manager.get_synapse_weight(0, 1) == 3.0
    
    # Update non-existent synapse
    result = synapse_manager.update_synapse_weight(1, 2, 3.0)
    assert not result
    
    # Update with invalid neuron ID
    result = synapse_manager.update_synapse_weight(max_neurons + 10, 1, 3.0)
    assert not result


def test_compute_synaptic_input(synapse_manager):
    """Test computing synaptic input from firing neurons."""
    max_neurons = 100
    
    # Add synapses
    synapse_manager.add_synapse(0, 2, 1.5)
    synapse_manager.add_synapse(1, 2, 2.0)
    synapse_manager.add_synapse(0, 3, 3.0)
    
    # Compute input from firing neurons
    inputs = synapse_manager.compute_synaptic_input([0, 1])
    assert inputs[2] == 3.5  # 1.5 + 2.0
    assert inputs[3] == 3.0  # 3.0
    
    # Empty firing neurons list
    inputs = synapse_manager.compute_synaptic_input([])
    assert np.all(inputs == 0)
    
    # Invalid neuron ID in firing neurons
    inputs = synapse_manager.compute_synaptic_input([max_neurons + 10])
    assert np.all(inputs == 0)


def test_optimize_storage(synapse_manager):
    """Test storage format optimization."""
    # Add synapses
    synapse_manager.add_synapse(0, 1, 1.5)
    synapse_manager.add_synapse(0, 2, 2.0)
    
    # Original format should be LIL
    assert type(synapse_manager.weights) == sp.lil_matrix
    
    # Optimize storage
    synapse_manager.optimize_storage()
    
    # New format should be CSR
    assert type(synapse_manager.weights) == sp.csr_matrix
    
    # Data should remain the same
    assert synapse_manager.weights[0, 1] == 1.5
    assert synapse_manager.weights[0, 2] == 2.0


def test_prune_weak_synapses(synapse_manager):
    """Test pruning of weak synapses."""
    # Add synapses with different weights
    synapse_manager.add_synapse(0, 1, 0.05)  # Below threshold
    synapse_manager.add_synapse(0, 2, 0.15)  # Above threshold
    synapse_manager.add_synapse(1, 2, 0.08)  # Below threshold
    assert synapse_manager.total_synapses == 3
    
    # Prune weak synapses
    pruned = synapse_manager.prune_weak_synapses(threshold=0.1)
    assert pruned == 2  # Two synapses should be pruned
    assert synapse_manager.total_synapses == 1
    assert synapse_manager.get_synapse_weight(0, 1) == 0.0  # Pruned
    assert synapse_manager.get_synapse_weight(0, 2) == 0.15  # Retained
    assert synapse_manager.get_synapse_weight(1, 2) == 0.0  # Pruned


def test_update_plasticity(synapse_manager):
    """Test updating of plastic synapses."""
    # Add plastic synapses with different plasticity types
    # STP (Short-Term Plasticity)
    synapse_manager.add_synapse(
        pre_neuron=0, post_neuron=1, weight=1.0,
        is_plastic=True, plasticity_coeff=1.2, plasticity_decay=0.9,
        plasticity_type=1, activity_factor=1.0, scaling_exponent=1.0
    )
    
    # LTP/LTD (Long-Term Potentiation/Depression)
    synapse_manager.add_synapse(
        pre_neuron=0, post_neuron=2, weight=1.0,
        is_plastic=True, plasticity_coeff=0.1, plasticity_decay=0.95,
        plasticity_type=2, activity_factor=1.0, scaling_exponent=1.0
    )
    
    # Non-plastic synapse (should not change)
    synapse_manager.add_synapse(
        pre_neuron=1, post_neuron=2, weight=1.0,
        is_plastic=False
    )
    
    # Update plasticity
    synapse_manager.update_plasticity(dt=1.0)
    
    # Check weight updates
    # STP: weight * (coeff ** exponent) * activity * (decay ** dt)
    # 1.0 * (1.2 ** 1.0) * 1.0 * (0.9 ** 1.0) = 1.2 * 0.9 = 1.08
    assert pytest.approx(synapse_manager.get_synapse_weight(0, 1), 0.00001) == 1.08
    
    # LTP/LTD: weight + (coeff ** exponent) * activity * (decay ** dt) * weight
    # 1.0 + (0.1 ** 1.0) * 1.0 * (0.95 ** 1.0) * 1.0 = 1.0 + 0.1 * 0.95 = 1.095
    assert pytest.approx(synapse_manager.get_synapse_weight(0, 2), 0.00001) == 1.095
    
    # Non-plastic synapse should not change
    assert synapse_manager.get_synapse_weight(1, 2) == 1.0


def test_finalize_synapses(synapse_manager):
    """Test finalizing synapses after creation."""
    # Add synapses
    synapse_manager.add_synapse(0, 1, 1.5)
    synapse_manager.add_synapse(0, 2, 2.0, True)
    
    # Finalize synapses
    synapse_manager.finalize_synapses()
    
    # Data should remain the same but in optimized format
    assert isinstance(synapse_manager.weights, sp.csr_matrix)
    assert synapse_manager.weights[0, 1] == 1.5
    assert synapse_manager.weights[0, 2] == 2.0


def test_get_synapse_info(synapse_manager):
    """Test getting detailed information about a synapse."""
    # Add a plastic synapse
    synapse_manager.add_synapse(
        pre_neuron=0, post_neuron=1, weight=1.5,
        is_plastic=True, plasticity_coeff=1.2, plasticity_decay=0.9,
        plasticity_type=1, activity_factor=1.1, scaling_exponent=1.05
    )
    
    # Get synapse info
    info = synapse_manager.get_synapse_info(0, 1)
    assert pytest.approx(info["weight"]) == 1.5
    assert info["is_plastic"]
    assert info["plasticity_type"] == 1
    assert pytest.approx(info["plasticity_coeff"]) == 1.2
    assert pytest.approx(info["plasticity_decay"]) == 0.9
    assert pytest.approx(info["activity_factor"]) == 1.1
    assert pytest.approx(info["scaling_exponent"]) == 1.05
    
    # Get info for non-existent synapse
    info = synapse_manager.get_synapse_info(1, 2)
    assert info == {}
    
    # Get info for non-plastic synapse
    synapse_manager.add_synapse(1, 2, 2.0, is_plastic=False)
    info = synapse_manager.get_synapse_info(1, 2)
    assert pytest.approx(info["weight"]) == 2.0
    assert not info["is_plastic"]
    assert "plasticity_type" not in info


def test_resize(synapse_manager):
    """Test resizing the synapse manager to accommodate more neurons."""
    max_neurons = 100
    
    # Add synapses
    synapse_manager.add_synapse(0, 1, 1.5)
    synapse_manager.add_synapse(0, 2, 2.0, True)
    
    # Resize to larger capacity
    new_size = max_neurons * 2
    result = synapse_manager.resize(new_size)
    assert result
    assert synapse_manager.max_neurons == new_size
    
    # Original synapses should remain
    assert synapse_manager.get_synapse_weight(0, 1) == 1.5
    assert synapse_manager.get_synapse_weight(0, 2) == 2.0
    
    # Should be able to add synapses with higher neuron IDs now
    result = synapse_manager.add_synapse(max_neurons + 10, max_neurons + 20, 3.0)
    assert result
    assert synapse_manager.get_synapse_weight(max_neurons + 10, max_neurons + 20) == 3.0
    
    # Trying to resize to smaller capacity should fail
    result = synapse_manager.resize(max_neurons)
    assert not result 