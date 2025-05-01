"""
Unit tests for ConnectomeManager.

These tests focus on verifying the correctness of ConnectomeManager functionality 
with minimal resource usage for fast execution during development.
"""

import pytest
import numpy as np
from feagi.bdu.connectome_manager import ConnectomeManager, NeuronPropertyType, CorticalArea
from feagi.utils.config import FeagiConfig


@pytest.fixture
def small_config():
    """Create a minimal FeagiConfig for testing."""
    config = FeagiConfig()
    config.set('connectome.max_neurons', 100)  # Minimal size for testing
    config.set('connectome.max_synapses_per_neuron', 10)
    config.set('connectome.fcl_window_size', 3)
    return config


@pytest.fixture
def connectome(small_config):
    """Create a ConnectomeManager with minimal capacity for fast testing."""
    connectome = ConnectomeManager(small_config)
    return connectome


@pytest.fixture
def test_area(connectome):
    """Create a small test cortical area."""
    area_id = 1
    area = connectome.add_cortical_area(
        area_id=area_id,
        name="Test Area",
        area_type="interconnect",
        dimensions=(5, 5, 2),  # Small dimensions for testing
        position=(0, 0, 0)
    )
    return area_id, area


@pytest.fixture
def test_neurons(connectome, test_area):
    """Create a few test neurons."""
    area_id = test_area[0]
    neuron_ids = []
    
    # Create 5 neurons with different positions
    for i in range(5):
        neuron_id = connectome.create_neuron(
            area_id=area_id,
            position=(i, 0, 0),
            threshold=1.0,
            refractory_period=5,
            decay_rate=0.9,
            resting_potential=0.0
        )
        neuron_ids.append(neuron_id)
    
    return neuron_ids


@pytest.mark.unit
def test_create_neuron(connectome, test_area):
    """Test neuron creation and retrieval."""
    area_id = test_area[0]
    
    # Create a neuron
    neuron_id = connectome.create_neuron(
        area_id=area_id,
        position=(2, 2, 1),
        threshold=1.0,
        refractory_period=5,
        decay_rate=0.9,
        resting_potential=0.0
    )
    
    # Verify neuron exists
    assert neuron_id in connectome._neuron_id_to_index
    
    # Verify neuron properties
    threshold = connectome.get_neuron_property(neuron_id, NeuronPropertyType.THRESHOLD)
    assert threshold == 1.0
    
    # Verify neuron position
    position = connectome.get_neuron_position(neuron_id)
    assert position == (2, 2, 1)
    
    # Verify area assignment
    neurons_in_area = connectome.get_neurons_by_area(area_id)
    assert neuron_id in neurons_in_area


@pytest.mark.unit
def test_create_multiple_neurons(connectome, test_area):
    """Test creation of multiple neurons."""
    area_id = test_area[0]
    
    # Create several neurons
    neuron_ids = []
    for x in range(3):
        for y in range(3):
            neuron_id = connectome.create_neuron(
                area_id=area_id,
                position=(x, y, 0)
            )
            neuron_ids.append(neuron_id)
    
    # Verify neuron count
    assert connectome.get_neuron_count() == 9
    
    # Verify all neurons are in the area
    neurons_in_area = connectome.get_neurons_by_area(area_id)
    assert len(neurons_in_area) == 9
    
    # Delete a neuron
    connectome.delete_neuron(neuron_ids[0])
    
    # Verify neuron count decreased
    assert connectome.get_neuron_count() == 8


@pytest.mark.unit
def test_create_synapses(connectome, test_area):
    """Test synapse creation and retrieval."""
    area_id = test_area[0]
    
    # Create two neurons
    pre_id = connectome.create_neuron(
        area_id=area_id,
        position=(0, 0, 0)
    )
    
    post_id = connectome.create_neuron(
        area_id=area_id,
        position=(1, 0, 0)
    )
    
    # Create a synapse
    result = connectome.create_synapse(
        pre_neuron_id=pre_id,
        post_neuron_id=post_id,
        weight=1.5,
        is_plastic=True,
        plasticity_coeff=0.1,
        plasticity_decay=0.01
    )
    
    # Verify synapse creation succeeded
    assert result
    
    # Verify outgoing connections
    outgoing = connectome.get_outgoing_connections(pre_id)
    assert len(outgoing) == 1
    post_id_result, weight = outgoing[0]
    assert post_id_result == post_id
    assert weight == 1.5
    
    # Verify incoming connections
    incoming = connectome.get_incoming_connections(post_id)
    assert len(incoming) == 1
    pre_id_result, weight = incoming[0]
    assert pre_id_result == pre_id
    assert weight == 1.5


@pytest.mark.unit
def test_membrane_potential_update(connectome, test_area):
    """Test updating membrane potentials."""
    area_id = test_area[0]
    
    # Create two neurons
    pre_id = connectome.create_neuron(
        area_id=area_id,
        position=(0, 0, 0),
        threshold=1.0
    )
    
    post_id = connectome.create_neuron(
        area_id=area_id,
        position=(1, 0, 0),
        threshold=0.5  # Lower threshold to ensure firing
    )
    
    # Create a synapse from pre to post
    connectome.create_synapse(
        pre_neuron_id=pre_id,
        post_neuron_id=post_id,
        weight=1.5  # Weight strong enough to trigger firing
    )
    
    # Get internal indices
    pre_idx = connectome._neuron_id_to_index[pre_id]
    post_idx = connectome._neuron_id_to_index[post_id]
    
    # Add pre_neuron to FCL manually (simulating it firing)
    connectome.fcl_manager.add_to_current_fcl([pre_idx])
    
    # Verify pre_neuron is in FCL at t=0
    assert pre_idx in connectome.fcl_manager.get_fcl(0)
    
    # Update membrane potentials
    firing_neurons = connectome.update_membrane_potentials(current_timestep=1)
    
    # Verify that post_neuron received synaptic input and fired
    assert post_id in firing_neurons
    
    # Verify post_neuron's membrane potential was reset after firing
    assert connectome.membrane_potentials[post_idx] == 0.0
    
    # Verify post_neuron is now in the current FCL
    assert post_idx in connectome.fcl_manager.get_fcl(0)


@pytest.mark.unit
def test_neuron_queries(connectome, test_area):
    """Test neuron query methods."""
    area_id = test_area[0]
    
    # Create neurons with different thresholds
    neuron_ids = []
    thresholds = [0.5, 0.7, 1.0, 1.2, 1.5]
    
    for i, threshold in enumerate(thresholds):
        neuron_id = connectome.create_neuron(
            area_id=area_id,
            position=(i, 0, 0),
            threshold=threshold
        )
        neuron_ids.append(neuron_id)
    
    # Test threshold range query
    low_threshold_neurons = connectome.query_neurons_by_threshold_range(0.4, 0.8)
    assert len(low_threshold_neurons) == 2
    assert neuron_ids[0] in low_threshold_neurons  # 0.5
    assert neuron_ids[1] in low_threshold_neurons  # 0.7
    
    high_threshold_neurons = connectome.query_neurons_by_threshold_range(1.1, 1.6)
    assert len(high_threshold_neurons) == 2
    assert neuron_ids[3] in high_threshold_neurons  # 1.2
    assert neuron_ids[4] in high_threshold_neurons  # 1.5
    
    # Test position query
    neurons_at_x0 = connectome.query_neurons_by_area_and_position(
        area_id, 
        x_range=(0, 0),
        y_range=(0, 0)
    )
    assert len(neurons_at_x0) == 1
    assert neuron_ids[0] in neurons_at_x0


@pytest.mark.unit
def test_get_set_neuron_property(connectome, test_area):
    """Test getting and setting neuron properties."""
    area_id = test_area[0]
    
    # Create a neuron
    neuron_id = connectome.create_neuron(
        area_id=area_id,
        position=(0, 0, 0),
        threshold=1.0
    )
    
    # Test get property
    threshold = connectome.get_neuron_property(neuron_id, NeuronPropertyType.THRESHOLD)
    assert threshold == 1.0
    
    # Test set property
    connectome.set_neuron_property(neuron_id, NeuronPropertyType.THRESHOLD, 2.0)
    new_threshold = connectome.get_neuron_property(neuron_id, NeuronPropertyType.THRESHOLD)
    assert new_threshold == 2.0 