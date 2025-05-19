"""
Unit tests for GPU-optimized ConnectomeManager.

These tests focus on verifying the correctness of the GPU-optimized 
ConnectomeManager functionality with minimal resource usage.
"""

import pytest
import numpy as np
from feagi.bdu.connectome_manager_gpu import ConnectomeManagerGPU, NeuronPropertyType
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
    """Create a ConnectomeManagerGPU with minimal capacity for fast testing."""
    connectome = ConnectomeManagerGPU(small_config)
    return connectome


@pytest.fixture
def test_area(connectome):
    """Create a small test cortical area."""
    area_id = connectome.add_cortical_area(
        name="Test Area",
        area_type="interconnect",
        dimensions=(5, 5, 2),  # Small dimensions for testing
        position=(0, 0, 0)
    )
    return area_id


@pytest.fixture
def test_neurons(connectome, test_area):
    """Create a few test neurons."""
    neuron_ids = []
    
    # Create 5 neurons with different positions
    for i in range(5):
        neuron_id = connectome.create_neuron(
            area_id=test_area,
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
    # Create a neuron
    neuron_id = connectome.create_neuron(
        area_id=test_area,
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
    neurons_in_area = connectome.get_neurons_by_area(test_area)
    assert neuron_id in neurons_in_area


@pytest.mark.unit
def test_create_multiple_neurons(connectome, test_area):
    """Test creation of multiple neurons."""
    # Create several neurons
    neuron_ids = []
    for x in range(3):
        for y in range(3):
            neuron_id = connectome.create_neuron(
                area_id=test_area,
                position=(x, y, 0)
            )
            neuron_ids.append(neuron_id)
    
    # Verify neuron count
    assert connectome.get_neuron_count() == 9
    
    # Verify all neurons are in the area
    neurons_in_area = connectome.get_neurons_by_area(test_area)
    assert len(neurons_in_area) == 9
    
    # Delete a neuron
    connectome.delete_neuron(neuron_ids[0])
    
    # Verify neuron count decreased
    assert connectome.get_neuron_count() == 8


@pytest.mark.unit
def test_create_synapses(connectome, test_area):
    """Test synapse creation and retrieval."""
    # Create two neurons
    pre_id = connectome.create_neuron(
        area_id=test_area,
        position=(0, 0, 0)
    )
    
    post_id = connectome.create_neuron(
        area_id=test_area,
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
def test_batch_create_synapses(connectome, test_area):
    """Test batch synapse creation."""
    # Create several neurons
    neuron_ids = []
    for i in range(5):
        neuron_id = connectome.create_neuron(
            area_id=test_area,
            position=(i, 0, 0)
        )
        neuron_ids.append(neuron_id)
    
    # Create synapses in a batch
    synapse_specs = [
        (neuron_ids[0], neuron_ids[1], 1.0),
        (neuron_ids[0], neuron_ids[2], 1.5),
        (neuron_ids[1], neuron_ids[3], 0.8),
        (neuron_ids[2], neuron_ids[4], 1.2)
    ]
    
    created = connectome.batch_create_synapses(synapse_specs)
    
    # Verify all synapses were created
    assert created == 4
    
    # Verify synapse weights
    assert connectome.get_synapse_weight(neuron_ids[0], neuron_ids[1]) == 1.0
    assert connectome.get_synapse_weight(neuron_ids[0], neuron_ids[2]) == 1.5
    assert connectome.get_synapse_weight(neuron_ids[1], neuron_ids[3]) == 0.8
    assert connectome.get_synapse_weight(neuron_ids[2], neuron_ids[4]) == 1.2
    
    # Verify total synapse count
    assert connectome.get_synapse_count() == 4


@pytest.mark.unit
def test_update_synapse_weight(connectome, test_area):
    """Test updating synapse weights."""
    # Create two neurons
    pre_id = connectome.create_neuron(
        area_id=test_area,
        position=(0, 0, 0)
    )
    
    post_id = connectome.create_neuron(
        area_id=test_area,
        position=(1, 0, 0)
    )
    
    # Create a synapse
    connectome.create_synapse(
        pre_neuron_id=pre_id,
        post_neuron_id=post_id,
        weight=1.0
    )
    
    # Update the weight
    result = connectome.update_synapse_weight(pre_id, post_id, 2.5)
    
    # Verify update succeeded
    assert result
    
    # Verify new weight
    weight = connectome.get_synapse_weight(pre_id, post_id)
    assert weight == 2.5


@pytest.mark.unit
def test_membrane_potential_update(connectome, test_area):
    """Test updating membrane potentials."""
    # Create two neurons
    pre_id = connectome.create_neuron(
        area_id=test_area,
        position=(0, 0, 0),
        threshold=1.0,
        membrane_potential=1.5  # Set above threshold to ensure firing
    )
    
    post_id = connectome.create_neuron(
        area_id=test_area,
        position=(1, 0, 0),
        threshold=0.5  # Lower threshold to ensure firing from incoming spike
    )
    
    # Create a synapse from pre to post
    connectome.create_synapse(
        pre_neuron_id=pre_id,
        post_neuron_id=post_id,
        weight=1.0
    )
    
    # Get pre-neuron index and add it to the FCL manually
    pre_idx = connectome.neuron_id_to_index[pre_id]
    connectome.fcl_manager.add_to_current_fcl([pre_idx])
    
    # Update membrane potentials to propagate activity
    fired_ids = connectome.update_membrane_potentials()
    
    # Post-neuron should fire due to incoming connection
    assert post_id in fired_ids


@pytest.mark.unit
def test_delete_neuron_with_synapses(connectome, test_area):
    """Test deletion of a neuron with synapses."""
    # Create three neurons
    neuron_ids = []
    for i in range(3):
        neuron_id = connectome.create_neuron(
            area_id=test_area,
            position=(i, 0, 0)
        )
        neuron_ids.append(neuron_id)
    
    # Create synapses between them
    connectome.create_synapse(neuron_ids[0], neuron_ids[1], 1.0)
    connectome.create_synapse(neuron_ids[1], neuron_ids[2], 1.0)
    connectome.create_synapse(neuron_ids[0], neuron_ids[2], 1.0)
    
    # Verify synapse count
    assert connectome.get_synapse_count() == 3
    
    # Delete the middle neuron
    connectome.delete_neuron(neuron_ids[1])
    
    # Verify neuron count decreased
    assert connectome.get_neuron_count() == 2
    
    # Verify synapses involving the deleted neuron are also deleted
    assert connectome.get_synapse_count() == 1
    assert connectome.get_synapse_weight(neuron_ids[0], neuron_ids[2]) == 1.0
    with pytest.raises(KeyError):
        connectome.get_synapse_weight(neuron_ids[0], neuron_ids[1]) 