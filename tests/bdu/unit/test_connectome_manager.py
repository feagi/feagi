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
Unit tests for ConnectomeManager.

These tests focus on verifying the correctness of ConnectomeManager functionality
with minimal resource usage for fast execution during development.
"""

import pytest

from feagi.bdu.connectome_manager import ConnectomeManager, NeuronPropertyType
from feagi.utils.config import FeagiConfig


@pytest.fixture
def small_config():
    """Create a minimal FeagiConfig for testing."""
    config = FeagiConfig()
    config.set("connectome.max_neurons", 100)  # Minimal size for testing
    config.set("connectome.max_synapses_per_neuron", 10)
    config.set("connectome.fcl_window_size", 3)
    return config


@pytest.fixture
def connectome(small_config):
    """Create a ConnectomeManager with minimal capacity for fast testing."""
    # Reset singleton before each test to ensure clean state
    ConnectomeManager.reset_singleton()
    connectome = ConnectomeManager(small_config)
    yield connectome
    # Reset singleton after each test to prevent state leakage
    ConnectomeManager.reset_singleton()


@pytest.fixture
def test_area(connectome):
    """Create a small test cortical area."""
    # Create area with a specific cortical_id that matches the expected area_id=1
    cortical_id = "C12345"
    area = connectome.add_cortical_area(
        name="Test Area",
        area_type="interconnect",
        dimensions=(5, 5, 2),  # Small dimensions for testing
        position=(0, 0, 0),
        cortical_id=cortical_id,
    )
    return cortical_id, area


@pytest.fixture
def test_neurons(connectome, test_area):
    """Create a few test neurons."""
    cortical_id = test_area[0]
    neuron_ids = []

    # Create 5 neurons with different positions
    for i in range(5):
        neuron_id = connectome.create_neuron(
            cortical_id=cortical_id,
            position=(i, 0, 0),
            threshold=1.0,
            refractory_period=5,
            decay_rate=0.9,
            resting_potential=0.0,
        )
        neuron_ids.append(neuron_id)

    return neuron_ids


@pytest.mark.unit
def test_create_neuron(connectome, test_area):
    """Test neuron creation and retrieval."""
    cortical_id = test_area[0]

    # Create a neuron
    neuron_id = connectome.create_neuron(
        cortical_id=cortical_id,
        position=(2, 2, 1),
        threshold=1.0,
        refractory_period=5,
        decay_rate=0.9,
        resting_potential=0.0,
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
    neurons_in_area = connectome.get_neurons_by_cortical_area(cortical_id)
    assert neuron_id in neurons_in_area


@pytest.mark.unit
def test_create_multiple_neurons(connectome, test_area):
    """Test creation of multiple neurons."""
    cortical_id = test_area[0]

    # Create several neurons
    neuron_ids = []
    for x in range(3):
        for y in range(3):
            neuron_id = connectome.create_neuron(
                cortical_id=cortical_id, position=(x, y, 0)
            )
            neuron_ids.append(neuron_id)

    # Verify neuron count
    assert connectome.get_neuron_count() == 9

    # Verify all neurons are in the area
    neurons_in_area = connectome.get_neurons_by_cortical_area(cortical_id)
    assert len(neurons_in_area) == 9

    # Delete a neuron
    connectome.delete_neuron(neuron_ids[0])

    # Verify neuron count decreased
    assert connectome.get_neuron_count() == 8


@pytest.mark.unit
def test_create_synapses(connectome, test_area):
    """Test synapse creation and retrieval."""
    cortical_id = test_area[0]

    # Create two neurons
    pre_id = connectome.create_neuron(cortical_id=cortical_id, position=(0, 0, 0))

    post_id = connectome.create_neuron(cortical_id=cortical_id, position=(1, 0, 0))

    # Create a synapse
    result = connectome.create_synapse(
        pre_neuron_id=pre_id,
        post_neuron_id=post_id,
        weight=1.5,
        is_plastic=True,
        plasticity_coeff=0.1,
        plasticity_decay=0.01,
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
    cortical_id = test_area[0]

    # Create two neurons
    pre_id = connectome.create_neuron(
        cortical_id=cortical_id, position=(0, 0, 0), threshold=1.0
    )

    post_id = connectome.create_neuron(
        cortical_id=cortical_id,
        position=(1, 0, 0),
        threshold=0.5,  # Lower threshold to ensure firing
    )

    # Create a synapse from pre to post
    connectome.create_synapse(
        pre_neuron_id=pre_id,
        post_neuron_id=post_id,
        weight=1.5,  # Weight strong enough to trigger firing
    )

    # Get internal indices
    pre_idx = connectome._neuron_id_to_index[pre_id]
    post_idx = connectome._neuron_id_to_index[post_id]

    # Set pre_neuron's membrane potential high enough to exceed threshold after decay
    # With default decay rate of 0.5, we need 2.0 to get 1.0 after decay
    connectome.set_neuron_property(pre_id, NeuronPropertyType.MEMBRANE_POTENTIAL, 2.1)

    # Update membrane potentials (first timestep - pre-neuron fires)
    firing_neurons_t1 = connectome.update_membrane_potentials(current_timestep=1)
    assert pre_id in firing_neurons_t1

    # Run second timestep (post-neuron should fire from synaptic input)
    firing_neurons_t2 = connectome.update_membrane_potentials(current_timestep=2)

    # Verify that post_neuron fired in the second timestep
    assert post_id in firing_neurons_t2

    # Check post-neuron membrane potential after second timestep
    post_potential_t2 = connectome.get_neuron_property(
        post_id, NeuronPropertyType.MEMBRANE_POTENTIAL
    )

    # Verify post_neuron's membrane potential was reset after firing
    assert post_potential_t2 == 0.0


@pytest.mark.unit
def test_neuron_queries(connectome, test_area):
    """Test neuron query methods."""
    cortical_id = test_area[0]

    # Create neurons with different thresholds
    neuron_ids = []
    thresholds = [0.5, 0.7, 1.0, 1.2, 1.5]

    for i, threshold in enumerate(thresholds):
        neuron_id = connectome.create_neuron(
            cortical_id=cortical_id, position=(i, 0, 0), threshold=threshold
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
        cortical_id, x_range=(0, 0), y_range=(0, 0)
    )
    assert len(neurons_at_x0) == 1
    assert neuron_ids[0] in neurons_at_x0


@pytest.mark.unit
def test_get_set_neuron_property(connectome, test_area):
    """Test getting and setting neuron properties."""
    cortical_id = test_area[0]

    # Create a neuron
    neuron_id = connectome.create_neuron(
        cortical_id=cortical_id, position=(0, 0, 0), threshold=1.0, decay_rate=0.5
    )

    # Get properties
    threshold = connectome.get_neuron_property(neuron_id, NeuronPropertyType.THRESHOLD)
    assert threshold == 1.0

    decay = connectome.get_neuron_property(neuron_id, "decay_rate")
    assert decay == 0.5

    # Set properties
    connectome.set_neuron_property(neuron_id, NeuronPropertyType.THRESHOLD, 2.0)
    connectome.set_neuron_property(neuron_id, "decay_rate", 0.7)

    # Verify changes
    assert (
        connectome.get_neuron_property(neuron_id, NeuronPropertyType.THRESHOLD) == 2.0
    )
    # Use approximate equality for floating point values
    assert abs(connectome.get_neuron_property(neuron_id, "decay_rate") - 0.7) < 1e-6


@pytest.mark.unit
def test_neuron_idx_auto_assignment(connectome, test_area):
    """Test that neuron indices are automatically assigned."""
    cortical_id = test_area[0]

    # Create a few neurons
    neuron_ids = []
    for i in range(3):
        neuron_id = connectome.create_neuron(
            cortical_id=cortical_id, position=(i, 0, 0)
        )
        neuron_ids.append(neuron_id)

    # Check that indices are unique
    indices = [connectome._neuron_id_to_index[n_id] for n_id in neuron_ids]
    assert len(set(indices)) == len(indices)

    # Delete a neuron and create a new one
    deleted_neuron_id = neuron_ids[1]
    deleted_idx = connectome._neuron_id_to_index[deleted_neuron_id]
    connectome.delete_neuron(deleted_neuron_id)

    new_neuron_id = connectome.create_neuron(
        cortical_id=cortical_id, position=(3, 0, 0)
    )

    # New neuron should have a valid index (may reuse deleted index for efficiency)
    new_idx = connectome._neuron_id_to_index[new_neuron_id]
    assert new_idx >= 0

    # Verify the deleted neuron is no longer in the mapping
    assert deleted_neuron_id not in connectome._neuron_id_to_index

    # Verify all current indices are still unique
    current_neuron_ids = [
        neuron_ids[0],
        neuron_ids[2],
        new_neuron_id,
    ]  # Skip deleted neuron
    current_indices = [
        connectome._neuron_id_to_index[n_id] for n_id in current_neuron_ids
    ]
    assert len(set(current_indices)) == len(current_indices)


@pytest.mark.unit
def test_check_neuron_index_uniqueness(connectome, test_area):
    """Test that neuron indices are unique."""
    cortical_id = test_area[0]

    # Create several neurons
    for i in range(5):
        connectome.create_neuron(cortical_id=cortical_id, position=(i, 0, 0))

    # Check that all indices are unique
    assert connectome.check_neuron_index_uniqueness()

    # Create a second area
    second_area_id = connectome.add_cortical_area(
        name="Second Area",
        dimensions=(3, 3, 3),
        position=(10, 10, 10),
        area_type="custom",
    )

    # Create neurons in second area
    for i in range(3):
        connectome.create_neuron(cortical_id=second_area_id, position=(i, 0, 0))

    # Check that indices are still unique across areas
    assert connectome.check_neuron_index_uniqueness()


@pytest.mark.unit
def test_create_neuron_out_of_bounds(connectome, test_area):
    """Test that creating a neuron outside area bounds raises an error."""
    cortical_id = test_area[0]

    # Get the area object
    area = connectome.get_cortical_area(cortical_id)

    # Get area dimensions
    width, height, depth = area.dimensions

    # Try to create a neuron outside the area bounds
    with pytest.raises(ValueError, match=r".*outside the bounds.*"):
        connectome.create_neuron(
            cortical_id=cortical_id,
            position=(width, height, depth),  # All coordinates are out of bounds
        )


@pytest.mark.unit
def test_batch_create_neurons_edge_cases(connectome, test_area):
    """Test edge cases for batch neuron creation."""
    cortical_id = test_area[0]

    # Get the area object
    area = connectome.get_cortical_area(cortical_id)

    # Test with empty list
    assert connectome.batch_create_neurons(cortical_id, []) == []

    # Test with duplicate positions
    with pytest.raises(ValueError, match=r".*Duplicate.*"):
        connectome.batch_create_neurons(
            cortical_id,
            [(0, 0, 0), (0, 0, 0)],  # Duplicate position
        )

    # Test with out of bounds position
    width, height, depth = area.dimensions
    with pytest.raises(ValueError, match=r".*outside the bounds.*"):
        connectome.batch_create_neurons(
            cortical_id,
            [(0, 0, 0), (width, height, depth)],  # Second position is out of bounds
        )


@pytest.mark.unit
def test_extreme_dimension_area_block_lookup(connectome, test_area):
    """Test that area with extreme dimensions can still look up neurons by position."""
    # Create a new area with extreme dimensions
    extreme_area_id = connectome.add_cortical_area(
        name="Extreme Area",
        dimensions=(1, 1, 100),  # Very tall and thin
        position=(0, 0, 0),
        area_type="custom",
    )

    # Create neurons at various z positions
    neuron_ids = []
    for z in range(0, 100, 10):  # Create 10 neurons
        neuron_id = connectome.create_neuron(
            cortical_id=extreme_area_id, position=(0, 0, z)
        )
        neuron_ids.append(neuron_id)

    # Test lookup by position
    for z in range(0, 100, 10):
        neurons = connectome.get_neurons_at_position(extreme_area_id, (0, 0, z))
        assert len(neurons) == 1

    # Test lookup with no neurons
    neurons = connectome.get_neurons_at_position(extreme_area_id, (0, 0, 5))
    assert len(neurons) == 0
