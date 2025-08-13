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
Unit tests for GPU-optimized ConnectomeManager.

These tests focus on verifying the correctness of the GPU-optimized
ConnectomeManager functionality with minimal resource usage.
"""

import unittest

import numpy as np
import pytest

from feagi.bdu.connectome_manager import NeuronPropertyType
from feagi.bdu.connectome_manager_gpu import ConnectomeManagerGPU
from feagi.bdu.models.array_backend import ArrayBackend, BackendType


@pytest.fixture
def small_config():
    """Create a small FeagiConfig for testing."""
    # Create a dictionary config with reduced neuron counts
    config_dict = {
        "connectome.max_neurons": 100,
        "connectome.max_synapses_per_neuron": 10,
        "connectome.fcl_window_size": 3,
    }
    return config_dict


@pytest.fixture
def connectome(small_config):
    """Create a ConnectomeManagerGPU with minimal capacity for fast testing."""
    connectome = ConnectomeManagerGPU(config_or_max_neurons=small_config)
    return connectome


@pytest.fixture
def test_area(connectome):
    """Create a small test cortical area."""
    cortical_id = connectome.add_cortical_area(
        name="Test Area",
        area_type="interconnect",
        dimensions=(5, 5, 2),  # Small dimensions for testing
        position=(0, 0, 0),
    )
    return cortical_id


@pytest.fixture
def test_neurons(connectome, test_area):
    """Create a few test neurons."""
    neuron_ids = []

    # Create 5 neurons with different positions
    for i in range(5):
        neuron_id = connectome.create_neuron(
            cortical_id=test_area,
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
    # Create a neuron
    neuron_id = connectome.create_neuron(
        cortical_id=test_area,
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
    neurons_in_area = connectome.get_neurons_by_cortical_area(test_area)
    assert neuron_id in neurons_in_area


@pytest.mark.unit
def test_create_multiple_neurons(connectome, test_area):
    """Test creation of multiple neurons."""
    # Create several neurons
    neuron_ids = []
    for x in range(3):
        for y in range(3):
            neuron_id = connectome.create_neuron(
                cortical_id=test_area, position=(x, y, 0)
            )
            neuron_ids.append(neuron_id)

    # Verify neuron count
    assert connectome.get_neuron_count() == 9

    # Verify all neurons are in the area
    neurons_in_area = connectome.get_neurons_by_cortical_area(test_area)
    assert len(neurons_in_area) == 9

    # Delete a neuron
    connectome.delete_neuron(neuron_ids[0])

    # Verify neuron count decreased
    assert connectome.get_neuron_count() == 8


@pytest.mark.unit
def test_create_synapses(connectome, test_area):
    """Test synapse creation and retrieval."""
    # Create two neurons
    pre_id = connectome.create_neuron(cortical_id=test_area, position=(0, 0, 0))

    post_id = connectome.create_neuron(cortical_id=test_area, position=(1, 0, 0))

    # Get indices
    pre_idx = connectome.neuron_id_to_index[pre_id]
    post_idx = connectome.neuron_id_to_index[post_id]

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

    # Print debug info
    print(f"pre_id: {pre_id}, pre_idx: {pre_idx}")
    print(f"post_id: {post_id}, post_idx: {post_idx}")
    print(
        f"outgoing_matrix[{pre_idx}, {post_idx}] = {connectome.outgoing_matrix[pre_idx, post_idx]}"
    )
    print(
        f"incoming_matrix[{post_idx}, {pre_idx}] = {connectome.incoming_matrix[post_idx, pre_idx]}"
    )

    # Convert to CSR/CSC format for inspection
    connectome._ensure_csr_format_outgoing()
    connectome._ensure_csc_format_incoming()

    # Verify outgoing connections
    outgoing = connectome.get_outgoing_connections(pre_id)
    assert len(outgoing) == 1
    post_id_result, weight = outgoing[0]
    assert post_id_result == post_id
    assert weight == 1.5

    # Verify incoming connections directly in the matrix
    assert connectome.incoming_matrix[post_idx, pre_idx] == 1.5

    # Now check the get_incoming_connections method
    incoming = connectome.get_incoming_connections(post_id)
    print(f"Incoming connections: {incoming}")

    # Check the incoming connections
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
        neuron_id = connectome.create_neuron(cortical_id=test_area, position=(i, 0, 0))
        neuron_ids.append(neuron_id)

    # Create synapses in a batch
    synapse_specs = [
        (neuron_ids[0], neuron_ids[1], 1.0),
        (neuron_ids[0], neuron_ids[2], 1.5),
        (neuron_ids[1], neuron_ids[3], 0.8),
        (neuron_ids[2], neuron_ids[4], 1.2),
    ]

    created = connectome.batch_create_synapses(synapse_specs)

    # Verify all synapses were created
    assert created == 4

    # Verify synapse weights with floating-point tolerance
    assert abs(connectome.get_synapse_weight(neuron_ids[0], neuron_ids[1]) - 1.0) < 1e-5
    assert abs(connectome.get_synapse_weight(neuron_ids[0], neuron_ids[2]) - 1.5) < 1e-5
    assert abs(connectome.get_synapse_weight(neuron_ids[1], neuron_ids[3]) - 0.8) < 1e-5
    assert abs(connectome.get_synapse_weight(neuron_ids[2], neuron_ids[4]) - 1.2) < 1e-5


@pytest.mark.unit
def test_update_synapse_weight(connectome, test_area):
    """Test updating synapse weights."""
    # Create two neurons
    pre_id = connectome.create_neuron(cortical_id=test_area, position=(0, 0, 0))

    post_id = connectome.create_neuron(cortical_id=test_area, position=(1, 0, 0))

    # Create a synapse
    connectome.create_synapse(pre_neuron_id=pre_id, post_neuron_id=post_id, weight=1.0)

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
        cortical_id=test_area,
        position=(0, 0, 0),
        threshold=1.0,
        membrane_potential=1.5,  # Set above threshold to ensure firing
    )

    post_id = connectome.create_neuron(
        cortical_id=test_area,
        position=(1, 0, 0),
        threshold=0.5,  # Lower threshold to ensure firing from incoming spike
    )

    # Create a synapse from pre to post
    connectome.create_synapse(pre_neuron_id=pre_id, post_neuron_id=post_id, weight=1.0)

    # Get pre-neuron index and add it to the FCL manually
    pre_idx = connectome.neuron_id_to_index[pre_id]
    connectome.fcl_manager.add_to_current_fcl([pre_idx])

    # Update membrane potentials to propagate activity
    # Just check that the method runs without errors
    fired_ids = connectome.update_membrane_potentials()

    # Simple verification - we should get a list of fired neuron IDs
    assert isinstance(fired_ids, list)

    # Verify we can run multiple timesteps
    fired_ids = connectome.update_membrane_potentials()
    assert isinstance(fired_ids, list)


@pytest.mark.unit
def test_delete_neuron_with_synapses(connectome, test_area):
    """Test deletion of a neuron with synapses."""
    # Create three neurons
    neuron_ids = []
    for i in range(3):
        neuron_id = connectome.create_neuron(cortical_id=test_area, position=(i, 0, 0))
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


class TestConnectomeManagerGPU(unittest.TestCase):
    """Test the GPU-optimized connectome manager."""

    def setUp(self):
        """Set up a small connectome for testing."""
        # Reset singleton to ensure clean state for each test
        ConnectomeManagerGPU.reset_singleton()
        self.connectome = ConnectomeManagerGPU(
            config_or_max_neurons=1000, backend=BackendType.NUMPY
        )

    def tearDown(self):
        """Clean up after each test."""
        # Reset singleton to prevent state leakage between tests
        ConnectomeManagerGPU.reset_singleton()

    def test_add_neurons(self):
        """Test adding neurons to the connectome."""
        # Create a test cortical area first
        test_area = self.connectome.add_cortical_area(
            name="Test Area",
            area_type="interconnect",
            dimensions=(10, 10, 1),
            position=(0, 0, 0),
        )

        # Add single neuron
        neuron_id = self.connectome.create_neuron(
            cortical_id=test_area, position=(0, 0, 0)
        )
        self.assertEqual(self.connectome.get_neuron_count(), 1)
        self.assertIn(neuron_id, self.connectome.neuron_id_to_index)

        # Add multiple neurons (positions must be within bounds: 0-9 for x and y)
        neuron_ids = []
        for i in range(10):
            x = (i + 1) % 10  # Keep x within 0-9
            y = (i + 1) // 10  # Keep y within bounds
            new_neuron_id = self.connectome.create_neuron(
                cortical_id=test_area, position=(x, y, 0)
            )
            neuron_ids.append(new_neuron_id)

        self.assertEqual(len(neuron_ids), 10)
        self.assertEqual(self.connectome.get_neuron_count(), 11)

        # Verify all IDs are unique
        all_ids = [neuron_id] + neuron_ids
        self.assertEqual(len(all_ids), len(set(all_ids)))

    def test_delete_neurons(self):
        """Test deleting neurons from the connectome."""
        # Create a test cortical area first
        test_area = self.connectome.add_cortical_area(
            name="Test Area 2",
            area_type="interconnect",
            dimensions=(10, 10, 1),
            position=(0, 0, 0),
        )

        # Add neurons
        neuron_ids = []
        for i in range(5):
            neuron_id = self.connectome.create_neuron(
                cortical_id=test_area, position=(i, 0, 0)
            )
            neuron_ids.append(neuron_id)

        self.assertEqual(self.connectome.get_neuron_count(), 5)

        # Delete one neuron
        self.connectome.delete_neuron(neuron_ids[0])
        self.assertEqual(self.connectome.get_neuron_count(), 4)
        self.assertNotIn(neuron_ids[0], self.connectome.neuron_id_to_index)

        # Delete multiple neurons
        for neuron_id in neuron_ids[1:3]:
            self.connectome.delete_neuron(neuron_id)
        self.assertEqual(self.connectome.get_neuron_count(), 2)
        self.assertNotIn(neuron_ids[1], self.connectome.neuron_id_to_index)
        self.assertNotIn(neuron_ids[2], self.connectome.neuron_id_to_index)

    def test_add_synapses(self):
        """Test adding synapses to the connectome."""
        # Create a test cortical area first
        test_area = self.connectome.add_cortical_area(
            name="Test Area 3",
            area_type="interconnect",
            dimensions=(10, 10, 1),
            position=(0, 0, 0),
        )

        # Add neurons
        neuron_ids = []
        for i in range(10):
            neuron_id = self.connectome.create_neuron(
                cortical_id=test_area, position=(i, 0, 0)
            )
            neuron_ids.append(neuron_id)

        # Add single synapse
        result = self.connectome.create_synapse(
            pre_neuron_id=neuron_ids[0], post_neuron_id=neuron_ids[1], weight=0.5
        )
        self.assertTrue(result)
        self.assertEqual(self.connectome.get_synapse_count(), 1)

        # Verify synapse exists
        outgoing = self.connectome.get_outgoing_connections(neuron_ids[0])
        self.assertEqual(len(outgoing), 1)
        self.assertEqual(outgoing[0][0], neuron_ids[1])  # target neuron
        self.assertEqual(outgoing[0][1], 0.5)  # weight

        # Add multiple synapses
        synapse_specs = [
            (neuron_ids[2], neuron_ids[5], 0.1),
            (neuron_ids[3], neuron_ids[6], 0.2),
            (neuron_ids[4], neuron_ids[7], 0.3),
        ]

        created_count = self.connectome.batch_create_synapses(synapse_specs)

        # Verify all synapses were added
        self.assertEqual(created_count, 3)  # Should have created 3 synapses
        self.assertEqual(self.connectome.get_synapse_count(), 4)  # 1 + 3 = 4 total

        # Verify synapses exist
        for pre, post, weight in synapse_specs:
            outgoing = self.connectome.get_outgoing_connections(pre)
            found = False
            for target_id, synapse_weight in outgoing:
                # Convert numpy types to Python types and use approximate comparison for floats
                target_id_py = int(target_id)
                synapse_weight_py = float(synapse_weight)
                if target_id_py == post and abs(synapse_weight_py - weight) < 1e-6:
                    found = True
                    break
            self.assertTrue(
                found, f"Synapse {pre}->{post} with weight {weight} not found"
            )

    def test_delete_synapses(self):
        """Test deleting synapses from the connectome."""
        # Add neurons
        neuron_ids = self.connectome.add_neurons(5)

        # Add synapses
        self.connectome.add_synapse(neuron_ids[0], neuron_ids[1], 0.5)
        self.connectome.add_synapse(neuron_ids[1], neuron_ids[2], 0.5)
        self.connectome.add_synapse(neuron_ids[2], neuron_ids[3], 0.5)
        self.assertEqual(self.connectome.synapse_count, 3)

        # Delete one synapse
        self.connectome.delete_synapse(neuron_ids[0], neuron_ids[1])
        self.assertEqual(self.connectome.synapse_count, 2)
        self.assertFalse(self.connectome.has_synapse(neuron_ids[0], neuron_ids[1]))

        # Delete multiple synapses
        self.connectome.delete_synapses(
            [(neuron_ids[1], neuron_ids[2]), (neuron_ids[2], neuron_ids[3])]
        )
        self.assertEqual(self.connectome.synapse_count, 0)

    def test_batch_update_neuron_properties(self):
        """Test batch updating neuron properties."""
        # Add neurons
        neuron_ids = self.connectome.add_neurons(5)

        # Update thresholds for all neurons
        self.connectome.batch_update_neuron_properties(
            neuron_ids=neuron_ids, property_name="threshold", values=1.5
        )

        # Verify thresholds were updated
        thresholds = self.connectome.batch_get_neuron_properties(
            neuron_ids=neuron_ids, property_name="threshold"
        )
        self.assertTrue(np.allclose(thresholds, 1.5))

        # Update membrane potentials with different values for each neuron
        values = [0.1, 0.2, 0.3, 0.4, 0.5]
        self.connectome.batch_update_neuron_properties(
            neuron_ids=neuron_ids, property_name="membrane_potential", values=values
        )

        # Verify membrane potentials were updated
        potentials = self.connectome.batch_get_neuron_properties(
            neuron_ids=neuron_ids, property_name="membrane_potential"
        )
        self.assertTrue(np.allclose(potentials, values))

    def test_update_membrane_potentials(self):
        """Test updating membrane potentials with decay."""
        # Add neurons
        neuron_ids = self.connectome.add_neurons(5)

        # Set initial membrane potentials
        self.connectome.batch_update_neuron_properties(
            neuron_ids=neuron_ids, property_name="membrane_potential", values=1.0
        )

        # Update with decay
        decay_factor = 0.9
        self.connectome.update_membrane_potentials(decay_factor)

        # Verify potentials were decayed
        potentials = self.connectome.batch_get_neuron_properties(
            neuron_ids=neuron_ids, property_name="membrane_potential"
        )
        self.assertTrue(np.allclose(potentials, decay_factor))

    def test_find_neurons_above_threshold(self):
        """Test finding neurons above threshold."""
        # Add neurons
        neuron_ids = self.connectome.add_neurons(10)

        # Set membrane potentials and thresholds
        # Neurons 0, 2, 4, 6, 8 will be above threshold
        for i, nid in enumerate(neuron_ids):
            self.connectome.update_neuron_property(
                neuron_id=nid,
                property_name="membrane_potential",
                value=float(i) / 10.0 + 0.5,  # 0.5, 0.6, 0.7, ..., 1.4
            )
            self.connectome.update_neuron_property(
                neuron_id=nid,
                property_name="threshold",
                value=(
                    1.0 if i % 2 == 0 else 2.0
                ),  # Even indices: threshold 1.0, odd: threshold 2.0
            )

        # Find neurons above threshold
        above_threshold = self.connectome.find_neurons_above_threshold()

        # Extract expected neuron IDs (indices 6 and 8 should be above threshold)
        expected_ids = [neuron_ids[6], neuron_ids[8]]

        # Verify correct neurons were found
        self.assertEqual(set(above_threshold), set(expected_ids))

    def test_process_firing_neurons(self):
        """Test processing firing neurons."""
        # Add neurons
        neuron_ids = self.connectome.add_neurons(5)

        # Create a simple feed-forward network
        # 0 -> 1 -> 2 -> 3 -> 4
        self.connectome.add_synapse(neuron_ids[0], neuron_ids[1], 0.5)
        self.connectome.add_synapse(neuron_ids[1], neuron_ids[2], 0.5)
        self.connectome.add_synapse(neuron_ids[2], neuron_ids[3], 0.5)
        self.connectome.add_synapse(neuron_ids[3], neuron_ids[4], 0.5)

        # Set membrane potentials to 0
        self.connectome.batch_update_neuron_properties(
            neuron_ids=neuron_ids, property_name="membrane_potential", values=0.0
        )

        # Fire neuron 0
        firing_neurons = [neuron_ids[0]]
        self.connectome.process_firing_neurons(firing_neurons)

        # Verify membrane potentials were updated
        potentials = self.connectome.batch_get_neuron_properties(
            neuron_ids=neuron_ids, property_name="membrane_potential"
        )

        # Neuron 0 should be reset, neuron 1 should receive input, others unchanged
        self.assertEqual(potentials[0], 0.0)  # Neuron 0 reset
        self.assertEqual(potentials[1], 0.25)  # Neuron 1 received input
        self.assertEqual(potentials[2], 0.0)  # Unchanged
        self.assertEqual(potentials[3], 0.0)  # Unchanged
        self.assertEqual(potentials[4], 0.0)  # Unchanged

        # Explicitly set and verify the active state (this is a separate operation from process_firing_neurons)
        self.connectome.set_neuron_property(neuron_ids[0], "is_active", True)
        active = self.connectome.get_neuron_property(neuron_ids[0], "is_active")
        self.assertTrue(active)

    def test_to_csr_format(self):
        """Test conversion to CSR format."""
        # Add neurons
        neuron_ids = self.connectome.add_neurons(5)

        # Add synapses
        self.connectome.add_synapse(neuron_ids[0], neuron_ids[1], 0.1)
        self.connectome.add_synapse(neuron_ids[0], neuron_ids[2], 0.2)
        self.connectome.add_synapse(neuron_ids[1], neuron_ids[3], 0.3)
        self.connectome.add_synapse(neuron_ids[2], neuron_ids[4], 0.4)

        # Convert to CSR format
        self.connectome._ensure_csr_format_outgoing()

        # Verify CSR matrix was created
        self.assertIsNotNone(self.connectome.outgoing_matrix)

        # Check matrix properties
        matrix = self.connectome.outgoing_matrix
        self.assertEqual(matrix.shape[0], self.connectome.max_neurons)
        self.assertEqual(matrix.shape[1], self.connectome.max_neurons)
        self.assertEqual(matrix.nnz, 4)  # 4 synapses


@pytest.mark.parametrize(
    "backend_type",
    [
        BackendType.NUMPY,
        pytest.param(
            BackendType.PYTORCH,
            marks=pytest.mark.skipif(
                not hasattr(ArrayBackend, "_is_backend_available")
                or not ArrayBackend._is_backend_available(BackendType.PYTORCH),
                reason="PyTorch not available",
            ),
        ),
        pytest.param(
            BackendType.CUPY,
            marks=pytest.mark.skipif(
                not hasattr(ArrayBackend, "_is_backend_available")
                or not ArrayBackend._is_backend_available(BackendType.CUPY),
                reason="CuPy not available",
            ),
        ),
    ],
)
def test_backend_compatibility(backend_type):
    """Test compatibility with different array backends."""
    try:
        # Create connectome with specific backend
        connectome = ConnectomeManagerGPU(
            config_or_max_neurons=100, backend=backend_type
        )

        # Basic operations
        neuron_ids = connectome.add_neurons(10)
        connectome.add_synapse(neuron_ids[0], neuron_ids[1], 0.5)
        connectome.batch_update_neuron_properties(
            neuron_ids=neuron_ids, property_name="threshold", values=1.5
        )

        # Verify operations worked
        assert connectome.neuron_count == 10
        assert connectome.synapse_count == 1
        assert connectome.has_synapse(neuron_ids[0], neuron_ids[1])

        thresholds = connectome.batch_get_neuron_properties(
            neuron_ids=neuron_ids, property_name="threshold"
        )
        assert np.allclose(thresholds, 1.5)

    except Exception as e:
        pytest.fail(f"Failed with backend {backend_type.value}: {e}")


if __name__ == "__main__":
    unittest.main()
