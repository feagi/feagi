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

"""Tests for the CoreAPIService class."""

from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.bdu.connectome_manager import ConnectomeManager


@pytest.fixture
def connectome_manager():
    """Create a mock ConnectomeManager for testing."""
    cm = MagicMock(spec=ConnectomeManager)

    # Set up basic mocks for cortical area methods
    cm.cortical_areas = {
        "area1": MagicMock(
            name="Area 1", position=(0, 0, 0), dimensions=(10, 10, 10), type="sensory"
        ),
        "area2": MagicMock(
            name="Area 2", position=(20, 0, 0), dimensions=(10, 10, 10), type="motor"
        ),
    }
    cm.cortical_neuron_map = {"area1": {1, 2, 3}, "area2": {4, 5, 6}}

    # Mock neurons
    cm.neurons = {
        1: {
            "cortical_id": "area1",
            "position": (1, 1, 1),
            "membrane_potential": 0.5,
            "threshold": 1.0,
        },
        2: {
            "cortical_id": "area1",
            "position": (2, 2, 2),
            "membrane_potential": 0.3,
            "threshold": 1.0,
        },
        3: {
            "cortical_id": "area1",
            "position": (3, 3, 3),
            "membrane_potential": 0.8,
            "threshold": 1.0,
        },
        4: {
            "cortical_id": "area2",
            "position": (1, 1, 1),
            "membrane_potential": 0.1,
            "threshold": 1.0,
        },
        5: {
            "cortical_id": "area2",
            "position": (2, 2, 2),
            "membrane_potential": 0.9,
            "threshold": 1.0,
        },
        6: {
            "cortical_id": "area2",
            "position": (3, 3, 3),
            "membrane_potential": 0.4,
            "threshold": 1.0,
        },
    }

    # Mock methods
    cm.create_neuron.side_effect = lambda **kwargs: len(cm.neurons) + 1
    cm.get_neurons_by_cortical_area.side_effect = lambda cortical_id: list(
        cm.cortical_neuron_map.get(cortical_id, [])
    )
    cm.get_cortical_area_for_neuron.side_effect = lambda neuron_id: cm.neurons[
        neuron_id
    ]["cortical_id"]

    # Mock FCL manager
    fcl_manager = MagicMock()
    cm.fcl_manager = fcl_manager

    return cm


@pytest.fixture
def core_api_service(connectome_manager):
    """Create a CoreAPIService instance with a mock ConnectomeManager."""
    # Patch the initialization to avoid needing Neuroembryogenesis and other dependencies
    with patch("feagi.api.core.services.core_api_service.FeagiStateManager"):
        with patch("feagi.api.core.services.core_api_service.FEAGI"):
            service = CoreAPIService(connectome_manager)

            # Mock any other methods that might be called during tests
            service._burst_engine = MagicMock()

            return service


def test_get_membrane_potentials(core_api_service, connectome_manager):
    """Test getting membrane potentials for neurons."""
    # Get membrane potentials for neurons 1, 3, and 5
    result = core_api_service.get_membrane_potentials([1, 3, 5])

    # Verify results
    assert result == {1: 0.5, 3: 0.8, 5: 0.9}


def test_update_membrane_potentials(core_api_service, connectome_manager):
    """Test updating membrane potentials for neurons."""
    # Update membrane potentials
    updates = {1: 0.7, 3: 0.2, 5: 0.6}

    result = core_api_service.update_membrane_potentials(updates)

    # Verify results
    assert result is True

    # Verify connectome manager neurons were updated
    assert connectome_manager.neurons[1]["membrane_potential"] == 0.7
    assert connectome_manager.neurons[3]["membrane_potential"] == 0.2
    assert connectome_manager.neurons[5]["membrane_potential"] == 0.6


def test_batch_create_neurons(core_api_service, connectome_manager):
    """Test batch creation of neurons."""
    cortical_id = "area1"
    positions = [(4, 4, 4), (5, 5, 5), (6, 6, 6)]
    properties = {"threshold": 1.5, "decay_rate": 0.2}

    # Mock the create_neuron method to return sequential IDs
    neuron_id = 7

    def mock_create_neuron(**kwargs):
        nonlocal neuron_id
        result = neuron_id
        neuron_id += 1
        return result

    connectome_manager.create_neuron.side_effect = mock_create_neuron

    # Call batch_create_neurons
    result = core_api_service.batch_create_neurons(cortical_id, positions, properties)

    # Verify results
    assert result == [7, 8, 9]

    # Verify connectome_manager.create_neuron was called correctly
    assert connectome_manager.create_neuron.call_count == 3

    # Check the arguments for each call
    calls = connectome_manager.create_neuron.call_args_list
    assert calls[0].kwargs["cortical_id"] == "area1"
    assert calls[0].kwargs["position"] == (4, 4, 4)
    assert calls[0].kwargs["threshold"] == 1.5
    assert calls[0].kwargs["decay_rate"] == 0.2


def test_batch_create_synapses(core_api_service, connectome_manager):
    """Test batch creation of synapses."""
    # Setup test data
    connections = [(1, 4, 0.5), (2, 5, 0.7), (3, 6, 0.3)]

    # Mock create_synapse method
    connectome_manager.create_synapse.return_value = True

    # Call batch_create_synapses
    result = core_api_service.batch_create_synapses(connections)

    # Verify results
    assert result == 3

    # Verify connectome_manager.create_synapse was called correctly
    assert connectome_manager.create_synapse.call_count == 3

    # Check the arguments for each call
    calls = connectome_manager.create_synapse.call_args_list
    assert calls[0].kwargs["pre_neuron_id"] == 1
    assert calls[0].kwargs["post_neuron_id"] == 4
    assert calls[0].kwargs["weight"] == 0.5

    assert calls[1].kwargs["pre_neuron_id"] == 2
    assert calls[1].kwargs["post_neuron_id"] == 5
    assert calls[1].kwargs["weight"] == 0.7

    assert calls[2].kwargs["pre_neuron_id"] == 3
    assert calls[2].kwargs["post_neuron_id"] == 6
    assert calls[2].kwargs["weight"] == 0.3
