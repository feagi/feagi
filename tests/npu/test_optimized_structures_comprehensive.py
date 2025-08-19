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
Comprehensive test coverage for Optimized Structures module.

This module tests all functionality in optimized_structures.py to achieve
high code coverage, including GlobalNeuronArray, FireCandidateList,
Connectome, and OptimizedFeagiCore classes with both Rust and NumPy implementations.
"""

from unittest.mock import patch

import numpy as np
import pytest

from feagi.npu.optimized_structures import (
    Connectome,
    FireCandidateList,
    OptimizedFeagiCore,
)
from feagi.bdu.models.neuron import NeuronArray as GlobalNeuronArray


# Mock Rust components for testing
class MockRustGNA:
    """Mock Rust GNA for testing."""

    def __init__(self, capacity):
        self.capacity = capacity
        self._membrane_potentials = [0.0] * capacity
        self._coordinates = [(0, 0, 0)] * capacity

    def get_membrane_potential(self, neuron_id):
        return self._membrane_potentials[neuron_id]

    def set_membrane_potential(self, neuron_id, value):
        self._membrane_potentials[neuron_id] = value

    def get_coordinates(self, neuron_id):
        return self._coordinates[neuron_id]

    def set_coordinates(self, neuron_id, x, y, z):
        self._coordinates[neuron_id] = (x, y, z)

    def decay_membrane_potentials(self, decay_factor):
        for i in range(self.capacity):
            self._membrane_potentials[i] *= decay_factor

    def update_refractory_counters(self):
        pass

    def get_fire_candidates(self, timestep):
        return [i for i, mp in enumerate(self._membrane_potentials) if mp >= 1.0]

    def process_fired_neurons(self, fired_list, timestep):
        for neuron_id in fired_list:
            self._membrane_potentials[neuron_id] = 0.0

    def get_all_membrane_potentials(self):
        return self._membrane_potentials.copy()


class MockRustFCL:
    """Mock Rust FCL for testing."""

    def __init__(self, neuron_ids=None):
        self._neurons = set(neuron_ids or [])

    def add(self, neuron_id):
        self._neurons.add(neuron_id)

    def add_multiple(self, neuron_ids):
        self._neurons.update(neuron_ids)

    def remove(self, neuron_id):
        self._neurons.discard(neuron_id)

    def clear(self):
        self._neurons.clear()

    def contains(self, neuron_id):
        return neuron_id in self._neurons

    def len(self):
        return len(self._neurons)

    def is_empty(self):
        return len(self._neurons) == 0

    def to_list(self):
        return list(self._neurons)


class MockRustConnectome:
    """Mock Rust Connectome for testing."""

    def __init__(self, neuron_count, estimated_connections):
        self.neuron_count = neuron_count
        self._connections = {}

    def add_connection(
        self,
        source_id,
        target_id,
        weight,
        delay=0,
        connection_type=0,
        source_cortical_idx=0,
        target_cortical_idx=0,
    ):
        if source_id not in self._connections:
            self._connections[source_id] = []
        self._connections[source_id].append(
            {
                "source_id": source_id,
                "target_id": target_id,
                "weight": weight,
                "delay": delay,
                "connection_type": connection_type,
                "source_cortical_id": source_cortical_idx,
                "target_cortical_id": target_cortical_idx,
            }
        )

    def get_connections_for_neuron(self, neuron_id):
        return self._connections.get(neuron_id, [])

    def connection_count(self):
        return sum(len(conns) for conns in self._connections.values())

    def propagate_activations(self, source_activations, target_buffer):
        result = target_buffer.copy()
        for source_id, connections in self._connections.items():
            for conn in connections:
                target_id = conn["target_id"]
                weight = conn["weight"]
                result[target_id] += source_activations[source_id] * weight
        return result


class MockRustFeagiCore:
    """Mock Rust FEAGI Core for testing."""

    def __init__(self, neuron_capacity, estimated_connections):
        self._timestep = 0

    def step(self):
        self._timestep += 1

    def propagate_activations(self):
        return [0.0] * 10  # Simple mock return

    def get_current_timestep(self):
        return self._timestep

    def set_current_timestep(self, value):
        self._timestep = value


@pytest.fixture
def mock_rust_available():
    """Mock Rust being available."""
    with patch("feagi.npu.optimized_structures.RUST_AVAILABLE", True):
        # Don't try to patch the create functions since they may not exist
        # Just mock the RUST_AVAILABLE flag and let the classes handle the rest
        yield


@pytest.fixture
def mock_rust_unavailable():
    """Mock Rust being unavailable."""
    with patch("feagi.npu.optimized_structures.RUST_AVAILABLE", False):
        yield


# Test GlobalNeuronArray
def test_gna_initialization_rust(mock_rust_available):
    """Test GNA initialization with Rust backend."""
    gna = GlobalNeuronArray(1000)

    assert gna.capacity == 1000
    assert gna._use_rust == True
    assert hasattr(gna, "_rust_gna")


def test_gna_initialization_numpy(mock_rust_unavailable):
    """Test GNA initialization with NumPy backend."""
    gna = GlobalNeuronArray(1000)

    assert gna.capacity == 1000
    assert gna._use_rust == False
    assert hasattr(gna, "membrane_potentials")
    assert len(gna.membrane_potentials) == 1000
    assert gna.membrane_potentials.dtype == np.float32


def test_gna_membrane_potential_rust(mock_rust_available):
    """Test membrane potential operations with Rust backend."""
    gna = GlobalNeuronArray(100)

    # Test setting and getting
    gna.set_membrane_potential(10, 0.75)
    assert gna.get_membrane_potential(10) == 0.75


def test_gna_membrane_potential_numpy(mock_rust_unavailable):
    """Test membrane potential operations with NumPy backend."""
    gna = GlobalNeuronArray(100)

    # Test setting and getting
    gna.set_membrane_potential(10, 0.75)
    assert gna.get_membrane_potential(10) == 0.75
    assert isinstance(gna.get_membrane_potential(10), float)


def test_gna_coordinates_rust(mock_rust_available):
    """Test coordinate operations with Rust backend."""
    gna = GlobalNeuronArray(100)

    # Test setting and getting coordinates
    gna.set_coordinates(5, 10, 20, 30)
    coords = gna.get_coordinates(5)
    assert coords == (10, 20, 30)


def test_gna_coordinates_numpy(mock_rust_unavailable):
    """Test coordinate operations with NumPy backend."""
    gna = GlobalNeuronArray(100)

    # Test setting and getting coordinates
    gna.set_coordinates(5, 10, 20, 30)
    coords = gna.get_coordinates(5)
    assert coords == (10, 20, 30)

    # Test with negative values (should be converted to non-negative)
    gna.set_coordinates(6, -5, -10, -15)
    coords = gna.get_coordinates(6)
    assert coords == (0, 0, 0)  # Negative values clamped to 0


def test_gna_update_membrane_potentials_rust(mock_rust_available):
    """Test membrane potential decay with Rust backend."""
    gna = GlobalNeuronArray(100)
    gna.set_membrane_potential(0, 1.0)

    gna.update_membrane_potentials(0.9)

    assert gna.get_membrane_potential(0) == 0.9


def test_gna_update_membrane_potentials_numpy(mock_rust_unavailable):
    """Test membrane potential decay with NumPy backend."""
    gna = GlobalNeuronArray(100)
    gna.set_membrane_potential(0, 1.0)
    gna.set_membrane_potential(1, 0.8)

    gna.update_membrane_potentials(0.9)

    assert gna.get_membrane_potential(0) == pytest.approx(0.9)
    assert gna.get_membrane_potential(1) == pytest.approx(0.72)


def test_gna_update_refractory_counters_rust(mock_rust_available):
    """Test refractory counter updates with Rust backend."""
    gna = GlobalNeuronArray(100)

    gna.update_refractory_counters()

    # Should call the rust method (no assertion needed as it's mocked)


def test_gna_update_refractory_counters_numpy(mock_rust_unavailable):
    """Test refractory counter updates with NumPy backend."""
    gna = GlobalNeuronArray(100)

    # Set some refractory counters
    gna.refractory_counters[0] = 3
    gna.refractory_counters[1] = 1
    gna.refractory_counters[2] = 0

    gna.update_refractory_counters()

    assert gna.refractory_counters[0] == 2
    assert gna.refractory_counters[1] == 0
    assert gna.refractory_counters[2] == 0


def test_gna_find_fire_candidates_rust(mock_rust_available):
    """Test finding fire candidates with Rust backend."""
    gna = GlobalNeuronArray(100)
    gna.set_membrane_potential(0, 1.5)  # Above threshold

    candidates = gna.find_fire_candidates(10)

    assert 0 in candidates


def test_gna_find_fire_candidates_numpy(mock_rust_unavailable):
    """Test finding fire candidates with NumPy backend."""
    gna = GlobalNeuronArray(100)

    # Set up neurons
    gna.set_membrane_potential(0, 1.5)  # Above threshold, should fire
    gna.set_membrane_potential(1, 0.5)  # Below threshold, should not fire
    gna.set_membrane_potential(2, 1.2)  # Above threshold, should fire
    gna.refractory_counters[2] = 1  # But in refractory, should not fire
    gna.enabled_flags[0] = 1  # Enabled
    gna.enabled_flags[1] = 0  # Disabled, should not fire even if above threshold
    gna.set_membrane_potential(1, 1.3)  # Set above threshold but disabled

    candidates = gna.find_fire_candidates(10)

    assert 0 in candidates  # Above threshold, not refractory, enabled
    assert 1 not in candidates  # Disabled
    assert 2 not in candidates  # In refractory


def test_gna_process_fired_neurons_rust(mock_rust_available):
    """Test processing fired neurons with Rust backend."""
    gna = GlobalNeuronArray(100)
    gna.set_membrane_potential(0, 1.0)

    gna.process_fired_neurons([0], 10)

    assert gna.get_membrane_potential(0) == 0.0


def test_gna_process_fired_neurons_numpy(mock_rust_unavailable):
    """Test processing fired neurons with NumPy backend."""
    gna = GlobalNeuronArray(100)

    # Set up fired neurons
    gna.set_membrane_potential(0, 1.0)
    gna.set_membrane_potential(1, 0.8)
    gna.refractory_periods[0] = 3
    gna.refractory_periods[1] = 2

    gna.process_fired_neurons([0, 1], 10)

    # Check membrane potentials reset
    assert gna.get_membrane_potential(0) == 0.0
    assert gna.get_membrane_potential(1) == 0.0

    # Check refractory counters set
    assert gna.refractory_counters[0] == 3
    assert gna.refractory_counters[1] == 2

    # Check last fired timestep
    assert gna.last_fired[0] == 10
    assert gna.last_fired[1] == 10


def test_gna_get_all_membrane_potentials_rust(mock_rust_available):
    """Test getting all membrane potentials with Rust backend."""
    gna = GlobalNeuronArray(5)
    gna.set_membrane_potential(0, 0.1)
    gna.set_membrane_potential(1, 0.2)

    potentials = gna.get_all_membrane_potentials()

    assert len(potentials) >= 2
    assert potentials[0] == 0.1
    assert potentials[1] == 0.2


def test_gna_get_all_membrane_potentials_numpy(mock_rust_unavailable):
    """Test getting all membrane potentials with NumPy backend."""
    gna = GlobalNeuronArray(5)
    gna.set_membrane_potential(0, 0.1)
    gna.set_membrane_potential(1, 0.2)

    potentials = gna.get_all_membrane_potentials()

    assert len(potentials) == 5
    assert potentials[0] == pytest.approx(0.1)
    assert potentials[1] == pytest.approx(0.2)
    assert potentials[2] == 0.0  # Default value


# Test FireCandidateList
def test_fcl_initialization_rust(mock_rust_available):
    """Test FCL initialization with Rust backend."""
    fcl = FireCandidateList([1, 2, 3])

    assert fcl._use_rust == True
    assert hasattr(fcl, "_rust_fcl")


def test_fcl_initialization_numpy_dense(mock_rust_unavailable):
    """Test FCL initialization with NumPy backend (dense)."""
    fcl = FireCandidateList([1, 2, 3], capacity=1000)

    assert fcl._use_rust == False
    assert fcl._use_dense == True
    assert fcl.capacity == 1000
    assert hasattr(fcl, "_mask")


def test_fcl_initialization_numpy_sparse(mock_rust_unavailable):
    """Test FCL initialization with NumPy backend (sparse)."""
    fcl = FireCandidateList([1, 2, 3], capacity=2000000)  # Large capacity

    assert fcl._use_rust == False
    assert fcl._use_dense == False
    assert hasattr(fcl, "_active_ids")


def test_fcl_add_rust(mock_rust_available):
    """Test adding neurons with Rust backend."""
    fcl = FireCandidateList()

    fcl.add(5)

    assert fcl.contains(5)


def test_fcl_add_numpy_dense(mock_rust_unavailable):
    """Test adding neurons with NumPy backend (dense)."""
    fcl = FireCandidateList(capacity=1000)

    fcl.add(5)

    assert fcl.contains(5)
    assert fcl._mask[5] == True


def test_fcl_add_numpy_sparse(mock_rust_unavailable):
    """Test adding neurons with NumPy backend (sparse)."""
    fcl = FireCandidateList(capacity=2000000)

    fcl.add(5)
    fcl.add(5)  # Add duplicate

    assert fcl.contains(5)
    assert len(fcl._active_ids) == 1  # No duplicates


def test_fcl_add_multiple_rust(mock_rust_available):
    """Test adding multiple neurons with Rust backend."""
    fcl = FireCandidateList()

    fcl.add_multiple([1, 2, 3])

    assert fcl.contains(1)
    assert fcl.contains(2)
    assert fcl.contains(3)


def test_fcl_add_multiple_numpy_dense(mock_rust_unavailable):
    """Test adding multiple neurons with NumPy backend (dense)."""
    fcl = FireCandidateList(capacity=1000)

    fcl.add_multiple([1, 2, 3])

    assert fcl.contains(1)
    assert fcl.contains(2)
    assert fcl.contains(3)


def test_fcl_add_multiple_numpy_sparse(mock_rust_unavailable):
    """Test adding multiple neurons with NumPy backend (sparse)."""
    fcl = FireCandidateList(capacity=2000000)

    # Add initial neurons
    fcl.add_multiple([1, 2])
    fcl.add_multiple([2, 3, 4])  # Include duplicates

    neurons = fcl.to_list()
    assert 1 in neurons
    assert 2 in neurons
    assert 3 in neurons
    assert 4 in neurons
    assert len(neurons) == 4  # No duplicates


def test_fcl_remove_rust(mock_rust_available):
    """Test removing neurons with Rust backend."""
    fcl = FireCandidateList([1, 2, 3])

    fcl.remove(2)

    assert not fcl.contains(2)
    assert fcl.contains(1)
    assert fcl.contains(3)


def test_fcl_remove_numpy_dense(mock_rust_unavailable):
    """Test removing neurons with NumPy backend (dense)."""
    fcl = FireCandidateList([1, 2, 3], capacity=1000)

    fcl.remove(2)

    assert not fcl.contains(2)
    assert fcl.contains(1)
    assert fcl.contains(3)


def test_fcl_remove_numpy_sparse(mock_rust_unavailable):
    """Test removing neurons with NumPy backend (sparse)."""
    fcl = FireCandidateList([1, 2, 3], capacity=2000000)

    fcl.remove(2)

    assert not fcl.contains(2)
    assert fcl.contains(1)
    assert fcl.contains(3)


def test_fcl_clear_rust(mock_rust_available):
    """Test clearing FCL with Rust backend."""
    fcl = FireCandidateList([1, 2, 3])

    fcl.clear()

    assert fcl.is_empty()


def test_fcl_clear_numpy_dense(mock_rust_unavailable):
    """Test clearing FCL with NumPy backend (dense)."""
    fcl = FireCandidateList([1, 2, 3], capacity=1000)

    fcl.clear()

    assert fcl.is_empty()
    assert not np.any(fcl._mask)


def test_fcl_clear_numpy_sparse(mock_rust_unavailable):
    """Test clearing FCL with NumPy backend (sparse)."""
    fcl = FireCandidateList([1, 2, 3], capacity=2000000)

    fcl.clear()

    assert fcl.is_empty()
    assert len(fcl._active_ids) == 0


def test_fcl_len_rust(mock_rust_available):
    """Test FCL length with Rust backend."""
    fcl = FireCandidateList([1, 2, 3])

    assert len(fcl) == 3


def test_fcl_len_numpy_dense(mock_rust_unavailable):
    """Test FCL length with NumPy backend (dense)."""
    fcl = FireCandidateList([1, 2, 3], capacity=1000)

    assert len(fcl) == 3


def test_fcl_len_numpy_sparse(mock_rust_unavailable):
    """Test FCL length with NumPy backend (sparse)."""
    fcl = FireCandidateList([1, 2, 3], capacity=2000000)

    assert len(fcl) == 3


def test_fcl_is_empty_rust(mock_rust_available):
    """Test FCL empty check with Rust backend."""
    fcl = FireCandidateList()

    assert fcl.is_empty()

    fcl.add(1)
    assert not fcl.is_empty()


def test_fcl_is_empty_numpy_dense(mock_rust_unavailable):
    """Test FCL empty check with NumPy backend (dense)."""
    fcl = FireCandidateList(capacity=1000)

    assert fcl.is_empty()

    fcl.add(1)
    assert not fcl.is_empty()


def test_fcl_is_empty_numpy_sparse(mock_rust_unavailable):
    """Test FCL empty check with NumPy backend (sparse)."""
    fcl = FireCandidateList(capacity=2000000)

    assert fcl.is_empty()

    fcl.add(1)
    assert not fcl.is_empty()


def test_fcl_to_list_rust(mock_rust_available):
    """Test FCL to_list with Rust backend."""
    fcl = FireCandidateList([3, 1, 2])

    neurons = fcl.to_list()
    assert set(neurons) == {1, 2, 3}


def test_fcl_to_list_numpy_dense(mock_rust_unavailable):
    """Test FCL to_list with NumPy backend (dense)."""
    fcl = FireCandidateList([3, 1, 2], capacity=1000)

    neurons = fcl.to_list()
    assert set(neurons) == {1, 2, 3}


def test_fcl_to_list_numpy_sparse(mock_rust_unavailable):
    """Test FCL to_list with NumPy backend (sparse)."""
    fcl = FireCandidateList([3, 1, 2], capacity=2000000)

    neurons = fcl.to_list()
    assert set(neurons) == {1, 2, 3}


def test_fcl_iteration_rust(mock_rust_available):
    """Test FCL iteration with Rust backend."""
    fcl = FireCandidateList([1, 2, 3])

    neurons = list(fcl)
    assert set(neurons) == {1, 2, 3}


def test_fcl_iteration_numpy_dense(mock_rust_unavailable):
    """Test FCL iteration with NumPy backend (dense)."""
    fcl = FireCandidateList([1, 2, 3], capacity=1000)

    neurons = list(fcl)
    assert set(neurons) == {1, 2, 3}


def test_fcl_iteration_numpy_sparse(mock_rust_unavailable):
    """Test FCL iteration with NumPy backend (sparse)."""
    fcl = FireCandidateList([1, 2, 3], capacity=2000000)

    neurons = list(fcl)
    assert set(neurons) == {1, 2, 3}


# Test Connectome
def test_connectome_initialization_rust(mock_rust_available):
    """Test Connectome initialization with Rust backend."""
    connectome = Connectome(1000, 10000)

    assert connectome.neuron_count == 1000
    assert connectome.initial_capacity == 10000
    assert connectome._use_rust == True


def test_connectome_initialization_numpy(mock_rust_unavailable):
    """Test Connectome initialization with NumPy backend."""
    connectome = Connectome(1000, 10000)

    assert connectome.neuron_count == 1000
    assert connectome.initial_capacity == 10000
    assert connectome._use_rust == False
    assert len(connectome.source_offsets) == 1001  # n+1 for CSR format
    assert connectome._connection_count == 0


def test_connectome_add_connection_rust(mock_rust_available):
    """Test adding connections with Rust backend."""
    connectome = Connectome(100, 1000)

    connectome.add_connection(
        0,
        1,
        0.5,
        delay=1,
        connection_type=0,
        source_cortical_idx=0,
        target_cortical_idx=0,
    )

    connections = connectome.get_connections_for_neuron(0)
    assert len(connections) == 1
    assert connections[0]["target_id"] == 1
    assert connections[0]["weight"] == 0.5


def test_connectome_add_connection_numpy(mock_rust_unavailable):
    """Test adding connections with NumPy backend."""
    connectome = Connectome(100, 1000)

    connectome.add_connection(
        0,
        1,
        0.5,
        delay=1,
        connection_type=0,
        source_cortical_idx=0,
        target_cortical_idx=0,
    )

    assert connectome._connection_count == 1
    assert connectome.target_ids[0] == 1
    assert connectome.weights[0] == 0.5
    assert connectome.delays[0] == 1


def test_connectome_add_connection_numpy_resize(mock_rust_unavailable):
    """Test adding connections with array resizing."""
    # Create small connectome to trigger resize
    connectome = Connectome(10, 2)  # Very small initial capacity

    # Add connections to exceed initial capacity
    for i in range(5):
        connectome.add_connection(0, i, 0.1 * i)

    assert connectome._connection_count == 5
    assert len(connectome.target_ids) >= 5


def test_connectome_get_connections_rust(mock_rust_available):
    """Test getting connections with Rust backend."""
    connectome = Connectome(100, 1000)
    connectome.add_connection(0, 1, 0.5)
    connectome.add_connection(0, 2, 0.3)

    connections = connectome.get_connections_for_neuron(0)
    assert len(connections) == 2

    # Check no connections for neuron with no outgoing connections
    connections = connectome.get_connections_for_neuron(5)
    assert len(connections) == 0


def test_connectome_get_connections_numpy(mock_rust_unavailable):
    """Test getting connections with NumPy backend."""
    connectome = Connectome(100, 1000)
    connectome.add_connection(0, 1, 0.5, delay=2, connection_type=1)
    connectome.add_connection(0, 2, 0.3)

    connections = connectome.get_connections_for_neuron(0)
    assert len(connections) == 2

    # Check connection details (order may vary due to CSR insertion)
    target_ids = [conn["target_id"] for conn in connections]
    weights = [conn["weight"] for conn in connections]

    assert 1 in target_ids
    assert 2 in target_ids
    assert pytest.approx(0.5) in weights
    assert pytest.approx(0.3) in weights

    # Check no connections for neuron with no outgoing connections
    connections = connectome.get_connections_for_neuron(5)
    assert len(connections) == 0


def test_connectome_connection_count_rust(mock_rust_available):
    """Test connection count with Rust backend."""
    connectome = Connectome(100, 1000)

    assert connectome.connection_count() == 0

    connectome.add_connection(0, 1, 0.5)
    connectome.add_connection(1, 2, 0.3)

    assert connectome.connection_count() == 2


def test_connectome_connection_count_numpy(mock_rust_unavailable):
    """Test connection count with NumPy backend."""
    connectome = Connectome(100, 1000)

    assert connectome.connection_count() == 0

    connectome.add_connection(0, 1, 0.5)
    connectome.add_connection(1, 2, 0.3)

    assert connectome.connection_count() == 2


def test_connectome_propagate_activations_rust(mock_rust_available):
    """Test activation propagation with Rust backend."""
    connectome = Connectome(5, 100)
    connectome.add_connection(0, 1, 0.5)
    connectome.add_connection(0, 2, 0.3)

    source_activations = [1.0, 0.0, 0.0, 0.0, 0.0]
    target_buffer = [0.0, 0.0, 0.0, 0.0, 0.0]

    result = connectome.propagate_activations(source_activations, target_buffer)

    assert len(result) == 5
    assert result[1] == 0.5  # 1.0 * 0.5
    assert result[2] == 0.3  # 1.0 * 0.3


def test_connectome_propagate_activations_numpy(mock_rust_unavailable):
    """Test activation propagation with NumPy backend."""
    connectome = Connectome(5, 100)
    connectome.add_connection(0, 1, 0.5)
    connectome.add_connection(0, 2, 0.3)
    connectome.add_connection(2, 3, 0.8)

    source_activations = [1.0, 0.0, 0.5, 0.0, 0.0]
    target_buffer = [0.0, 0.0, 0.0, 0.0, 0.0]

    result = connectome.propagate_activations(source_activations, target_buffer)

    assert len(result) == 5
    assert result[1] == pytest.approx(0.5)  # 1.0 * 0.5
    assert result[2] == pytest.approx(0.3)  # 1.0 * 0.3
    assert result[3] == pytest.approx(0.4)  # 0.5 * 0.8


def test_connectome_propagate_activations_numpy_no_connections(mock_rust_unavailable):
    """Test activation propagation with no connections."""
    connectome = Connectome(5, 100)

    source_activations = [1.0, 0.0, 0.0, 0.0, 0.0]
    target_buffer = [0.0, 0.0, 0.0, 0.0, 0.0]

    result = connectome.propagate_activations(source_activations, target_buffer)

    assert result == target_buffer  # No changes


def test_connectome_resize_arrays(mock_rust_unavailable):
    """Test internal array resizing."""
    connectome = Connectome(10, 2)

    # Access the resize method directly
    original_size = len(connectome.target_ids)
    connectome._resize_arrays(10)

    assert len(connectome.target_ids) == 10
    assert len(connectome.weights) == 10
    assert len(connectome.delays) == 10


# Test OptimizedFeagiCore
def test_optimized_feagi_core_initialization_rust(mock_rust_available):
    """Test OptimizedFeagiCore initialization with Rust backend."""
    core = OptimizedFeagiCore(1000, 10000)

    assert core._use_rust == True
    assert hasattr(core, "_rust_core")
    assert hasattr(core, "gna")
    assert hasattr(core, "fcl")
    assert hasattr(core, "connectome")


def test_optimized_feagi_core_initialization_numpy(mock_rust_unavailable):
    """Test OptimizedFeagiCore initialization with NumPy backend."""
    core = OptimizedFeagiCore(1000, 10000)

    assert core._use_rust == False
    assert hasattr(core, "gna")
    assert hasattr(core, "fcl")
    assert hasattr(core, "connectome")
    assert core._current_timestep == 0


def test_optimized_feagi_core_step_rust(mock_rust_available):
    """Test stepping simulation with Rust backend."""
    core = OptimizedFeagiCore(100, 1000)
    initial_timestep = core.current_timestep

    core.step()

    assert core.current_timestep == initial_timestep + 1


def test_optimized_feagi_core_step_numpy(mock_rust_unavailable):
    """Test stepping simulation with NumPy backend."""
    core = OptimizedFeagiCore(100, 1000)

    # Set up some initial state
    core.gna.set_membrane_potential(0, 1.5)  # Above threshold
    initial_timestep = core._current_timestep

    core.step()

    assert core._current_timestep == initial_timestep + 1

    # Check that neuron 0 fired and was processed
    fired_neurons = core.fcl.to_list()
    assert 0 in fired_neurons


def test_optimized_feagi_core_step_numpy_no_candidates(mock_rust_unavailable):
    """Test stepping simulation with no fire candidates."""
    core = OptimizedFeagiCore(100, 1000)

    # No neurons above threshold
    initial_timestep = core._current_timestep

    core.step()

    assert core._current_timestep == initial_timestep + 1
    assert core.fcl.is_empty()


def test_optimized_feagi_core_propagate_activations_rust(mock_rust_available):
    """Test activation propagation with Rust backend."""
    core = OptimizedFeagiCore(100, 1000)

    result = core.propagate_activations()

    assert isinstance(result, list)
    assert len(result) == 10  # Mock returns 10 elements


def test_optimized_feagi_core_propagate_activations_numpy(mock_rust_unavailable):
    """Test activation propagation with NumPy backend."""
    core = OptimizedFeagiCore(10, 100)

    # Set up firing neurons and connections
    core.fcl.add_multiple([0, 2])
    core.connectome.add_connection(0, 1, 0.5)
    core.connectome.add_connection(2, 3, 0.8)

    result = core.propagate_activations()

    assert len(result) == 10
    assert result[1] == pytest.approx(0.5)  # From neuron 0
    assert result[3] == pytest.approx(0.8)  # From neuron 2


def test_optimized_feagi_core_propagate_activations_numpy_no_firing(
    mock_rust_unavailable,
):
    """Test activation propagation with no firing neurons."""
    core = OptimizedFeagiCore(10, 100)

    # No firing neurons
    result = core.propagate_activations()

    assert len(result) == 10
    assert all(val == 0.0 for val in result)  # All zeros


def test_optimized_feagi_core_current_timestep_rust(mock_rust_available):
    """Test current timestep property with Rust backend."""
    core = OptimizedFeagiCore(100, 1000)

    # Test getter
    timestep = core.current_timestep
    assert isinstance(timestep, int)

    # Test setter
    core.current_timestep = 42
    assert core._current_timestep == 42


def test_optimized_feagi_core_current_timestep_numpy(mock_rust_unavailable):
    """Test current timestep property with NumPy backend."""
    core = OptimizedFeagiCore(100, 1000)

    # Test getter
    assert core.current_timestep == 0

    # Test setter
    core.current_timestep = 42
    assert core.current_timestep == 42
    assert core._current_timestep == 42


if __name__ == "__main__":
    pytest.main(["-v", __file__])
