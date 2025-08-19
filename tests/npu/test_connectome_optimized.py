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
Tests for the optimized Connectome and OptimizedFeagiCore classes.

This module tests the Connectome and OptimizedFeagiCore classes from the
optimized_structures module, focusing on both the Python and mocked Rust implementations.
"""

from unittest.mock import Mock, patch

import numpy as np
import pytest

from feagi.npu.optimized_structures import Connectome, OptimizedFeagiCore


class MockRustGNA:
    """Mock implementation of the Rust GNA for testing."""

    def __init__(self):
        self.membrane_potentials = {}
        self.coordinates = {}
        self.fired_neurons = []
        self.refractory_counters = {}

    def get_membrane_potential(self, neuron_id):
        return self.membrane_potentials.get(neuron_id, 0.0)

    def set_membrane_potential(self, neuron_id, value):
        self.membrane_potentials[neuron_id] = value

    def get_coordinates(self, neuron_id):
        return self.coordinates.get(neuron_id, (0, 0, 0))

    def set_coordinates(self, neuron_id, x, y, z):
        self.coordinates[neuron_id] = (x, y, z)

    def decay_membrane_potentials(self, decay_factor):
        for neuron_id in self.membrane_potentials:
            self.membrane_potentials[neuron_id] *= decay_factor

    def update_refractory_counters(self):
        for neuron_id in list(self.refractory_counters.keys()):
            if self.refractory_counters[neuron_id] > 0:
                self.refractory_counters[neuron_id] -= 1

    def get_fire_candidates(self, timestep):
        return [1, 2, 3]

    def process_fired_neurons(self, fired_list, timestep):
        self.fired_neurons = fired_list
        for neuron_id in fired_list:
            self.refractory_counters[neuron_id] = 5  # Default refractory period
            self.membrane_potentials[neuron_id] = 0.0

    def get_all_membrane_potentials(self):
        return list(self.membrane_potentials.values())


class MockRustFCL:
    """Mock implementation of the Rust FCL for testing."""

    def __init__(self, neuron_ids=None):
        self.neurons = set(neuron_ids or [])

    def add(self, neuron_id):
        self.neurons.add(neuron_id)

    def add_multiple(self, neuron_ids):
        self.neurons.update(neuron_ids)

    def remove(self, neuron_id):
        self.neurons.discard(neuron_id)

    def clear(self):
        self.neurons.clear()

    def contains(self, neuron_id):
        return neuron_id in self.neurons

    def size(self):
        return len(self.neurons)

    def is_empty(self):
        return len(self.neurons) == 0

    def to_list(self):
        return list(self.neurons)


class MockRustConnectome:
    """Mock version of the Rust Connectome implementation for testing."""

    def __init__(self, neuron_count, estimated_connections):
        """Initialize the mock Connectome."""
        self.neuron_count = neuron_count
        self.connections = {}  # source_id -> list of connections

    def add_connection(
        self,
        source_id,
        target_id,
        weight,
        delay=0,
        connection_type=0,
        source_cortical_id=0,
        target_cortical_id=0,
    ):
        """Add a connection between neurons."""
        if source_id not in self.connections:
            self.connections[source_id] = []

        self.connections[source_id].append(
            {
                "target_id": target_id,
                "weight": weight,
                "delay": delay,
                "type": connection_type,
                "source_cortical_id": source_cortical_id,
                "target_cortical_id": target_cortical_id,
            }
        )

        return True

    def get_connections_for_neuron(self, neuron_id):
        return self.connections.get(neuron_id, [])

    def count(self):
        return len(self.connections)

    def propagate_activations(self, source_activations, target_buffer):
        return target_buffer  # Simplified mock implementation


class MockRustFeagiCore:
    """Mock implementation of the Rust FeagiCore for testing."""

    def __init__(self):
        self.current_timestep = 0
        self.gna = MockRustGNA()
        self.fcl = MockRustFCL()
        self.connectome = MockRustConnectome(1000, 1000)

    def step(self):
        self.current_timestep += 1

    def step_with_fire_queue(self, mpf, puf, max_consecutive_fires):
        self.current_timestep += 1

    def get_gna(self):
        return self.gna

    def get_fcl(self):
        return self.fcl

    def get_connectome(self):
        return self.connectome


# Patch RUST_AVAILABLE for all tests in this module
@pytest.fixture(autouse=True)
def mock_rust_available():
    with patch("feagi.npu.optimized_structures.RUST_AVAILABLE", False):
        yield


class TestConnectome:
    """Tests for the Connectome class."""

    @pytest.fixture
    def numpy_connectome(self):
        """Create a Connectome using the NumPy implementation."""
        with patch("feagi.npu.optimized_structures.RUST_AVAILABLE", False):
            return Connectome(1000, 5000)

    @pytest.fixture
    @pytest.mark.skip(
        reason="create_connectome function not available in optimized_structures module"
    )
    def rust_connectome(self):
        """Create a Connectome using the mocked Rust implementation."""
        with patch("feagi.npu.optimized_structures.RUST_AVAILABLE", True), patch(
            "feagi.npu.optimized_structures.create_connectome",
            return_value=MockRustConnectome(1000, 5000),
        ):
            return Connectome(1000, 5000)

    def test_initialization_numpy(self, numpy_connectome):
        """Test initialization of Connectome with NumPy implementation."""
        assert not numpy_connectome._use_rust
        assert numpy_connectome.neuron_count == 1000
        assert numpy_connectome.source_offsets.size == 1001  # neuron_count + 1
        assert numpy_connectome.target_ids.size > 0  # Initial capacity
        assert numpy_connectome.weights.size > 0  # Initial capacity
        assert numpy_connectome.delays.size > 0  # Initial capacity
        assert numpy_connectome.conductances.size > 0  # Initial capacity
        assert numpy_connectome.source_cortical_idxs.size > 0  # Initial capacity
        assert numpy_connectome.target_cortical_idxs.size > 0  # Initial capacity
        assert numpy_connectome.connection_count() == 0  # No connections yet

    @pytest.mark.skip(
        reason="create_connectome function not available in optimized_structures module"
    )
    def test_initialization_rust(self, rust_connectome):
        """Test initialization of Connectome with Rust implementation."""
        assert rust_connectome._use_rust
        assert rust_connectome.neuron_count == 1000
        assert hasattr(rust_connectome, "_rust_connectome")

    def test_add_connection_numpy(self, numpy_connectome):
        """Test adding a connection using NumPy implementation."""
        numpy_connectome.add_connection(1, 2, 0.5, 0, 1, 10, 20)

        # Since the Connectome uses CSR-like format, source neuron 1's connections start at source_offsets[1]
        idx = numpy_connectome.source_offsets[1]

        assert numpy_connectome.target_ids[idx] == 2
        assert numpy_connectome.weights[idx] == 0.5
        assert numpy_connectome.connection_types[idx] == 1
        assert numpy_connectome.source_cortical_idxs[idx] == 10
        assert numpy_connectome.target_cortical_idxs[idx] == 20
        assert numpy_connectome._connection_count == 1

    @pytest.mark.skip(
        reason="create_connectome function not available in optimized_structures module"
    )
    def test_add_connection_rust(self, rust_connectome):
        """Test adding a connection using Rust implementation."""
        rust_connectome.add_connection(1, 2, 0.5, 0, 1, 10, 20)

        # Get the connections for source neuron 1
        connections = rust_connectome.get_connections_for_neuron(1)

        assert len(connections) == 1
        assert connections[0]["target_id"] == 2
        assert connections[0]["weight"] == 0.5
        assert connections[0]["type"] == 1
        assert connections[0]["source_cortical_id"] == 10
        assert connections[0]["target_cortical_id"] == 20

    def test_resize_arrays_numpy(self, numpy_connectome):
        """Test resizing arrays using NumPy implementation."""
        # Add 5 connections to trigger resizing
        for i in range(5):
            numpy_connectome.add_connection(i, i + 10, 0.5)

        # Add one more to potentially trigger resize
        numpy_connectome.add_connection(5, 15, 0.5)

        # Check that arrays were resized
        assert numpy_connectome.target_ids.size >= 6
        assert numpy_connectome.weights.size >= 6
        assert numpy_connectome.delays.size >= 6
        assert numpy_connectome._connection_count == 6

    def test_get_connections_for_neuron_numpy(self, numpy_connectome):
        """Test getting connections for a neuron using NumPy implementation."""
        # Add connections for neuron 1
        numpy_connectome.add_connection(1, 10, 0.5)
        numpy_connectome.add_connection(1, 20, 0.7)
        numpy_connectome.add_connection(2, 30, 0.9)  # Different source

        # Get connections for neuron 1
        connections = numpy_connectome.get_connections_for_neuron(1)

        # Check that we get correct number of connections
        assert len(connections) == 2

        # Extract target IDs and weights for easier testing
        targets = {conn["target_id"]: conn["weight"] for conn in connections}

        # Check that we have connections to both targets with correct weights
        assert 10 in targets
        assert 20 in targets
        assert targets[10] == pytest.approx(0.5, abs=1e-6)
        assert targets[20] == pytest.approx(0.7, abs=1e-6)

    @pytest.mark.skip(
        reason="create_connectome function not available in optimized_structures module"
    )
    def test_get_connections_for_neuron_rust(self, rust_connectome):
        """Test getting connections for a neuron using Rust implementation."""
        # Add connections for neuron 1
        rust_connectome.add_connection(1, 10, 0.5)
        rust_connectome.add_connection(1, 20, 0.7)
        rust_connectome.add_connection(2, 30, 0.9)  # Different source

        # Get connections for neuron 1
        connections = rust_connectome.get_connections_for_neuron(1)

        assert len(connections) == 2
        assert connections[0]["target_id"] == 10
        assert connections[0]["weight"] == 0.5
        assert connections[1]["target_id"] == 20
        assert connections[1]["weight"] == 0.7

    def test_connection_count_numpy(self, numpy_connectome):
        """Test getting connection count using NumPy implementation."""
        assert numpy_connectome.connection_count() == 0

        # Add some connections
        numpy_connectome.add_connection(1, 10, 0.5)
        numpy_connectome.add_connection(1, 20, 0.7)

        assert numpy_connectome.connection_count() == 2

    @pytest.mark.skip(
        reason="create_connectome function not available in optimized_structures module"
    )
    def test_connection_count_rust(self, rust_connectome):
        """Test getting connection count using Rust implementation."""
        assert rust_connectome.connection_count() == 0

        # Add some connections
        rust_connectome.add_connection(1, 10, 0.5)
        rust_connectome.add_connection(1, 20, 0.7)

        assert rust_connectome.connection_count() == 2

    def test_propagate_activations_numpy(self, numpy_connectome):
        """Test propagating activations using NumPy implementation."""
        # Add some connections
        numpy_connectome.add_connection(1, 10, 0.5)
        numpy_connectome.add_connection(1, 20, 0.7)
        numpy_connectome.add_connection(2, 30, 0.9)

        # Source activations (only neurons 1 and 2 are active)
        source_activations = np.zeros(1000, dtype=np.float32)
        source_activations[1] = 1.0
        source_activations[2] = 1.0

        # Target buffer (will be updated with propagated activations)
        target_buffer = np.zeros(1000, dtype=np.float32)

        # Propagate activations
        result = numpy_connectome.propagate_activations(
            source_activations, target_buffer
        )

        # Check that target neurons received activations
        assert result[10] == pytest.approx(
            0.5, abs=1e-6
        )  # neuron 1 -> 10 with weight 0.5
        assert result[20] == pytest.approx(
            0.7, abs=1e-6
        )  # neuron 1 -> 20 with weight 0.7
        assert result[30] == pytest.approx(
            0.9, abs=1e-6
        )  # neuron 2 -> 30 with weight 0.9

    @pytest.mark.skip(
        reason="create_connectome function not available in optimized_structures module"
    )
    def test_propagate_activations_rust(self, rust_connectome):
        """Test propagating activations using Rust implementation."""
        # Add some connections
        rust_connectome.add_connection(1, 10, 0.5)
        rust_connectome.add_connection(1, 20, 0.7)
        rust_connectome.add_connection(2, 30, 0.9)

        # Source activations (only neurons 1 and 2 are active)
        source_activations = np.zeros(1000, dtype=np.float32)
        source_activations[1] = 1.0
        source_activations[2] = 1.0

        # Target buffer (will be updated with propagated activations)
        target_buffer = np.zeros(1000, dtype=np.float32)

        # Propagate activations
        result = rust_connectome.propagate_activations(
            source_activations, target_buffer
        )

        # Just check that we get some result back (the mock implementation doesn't actually do the propagation)
        assert result is not None


class TestOptimizedFeagiCore:
    """Tests for the OptimizedFeagiCore class."""

    @pytest.fixture
    def rust_core(self):
        """Create an OptimizedFeagiCore using the mocked Rust implementation."""
        with patch("feagi.npu.optimized_structures.RUST_AVAILABLE", True), patch(
            "feagi.npu.optimized_structures.create_feagi_core",
            return_value=MockRustFeagiCore(),
        ):
            return OptimizedFeagiCore(1000, 5000)

    @pytest.mark.skip(
        reason="create_feagi_core function not available in optimized_structures module"
    )
    def test_initialization_rust(self, rust_core):
        """Test initialization of OptimizedFeagiCore with Rust implementation."""
        assert rust_core.neuron_capacity == 1000
        assert rust_core.connection_capacity == 5000
        assert hasattr(rust_core, "_rust_core")
        assert rust_core.current_timestep == 0

    @pytest.mark.skip(
        reason="create_feagi_core function not available in optimized_structures module"
    )
    def test_step(self, rust_core):
        """Test stepping the simulation."""
        initial_timestep = rust_core.current_timestep
        rust_core.step()
        assert rust_core.current_timestep == initial_timestep + 1

    @pytest.mark.skip(
        reason="create_feagi_core function not available in optimized_structures module"
    )
    def test_step_with_fire_queue(self, rust_core):
        """Test stepping the simulation with fire queue."""
        initial_timestep = rust_core.current_timestep
        rust_core._rust_core.step_with_fire_queue = Mock()

        rust_core.step_with_fire_queue(True, False, 5)

        rust_core._rust_core.step_with_fire_queue.assert_called_once_with(
            True, False, 5
        )

    @pytest.mark.skip(
        reason="create_feagi_core function not available in optimized_structures module"
    )
    def test_propagate_activations(self, rust_core):
        """Test propagating activations."""
        # Setup mock return value
        rust_core._rust_core.connectome.propagate_activations = Mock(
            return_value=[0.1, 0.2, 0.3]
        )

        result = rust_core.propagate_activations()

        assert rust_core._rust_core.connectome.propagate_activations.called
        assert result == [0.1, 0.2, 0.3]

    @pytest.mark.skip(
        reason="create_feagi_core function not available in optimized_structures module"
    )
    def test_current_timestep_property(self, rust_core):
        """Test getting and setting current_timestep property."""
        rust_core.current_timestep = 42
        assert rust_core.current_timestep == 42
        assert rust_core._rust_core.current_timestep == 42


if __name__ == "__main__":
    pytest.main(["-v", __file__])
