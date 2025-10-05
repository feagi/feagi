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

import pytest

# Try to import the optimized structures
try:
    from feagi.npu.optimized_structures import (
        Connectome,
        FireCandidateList,
        GlobalNeuronArray,
        OptimizedFeagiCore,
    )

    OPTIMIZED_AVAILABLE = True
except ImportError:
    OPTIMIZED_AVAILABLE = False
    pytest.skip("Optimized structures not available", allow_module_level=True)


class TestGlobalNeuronArray:
    @pytest.fixture
    def gna(self):
        """Create a GNA with 1000 neurons."""
        return GlobalNeuronArray(1000)

    def test_initialization(self, gna):
        """Test that the GNA is initialized correctly."""
        assert gna.capacity == 1000

        # Get all membrane potentials
        potentials = gna.get_all_membrane_potentials()
        assert len(potentials) == 1000
        assert all(p == 0.0 for p in potentials)

    def test_membrane_potential_access(self, gna):
        """Test setting and getting membrane potentials."""
        # Set a membrane potential
        gna.set_membrane_potential(42, 0.5)

        # Get the membrane potential
        assert gna.get_membrane_potential(42) == 0.5

    def test_membrane_potential_decay(self, gna):
        """Test membrane potential decay."""
        # Set some membrane potentials
        for i in range(100):
            gna.set_membrane_potential(i, 1.0)

        # Decay membrane potentials
        gna.update_membrane_potentials(0.9)

        # Check that they've been decayed
        for i in range(100):
            assert gna.get_membrane_potential(i) == pytest.approx(0.9)

    def test_find_fire_candidates(self, gna):
        """Test finding neurons ready to fire."""
        # Set some neurons above threshold
        for i in range(0, 100, 2):
            gna.set_membrane_potential(i, 2.0)  # Above default threshold of 1.0

        # Find fire candidates
        candidates = gna.find_fire_candidates(0)

        # Check that we have the right neurons
        assert len(candidates) == 50
        assert all(i % 2 == 0 for i in candidates)

    def test_process_fired_neurons(self, gna):
        """Test processing fired neurons."""
        # Set up some fired neurons
        fired_list = list(range(0, 100, 2))

        # Set initial membrane potentials
        for i in fired_list:
            gna.set_membrane_potential(i, 1.0)

        # Process fired neurons
        gna.process_fired_neurons(fired_list, 42)

        # Check that membrane potentials are reset
        for i in fired_list:
            assert gna.get_membrane_potential(i) == 0.0

        # We can't easily check refractory periods in the optimized implementation


class TestFireCandidateList:
    @pytest.fixture
    def fcl(self):
        """Create an empty FCL."""
        return FireCandidateList()

    @pytest.fixture
    def fcl_with_neurons(self):
        """Create an FCL with some neurons."""
        return FireCandidateList([1, 2, 3, 5, 8, 13, 21])

    def test_empty_initialization(self, fcl):
        """Test that an empty FCL is initialized correctly."""
        assert len(fcl) == 0
        assert fcl.is_empty()
        assert fcl.to_list() == []

    def test_initialized_with_neurons(self, fcl_with_neurons):
        """Test that an FCL initialized with neurons contains them."""
        assert len(fcl_with_neurons) == 7
        assert not fcl_with_neurons.is_empty()
        assert sorted(fcl_with_neurons.to_list()) == [1, 2, 3, 5, 8, 13, 21]

    def test_add_neuron(self, fcl):
        """Test adding a neuron to the FCL."""
        fcl.add(42)
        assert len(fcl) == 1
        assert not fcl.is_empty()
        assert fcl.contains(42)
        assert fcl.to_list() == [42]

    def test_add_multiple_neurons(self, fcl):
        """Test adding multiple neurons to the FCL."""
        fcl.add_multiple([1, 2, 3, 5, 8])
        assert len(fcl) == 5
        assert not fcl.is_empty()
        assert all(fcl.contains(i) for i in [1, 2, 3, 5, 8])
        assert sorted(fcl.to_list()) == [1, 2, 3, 5, 8]

    def test_remove_neuron(self, fcl_with_neurons):
        """Test removing a neuron from the FCL."""
        fcl_with_neurons.remove(5)
        assert len(fcl_with_neurons) == 6
        assert not fcl_with_neurons.is_empty()
        assert not fcl_with_neurons.contains(5)
        assert sorted(fcl_with_neurons.to_list()) == [1, 2, 3, 8, 13, 21]

    def test_clear(self, fcl_with_neurons):
        """Test clearing the FCL."""
        fcl_with_neurons.clear()
        assert len(fcl_with_neurons) == 0
        assert fcl_with_neurons.is_empty()
        assert fcl_with_neurons.to_list() == []

    def test_iteration(self, fcl_with_neurons):
        """Test iterating over the FCL."""
        neurons = sorted(list(fcl_with_neurons))
        assert neurons == [1, 2, 3, 5, 8, 13, 21]


class TestConnectome:
    @pytest.fixture
    def connectome(self):
        """Create an empty connectome with 1000 neurons."""
        return Connectome(1000)

    def test_initialization(self, connectome):
        """Test that the connectome is initialized correctly."""
        assert connectome.neuron_count == 1000
        assert connectome.connection_count() == 0

    def test_add_connection(self, connectome):
        """Test adding a connection to the connectome."""
        connectome.add_connection(1, 2, 0.5)
        assert connectome.connection_count() == 1

        # Get connections for neuron 1
        connections = connectome.get_connections_for_neuron(1)
        assert len(connections) == 1
        assert connections[0]["target_id"] == 2
        assert connections[0]["weight"] == 0.5

    def test_add_multiple_connections(self, connectome):
        """Test adding multiple connections to the connectome."""
        # Add connections from neuron 1 to neurons 2, 3, 5
        connectome.add_connection(1, 2, 0.5)
        connectome.add_connection(1, 3, 0.6)
        connectome.add_connection(1, 5, 0.7)

        # Add connections from neuron 2 to neurons 3, 4
        connectome.add_connection(2, 3, 0.8)
        connectome.add_connection(2, 4, 0.9)

        assert connectome.connection_count() == 5

        # Get connections for neuron 1
        connections = connectome.get_connections_for_neuron(1)
        assert len(connections) == 3

        # Get connections for neuron 2
        connections = connectome.get_connections_for_neuron(2)
        assert len(connections) == 2

    def test_propagate_activations(self, connectome):
        """Test propagating activations through the connectome."""
        # Add connections
        connectome.add_connection(0, 1, 0.5)
        connectome.add_connection(0, 2, 0.6)
        connectome.add_connection(1, 2, 0.7)
        connectome.add_connection(1, 3, 0.8)

        # Create source activations (only neurons 0 and 1 are active)
        source_activations = [0.0] * 1000
        source_activations[0] = 1.0
        source_activations[1] = 0.5

        # Create target buffer
        target_buffer = [0.0] * 1000

        # Propagate activations
        result = connectome.propagate_activations(source_activations, target_buffer)

        # Check the result with pytest.approx to account for floating point precision
        assert result[1] == pytest.approx(0.5, abs=1e-5)  # From neuron 0 to 1
        assert result[2] == pytest.approx(
            0.6 + 0.5 * 0.7, abs=1e-5
        )  # From neuron 0 to 2 + neuron 1 to 2
        assert result[3] == pytest.approx(0.5 * 0.8, abs=1e-5)  # From neuron 1 to 3
        assert result[0] == pytest.approx(
            0.0, abs=1e-5
        )  # Neuron 0 has no incoming connections


class TestOptimizedFeagiCore:
    @pytest.fixture
    def feagi_core(self):
        """Create a FEAGI core with 1000 neurons."""
        return OptimizedFeagiCore(1000)

    def test_initialization(self, feagi_core):
        """Test that the FEAGI core is initialized correctly."""
        assert feagi_core.current_timestep == 0

    def test_step(self, feagi_core):
        """Test stepping the simulation."""
        # Perform a step
        feagi_core.step()

        # Check that the timestep has increased
        assert feagi_core.current_timestep == 1

    def test_propagate_activations(self, feagi_core):
        """Test propagating activations through the FEAGI core."""
        # This is a basic test since we can't easily access the internal components
        # of the optimized core from Python

        # Propagate activations
        result = feagi_core.propagate_activations()

        # Check that we got a result of the right size
        assert len(result) == 1000
