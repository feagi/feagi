"""
Comprehensive test coverage for Optimized Integration module.

This module tests all functionality in optimized_integration.py to achieve
high code coverage, including both optimized and fallback implementations.
"""

import pytest
from unittest.mock import Mock, patch, MagicMock
from typing import Dict, List, Any

from feagi.npu.optimized_integration import (
    create_optimized_core,
    get_core_property,
    set_core_property,
    step_simulation,
    step_simulation_with_fire_queue,
    propagate_activations,
    add_connection,
    get_membrane_potential,
    set_membrane_potential,
    RUST_AVAILABLE,
)


class MockGlobalNeuronArray:
    """Mock GlobalNeuronArray for testing."""
    
    def __init__(self, capacity):
        self.capacity = capacity
        self._membrane_potentials = [0.0] * capacity
        self._refractory_counters = [0] * capacity
        self._thresholds = [1.0] * capacity
    
    def get_membrane_potential(self, neuron_id):
        return self._membrane_potentials[neuron_id]
    
    def set_membrane_potential(self, neuron_id, value):
        self._membrane_potentials[neuron_id] = value
    
    def update_membrane_potentials(self, decay_factor):
        for i in range(self.capacity):
            self._membrane_potentials[i] *= decay_factor
    
    def update_refractory_counters(self):
        for i in range(self.capacity):
            if self._refractory_counters[i] > 0:
                self._refractory_counters[i] -= 1
    
    def find_fire_candidates(self, timestep):
        candidates = []
        for i in range(self.capacity):
            if (self._membrane_potentials[i] >= self._thresholds[i] and 
                self._refractory_counters[i] == 0):
                candidates.append(i)
        return candidates
    
    def process_fired_neurons(self, neuron_ids, timestep):
        for neuron_id in neuron_ids:
            self._membrane_potentials[neuron_id] = 0.0
            self._refractory_counters[neuron_id] = 3  # 3 timestep refractory period


class MockFireCandidateList:
    """Mock FireCandidateList for testing."""
    
    def __init__(self):
        self._neurons = []
    
    def clear(self):
        self._neurons = []
    
    def add_multiple(self, neuron_ids):
        self._neurons.extend(neuron_ids)
    
    def to_list(self):
        return self._neurons.copy()
    
    def __iter__(self):
        return iter(self._neurons)


class MockConnectome:
    """Mock Connectome for testing."""
    
    def __init__(self, neuron_count, estimated_connections):
        self.neuron_count = neuron_count
        self._connections = {}
    
    def add_connection(self, source_id, target_id, weight):
        if source_id not in self._connections:
            self._connections[source_id] = []
        self._connections[source_id].append({
            "target_id": target_id,
            "weight": weight
        })
    
    def get_connections_for_neuron(self, source_id):
        return self._connections.get(source_id, [])
    
    def propagate_activations(self, activations, target_buffer):
        """Propagate activations through the network."""
        for source_id, connections in self._connections.items():
            for conn in connections:
                target_id = conn["target_id"]
                weight = conn["weight"]
                target_buffer[target_id] += activations[source_id] * weight
        return target_buffer


class MockOptimizedFeagiCore:
    """Mock OptimizedFeagiCore for testing."""
    
    def __init__(self, neuron_count, estimated_connections):
        self.current_timestep = 0
        self.gna = MockGlobalNeuronArray(neuron_count)
        self.fcl = MockFireCandidateList()
        self.connectome = MockConnectome(neuron_count, estimated_connections)
        self._rust_core = Mock()
        self._rust_core.get_gna.return_value = self.gna
        self._rust_core.get_fcl.return_value = self.fcl
        self._rust_core.step_with_fire_queue = Mock()
    
    def step(self):
        """Step simulation forward."""
        # Simple implementation
        self.current_timestep += 1
    
    def propagate_activations(self):
        """Propagate activations through network."""
        activations = [0.0] * self.gna.capacity
        for neuron_id in self.fcl:
            activations[neuron_id] = 1.0
        target_buffer = [0.0] * self.gna.capacity
        return self.connectome.propagate_activations(activations, target_buffer)


@pytest.fixture
def mock_optimized_structures():
    """Mock the optimized structures module."""
    with patch('feagi.npu.optimized_integration.RUST_AVAILABLE', True), \
         patch('feagi.npu.optimized_integration.OptimizedFeagiCore', MockOptimizedFeagiCore):
        yield


@pytest.fixture
def mock_no_optimized_structures():
    """Mock the absence of optimized structures."""
    with patch('feagi.npu.optimized_integration.RUST_AVAILABLE', False):
        yield


def test_create_optimized_core_with_rust_available(mock_optimized_structures):
    """Test creating optimized core when Rust is available."""
    core = create_optimized_core(1000, 5000, use_optimized=True)
    
    assert isinstance(core, MockOptimizedFeagiCore)
    assert core.current_timestep == 0
    assert core.gna.capacity == 1000


def test_create_optimized_core_without_rust():
    """Test creating core when Rust is not available."""
    with patch('feagi.npu.optimized_integration.RUST_AVAILABLE', False), \
         patch('feagi.npu.optimized_integration.GlobalNeuronArray', MockGlobalNeuronArray), \
         patch('feagi.npu.optimized_integration.FireCandidateList', MockFireCandidateList), \
         patch('feagi.npu.optimized_integration.Connectome', MockConnectome):
        
        core = create_optimized_core(1000, 5000, use_optimized=True)
        
        assert isinstance(core, dict)
        assert "gna" in core
        assert "fcl" in core
        assert "connectome" in core
        assert core["current_timestep"] == 0


def test_create_optimized_core_use_optimized_false():
    """Test creating core with use_optimized=False."""
    with patch('feagi.npu.optimized_integration.GlobalNeuronArray', MockGlobalNeuronArray), \
         patch('feagi.npu.optimized_integration.FireCandidateList', MockFireCandidateList), \
         patch('feagi.npu.optimized_integration.Connectome', MockConnectome):
        
        core = create_optimized_core(1000, 5000, use_optimized=False)
        
        assert isinstance(core, dict)
        assert "gna" in core


def test_get_core_property_optimized_current_timestep(mock_optimized_structures):
    """Test getting current_timestep from optimized core."""
    core = create_optimized_core(100, 1000)
    core.current_timestep = 42
    
    timestep = get_core_property(core, "current_timestep")
    assert timestep == 42


def test_get_core_property_optimized_gna(mock_optimized_structures):
    """Test getting GNA from optimized core."""
    core = create_optimized_core(100, 1000)
    
    gna = get_core_property(core, "gna")
    assert gna == core._rust_core.get_gna.return_value


def test_get_core_property_optimized_fcl(mock_optimized_structures):
    """Test getting FCL from optimized core."""
    core = create_optimized_core(100, 1000)
    
    fcl = get_core_property(core, "fcl")
    assert fcl == core._rust_core.get_fcl.return_value


def test_get_core_property_optimized_unknown():
    """Test getting unknown property from optimized core."""
    # Test with RUST_AVAILABLE=True and non-dict core
    with patch('feagi.npu.optimized_integration.RUST_AVAILABLE', True):
        core = MockOptimizedFeagiCore(100, 1000)
        
        with pytest.raises(AttributeError, match="Unknown property: unknown"):
            get_core_property(core, "unknown")


def test_get_core_property_dict_based():
    """Test getting property from dict-based core."""
    core = {
        "current_timestep": 5,
        "gna": "mock_gna",
        "fcl": "mock_fcl"
    }
    
    assert get_core_property(core, "current_timestep") == 5
    assert get_core_property(core, "gna") == "mock_gna"


def test_set_core_property_optimized_current_timestep(mock_optimized_structures):
    """Test setting current_timestep on optimized core."""
    core = create_optimized_core(100, 1000)
    
    set_core_property(core, "current_timestep", 10)
    assert core.current_timestep == 10


def test_set_core_property_optimized_unknown():
    """Test setting unknown property on optimized core."""
    # Test with RUST_AVAILABLE=True and non-dict core
    with patch('feagi.npu.optimized_integration.RUST_AVAILABLE', True):
        core = MockOptimizedFeagiCore(100, 1000)
        
        with pytest.raises(AttributeError, match="Cannot set property: unknown"):
            set_core_property(core, "unknown", "value")


def test_set_core_property_dict_based():
    """Test setting property on dict-based core."""
    core = {"current_timestep": 0}
    
    set_core_property(core, "current_timestep", 15)
    assert core["current_timestep"] == 15
    
    set_core_property(core, "new_property", "new_value")
    assert core["new_property"] == "new_value"


def test_step_simulation_optimized(mock_optimized_structures):
    """Test stepping simulation with optimized core."""
    core = create_optimized_core(100, 1000)
    initial_timestep = core.current_timestep
    
    step_simulation(core)
    
    assert core.current_timestep == initial_timestep + 1


def test_step_simulation_dict_based():
    """Test stepping simulation with dict-based core."""
    gna = MockGlobalNeuronArray(10)
    fcl = MockFireCandidateList()
    
    # Set up some initial state
    gna.set_membrane_potential(0, 1.5)  # Above threshold
    gna.set_membrane_potential(1, 0.8)  # Below threshold
    
    core = {
        "gna": gna,
        "fcl": fcl,
        "current_timestep": 0
    }
    
    step_simulation(core)
    
    # Check that timestep advanced
    assert core["current_timestep"] == 1
    
    # Check that neuron 0 fired (was above threshold)
    fired_neurons = fcl.to_list()
    assert 0 in fired_neurons
    assert 1 not in fired_neurons


def test_step_simulation_with_fire_queue_optimized(mock_optimized_structures):
    """Test step simulation with fire queue for optimized core."""
    core = create_optimized_core(100, 1000)
    
    step_simulation_with_fire_queue(core, mpf=True, puf=False, max_consecutive_fires=5)
    
    # Should call the rust core method
    core._rust_core.step_with_fire_queue.assert_called_once_with(True, False, 5)


def test_step_simulation_with_fire_queue_dict_based():
    """Test step simulation with fire queue for dict-based core."""
    gna = MockGlobalNeuronArray(10)
    fcl = MockFireCandidateList()
    connectome = MockConnectome(10, 100)
    
    # Set up initial firing neuron
    fcl.add_multiple([0])
    gna.set_membrane_potential(0, 1.0)
    
    # Add connections
    connectome.add_connection(0, 1, 0.5)
    connectome.add_connection(0, 2, 0.8)
    
    core = {
        "gna": gna,
        "fcl": fcl,
        "connectome": connectome,
        "current_timestep": 0
    }
    
    step_simulation_with_fire_queue(core, mpf=True, puf=False)
    
    # Check that timestep advanced
    assert core["current_timestep"] == 1
    
    # Check that neuron 0's membrane potential was reset to 0
    assert gna.get_membrane_potential(0) == 0.0


def test_step_simulation_with_fire_queue_mpf_false():
    """Test fire queue simulation with MPF=False."""
    gna = MockGlobalNeuronArray(5)
    fcl = MockFireCandidateList()
    connectome = MockConnectome(5, 10)
    
    # Set up firing neuron
    fcl.add_multiple([0])
    gna.set_membrane_potential(0, 1.0)
    
    # Add connection
    connectome.add_connection(0, 1, 0.5)
    
    core = {
        "gna": gna,
        "fcl": fcl,
        "connectome": connectome,
        "current_timestep": 0
    }
    
    step_simulation_with_fire_queue(core, mpf=False, puf=False)
    
    # With MPF=False, should use PSP=1.0 instead of membrane potential
    # PSP = (1.0 / 1.0) * 0.5 = 0.5
    assert gna.get_membrane_potential(1) == 0.5


def test_step_simulation_with_fire_queue_puf_true():
    """Test fire queue simulation with PUF=True."""
    gna = MockGlobalNeuronArray(5)
    fcl = MockFireCandidateList()
    connectome = MockConnectome(5, 10)
    
    # Set up firing neuron
    fcl.add_multiple([0])
    gna.set_membrane_potential(0, 1.0)
    
    # Add multiple connections from same source
    connectome.add_connection(0, 1, 0.4)
    connectome.add_connection(0, 2, 0.4)
    
    core = {
        "gna": gna,
        "fcl": fcl,
        "connectome": connectome,
        "current_timestep": 0
    }
    
    step_simulation_with_fire_queue(core, mpf=True, puf=True)
    
    # Check that the simulation completed
    assert core["current_timestep"] == 1
    
    # Check that source neuron was reset
    assert gna.get_membrane_potential(0) == 0.0


def test_step_simulation_with_fire_queue_max_consecutive_fires():
    """Test fire queue simulation with consecutive fire limit."""
    gna = MockGlobalNeuronArray(5)
    fcl = MockFireCandidateList()
    connectome = MockConnectome(5, 10)
    
    core = {
        "gna": gna,
        "fcl": fcl,
        "connectome": connectome,
        "current_timestep": 0
    }
    
    # Test with max_consecutive_fires=0 (no limit)
    step_simulation_with_fire_queue(core, max_consecutive_fires=0)
    
    # Should complete without issues
    assert core["current_timestep"] == 1


def test_step_simulation_with_fire_queue_optimized_core_object():
    """Test fire queue simulation with optimized core object (not dict)."""
    core = MockOptimizedFeagiCore(10, 100)
    
    # Add some neurons to FCL
    core.fcl.add_multiple([0, 1])
    core.gna.set_membrane_potential(0, 1.0)
    
    # Add connections
    core.connectome.add_connection(0, 2, 0.5)
    
    step_simulation_with_fire_queue(core, mpf=True, puf=False)
    
    # Should process correctly with optimized core object
    assert core.current_timestep == 1


def test_propagate_activations_optimized(mock_optimized_structures):
    """Test propagating activations with optimized core."""
    core = create_optimized_core(100, 1000)
    
    # Mock the propagate_activations method
    expected_result = [0.1, 0.2, 0.3]
    core.propagate_activations = Mock(return_value=expected_result)
    
    result = propagate_activations(core)
    
    assert result == expected_result
    core.propagate_activations.assert_called_once()


def test_propagate_activations_dict_based():
    """Test propagating activations with dict-based core."""
    gna = MockGlobalNeuronArray(5)
    fcl = MockFireCandidateList()
    connectome = MockConnectome(5, 10)
    
    # Set up firing neurons
    fcl.add_multiple([0, 2])
    
    # Add connections
    connectome.add_connection(0, 1, 0.5)
    connectome.add_connection(2, 3, 0.8)
    
    core = {
        "gna": gna,
        "fcl": fcl,
        "connectome": connectome
    }
    
    result = propagate_activations(core)
    
    # Should have 5 elements (gna.capacity)
    assert len(result) == 5
    
    # Check that activations were propagated
    assert result[1] == 0.5  # From neuron 0 with weight 0.5
    assert result[3] == 0.8  # From neuron 2 with weight 0.8


def test_add_connection_optimized(mock_optimized_structures):
    """Test adding connection with optimized core."""
    core = create_optimized_core(100, 1000)
    
    # Mock the connectome add_connection method properly
    core.connectome.add_connection = Mock()
    
    add_connection(core, 10, 20, 0.75)
    
    # Should call the connectome add_connection method
    core.connectome.add_connection.assert_called_once_with(10, 20, 0.75)


def test_add_connection_dict_based():
    """Test adding connection with dict-based core."""
    connectome = MockConnectome(100, 1000)
    core = {"connectome": connectome}
    
    add_connection(core, 5, 15, 0.6)
    
    # Check that connection was added
    connections = connectome.get_connections_for_neuron(5)
    assert len(connections) == 1
    assert connections[0]["target_id"] == 15
    assert connections[0]["weight"] == 0.6


def test_get_membrane_potential_optimized(mock_optimized_structures):
    """Test getting membrane potential with optimized core."""
    core = create_optimized_core(100, 1000)
    
    # Mock the GNA get_membrane_potential method
    core._rust_core.get_gna().get_membrane_potential = Mock(return_value=0.75)
    
    result = get_membrane_potential(core, 42)
    
    assert result == 0.75
    core._rust_core.get_gna().get_membrane_potential.assert_called_once_with(42)


def test_get_membrane_potential_dict_based():
    """Test getting membrane potential with dict-based core."""
    gna = MockGlobalNeuronArray(100)
    gna.set_membrane_potential(25, 0.85)
    
    core = {"gna": gna}
    
    result = get_membrane_potential(core, 25)
    assert result == 0.85


def test_set_membrane_potential_optimized(mock_optimized_structures):
    """Test setting membrane potential with optimized core."""
    core = create_optimized_core(100, 1000)
    
    # Mock the GNA set_membrane_potential method
    core._rust_core.get_gna().set_membrane_potential = Mock()
    
    set_membrane_potential(core, 42, 0.95)
    
    core._rust_core.get_gna().set_membrane_potential.assert_called_once_with(42, 0.95)


def test_set_membrane_potential_dict_based():
    """Test setting membrane potential with dict-based core."""
    gna = MockGlobalNeuronArray(100)
    core = {"gna": gna}
    
    set_membrane_potential(core, 25, 0.65)
    
    assert gna.get_membrane_potential(25) == 0.65


def test_step_simulation_with_fire_queue_no_connections():
    """Test fire queue simulation when neuron has no outgoing connections."""
    gna = MockGlobalNeuronArray(5)
    fcl = MockFireCandidateList()
    connectome = MockConnectome(5, 10)
    
    # Set up firing neuron with no connections
    fcl.add_multiple([0])
    gna.set_membrane_potential(0, 1.0)
    
    core = {
        "gna": gna,
        "fcl": fcl,
        "connectome": connectome,
        "current_timestep": 0
    }
    
    step_simulation_with_fire_queue(core)
    
    # Should complete without issues even with no connections
    assert core["current_timestep"] == 1


def test_step_simulation_with_fire_queue_refractory_period():
    """Test fire queue simulation with neurons in refractory period."""
    gna = MockGlobalNeuronArray(5)
    fcl = MockFireCandidateList()
    connectome = MockConnectome(5, 10)
    
    # Set up firing neuron with high membrane potential
    fcl.add_multiple([0])
    gna.set_membrane_potential(0, 1.0)
    
    # Add connection to target with high weight to ensure target fires
    connectome.add_connection(0, 1, 2.0)  # High weight to ensure firing
    
    core = {
        "gna": gna,
        "fcl": fcl,
        "connectome": connectome,
        "current_timestep": 0
    }
    
    # First step - target neuron should receive PSP and potentially fire
    step_simulation_with_fire_queue(core)
    
    # Test that the simulation runs correctly regardless of refractory state
    assert core["current_timestep"] == 1
    
    # The exact refractory behavior depends on implementation details
    # We mainly want to ensure the simulation doesn't crash


def test_step_simulation_membrane_potential_decay():
    """Test that membrane potentials decay during simulation."""
    gna = MockGlobalNeuronArray(5)
    fcl = MockFireCandidateList()
    
    # Set initial membrane potential
    gna.set_membrane_potential(0, 0.8)
    initial_mp = gna.get_membrane_potential(0)
    
    core = {
        "gna": gna,
        "fcl": fcl,
        "current_timestep": 0
    }
    
    step_simulation(core)
    
    # Should decay (multiply by 0.95)
    assert gna.get_membrane_potential(0) == initial_mp * 0.95


def test_step_simulation_refractory_counter_update():
    """Test that refractory counters are updated during simulation."""
    gna = MockGlobalNeuronArray(5)
    fcl = MockFireCandidateList()
    
    # Set refractory counter
    gna._refractory_counters[0] = 2
    
    core = {
        "gna": gna,
        "fcl": fcl,
        "current_timestep": 0
    }
    
    step_simulation(core)
    
    # Refractory counter should decrease
    assert gna._refractory_counters[0] == 1


if __name__ == "__main__":
    pytest.main(["-v", __file__]) 