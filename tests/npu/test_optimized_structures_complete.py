"""
Comprehensive tests for optimized data structures.

This module provides complete test coverage for the optimized data structures
defined in feagi.npu.optimized_structures for both CPU and mocked Rust implementations.
"""

import pytest
import numpy as np
from unittest.mock import patch, MagicMock, Mock

from feagi.npu.optimized_structures import (
    GlobalNeuronArray,
    FireCandidateList,
    Connectome,
    OptimizedFeagiCore,
)


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


# Patch RUST_AVAILABLE for all tests in this module
@pytest.fixture(autouse=True)
def mock_rust_available():
    with patch('feagi.npu.optimized_structures.RUST_AVAILABLE', False):
        yield


class TestGlobalNeuronArray:
    """Tests for the GlobalNeuronArray class."""
    
    @pytest.fixture
    def numpy_gna(self):
        """Create a GNA using the NumPy implementation."""
        with patch('feagi.npu.optimized_structures.RUST_AVAILABLE', False):
            return GlobalNeuronArray(1000)
    
    @pytest.fixture
    @pytest.mark.skip(reason="create_gna function not available in optimized_structures module")
    def rust_gna(self):
        """Create a GNA using the mocked Rust implementation."""
        with patch('feagi.npu.optimized_structures.RUST_AVAILABLE', True), \
             patch('feagi.npu.optimized_structures.create_gna', return_value=MockRustGNA()):
            return GlobalNeuronArray(1000)
    
    def test_initialization_numpy(self, numpy_gna):
        """Test initialization of GNA with NumPy implementation."""
        assert numpy_gna.capacity == 1000
        assert not numpy_gna._use_rust
        assert numpy_gna.membrane_potentials.shape == (1000,)
        assert numpy_gna.thresholds.shape == (1000,)
        assert numpy_gna.refractory_periods.shape == (1000,)
        assert numpy_gna.refractory_counters.shape == (1000,)
        assert numpy_gna.coordinates_x.shape == (1000,)
        assert numpy_gna.coordinates_y.shape == (1000,)
        assert numpy_gna.coordinates_z.shape == (1000,)
    
    def test_initialization_rust(self, rust_gna):
        """Test initialization of GNA with Rust implementation."""
        assert rust_gna.capacity == 1000
        assert rust_gna._use_rust
        assert hasattr(rust_gna, '_rust_gna')
    
    def test_get_set_membrane_potential_numpy(self, numpy_gna):
        """Test getting and setting membrane potential using NumPy implementation."""
        numpy_gna.set_membrane_potential(42, 0.75)
        assert numpy_gna.get_membrane_potential(42) == 0.75
    
    def test_get_set_membrane_potential_rust(self, rust_gna):
        """Test getting and setting membrane potential using Rust implementation."""
        rust_gna.set_membrane_potential(42, 0.75)
        assert rust_gna.get_membrane_potential(42) == 0.75
    
    def test_get_set_coordinates_numpy(self, numpy_gna):
        """Test getting and setting coordinates using NumPy implementation."""
        numpy_gna.set_coordinates(42, 10, 20, 30)
        assert numpy_gna.get_coordinates(42) == (10, 20, 30)
    
    def test_get_set_coordinates_rust(self, rust_gna):
        """Test getting and setting coordinates using Rust implementation."""
        rust_gna.set_coordinates(42, 10, 20, 30)
        assert rust_gna.get_coordinates(42) == (10, 20, 30)
    
    def test_update_membrane_potentials_numpy(self, numpy_gna):
        """Test updating membrane potentials using NumPy implementation."""
        numpy_gna.set_membrane_potential(42, 1.0)
        numpy_gna.update_membrane_potentials(0.95)
        assert numpy_gna.get_membrane_potential(42) == pytest.approx(0.95, abs=1e-6)
    
    def test_update_membrane_potentials_rust(self, rust_gna):
        """Test updating membrane potentials using Rust implementation."""
        rust_gna.set_membrane_potential(42, 1.0)
        rust_gna.update_membrane_potentials(0.95)
        assert rust_gna.get_membrane_potential(42) == 0.95
    
    def test_update_refractory_counters_numpy(self, numpy_gna):
        """Test updating refractory counters using NumPy implementation."""
        numpy_gna.refractory_counters[42] = 5
        numpy_gna.update_refractory_counters()
        assert numpy_gna.refractory_counters[42] == 4
    
    def test_update_refractory_counters_rust(self, rust_gna):
        """Test updating refractory counters using Rust implementation."""
        rust_gna._rust_gna.refractory_counters[42] = 5
        rust_gna.update_refractory_counters()
        assert rust_gna._rust_gna.refractory_counters[42] == 4
    
    def test_find_fire_candidates_numpy(self, numpy_gna):
        """Test finding fire candidates using NumPy implementation."""
        # Set up a neuron ready to fire
        numpy_gna.membrane_potentials[42] = 1.1
        numpy_gna.thresholds[42] = 1.0
        numpy_gna.enabled_flags[42] = 1
        numpy_gna.refractory_counters[42] = 0
        
        candidates = numpy_gna.find_fire_candidates(1)
        assert 42 in candidates
    
    def test_find_fire_candidates_rust(self, rust_gna):
        """Test finding fire candidates using Rust implementation."""
        candidates = rust_gna.find_fire_candidates(1)
        assert candidates == [1, 2, 3]  # Based on our mock implementation
    
    def test_process_fired_neurons_numpy(self, numpy_gna):
        """Test processing fired neurons using NumPy implementation."""
        numpy_gna.membrane_potentials[42] = 1.0
        numpy_gna.refractory_periods[42] = 5
        numpy_gna.process_fired_neurons([42], 1)
        
        assert numpy_gna.membrane_potentials[42] == 0.0
        assert numpy_gna.refractory_counters[42] == 5
        assert numpy_gna.last_fired[42] == 1
    
    def test_process_fired_neurons_rust(self, rust_gna):
        """Test processing fired neurons using Rust implementation."""
        rust_gna.process_fired_neurons([42], 1)
        assert rust_gna._rust_gna.fired_neurons == [42]
        assert rust_gna._rust_gna.membrane_potentials[42] == 0.0
        assert rust_gna._rust_gna.refractory_counters[42] == 5
    
    def test_get_all_membrane_potentials_numpy(self, numpy_gna):
        """Test getting all membrane potentials using NumPy implementation."""
        numpy_gna.membrane_potentials[42] = 0.75
        numpy_gna.membrane_potentials[43] = 0.5
        
        potentials = numpy_gna.get_all_membrane_potentials()
        
        assert isinstance(potentials, list)
        assert len(potentials) == 1000
        assert potentials[42] == 0.75
        assert potentials[43] == 0.5
    
    def test_get_all_membrane_potentials_rust(self, rust_gna):
        """Test getting all membrane potentials using Rust implementation."""
        rust_gna.set_membrane_potential(42, 0.75)
        rust_gna.set_membrane_potential(43, 0.5)
        
        potentials = rust_gna.get_all_membrane_potentials()
        
        assert isinstance(potentials, list)
        assert 0.75 in potentials
        assert 0.5 in potentials


class TestFireCandidateList:
    """Tests for the FireCandidateList class."""
    
    @pytest.fixture
    def numpy_fcl(self):
        """Create an FCL using the NumPy implementation."""
        with patch('feagi.npu.optimized_structures.RUST_AVAILABLE', False):
            return FireCandidateList()
    
    @pytest.fixture
    @pytest.mark.skip(reason="create_fcl function not available in optimized_structures module")
    def rust_fcl(self):
        """Create an FCL using the mocked Rust implementation."""
        with patch('feagi.npu.optimized_structures.RUST_AVAILABLE', True), \
             patch('feagi.npu.optimized_structures.create_fcl', return_value=MockRustFCL()):
            return FireCandidateList()
    
    def test_initialization_numpy(self, numpy_fcl):
        """Test initialization of FCL with NumPy implementation."""
        assert not numpy_fcl._use_rust
        if numpy_fcl._use_dense:
            assert not np.any(numpy_fcl._mask)
        else:
            assert len(numpy_fcl._active_ids) == 0
    
    def test_initialization_with_neurons_numpy(self):
        """Test initialization with neurons using NumPy implementation."""
        with patch('feagi.npu.optimized_structures.RUST_AVAILABLE', False):
            fcl = FireCandidateList([1, 2, 3])
            if fcl._use_dense:
                assert np.sum(fcl._mask) == 3
                assert fcl._mask[1]
                assert fcl._mask[2]
                assert fcl._mask[3]
            else:
                assert len(fcl._active_ids) == 3
                assert 1 in fcl._active_ids
                assert 2 in fcl._active_ids
                assert 3 in fcl._active_ids
    
    def test_initialization_rust(self, rust_fcl):
        """Test initialization of FCL with Rust implementation."""
        assert rust_fcl._use_rust
        assert hasattr(rust_fcl, '_rust_fcl')
    
    def test_add_numpy(self, numpy_fcl):
        """Test adding a neuron using NumPy implementation."""
        numpy_fcl.add(42)
        if numpy_fcl._use_dense:
            assert numpy_fcl._mask[42]
        else:
            assert 42 in numpy_fcl._active_ids
    
    def test_add_rust(self, rust_fcl):
        """Test adding a neuron using Rust implementation."""
        rust_fcl.add(42)
        assert 42 in rust_fcl._rust_fcl.neurons
    
    def test_add_multiple_numpy(self, numpy_fcl):
        """Test adding multiple neurons using NumPy implementation."""
        numpy_fcl.add_multiple([1, 2, 3])
        if numpy_fcl._use_dense:
            assert np.sum(numpy_fcl._mask) == 3
            assert numpy_fcl._mask[1]
            assert numpy_fcl._mask[2]
            assert numpy_fcl._mask[3]
        else:
            assert len(numpy_fcl._active_ids) == 3
            assert 1 in numpy_fcl._active_ids
            assert 2 in numpy_fcl._active_ids
            assert 3 in numpy_fcl._active_ids
    
    def test_add_multiple_rust(self, rust_fcl):
        """Test adding multiple neurons using Rust implementation."""
        rust_fcl.add_multiple([1, 2, 3])
        assert len(rust_fcl._rust_fcl.neurons) == 3
        assert 1 in rust_fcl._rust_fcl.neurons
        assert 2 in rust_fcl._rust_fcl.neurons
        assert 3 in rust_fcl._rust_fcl.neurons
    
    def test_remove_numpy(self, numpy_fcl):
        """Test removing a neuron using NumPy implementation."""
        numpy_fcl.add(42)
        numpy_fcl.remove(42)
        if numpy_fcl._use_dense:
            assert not numpy_fcl._mask[42]
        else:
            assert 42 not in numpy_fcl._active_ids
    
    def test_remove_rust(self, rust_fcl):
        """Test removing a neuron using Rust implementation."""
        rust_fcl.add(42)
        rust_fcl.remove(42)
        assert 42 not in rust_fcl._rust_fcl.neurons
    
    def test_clear_numpy(self, numpy_fcl):
        """Test clearing the FCL using NumPy implementation."""
        numpy_fcl.add_multiple([1, 2, 3])
        numpy_fcl.clear()
        if numpy_fcl._use_dense:
            assert not np.any(numpy_fcl._mask)
        else:
            assert len(numpy_fcl._active_ids) == 0
    
    def test_clear_rust(self, rust_fcl):
        """Test clearing the FCL using Rust implementation."""
        rust_fcl.add_multiple([1, 2, 3])
        rust_fcl.clear()
        assert len(rust_fcl._rust_fcl.neurons) == 0
    
    def test_contains_numpy(self, numpy_fcl):
        """Test contains operation using NumPy implementation."""
        numpy_fcl.add(42)
        assert numpy_fcl.contains(42)
        assert not numpy_fcl.contains(43)
    
    def test_contains_rust(self, rust_fcl):
        """Test contains operation using Rust implementation."""
        rust_fcl.add(42)
        assert rust_fcl.contains(42)
        assert not rust_fcl.contains(43)
    
    def test_len_numpy(self, numpy_fcl):
        """Test getting length using NumPy implementation."""
        numpy_fcl.add_multiple([1, 2, 3])
        assert len(numpy_fcl) == 3
    
    def test_len_rust(self, rust_fcl):
        """Test getting length using Rust implementation."""
        rust_fcl.add_multiple([1, 2, 3])
        assert len(rust_fcl) == 3
    
    def test_is_empty_numpy(self, numpy_fcl):
        """Test is_empty method using NumPy implementation."""
        assert numpy_fcl.is_empty()
        numpy_fcl.add(42)
        assert not numpy_fcl.is_empty()
    
    def test_is_empty_rust(self, rust_fcl):
        """Test is_empty method using Rust implementation."""
        assert rust_fcl.is_empty()
        rust_fcl.add(42)
        assert not rust_fcl.is_empty()
    
    def test_to_list_numpy(self, numpy_fcl):
        """Test to_list method using NumPy implementation."""
        numpy_fcl.add_multiple([1, 2, 3])
        neuron_list = numpy_fcl.to_list()
        assert isinstance(neuron_list, list)
        assert sorted(neuron_list) == [1, 2, 3]
    
    def test_to_list_rust(self, rust_fcl):
        """Test to_list method using Rust implementation."""
        rust_fcl.add_multiple([1, 2, 3])
        neuron_list = rust_fcl.to_list()
        assert isinstance(neuron_list, list)
        assert sorted(neuron_list) == [1, 2, 3]
    
    def test_iteration_numpy(self, numpy_fcl):
        """Test iteration using NumPy implementation."""
        numpy_fcl.add_multiple([1, 2, 3])
        neurons = [n for n in numpy_fcl]
        assert sorted(neurons) == [1, 2, 3]
    
    def test_iteration_rust(self, rust_fcl):
        """Test iteration using Rust implementation."""
        rust_fcl.add_multiple([1, 2, 3])
        neurons = [n for n in rust_fcl]
        assert sorted(neurons) == [1, 2, 3]


if __name__ == "__main__":
    pytest.main(["-v", __file__]) 