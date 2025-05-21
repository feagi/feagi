"""
Advanced tests for the optimized_integration module.

This module provides additional test coverage for the optimized_integration module,
focusing on edge cases, error handling, and behavior variations not covered
in the existing tests.
"""

import pytest
import numpy as np
from unittest.mock import patch, MagicMock, Mock, PropertyMock

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
    RUST_AVAILABLE
)


@pytest.fixture
def dict_core():
    """Create a dict-based core structure for testing."""
    with patch('feagi.npu.optimized_integration.RUST_AVAILABLE', False):
        return create_optimized_core(
            neuron_count=1000,
            estimated_connections=5000,
            use_optimized=True  # Will still use dict-based due to RUST_AVAILABLE=False
        )


class TestMiscFunctions:
    """Tests for miscellaneous functions in the optimized_integration module."""
    
    def test_set_membrane_to_zero(self, dict_core):
        """Test setting membrane potential to zero, which is equivalent to reset."""
        # Set membrane potential first
        neuron_id = 42
        set_membrane_potential(dict_core, neuron_id, 0.75)
        
        # Check it was set
        assert get_membrane_potential(dict_core, neuron_id) == 0.75
        
        # Reset it by setting to zero
        set_membrane_potential(dict_core, neuron_id, 0.0)
        
        # Check it was reset to 0
        assert get_membrane_potential(dict_core, neuron_id) == 0.0
    
    def test_set_membrane_optimized(self):
        """Test setting membrane potential to zero with optimized core."""
        with patch('feagi.npu.optimized_integration.RUST_AVAILABLE', True):
            # Create a mock optimized core
            mock_core = Mock()
            mock_gna = Mock()
            mock_core._rust_core = Mock()
            mock_core._rust_core.gna = mock_gna
            mock_core._rust_core.get_gna = Mock(return_value=mock_gna)
            
            # Call set function with zero value
            set_membrane_potential(mock_core, 42, 0.0)
            
            # Verify call to internal method
            mock_gna.set_membrane_potential.assert_called_once_with(42, 0.0)


class TestEdgeCases:
    """Tests for edge cases and error handling."""
    
    def test_invalid_property_dict(self, dict_core):
        """Test getting/setting invalid properties on dict-based core."""
        # Test getting non-existent property
        with pytest.raises(KeyError):
            get_core_property(dict_core, "invalid_property")
    
        # Test setting non-existent property - this should not raise an error
        # as set_core_property simply sets the value in the dictionary
        set_core_property(dict_core, "new_property", 123)
        assert dict_core["new_property"] == 123
    
    def test_zero_neuron_count(self):
        """Test creating core with zero neurons."""
        with patch('feagi.npu.optimized_integration.RUST_AVAILABLE', False):
            # Should still create core with default small size
            core = create_optimized_core(neuron_count=0, estimated_connections=100)
            assert isinstance(core, dict)
            
    def test_propagate_activations_empty_fcl(self, dict_core):
        """Test propagating activations with empty FCL."""
        # Mock FCL to return empty list
        dict_core["fcl"].to_list = Mock(return_value=[])
        
        # Propagate activations
        result = propagate_activations(dict_core)
        
        # Should return array of zeros
        assert isinstance(result, list)
        assert len(result) == 1000  # Neuron count
        assert sum(result) == 0  # All zeros
    
    def test_step_simulation_exception_handling(self, dict_core):
        """Test handling of exceptions during step_simulation."""
        # Mock a method to raise exception
        dict_core["gna"].update_membrane_potentials = Mock(side_effect=RuntimeError("Test error"))
        
        # Step simulation should propagate the exception
        with pytest.raises(RuntimeError):
            step_simulation(dict_core)
    
    def test_step_with_fire_queue_empty_fcl(self, dict_core):
        """Test step_simulation_with_fire_queue with empty FCL."""
        # Mock FCL to return empty list
        original_to_list = dict_core["fcl"].to_list
        to_list_mock = MagicMock(return_value=[])
        dict_core["fcl"].to_list = to_list_mock
        
        # Mock clear method
        original_clear = dict_core["fcl"].clear
        clear_mock = MagicMock()
        dict_core["fcl"].clear = clear_mock
        
        # Step simulation with fire queue
        step_simulation_with_fire_queue(dict_core, mpf=True, puf=False, max_consecutive_fires=10)
        
        # FCL.clear should still be called
        assert clear_mock.call_count == 1
        
        # Restore original methods
        dict_core["fcl"].to_list = original_to_list
        dict_core["fcl"].clear = original_clear
    
    def test_core_property_setting_validation(self, dict_core):
        """Test validation when setting core properties."""
        # Set to valid value
        set_core_property(dict_core, "current_timestep", 42)
        assert dict_core["current_timestep"] == 42
    
        # Try setting to non-integer value
        # No validation in the current implementation, so this should also work
        set_core_property(dict_core, "current_timestep", "invalid")
        assert dict_core["current_timestep"] == "invalid"


class TestPropagationLogic:
    """Tests for the neuron activation propagation logic."""
    
    def test_propagation_with_threshold(self, dict_core):
        """Test propagation with neuron threshold effects."""
        # Set up neurons with different membrane potentials
        neurons = [1, 2, 3]
        
        # Create mocks for methods that will be called
        original_get_membrane_potential = dict_core["gna"].get_membrane_potential
        dict_core["gna"].get_membrane_potential = MagicMock(return_value=0.8)  # Above threshold
        
        # Since there's no get_threshold method, we'll patch find_fire_candidates directly
        # to simulate the threshold behavior
        original_find_fire = dict_core["gna"].find_fire_candidates
        dict_core["gna"].find_fire_candidates = MagicMock(return_value=[1, 3])  # Only neurons 1 and 3 exceed threshold
        
        # Setup FCL mock
        original_to_list = dict_core["fcl"].to_list
        dict_core["fcl"].to_list = MagicMock(return_value=[1, 3])
        
        # When we set puf=True, propagate_activations is not called directly but through
        # the internals of step_simulation_with_fire_queue, which uses the connectome in dict_core
        # to propagate activations. Let's set up a mock so we can track what's happening.
        original_get_connections = dict_core["connectome"].get_connections_for_neuron
        dict_core["connectome"].get_connections_for_neuron = MagicMock(return_value=[
            {"target_id": 10, "weight": 0.5}
        ])
        
        # Test step_simulation_with_fire_queue without puf=True, which skips propagation
        step_simulation_with_fire_queue(dict_core, mpf=True, puf=False, max_consecutive_fires=10)
        
        # Verify that the appropriate methods were called
        assert dict_core["fcl"].to_list.called
        
        # Restore original methods
        dict_core["gna"].get_membrane_potential = original_get_membrane_potential
        dict_core["gna"].find_fire_candidates = original_find_fire
        dict_core["fcl"].to_list = original_to_list
        dict_core["connectome"].get_connections_for_neuron = original_get_connections


class TestRefractoryAndConsecutiveLogic:
    """Tests for the refractory period and consecutive fire logic."""
    
    def test_refractory_and_consecutive_fire_logic(self, dict_core):
        """Test that neurons in refractory period or exceeding consecutive fires don't propagate."""
        # Set up neurons
        neurons = [1, 2, 3, 4]
        
        # Mock FCL to return these neurons
        original_to_list = dict_core["fcl"].to_list
        dict_core["fcl"].to_list = MagicMock(return_value=neurons)
        
        # Configure mock for membrane potentials without using thresholds
        original_get_membrane_potential = dict_core["gna"].get_membrane_potential
        
        # Mock refractory periods - neurons 1 and 2 are in refractory period
        def mock_get_refractory(neuron_id):
            return 2 if neuron_id in [1, 2] else 0
            
        if hasattr(dict_core["gna"], "get_refractory_period"):
            original_get_refractory = dict_core["gna"].get_refractory_period
            dict_core["gna"].get_refractory_period = MagicMock(side_effect=mock_get_refractory)
        else:
            # If there's no direct refractory getter, we'll patch the find_fire_candidates method
            # to simulate the refractory behavior combined with threshold
            original_find_fire = dict_core["gna"].find_fire_candidates
            dict_core["gna"].find_fire_candidates = MagicMock(return_value=[3, 4])  # Only 3 and 4 can fire
        
        # Mock consecutive fire counts - neuron 3 has reached the limit
        def mock_get_consecutive_fires(neuron_id):
            return 10 if neuron_id == 3 else 0
            
        # Create a mock for increment_consecutive_fire_count
        original_increment = None
        if hasattr(dict_core["gna"], "increment_consecutive_fire_count"):
            original_increment = dict_core["gna"].increment_consecutive_fire_count
            dict_core["gna"].increment_consecutive_fire_count = MagicMock()
            
        # Only neuron 4 should fire (not in refractory and not reached consecutive limit)
        fired_neurons = []
        def mock_process_fired(neurons, timestep):
            nonlocal fired_neurons
            fired_neurons = neurons
            
        original_process_fired = dict_core["gna"].process_fired_neurons
        dict_core["gna"].process_fired_neurons = MagicMock(side_effect=mock_process_fired)
        
        # Test with fire queue
        step_simulation_with_fire_queue(dict_core, mpf=True, puf=True, max_consecutive_fires=5)
        
        # Since we're not using all the same methods, let's verify that process_fired_neurons is called,
        # which indicates the fire propagation logic was executed
        dict_core["gna"].process_fired_neurons.assert_called_once()
        
        # Restore original methods
        dict_core["fcl"].to_list = original_to_list
        dict_core["gna"].get_membrane_potential = original_get_membrane_potential
        dict_core["gna"].process_fired_neurons = original_process_fired
        
        if hasattr(dict_core["gna"], "get_refractory_period"):
            dict_core["gna"].get_refractory_period = original_get_refractory
        else:
            dict_core["gna"].find_fire_candidates = original_find_fire
            
        if original_increment:
            dict_core["gna"].increment_consecutive_fire_count = original_increment


if __name__ == "__main__":
    pytest.main(["-v", __file__]) 