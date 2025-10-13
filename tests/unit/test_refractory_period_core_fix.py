"""
Test for the CRITICAL refractory period fix in NeuronArray.embedded_optimized_neural_update().

This test validates that neurons properly:
1. Reset membrane potentials when firing
2. Set refractory counters to refractory periods when firing  
3. Cannot fire during refractory period
4. Can fire again after refractory period expires
"""

import pytest
import numpy as np
from unittest.mock import Mock

from feagi.bdu.models.neuron import NeuronArray


class MockNeuronMappingProvider:
    """Mock mapping provider for testing."""
    
    def __init__(self):
        self.id_to_index = {}
        self.index_to_id = {}
        
    def get_neuron_index(self, neuron_id: int):
        return self.id_to_index.get(neuron_id)
        
    def get_neuron_id(self, index: int):
        return self.index_to_id.get(index)
        
    def set_neuron_mapping(self, neuron_id: int, index: int):
        self.id_to_index[neuron_id] = index
        self.index_to_id[index] = neuron_id
        
    def remove_neuron_mapping(self, neuron_id: int):
        if neuron_id in self.id_to_index:
            index = self.id_to_index[neuron_id]
            del self.id_to_index[neuron_id]
            del self.index_to_id[index]
            
    def has_neuron(self, neuron_id: int):
        return neuron_id in self.id_to_index
        
    def get_all_neuron_ids(self):
        return list(self.id_to_index.keys())


class TestRefractoryPeriodCoreFix:
    """Test the critical refractory period fix in NeuronArray."""
    
    @pytest.fixture
    def neuron_array(self):
        """Create a test neuron array."""
        mapping_provider = MockNeuronMappingProvider()
        return NeuronArray(max_neurons=100, mapping_provider=mapping_provider)
    
    def test_refractory_period_enforcement_core_fix(self, neuron_array):
        """Test that the core fix properly enforces refractory periods."""
        
        # Create a test neuron with high threshold and long refractory period
        neuron_id = neuron_array.create_neuron(
            cortical_idx=1,
            position=(0, 0, 0),
            threshold=1.0,
            membrane_potential=0.0,  # Start at 0
            resting_potential=0.0,
            leak_coefficient=0.9,
            refractory_period=5  # 5 timestep refractory period
        )
        
        neuron_index = neuron_array.mapping_provider.get_neuron_index(neuron_id)
        
        print(f"Created neuron {neuron_id} at index {neuron_index}")
        
        # STEP 1: Set membrane potential above threshold to force firing
        neuron_array.membrane_potentials[neuron_index] = 2.0  # Above threshold
        
        print(f"Set membrane potential to {neuron_array.membrane_potentials[neuron_index]}")
        print(f"Threshold: {neuron_array.thresholds[neuron_index]}")
        print(f"Refractory counter: {neuron_array.refractory_counters[neuron_index]}")
        
        # STEP 2: Run neural update - neuron should fire
        fired_neurons = neuron_array.embedded_optimized_neural_update(timestep=1)
        
        print(f"Fired neurons: {fired_neurons}")
        print(f"After firing - membrane potential: {neuron_array.membrane_potentials[neuron_index]}")
        print(f"After firing - refractory counter: {neuron_array.refractory_counters[neuron_index]}")
        
        # VALIDATION 1: Neuron should have fired
        assert neuron_id in fired_neurons, f"Neuron {neuron_id} should have fired!"
        
        # VALIDATION 2: Membrane potential should be reset to resting potential (CRITICAL FIX)
        assert neuron_array.membrane_potentials[neuron_index] == neuron_array.resting_potentials[neuron_index], \
            f"Membrane potential should be reset to resting potential after firing!"
        
        # VALIDATION 3: Refractory counter should be set to refractory period (CRITICAL FIX)
        assert neuron_array.refractory_counters[neuron_index] == 5, \
            f"Refractory counter should be set to 5 after firing, got {neuron_array.refractory_counters[neuron_index]}"
        
        print("✅ STEP 1 PASSED: Neuron fired and refractory counter was set!")
        
        # STEP 3: Try to fire again immediately - should be blocked by refractory period
        neuron_array.membrane_potentials[neuron_index] = 2.0  # Set high again
        fired_neurons = neuron_array.embedded_optimized_neural_update(timestep=2)
        
        print(f"Attempted to fire again - fired neurons: {fired_neurons}")
        print(f"Refractory counter: {neuron_array.refractory_counters[neuron_index]}")
        
        # VALIDATION 4: Neuron should NOT fire due to refractory period
        assert neuron_id not in fired_neurons, \
            f"Neuron {neuron_id} should NOT fire during refractory period!"
        
        # VALIDATION 5: Refractory counter should have decremented
        assert neuron_array.refractory_counters[neuron_index] == 4, \
            f"Refractory counter should have decremented to 4, got {neuron_array.refractory_counters[neuron_index]}"
        
        print("✅ STEP 2 PASSED: Neuron blocked by refractory period!")
        
        # STEP 4: Wait for refractory period to expire (4 more timesteps)
        for timestep in range(3, 7):  # timesteps 3, 4, 5, 6
            neuron_array.membrane_potentials[neuron_index] = 2.0  # Keep trying to fire
            fired_neurons = neuron_array.embedded_optimized_neural_update(timestep=timestep)
            
            print(f"Timestep {timestep}: refractory counter = {neuron_array.refractory_counters[neuron_index]}, fired = {neuron_id in fired_neurons}")
            
            if timestep < 6:  # Still in refractory period
                assert neuron_id not in fired_neurons, \
                    f"Neuron should still be in refractory period at timestep {timestep}"
            else:  # Refractory period should be over
                expected_counter = max(0, 5 - (timestep - 1))
                print(f"Expected counter: {expected_counter}, actual: {neuron_array.refractory_counters[neuron_index]}")
        
        # STEP 5: Neuron should be able to fire again after refractory period
        neuron_array.membrane_potentials[neuron_index] = 2.0
        fired_neurons = neuron_array.embedded_optimized_neural_update(timestep=7)
        
        print(f"After refractory period - fired neurons: {fired_neurons}")
        print(f"Refractory counter: {neuron_array.refractory_counters[neuron_index]}")
        
        # VALIDATION 6: Neuron should fire again after refractory period expires
        assert neuron_id in fired_neurons, \
            f"Neuron {neuron_id} should fire again after refractory period expires!"
        
        # VALIDATION 7: Refractory counter should be reset to refractory period again
        assert neuron_array.refractory_counters[neuron_index] == 5, \
            f"Refractory counter should be reset to 5 after second firing"
        
        print("✅ SUCCESS: Refractory period fix is working correctly!")
        
    def test_user_scenario_m_rig_refractory_10(self, neuron_array):
        """Test the specific user scenario: refractory_period=10 on m__rig equivalent neuron."""
        
        # Create neuron equivalent to m__rig with refractory_period=10
        neuron_id = neuron_array.create_neuron(
            cortical_idx=2,  # Represents m__rig cortical area
            position=(0, 0, 0),
            threshold=1.0,
            membrane_potential=0.0,
            resting_potential=0.0,
            leak_coefficient=0.9,
            refractory_period=10  # User's setting
        )
        
        neuron_index = neuron_array.mapping_provider.get_neuron_index(neuron_id)
        
        print(f"Testing user scenario: neuron with refractory_period=10")
        
        # Simulate power injection (high membrane potential)
        neuron_array.membrane_potentials[neuron_index] = 3.0  # High power
        
        # Fire once
        fired_neurons = neuron_array.embedded_optimized_neural_update(timestep=1)
        assert neuron_id in fired_neurons, "Neuron should fire initially"
        assert neuron_array.refractory_counters[neuron_index] == 10, "Refractory counter should be 10"
        
        print(f"✅ Neuron fired and refractory counter set to 10")
        
        # Try to fire for the next 10 timesteps - should be blocked
        for timestep in range(2, 12):  # timesteps 2-11 (10 timesteps)
            neuron_array.membrane_potentials[neuron_index] = 3.0  # Keep power high
            fired_neurons = neuron_array.embedded_optimized_neural_update(timestep=timestep)
            
            # Should NOT fire during refractory period
            assert neuron_id not in fired_neurons, \
                f"Neuron should NOT fire at timestep {timestep} (refractory period)"
            
            expected_counter = 10 - (timestep - 1)
            actual_counter = neuron_array.refractory_counters[neuron_index]
            print(f"Timestep {timestep}: refractory_counter={actual_counter} (expected={expected_counter})")
        
        # At timestep 12, refractory period should be over
        neuron_array.membrane_potentials[neuron_index] = 3.0
        fired_neurons = neuron_array.embedded_optimized_neural_update(timestep=12)
        
        assert neuron_id in fired_neurons, "Neuron should fire again after 10 timesteps"
        assert neuron_array.refractory_counters[neuron_index] == 10, "Refractory counter reset to 10"
        
        print("✅ SUCCESS: User scenario with refractory_period=10 works correctly!")
        print("✅ Neurons will NOT fire constantly anymore!") 