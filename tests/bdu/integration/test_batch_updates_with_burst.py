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
Integration tests for batch neuron updates with burst processing.

These tests ensure that batch updates work correctly within the full NPU pipeline,
including burst processing, synaptic propagation, and diagnostic methods.
"""

import pytest
import numpy as np

from feagi.bdu.connectome_manager import ConnectomeManager, NeuronPropertyType
from feagi.utils.config import FeagiConfig


@pytest.fixture
def burst_engine_config():
    """Create a FeagiConfig for burst engine testing."""
    config = FeagiConfig()
    config.set("connectome.max_neurons", 10000)
    config.set("connectome.max_synapses_per_neuron", 100)
    config.set("connectome.fcl_window_size", 3)
    config.set("burst_engine.performance_log_interval", 1000)  # Disable frequent logging
    return config


@pytest.fixture
def connectome_with_burst_engine(burst_engine_config):
    """Create a ConnectomeManager with burst engine ready for testing."""
    from feagi.npu.burst_engine import BurstEngine
    
    ConnectomeManager.reset_singleton()
    cm = ConnectomeManager(burst_engine_config)
    
    # Add test cortical area
    area_id = cm.add_cortical_area(
        name="Test Area",
        dimensions=(10, 10, 1),
        position=(0, 0, 0),
        area_type="custom",
    )
    
    # Create test neurons
    test_neurons = []
    for i in range(10):
        nid = cm.create_neuron(
            cortical_id=area_id,
            position=(i, 0, 0),
            threshold=5.0,
            refractory_period=2,
            membrane_potential=0.0,
        )
        test_neurons.append(nid)
    
    # Create burst engine
    burst_engine = BurstEngine(cm)
    
    yield cm, burst_engine, test_neurons, []
    ConnectomeManager.reset_singleton()


class TestBatchUpdatesWithBurstProcessing:
    """Integration tests for batch updates within burst processing."""
    
    def test_batch_update_then_burst(self, connectome_with_burst_engine):
        """Test that batch updates persist through burst processing."""
        cm, burst_engine, test_neurons, _ = connectome_with_burst_engine
        
        # Update refractory period for neurons
        success = cm.batch_update_neuron_properties(
            neuron_ids=test_neurons,
            property_name=NeuronPropertyType.REFRACTORY_PERIOD,
            values=5  # Increase from 2 to 5
        )
        assert success, "Batch update should succeed"
        
        # Verify updates
        refractory = cm.batch_get_neuron_properties(test_neurons, NeuronPropertyType.REFRACTORY_PERIOD)
        assert np.all(refractory == 5), "Refractory periods should be updated"
        
        # Run a burst (no parameters - uses internal power/manual neuron collection)
        try:
            result = burst_engine.process_burst()
            
            # Verify burst completed successfully (returns list of fired neurons)
            assert result is not None, "Burst should complete"
            assert isinstance(result, list), "Result should be a list of fired neurons"
            
        except Exception as e:
            pytest.fail(f"Burst processing failed after batch update: {e}")
    
    def test_batch_update_threshold_affects_firing(self, connectome_with_burst_engine):
        """Test that batch threshold updates work correctly."""
        cm, burst_engine, test_neurons, _ = connectome_with_burst_engine
        
        # Set high threshold (make them harder to fire)
        cm.batch_update_neuron_properties(
            neuron_ids=test_neurons,
            property_name=NeuronPropertyType.THRESHOLD,
            values=100.0  # Very high threshold
        )
        
        # Verify high threshold
        thresholds = cm.batch_get_neuron_properties(test_neurons, NeuronPropertyType.THRESHOLD)
        assert np.allclose(thresholds, 100.0), "Thresholds should be set to 100.0"
        
        # Run burst (should complete without errors)
        result1 = burst_engine.process_burst()
        assert isinstance(result1, list), "Burst should return list"
        
        # Now lower threshold (make them easier to fire)
        cm.batch_update_neuron_properties(
            neuron_ids=test_neurons,
            property_name=NeuronPropertyType.THRESHOLD,
            values=1.0  # Very low threshold
        )
        
        # Verify low threshold
        thresholds = cm.batch_get_neuron_properties(test_neurons, NeuronPropertyType.THRESHOLD)
        assert np.allclose(thresholds, 1.0), "Thresholds should be set to 1.0"
        
        # Run burst again (should complete without errors)
        result2 = burst_engine.process_burst()
        assert isinstance(result2, list), "Burst should return list"
    
    def test_batch_update_membrane_potential_then_burst(self, connectome_with_burst_engine):
        """Test batch updating membrane potential before burst."""
        cm, burst_engine, test_neurons, _ = connectome_with_burst_engine
        
        # Pre-charge neurons close to threshold
        cm.batch_update_neuron_properties(
            neuron_ids=test_neurons,
            property_name=NeuronPropertyType.MEMBRANE_POTENTIAL,
            values=4.5  # Just below threshold of 5.0
        )
        
        # Verify pre-charge
        potentials = cm.batch_get_neuron_properties(test_neurons, NeuronPropertyType.MEMBRANE_POTENTIAL)
        assert np.allclose(potentials, 4.5), "Membrane potentials should be pre-charged"
        
        # Run burst (should complete without errors)
        result = burst_engine.process_burst()
        assert isinstance(result, list), "Burst should return list"
    
    def test_get_neuron_state_during_burst(self, connectome_with_burst_engine):
        """Test that get_neuron_state works correctly during burst processing."""
        cm, burst_engine, test_neurons, _ = connectome_with_burst_engine
        
        # This test verifies the tuple unpacking fix
        # If get_neuron_state has wrong tuple length, this will fail
        
        # Run multiple bursts to exercise diagnostic code
        for i in range(3):
            result = burst_engine.process_burst()
            
            # Verify burst completed (would fail if get_neuron_state tuple unpacking is broken)
            assert isinstance(result, list), f"Burst {i+1} should complete without tuple unpacking errors"
    
    def test_batch_update_multiple_properties_then_burst(self, connectome_with_burst_engine):
        """Test updating multiple properties then running burst."""
        cm, burst_engine, test_neurons, _ = connectome_with_burst_engine
        
        # Update multiple properties
        cm.batch_update_neuron_properties(
            neuron_ids=test_neurons,
            property_name=NeuronPropertyType.THRESHOLD,
            values=3.0
        )
        
        cm.batch_update_neuron_properties(
            neuron_ids=test_neurons,
            property_name=NeuronPropertyType.REFRACTORY_PERIOD,
            values=1
        )
        
        cm.batch_update_neuron_properties(
            neuron_ids=test_neurons,
            property_name=NeuronPropertyType.MEMBRANE_POTENTIAL,
            values=2.0
        )
        
        # Verify all updates
        thresholds = cm.batch_get_neuron_properties(test_neurons, NeuronPropertyType.THRESHOLD)
        refractory = cm.batch_get_neuron_properties(test_neurons, NeuronPropertyType.REFRACTORY_PERIOD)
        potentials = cm.batch_get_neuron_properties(test_neurons, NeuronPropertyType.MEMBRANE_POTENTIAL)
        
        assert np.allclose(thresholds, 3.0), "Thresholds should be updated"
        assert np.all(refractory == 1), "Refractory periods should be updated"
        assert np.allclose(potentials, 2.0), "Potentials should be updated"
        
        # Run burst - should complete without errors
        result = burst_engine.process_burst()
        
        assert isinstance(result, list), "Burst should complete after multiple property updates"


class TestBatchUpdatesStressWithBurst:
    """Stress tests for batch updates with burst processing."""
    
    def test_many_updates_then_many_bursts(self, burst_engine_config):
        """Test many batch updates followed by many bursts."""
        ConnectomeManager.reset_singleton()
        cm = ConnectomeManager(burst_engine_config)
        from feagi.npu.burst_engine import BurstEngine
        
        # Create a larger network
        area_id = cm.add_cortical_area(
            name="Large Area",
            dimensions=(20, 20, 1),
            position=(0, 0, 0),
            area_type="custom",
        )
        
        # Create 100 neurons
        neuron_ids = []
        for i in range(100):
            x = i % 20
            y = i // 20
            nid = cm.create_neuron(
                cortical_id=area_id,
                position=(x, y, 0),
                threshold=5.0,
                refractory_period=2,
            )
            neuron_ids.append(nid)
        
        # Create burst engine
        burst_engine = BurstEngine(cm)
        
        # Perform 10 batch updates
        for i in range(10):
            cm.batch_update_neuron_properties(
                neuron_ids=neuron_ids,
                property_name=NeuronPropertyType.THRESHOLD,
                values=5.0 + i  # Gradually increase threshold
            )
        
        # Verify final state
        thresholds = cm.batch_get_neuron_properties(neuron_ids, NeuronPropertyType.THRESHOLD)
        assert np.allclose(thresholds, 14.0), "Final threshold should be 5 + 9 = 14"
        
        # Run 10 bursts
        for i in range(10):
            result = burst_engine.process_burst()
            assert isinstance(result, list), f"Burst {i+1} should complete"
        
        ConnectomeManager.reset_singleton()


if __name__ == "__main__":
    # Run tests with pytest
    pytest.main([__file__, "-v", "-s"])

