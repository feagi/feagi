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
Unit tests for batch neuron property updates.

Tests the Rust NPU batch update functions exposed through ConnectomeManager.
"""

import pytest
import numpy as np

from feagi.bdu.connectome_manager import ConnectomeManager, NeuronPropertyType
from feagi.utils.config import FeagiConfig


@pytest.fixture
def small_config():
    """Create a minimal FeagiConfig for testing."""
    config = FeagiConfig()
    config.set("connectome.max_neurons", 1000)
    config.set("connectome.max_synapses_per_neuron", 10)
    config.set("connectome.fcl_window_size", 3)
    return config


@pytest.fixture
def test_area(small_config):
    """Create a ConnectomeManager with a test cortical area."""
    ConnectomeManager.reset_singleton()
    cm = ConnectomeManager(small_config)
    
    # Add test cortical area
    cortical_id = cm.add_cortical_area(
        name="Test Area",
        dimensions=(10, 10, 10),
        position=(0, 0, 0),
        area_type="custom",
    )
    
    yield cm, cortical_id
    ConnectomeManager.reset_singleton()


@pytest.fixture
def connectome(test_area):
    """Create a ConnectomeManager with test neurons."""
    cm, cortical_id = test_area
    
    # Add 10 test neurons
    test_neuron_ids = []
    for i in range(10):
        neuron_id = cm.create_neuron(
            cortical_id=cortical_id,
            position=(i, 0, 0),
            threshold=10.0,
            refractory_period=1,
        )
        test_neuron_ids.append(neuron_id)
    
    yield cm, test_neuron_ids


class TestBatchNeuronUpdates:
    """Test batch neuron property updates via Rust NPU."""
    
    def test_batch_update_refractory_period(self, connectome):
        """Test batch updating refractory period."""
        cm, neuron_ids = connectome
        
        # Update first 5 neurons to refractory period 10
        target_ids = neuron_ids[:5]
        success = cm.batch_update_neuron_properties(
            neuron_ids=target_ids,
            property_name=NeuronPropertyType.REFRACTORY_PERIOD,
            values=10
        )
        
        assert success, "Batch update should succeed"
        
        # Verify updates
        for nid in target_ids:
            props = cm.batch_get_neuron_properties([nid], NeuronPropertyType.REFRACTORY_PERIOD)
            assert props[0] == 10, f"Neuron {nid} should have refractory period 10"
    
    def test_batch_update_threshold(self, connectome):
        """Test batch updating firing threshold."""
        cm, neuron_ids = connectome
        
        # Update all neurons to threshold 15.0
        success = cm.batch_update_neuron_properties(
            neuron_ids=neuron_ids,
            property_name=NeuronPropertyType.THRESHOLD,
            values=15.0
        )
        
        assert success, "Batch update should succeed"
        
        # Verify updates
        thresholds = cm.batch_get_neuron_properties(neuron_ids, NeuronPropertyType.THRESHOLD)
        assert np.allclose(thresholds, 15.0), "All neurons should have threshold 15.0"
    
    def test_batch_update_leak_coefficient(self, connectome):
        """Test batch updating leak coefficient (decay rate)."""
        cm, neuron_ids = connectome
        
        # Update leak coefficient to 0.5
        success = cm.batch_update_neuron_properties(
            neuron_ids=neuron_ids,
            property_name=NeuronPropertyType.DECAY_RATE,
            values=0.5
        )
        
        assert success, "Batch update should succeed"
        
        # Verify updates
        leak_coeffs = cm.batch_get_neuron_properties(neuron_ids, NeuronPropertyType.DECAY_RATE)
        assert np.allclose(leak_coeffs, 0.5), "All neurons should have leak coefficient 0.5"
    
    def test_batch_update_membrane_potential(self, connectome):
        """Test batch updating membrane potential."""
        cm, neuron_ids = connectome
        
        # Update membrane potential to 5.0
        success = cm.batch_update_neuron_properties(
            neuron_ids=neuron_ids,
            property_name=NeuronPropertyType.MEMBRANE_POTENTIAL,
            values=5.0
        )
        
        assert success, "Batch update should succeed"
        
        # Verify updates
        potentials = cm.batch_get_neuron_properties(neuron_ids, NeuronPropertyType.MEMBRANE_POTENTIAL)
        assert np.allclose(potentials, 5.0), "All neurons should have membrane potential 5.0"
    
    def test_batch_update_resting_potential(self, connectome):
        """Test batch updating resting potential."""
        cm, neuron_ids = connectome
        
        # Update resting potential to -1.0
        success = cm.batch_update_neuron_properties(
            neuron_ids=neuron_ids,
            property_name=NeuronPropertyType.RESTING_POTENTIAL,
            values=-1.0
        )
        
        assert success, "Batch update should succeed"
        
        # Verify updates
        resting = cm.batch_get_neuron_properties(neuron_ids, NeuronPropertyType.RESTING_POTENTIAL)
        assert np.allclose(resting, -1.0), "All neurons should have resting potential -1.0"
    
    def test_batch_update_with_list_of_values(self, connectome):
        """Test batch updating with different values per neuron."""
        cm, neuron_ids = connectome
        
        # Update each neuron with different threshold
        different_thresholds = [5.0 + i for i in range(len(neuron_ids))]
        success = cm.batch_update_neuron_properties(
            neuron_ids=neuron_ids,
            property_name=NeuronPropertyType.THRESHOLD,
            values=different_thresholds
        )
        
        assert success, "Batch update with list should succeed"
        
        # Verify updates
        thresholds = cm.batch_get_neuron_properties(neuron_ids, NeuronPropertyType.THRESHOLD)
        assert np.allclose(thresholds, different_thresholds), "Each neuron should have correct threshold"
    
    def test_batch_update_invalid_neuron_ids(self, connectome):
        """Test batch updating with invalid neuron IDs."""
        cm, neuron_ids = connectome
        
        # Try to update non-existent neurons (should not raise error, just skip invalid)
        invalid_ids = [999999, 888888]
        success = cm.batch_update_neuron_properties(
            neuron_ids=invalid_ids,
            property_name=NeuronPropertyType.THRESHOLD,
            values=10.0
        )
        
        # Should return False because no valid neurons were found
        assert not success, "Batch update with all invalid IDs should return False"
    
    def test_batch_update_mixed_valid_invalid_ids(self, connectome):
        """Test batch updating with mix of valid and invalid neuron IDs."""
        cm, neuron_ids = connectome
        
        # Mix valid and invalid IDs
        mixed_ids = neuron_ids[:3] + [999999, 888888]
        success = cm.batch_update_neuron_properties(
            neuron_ids=mixed_ids,
            property_name=NeuronPropertyType.THRESHOLD,
            values=12.0
        )
        
        # Should succeed because some neurons were valid
        assert success, "Batch update with some valid IDs should succeed"
        
        # Verify only valid neurons were updated
        for nid in neuron_ids[:3]:
            props = cm.batch_get_neuron_properties([nid], NeuronPropertyType.THRESHOLD)
            assert np.isclose(props[0], 12.0), f"Valid neuron {nid} should be updated"
    
    def test_batch_update_length_mismatch(self, connectome):
        """Test batch updating with mismatched neuron_ids and values lengths."""
        cm, neuron_ids = connectome
        
        # Try to update with mismatched lengths
        with pytest.raises(ValueError, match="Length of values.*must match"):
            cm.batch_update_neuron_properties(
                neuron_ids=neuron_ids,
                property_name=NeuronPropertyType.THRESHOLD,
                values=[10.0, 20.0]  # Only 2 values for 10 neurons
            )
    
    def test_batch_update_multiple_properties_sequentially(self, connectome):
        """Test updating multiple properties on same neurons."""
        cm, neuron_ids = connectome
        
        target_ids = neuron_ids[:5]
        
        # Update threshold
        success1 = cm.batch_update_neuron_properties(
            neuron_ids=target_ids,
            property_name=NeuronPropertyType.THRESHOLD,
            values=20.0
        )
        
        # Update refractory period
        success2 = cm.batch_update_neuron_properties(
            neuron_ids=target_ids,
            property_name=NeuronPropertyType.REFRACTORY_PERIOD,
            values=5
        )
        
        # Update membrane potential
        success3 = cm.batch_update_neuron_properties(
            neuron_ids=target_ids,
            property_name=NeuronPropertyType.MEMBRANE_POTENTIAL,
            values=3.0
        )
        
        assert all([success1, success2, success3]), "All updates should succeed"
        
        # Verify all properties
        thresholds = cm.batch_get_neuron_properties(target_ids, NeuronPropertyType.THRESHOLD)
        refractory = cm.batch_get_neuron_properties(target_ids, NeuronPropertyType.REFRACTORY_PERIOD)
        potentials = cm.batch_get_neuron_properties(target_ids, NeuronPropertyType.MEMBRANE_POTENTIAL)
        
        assert np.allclose(thresholds, 20.0), "Thresholds should be updated"
        assert np.all(refractory == 5), "Refractory periods should be updated"
        assert np.allclose(potentials, 3.0), "Membrane potentials should be updated"
    
    def test_batch_update_unsupported_property(self, connectome):
        """Test batch updating unsupported property raises error."""
        cm, neuron_ids = connectome
        
        # Try to update POSITION (not supported as per user request)
        with pytest.raises(NotImplementedError):
            cm.batch_update_neuron_properties(
                neuron_ids=neuron_ids,
                property_name=NeuronPropertyType.POSITION,
                values=(1, 2, 3)
            )
    
    def test_batch_update_via_string_property_name(self, connectome):
        """Test batch updating using string property names."""
        cm, neuron_ids = connectome
        
        # Use string instead of enum
        success = cm.batch_update_neuron_properties(
            neuron_ids=neuron_ids,
            property_name="threshold",  # String name
            values=18.0
        )
        
        assert success, "Batch update with string property name should work"
        
        # Verify updates
        thresholds = cm.batch_get_neuron_properties(neuron_ids, NeuronPropertyType.THRESHOLD)
        assert np.allclose(thresholds, 18.0), "Thresholds should be updated"
    
    def test_batch_update_type_conversions(self, connectome):
        """Test batch updating handles type conversions correctly."""
        cm, neuron_ids = connectome
        
        # Test float to int conversion for refractory period
        success = cm.batch_update_neuron_properties(
            neuron_ids=neuron_ids,
            property_name=NeuronPropertyType.REFRACTORY_PERIOD,
            values=7.9  # Should be converted to int(8)
        )
        
        assert success, "Batch update with float for u16 should succeed"
        
        # Verify conversion happened
        refractory = cm.batch_get_neuron_properties(neuron_ids, NeuronPropertyType.REFRACTORY_PERIOD)
        # Should be rounded to 8
        assert np.all(refractory == 8), "Float should be converted to int"


class TestBatchUpdateIntegration:
    """Integration tests for batch updates with API workflow."""
    
    def test_api_workflow_refractory_period_update(self, connectome):
        """Test the full API workflow for updating refractory period (original failing case)."""
        cm, neuron_ids = connectome
        
        # Get the cortical area (it's the first one added in the fixture)
        cortical_areas = list(cm.cortical_areas.values())
        assert len(cortical_areas) > 0, "Should have at least one cortical area"
        cortical_area = cortical_areas[0]
        
        # Get all neurons in the area (use cortical_idx method)
        area_neuron_ids = cm.get_neurons_by_cortical_idx(cortical_area.cortical_idx)
        assert len(area_neuron_ids) > 0, "Area should have neurons"
        
        # Update refractory period (the original failing case)
        success = cm.batch_update_neuron_properties(
            neuron_ids=area_neuron_ids,
            property_name=NeuronPropertyType.REFRACTORY_PERIOD,
            values=1  # From the original error log
        )
        
        assert success, "API workflow update should succeed"
        
        # Verify all neurons in area were updated
        refractory_periods = cm.batch_get_neuron_properties(
            area_neuron_ids, 
            NeuronPropertyType.REFRACTORY_PERIOD
        )
        assert np.all(refractory_periods == 1), "All neurons should have refractory period 1"
    
    def test_stress_batch_update_large_batch(self, small_config):
        """Test batch updating large number of neurons."""
        ConnectomeManager.reset_singleton()
        
        # Create larger config for stress test
        large_config = FeagiConfig()
        large_config.set("connectome.max_neurons", 10000)
        large_config.set("connectome.max_synapses_per_neuron", 10)
        
        cm = ConnectomeManager(large_config)
        
        # Add cortical area
        cortical_id = cm.add_cortical_area(
            name="Large Area",
            dimensions=(100, 100, 1),
            position=(0, 0, 0),
            area_type="custom",
        )
        
        # Add 1000 neurons
        neuron_ids = []
        for i in range(1000):
            x = i % 100
            y = i // 100
            nid = cm.create_neuron(
                cortical_id=cortical_id,
                position=(x, y, 0),
                threshold=10.0,
            )
            neuron_ids.append(nid)
        
        # Batch update all 1000 neurons
        import time
        start = time.time()
        success = cm.batch_update_neuron_properties(
            neuron_ids=neuron_ids,
            property_name=NeuronPropertyType.THRESHOLD,
            values=25.0
        )
        duration = time.time() - start
        
        assert success, "Large batch update should succeed"
        assert duration < 0.1, f"Batch update should be fast (<100ms), got {duration*1000:.1f}ms"
        
        # Verify updates (sample check)
        sample_ids = neuron_ids[::100]  # Every 100th neuron
        thresholds = cm.batch_get_neuron_properties(sample_ids, NeuronPropertyType.THRESHOLD)
        assert np.allclose(thresholds, 25.0), "Sample neurons should be updated"
        
        ConnectomeManager.reset_singleton()


if __name__ == "__main__":
    # Run tests with pytest
    pytest.main([__file__, "-v", "-s"])

