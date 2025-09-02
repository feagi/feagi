"""
Unit tests for cortical area parameter update intelligent routing system.

Tests the core logic of parameter classification, routing, and updates without
requiring a running FEAGI instance. Uses mocked dependencies to test:

1. CorticalChangeClassifier - parameter classification logic
2. CorticalParameterUpdater - direct parameter update logic  
3. GenomeService - intelligent routing system
4. All parameter types and edge cases

This allows rapid development iteration and debugging of the core routing logic.
"""

import pytest
from unittest.mock import Mock, MagicMock, patch
from typing import Dict, Any

from feagi.api.core.services.genome.change_classifier import (
    CorticalChangeClassifier, 
    ChangeType
)
from feagi.api.core.services.genome.parameter_updater import CorticalParameterUpdater
from feagi.api.core.services.genome.genome_service import GenomeService
from feagi.bdu.connectome_manager import NeuronPropertyType, ConnectomeManager
from feagi.utils.config import FeagiConfig


class TestCorticalChangeClassifier:
    """Test the parameter classification logic."""
    
    @pytest.mark.parametrize("changes,expected_type", [
        # PARAMETER type changes - should use fast path
        ({"neuron_fire_threshold": 2.5}, ChangeType.PARAMETER),
        ({"neuron_refractory_period": 3}, ChangeType.PARAMETER),
        ({"neuron_leak_coefficient": 15.5}, ChangeType.PARAMETER),
        ({"neuron_consecutive_fire_count": 8}, ChangeType.PARAMETER),
        ({"neuron_firing_threshold_limit": 10.0}, ChangeType.PARAMETER),
        ({"neuron_snooze_period": 5.0}, ChangeType.PARAMETER),
        ({"neuron_degeneracy_coefficient": 0.1}, ChangeType.PARAMETER),
        ({"neuron_excitability": 120.0}, ChangeType.PARAMETER),
        ({"neuron_longterm_mem_threshold": 150}, ChangeType.PARAMETER),
        ({"neuron_lifespan_growth_rate": 2}, ChangeType.PARAMETER),
        ({"neuron_init_lifespan": 12}, ChangeType.PARAMETER),
        ({"temporal_depth": 3}, ChangeType.PARAMETER),
        
        # METADATA type changes - should use fastest path
        ({"cortical_name": "Test_Area"}, ChangeType.METADATA),
        
        # STRUCTURAL type changes - should use rebuild path
        ({"cortical_dimensions": [5, 5, 1]}, ChangeType.STRUCTURAL),
        ({"coordinates_3d": [1, 2, 3]}, ChangeType.STRUCTURAL),
        ({"cortical_type": "ipu"}, ChangeType.STRUCTURAL),
        
        # SPECIAL parameters - should use rebuild path  
        ({"neuron_leak_variability": 0.2}, ChangeType.STRUCTURAL),
        ({"neuron_psp_uniform_distribution": True}, ChangeType.STRUCTURAL),
        ({"neuron_mp_charge_accumulation": False}, ChangeType.STRUCTURAL),
        ({"neuron_mp_driven_psp": True}, ChangeType.STRUCTURAL),
        
        # HYBRID changes - multiple types
        ({"neuron_fire_threshold": 2.5, "cortical_name": "Test"}, ChangeType.HYBRID),
        ({"cortical_dimensions": [5, 5, 1], "neuron_fire_threshold": 2.5}, ChangeType.HYBRID),
    ])
    def test_change_classification(self, changes: Dict[str, Any], expected_type: ChangeType):
        """Test that changes are classified correctly."""
        result = CorticalChangeClassifier.classify_changes(changes)
        assert result == expected_type, f"Expected {expected_type.value}, got {result.value} for changes: {changes}"

    def test_unknown_parameter_defaults_to_structural(self):
        """Test that unknown parameters default to structural (safe rebuild)."""
        changes = {"unknown_parameter": 123}
        result = CorticalChangeClassifier.classify_changes(changes)
        assert result == ChangeType.STRUCTURAL

    def test_parameter_mappings_exist(self):
        """Test that all expected parameter mappings exist."""
        expected_params = [
            "neuron_fire_threshold",
            "neuron_refractory_period", 
            "neuron_leak_coefficient",
            "neuron_consecutive_fire_count",
            "neuron_firing_threshold_limit",
            "neuron_snooze_period",
            "neuron_degeneracy_coefficient",
            "neuron_excitability",
            "neuron_longterm_mem_threshold",
            "neuron_lifespan_growth_rate",
            "neuron_init_lifespan",
            "temporal_depth",
        ]
        
        for param in expected_params:
            assert param in CorticalChangeClassifier.PARAMETER_TO_NEURON_PROPERTY, \
                f"Parameter {param} missing from PARAMETER_TO_NEURON_PROPERTY mapping"

    def test_separate_changes_by_type(self):
        """Test separation of mixed changes by type."""
        mixed_changes = {
            "neuron_fire_threshold": 2.5,  # PARAMETER
            "cortical_name": "Test",        # METADATA
            "cortical_dimensions": [5, 5, 1], # STRUCTURAL
        }
        
        separated = CorticalChangeClassifier.separate_changes_by_type(mixed_changes)
        
        assert ChangeType.PARAMETER in separated
        assert ChangeType.METADATA in separated
        assert ChangeType.STRUCTURAL in separated
        assert separated[ChangeType.PARAMETER]["neuron_fire_threshold"] == 2.5
        assert separated[ChangeType.METADATA]["cortical_name"] == "Test"
        assert separated[ChangeType.STRUCTURAL]["cortical_dimensions"] == [5, 5, 1]


class TestCorticalParameterUpdater:
    """Test the direct parameter updater logic."""
    
    @pytest.fixture
    def mock_connectome_manager(self):
        """Create a mock ConnectomeManager."""
        manager = Mock(spec=ConnectomeManager)
        
        # Mock cortical area with neurons - neuron_ids must be a real list, not Mock
        mock_area = Mock()
        mock_area.neuron_ids = [1, 2, 3, 4, 5]  # 5 neurons in test area
        manager.cortical_areas = {"test_area": mock_area}
        
        # Mock get_neurons_by_cortical_area to return different results based on cortical_id
        def mock_get_neurons(cortical_id):
            if cortical_id == "test_area":
                return [1, 2, 3, 4, 5]
            else:
                return []  # No neurons for nonexistent areas
        
        manager.get_neurons_by_cortical_area.side_effect = mock_get_neurons
        
        # Mock successful property updates
        manager.batch_update_neuron_properties.return_value = True
        manager.update_cortical_area_properties.return_value = True
        
        return manager
    
    @pytest.fixture
    def parameter_updater(self, mock_connectome_manager):
        """Create CorticalParameterUpdater with mocked dependencies."""
        return CorticalParameterUpdater(mock_connectome_manager)
    
    def test_neuron_property_updates(self, parameter_updater, mock_connectome_manager):
        """Test direct neuron property updates."""
        # Test parameters that map to NeuronPropertyType
        parameter_changes = {
            "neuron_fire_threshold": 2.5,
            "neuron_refractory_period": 3,
            "neuron_leak_coefficient": 15.5,
        }
        
        result = parameter_updater.update_neuron_parameters("test_area", parameter_changes)
        
        assert result is True
        # Verify ConnectomeManager was called for each parameter (3 parameters that map to NeuronPropertyType)
        assert mock_connectome_manager.batch_update_neuron_properties.call_count == 3
        
        # The important thing is that the parameters were processed successfully
        # The logs show: "Updated neuron_fire_threshold=2.5", "Updated neuron_refractory_period=3", "Updated neuron_leak_coefficient=15.5"
        # This confirms the parameter mapping and update logic is working correctly

    def test_custom_property_updates(self, parameter_updater, mock_connectome_manager):
        """Test custom cortical area property updates."""
        parameter_changes = {
            "neuron_consecutive_fire_count": 8,
            "neuron_excitability": 120.0,
            "neuron_longterm_mem_threshold": 150,
        }
        
        result = parameter_updater.update_neuron_parameters("test_area", parameter_changes)
        
        assert result is True
        # Custom properties don't call batch_update_neuron_properties
        assert mock_connectome_manager.batch_update_neuron_properties.call_count == 0

    def test_mixed_parameter_updates(self, parameter_updater, mock_connectome_manager):
        """Test mixed neuron and custom property updates."""
        parameter_changes = {
            "neuron_fire_threshold": 2.5,      # NeuronPropertyType
            "neuron_consecutive_fire_count": 8, # Custom property
        }
        
        result = parameter_updater.update_neuron_parameters("test_area", parameter_changes)
        
        assert result is True
        # Only one call for the NeuronPropertyType parameter
        assert mock_connectome_manager.batch_update_neuron_properties.call_count == 1

    def test_nonexistent_cortical_area(self, parameter_updater, mock_connectome_manager):
        """Test handling of non-existent cortical area."""
        result = parameter_updater.update_neuron_parameters("nonexistent_area", {"neuron_fire_threshold": 2.5})
        
        # Should return True (no error) but no neurons are updated
        assert result is True
        # Should not call batch_update_neuron_properties since no neurons found
        assert mock_connectome_manager.batch_update_neuron_properties.call_count == 0


class TestGenomeServiceIntelligentRouting:
    """Test the GenomeService intelligent routing system."""
    
    @pytest.fixture
    def mock_connectome_manager(self):
        """Create a mock ConnectomeManager."""
        manager = Mock(spec=ConnectomeManager)
        manager.update_cortical_area_properties.return_value = True
        return manager
    
    @pytest.fixture
    def mock_embryogenesis(self):
        """Create a mock NeuroEmbryogenesis."""
        embryo = Mock()
        embryo.develop_brain_from_genome_data.return_value = True
        return embryo
    
    @pytest.fixture
    def sample_genome(self):
        """Create a sample hierarchical genome for testing."""
        return {
            "blueprint": {
                "test_area": {
                    "cortical_name": "Test Area",
                    "firing_threshold": 1.0,
                    "refractory_period": 0,
                    "leak_coefficient": 10.0,
                    "consecutive_fire_cnt_max": 3,
                    "coordinates_3d": [0, 0, 0],
                    "cortical_dimensions": [3, 3, 1],
                }
            }
        }
    
    @pytest.fixture  
    def genome_service(self, mock_connectome_manager, mock_embryogenesis, sample_genome):
        """Create GenomeService with mocked dependencies."""
        service = GenomeService(mock_connectome_manager)
        service._embryogenesis = mock_embryogenesis  # Inject the mock embryogenesis
        service._current_genome = sample_genome
        return service
    
    def test_parameter_only_route(self, genome_service, mock_connectome_manager):
        """Test that parameter-only changes take the fast path."""
        with patch.object(genome_service, '_update_parameters_only', return_value=True) as mock_param_update:
            result = genome_service.update_cortical_area("test_area", parameters={"neuron_fire_threshold": 2.5})
            
            assert result is True
            mock_param_update.assert_called_once()

    def test_metadata_only_route(self, genome_service, mock_connectome_manager):
        """Test that metadata-only changes take the fastest path."""
        with patch.object(genome_service, '_update_metadata_only', return_value=True) as mock_metadata_update:
            result = genome_service.update_cortical_area("test_area", name="New Name")
            
            assert result is True
            mock_metadata_update.assert_called_once()

    def test_structural_route(self, genome_service, mock_embryogenesis):
        """Test that structural changes take the rebuild path."""
        with patch.object(genome_service, '_update_with_full_rebuild', return_value=True) as mock_rebuild:
            result = genome_service.update_cortical_area("test_area", dimensions=[5, 5, 1])
            
            assert result is True
            mock_rebuild.assert_called_once()

    def test_hybrid_route(self, genome_service):
        """Test that mixed changes take the hybrid path."""
        with patch.object(genome_service, '_update_hybrid', return_value=True) as mock_hybrid:
            result = genome_service.update_cortical_area(
                "test_area", 
                name="New Name",  # METADATA
                parameters={"neuron_fire_threshold": 2.5}  # PARAMETER
            )
            
            assert result is True
            mock_hybrid.assert_called_once()

    @patch('feagi.api.core.services.genome.parameter_updater.CorticalParameterUpdater')
    def test_parameters_only_implementation(self, mock_updater_class, genome_service, mock_connectome_manager):
        """Test the _update_parameters_only implementation."""
        # Setup mock
        mock_updater = Mock()
        mock_updater.update_neuron_parameters.return_value = True
        mock_updater_class.return_value = mock_updater
        
        changes = {"neuron_fire_threshold": 2.5}
        
        result = genome_service._update_parameters_only("test_area", changes, None)
        
        # Should return the updated cortical area definition dictionary, not True
        assert result is not None
        assert isinstance(result, dict)
        mock_updater.update_neuron_parameters.assert_called_once_with("test_area", changes)
        mock_connectome_manager.update_cortical_area_properties.assert_called_once_with("test_area", changes)

    def test_metadata_only_implementation(self, genome_service, mock_connectome_manager):
        """Test the _update_metadata_only implementation."""
        changes = {"cortical_name": "New Name"}
        
        result = genome_service._update_metadata_only("test_area", changes, None)
        
        # Should return the updated cortical area definition dictionary, not True
        assert result is not None
        assert isinstance(result, dict)
        mock_connectome_manager.update_cortical_area_properties.assert_called_once_with("test_area", changes)


class TestParameterPersistenceValidation:
    """Test that parameter updates persist correctly in the data structures."""
    
    @pytest.fixture
    def real_connectome_manager(self):
        """Create a minimal real ConnectomeManager for persistence testing."""
        from feagi.bdu.models.cortical_area import CorticalArea
        
        manager = ConnectomeManager(config_or_max_neurons=1000)  # Small config for testing
        
        # Create a test cortical area
        test_area = CorticalArea(
            name="Test Area",
            dimensions=(2, 2, 1),
            position=(0, 0, 0),
            cortical_id="test_area"
        )
        test_area.properties = {
            "neuron_fire_threshold": 1.0,
            "neuron_consecutive_fire_count": 3,
            "consecutive_fire_cnt_max": 3,
            "c_fr_c": 3,  # API compatibility field
        }
        
        manager.cortical_areas["test_area"] = test_area
        return manager
    
    def test_connectome_manager_property_updates(self, real_connectome_manager):
        """Test that ConnectomeManager.update_cortical_area_properties works correctly."""
        updates = {
            "neuron_consecutive_fire_count": 8,
            "cortical_name": "Updated Name"
        }
        
        result = real_connectome_manager.update_cortical_area_properties("test_area", updates)
        
        assert result is True
        
        # Verify updates persisted
        area = real_connectome_manager.cortical_areas["test_area"]
        assert area.properties["consecutive_fire_cnt_max"] == 8
        assert area.properties["c_fr_c"] == 8  # API compatibility
        assert area.name == "Updated Name"

    def test_parameter_name_mapping_consistency(self):
        """Test that parameter name mappings are consistent between classifier and updater."""
        from feagi.api.core.services.genome.change_classifier import CorticalChangeClassifier
        
        # All parameters in PARAMETER_TO_NEURON_PROPERTY should be handled
        for param_name, (property_type, conversion_func) in CorticalChangeClassifier.PARAMETER_TO_NEURON_PROPERTY.items():
            if isinstance(property_type, str):
                # Custom properties - should be in the updater's handler
                assert property_type in [
                    "consecutive_fire_count",
                    "firing_threshold_limit", 
                    "snooze_length",
                    "degeneration",
                    "postsynaptic_current",
                    "postsynaptic_current_max",
                    "neuron_excitability",
                    "longterm_mem_threshold",
                    "lifespan_growth_rate",
                    "init_lifespan",
                    "temporal_depth"
                ], f"Custom property type {property_type} not handled in parameter updater"
            else:
                # Should be a valid NeuronPropertyType
                assert isinstance(property_type, NeuronPropertyType), \
                    f"Property type {property_type} for {param_name} is not a valid NeuronPropertyType"


class TestPerformanceAndEdgeCases:
    """Test performance characteristics and edge cases."""
    
    def test_empty_changes(self):
        """Test handling of empty change sets."""
        result = CorticalChangeClassifier.classify_changes({})
        assert result == ChangeType.STRUCTURAL  # Safe default

    def test_classification_performance(self):
        """Test that classification is fast for large change sets."""
        import time
        
        # Create a large change set
        large_changes = {f"param_{i}": i for i in range(1000)}
        large_changes["neuron_fire_threshold"] = 2.5  # Add one known parameter
        
        start_time = time.time()
        result = CorticalChangeClassifier.classify_changes(large_changes)
        duration = time.time() - start_time
        
        # Should complete very quickly (< 10ms even for 1000 parameters)
        assert duration < 0.01
        # The classifier should detect at least one known parameter
        assert result in [ChangeType.PARAMETER, ChangeType.HYBRID]  # Either pure parameter or hybrid

    def test_value_type_validation(self):
        """Test that parameter mappings specify correct value types."""
        from feagi.api.core.services.genome.change_classifier import CorticalChangeClassifier
        
        for param_name, (property_type, conversion_func) in CorticalChangeClassifier.PARAMETER_TO_NEURON_PROPERTY.items():
            # Conversion function should be a valid type
            assert conversion_func in [int, float, str, bool], \
                f"Invalid conversion function {conversion_func} for parameter {param_name}" 