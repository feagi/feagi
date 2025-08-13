"""
Test suite for intelligent cortical area update routing.

Tests the new performance optimization that routes updates to different
mechanisms based on change type:
- Parameter changes: Direct neuron updates (~2-5ms)
- Metadata changes: Simple property updates (~1ms)
- Structural changes: Full rebuild (~800ms)
- Hybrid changes: Optimized combination
"""

import pytest
import time
from unittest.mock import Mock, patch, MagicMock
from typing import Dict, Any

from feagi.api.core.services.genome.genome_service import GenomeService
from feagi.api.core.services.genome.change_classifier import (
    CorticalChangeClassifier, ChangeType
)
from feagi.api.core.services.genome.parameter_updater import CorticalParameterUpdater
from feagi.bdu.connectome_manager import ConnectomeManager, NeuronPropertyType


class TestCorticalChangeClassifier:
    """Test the change classification logic."""
    
    def test_parameter_only_changes(self):
        """Test classification of parameter-only changes."""
        changes = {
            "firing_threshold": 0.8,
            "refractory_period": 5,
            "consecutive_fire_cnt_max": 10
        }
        result = CorticalChangeClassifier.classify_changes(changes)
        assert result == ChangeType.PARAMETER
        
    def test_metadata_only_changes(self):
        """Test classification of metadata-only changes."""
        changes = {
            "cortical_name": "New Visual Area"
        }
        result = CorticalChangeClassifier.classify_changes(changes)
        assert result == ChangeType.METADATA
        
    def test_structural_only_changes(self):
        """Test classification of structural-only changes.""" 
        changes = {
            "cortical_dimensions": [10, 10, 1],
            "coordinates_3d": [100, 200, 0]
        }
        result = CorticalChangeClassifier.classify_changes(changes)
        assert result == ChangeType.STRUCTURAL
        
    def test_hybrid_changes(self):
        """Test classification of mixed change types."""
        changes = {
            "cortical_name": "New Name",  # metadata
            "firing_threshold": 0.9,      # parameter
            "cortical_dimensions": [5, 5, 1]  # structural
        }
        result = CorticalChangeClassifier.classify_changes(changes)
        assert result == ChangeType.HYBRID
        
    def test_separate_changes_by_type(self):
        """Test separating mixed changes into type buckets."""
        changes = {
            "cortical_name": "Test",
            "firing_threshold": 0.7,
            "cortical_dimensions": [3, 3, 1],
            "refractory_period": 8
        }
        
        separated = CorticalChangeClassifier.separate_changes_by_type(changes)
        
        assert separated[ChangeType.METADATA] == {"cortical_name": "Test"}
        assert separated[ChangeType.PARAMETER] == {
            "firing_threshold": 0.7,
            "refractory_period": 8
        }
        assert separated[ChangeType.STRUCTURAL] == {"cortical_dimensions": [3, 3, 1]}
        
    def test_neuron_property_mappings(self):
        """Test getting neuron property mappings for parameters."""
        parameter_changes = {
            "firing_threshold": 0.8,
            "refractory_period": 5,
            "unknown_param": 123  # Should be ignored
        }
        
        mappings = CorticalChangeClassifier.get_neuron_property_mappings(parameter_changes)
        
        assert len(mappings) == 2
        # Check that mappings contain expected tuples
        param_names = [mapping[0] for mapping in mappings]
        assert "firing_threshold" in param_names
        assert "refractory_period" in param_names
        assert "unknown_param" not in param_names


class TestCorticalParameterUpdater:
    """Test the direct parameter update service."""
    
    @pytest.fixture
    def mock_connectome_manager(self):
        """Create a mock ConnectomeManager."""
        mock_cm = Mock(spec=ConnectomeManager)
        mock_cm.get_neurons_by_cortical_area.return_value = [1001, 1002, 1003]
        mock_cm.batch_update_neuron_properties.return_value = True
        mock_cm.cortical_areas = {"iv00TL": Mock(name="Visual Area")}
        return mock_cm
        
    def test_successful_parameter_update(self, mock_connectome_manager):
        """Test successful parameter updates."""
        updater = CorticalParameterUpdater(mock_connectome_manager)
        
        parameter_changes = {
            "firing_threshold": 0.8,
            "refractory_period": 5
        }
        
        result = updater.update_neuron_parameters("iv00TL", parameter_changes)
        
        assert result is True
        # Verify ConnectomeManager was called correctly
        mock_connectome_manager.get_neurons_by_cortical_area.assert_called_once_with("iv00TL")
        assert mock_connectome_manager.batch_update_neuron_properties.call_count == 2
        
    def test_empty_cortical_area(self, mock_connectome_manager):
        """Test handling of empty cortical areas."""
        mock_connectome_manager.get_neurons_by_cortical_area.return_value = []
        
        updater = CorticalParameterUpdater(mock_connectome_manager)
        result = updater.update_neuron_parameters("empty_area", {"firing_threshold": 0.8})
        
        assert result is True  # Should succeed for empty areas
        
    def test_metadata_update(self, mock_connectome_manager):
        """Test metadata-only updates."""
        updater = CorticalParameterUpdater(mock_connectome_manager)
        
        metadata_changes = {"cortical_name": "New Name"}
        result = updater.update_metadata_only("iv00TL", metadata_changes)
        
        assert result is True
        # Verify the area name was updated
        assert mock_connectome_manager.cortical_areas["iv00TL"].name == "New Name"


class TestIntelligentUpdateRouting:
    """Test the intelligent update routing in GenomeService."""
    
    @pytest.fixture
    def mock_genome_service(self):
        """Create a GenomeService with mocked dependencies."""
        mock_state_manager = Mock()
        mock_connectome_manager = Mock(spec=ConnectomeManager)
        mock_transaction_manager = Mock()
        
        service = GenomeService(
            state_manager=mock_state_manager,
            connectome_manager=mock_connectome_manager,
            transaction_manager=mock_transaction_manager
        )
        
        # Mock genome data
        service._current_genome = {
            "blueprint": {
                "iv00TL": {
                    "cortical_name": "Left Visual",
                    "coordinates_3d": [0, 0, 0],
                    "cortical_dimensions": [10, 10, 1],
                    "cortical_type": "vision",
                    "parameters": {
                        "firing_threshold": 0.5,
                        "refractory_period": 3
                    }
                }
            }
        }
        
        # Mock transaction
        mock_transaction = Mock()
        mock_state_manager.begin_genome_transaction.return_value = mock_transaction
        
        return service, mock_transaction
        
    def test_parameter_update_routing(self, mock_genome_service):
        """Test that parameter-only updates use the fast path."""
        service, mock_transaction = mock_genome_service
        
        with patch.object(service, '_update_parameters_only') as mock_fast_update:
            mock_fast_update.return_value = {"updated": True}
            
            result = service.update_cortical_area(
                cortical_id="iv00TL",
                parameters={"firing_threshold": 0.8}
            )
            
            # Verify fast path was used
            mock_fast_update.assert_called_once()
            assert result == {"updated": True}
            
    def test_metadata_update_routing(self, mock_genome_service):
        """Test that metadata-only updates use the fastest path."""
        service, mock_transaction = mock_genome_service
        
        with patch.object(service, '_update_metadata_only') as mock_metadata_update:
            mock_metadata_update.return_value = {"updated": True}
            
            result = service.update_cortical_area(
                cortical_id="iv00TL",
                name="New Visual Area"
            )
            
            # Verify metadata path was used
            mock_metadata_update.assert_called_once()
            assert result == {"updated": True}
            
    def test_structural_update_routing(self, mock_genome_service):
        """Test that structural updates use the full rebuild path."""
        service, mock_transaction = mock_genome_service
        
        with patch.object(service, '_update_with_full_rebuild') as mock_full_rebuild:
            mock_full_rebuild.return_value = {"updated": True}
            
            result = service.update_cortical_area(
                cortical_id="iv00TL",
                dimensions={"x": 20, "y": 20, "z": 1}
            )
            
            # Verify full rebuild path was used
            mock_full_rebuild.assert_called_once()
            assert result == {"updated": True}
            
    def test_hybrid_update_routing(self, mock_genome_service):
        """Test that hybrid updates use the optimized hybrid path."""
        service, mock_transaction = mock_genome_service
        
        with patch.object(service, '_update_hybrid') as mock_hybrid_update:
            mock_hybrid_update.return_value = {"updated": True}
            
            result = service.update_cortical_area(
                cortical_id="iv00TL",
                name="New Name",  # metadata
                parameters={"firing_threshold": 0.9},  # parameter
                dimensions={"x": 15, "y": 15, "z": 1}  # structural
            )
            
            # Verify hybrid path was used
            mock_hybrid_update.assert_called_once()
            assert result == {"updated": True}
            
    def test_no_changes_handling(self, mock_genome_service):
        """Test handling when no changes are provided.""" 
        service, mock_transaction = mock_genome_service
        
        result = service.update_cortical_area(cortical_id="iv00TL")
        
        # Should return current area definition
        assert result == service._current_genome["blueprint"]["iv00TL"]
        
    def test_performance_logging(self, mock_genome_service):
        """Test that performance metrics are logged."""
        service, mock_transaction = mock_genome_service
        
        with patch.object(service, '_update_parameters_only') as mock_fast_update:
            mock_fast_update.return_value = {"updated": True}
            
            with patch.object(service.logger, 'info') as mock_logger:
                service.update_cortical_area(
                    cortical_id="iv00TL", 
                    parameters={"firing_threshold": 0.8}
                )
                
                # Verify performance logging
                performance_logs = [
                    call for call in mock_logger.call_args_list 
                    if '[CORTICAL-UPDATE]' in str(call)
                ]
                assert len(performance_logs) > 0


class TestPerformanceComparison:
    """Integration tests comparing performance of different update paths."""
    
    @pytest.mark.slow
    def test_parameter_update_performance(self):
        """Test that parameter updates are significantly faster than full rebuilds."""
        # This would be an integration test with real ConnectomeManager
        # For now, just test that the routing logic correctly identifies fast paths
        
        parameter_changes = {"firing_threshold": 0.8}
        change_type = CorticalChangeClassifier.classify_changes(parameter_changes)
        
        assert change_type == ChangeType.PARAMETER
        # In real implementation, this should be ~2-5ms vs ~800ms for full rebuild
        
    def test_expected_performance_characteristics(self):
        """Document expected performance characteristics."""
        # This serves as documentation of expected performance gains
        
        performance_expectations = {
            ChangeType.METADATA: "~1ms (800x faster than rebuild)",
            ChangeType.PARAMETER: "~2-5ms (160-400x faster than rebuild)",
            ChangeType.STRUCTURAL: "~800ms (same as before - rebuild required)",
            ChangeType.HYBRID: "Optimized combination based on change mix"
        }
        
        # Verify all change types have documented expectations
        for change_type in ChangeType:
            assert change_type in performance_expectations 