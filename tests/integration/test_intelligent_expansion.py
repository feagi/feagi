"""
Integration test for intelligent cortical area expansion with pattern extension.

This test validates that when a cortical area with internal connectivity is expanded,
the existing synaptic patterns are properly extended to the newly created neurons.
"""

import pytest
from unittest.mock import Mock, patch
from feagi.api.core.services.expansion.pattern_extender import PatternExtender
from feagi.api.core.services.expansion.connection_analyzer import ConnectionAnalyzer


class TestIntelligentExpansion:
    """Test intelligent pattern extension during cortical area expansion."""
    
    def test_pattern_extension_for_lateral_connectivity(self):
        """Test that lateral connectivity patterns are extended to new neurons during expansion."""
        # Mock connectome manager and state manager
        connectome_manager = Mock()
        state_manager = Mock()
        
        # Mock cortical area with existing neurons
        mock_area = Mock()
        mock_area._neuron_indices = {1, 2, 3, 4, 5, 6, 7, 8, 9}  # Original 9 neurons
        connectome_manager.cortical_areas = {"CP2asd": mock_area}
        
        # Mock genome with lateral connectivity mapping
        mock_genome = {
            "cortical_mappings": [
                {
                    "source": "CP2asd",
                    "destination": "CP2asd", 
                    "morphology": "lateral_+x",
                    "morphology_parameters": {}
                }
            ],
            "neuron_morphologies": {
                "lateral_+x": {
                    "type": "vectors",
                    "parameters": {"vectors": [[1, 0, 0]]},
                    "dimension_sensitive": False  # Dimension-agnostic
                }
            }
        }
        state_manager.genome = mock_genome
        
        # Mock synapse manager
        connectome_manager.synapse_manager.get_all_synapses.return_value = ["synapse1", "synapse2"]  # Initial synapses
        
        # Test pattern extender
        extender = PatternExtender(connectome_manager, state_manager)
        
        # Simulate new neurons from expansion (neurons 10, 11, 12)
        new_neurons = {10, 11, 12}
        
        # Test pattern extension
        result = extender.extend_patterns_for_expansion(
            cortical_id="CP2asd",
            old_dimensions=(3, 3, 1),
            new_dimensions=(4, 3, 1), 
            new_neurons=new_neurons
        )
        
        # Verify that the pattern extension was attempted
        assert isinstance(result, int)  # Should return number of synapses created
    
    def test_connection_analyzer_identifies_internal_mappings(self):
        """Test that ConnectionAnalyzer correctly identifies internal connectivity."""
        # Mock components
        connectome_manager = Mock()
        state_manager = Mock()
        
        # Mock genome with internal mapping
        mock_genome = {
            "cortical_mappings": [
                {
                    "source": "CP2asd",
                    "destination": "CP2asd",  # Internal mapping
                    "morphology": "lateral_+x"
                },
                {
                    "source": "CP2asd", 
                    "destination": "other_area",  # Outgoing mapping
                    "morphology": "projector"
                },
                {
                    "source": "other_area",
                    "destination": "CP2asd",  # Incoming mapping
                    "morphology": "random"
                }
            ],
            "neuron_morphologies": {
                "lateral_+x": {"dimension_sensitive": False},
                "projector": {"dimension_sensitive": True},
                "random": {"dimension_sensitive": False}
            }
        }
        state_manager.genome = mock_genome
        
        # Test analyzer
        analyzer = ConnectionAnalyzer(connectome_manager, state_manager)
        analysis = analyzer.analyze_area_connectivity("CP2asd")
        
        # Verify analysis results
        assert analysis["cortical_id"] == "CP2asd"
        assert analysis["total_mappings"] == 3
        assert analysis["internal_count"] == 1
        assert analysis["incoming_count"] == 1
        assert analysis["outgoing_count"] == 1
        assert analysis["dimension_sensitive_count"] == 1  # projector
        assert analysis["dimension_agnostic_count"] == 2   # lateral_+x and random
    
    def test_expansion_recommendation_logic(self):
        """Test that expansion recommendations are generated correctly."""
        # Mock components  
        connectome_manager = Mock()
        state_manager = Mock()
        
        # Mock genome with mixed sensitivity mappings
        mock_genome = {
            "cortical_mappings": [
                {
                    "source": "test_area",
                    "destination": "test_area",
                    "morphology": "lateral_pattern"
                }
            ],
            "neuron_morphologies": {
                "lateral_pattern": {"dimension_sensitive": False}
            }
        }
        state_manager.genome = mock_genome
        
        # Test analyzer
        analyzer = ConnectionAnalyzer(connectome_manager, state_manager)
        recommendation = analyzer.get_expansion_recommendation("test_area")
        
        # Verify recommendations
        assert recommendation["cortical_id"] == "test_area"
        assert recommendation["should_preserve_patterns"] is True   # Has dimension-agnostic mappings
        assert recommendation["should_reconstruct_patterns"] is False  # No dimension-sensitive mappings
        assert recommendation["has_internal_connectivity"] is True
        assert len(recommendation["preservation_candidates"]) == 1
        assert len(recommendation["reconstruction_candidates"]) == 0
    
    def test_dimension_sensitive_vs_agnostic_classification(self):
        """Test that morphologies are correctly classified by dimension sensitivity."""
        # Mock components
        connectome_manager = Mock()
        state_manager = Mock()
        
        # Mock genome with both types
        mock_genome = {
            "cortical_mappings": [
                {
                    "source": "area1",
                    "destination": "area2", 
                    "morphology": "projector_func"  # dimension-sensitive
                },
                {
                    "source": "area1",
                    "destination": "area1",
                    "morphology": "lateral_vector"  # dimension-agnostic
                }
            ],
            "neuron_morphologies": {
                "projector_func": {
                    "type": "functions",
                    "dimension_sensitive": True
                },
                "lateral_vector": {
                    "type": "vectors", 
                    "dimension_sensitive": False
                }
            }
        }
        state_manager.genome = mock_genome
        
        # Test analyzer
        analyzer = ConnectionAnalyzer(connectome_manager, state_manager)
        analysis = analyzer.analyze_area_connectivity("area1")
        
        # Verify classification
        assert analysis["dimension_sensitive_count"] == 1
        assert analysis["dimension_agnostic_count"] == 1
        
        # Check specific mappings
        sensitive_mapping = analysis["dimension_sensitive_mappings"][0]
        agnostic_mapping = analysis["dimension_agnostic_mappings"][0]
        
        assert sensitive_mapping["morphology"] == "projector_func"
        assert agnostic_mapping["morphology"] == "lateral_vector"


if __name__ == "__main__":
    pytest.main([__file__, "-v"]) 