"""
Integration tests for morphology dimension_sensitive feature.

Tests genome migration, API integration, and auto-detection logic
for the new dimension_sensitive morphology field.
"""

import pytest
from unittest.mock import Mock, patch
from feagi.evo.genome_validator import add_missing_dimension_sensitive_fields, morphology_validator
from feagi.api.v1.schemas import CreateMorphologyRequest, DirectMorphologyRequest
from feagi.api.v1.morphology import MorphologyAPI


class TestDimensionSensitiveMigration:
    """Test automatic migration of dimension_sensitive fields."""
    
    def test_add_missing_dimension_sensitive_fields_patterns(self):
        """Test auto-migration for patterns morphology type."""
        genome = {
            "neuron_morphologies": {
                "lateral_pattern": {
                    "type": "patterns",
                    "parameters": {"patterns": [[[0, 1, 0], [1, 0, 0]]]}
                }
            }
        }
        
        added_count = add_missing_dimension_sensitive_fields(genome)
        
        assert added_count == 1
        assert genome["neuron_morphologies"]["lateral_pattern"]["dimension_sensitive"] is False
    
    def test_add_missing_dimension_sensitive_fields_vectors(self):
        """Test auto-migration for vectors morphology type."""
        genome = {
            "neuron_morphologies": {
                "vector_connection": {
                    "type": "vectors", 
                    "parameters": {"vectors": [[1, 0, 0], [0, 1, 0]]}
                }
            }
        }
        
        added_count = add_missing_dimension_sensitive_fields(genome)
        
        assert added_count == 1
        assert genome["neuron_morphologies"]["vector_connection"]["dimension_sensitive"] is False
    
    def test_add_missing_dimension_sensitive_fields_functions(self):
        """Test auto-migration for functions morphology type."""
        genome = {
            "neuron_morphologies": {
                "projector_func": {
                    "type": "functions",
                    "parameters": {"projection_type": "grid"}
                }
            }
        }
        
        added_count = add_missing_dimension_sensitive_fields(genome)
        
        assert added_count == 1
        assert genome["neuron_morphologies"]["projector_func"]["dimension_sensitive"] is True
    
    def test_add_missing_dimension_sensitive_fields_composite_default(self):
        """Test auto-migration for composite morphology type (conservative default)."""
        genome = {
            "neuron_morphologies": {
                "complex_morph": {
                    "type": "composite",
                    "parameters": {"src_seed": "test", "src_pattern": "test", "mapper_morphology": "test"}
                }
            }
        }
        
        added_count = add_missing_dimension_sensitive_fields(genome)
        
        assert added_count == 1
        assert genome["neuron_morphologies"]["complex_morph"]["dimension_sensitive"] is False
    
    def test_no_migration_when_field_exists(self):
        """Test that existing dimension_sensitive fields are not modified."""
        genome = {
            "neuron_morphologies": {
                "existing_morph": {
                    "type": "patterns",
                    "parameters": {"patterns": [[[0, 1, 0], [1, 0, 0]]]},
                    "dimension_sensitive": True  # Explicitly set to True
                }
            }
        }
        
        added_count = add_missing_dimension_sensitive_fields(genome)
        
        assert added_count == 0
        assert genome["neuron_morphologies"]["existing_morph"]["dimension_sensitive"] is True
    
    def test_migration_mixed_morphologies(self):
        """Test migration with mix of existing and missing fields."""
        genome = {
            "neuron_morphologies": {
                "needs_migration": {
                    "type": "patterns",
                    "parameters": {"patterns": [[[0, 1, 0], [1, 0, 0]]]}
                },
                "has_field": {
                    "type": "functions",
                    "parameters": {"projection_type": "grid"},
                    "dimension_sensitive": False  # Explicitly set
                }
            }
        }
        
        added_count = add_missing_dimension_sensitive_fields(genome)
        
        assert added_count == 1
        assert genome["neuron_morphologies"]["needs_migration"]["dimension_sensitive"] is False
        assert genome["neuron_morphologies"]["has_field"]["dimension_sensitive"] is False


class TestMorphologyAPIIntegration:
    """Test API integration for dimension_sensitive parameter."""
    
    def test_auto_detect_dimension_sensitive_patterns(self):
        """Test auto-detection logic for patterns type."""
        core_api_service = Mock()
        api = MorphologyAPI(core_api_service)
        
        result = api._auto_detect_dimension_sensitive("patterns")
        assert result is False
    
    def test_auto_detect_dimension_sensitive_vectors(self):
        """Test auto-detection logic for vectors type."""
        core_api_service = Mock()
        api = MorphologyAPI(core_api_service)
        
        result = api._auto_detect_dimension_sensitive("vectors")
        assert result is False
    
    def test_auto_detect_dimension_sensitive_functions(self):
        """Test auto-detection logic for functions type."""
        core_api_service = Mock()
        api = MorphologyAPI(core_api_service)
        
        result = api._auto_detect_dimension_sensitive("functions")
        assert result is True
    
    def test_auto_detect_dimension_sensitive_unknown(self):
        """Test auto-detection logic for unknown type (conservative default)."""
        core_api_service = Mock()
        api = MorphologyAPI(core_api_service)
        
        result = api._auto_detect_dimension_sensitive("unknown_type")
        assert result is False


class TestSchemaValidation:
    """Test Pydantic schema validation for dimension_sensitive parameter."""
    
    def test_create_morphology_request_with_dimension_sensitive(self):
        """Test CreateMorphologyRequest with explicit dimension_sensitive."""
        request_data = {
            "morphology_data": {
                "name": "test_morph",
                "type": "patterns",
                "parameters": {"patterns": [[[0, 1, 0], [1, 0, 0]]]}
            },
            "dimension_sensitive": True
        }
        
        request = CreateMorphologyRequest(**request_data)
        
        assert request.dimension_sensitive is True
        assert request.morphology_data["name"] == "test_morph"
    
    def test_create_morphology_request_without_dimension_sensitive(self):
        """Test CreateMorphologyRequest without dimension_sensitive (should be None).""" 
        request_data = {
            "morphology_data": {
                "name": "test_morph",
                "type": "patterns", 
                "parameters": {"patterns": [[[0, 1, 0], [1, 0, 0]]]}
            }
        }
        
        request = CreateMorphologyRequest(**request_data)
        
        assert request.dimension_sensitive is None
    
    def test_direct_morphology_request_with_dimension_sensitive(self):
        """Test DirectMorphologyRequest with explicit dimension_sensitive."""
        request_data = {
            "morphology_name": "test_direct",
            "morphology_type": "functions",
            "morphology_parameters": {"projection_type": "grid"},
            "dimension_sensitive": True
        }
        
        request = DirectMorphologyRequest(**request_data)
        
        assert request.dimension_sensitive is True
        assert request.morphology_name == "test_direct"
    
    def test_direct_morphology_request_without_dimension_sensitive(self):
        """Test DirectMorphologyRequest without dimension_sensitive (should be None)."""
        request_data = {
            "morphology_name": "test_direct",
            "morphology_type": "functions", 
            "morphology_parameters": {"projection_type": "grid"}
        }
        
        request = DirectMorphologyRequest(**request_data)
        
        assert request.dimension_sensitive is None


if __name__ == "__main__":
    pytest.main([__file__, "-v"]) 