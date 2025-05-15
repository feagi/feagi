"""
Test module for CorticalMapping class.
"""

import pytest
from feagi.bdu.connectivity.cortical_mappings import CorticalMapping

@pytest.fixture
def cortical_mapping():
    """Create a basic cortical mapping for testing."""
    return CorticalMapping(
        mapping_id="test_mapping",
        name="Test Mapping",
        source_area_id="source_area",
        target_area_id="target_area",
        morphology_id="test_morphology",
        morphology_scalar=[1.0, 1.0, 1.0],
        plasticity_flag=True,
        psc_multiplier=1.5
    )

def test_cortical_mapping_init():
    """Test cortical mapping initialization."""
    mapping = CorticalMapping(
        mapping_id="test_mapping",
        name="Test Mapping",
        source_area_id="source_area",
        target_area_id="target_area",
        morphology_id="test_morphology",
        morphology_scalar=[1.0, 1.0, 1.0],
        plasticity_flag=True,
        psc_multiplier=1.5,
        properties={"synapse_delay": 2.0}
    )
    
    # Check properties
    assert mapping.id == "test_mapping"
    assert mapping.name == "Test Mapping"
    assert mapping.source_area_id == "source_area"
    assert mapping.target_area_id == "target_area"
    assert mapping.morphology_id == "test_morphology"
    assert mapping.morphology_scalar == [1.0, 1.0, 1.0]
    assert mapping.plasticity_flag == True
    assert mapping.psc_multiplier == 1.5
    assert mapping.properties["synapse_delay"] == 2.0

def test_cortical_mapping_to_dict(cortical_mapping):
    """Test conversion to dictionary."""
    mapping_dict = cortical_mapping.to_dict()
    
    # Check dictionary contents
    assert mapping_dict["id"] == "test_mapping"
    assert mapping_dict["name"] == "Test Mapping"
    assert mapping_dict["source_area_id"] == "source_area"
    assert mapping_dict["target_area_id"] == "target_area"
    assert mapping_dict["morphology_id"] == "test_morphology"
    assert mapping_dict["morphology_scalar"] == [1.0, 1.0, 1.0]
    assert mapping_dict["plasticity_flag"] == True
    assert mapping_dict["psc_multiplier"] == 1.5

def test_cortical_mapping_from_dict():
    """Test creating mapping from dictionary."""
    mapping_dict = {
        "id": "test_mapping_2",
        "name": "Test Mapping 2",
        "source_area_id": "area1",
        "target_area_id": "area2",
        "morphology_id": "morphology2",
        "morphology_scalar": [2.0, 2.0, 2.0],
        "plasticity_flag": False,
        "psc_multiplier": 0.8,
        "properties": {"synapse_type": "inhibitory"}
    }
    
    mapping = CorticalMapping.from_dict(mapping_dict)
    
    # Check properties
    assert mapping.id == "test_mapping_2"
    assert mapping.name == "Test Mapping 2"
    assert mapping.source_area_id == "area1"
    assert mapping.target_area_id == "area2"
    assert mapping.morphology_id == "morphology2"
    assert mapping.morphology_scalar == [2.0, 2.0, 2.0]
    assert mapping.plasticity_flag == False
    assert mapping.psc_multiplier == 0.8
    assert mapping.properties["synapse_type"] == "inhibitory"

def test_cortical_mapping_update(cortical_mapping):
    """Test updating mapping properties."""
    updates = {
        "name": "Updated Mapping",
        "morphology_id": "new_morphology",
        "plasticity_flag": False,
        "psc_multiplier": 2.0,
        "properties": {"update_flag": True}
    }
    
    cortical_mapping.update(updates)
    
    # Check updated properties
    assert cortical_mapping.name == "Updated Mapping"
    assert cortical_mapping.morphology_id == "new_morphology"
    assert cortical_mapping.plasticity_flag == False
    assert cortical_mapping.psc_multiplier == 2.0
    assert cortical_mapping.properties["update_flag"] == True
    
    # Check that non-updated properties remain the same
    assert cortical_mapping.id == "test_mapping"
    assert cortical_mapping.source_area_id == "source_area"
    assert cortical_mapping.target_area_id == "target_area"
    assert cortical_mapping.morphology_scalar == [1.0, 1.0, 1.0]

def test_cortical_mapping_update_invalid_property(cortical_mapping):
    """Test updating with an invalid property."""
    with pytest.raises(KeyError):
        cortical_mapping.update({"invalid_property": "value"})

def test_cortical_mapping_validate():
    """Test mapping validation."""
    # Valid mapping
    valid_mapping = CorticalMapping(
        mapping_id="test_mapping",
        name="Test Mapping",
        source_area_id="source_area",
        target_area_id="target_area",
        morphology_id="test_morphology",
        morphology_scalar=[1.0, 1.0, 1.0],
        plasticity_flag=True,
        psc_multiplier=1.5
    )
    assert valid_mapping.validate() == True
    
    # Invalid mapping: missing name
    invalid_mapping = CorticalMapping(
        mapping_id="test_mapping",
        name="",  # Empty name
        source_area_id="source_area",
        target_area_id="target_area",
        morphology_id="test_morphology",
        morphology_scalar=[1.0, 1.0, 1.0],
        plasticity_flag=True,
        psc_multiplier=1.5
    )
    assert invalid_mapping.validate() == False
    
    # Invalid mapping: missing source area
    invalid_mapping = CorticalMapping(
        mapping_id="test_mapping",
        name="Test Mapping",
        source_area_id="",  # Empty source area
        target_area_id="target_area",
        morphology_id="test_morphology",
        morphology_scalar=[1.0, 1.0, 1.0],
        plasticity_flag=True,
        psc_multiplier=1.5
    )
    assert invalid_mapping.validate() == False
    
    # Invalid mapping: missing target area
    invalid_mapping = CorticalMapping(
        mapping_id="test_mapping",
        name="Test Mapping",
        source_area_id="source_area",
        target_area_id="",  # Empty target area
        morphology_id="test_morphology",
        morphology_scalar=[1.0, 1.0, 1.0],
        plasticity_flag=True,
        psc_multiplier=1.5
    )
    assert invalid_mapping.validate() == False
    
    # Invalid mapping: missing morphology ID
    invalid_mapping = CorticalMapping(
        mapping_id="test_mapping",
        name="Test Mapping",
        source_area_id="source_area",
        target_area_id="target_area",
        morphology_id="",  # Empty morphology ID
        morphology_scalar=[1.0, 1.0, 1.0],
        plasticity_flag=True,
        psc_multiplier=1.5
    )
    assert invalid_mapping.validate() == False
    
    # Invalid mapping: invalid morphology scalar
    invalid_mapping = CorticalMapping(
        mapping_id="test_mapping",
        name="Test Mapping",
        source_area_id="source_area",
        target_area_id="target_area",
        morphology_id="test_morphology",
        morphology_scalar=[1.0, 1.0],  # Should have 3 values
        plasticity_flag=True,
        psc_multiplier=1.5
    )
    assert invalid_mapping.validate() == False

def test_cortical_mapping_morphology_scalar_validation():
    """Test validation of morphology scalar values."""
    # Test with valid scalar
    valid_mapping = CorticalMapping(
        mapping_id="test_mapping",
        name="Test Mapping",
        source_area_id="source_area",
        target_area_id="target_area",
        morphology_id="test_morphology",
        morphology_scalar=[1.0, 1.0, 1.0],  # Valid 3D scalar
        plasticity_flag=True,
        psc_multiplier=1.5
    )
    assert valid_mapping.validate() == True
    
    # Test with negative scalar (invalid)
    invalid_mapping = CorticalMapping(
        mapping_id="test_mapping",
        name="Test Mapping",
        source_area_id="source_area",
        target_area_id="target_area",
        morphology_id="test_morphology",
        morphology_scalar=[-1.0, 1.0, 1.0],  # Negative value
        plasticity_flag=True,
        psc_multiplier=1.5
    )
    assert invalid_mapping.validate() == False
    
    # Test with too many values
    invalid_mapping = CorticalMapping(
        mapping_id="test_mapping",
        name="Test Mapping",
        source_area_id="source_area",
        target_area_id="target_area",
        morphology_id="test_morphology",
        morphology_scalar=[1.0, 1.0, 1.0, 1.0],  # 4 values instead of 3
        plasticity_flag=True,
        psc_multiplier=1.5
    )
    assert invalid_mapping.validate() == False 