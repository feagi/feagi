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
Test module for CorticalMapping class.
"""

import pytest

from feagi.bdu.connectivity.cortical_mappings import CorticalMapping


@pytest.fixture
def cortical_mapping():
    """Create a basic cortical mapping for testing."""
    return CorticalMapping(
        source_cortical_id="source_area",
        target_cortical_id="target_area",
        mapping_type="topological",
        parameters={
            "morphology_id": "test_morphology",
            "morphology_scalar": [1.0, 1.0, 1.0],
            "plasticity_flag": True,
            "psc_multiplier": 1.5,
        },
        name="Test Mapping",
        mapping_id="test_mapping",
    )


def test_cortical_mapping_init():
    """Test cortical mapping initialization."""
    mapping = CorticalMapping(
        source_cortical_id="source_area",
        target_cortical_id="target_area",
        mapping_type="topological",
        parameters={
            "morphology_id": "test_morphology",
            "morphology_scalar": [1.0, 1.0, 1.0],
            "plasticity_flag": True,
            "psc_multiplier": 1.5,
            "synapse_delay": 2.0,
        },
        name="Test Mapping",
        mapping_id="test_mapping",
    )

    # Check properties
    assert mapping.id == "test_mapping"
    assert mapping.name == "Test Mapping"
    assert mapping.source_cortical_id == "source_area"
    assert mapping.target_cortical_id == "target_area"
    assert mapping.parameters["morphology_id"] == "test_morphology"
    assert mapping.parameters["morphology_scalar"] == [1.0, 1.0, 1.0]
    assert mapping.parameters["plasticity_flag"]
    assert mapping.parameters["psc_multiplier"] == 1.5
    assert mapping.parameters["synapse_delay"] == 2.0


def test_cortical_mapping_to_dict(cortical_mapping):
    """Test conversion to dictionary."""
    mapping_dict = cortical_mapping.to_dict()

    # Check dictionary contents
    assert mapping_dict["id"] == "test_mapping"
    assert mapping_dict["name"] == "Test Mapping"
    assert mapping_dict["source_cortical_id"] == "source_area"
    assert mapping_dict["target_cortical_id"] == "target_area"
    assert mapping_dict["mapping_type"] == "topological"
    assert mapping_dict["parameters"]["morphology_id"] == "test_morphology"
    assert mapping_dict["parameters"]["morphology_scalar"] == [1.0, 1.0, 1.0]
    assert mapping_dict["parameters"]["plasticity_flag"]
    assert mapping_dict["parameters"]["psc_multiplier"] == 1.5


def test_cortical_mapping_from_dict():
    """Test creating mapping from dictionary."""
    mapping_dict = {
        "id": "test_mapping_2",
        "name": "Test Mapping 2",
        "source_cortical_id": "area1",
        "target_cortical_id": "area2",
        "mapping_type": "topological",
        "parameters": {
            "morphology_id": "morphology2",
            "morphology_scalar": [2.0, 2.0, 2.0],
            "plasticity_flag": False,
            "psc_multiplier": 0.8,
            "synapse_type": "inhibitory",
        },
    }

    mapping = CorticalMapping.from_dict(mapping_dict)

    # Check properties
    assert mapping.id == "test_mapping_2"
    assert mapping.name == "Test Mapping 2"
    assert mapping.source_cortical_id == "area1"
    assert mapping.target_cortical_id == "area2"
    assert mapping.mapping_type == "topological"
    assert mapping.parameters["morphology_id"] == "morphology2"
    assert mapping.parameters["morphology_scalar"] == [2.0, 2.0, 2.0]
    assert not mapping.parameters["plasticity_flag"]
    assert mapping.parameters["psc_multiplier"] == 0.8
    assert mapping.parameters["synapse_type"] == "inhibitory"


def test_cortical_mapping_update(cortical_mapping):
    """Test updating mapping properties."""
    updates = {
        "name": "Updated Mapping",
        "parameters": {
            "morphology_id": "new_morphology",
            "plasticity_flag": False,
            "psc_multiplier": 2.0,
            "update_flag": True,
        },
    }

    cortical_mapping.update(updates)

    # Check updated properties
    assert cortical_mapping.name == "Updated Mapping"
    assert cortical_mapping.parameters["morphology_id"] == "new_morphology"
    assert not cortical_mapping.parameters["plasticity_flag"]
    assert cortical_mapping.parameters["psc_multiplier"] == 2.0
    assert cortical_mapping.parameters["update_flag"]

    # Check that non-updated properties remain the same
    assert cortical_mapping.id == "test_mapping"
    assert cortical_mapping.source_cortical_id == "source_area"
    assert cortical_mapping.target_cortical_id == "target_area"
    assert cortical_mapping.parameters["morphology_scalar"] == [1.0, 1.0, 1.0]


def test_cortical_mapping_update_invalid_property(cortical_mapping):
    """Test updating with an invalid property."""
    with pytest.raises(KeyError):
        cortical_mapping.update({"invalid_property": "value"})


def test_cortical_mapping_validate():
    """Test mapping validation."""
    # Valid mapping
    valid_mapping = CorticalMapping(
        source_cortical_id="source_area",
        target_cortical_id="target_area",
        mapping_type="topological",
        parameters={
            "morphology_id": "test_morphology",
            "morphology_scalar": [1.0, 1.0, 1.0],
            "plasticity_flag": True,
            "psc_multiplier": 1.5,
        },
        name="Test Mapping",
        mapping_id="test_mapping",
    )
    assert valid_mapping.validate()

    # Invalid mapping: missing name
    invalid_mapping = CorticalMapping(
        source_cortical_id="source_area",
        target_cortical_id="target_area",
        mapping_type="topological",
        parameters={
            "morphology_id": "test_morphology",
            "morphology_scalar": [1.0, 1.0, 1.0],
            "plasticity_flag": True,
            "psc_multiplier": 1.5,
        },
        name="",  # Empty name
        mapping_id="test_mapping",
    )
    assert not invalid_mapping.validate()

    # Invalid mapping: missing source area
    invalid_mapping = CorticalMapping(
        source_cortical_id="",  # Empty source area
        target_cortical_id="target_area",
        mapping_type="topological",
        parameters={
            "morphology_id": "test_morphology",
            "morphology_scalar": [1.0, 1.0, 1.0],
            "plasticity_flag": True,
            "psc_multiplier": 1.5,
        },
        name="Test Mapping",
        mapping_id="test_mapping",
    )
    assert not invalid_mapping.validate()

    # Invalid mapping: missing target area
    invalid_mapping = CorticalMapping(
        source_cortical_id="source_area",
        target_cortical_id="",  # Empty target area
        mapping_type="topological",
        parameters={
            "morphology_id": "test_morphology",
            "morphology_scalar": [1.0, 1.0, 1.0],
            "plasticity_flag": True,
            "psc_multiplier": 1.5,
        },
        name="Test Mapping",
        mapping_id="test_mapping",
    )
    assert not invalid_mapping.validate()

    # Invalid mapping: missing mapping type
    invalid_mapping = CorticalMapping(
        source_cortical_id="source_area",
        target_cortical_id="target_area",
        mapping_type="",  # Empty mapping type
        parameters={
            "morphology_id": "test_morphology",
            "morphology_scalar": [1.0, 1.0, 1.0],
            "plasticity_flag": True,
            "psc_multiplier": 1.5,
        },
        name="Test Mapping",
        mapping_id="test_mapping",
    )
    assert not invalid_mapping.validate()


def test_cortical_mapping_morphology_scalar_validation():
    """Test validation of morphology scalar values."""
    # Test with valid scalar
    valid_mapping = CorticalMapping(
        source_cortical_id="source_area",
        target_cortical_id="target_area",
        mapping_type="topological",
        parameters={
            "morphology_id": "test_morphology",
            "morphology_scalar": [1.0, 1.0, 1.0],  # Valid 3D scalar
            "plasticity_flag": True,
            "psc_multiplier": 1.5,
        },
        name="Test Mapping",
        mapping_id="test_mapping",
    )
    assert valid_mapping.validate()

    # Note: The current validation method doesn't check morphology scalar details
    # These tests would need to be updated if more detailed validation is added
    # For now, we just test that the mapping can be created successfully
