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
Test module for BrainRegion class using direct imports.
"""

import pytest
import sys
import os

# Add the project root to sys.path if needed
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# Direct import of the BrainRegion class without going through __init__.py
from feagi.bdu.models.brain_region import BrainRegion

@pytest.fixture
def brain_region():
    """Create a simple brain region for testing."""
    return BrainRegion(
        region_id="test_region",
        name="Test Region",
        region_type="custom"
    )

def test_brain_region_init():
    """Test brain region initialization."""
    region = BrainRegion(
        region_id="test_region",
        name="Test Region",
        region_type="custom",
        properties={"test_prop": "test_value"}
    )
    
    # Check basic properties
    assert region.id == "test_region"
    assert region.name == "Test Region"
    assert region.region_type == "custom"
    assert region.properties["test_prop"] == "test_value"
    
    # Check derived properties
    assert len(region.cortical_areas) == 0

def test_add_area(brain_region):
    """Test adding cortical areas to the brain region."""
    region = brain_region
    
    # Add areas
    region.add_area("area1")
    region.add_area("area2")
    region.add_area("area3")
    
    # Check area counts
    assert len(region.cortical_areas) == 3
    
    # Check area tracking
    assert "area1" in region.cortical_areas
    assert "area2" in region.cortical_areas
    assert "area3" in region.cortical_areas
    
    # Try adding a duplicate area (shouldn't change the set)
    region.add_area("area1")
    assert len(region.cortical_areas) == 3

def test_remove_area(brain_region):
    """Test removing cortical areas from the brain region."""
    region = brain_region
    
    # Add areas
    region.add_area("area1")
    region.add_area("area2")
    region.add_area("area3")
    
    # Verify initial state
    assert len(region.cortical_areas) == 3
    
    # Remove an area
    result = region.remove_area("area1")
    
    # Check updated state
    assert result == True
    assert len(region.cortical_areas) == 2
    assert "area1" not in region.cortical_areas
    assert "area2" in region.cortical_areas
    assert "area3" in region.cortical_areas
    
    # Try to remove a non-existent area
    result = region.remove_area("nonexistent_area")
    assert result == False
    assert len(region.cortical_areas) == 2

def test_contains_area(brain_region):
    """Test checking if a brain region contains a cortical area."""
    region = brain_region
    
    # Add areas
    region.add_area("area1")
    region.add_area("area2")
    
    # Check containment
    assert region.contains_area("area1") == True
    assert region.contains_area("area2") == True
    assert region.contains_area("area3") == False
    assert region.contains_area("") == False

def test_get_all_areas(brain_region):
    """Test getting all cortical areas in the brain region."""
    region = brain_region
    
    # Add areas
    region.add_area("area1")
    region.add_area("area2")
    region.add_area("area3")
    
    # Get all areas
    all_areas = region.get_all_areas()
    
    # Check result
    assert isinstance(all_areas, list)
    assert sorted(all_areas) == ["area1", "area2", "area3"]

def test_to_dict(brain_region):
    """Test conversion to dictionary."""
    region = brain_region
    
    # Add some test data
    region.add_area("area1")
    region.add_area("area2")
    
    # Get dictionary representation
    region_dict = region.to_dict()
    
    # Check dictionary contents
    assert region_dict["id"] == "test_region"
    assert region_dict["name"] == "Test Region"
    assert region_dict["region_type"] == "custom"
    assert sorted(region_dict["cortical_areas"]) == ["area1", "area2"]

def test_update_properties(brain_region):
    """Test updating brain region properties."""
    region = brain_region
    
    # Initial properties
    region.properties = {"key1": "value1"}
    
    # Update properties
    updates = {
        "name": "Updated Region",
        "region_type": "new_type",
        "properties": {"key1": "new_value", "key2": "value2"}
    }
    
    region.update(updates)
    
    # Check updates
    assert region.name == "Updated Region"
    assert region.region_type == "new_type"
    assert region.properties["key1"] == "new_value"
    assert region.properties["key2"] == "value2"
    
    # Try updating an invalid property
    with pytest.raises(KeyError):
        region.update({"invalid_property": "value"})

def test_clear_areas(brain_region):
    """Test clearing all cortical areas from the brain region."""
    region = brain_region
    
    # Add areas
    region.add_area("area1")
    region.add_area("area2")
    region.add_area("area3")
    
    # Verify initial state
    assert len(region.cortical_areas) == 3
    
    # Clear areas
    region.clear_areas()
    
    # Check updated state
    assert len(region.cortical_areas) == 0 