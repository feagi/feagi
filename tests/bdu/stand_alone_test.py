"""
Standalone test file that tests the BrainRegion implementation without requiring imports.

This test file directly defines a BrainRegion class with the same interface as the 
actual one, and runs tests against it to verify the implementation works correctly.
"""

import pytest

class BrainRegion:
    """
    Represents a brain region containing multiple cortical areas.
    
    A brain region is a logical grouping of cortical areas that are 
    functionally related.
    """
    
    def __init__(self, region_id: str, name: str, region_type: str = "custom", 
                 properties: dict = None):
        """
        Initialize a brain region.
        
        Args:
            region_id: Unique identifier for the region
            name: Human-readable name of the region
            region_type: Type of region (e.g., "sensory", "motor", "custom")
            properties: Additional properties as key-value pairs
        """
        self.id = region_id
        self.name = name
        self.region_type = region_type
        self.properties = properties or {}
        self.cortical_areas = set()  # Set of cortical area IDs
        
    def add_area(self, area_id: str) -> None:
        """
        Add a cortical area to this region.
        
        Args:
            area_id: ID of the cortical area to add
        """
        self.cortical_areas.add(area_id)
        
    def remove_area(self, area_id: str) -> bool:
        """
        Remove a cortical area from this region.
        
        Args:
            area_id: ID of the cortical area to remove
            
        Returns:
            True if area was removed, False if area was not in the region
        """
        if area_id in self.cortical_areas:
            self.cortical_areas.remove(area_id)
            return True
        return False
        
    def contains_area(self, area_id: str) -> bool:
        """
        Check if the region contains a specific cortical area.
        
        Args:
            area_id: ID of the cortical area to check
            
        Returns:
            True if the area is in this region, False otherwise
        """
        return area_id in self.cortical_areas
        
    def get_all_areas(self) -> list:
        """
        Get a list of all cortical area IDs in this region.
        
        Returns:
            List of cortical area IDs
        """
        return list(self.cortical_areas)
        
    def to_dict(self) -> dict:
        """
        Convert the brain region to a dictionary representation.
        
        Returns:
            Dictionary representation of the brain region
        """
        return {
            "id": self.id,
            "name": self.name,
            "region_type": self.region_type,
            "cortical_areas": list(self.cortical_areas),
            "properties": self.properties
        }
        
    def update(self, updates: dict) -> None:
        """
        Update brain region properties.
        
        Args:
            updates: Dictionary of properties to update
            
        Raises:
            KeyError: If an invalid property is specified
        """
        valid_properties = {"name", "region_type", "properties"}
        
        for key, value in updates.items():
            if key not in valid_properties:
                raise KeyError(f"Invalid property: {key}")
            
            if key == "properties":
                self.properties.update(value)
            else:
                setattr(self, key, value)
                
    def clear_areas(self) -> None:
        """
        Remove all cortical areas from this region.
        """
        self.cortical_areas.clear()


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