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
Test module for CorticalArea class.
"""

import pytest
from feagi.bdu.models.cortical_area import CorticalArea

@pytest.fixture
def cortical_area():
    """Create a simple cortical area for testing."""
    return CorticalArea(
        cortical_id="test_area",
        name="Test Area",
        dimensions=(10, 10, 5),
        position=(0, 0, 0),
        area_type="custom"
    )

def test_cortical_area_init():
    """Test cortical area initialization."""
    area = CorticalArea(
        cortical_id="test_area",
        name="Test Area",
        dimensions=(10, 10, 5),
        position=(0, 0, 0),
        area_type="custom",
        properties={"test_prop": "test_value"}
    )
    
    # Check basic properties
    assert area.id == "test_area"
    assert area.name == "Test Area"
    assert area.dimensions == (10, 10, 5)
    assert area.position == (0, 0, 0)
    assert area.area_type == "custom"
    assert area.properties["test_prop"] == "test_value"
    
    # Check derived properties
    assert area.volume == 10 * 10 * 5 == 500

def test_contains_position(cortical_area):
    """Test position validation within cortical area."""
    area = cortical_area
    
    # Valid positions
    assert area.contains_position((0, 0, 0)) == True  # Origin
    assert area.contains_position((5, 5, 2)) == True  # Middle
    assert area.contains_position((9, 9, 4)) == True  # Far corner
    
    # Invalid positions
    assert area.contains_position((-1, 0, 0)) == False  # Negative x
    assert area.contains_position((0, -1, 0)) == False  # Negative y
    assert area.contains_position((0, 0, -1)) == False  # Negative z
    assert area.contains_position((10, 0, 0)) == False  # x too large
    assert area.contains_position((0, 10, 0)) == False  # y too large
    assert area.contains_position((0, 0, 5)) == False   # z too large

def test_add_neuron(cortical_area):
    """Test adding neurons to the cortical area."""
    area = cortical_area
    
    # Add neurons
    area.add_neuron(1, (0, 0, 0))
    area.add_neuron(2, (1, 1, 1))
    area.add_neuron(3, (2, 2, 2))
    
    # Check neuron counts
    assert area.neuron_count == 3
    
    # Check neuron tracking
    assert 1 in area.neurons
    assert 2 in area.neurons
    assert 3 in area.neurons
    
    # Check position mapping
    assert area.get_neurons_at_position((0, 0, 0)) == [1]
    assert area.get_neurons_at_position((1, 1, 1)) == [2]
    assert area.get_neurons_at_position((2, 2, 2)) == [3]
    
    # Check empty position
    assert area.get_neurons_at_position((3, 3, 3)) == []

def test_multiple_neurons_at_position(cortical_area):
    """Test adding multiple neurons at the same position."""
    area = cortical_area
    
    # Add multiple neurons at the same position
    area.add_neuron(1, (0, 0, 0))
    area.add_neuron(2, (0, 0, 0))
    area.add_neuron(3, (0, 0, 0))
    
    # Check neuron counts
    assert area.neuron_count == 3
    
    # Check position mapping (should return all neurons at this position)
    assert sorted(area.get_neurons_at_position((0, 0, 0))) == [1, 2, 3]

def test_remove_neuron(cortical_area):
    """Test removing neurons from the cortical area."""
    area = cortical_area
    
    # Add neurons
    area.add_neuron(1, (0, 0, 0))
    area.add_neuron(2, (1, 1, 1))
    area.add_neuron(3, (0, 0, 0))  # Same position as neuron 1
    
    # Verify initial state
    assert area.neuron_count == 3
    assert sorted(area.get_neurons_at_position((0, 0, 0))) == [1, 3]
    
    # Remove a neuron
    area.remove_neuron(1)
    
    # Check updated state
    assert area.neuron_count == 2
    assert 1 not in area.neurons
    assert sorted(area.get_neurons_at_position((0, 0, 0))) == [3]
    
    # Remove another neuron
    area.remove_neuron(3)
    
    # Check that the position is now empty
    assert area.get_neurons_at_position((0, 0, 0)) == []
    
    # Try to remove a non-existent neuron
    area.remove_neuron(999)  # Should not raise an error, just do nothing
    assert area.neuron_count == 1

def test_get_all_neurons(cortical_area):
    """Test getting all neurons in the cortical area."""
    area = cortical_area
    
    # Add neurons
    area.add_neuron(1, (0, 0, 0))
    area.add_neuron(2, (1, 1, 1))
    area.add_neuron(3, (2, 2, 2))
    
    # Get all neurons
    all_neurons = area.get_all_neurons()
    
    # Check result
    assert sorted(all_neurons) == [1, 2, 3]

def test_to_dict(cortical_area):
    """Test conversion to dictionary."""
    area = cortical_area
    
    # Add some test data
    area.add_neuron(1, (0, 0, 0))
    area.add_neuron(2, (1, 1, 1))
    
    # Get dictionary representation
    area_dict = area.to_dict()
    
    # Check dictionary contents
    assert area_dict["id"] == "test_area"
    assert area_dict["name"] == "Test Area"
    assert area_dict["dimensions"] == (10, 10, 5)
    assert area_dict["position"] == (0, 0, 0)
    assert area_dict["area_type"] == "custom"
    assert area_dict["neuron_count"] == 2

def test_update_properties(cortical_area):
    """Test updating cortical area properties."""
    area = cortical_area
    
    # Initial properties
    area.properties = {"key1": "value1"}
    
    # Update properties
    updates = {
        "name": "Updated Area",
        "position": (5, 5, 5),
        "properties": {"key1": "new_value", "key2": "value2"}
    }
    
    area.update(updates)
    
    # Check updates
    assert area.name == "Updated Area"
    assert area.position == (5, 5, 5)
    assert area.properties["key1"] == "new_value"
    assert area.properties["key2"] == "value2"
    
    # Try updating an invalid property
    with pytest.raises(KeyError):
        area.update({"invalid_property": "value"}) 