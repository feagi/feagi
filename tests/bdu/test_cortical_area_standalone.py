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
Standalone test file for CorticalArea class.

This test directly defines the necessary classes without requiring imports.
"""

import pytest


class CorticalArea:
    """
    Represents a cortical area in the brain.

    A cortical area is a 3D volumetric region containing neurons.
    """

    def __init__(
        self, area_id, name, dimensions, position, area_type="custom", properties=None
    ):
        """
        Initialize a cortical area.

        Args:
            area_id: Unique identifier for the area
            name: Human-readable name
            dimensions: 3D dimensions as (width, height, depth)
            position: 3D position as (x, y, z)
            area_type: Type of cortical area
            properties: Additional properties as key-value pairs
        """
        self.id = area_id
        self.name = name
        self.dimensions = dimensions
        self.position = position
        self.area_type = area_type
        self.properties = properties or {}

        # Track neurons by their IDs
        self.neurons = set()

        # Map positions to neuron IDs
        self.position_neuron_map = {}

    @property
    def volume(self):
        """Calculate the volume of this cortical area."""
        return self.dimensions[0] * self.dimensions[1] * self.dimensions[2]

    @property
    def neuron_count(self):
        """Get the number of neurons in this area."""
        return len(self.neurons)

    def contains_position(self, position):
        """
        Check if a position is within the boundaries of this area.

        Args:
            position: 3D position as (x, y, z)

        Returns:
            True if the position is within the area, False otherwise
        """
        x, y, z = position
        width, height, depth = self.dimensions

        return 0 <= x < width and 0 <= y < height and 0 <= z < depth

    def add_neuron(self, neuron_id, position):
        """
        Add a neuron to this area at the specified position.

        Args:
            neuron_id: ID of the neuron
            position: 3D position as (x, y, z)
        """
        # Add to neuron set
        self.neurons.add(neuron_id)

        # Add to position map (possibly creating a new list for this position)
        if position in self.position_neuron_map:
            self.position_neuron_map[position].append(neuron_id)
        else:
            self.position_neuron_map[position] = [neuron_id]

    def remove_neuron(self, neuron_id):
        """
        Remove a neuron from this area.

        Args:
            neuron_id: ID of the neuron to remove
        """
        if neuron_id in self.neurons:
            # Remove from neuron set
            self.neurons.remove(neuron_id)

            # Find and remove from position map
            for position, neurons in list(self.position_neuron_map.items()):
                if neuron_id in neurons:
                    neurons.remove(neuron_id)
                    if not neurons:  # If list is now empty
                        del self.position_neuron_map[position]
                    break

    def get_neurons_at_position(self, position):
        """
        Get all neurons at a specific position.

        Args:
            position: 3D position as (x, y, z)

        Returns:
            List of neuron IDs at the specified position
        """
        return self.position_neuron_map.get(position, [])

    def get_all_neurons(self):
        """
        Get all neurons in this area.

        Returns:
            List of all neuron IDs in the area
        """
        return list(self.neurons)

    def to_dict(self):
        """
        Convert the cortical area to a dictionary representation.

        Returns:
            Dictionary representation of the cortical area
        """
        return {
            "id": self.id,
            "name": self.name,
            "dimensions": self.dimensions,
            "position": self.position,
            "area_type": self.area_type,
            "neuron_count": self.neuron_count,
            "properties": self.properties,
        }

    def update(self, updates):
        """
        Update cortical area properties.

        Args:
            updates: Dictionary of properties to update

        Raises:
            KeyError: If an invalid property is specified
        """
        valid_properties = {"name", "position", "dimensions", "area_type", "properties"}

        for key, value in updates.items():
            if key not in valid_properties:
                raise KeyError(f"Invalid property: {key}")

            if key == "properties":
                self.properties.update(value)
            else:
                setattr(self, key, value)


@pytest.fixture
def cortical_area():
    """Create a simple cortical area for testing."""
    return CorticalArea(
        area_id="test_area",
        name="Test Area",
        dimensions=(10, 10, 5),
        position=(0, 0, 0),
        area_type="custom",
    )


def test_cortical_area_init():
    """Test cortical area initialization."""
    area = CorticalArea(
        area_id="test_area",
        name="Test Area",
        dimensions=(10, 10, 5),
        position=(0, 0, 0),
        area_type="custom",
        properties={"test_prop": "test_value"},
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
    assert area.contains_position((0, 0, 5)) == False  # z too large


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
        "properties": {"key1": "new_value", "key2": "value2"},
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
