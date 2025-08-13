"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import random
import string
from typing import Any, Dict, List, Optional, Set, Tuple

"""Cortical area data model for the BDU.

This module provides the data model for representing cortical areas,
which are 3D regions containing populations of neurons.
"""


def generate_cortical_id(prefix="C", seed="___"):
    """Generate a 6-character cortical ID.

    Args:
        prefix: Prefix character ('C' for custom, 'M' for memory)
        seed: Optional 3-character seed to use

    Returns:
        A 6-character cortical ID
    """
    seed = seed.replace("-", "_")
    chars = string.ascii_uppercase + string.digits

    # Generate random characters for the part after the prefix
    suffix = "".join(random.choice(chars) for _ in range(2)) + seed[:3]

    # Ensure the total length is 6 characters
    return (prefix + suffix)[:6]


class CorticalArea:
    """Represents a cortical area in the connectome.

    Cortical areas are three-dimensional regions that contain populations of
    neurons and have specific functional roles in the brain.
    """

    def __init__(
        self,
        name: str,
        dimensions: Tuple[int, int, int],
        position: Tuple[int, int, int],
        area_type: str = "custom",
        properties: Optional[Dict[str, Any]] = None,
        cortical_id: Optional[str] = None,
        cortical_idx: Optional[int] = None,
    ):
        """Initialize a new cortical area.

        Args:
            name: Human-readable name for this area
            dimensions: 3D dimensions of the area (width, height, depth)
            position: 3D coordinates of the area's origin in the brain space
            area_type: Type of cortical area (e.g., "sensory", "motor", "custom")
            properties: Additional properties for the area (optional)
            cortical_id: 6-character unique identifier for this area (optional,
                generated if not provided)
            cortical_idx: Integer index for this area (optional, assigned by
                ConnectomeManager)
        """
        self.name = name
        self.dimensions = dimensions
        self.position = position
        self.area_type = area_type
        self.properties = properties or {}

        # Generate cortical_id if not provided
        prefix = "M" if area_type == "memory" else "C"
        self.cortical_id = cortical_id or generate_cortical_id(prefix)

        # cortical_idx is assigned by ConnectomeManager
        self.cortical_idx = cortical_idx

        # Track neuron indices within this area
        self._neuron_indices: Set[int] = set()

        # Cache for neuron positions
        self._position_map: Dict[int, Tuple[int, int, int]] = {}

        # Map positions to neuron IDs
        self._position_to_neurons: Dict[Tuple[int, int, int], List[int]] = {}

        # Region this area belongs to (if any)
        self.region_id: Optional[str] = None

        # For test compatibility
        self.neurons = self._neuron_indices

        # For backward compatibility
        self.id = self.cortical_id

    @property
    def width(self) -> int:
        """Get the width of the area."""
        return self.dimensions[0]

    @property
    def height(self) -> int:
        """Get the height of the area."""
        return self.dimensions[1]

    @property
    def depth(self) -> int:
        """Get the depth of the area."""
        return self.dimensions[2]

    @property
    def volume(self) -> int:
        """Get the total volume of the area."""
        return self.width * self.height * self.depth

    @property
    def neuron_count(self) -> int:
        """Get the number of neurons in this area."""
        return len(self._neuron_indices)

    def contains_position(self, position: Tuple[int, int, int]) -> bool:
        """Check if a position is within the boundaries of this area.

        Args:
            position: 3D coordinates to check

        Returns:
            True if the position is within this area, False otherwise
        """
        x, y, z = position
        return (
            0 <= x < self.width
            and 0 <= y < self.height
            and 0 <= z < self.depth
        )

    def resize(self, new_dimensions: Tuple[int, int, int]) -> List[int]:
        """Resize the cortical area to new dimensions.

        Args:
            new_dimensions: New 3D dimensions (width, height, depth)

        Returns:
            List of neuron indices that are outside the new boundaries
        """
        if new_dimensions == self.dimensions:
            return []

        old_w, old_h, old_d = self.dimensions
        new_w, new_h, new_d = new_dimensions

        removed_indices = []

        # Find neurons that would be outside the new boundaries
        for neuron_idx in self._neuron_indices:
            x, y, z = self._position_map.get(neuron_idx, (0, 0, 0))
            if x >= new_w or y >= new_h or z >= new_d:
                removed_indices.append(neuron_idx)

        # Update dimensions
        self.dimensions = new_dimensions

        # Remove out-of-bounds neurons from tracking
        for idx in removed_indices:
            self._neuron_indices.discard(idx)
            if idx in self._position_map:
                pos = self._position_map[idx]
                del self._position_map[idx]

                # Also update position_to_neurons map
                if pos in self._position_to_neurons:
                    if idx in self._position_to_neurons[pos]:
                        self._position_to_neurons[pos].remove(idx)
                    if not self._position_to_neurons[pos]:
                        del self._position_to_neurons[pos]

        return removed_indices

    def add_neuron(
        self, neuron_id: int, position: Tuple[int, int, int]
    ) -> bool:
        """Add a neuron to this area.

        Args:
            neuron_id: Unique identifier for the neuron
            position: 3D coordinates of the neuron within this area

        Returns:
            True if the neuron was added, False if it was invalid
        """
        if not self.contains_position(position):
            return False

        self._neuron_indices.add(neuron_id)
        self._position_map[neuron_id] = position

        # Update position to neurons mapping
        if position not in self._position_to_neurons:
            self._position_to_neurons[position] = []
        self._position_to_neurons[position].append(neuron_id)

        return True

    def remove_neuron(self, neuron_id: int) -> bool:
        """Remove a neuron from this area.

        Args:
            neuron_id: Unique identifier for the neuron

        Returns:
            True if the neuron was removed, False if it didn't exist
        """
        if neuron_id not in self._neuron_indices:
            return False

        self._neuron_indices.remove(neuron_id)

        # Update position mapping
        if neuron_id in self._position_map:
            pos = self._position_map[neuron_id]
            del self._position_map[neuron_id]

            # Also update position_to_neurons map
            if pos in self._position_to_neurons:
                if neuron_id in self._position_to_neurons[pos]:
                    self._position_to_neurons[pos].remove(neuron_id)
                if not self._position_to_neurons[pos]:
                    del self._position_to_neurons[pos]

        return True

    def get_neuron_position(
        self, neuron_id: int
    ) -> Optional[Tuple[int, int, int]]:
        """Get the position of a neuron in this area.

        Args:
            neuron_id: Unique identifier for the neuron

        Returns:
            3D coordinates if the neuron exists in this area, None otherwise
        """
        return self._position_map.get(neuron_id)

    def update_neuron_position(
        self, neuron_id: int, new_position: Tuple[int, int, int]
    ) -> bool:
        """Update the position of a neuron in this area.

        Args:
            neuron_id: Unique identifier for the neuron
            new_position: New 3D coordinates for the neuron

        Returns:
            True if the position was updated, False if invalid
        """
        if neuron_id not in self._neuron_indices:
            return False

        if not self.contains_position(new_position):
            return False

        # Get the old position for updating position_to_neurons map
        old_position = self._position_map.get(neuron_id)

        # Update position in _position_map
        self._position_map[neuron_id] = new_position

        # Update position_to_neurons map
        if old_position and old_position in self._position_to_neurons:
            if neuron_id in self._position_to_neurons[old_position]:
                self._position_to_neurons[old_position].remove(neuron_id)
            if not self._position_to_neurons[old_position]:
                del self._position_to_neurons[old_position]

        if new_position not in self._position_to_neurons:
            self._position_to_neurons[new_position] = []
        self._position_to_neurons[new_position].append(neuron_id)

        return True

    def get_all_neurons(self) -> List[int]:
        """Get all neurons in this area.

        Returns:
            List of neuron IDs
        """
        return list(self._neuron_indices)

    def get_neurons_at_position(
        self, position: Tuple[int, int, int]
    ) -> List[int]:
        """Get all neurons at a specific position.

        Args:
            position: 3D coordinates to check

        Returns:
            List of neuron IDs at the specified position
        """
        return self._position_to_neurons.get(position, [])

    def to_dict(self) -> Dict[str, Any]:
        """Convert cortical area to dictionary for serialization.

        Returns:
            Dictionary representation of the cortical area
        """
        return {
            "id": self.id,
            "name": self.name,
            "dimensions": self.dimensions,
            "position": self.position,
            "area_type": self.area_type,
            "properties": self.properties,
            "neuron_count": self.neuron_count,
            "cortical_idx": self.cortical_idx,
        }

    def update(self, updates: Dict[str, Any]) -> None:
        """Update cortical area properties.

        Args:
            updates: Dictionary of properties to update

        Raises:
            KeyError: If an invalid property is specified
        """
        valid_properties = {
            "name",
            "position",
            "dimensions",
            "area_type",
            "properties",
        }

        for key, value in updates.items():
            if key not in valid_properties:
                raise KeyError(f"Invalid property: {key}")

            if key == "properties":
                self.properties.update(value)
            else:
                setattr(self, key, value)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]):
        """Create a cortical area from dictionary data.

        Args:
            data: Dictionary representation of the cortical area

        Returns:
            Instantiated cortical area object
        """
        area = cls(
            name=data["name"],
            dimensions=data["dimensions"],
            position=data["position"],
            area_type=data.get("area_type", "custom"),
            properties=data.get("properties", {}),
            cortical_id=data["id"],
            cortical_idx=data.get("cortical_idx"),
        )
        area.region_id = data.get("region_id")
        return area
