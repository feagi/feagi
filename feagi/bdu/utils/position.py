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

from typing import List, Tuple

import numpy as np

"""Position utilities for the BDU.

This module provides utilities for working with 3D positions and coordinates
in the neural space.
"""


def linearize_position(
    position: Tuple[int, int, int], dimensions: Tuple[int, int, int]
) -> int:
    """Convert a 3D position to a linear index based on the dimensions.

    Args:
        position: A 3D tuple (x, y, z) representing the position
        dimensions: A 3D tuple (width, height, depth) representing the dimensions

    Returns:
        Linear index corresponding to the position
    """
    x, y, z = position
    width, height, depth = dimensions

    return x + (y * width) + (z * width * height)


def delinearize_position(
    linear_index: int, dimensions: Tuple[int, int, int]
) -> Tuple[int, int, int]:
    """Convert a linear index back to a 3D position based on the dimensions.

    Args:
        linear_index: The linear index to convert
        dimensions: A 3D tuple (width, height, depth) representing the dimensions

    Returns:
        A 3D tuple (x, y, z) representing the position
    """
    width, height, depth = dimensions

    z = linear_index // (width * height)
    remainder = linear_index % (width * height)
    y = remainder // width
    x = remainder % width

    return (x, y, z)


def validate_position(
    position: Tuple[int, int, int], dimensions: Tuple[int, int, int]
) -> bool:
    """Check if a position is valid within the given dimensions.

    Args:
        position: A 3D tuple (x, y, z) representing the position
        dimensions: A 3D tuple (width, height, depth) representing the dimensions

    Returns:
        True if the position is within bounds, False otherwise

    Raises:
        ValueError: If position or dimensions are not valid 3D tuples
        TypeError: If position or dimensions are not iterable
    """
    # Validate inputs
    if not isinstance(position, (tuple, list)):
        raise TypeError(
            f"Position must be a tuple or list, got {type(position)}"
        )

    if not isinstance(dimensions, (tuple, list)):
        raise TypeError(
            f"Dimensions must be a tuple or list, got {type(dimensions)}"
        )

    if len(position) != 3:
        raise ValueError(
            f"Position must have 3 coordinates (x,y,z), got {len(position)}"
        )

    if len(dimensions) != 3:
        raise ValueError(
            f"Dimensions must have 3 values (width,height,depth), got {len(dimensions)}"
        )

    # Get coordinates and dimensions
    x, y, z = position
    width, height, depth = dimensions

    # Check bounds
    return (0 <= x < width) and (0 <= y < height) and (0 <= z < depth)


def calculate_distance(
    pos1: Tuple[int, int, int], pos2: Tuple[int, int, int]
) -> float:
    """Calculate the Euclidean distance between two 3D positions.

    Args:
        pos1: First position as a 3D tuple (x, y, z)
        pos2: Second position as a 3D tuple (x, y, z)

    Returns:
        The Euclidean distance between the positions
    """
    return np.sqrt(sum((a - b) ** 2 for a, b in zip(pos1, pos2)))


def get_neighbors(
    position: Tuple[int, int, int],
    dimensions: Tuple[int, int, int],
    distance: int = 1,
) -> List[Tuple[int, int, int]]:
    """Get all valid neighbor positions within a given distance.

    Args:
        position: Center position as a 3D tuple (x, y, z)
        dimensions: A 3D tuple (width, height, depth) representing the dimensions
        distance: Maximum distance for neighbors (default: 1 for immediate neighbors)

    Returns:
        List of valid neighbor positions
    """
    x, y, z = position
    width, height, depth = dimensions
    neighbors = []

    for dx in range(-distance, distance + 1):
        for dy in range(-distance, distance + 1):
            for dz in range(-distance, distance + 1):
                if dx == 0 and dy == 0 and dz == 0:
                    continue  # Skip the center position

                nx, ny, nz = x + dx, y + dy, z + dz
                if validate_position((nx, ny, nz), dimensions):
                    neighbors.append((nx, ny, nz))

    return neighbors
