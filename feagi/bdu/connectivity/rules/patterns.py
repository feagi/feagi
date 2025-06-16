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
Pattern-based connectivity rules for the BDU.

This module contains functions for pattern-based morphologies that use
pattern matching to determine connection patterns between cortical areas.
These work with the find_destination_coordinates() function in synaptogenesis.py.
"""

from typing import Any, Dict, Generator, List, Set, Tuple

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)

# Type aliases for improved code readability and Rust compatibility
AreaId = int
NeuronId = int
Position = Tuple[int, int, int]
BoundingBox = Tuple[
    Tuple[int, int, int], Tuple[int, int, int]
]  # ((min_x, min_y, min_z), (max_x, max_y, max_z))


def check_pattern_validity(pattern: List[Any]) -> bool:
    """
    Check if a pattern contains valid elements.

    Args:
        pattern: List of pattern elements

    Returns:
        True if all elements are valid, False otherwise
    """
    valid_patterns = {"*", "?", "!"}
    for element in pattern:
        if element not in valid_patterns:
            try:
                value = int(element)
                if value < 0:
                    return False
            except (ValueError, TypeError):
                return False
    return True


def validate_pattern_element(element: Any) -> bool:
    """
    Validate a single pattern element.

    Args:
        element: Pattern element to validate

    Returns:
        True if element is valid, False otherwise
    """
    valid_patterns = {"*", "?", "!"}

    if element in valid_patterns:
        return True

    try:
        value = int(element)
        return value >= 0
    except (ValueError, TypeError):
        return False


def validate_pattern(pattern: List[Any]) -> bool:
    """
    Validate that a pattern contains only valid elements.

    Args:
        pattern: List of pattern elements to validate

    Returns:
        True if all elements are valid, False otherwise
    """
    return all(validate_pattern_element(element) for element in pattern)


def match_pattern_element(element: Any, coordinate: int) -> bool:
    """
    Check if a coordinate matches a pattern element.

    Args:
        element: Pattern element ("*", "?", "!", or integer)
        coordinate: Coordinate value to match against

    Returns:
        True if coordinate matches the pattern element
    """
    if element == "*":  # Wildcard - matches any coordinate
        return True
    elif element == "?":  # Skip - doesn't match (used for spacing)
        return False
    elif element == "!":  # Exclude - explicitly doesn't match
        return False
    else:
        try:
            # Exact match for integer values
            return int(element) == coordinate
        except (ValueError, TypeError):
            return False


def apply_pattern_to_coordinates(
    pattern: List[Any], coordinates: List[Position]
) -> Generator[Position, None, None]:
    """
    Apply a pattern to filter coordinates.

    Args:
        pattern: Pattern to apply (3-element list for x, y, z)
        coordinates: List of coordinates to filter

    Yields:
        Coordinates that match the pattern
    """
    if len(pattern) != 3:
        logger.warning(f"Pattern must have 3 elements for x,y,z, got {len(pattern)}")
        return

    for x, y, z in coordinates:
        if (
            match_pattern_element(pattern[0], x)
            and match_pattern_element(pattern[1], y)
            and match_pattern_element(pattern[2], z)
        ):
            yield (x, y, z)


def find_source_coordinates(
    src_pattern: List[Any], src_cortical_boundary: Position
) -> Generator[Position, None, None]:
    """
    Generate coordinates within the cortical boundary that match the given pattern.

    Args:
        src_pattern: A tuple (x, y, z) where each element can be an integer or "*".
                    "*" matches all positions along that axis.
        src_cortical_boundary: Dimensions defining the size of the cortical area.

    Yields:
        Coordinates (as tuples) that match the pattern within the cortical boundaries.
    """
    # Generate ranges based on pattern and boundary
    x_range = (
        range(src_cortical_boundary[0]) if src_pattern[0] == "*" else [src_pattern[0]]
    )
    y_range = (
        range(src_cortical_boundary[1]) if src_pattern[1] == "*" else [src_pattern[1]]
    )
    z_range = (
        range(src_cortical_boundary[2]) if src_pattern[2] == "*" else [src_pattern[2]]
    )

    # Use a generator expression to yield each matching coordinate
    for x in x_range:
        for y in y_range:
            for z in z_range:
                yield (x, y, z)


def find_destination_coordinates(
    dst_cortical_boundary: Position,
    src_coordinate: Position,
    src_pattern: List[Any],
    dst_pattern: List[Any],
) -> Generator[Position, None, None]:
    """
    Generate destination coordinates that match the given patterns.

    Args:
        dst_cortical_boundary: Dimensions of the destination cortical area
        src_coordinate: Source coordinate (x, y, z)
        src_pattern: Pattern used for the source coordinates
        dst_pattern: Pattern for mapping to destination coordinates

    Yields:
        Matching destination coordinates
    """
    # Generate ranges based on dst_pattern, dst_cortical_boundary, and src_coordinate
    x_range = (
        range(dst_cortical_boundary[0])
        if dst_pattern[0] == "*"
        else (
            [src_coordinate[0]]
            if (
                dst_pattern[0] == "?"
                and src_coordinate[0] < dst_cortical_boundary[0]
                and (
                    src_coordinate[0] == src_pattern[0] or src_pattern[0] in ["*", "?"]
                )
            )
            else (
                [i for i in range(dst_cortical_boundary[0]) if i != src_coordinate[0]]
                if dst_pattern[0] == "!"
                else (
                    [dst_pattern[0]]
                    if (
                        isinstance(dst_pattern[0], int)
                        and (
                            src_pattern[0] == src_coordinate[0]
                            or src_pattern[0] == "*"
                            or (
                                src_pattern[0] == "?"
                                and dst_pattern[0] == src_coordinate[0]
                            )
                        )
                    )
                    else []
                )
            )
        )
    )

    y_range = (
        range(dst_cortical_boundary[1])
        if dst_pattern[1] == "*"
        else (
            [src_coordinate[1]]
            if (
                dst_pattern[1] == "?"
                and src_coordinate[1] < dst_cortical_boundary[1]
                and (
                    src_coordinate[1] == src_pattern[1] or src_pattern[1] in ["*", "?"]
                )
            )
            else (
                [i for i in range(dst_cortical_boundary[1]) if i != src_coordinate[1]]
                if dst_pattern[1] == "!"
                else (
                    [dst_pattern[1]]
                    if (
                        isinstance(dst_pattern[1], int)
                        and (
                            src_pattern[1] == src_coordinate[1]
                            or src_pattern[1] == "*"
                            or (
                                src_pattern[1] == "?"
                                and dst_pattern[1] == src_coordinate[1]
                            )
                        )
                    )
                    else []
                )
            )
        )
    )

    z_range = (
        range(dst_cortical_boundary[2])
        if dst_pattern[2] == "*"
        else (
            [src_coordinate[2]]
            if (
                dst_pattern[2] == "?"
                and src_coordinate[2] < dst_cortical_boundary[2]
                and (
                    src_coordinate[2] == src_pattern[2] or src_pattern[2] in ["*", "?"]
                )
            )
            else (
                [i for i in range(dst_cortical_boundary[2]) if i != src_coordinate[2]]
                if dst_pattern[2] == "!"
                else (
                    [dst_pattern[2]]
                    if (
                        isinstance(dst_pattern[2], int)
                        and (
                            src_pattern[2] == src_coordinate[2]
                            or src_pattern[2] == "*"
                            or (
                                src_pattern[2] == "?"
                                and dst_pattern[2] == src_coordinate[2]
                            )
                        )
                    )
                    else []
                )
            )
        )
    )

    # Use a generator expression to yield each matching destination coordinate
    for x in x_range:
        for y in y_range:
            for z in z_range:
                yield (x, y, z)


def generate_pattern_coordinates(
    src_pattern: List[Any],
    dst_pattern: List[Any],
    src_coordinate: Position,
    dst_dimensions: Position,
) -> Generator[Position, None, None]:
    """
    Generate destination coordinates based on source and destination patterns.

    Args:
        src_pattern: Source pattern specification
        dst_pattern: Destination pattern specification
        src_coordinate: Source coordinate that matched src_pattern
        dst_dimensions: Destination area dimensions

    Yields:
        Valid destination coordinates matching the dst_pattern
    """
    width, height, depth = dst_dimensions

    # Generate all possible coordinates in destination area
    all_coords = [
        (x, y, z) for x in range(width) for y in range(height) for z in range(depth)
    ]

    # Apply destination pattern to filter coordinates
    yield from apply_pattern_to_coordinates(dst_pattern, all_coords)


def calculate_pattern_offset(
    src_coordinate: Position, src_pattern: List[Any], dst_pattern: List[Any]
) -> Position:
    """
    Calculate coordinate offset based on pattern transformation.

    Args:
        src_coordinate: Source coordinate
        src_pattern: Source pattern that was matched
        dst_pattern: Destination pattern to apply

    Returns:
        Calculated destination coordinate
    """
    # This is a simplified implementation
    # More complex pattern transformations could be implemented here
    offset_x = (
        0
        if dst_pattern[0] == "*"
        else (int(dst_pattern[0]) if dst_pattern[0].isdigit() else src_coordinate[0])
    )
    offset_y = (
        0
        if dst_pattern[1] == "*"
        else (int(dst_pattern[1]) if dst_pattern[1].isdigit() else src_coordinate[1])
    )
    offset_z = (
        0
        if dst_pattern[2] == "*"
        else (int(dst_pattern[2]) if dst_pattern[2].isdigit() else src_coordinate[2])
    )

    return (offset_x, offset_y, offset_z)


def define_subregions(
    area_id: AreaId, parameters: Dict[str, Any], cortical_dimensions: Position
) -> Set[BoundingBox]:
    """
    Define subregions within a cortical area for targeted synaptogenesis.

    Args:
        area_id: ID of the cortical area
        parameters: Dictionary of parameters, must include 'src_seed' and 'src_pattern'
        cortical_dimensions: Dimensions of the cortical area (width, height, depth)

    Returns:
        Set of subregion bounding boxes
    """
    subregions: Set[BoundingBox] = set()
    width, height, depth = cortical_dimensions

    if "src_seed" in parameters and "src_pattern" in parameters:
        seed = parameters["src_seed"]
        # pattern format expected as [[c, s], [c, s], [c, s]] where c indicates choose and s as skip
        pattern = parameters["src_pattern"]

        seed_pointer = [0, 0, 0]

        while seed_pointer[0] <= width:
            for _x_i in range(pattern[0][0]):
                while seed_pointer[1] <= height:
                    for _y_i in range(pattern[1][0]):
                        while seed_pointer[2] <= depth:
                            # Chosen regions
                            for _z_i in range(pattern[2][0]):
                                if (
                                    seed_pointer[0] + seed[0] <= width
                                    and seed_pointer[1] + seed[1] <= height
                                    and seed_pointer[2] + seed[2] <= depth
                                ):
                                    subregions.add(
                                        (
                                            tuple(seed_pointer),
                                            (
                                                seed_pointer[0] + seed[0],
                                                seed_pointer[1] + seed[1],
                                                seed_pointer[2] + seed[2],
                                            ),
                                        )
                                    )
                                seed_pointer[2] += seed[2]
                            # Skip regions
                            for _z_j in range(pattern[2][1]):
                                seed_pointer[2] += seed[2]
                        seed_pointer[1] += seed[1]
                        seed_pointer[2] = 0

                    for _y_j in range(pattern[1][1]):
                        seed_pointer[1] += seed[1]
                seed_pointer[0] += seed[0]
                seed_pointer[1] = 0
                seed_pointer[2] = 0

            for _x_j in range(pattern[0][1]):
                seed_pointer[0] += seed[0]
    return subregions
