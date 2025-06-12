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
Synaptogenesis Rules Module.

This module provides rules for creating synaptic connections between neurons
in different cortical areas based on their spatial relationships and morphologies.
The implementation is optimized for performance and compatibility with Rust.
"""

import re

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)
import random
import traceback
from enum import Enum
from math import prod
from random import randrange
from typing import (
    Any,
    Callable,
    Dict,
    Generator,
    Iterator,
    List,
    Optional,
    Set,
    Tuple,
    Union,
)

import numpy as np
from sympy import sympify

# Import bitmap implementation for efficient set operations
try:
    from feagi.npu.fcl_manager import BitMap
except ImportError:
    # Fallback implementation if the main one is not available
    class BitMap(set):
        """Simple set-based fallback for environments without proper BitMap."""

        def __init__(self, elements=None):
            super().__init__(elements or [])

        def add(self, element: int) -> None:
            super().add(element)

        def copy(self) -> "BitMap":
            return BitMap(self)

        def is_empty(self) -> bool:
            return len(self) == 0


# Type aliases for improved code readability and Rust compatibility
AreaId = int
NeuronId = int
Position = Tuple[int, int, int]
VoxelIndex = Tuple[int, int, int, int]  # area_id, x, y, z
LinearPosition = int
BoundingBox = Tuple[
    Tuple[int, int, int], Tuple[int, int, int]
]  # ((min_x, min_y, min_z), (max_x, max_y, max_z))
MorphologyParams = Dict[str, Any]


class RuleType(Enum):
    """Types of synaptogenesis rules."""

    VECTORS = "vectors"
    PATTERNS = "patterns"
    FUNCTIONS = "functions"
    PLACEHOLDER = "placeholder"


class MorphologyFunction(Enum):
    """Available function-based morphologies."""

    EXPANDER_X = "expander_x"
    REDUCER_X = "reducer_x"
    RANDOMIZER = "randomizer"
    LATERAL_PAIRS_X = "lateral_pairs_x"
    BLOCK_CONNECTION = "block_connection"
    LAST_TO_FIRST = "last_to_first"
    PROJECTOR = "projector"
    PROJECTOR_XY = "projector_xy"
    PROJECTOR_XZ = "projector_xz"
    PROJECTOR_YZ = "projector_yz"
    PROJECT_FROM_END_X = "project_from_end_x"
    PROJECT_FROM_END_Y = "project_from_end_y"
    PROJECT_FROM_END_Z = "project_from_end_z"
    MEMORY = "memory"


# Helper functions for position handling


def linearize_position(position: Position, dimensions: Position) -> LinearPosition:
    """
    Convert a 3D position to a linearized 1D index.

    Args:
        position: 3D position (x, y, z)
        dimensions: Dimensions of the cortical area (width, height, depth)

    Returns:
        Linearized position index
    """
    x, y, z = position
    width, height, depth = dimensions
    return x + (y * width) + (z * width * height)


def delinearize_position(linear_pos: LinearPosition, dimensions: Position) -> Position:
    """
    Convert a linearized 1D index back to a 3D position.

    Args:
        linear_pos: Linearized position index
        dimensions: Dimensions of the cortical area (width, height, depth)

    Returns:
        3D position (x, y, z)
    """
    width, height, depth = dimensions
    z = linear_pos // (width * height)
    remainder = linear_pos % (width * height)
    y = remainder // width
    x = remainder % width
    return (x, y, z)


def preprocess_expression(expr: str) -> str:
    """
    Preprocess algebraic expressions for evaluation.

    Args:
        expr: Expression string

    Returns:
        Preprocessed expression string
    """
    # Add * for implicit multiplication (e.g., 2x -> 2*x)
    expr = re.sub(r"(\d)([a-zA-Z])", r"\1*\2", expr)
    # Replace ^ with ** for exponentiation
    expr = expr.replace("^", "**")
    return expr


def evaluate_expression(expr: Union[str, int], x: int, y: int, z: int) -> int:
    """
    Evaluate an algebraic expression with the given x, y, z values.

    Args:
        expr: Expression string or integer value
        x, y, z: Variable values

    Returns:
        Evaluated integer result
    """
    if isinstance(expr, (int, float)):
        return int(expr)

    try:
        result = sympify(preprocess_expression(expr)).subs({"x": x, "y": y, "z": z})
        return int(result)
    except Exception as e:
        logger.error(f"Error evaluating expression '{expr}': {e}")
        return 0


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
            for x_i in range(pattern[0][0]):
                while seed_pointer[1] <= height:
                    for y_i in range(pattern[1][0]):
                        while seed_pointer[2] <= depth:
                            # Chosen regions
                            for z_i in range(pattern[2][0]):
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
                            for z_j in range(pattern[2][1]):
                                seed_pointer[2] += seed[2]
                        seed_pointer[1] += seed[1]
                        seed_pointer[2] = 0

                    for y_j in range(pattern[1][1]):
                        seed_pointer[1] += seed[1]
                seed_pointer[0] += seed[0]
                seed_pointer[1] = 0
                seed_pointer[2] = 0

            for x_j in range(pattern[0][1]):
                seed_pointer[0] += seed[0]
    return subregions


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


def match_vectors(
    src_voxel: Position,
    dst_area_id: AreaId,
    vector: Union[Position, str],
    morphology_scalar: float,
    src_subregion: BoundingBox,
    connectome_manager,
) -> Set[Position]:
    """
    Find target positions that match vector rules.

    Args:
        src_voxel: Source neuron position
        dst_area_id: Destination area ID
        vector: Vector definition or algebraic expression
        morphology_scalar: Scalar multiplier for the vector
        src_subregion: Bounding box of the source subregion
        connectome_manager: Reference to the ConnectomeManager

    Returns:
        Set of destination positions
    """
    # Get destination area dimensions
    if dst_area_id not in connectome_manager._areas:
        logger.error(f"Destination area {dst_area_id} not found")
        return set()

    dst_area = connectome_manager._areas[dst_area_id]
    dst_dimensions = dst_area.dimensions

    positions = set()

    # Process vector-based destination coordinate
    if isinstance(vector, list) or isinstance(vector, tuple):
        # Convert to a list if it's a tuple
        if isinstance(vector, tuple):
            vector = list(vector)

        # Apply the morphology scalar
        scaled_vector = [int(v * morphology_scalar) for v in vector]

        # Calculate destination voxel
        dst_voxel = tuple(
            max(0, min(src_voxel[i] + scaled_vector[i], dst_dimensions[i] - 1))
            for i in range(3)
        )

        positions.add(dst_voxel)

    # Process algebraic expression-based vector
    elif isinstance(vector, str):
        logger.debug(f"Evaluating expression: {vector}")

        # Create variables dictionary for evaluation
        x, y, z = src_voxel
        variables = {
            "x": x,
            "y": y,
            "z": z,
            "src_x": x,
            "src_y": y,
            "src_z": z,
            "scalar": morphology_scalar,
        }

        # Evaluate expression
        components = [c.strip() for c in vector.split(",")]
        if len(components) != 3:
            logger.error(
                f"Invalid vector expression {vector}, should contain 3 components"
            )
            return positions

        # Parse x, y, z components
        try:
            dst_x = evaluate_expression(components[0], variables)
            dst_y = evaluate_expression(components[1], variables)
            dst_z = evaluate_expression(components[2], variables)

            # Ensure coordinates are within bounds
            dst_x = max(0, min(int(dst_x), dst_dimensions[0] - 1))
            dst_y = max(0, min(int(dst_y), dst_dimensions[1] - 1))
            dst_z = max(0, min(int(dst_z), dst_dimensions[2] - 1))

            positions.add((dst_x, dst_y, dst_z))
        except Exception as e:
            logger.error(f"Error evaluating vector expression {vector}: {str(e)}")

    return positions


def syn_expander_x(
    src_area_id: AreaId,
    dst_area_id: AreaId,
    src_neuron_id: NeuronId,
    src_subregion: BoundingBox,
    connectome_manager,
) -> Set[Position]:
    """
    Expander_X morphology: maps neurons by expanding coordinates from source to destination.

    Args:
        src_area_id: Source area ID
        dst_area_id: Destination area ID
        src_neuron_id: Source neuron ID
        src_subregion: Source region bounding box
        connectome_manager: Reference to ConnectomeManager

    Returns:
        Set of destination positions
    """
    # Get source neuron position
    src_pos = connectome_manager.get_neuron_position(src_neuron_id)
    if not src_pos:
        logger.error(f"Cannot find position for source neuron {src_neuron_id}")
        return set()

    # Get area dimensions
    if (
        src_area_id not in connectome_manager._areas
        or dst_area_id not in connectome_manager._areas
    ):
        logger.error(f"Source or destination area not found")
        return set()

    src_area = connectome_manager._areas[src_area_id]
    dst_area = connectome_manager._areas[dst_area_id]

    src_dims = src_area.dimensions
    dst_dims = dst_area.dimensions

    # Calculate the expansion ratio in each dimension
    ratios = [dst_dims[i] / src_dims[i] if src_dims[i] > 0 else 1.0 for i in range(3)]

    # Compute the destination position by scaling coordinates
    dst_x = min(int(src_pos[0] * ratios[0]), dst_dims[0] - 1)
    dst_y = min(int(src_pos[1] * ratios[1]), dst_dims[1] - 1)
    dst_z = min(int(src_pos[2] * ratios[2]), dst_dims[2] - 1)

    # Return as a set with a single position
    return {(dst_x, dst_y, dst_z)}


def syn_reducer_x(
    src_area_id: AreaId,
    dst_area_id: AreaId,
    src_neuron_id: NeuronId,
    src_subregion: BoundingBox,
    connectome_manager,
    dst_y_index: int = 0,
    dst_z_index: int = 0,
) -> List[Position]:
    """
    Implement the reducer rule for x-dimension.

    This rule reverses the expander rule, mapping source neurons to their
    component representations in the destination area.

    Args:
        src_area_id: Source area ID
        dst_area_id: Destination area ID
        src_neuron_id: Source neuron ID
        src_subregion: Source subregion bounding box
        connectome_manager: Reference to the ConnectomeManager
        dst_y_index: Y-coordinate in destination (default 0)
        dst_z_index: Z-coordinate in destination (default 0)

    Returns:
        List of matching destination positions
    """
    # Get dimensions
    src_cortical_dim_x = src_subregion[1][0] - src_subregion[0][0]
    dst_area = connectome_manager.get_area(dst_area_id)
    dst_cortical_dim_x = dst_area.dimensions[0]

    # Check for sufficient space in destination area
    if src_cortical_dim_x > 2**dst_cortical_dim_x:
        logger.warning(
            f"Area {dst_area_id} does not have enough blocks on x dim for synaptogenesis"
        )

    # Get source neuron position
    src_neuron_pos = connectome_manager.get_neuron_position(src_neuron_id)
    if not src_neuron_pos:
        return []

    src_neuron_block_index_x = src_neuron_pos[1]  # x-coordinate (after area_id)

    # Convert source neuron's x position to binary and pad
    src_neuron_bin_str = bin(src_neuron_block_index_x)[2:]
    if len(src_neuron_bin_str) < dst_cortical_dim_x:
        src_neuron_bin_str = src_neuron_bin_str.rjust(dst_cortical_dim_x, "0")

    candidate_list = []

    # For each destination x-position, check if the corresponding bit is set
    for dst_x_index in range(dst_cortical_dim_x):
        if dst_x_index < len(src_neuron_bin_str) and int(
            src_neuron_bin_str[dst_x_index]
        ):
            candidate_list.append((dst_x_index, dst_y_index, dst_z_index))

    return candidate_list


def syn_randomizer(dst_area_id: AreaId, connectome_manager) -> Position:
    """
    Select a random position in the destination area.

    Args:
        dst_area_id: Destination area ID
        connectome_manager: Reference to the ConnectomeManager

    Returns:
        Random position in the destination area
    """
    dst_area = connectome_manager.get_area(dst_area_id)
    dst_dims = dst_area.dimensions

    random_location = (
        randrange(0, dst_dims[0]),
        randrange(0, dst_dims[1]),
        randrange(0, dst_dims[2]),
    )

    return random_location


def syn_lateral_pairs_x(
    neuron_id: NeuronId, area_id: AreaId, src_subregion: BoundingBox, connectome_manager
) -> Optional[Position]:
    """
    Create lateral connections between neighboring neurons on the x-axis.

    Creates connections in the pattern:
    0->1  2->3 ...
    0<-1  2<-3 ...

    Args:
        neuron_id: Source neuron ID
        area_id: Area ID
        src_subregion: Source subregion bounding box
        connectome_manager: Reference to the ConnectomeManager

    Returns:
        Position of the target neuron, or None if no valid target
    """
    # Get the cortical dimensions and neuron position
    cortical_dim_x = src_subregion[1][0] - src_subregion[0][0]

    neuron_pos = connectome_manager.get_neuron_position(neuron_id)
    if not neuron_pos:
        return None

    neuron_block_index_x = neuron_pos[1]  # x-coordinate
    neuron_block_index_y = neuron_pos[2]  # y-coordinate
    neuron_block_index_z = neuron_pos[3]  # z-coordinate

    # Even neurons connect to the neuron to their right
    if neuron_block_index_x % 2 == 0:
        if neuron_block_index_x + 1 < cortical_dim_x:
            return (
                neuron_block_index_x + 1,
                neuron_block_index_y,
                neuron_block_index_z,
            )
    # Odd neurons connect to the neuron to their left
    else:
        if neuron_block_index_x - 1 >= 0:
            return (
                neuron_block_index_x - 1,
                neuron_block_index_y,
                neuron_block_index_z,
            )

    return None


def syn_block_connection(
    src_area_id: AreaId,
    dst_area_id: AreaId,
    src_neuron_id: NeuronId,
    src_subregion: BoundingBox,
    connectome_manager,
    scaling_factor: int = 10,
) -> Position:
    """
    Map blocks of neurons from source to destination with scaling.

    Maps blocks such that voxel x to x+s from source connected to voxel x//s
    from destination on the x-axis.

    Args:
        src_area_id: Source area ID
        dst_area_id: Destination area ID
        src_neuron_id: Source neuron ID
        src_subregion: Source subregion bounding box
        connectome_manager: Reference to the ConnectomeManager
        scaling_factor: Scaling factor for block connection (default 10)

    Returns:
        Position in the destination area
    """
    # Get the cortical dimensions
    src_cortical_dim_x = src_subregion[1][0] - src_subregion[0][0]
    dst_area = connectome_manager.get_area(dst_area_id)
    dst_cortical_dim_x = dst_area.dimensions[0]

    # Check scaling compatibility
    if src_cortical_dim_x != dst_cortical_dim_x * scaling_factor:
        logger.warning(
            f"Areas {src_area_id} and {dst_area_id} don't have matching blocks for synaptogenesis"
        )

    # Get the neuron's position
    neuron_pos = connectome_manager.get_neuron_position(src_neuron_id)
    if not neuron_pos:
        # Return a default position if neuron position can't be found
        return (0, 0, 0)

    neuron_block_index_x = neuron_pos[1]  # x-coordinate
    neuron_block_index_y = neuron_pos[2]  # y-coordinate
    neuron_block_index_z = neuron_pos[3]  # z-coordinate

    # Calculate the destination position by scaling
    return (
        neuron_block_index_x // scaling_factor,
        neuron_block_index_y,
        neuron_block_index_z,
    )


def syn_projector(
    src_area_id: AreaId,
    dst_area_id: AreaId,
    src_neuron_id: NeuronId,
    src_subregion: BoundingBox,
    connectome_manager,
    transpose: Optional[Tuple[str, str, str]] = None,
    project_last_layer_of: Optional[str] = None,
) -> List[Position]:
    """
    Project neurons from source to destination while maintaining topology.

    This is a complex mapping function that handles various projections including:
    - Standard projection maintaining relative positions
    - Transposed projections (swapping axes)
    - Special projections from the last layer of an axis

    Args:
        src_area_id: Source area ID
        dst_area_id: Destination area ID
        src_neuron_id: Source neuron ID
        src_subregion: Source subregion bounding box
        connectome_manager: Reference to the ConnectomeManager
        transpose: Optional tuple of axes to transpose ("x", "y", "z")
        project_last_layer_of: Optional axis to project from last layer ("x", "y", or "z")

    Returns:
        List of matching positions in the destination area
    """
    src_area = connectome_manager.get_area(src_area_id)
    src_dimensions = src_area.dimensions

    dst_area = connectome_manager.get_area(dst_area_id)
    dst_dimensions = dst_area.dimensions

    # These will be updated based on the transpose and project_last_layer parameters
    src_shape = [0, 0, 0]
    dst_shape = list(dst_dimensions)

    # Get the neuron's position
    neuron_pos = connectome_manager.get_neuron_position(src_neuron_id)
    if not neuron_pos:
        return []

    # Default neuron location
    neuron_location = [neuron_pos[1], neuron_pos[2], neuron_pos[3]]

    # Apply transpose if specified
    if transpose:
        # Map axis names to indices
        axis_map = {"x": 0, "y": 1, "z": 2}
        transpose_indices = [axis_map[axis] for axis in transpose]

        # Transpose subregion
        transposed_subregion = [
            [
                src_subregion[0][transpose_indices[0]],
                src_subregion[0][transpose_indices[1]],
                src_subregion[0][transpose_indices[2]],
            ],
            [
                src_subregion[1][transpose_indices[0]],
                src_subregion[1][transpose_indices[1]],
                src_subregion[1][transpose_indices[2]],
            ],
        ]
        src_subregion = transposed_subregion

        # Transpose neuron location
        neuron_location = [
            neuron_location[transpose_indices[0]],
            neuron_location[transpose_indices[1]],
            neuron_location[transpose_indices[2]],
        ]

    # Handle special projection from last layer
    if project_last_layer_of:
        if project_last_layer_of == "x":
            src_shape = [
                src_dimensions[0] - 1,  # -1 because we're using the last layer
                src_subregion[1][1] - src_subregion[0][1],
                src_subregion[1][2] - src_subregion[0][2],
            ]
            dst_shape[0] = 1  # Project to a single layer in destination

        elif project_last_layer_of == "y":
            src_shape = [
                src_subregion[1][0] - src_subregion[0][0],
                src_dimensions[1] - 1,  # -1 because we're using the last layer
                src_subregion[1][2] - src_subregion[0][2],
            ]
            dst_shape[1] = 1  # Project to a single layer in destination

        elif project_last_layer_of == "z":
            src_shape = [
                src_subregion[1][0] - src_subregion[0][0],
                src_subregion[1][1] - src_subregion[0][1],
                src_dimensions[2] - 1,  # -1 because we're using the last layer
            ]
            dst_shape[2] = 1  # Project to a single layer in destination
    else:
        # Standard projection - use full source subregion
        src_shape = [
            src_subregion[1][0] - src_subregion[0][0],
            src_subregion[1][1] - src_subregion[0][1],
            src_subregion[1][2] - src_subregion[0][2],
        ]

    # Dictionary to store potential destination voxels for each axis
    dst_vox_dict = {0: set(), 1: set(), 2: set()}
    candidate_list = []

    try:
        # For each axis
        for i in range(3):
            if src_shape[i] > dst_shape[i]:
                # Source is larger: scale down
                ratio = src_shape[i] / dst_shape[i]
                target_vox = int((neuron_location[i] - src_subregion[0][i]) / ratio)

                # Special handling for project_last_layer
                if (
                    (project_last_layer_of == "x" and i == 0)
                    or (project_last_layer_of == "y" and i == 1)
                    or (project_last_layer_of == "z" and i == 2)
                ):
                    target_vox = 0  # Always map to the first layer

                dst_vox_dict[i].add(target_vox)

            elif src_shape[i] < dst_shape[i]:
                # Source is smaller: scale up
                ratio = dst_shape[i] / src_shape[i]

                # Find all voxels that map to this source voxel
                for vox in range(dst_shape[i]):
                    # Integer division to group voxels
                    if int(vox / ratio) == (neuron_location[i] - src_subregion[0][i]):
                        dst_vox_dict[i].add(vox)

            elif src_shape[i] == dst_shape[i]:
                # Source and destination are the same size: maintain position
                target_vox = neuron_location[i] - src_subregion[0][i]
                if 0 <= target_vox < dst_shape[i]:  # Ensure within bounds
                    dst_vox_dict[i].add(target_vox)

    except ZeroDivisionError:
        logger.warning("Zero division error during projection calculation")
        return []

    # Generate all combinations of the destination coordinates
    if dst_vox_dict[0] and dst_vox_dict[1] and dst_vox_dict[2]:
        for x in dst_vox_dict[0]:
            for y in dst_vox_dict[1]:
                for z in dst_vox_dict[2]:
                    # Ensure within bounds
                    if (
                        0 <= x < dst_dimensions[0]
                        and 0 <= y < dst_dimensions[1]
                        and 0 <= z < dst_dimensions[2]
                    ):
                        candidate_list.append((x, y, z))

    return candidate_list


def syn_memory(
    src_area_id: AreaId, dst_area_id: AreaId, memory_register: Dict[AreaId, Set[AreaId]]
) -> None:
    """
    Register source-destination area relationship in memory register.

    Args:
        src_area_id: Source area ID
        dst_area_id: Destination area ID
        memory_register: Memory register dictionary to update
    """
    if dst_area_id not in memory_register:
        memory_register[dst_area_id] = set()

    memory_register[dst_area_id].add(src_area_id)


def last_to_first(src_area_id: AreaId, connectome_manager) -> List[Position]:
    """
    Generate a connection from the last neuron to the first.

    Args:
        src_area_id: Source area ID
        connectome_manager: Reference to the ConnectomeManager

    Returns:
        Position of the first voxel [0, 0, 0]
    """
    return [(0, 0, 0)]


def neighbor_finder(position, neighbor_range=1, include_self=False):
    """
    Find all neighboring positions within a given range.

    Args:
        position: 3D position as (x, y, z)
        neighbor_range: Range of positions to include as neighbors
        include_self: Whether to include the center position

    Returns:
        List of all neighboring positions (x, y, z)
    """
    x, y, z = position
    neighbors = []

    # Iterate through all possible offsets within the range
    for dx in range(-neighbor_range, neighbor_range + 1):
        for dy in range(-neighbor_range, neighbor_range + 1):
            for dz in range(-neighbor_range, neighbor_range + 1):
                # Skip the center position if not requested
                if not include_self and (dx, dy, dz) == (0, 0, 0):
                    continue

                # Add this neighbor position
                neighbors.append((x + dx, y + dy, z + dz))

    return neighbors


def neighbor_finder_extended(
    src_area_id: AreaId,
    src_position: Position,
    src_neuron_id: NeuronId,
    dst_area_id: Optional[AreaId] = None,
    dst_position: Optional[Position] = None,
    radius: Optional[int] = None,
    connection_mask: Optional[int] = None,
    self_connection: bool = False,
) -> List[Position]:
    """Extended neighbor finder for complex connection patterns."""
    # Implementation for extended neighbor finding
    return []


def find_candidate_neurons(
    src_area_id: AreaId,
    dst_area_id: AreaId,
    src_neuron_id: NeuronId,
    morphology: Dict[str, Any],
    src_subregion: BoundingBox,
    connectome_manager,
    memory_register: Dict[AreaId, Set[AreaId]],
    morphology_id_overwrite: Optional[str] = None,
) -> List[Tuple[NeuronId, float]]:
    """
    Find candidate neurons in the destination area for synaptic connections.

    This is the main entry point for synaptogenesis rules. It determines which neurons
    in the destination area should be connected to the source neuron based on
    morphology rules.

    Args:
        src_area_id: Source area ID
        dst_area_id: Destination area ID
        src_neuron_id: Source neuron ID
        morphology: Morphology parameters (dict with morphology_id, morphology_scalar, etc.)
        src_subregion: Source subregion bounding box
        connectome_manager: Reference to the ConnectomeManager
        memory_register: Memory register dictionary
        morphology_id_overwrite: Optional override for morphology ID

    Returns:
        List of (target_neuron_id, weight) tuples for synaptic connections
    """
    # Get source neuron position
    src_neuron_pos = connectome_manager.get_neuron_position(src_neuron_id)
    if not src_neuron_pos:
        logger.error(f"Cannot find position for source neuron {src_neuron_id}")
        return []

    # Get position tuple (x, y, z)
    src_voxel = src_neuron_pos

    # Determine morphology ID and parameters
    neuron_morphology = morphology_id_overwrite or morphology["morphology_id"]
    morphology_scalar = morphology["morphology_scalar"]
    psc_multiplier = morphology["postSynapticCurrent_multiplier"]

    # Get source area properties
    if src_area_id not in connectome_manager._areas:
        logger.error(f"Source area {src_area_id} not found")
        return []

    src_area = connectome_manager._areas[src_area_id]
    psc_base = src_area.properties.get("postsynaptic_current", 1.0)
    post_synaptic_current = psc_multiplier * psc_base

    # Storage for candidate positions and neurons
    raw_candidate_positions = set()
    candidate_neuron_list = []

    # Get morphology type
    if not hasattr(connectome_manager, "get_morphologies_registry"):
        logger.error("ConnectomeManager does not have morphologies registry")
        # Use default morphology type from the parameter
        morphology_type = RuleType.FUNCTIONS.value
    else:
        morphologies_registry = connectome_manager.get_morphologies_registry()
        if neuron_morphology not in morphologies_registry:
            logger.error(f"Morphology {neuron_morphology} not found in registry")
            return []

        morphology_type = morphologies_registry[neuron_morphology]["type"]

    try:
        # Process based on morphology type
        if morphology_type == RuleType.VECTORS.value:
            for vector in morphologies_registry[neuron_morphology]["parameters"][
                "vectors"
            ]:
                positions = match_vectors(
                    src_voxel=src_voxel,
                    dst_area_id=dst_area_id,
                    vector=vector,
                    morphology_scalar=morphology_scalar,
                    src_subregion=src_subregion,
                    connectome_manager=connectome_manager,
                )

                # Add all candidate positions to the set
                for pos in positions:
                    raw_candidate_positions.add(pos)

        elif morphology_type == RuleType.PATTERNS.value:
            for pattern in morphologies_registry[neuron_morphology]["parameters"][
                "patterns"
            ]:
                # Get destination area dimensions
                if dst_area_id not in connectome_manager._areas:
                    logger.error(f"Destination area {dst_area_id} not found")
                    continue

                dst_area = connectome_manager._areas[dst_area_id]
                dst_dimensions = dst_area.dimensions

                source_pattern = pattern[0]
                destination_pattern = pattern[1]

                # Find matching destination coordinates
                for dst_pos in find_destination_coordinates(
                    dst_cortical_boundary=dst_dimensions,
                    src_coordinate=src_voxel,
                    src_pattern=source_pattern,
                    dst_pattern=destination_pattern,
                ):
                    raw_candidate_positions.add(dst_pos)

        elif morphology_type == RuleType.FUNCTIONS.value:
            # Process function-based morphologies
            if neuron_morphology == MorphologyFunction.EXPANDER_X.value:
                positions = syn_expander_x(
                    src_area_id=src_area_id,
                    dst_area_id=dst_area_id,
                    src_neuron_id=src_neuron_id,
                    src_subregion=src_subregion,
                    connectome_manager=connectome_manager,
                )
                for pos in positions:
                    raw_candidate_positions.add(pos)

            elif neuron_morphology == MorphologyFunction.REDUCER_X.value:
                positions = syn_reducer_x(
                    src_area_id=src_area_id,
                    dst_area_id=dst_area_id,
                    src_neuron_id=src_neuron_id,
                    src_subregion=src_subregion,
                    connectome_manager=connectome_manager,
                )
                for pos in positions:
                    raw_candidate_positions.add(pos)

            elif neuron_morphology == MorphologyFunction.RANDOMIZER.value:
                pos = syn_randomizer(
                    dst_area_id=dst_area_id, connectome_manager=connectome_manager
                )
                raw_candidate_positions.add(pos)

            elif neuron_morphology == MorphologyFunction.LATERAL_PAIRS_X.value:
                pos = syn_lateral_pairs_x(
                    neuron_id=src_neuron_id,
                    area_id=src_area_id,
                    src_subregion=src_subregion,
                    connectome_manager=connectome_manager,
                )
                if pos:
                    raw_candidate_positions.add(pos)

            elif neuron_morphology == MorphologyFunction.BLOCK_CONNECTION.value:
                pos = syn_block_connection(
                    src_area_id=src_area_id,
                    dst_area_id=dst_area_id,
                    src_neuron_id=src_neuron_id,
                    src_subregion=src_subregion,
                    connectome_manager=connectome_manager,
                )
                raw_candidate_positions.add(pos)

            elif neuron_morphology == MorphologyFunction.LAST_TO_FIRST.value:
                # Check if this is the last neuron in the area
                src_dimensions = src_area.dimensions
                last_pos = (
                    src_dimensions[0] - 1,
                    src_dimensions[1] - 1,
                    src_dimensions[2] - 1,
                )

                if src_voxel == last_pos:
                    raw_candidate_positions.add((0, 0, 0))

            elif neuron_morphology == MorphologyFunction.PROJECTOR.value:
                positions = syn_projector(
                    src_area_id=src_area_id,
                    dst_area_id=dst_area_id,
                    src_neuron_id=src_neuron_id,
                    src_subregion=src_subregion,
                    connectome_manager=connectome_manager,
                )
                for pos in positions:
                    raw_candidate_positions.add(pos)

            # Handle special memory morphology that doesn't produce voxel positions
            elif neuron_morphology == MorphologyFunction.MEMORY.value:
                syn_memory(src_area_id, dst_area_id, memory_register)
                # No positions are added for memory function

            else:
                logger.warning(f"Unsupported morphology function: {neuron_morphology}")

        else:
            logger.warning(f"Unsupported morphology type: {morphology_type}")

        # For each candidate position, find the neurons there and add them to the result list
        for dst_pos in raw_candidate_positions:
            if dst_pos is None:
                continue

            # Get neurons at this position in the destination area
            dst_neurons = connectome_manager.get_neurons_at_position(
                dst_area_id, dst_pos
            )

            # Add each destination neuron with its weight to the candidate list
            for dst_neuron_id in dst_neurons:
                if dst_neuron_id:
                    candidate_neuron_list.append((dst_neuron_id, post_synaptic_current))

        logger.debug(f"Found {len(candidate_neuron_list)} destination neurons")
        return candidate_neuron_list

    except Exception as e:
        logger.error(f"Error in neighbor_finder: {str(e)}")
        logger.exception("Exception details:")
        return []


class SynapseRule:
    """
    Base class for synaptogenesis rules.

    This class defines the interface for synapse formation rules that
    determine how neurons connect between cortical areas.
    """

    def __init__(
        self,
        rule_type: str,
        source_area: str,
        target_area: str,
        parameters: Dict[str, Any] = None,
    ):
        """
        Initialize a synapse rule.

        Args:
            rule_type: Type of synapse rule (e.g., "one_to_one", "random", "distance_based")
            source_area: ID of the source cortical area
            target_area: ID of the target cortical area
            parameters: Additional parameters specific to this rule type
        """
        self.rule_type = rule_type
        self.source_area = source_area
        self.target_area = target_area
        self.parameters = parameters or {}

    def generate_connections(
        self,
        source_neurons: List[int],
        target_neurons: List[int],
        source_positions: Dict[int, Tuple[int, int, int]],
        target_positions: Dict[int, Tuple[int, int, int]],
    ) -> Dict[int, List[int]]:
        """
        Generate connections between source and target neurons.

        Args:
            source_neurons: List of source neuron IDs
            target_neurons: List of target neuron IDs
            source_positions: Dictionary mapping source neuron IDs to positions
            target_positions: Dictionary mapping target neuron IDs to positions

        Returns:
            Dictionary mapping source neuron IDs to lists of target neuron IDs
        """
        # Base implementation is a no-op
        # Subclasses should override this method with specific connection logic
        return {}

    def to_dict(self) -> Dict[str, Any]:
        """
        Convert the rule to a dictionary representation.

        Returns:
            Dictionary representation of the rule
        """
        return {
            "rule_type": self.rule_type,
            "source_area": self.source_area,
            "target_area": self.target_area,
            "parameters": self.parameters,
        }

    @classmethod
    def from_dict(cls, rule_dict: Dict[str, Any]) -> "SynapseRule":
        """
        Create a synapse rule from a dictionary representation.

        Args:
            rule_dict: Dictionary representation of a synapse rule

        Returns:
            A new SynapseRule instance or subclass instance
        """
        rule_type = rule_dict.get("rule_type", "default")
        source_area = rule_dict.get("source_area", "")
        target_area = rule_dict.get("target_area", "")
        parameters = rule_dict.get("parameters", {})

        # Create the appropriate rule type based on the rule_type
        if rule_type == "one_to_one":
            return OneToOneRule(source_area, target_area, parameters)
        elif rule_type == "random":
            return RandomRule(source_area, target_area, parameters)
        elif rule_type == "distance_based":
            return DistanceBasedRule(source_area, target_area, parameters)
        else:
            # Default to base class if rule_type is unknown
            return cls(rule_type, source_area, target_area, parameters)


class OneToOneRule(SynapseRule):
    """
    Rule that connects neurons with the same relative position in source and target areas.

    This rule creates one-to-one mappings between neurons in source and target areas,
    matching them based on their relative positions.
    """

    def __init__(
        self, source_area: str, target_area: str, parameters: Dict[str, Any] = None
    ):
        """
        Initialize a one-to-one mapping rule.

        Args:
            source_area: ID of the source cortical area
            target_area: ID of the target cortical area
            parameters:
                - scale_factor: Factor to scale positions by (default: 1.0)
                - offset: Position offset to apply (default: (0, 0, 0))
        """
        super().__init__("one_to_one", source_area, target_area, parameters)

    def generate_connections(
        self,
        source_neurons: List[int],
        target_neurons: List[int],
        source_positions: Dict[int, Tuple[int, int, int]],
        target_positions: Dict[int, Tuple[int, int, int]],
    ) -> Dict[int, List[int]]:
        """
        Generate one-to-one connections based on relative positions.

        Args:
            source_neurons: List of source neuron IDs
            target_neurons: List of target neuron IDs
            source_positions: Dictionary mapping source neuron IDs to positions
            target_positions: Dictionary mapping target neuron IDs to positions

        Returns:
            Dictionary mapping source neuron IDs to lists of target neuron IDs
        """
        connections = {}

        # Get parameter values
        scale_factor = self.parameters.get("scale_factor", 1.0)
        offset = self.parameters.get("offset", (0, 0, 0))

        # Create a lookup of target positions to target neurons
        target_pos_to_neuron = {}
        for neuron_id in target_neurons:
            if neuron_id in target_positions:
                pos = target_positions[neuron_id]
                target_pos_to_neuron[pos] = neuron_id

        # For each source neuron, find a matching target neuron
        for source_id in source_neurons:
            if source_id not in source_positions:
                continue

            # Get source position and apply scaling/offset
            sx, sy, sz = source_positions[source_id]
            tx = int(sx * scale_factor) + offset[0]
            ty = int(sy * scale_factor) + offset[1]
            tz = int(sz * scale_factor) + offset[2]

            # Look for a target neuron at the corresponding position
            target_pos = (tx, ty, tz)
            if target_pos in target_pos_to_neuron:
                target_id = target_pos_to_neuron[target_pos]
                connections[source_id] = [target_id]

        return connections


class RandomRule(SynapseRule):
    """
    Rule that creates random connections between source and target neurons.

    This rule connects a variable number of source neurons to randomly selected
    target neurons.
    """

    def __init__(
        self, source_area: str, target_area: str, parameters: Dict[str, Any] = None
    ):
        """
        Initialize a random connection rule.

        Args:
            source_area: ID of the source cortical area
            target_area: ID of the target cortical area
            parameters:
                - connection_probability: Probability of forming a connection (default: 0.1)
                - max_connections: Maximum number of connections per source neuron (default: 10)
                - seed: Random seed for reproducibility (default: None)
        """
        super().__init__("random", source_area, target_area, parameters)

    def generate_connections(
        self,
        source_neurons: List[int],
        target_neurons: List[int],
        source_positions: Dict[int, Tuple[int, int, int]],
        target_positions: Dict[int, Tuple[int, int, int]],
    ) -> Dict[int, List[int]]:
        """
        Generate random connections between source and target neurons.

        Args:
            source_neurons: List of source neuron IDs
            target_neurons: List of target neuron IDs
            source_positions: Dictionary mapping source neuron IDs to positions
            target_positions: Dictionary mapping target neuron IDs to positions

        Returns:
            Dictionary mapping source neuron IDs to lists of target neuron IDs
        """
        connections = {}

        # Get parameter values
        connection_prob = self.parameters.get("connection_probability", 0.1)
        max_connections = self.parameters.get("max_connections", 10)
        seed = self.parameters.get("seed", None)

        # Set random seed if provided
        if seed is not None:
            random.seed(seed)

        # For each source neuron, randomly connect to target neurons
        for source_id in source_neurons:
            # Determine number of connections for this source neuron
            num_connections = min(
                max_connections, int(connection_prob * len(target_neurons))
            )

            # Randomly select target neurons without replacement
            if num_connections > 0:
                connections[source_id] = random.sample(target_neurons, num_connections)

        return connections


class DistanceBasedRule(SynapseRule):
    """
    Rule that connects neurons based on distance between their positions.

    This rule creates connections where the probability depends on the
    distance between neurons (closer neurons more likely to connect).
    """

    def __init__(
        self, source_area: str, target_area: str, parameters: Dict[str, Any] = None
    ):
        """
        Initialize a distance-based connection rule.

        Args:
            source_area: ID of the source cortical area
            target_area: ID of the target cortical area
            parameters:
                - max_distance: Maximum distance for connections (default: 5)
                - distance_falloff: How quickly connection probability drops with distance (default: 2)
                - max_connections: Maximum number of connections per source neuron (default: 10)
                - seed: Random seed for reproducibility (default: None)
        """
        super().__init__("distance_based", source_area, target_area, parameters)

    def generate_connections(
        self,
        source_neurons: List[int],
        target_neurons: List[int],
        source_positions: Dict[int, Tuple[int, int, int]],
        target_positions: Dict[int, Tuple[int, int, int]],
    ) -> Dict[int, List[int]]:
        """
        Generate distance-based connections between source and target neurons.

        Args:
            source_neurons: List of source neuron IDs
            target_neurons: List of target neuron IDs
            source_positions: Dictionary mapping source neuron IDs to positions
            target_positions: Dictionary mapping target neuron IDs to positions

        Returns:
            Dictionary mapping source neuron IDs to lists of target neuron IDs
        """
        connections = {}

        # Get parameter values
        max_distance = self.parameters.get("max_distance", 5)
        distance_falloff = self.parameters.get("distance_falloff", 2)
        max_connections = self.parameters.get("max_connections", 10)
        seed = self.parameters.get("seed", None)

        # Set random seed if provided
        if seed is not None:
            random.seed(seed)

        # For each source neuron, find target neurons within distance range
        for source_id in source_neurons:
            if source_id not in source_positions:
                continue

            source_pos = source_positions[source_id]

            # Calculate distances to all target neurons
            candidates = []
            for target_id in target_neurons:
                if target_id in target_positions:
                    target_pos = target_positions[target_id]

                    # Calculate Euclidean distance
                    distance = (
                        (source_pos[0] - target_pos[0]) ** 2
                        + (source_pos[1] - target_pos[1]) ** 2
                        + (source_pos[2] - target_pos[2]) ** 2
                    ) ** 0.5

                    # If within maximum distance, add as candidate with probability weight
                    if distance <= max_distance:
                        # Connection probability decreases with distance
                        weight = 1.0 / (1.0 + distance**distance_falloff)
                        candidates.append((target_id, weight))

            # If we have candidates, select some based on weighted probability
            if candidates:
                # Extract IDs and weights
                ids, weights = zip(*candidates)

                # Normalize weights to probabilities
                total_weight = sum(weights)
                if total_weight > 0:
                    probs = [w / total_weight for w in weights]

                    # Select target neurons with weighted probability
                    num_connections = min(max_connections, len(candidates))
                    connections[source_id] = []

                    # Use weighted random sampling without replacement
                    for _ in range(num_connections):
                        if not ids:
                            break

                        # Select a neuron with probability proportional to its weight
                        index = random.choices(range(len(ids)), weights=probs, k=1)[0]
                        connections[source_id].append(ids[index])

                        # Remove selected neuron from candidates
                        ids = ids[:index] + ids[index + 1 :]
                        probs = probs[:index] + probs[index + 1 :]

        return connections
