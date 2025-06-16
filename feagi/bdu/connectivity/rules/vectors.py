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
Vector-based connectivity rules for the BDU.

This module contains functions for vector-based morphologies that use
vector operations to determine connection patterns between cortical areas.
These work with the match_vectors() function in synaptogenesis.py.
"""

import re
from typing import Any, Dict, List, Set, Tuple, Union

from sympy import sympify

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)

# Type aliases for improved code readability and Rust compatibility
AreaId = int
NeuronId = int
Position = Tuple[int, int, int]
BoundingBox = Tuple[
    Tuple[int, int, int], Tuple[int, int, int]
]  # ((min_x, min_y, min_z), (max_x, max_y, max_z))


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


def evaluate_expression(expr: Union[str, int], variables: Dict[str, Any]) -> int:
    """
    Evaluate an algebraic expression with the given variables.

    Args:
        expr: Expression string or integer value
        variables: Dictionary of variable values

    Returns:
        Evaluated integer result
    """
    if isinstance(expr, (int, float)):
        return int(expr)

    try:
        result = sympify(preprocess_expression(expr)).subs(variables)
        return int(result)
    except Exception as e:
        logger.error(f"Error evaluating expression '{expr}': {e}")
        return 0


def apply_vector_offset(
    src_position: Position, vector: Union[Position, str], morphology_scalar: float = 1.0
) -> Position:
    """
    Apply a vector offset to a source position.

    Args:
        src_position: Source neuron position (x, y, z)
        vector: Vector offset as tuple (dx, dy, dz) or string representation
        morphology_scalar: Scaling factor for the vector

    Returns:
        New position after applying vector offset
    """
    if isinstance(vector, str):
        # Parse string vector representation like "(1,0,0)" or "1,0,0"
        vector_str = vector.strip("()")
        vector = tuple(map(int, vector_str.split(",")))

    # Apply scalar and offset
    scaled_vector = tuple(int(v * morphology_scalar) for v in vector)
    result_position = tuple(src_position[i] + scaled_vector[i] for i in range(3))

    return result_position


def validate_vector_position(position: Position, dst_dimensions: Position) -> bool:
    """
    Validate that a position is within the destination area bounds.

    Args:
        position: Position to validate (x, y, z)
        dst_dimensions: Destination area dimensions (width, height, depth)

    Returns:
        True if position is valid, False otherwise
    """
    x, y, z = position
    width, height, depth = dst_dimensions

    return 0 <= x < width and 0 <= y < height and 0 <= z < depth


def generate_vector_candidates(
    src_position: Position,
    vectors: List[Union[Position, str]],
    morphology_scalar: float,
    dst_dimensions: Position,
) -> Set[Position]:
    """
    Generate candidate positions by applying multiple vectors to a source position.

    Args:
        src_position: Source neuron position
        vectors: List of vector offsets to apply
        morphology_scalar: Scaling factor for vectors
        dst_dimensions: Destination area dimensions for bounds checking

    Returns:
        Set of valid candidate positions
    """
    candidates = set()

    for vector in vectors:
        candidate_pos = apply_vector_offset(src_position, vector, morphology_scalar)

        if validate_vector_position(candidate_pos, dst_dimensions):
            candidates.add(candidate_pos)
        else:
            logger.debug(
                f"Vector candidate {candidate_pos} out of bounds for dimensions {dst_dimensions}"
            )

    return candidates


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
    logger.debug(
        f"[MATCH_VECTORS] Starting with dst_area_id: {dst_area_id}, vector: {vector}"
    )
    if dst_area_id not in connectome_manager.cortical_areas:
        logger.error(f"Destination area {dst_area_id} not found")
        return set()

    dst_area = connectome_manager.cortical_areas[dst_area_id]
    dst_dimensions = dst_area.dimensions
    logger.debug(f"[MATCH_VECTORS] dst_dimensions: {dst_dimensions}")

    positions = set()

    # Process vector-based destination coordinate
    logger.debug(
        f"[MATCH_VECTORS] Checking vector type: {type(vector)}, isinstance list: {isinstance(vector, list)}, isinstance tuple: {isinstance(vector, tuple)}"
    )
    if isinstance(vector, list) or isinstance(vector, tuple):
        logger.debug("[MATCH_VECTORS] Processing vector as list/tuple")
        # Convert to a list if it's a tuple
        if isinstance(vector, tuple):
            vector = list(vector)
            logger.debug(f"[MATCH_VECTORS] Converted tuple to list: {vector}")

        # Apply the morphology scalar
        scaled_vector = [int(v * morphology_scalar) for v in vector]

        # DEBUG: Add logging to see what's happening
        logger.debug(
            f"[MATCH_VECTORS DEBUG] src_voxel: {src_voxel}, vector: {vector}, scaled_vector: {scaled_vector}, dst_dimensions: {dst_dimensions}"
        )

        # Calculate destination voxel
        dst_voxel = tuple(
            max(0, min(src_voxel[i] + scaled_vector[i], dst_dimensions[i] - 1))
            for i in range(3)
        )

        logger.debug(f"[MATCH_VECTORS DEBUG] calculated dst_voxel: {dst_voxel}")
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

    logger.debug(f"[MATCH_VECTORS] Returning positions: {positions}")
    return positions
