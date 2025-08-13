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

"""
Synaptogenesis Rules Module.

This module provides rules for creating synaptic connections between neurons
in different cortical areas based on their spatial relationships and morphologies.
The implementation is optimized for performance and compatibility with Rust.
"""

import re
from enum import Enum
from typing import Any, Dict, List, Optional, Set, Tuple, Union

from sympy import sympify

from feagi.utils.logger import setup_logger

# Import function-based morphology implementations
from .rules.functions import (
    syn_block_connection,
    syn_expander_x,
    syn_last_to_first,
    syn_lateral_pairs_x,
    syn_memory,
    syn_projector,
    syn_randomizer,
    syn_reducer_x,
)

# Import pattern-based morphology implementations
from .rules.patterns import (
    check_pattern_validity,
    define_subregions,
    find_destination_coordinates,
    find_source_coordinates,
)

# Import vector-based morphology implementations
from .rules.vectors import match_vectors

logger = setup_logger(__name__)

# Explicitly export imported functions for use by test modules and external consumers
__all__ = [
    # Function-based morphologies
    "syn_block_connection",
    "syn_expander_x",
    "syn_last_to_first",
    "syn_lateral_pairs_x",
    "syn_memory",
    "syn_projector",
    "syn_randomizer",
    "syn_reducer_x",
    # Pattern-based functions
    "check_pattern_validity",
    "define_subregions",
    "find_destination_coordinates",
    "find_source_coordinates",
    # Vector-based functions
    "match_vectors",
    # Helper functions
    "linearize_position",
    "delinearize_position",
    "evaluate_expression",
    "neighbor_finder",
    "neighbor_finder_extended",
    "find_candidate_neurons",
    # Enums and types
    "RuleType",
    "MorphologyFunction",
]


def _is_debug_bdu_enabled() -> bool:
    """Check if BDU (Brain Development Unit) debugging is enabled.

    Returns:
    True if BDU debugging is enabled, False otherwise
    """
    try:
        from feagi.core.state_manager import FeagiStateManager

        state_manager = FeagiStateManager.instance()
        return state_manager.is_debug_bdu_enabled()
    except Exception:
        return False


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


def linearize_position(
    position: Position, dimensions: Position
) -> LinearPosition:
    """Convert a 3D position to a linearized 1D index.

    Args:
    position: 3D position (x, y, z)
    dimensions: Dimensions of the cortical area (width, height, depth)

    Returns:
    Linearized position index
    """
    x, y, z = position
    width, height, depth = dimensions
    return x + (y * width) + (z * width * height)


def delinearize_position(
    linear_pos: LinearPosition, dimensions: Position
) -> Position:
    """Convert a linearized 1D index back to a 3D position.

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
    """Preprocess algebraic expressions for evaluation.

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
    """Evaluate an algebraic expression with the given x, y, z values.

    Args:
    expr: Expression string or integer value
    x, y, z: Variable values

    Returns:
    Evaluated integer result
    """
    if isinstance(expr, (int, float)):
        return int(expr)

    try:
        result = sympify(preprocess_expression(expr)).subs(
            {"x": x, "y": y, "z": z}
        )
        return int(result)
    except Exception as e:
        logger.error(f"Error evaluating expression '{expr}': {e}")
        return 0


def neighbor_finder(position, neighbor_range=1, include_self=False):
    """Find all neighboring positions within a given range.

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
    """Find candidate neurons in the destination area for synaptic connections.

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
    debug_bdu = _is_debug_bdu_enabled()

    if debug_bdu:
        logger.info(
            f"[BDU DEBUG] ===== SYNAPTOGENESIS: {src_area_id} -> {dst_area_id} ====="
        )
        logger.info(f"[BDU DEBUG] Source neuron: {src_neuron_id}")
        logger.info(f"[BDU DEBUG] Morphology: {morphology}")
        logger.info(f"[BDU DEBUG] Source subregion: {src_subregion}")

    # Get source neuron position
    src_neuron_pos = connectome_manager.get_neuron_position(src_neuron_id)
    if not src_neuron_pos:
        logger.error(f"Cannot find position for source neuron {src_neuron_id}")
        return []

    # Get position tuple (x, y, z)
    src_voxel = src_neuron_pos

    if debug_bdu:
        logger.info(f"[BDU DEBUG] Source neuron position: {src_voxel}")

    # Determine morphology ID and parameters
    neuron_morphology = morphology_id_overwrite or morphology["morphology_id"]
    morphology_scalar = morphology["morphology_scalar"]
    psc_multiplier = morphology["postSynapticCurrent_multiplier"]

    if debug_bdu:
        logger.info(f"[BDU DEBUG] Using morphology: {neuron_morphology}")
        logger.info(f"[BDU DEBUG] Morphology scalar: {morphology_scalar}")
        logger.info(f"[BDU DEBUG] PSC multiplier: {psc_multiplier}")

    # Get source area properties
    try:
        src_area = connectome_manager.get_cortical_area(src_area_id)
    except Exception as e:
        logger.error(f"Source area {src_area_id} not found: {e}")
        return []
    psc_base = src_area.properties.get("postsynaptic_current", 1.0)
    post_synaptic_current = psc_multiplier * psc_base

    # Storage for candidate positions and neurons
    raw_candidate_positions = set()
    candidate_neuron_list = []

    # Get morphology type - for function morphologies, we know the type directly
    # This follows the legacy pattern where function morphologies are handled directly
    if neuron_morphology in [e.value for e in MorphologyFunction]:
        morphology_type = RuleType.FUNCTIONS.value
        morphologies_registry = None  # Not needed for function morphologies
    elif hasattr(connectome_manager, "get_morphologies_registry"):
        morphologies_registry = connectome_manager.get_morphologies_registry()
        if neuron_morphology not in morphologies_registry:
            logger.error(
                f"Morphology {neuron_morphology} not found in registry"
            )
            return []
        morphology_type = morphologies_registry[neuron_morphology]["type"]
    else:
        logger.error(
            f"Unknown morphology {neuron_morphology} and no registry available"
        )
        return []

    try:
        # Process based on morphology type
        if morphology_type == RuleType.VECTORS.value:
            for vector in morphologies_registry[neuron_morphology][
                "parameters"
            ]["vectors"]:
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
            for pattern in morphologies_registry[neuron_morphology][
                "parameters"
            ]["patterns"]:
                # Get destination area dimensions
                try:
                    dst_area = connectome_manager.get_cortical_area(
                        dst_area_id
                    )
                except Exception as e:
                    logger.error(
                        f"Destination area {dst_area_id} not found: {e}"
                    )
                    continue
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
                    dst_area_id=dst_area_id,
                    connectome_manager=connectome_manager,
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

            elif (
                neuron_morphology == MorphologyFunction.BLOCK_CONNECTION.value
            ):
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
                if debug_bdu:
                    logger.info("[BDU DEBUG] Processing PROJECTOR morphology")
                positions = syn_projector(
                    src_area_id=src_area_id,
                    dst_area_id=dst_area_id,
                    src_neuron_id=src_neuron_id,
                    src_subregion=src_subregion,
                    connectome_manager=connectome_manager,
                )
                if debug_bdu:
                    logger.info(
                        f"[BDU DEBUG] PROJECTOR returned {len(positions)} candidate positions: {positions}"
                    )
                for pos in positions:
                    raw_candidate_positions.add(pos)

            elif neuron_morphology == MorphologyFunction.PROJECTOR_XY.value:
                positions = syn_projector(
                    src_area_id=src_area_id,
                    dst_area_id=dst_area_id,
                    src_neuron_id=src_neuron_id,
                    src_subregion=src_subregion,
                    connectome_manager=connectome_manager,
                    transpose=("y", "x", "z"),
                )
                for pos in positions:
                    raw_candidate_positions.add(pos)

            elif neuron_morphology == MorphologyFunction.PROJECTOR_XZ.value:
                positions = syn_projector(
                    src_area_id=src_area_id,
                    dst_area_id=dst_area_id,
                    src_neuron_id=src_neuron_id,
                    src_subregion=src_subregion,
                    connectome_manager=connectome_manager,
                    transpose=("z", "y", "x"),
                )
                for pos in positions:
                    raw_candidate_positions.add(pos)

            elif neuron_morphology == MorphologyFunction.PROJECTOR_YZ.value:
                positions = syn_projector(
                    src_area_id=src_area_id,
                    dst_area_id=dst_area_id,
                    src_neuron_id=src_neuron_id,
                    src_subregion=src_subregion,
                    connectome_manager=connectome_manager,
                    transpose=("x", "z", "y"),
                )
                for pos in positions:
                    raw_candidate_positions.add(pos)

            elif (
                neuron_morphology
                == MorphologyFunction.PROJECT_FROM_END_X.value
            ):
                # Only project if neuron is in the last layer of x dimension
                src_dimensions = src_area.dimensions
                if src_voxel[0] == src_dimensions[0] - 1:
                    positions = syn_projector(
                        src_area_id=src_area_id,
                        dst_area_id=dst_area_id,
                        src_neuron_id=src_neuron_id,
                        src_subregion=src_subregion,
                        connectome_manager=connectome_manager,
                        project_last_layer_of="x",
                    )
                    for pos in positions:
                        raw_candidate_positions.add(pos)

            elif (
                neuron_morphology
                == MorphologyFunction.PROJECT_FROM_END_Y.value
            ):
                # Only project if neuron is in the last layer of y dimension
                src_dimensions = src_area.dimensions
                if src_voxel[1] == src_dimensions[1] - 1:
                    positions = syn_projector(
                        src_area_id=src_area_id,
                        dst_area_id=dst_area_id,
                        src_neuron_id=src_neuron_id,
                        src_subregion=src_subregion,
                        connectome_manager=connectome_manager,
                        project_last_layer_of="y",
                    )
                    for pos in positions:
                        raw_candidate_positions.add(pos)

            elif (
                neuron_morphology
                == MorphologyFunction.PROJECT_FROM_END_Z.value
            ):
                # Only project if neuron is in the last layer of z dimension
                src_dimensions = src_area.dimensions
                if src_voxel[2] == src_dimensions[2] - 1:
                    positions = syn_projector(
                        src_area_id=src_area_id,
                        dst_area_id=dst_area_id,
                        src_neuron_id=src_neuron_id,
                        src_subregion=src_subregion,
                        connectome_manager=connectome_manager,
                        project_last_layer_of="z",
                    )
                    for pos in positions:
                        raw_candidate_positions.add(pos)

            # Handle special memory morphology that doesn't produce voxel positions
            elif neuron_morphology == MorphologyFunction.MEMORY.value:
                syn_memory(src_area_id, dst_area_id, memory_register)
                # No positions are added for memory function

            else:
                logger.warning(
                    f"Unsupported morphology function: {neuron_morphology}"
                )

        else:
            logger.warning(f"Unsupported morphology type: {morphology_type}")

        # Legacy-style batch lookup: single O(N) pass instead of O(P×N) individual lookups
        if debug_bdu:
            logger.info(
                f"[BDU DEBUG] Processing {len(raw_candidate_positions)} candidate positions"
            )
            logger.info(
                f"[BDU DEBUG] Candidate positions: {sorted(list(raw_candidate_positions))}"
            )

        # Use legacy batch approach for performance (like voxels.voxel_list_to_neuron_list)
        candidate_neuron_list = (
            connectome_manager.batch_voxel_to_neuron_lookup(
                cortical_id=dst_area_id,
                candidate_positions=raw_candidate_positions,
                post_synaptic_current=post_synaptic_current,
            )
        )

        if debug_bdu:
            logger.info(
                f"[BDU DEBUG] Final candidate neurons: {len(candidate_neuron_list)} found"
            )
            if candidate_neuron_list:
                neuron_ids = [
                    neuron_id for neuron_id, weight in candidate_neuron_list
                ]
                weights = [
                    weight for neuron_id, weight in candidate_neuron_list
                ]
                logger.info(f"[BDU DEBUG] Candidate neuron IDs: {neuron_ids}")
                logger.info(f"[BDU DEBUG] Candidate weights: {weights}")
            logger.info(
                f"[BDU DEBUG] ===== END SYNAPTOGENESIS: {src_area_id} -> {dst_area_id} ====="
            )

        logger.debug(f"Found {len(candidate_neuron_list)} destination neurons")
        return candidate_neuron_list

    except Exception as e:
        logger.error(f"Error in neighbor_finder: {str(e)}")
        logger.exception("Exception details:")
        return []
