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
Function-based synapse rules for the BDU.

This module contains the actual syn_* functions that are called dynamically
for function-based morphologies. These functions are referenced by name
from the morphology definitions and called directly.

All functions in this module should start with 'syn_' prefix.
"""

from random import randrange
from typing import Dict, List, Optional, Set, Tuple

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)

# Type aliases for improved code readability and Rust compatibility
AreaId = int
NeuronId = int
Position = Tuple[int, int, int]
BoundingBox = Tuple[
    Tuple[int, int, int], Tuple[int, int, int]
]  # ((min_x, min_y, min_z), (max_x, max_y, max_z))


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
        src_area_id not in connectome_manager.cortical_areas
        or dst_area_id not in connectome_manager.cortical_areas
    ):
        logger.error("Source or destination area not found")
        return set()

    src_area = connectome_manager.cortical_areas[src_area_id]
    dst_area = connectome_manager.cortical_areas[dst_area_id]

    src_dims = src_area.dimensions
    dst_dims = dst_area.dimensions

    # Calculate the expansion ratio in each dimension
    ratios = [
        dst_dims[i] / src_dims[i] if src_dims[i] > 0 else 1.0 for i in range(3)
    ]

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
    """Implement the reducer rule for x-dimension.

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
    dst_area = connectome_manager.get_cortical_area(dst_area_id)
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

    src_neuron_block_index_x = src_neuron_pos[0]  # x-coordinate

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
    """Select a random position in the destination area.

    Args:
        dst_area_id: Destination area ID
        connectome_manager: Reference to the ConnectomeManager

    Returns:
        Random position in the destination area
    """
    dst_area = connectome_manager.get_cortical_area(dst_area_id)
    dst_dims = dst_area.dimensions

    random_location = (
        randrange(0, dst_dims[0]),
        randrange(0, dst_dims[1]),
        randrange(0, dst_dims[2]),
    )

    return random_location


def syn_lateral_pairs_x(
    neuron_id: NeuronId,
    area_id: AreaId,
    src_subregion: BoundingBox,
    connectome_manager,
) -> Optional[Position]:
    """Create lateral connections between neighboring neurons on the x-axis.

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

    neuron_block_index_x = neuron_pos[0]  # x-coordinate
    neuron_block_index_y = neuron_pos[1]  # y-coordinate
    neuron_block_index_z = neuron_pos[2]  # z-coordinate

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
    """Map blocks of neurons from source to destination with scaling.

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
    dst_area = connectome_manager.get_cortical_area(dst_area_id)
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

    neuron_block_index_x = neuron_pos[0]  # x-coordinate
    neuron_block_index_y = neuron_pos[1]  # y-coordinate
    neuron_block_index_z = neuron_pos[2]  # z-coordinate

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
    """Project neurons from source to destination while maintaining topology.

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
    debug_bdu = _is_debug_bdu_enabled()

    if debug_bdu:
        logger.info(
            f"[BDU DEBUG] syn_projector: {src_area_id} -> {dst_area_id}, neuron {src_neuron_id}"
        )
        logger.info(
            f"[BDU DEBUG] transpose: {transpose}, project_last_layer_of: {project_last_layer_of}"
        )

    src_area = connectome_manager.get_cortical_area(src_area_id)
    src_dimensions = src_area.dimensions

    dst_area = connectome_manager.get_cortical_area(dst_area_id)
    dst_dimensions = dst_area.dimensions

    if debug_bdu:
        logger.info(f"[BDU DEBUG] Source dimensions: {src_dimensions}")
        logger.info(f"[BDU DEBUG] Destination dimensions: {dst_dimensions}")
        logger.info(f"[BDU DEBUG] Source subregion: {src_subregion}")

    # These will be updated based on the transpose and project_last_layer parameters
    src_shape = [0, 0, 0]
    dst_shape = list(dst_dimensions)

    # Get the neuron's position
    neuron_pos = connectome_manager.get_neuron_position(src_neuron_id)
    if not neuron_pos:
        if debug_bdu:
            logger.warning(
                f"[BDU DEBUG] Could not find position for neuron {src_neuron_id}"
            )
        return []

    # Default neuron location (x, y, z)
    neuron_location = list(neuron_pos)

    if debug_bdu:
        logger.info(f"[BDU DEBUG] Neuron location: {neuron_location}")

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
                target_vox = int(
                    (neuron_location[i] - src_subregion[0][i]) / ratio
                )

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
                    if int(vox / ratio) == (
                        neuron_location[i] - src_subregion[0][i]
                    ):
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
    if debug_bdu:
        logger.info("[BDU DEBUG] Destination voxel dictionary:")
        logger.info(
            f"[BDU DEBUG]   X candidates: {sorted(list(dst_vox_dict[0]))}"
        )
        logger.info(
            f"[BDU DEBUG]   Y candidates: {sorted(list(dst_vox_dict[1]))}"
        )
        logger.info(
            f"[BDU DEBUG]   Z candidates: {sorted(list(dst_vox_dict[2]))}"
        )

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
                    elif debug_bdu:
                        logger.warning(
                            f"[BDU DEBUG] Position ({x}, {y}, {z}) out of bounds for destination {dst_dimensions}"
                        )

    if debug_bdu:
        logger.info(
            f"[BDU DEBUG] syn_projector final candidates: {candidate_list}"
        )

    return candidate_list


def syn_memory(
    src_area_id: AreaId,
    dst_area_id: AreaId,
    memory_register: Dict[AreaId, Set[AreaId]],
) -> None:
    """Register source-destination area relationship in memory register.

    Args:
        src_area_id: Source area ID
        dst_area_id: Destination area ID
        memory_register: Memory register dictionary to update
    """
    if dst_area_id not in memory_register:
        memory_register[dst_area_id] = set()

    memory_register[dst_area_id].add(src_area_id)


def syn_last_to_first(
    src_area_id: AreaId, connectome_manager
) -> List[Position]:
    """Generate a connection from the last neuron to the first.

    Args:
        src_area_id: Source area ID
        connectome_manager: Reference to the ConnectomeManager

    Returns:
        Position of the first voxel [0, 0, 0]
    """
    return [(0, 0, 0)]
