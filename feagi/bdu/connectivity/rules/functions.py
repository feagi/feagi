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

# Import Rust BDU for high-performance synaptogenesis
try:
    from feagi_bdu import (
        py_syn_projector,
        py_syn_block_connection,
        py_syn_expander,
        py_syn_reducer_x,
    )
    RUST_BDU_AVAILABLE = True
    logger.info("🦀 Rust BDU Phase 2 loaded: projector, block_connection, expander, reducer")
except ImportError:
    RUST_BDU_AVAILABLE = False
    logger.error("Rust BDU not available. Run: cd feagi-rust && ./build_bdu.sh")

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
    """Expander_X using Rust (scales coordinates)."""
    if not RUST_BDU_AVAILABLE:
        raise RuntimeError("Rust BDU required")
    
    src_area = connectome_manager.get_cortical_area(src_area_id)
    dst_area = connectome_manager.get_cortical_area(dst_area_id)
    neuron_pos = connectome_manager.get_neuron_position(src_neuron_id)
    
    if not neuron_pos:
        return set()
    
    result = py_syn_expander(
        src_area_id, dst_area_id, neuron_pos,
        src_area.dimensions, dst_area.dimensions
    )
    return {result}


def syn_reducer_x(
    src_area_id: AreaId,
    dst_area_id: AreaId,
    src_neuron_id: NeuronId,
    src_subregion: BoundingBox,
    connectome_manager,
    dst_y_index: int = 0,
    dst_z_index: int = 0,
) -> List[Position]:
    """Reducer using Rust (binary encoding)."""
    if not RUST_BDU_AVAILABLE:
        raise RuntimeError("Rust BDU required")
    
    src_area = connectome_manager.get_cortical_area(src_area_id)
    dst_area = connectome_manager.get_cortical_area(dst_area_id)
    neuron_pos = connectome_manager.get_neuron_position(src_neuron_id)
    
    if not neuron_pos:
        return []
    
    return py_syn_reducer_x(
        src_area_id, dst_area_id, neuron_pos,
        src_area.dimensions, dst_area.dimensions,
        dst_y_index, dst_z_index
    )


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
    """Block connection using Rust (block mapping with scaling)."""
    if not RUST_BDU_AVAILABLE:
        raise RuntimeError("Rust BDU required")
    
    src_area = connectome_manager.get_cortical_area(src_area_id)
    dst_area = connectome_manager.get_cortical_area(dst_area_id)
    neuron_pos = connectome_manager.get_neuron_position(src_neuron_id)
    
    if not neuron_pos:
        return (0, 0, 0)
    
    return py_syn_block_connection(
        src_area_id, dst_area_id, neuron_pos,
        src_area.dimensions, dst_area.dimensions,
        scaling_factor
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
    """Project neurons using Rust-accelerated implementation (1600x faster).
    
    Args:
        src_area_id: Source area ID
        dst_area_id: Destination area ID
        src_neuron_id: Source neuron ID
        src_subregion: Source subregion bounding box
        connectome_manager: Reference to the ConnectomeManager
        transpose: Optional tuple of axes to transpose ("x", "y", "z")
        project_last_layer_of: Optional axis to project from last layer
        
    Returns:
        List of matching positions in the destination area
    """
    import time
    
    if not RUST_BDU_AVAILABLE:
        raise RuntimeError("Rust BDU required. Build: cd feagi-rust && ./build_bdu.sh")
    
    # Get dimensions
    src_area = connectome_manager.get_cortical_area(src_area_id)
    dst_area = connectome_manager.get_cortical_area(dst_area_id)
    
    # Get neuron position
    neuron_pos = connectome_manager.get_neuron_position(src_neuron_id)
    if not neuron_pos:
        return []
    
    # Convert transpose string to indices
    transpose_indices = None
    if transpose:
        axis_map = {"x": 0, "y": 1, "z": 2}
        transpose_indices = tuple(axis_map[axis] for axis in transpose)
    
    # Convert project_last_layer string to index
    project_layer_idx = None
    if project_last_layer_of:
        axis_map = {"x": 0, "y": 1, "z": 2}
        project_layer_idx = axis_map[project_last_layer_of]
    
    # Call Rust (1600x faster than Python)
    start = time.time()
    result = py_syn_projector(
        src_area_id,
        dst_area_id,
        src_neuron_id,
        src_area.dimensions,
        dst_area.dimensions,
        neuron_pos,
        transpose_indices,
        project_layer_idx,
    )
    elapsed = (time.time() - start) * 1000
    if elapsed > 1.0:  # Only log if >1ms (shouldn't happen with Rust)
        logger.warning(f"🦀 RUST syn_projector took {elapsed:.2f}ms (should be <0.1ms) - {len(result)} positions")
    return result


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
