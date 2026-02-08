"""
XYZP Data Decoders

High-performance decoders for FEAGI's Structure-of-Arrays (SoA) XYZP neuron voxel format.

Standard XYZP Format:
{
    "cortical_id": {
        "x": [x1, x2, ...],  # X coordinates
        "y": [y1, y2, ...],  # Y coordinates  
        "z": [z1, z2, ...],  # Z coordinates (channel/depth)
        "p": [p1, p2, ...],  # Potential/activation values
    }
}

Copyright 2016-2025 Neuraville Inc. All Rights Reserved.
"""

from typing import Dict, List, Tuple, Optional
import base64
import binascii
import logging

logger = logging.getLogger(__name__)


def _parse_cortical_unit_index_from_b64(cortical_id: str) -> Optional[int]:
    """
    Parse cortical unit index (group) from CorticalID.

    Accepts either base64-encoded ID (preferred) or raw 8-byte string
    (legacy Rust SDK used String::from_utf8_lossy(cortical_id.as_bytes())).

    Args:
        cortical_id: Base64-encoded cortical ID, or 8-character string (bytes as latin-1)

    Returns:
        Unit index (byte 7) if parse succeeds, otherwise None.
    """
    try:
        raw = base64.b64decode(cortical_id, validate=True)
    except (binascii.Error, ValueError):
        raw = None
    if raw is not None and len(raw) == 8:
        return int(raw[7])
    # Fallback: raw 8-byte string from legacy Rust JSON key (e.g. from_utf8_lossy)
    if len(cortical_id) == 8:
        try:
            b = cortical_id.encode("latin-1")
            return int(b[7])
        except (IndexError, ValueError):
            pass
    return None


def decode_motor_xyzp(
    xyzp_data: dict,
    cortical_ids: Optional[List[str]] = None,
    include_groups: bool = False,
) -> Dict[str, float]:
    """
    Decode motor output from XYZP SoA format to motor index → power mapping.
    
    Motor encoding in XYZP:
    - X coordinate = motor index (0, 1, 2, ...)
    - P value = motor power (-100.0 to +100.0)
    
    Args:
        xyzp_data: Raw XYZP SoA data from FEAGI
        cortical_ids: Optional list of cortical IDs to decode (None = all motor areas)
        include_groups: If True, emit keys as "{group}:{channel}" when possible
    
    Returns:
        Dict mapping motor index (as string) → power value, optionally grouped
        
    Example:
        >>> xyzp = {"omot\\x04\\x00\\x00\\x00": {"x": [0, 1], "y": [0, 0], "z": [0, 0], "p": [50.0, -30.0]}}
        >>> decode_motor_xyzp(xyzp)
        {'0': 50.0, '1': -30.0}
    """
    motors: Dict[str, float] = {}
    cortical_group_map: Dict[str, Optional[int]] = {}
    groups_found = set()

    for cortical_id in xyzp_data:
        group_id = _parse_cortical_unit_index_from_b64(cortical_id)
        cortical_group_map[cortical_id] = group_id
        if group_id is not None:
            groups_found.add(group_id)

    use_group_keys = include_groups or len(groups_found) > 1
    
    for cortical_id, neuron_data in xyzp_data.items():
        # Filter by cortical_ids if provided
        if cortical_ids and cortical_id not in cortical_ids:
            continue
        
        try:
            x_coords = neuron_data.get('x', [])
            p_values = neuron_data.get('p', [])
            
            if len(x_coords) != len(p_values):
                logger.warning(f"Mismatched x/p lengths in {cortical_id}: {len(x_coords)} vs {len(p_values)}")
                continue
            
            # Map X coordinate (motor index) to P value (motor power)
            # Use string keys for controller compatibility
            group_id = cortical_group_map.get(cortical_id)
            for motor_idx, power in zip(x_coords, p_values):
                channel_key = str(int(motor_idx))
                if use_group_keys and group_id is not None:
                    channel_key = f"{group_id}:{channel_key}"
                motors[channel_key] = float(power)
        
        except (KeyError, ValueError, TypeError) as e:
            logger.error(f"Error decoding motor data from {cortical_id}: {e}")
            continue
    
    return motors


def decode_sensor_xyzp_to_grid(xyzp_data: dict, cortical_id: str, 
                                grid_shape: Tuple[int, int, int]) -> Dict[int, List[float]]:
    """
    Decode sensory input from XYZP SoA to 3D grid format (channel → 2D arrays).
    
    Sensor encoding in XYZP:
    - X, Y = spatial position in 2D sensor plane
    - Z = channel index (e.g., RGB channels)
    - P = sensor value (e.g., pixel intensity)
    
    Args:
        xyzp_data: Raw XYZP SoA data
        cortical_id: Cortical area ID to decode
        grid_shape: (width, height, channels) of the sensor grid
    
    Returns:
        Dict mapping channel_idx → flat list of values (row-major order)
    """
    width, height, channels = grid_shape
    
    # Initialize empty grids for each channel
    grids = {ch: [0.0] * (width * height) for ch in range(channels)}
    
    if cortical_id not in xyzp_data:
        return grids
    
    try:
        neuron_data = xyzp_data[cortical_id]
        x_coords = neuron_data.get('x', [])
        y_coords = neuron_data.get('y', [])
        z_coords = neuron_data.get('z', [])
        p_values = neuron_data.get('p', [])
        
        if not (len(x_coords) == len(y_coords) == len(z_coords) == len(p_values)):
            logger.warning(f"Mismatched coordinate lengths in {cortical_id}")
            return grids
        
        # Map (x, y, z, p) to grid[z][y * width + x] = p
        for x, y, z, p in zip(x_coords, y_coords, z_coords, p_values):
            if 0 <= x < width and 0 <= y < height and 0 <= z < channels:
                flat_idx = int(y) * width + int(x)
                grids[int(z)][flat_idx] = float(p)
            else:
                logger.debug(f"Out-of-bounds neuron: ({x},{y},{z}) for shape {grid_shape}")
        
        return grids
    
    except (KeyError, ValueError, TypeError) as e:
        logger.error(f"Error decoding sensor grid from {cortical_id}: {e}")
        return grids


def decode_1d_array_xyzp(xyzp_data: dict, cortical_id: str, length: int) -> List[float]:
    """
    Decode 1D array (e.g., proximity sensors) from XYZP format.
    
    1D encoding in XYZP:
    - X = index in 1D array
    - P = sensor value
    
    Args:
        xyzp_data: Raw XYZP SoA data
        cortical_id: Cortical area ID to decode
        length: Expected length of 1D array
    
    Returns:
        List of values (0.0 if no neuron at that index)
    """
    array = [0.0] * length
    
    if cortical_id not in xyzp_data:
        return array
    
    try:
        neuron_data = xyzp_data[cortical_id]
        x_coords = neuron_data.get('x', [])
        p_values = neuron_data.get('p', [])
        
        if len(x_coords) != len(p_values):
            logger.warning(f"Mismatched x/p lengths in {cortical_id}")
            return array
        
        for x, p in zip(x_coords, p_values):
            if 0 <= x < length:
                array[int(x)] = float(p)
        
        return array
    
    except (KeyError, ValueError, TypeError) as e:
        logger.error(f"Error decoding 1D array from {cortical_id}: {e}")
        return array

