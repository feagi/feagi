"""
XYZP Data Decoders

High-performance decoders for FEAGI's Structure-of-Arrays (SoA) XYZP
neuron voxel format.

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
_last_decode_log: Dict[str, float] = {}


def _parse_cortical_unit_index_from_b64(cortical_id: str) -> Optional[int]:
    """
    Parse cortical unit index (group) from CorticalID.

    Accepts either base64-encoded ID (preferred) or raw 8-byte string
    (legacy Rust SDK used String::from_utf8_lossy(cortical_id.as_bytes())).

    Args:
        cortical_id: Base64-encoded cortical ID, or 8-character string
            (bytes as latin-1)

    Returns:
        Unit index (byte 7) if parse succeeds, otherwise None.
    """
    try:
        raw = base64.b64decode(cortical_id, validate=True)
    except (binascii.Error, ValueError):
        raw = None
    if raw is not None and len(raw) == 8:
        return int(raw[7])
    # Fallback: raw 8-byte string from legacy Rust JSON key
    # (e.g. from_utf8_lossy)
    if len(cortical_id) == 8:
        try:
            b = cortical_id.encode("latin-1")
            return int(b[7])
        except (IndexError, ValueError):
            pass
    return None


def _parse_cortical_id_bytes(cortical_id: str) -> Optional[bytes]:
    """
    Decode cortical ID to raw 8-byte ID.

    Accepts either:
    - Base64 cortical ID (preferred wire format), or
    - Legacy raw 8-byte string where bytes were converted via latin-1.
    """
    try:
        raw = base64.b64decode(cortical_id, validate=True)
    except (binascii.Error, ValueError):
        raw = None
    if raw is not None and len(raw) == 8:
        return raw

    # Fallback for legacy raw 8-byte string keys.
    if len(cortical_id) == 8:
        try:
            raw_legacy = cortical_id.encode("latin-1")
            if len(raw_legacy) == 8:
                return raw_legacy
        except UnicodeEncodeError:
            pass

    return None


def _decode_signed_percentage_linear(
    z_positive: List[int],
    z_negative: List[int],
    z_depth: int,
) -> float:
    """Decode signed percentage (linear) from positive/negative z bins."""
    if z_depth <= 0:
        return 0.0

    z_span = max(1, z_depth - 1)

    if z_positive:
        positive = 1.0 - (sum(z_positive) / (z_span * len(z_positive)))
    else:
        positive = 0.0

    if z_negative:
        negative = 1.0 - (sum(z_negative) / (z_span * len(z_negative)))
    else:
        negative = 0.0

    return max(-1.0, min(1.0, positive - negative))


def _decode_signed_percentage_fractional(
    z_positive: List[int],
    z_negative: List[int],
) -> float:
    """Decode signed percentage (fractional/exponential) from z bins."""
    positive = sum(0.5 ** z for z in z_positive)
    negative = sum(0.5 ** z for z in z_negative)
    return max(-1.0, min(1.0, positive - negative))


def _decode_unsigned_percentage_linear(z_values: List[int], z_depth: int) -> float:
    """Decode unsigned percentage (linear) from z bins."""
    if z_depth <= 0 or not z_values:
        return 0.0
    z_span = max(1, z_depth - 1)
    return max(0.0, min(1.0, 1.0 - (sum(z_values) / (z_span * len(z_values)))))


def _decode_unsigned_percentage_fractional(z_values: List[int]) -> float:
    """Decode unsigned percentage (fractional/exponential) from z bins."""
    if not z_values:
        return 0.0
    return max(0.0, min(1.0, sum(0.5 ** z for z in z_values)))


def _normalize_unsigned_to_signed(value_0_1: float) -> float:
    """Map [0,1] percentage to signed normalized range [-1,1]."""
    return max(-1.0, min(1.0, (value_0_1 * 2.0) - 1.0))


def _resolve_motor_linear_depth(unit_ref: bytes, observed_depth: int) -> int:
    """
    Resolve stable Z depth for linear motor decode.

    Motor templates use 10 Z bins by default. When only a subset of bins fire in
    a single packet, deriving depth from observed max Z can collapse scaling.
    Keep decode depth stable at least at template default.
    """
    if unit_ref in (b"pse", b"mot"):
        return max(10, observed_depth)
    return max(1, observed_depth)


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
        cortical_ids: Optional list of cortical IDs to decode
            (None = all motor areas)
        include_groups: If True, emit keys as "{group}:{channel}" when possible
    
    Returns:
        Dict mapping motor index (as string) → power value, optionally grouped
        
    Example:
        >>> xyzp = {
        ...   "omot\\x04\\x00\\x00\\x00": {
        ...     "x": [0, 1], "y": [0, 0], "z": [0, 0], "p": [50.0, -30.0]
        ...   }
        ... }
        >>> decode_motor_xyzp(xyzp)
        {'0': 0.5, '1': -0.3}
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
            x_coords = neuron_data.get("x", [])
            y_coords = neuron_data.get("y", [])
            z_coords = neuron_data.get("z", [])
            p_values = neuron_data.get("p", [])

            lengths = (
                len(x_coords),
                len(y_coords),
                len(z_coords),
                len(p_values),
            )
            if len(set(lengths)) != 1:
                logger.warning(
                    "Mismatched x/y/z/p lengths in %s",
                    cortical_id,
                )
                continue

            group_id = cortical_group_map.get(cortical_id)

            raw_cid = _parse_cortical_id_bytes(cortical_id)
            if raw_cid is None:
                # Robust fallback: decode by XYZ lanes/depth instead of p/100,
                # so incremental magnitude remains z-dependent even when CID parsing fails.
                positive_by_channel: Dict[int, List[int]] = {}
                negative_by_channel: Dict[int, List[int]] = {}
                max_z_seen = 0
                for x, y, z, p in zip(x_coords, y_coords, z_coords, p_values):
                    # Do not hard-gate on y==0 in Python path; some streams may
                    # carry non-zero y for 1D motor channels.
                    if float(p) == 0.0:
                        continue
                    x_int = int(x)
                    z_int = int(z)
                    max_z_seen = max(max_z_seen, z_int)
                    channel_idx = x_int // 2
                    if x_int % 2 == 0:
                        positive_by_channel.setdefault(channel_idx, []).append(z_int)
                    else:
                        negative_by_channel.setdefault(channel_idx, []).append(z_int)

                if positive_by_channel or negative_by_channel:
                    z_depth = max(10, max_z_seen + 1)
                    channel_ids = set(positive_by_channel).union(set(negative_by_channel))
                    for channel_idx in channel_ids:
                        z_pos = positive_by_channel.get(channel_idx, [])
                        z_neg = negative_by_channel.get(channel_idx, [])
                        if positioning_fractional:
                            pos = _decode_unsigned_percentage_fractional(z_pos)
                            neg = _decode_unsigned_percentage_fractional(z_neg)
                        else:
                            pos = _decode_unsigned_percentage_linear(z_pos, z_depth)
                            neg = _decode_unsigned_percentage_linear(z_neg, z_depth)
                        decoded = max(-1.0, min(1.0, pos - neg))
                        channel_key = str(channel_idx)
                        if use_group_keys and group_id is not None:
                            channel_key = f"{group_id}:{channel_key}:incremental"
                        else:
                            channel_key = f"{channel_key}:incremental"
                        motors[channel_key] = decoded
                        print(
                            f"[XYZP-FALLBACK] cid={cortical_id!r} ch={channel_idx} "
                            f"pos={pos:.4f} neg={neg:.4f} decoded={decoded:.4f} "
                            f"x={x_coords[:8]} z={z_coords[:8]}",
                            flush=True,
                        )
                    continue

                # Last resort when no usable XYZ lanes are present.
                for motor_idx, power in zip(x_coords, p_values):
                    channel_key = str(int(motor_idx))
                    if use_group_keys and group_id is not None:
                        channel_key = f"{group_id}:{channel_key}:incremental"
                    else:
                        channel_key = f"{channel_key}:incremental"
                    motors[channel_key] = float(power) / 100.0
                continue

            unit_ref = raw_cid[1:4]
            data_type_flag = raw_cid[4] | (raw_cid[5] << 8)
            variant = data_type_flag & 0xFF
            frame_incremental = ((data_type_flag >> 8) & 0x01) == 1
            positioning_fractional = ((data_type_flag >> 9) & 0x01) == 1
            command_mode = "incremental" if frame_incremental else "absolute"

            # PositionalServo/RotaryMotor lane decode:
            # even X -> forward/positive lane, odd X -> backward/negative lane.
            # For signed percentage (variant=5), decode as signed [-1..1].
            # For incremental unsigned percentage (variant=1 + incremental),
            # decode each lane as unsigned [0..1] then apply forward-backward.
            should_decode_lanes = (
                unit_ref in (b"pse", b"mot")
                and (
                    variant == 5
                    or (variant == 1 and frame_incremental)
                )
            )
            if should_decode_lanes:
                positive_by_channel: Dict[int, List[int]] = {}
                negative_by_channel: Dict[int, List[int]] = {}
                max_z_seen = 0

                for x, y, z, p in zip(x_coords, y_coords, z_coords, p_values):
                    # Do not hard-gate on y==0 in Python path; some streams may
                    # carry non-zero y for 1D motor channels.
                    if float(p) == 0.0:
                        continue
                    x_int = int(x)
                    z_int = int(z)
                    max_z_seen = max(max_z_seen, z_int)
                    channel_idx = x_int // 2
                    if x_int % 2 == 0:
                        positive_by_channel.setdefault(
                            channel_idx,
                            [],
                        ).append(z_int)
                    else:
                        negative_by_channel.setdefault(
                            channel_idx,
                            [],
                        ).append(z_int)

                z_depth = _resolve_motor_linear_depth(
                    unit_ref,
                    max_z_seen + 1,
                )
                channel_ids = set(positive_by_channel).union(
                    set(negative_by_channel),
                )
                for channel_idx in channel_ids:
                    z_pos = positive_by_channel.get(channel_idx, [])
                    z_neg = negative_by_channel.get(channel_idx, [])
                    if variant == 5:
                        if positioning_fractional:
                            decoded = _decode_signed_percentage_fractional(
                                z_pos,
                                z_neg,
                            )
                        else:
                            decoded = _decode_signed_percentage_linear(
                                z_pos,
                                z_neg,
                                z_depth,
                            )
                    else:
                        if positioning_fractional:
                            positive = _decode_unsigned_percentage_fractional(
                                z_pos
                            )
                            negative = _decode_unsigned_percentage_fractional(
                                z_neg
                            )
                        else:
                            positive = _decode_unsigned_percentage_linear(
                                z_pos,
                                z_depth,
                            )
                            negative = _decode_unsigned_percentage_linear(
                                z_neg,
                                z_depth,
                            )
                        decoded = max(-1.0, min(1.0, positive - negative))
                    channel_key = str(channel_idx)
                    if use_group_keys and group_id is not None:
                        channel_key = (
                            f"{group_id}:{channel_key}:{command_mode}"
                        )
                    else:
                        channel_key = f"{channel_key}:{command_mode}"
                    motors[channel_key] = decoded
                    prev = _last_decode_log.get(channel_key)
                    if prev is None or abs(prev - decoded) > 1e-6:
                        logger.info(
                            "[XYZP-DECODE] cid=%s mode=%s lane=paired channel=%d decoded=%.6f x_pairs=%s",
                            cortical_id,
                            command_mode,
                            channel_idx,
                            decoded,
                            [x for x in x_coords[:12]],
                        )
                        _last_decode_log[channel_key] = decoded
                continue

            # Unsigned percentage absolute decode (single lane per channel):
            # X -> channel index, Z -> magnitude, Y must be 0.
            # This path is required for PositionalServo absolute areas.
            if unit_ref in (b"pse", b"mot") and variant == 1 and not frame_incremental:
                z_by_channel: Dict[int, List[int]] = {}
                max_z_seen = 0
                for x, y, z, p in zip(x_coords, y_coords, z_coords, p_values):
                    # Do not hard-gate on y==0 in Python path; some streams may
                    # carry non-zero y for 1D motor channels.
                    if float(p) == 0.0:
                        continue
                    channel_idx = int(x)
                    z_int = int(z)
                    max_z_seen = max(max_z_seen, z_int)
                    z_by_channel.setdefault(channel_idx, []).append(z_int)

                z_depth = _resolve_motor_linear_depth(
                    unit_ref,
                    max_z_seen + 1,
                )
                for channel_idx, z_values in z_by_channel.items():
                    if positioning_fractional:
                        decoded_unsigned = _decode_unsigned_percentage_fractional(
                            z_values
                        )
                    else:
                        decoded_unsigned = _decode_unsigned_percentage_linear(
                            z_values,
                            z_depth,
                        )
                    # Normalize absolute unsigned percentage to signed motor command
                    # so ServoMotor absolute mapping can span full joint range.
                    decoded = _normalize_unsigned_to_signed(decoded_unsigned)
                    channel_key = str(channel_idx)
                    if use_group_keys and group_id is not None:
                        channel_key = (
                            f"{group_id}:{channel_key}:{command_mode}"
                        )
                    else:
                        channel_key = f"{channel_key}:{command_mode}"
                    motors[channel_key] = decoded
                    prev = _last_decode_log.get(channel_key)
                    if prev is None or abs(prev - decoded) > 1e-6:
                        logger.info(
                            "[XYZP-DECODE] cid=%s mode=%s lane=single channel=%d decoded=%.6f x_vals=%s",
                            cortical_id,
                            command_mode,
                            channel_idx,
                            decoded,
                            [x for x in x_coords[:12]],
                        )
                        _last_decode_log[channel_key] = decoded
                continue

            # Fallback decoder for non-signed-percentage motor formats.
            for motor_idx, power in zip(x_coords, p_values):
                channel_key = str(int(motor_idx))
                if use_group_keys and group_id is not None:
                    channel_key = f"{group_id}:{channel_key}"
                motors[channel_key] = float(power) / 100.0

        except (KeyError, ValueError, TypeError) as e:
            logger.error(f"Error decoding motor data from {cortical_id}: {e}")
            continue

    return motors


def decode_sensor_xyzp_to_grid(
    xyzp_data: dict,
    cortical_id: str,
    grid_shape: Tuple[int, int, int],
) -> Dict[int, List[float]]:
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
        
        if not (
            len(x_coords) == len(y_coords) == len(z_coords) == len(p_values)
        ):
            logger.warning(f"Mismatched coordinate lengths in {cortical_id}")
            return grids
        
        # Map (x, y, z, p) to grid[z][y * width + x] = p
        for x, y, z, p in zip(x_coords, y_coords, z_coords, p_values):
            if 0 <= x < width and 0 <= y < height and 0 <= z < channels:
                flat_idx = int(y) * width + int(x)
                grids[int(z)][flat_idx] = float(p)
            else:
                logger.debug(
                    "Out-of-bounds neuron: (%s,%s,%s) for shape %s",
                    x,
                    y,
                    z,
                    grid_shape,
                )
        
        return grids
    
    except (KeyError, ValueError, TypeError) as e:
        logger.error(f"Error decoding sensor grid from {cortical_id}: {e}")
        return grids


def decode_1d_array_xyzp(
    xyzp_data: dict,
    cortical_id: str,
    length: int,
) -> List[float]:
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

