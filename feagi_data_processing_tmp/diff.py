"""
Frame and segmented diffs (pure NumPy).

This module provides simple utilities to compute absolute diffs, binary masks,
and pixel-wise mean squared error (MSE) between frames, and between dictionaries
of segmented tiles produced by segmentation.py.
"""

from __future__ import annotations

from typing import Dict, Tuple

import numpy as np


def _ensure_rgb_uint8(img: np.ndarray) -> np.ndarray:
    if img.ndim != 3 or img.shape[2] != 3:
        raise ValueError("Expected (H, W, 3) RGB array")
    if img.dtype != np.uint8:
        return img.astype(np.uint8, copy=False)
    return img


def frame_diff(
    prev_rgb: np.ndarray,
    curr_rgb: np.ndarray,
    *,
    threshold: int = 0,
) -> Tuple[np.ndarray, np.ndarray, float]:
    """Compute per-pixel absolute diff, binary mask, and MSE.

    Args:
        prev_rgb: Previous frame (H, W, 3), uint8.
        curr_rgb: Current frame (H, W, 3), uint8.
        threshold: Optional threshold for mask (0..255). Mask is 1 where
                   max(abs(diff across channels)) > threshold.

    Returns:
        diff_rgb: Absolute difference (H, W, 3), uint8.
        mask: Binary mask (H, W), uint8 {0, 1}.
        mse: Mean squared error across all channels.
    """
    a = _ensure_rgb_uint8(prev_rgb)
    b = _ensure_rgb_uint8(curr_rgb)
    if a.shape != b.shape:
        raise ValueError("Frames must have the same shape")

    # Compute absolute diff in uint8 without overflow using int16
    diff = np.abs(a.astype(np.int16) - b.astype(np.int16)).astype(np.uint8)
    # Binary mask using max across channels
    if threshold <= 0:
        mask = (np.max(diff, axis=2) > 0).astype(np.uint8)
    else:
        thr = int(max(0, min(255, threshold)))
        mask = (np.max(diff, axis=2) > thr).astype(np.uint8)
    # MSE across channels
    se = (a.astype(np.float32) - b.astype(np.float32)) ** 2
    mse = float(np.mean(se))
    return diff, mask, mse


def segmented_diff(
    prev_tiles: Dict[str, np.ndarray],
    curr_tiles: Dict[str, np.ndarray],
    *,
    threshold: int = 0,
) -> Dict[str, Tuple[np.ndarray, np.ndarray, float]]:
    """Compute diffs per-tile for segmented dictionaries.

    For keys present in both dictionaries, compute (diff_rgb, mask, mse). Keys
    present in only one dict are ignored.

    Args:
        prev_tiles: Dict of RGB tiles.
        curr_tiles: Dict of RGB tiles.
        threshold: Threshold for mask.

    Returns:
        Mapping key -> (diff_rgb, mask, mse)
    """
    out: Dict[str, Tuple[np.ndarray, np.ndarray, float]] = {}
    keys = sorted(set(prev_tiles.keys()) & set(curr_tiles.keys()))
    for k in keys:
        diff_rgb, mask, mse = frame_diff(prev_tiles[k], curr_tiles[k], threshold=threshold)
        out[k] = (diff_rgb, mask, mse)
    return out



