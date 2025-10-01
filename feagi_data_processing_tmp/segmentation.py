"""
Pure-NumPy 3x3 segmented vision utilities.

This module provides utilities to generate a 3x3 segmented layout from a single
RGB frame. It is designed to be a lightweight, OS-agnostic fallback when native
libraries are unavailable.

Key functions:
- segment_image_3x3: produce 9 tiles (center + 8 peripherals) with gaze support
- build_mosaic_from_segments: assemble a mosaic image from produced tiles

Conventions:
- Input frames are RGB uint8 arrays of shape (H, W, 3)
- Tile IDs follow FEAGI's conventional naming:
  Row1: [iic600, iic700, iic800]
  Row2: [iic300, iic400, iic500]
  Row3: [iic000, iic100, iic200]

The output tiles have sizes:
- Corners: per_dims (per_w, per_h)
- Top/Bottom middle: (center_w, per_h)
- Left/Right middle: (per_w, center_h)
- Center: center_dims (center_w, center_h)
"""

from __future__ import annotations

from typing import Dict, Tuple

import numpy as np


TileMap = Dict[str, np.ndarray]


def _resize_nn(img: np.ndarray, new_w: int, new_h: int) -> np.ndarray:
    """Nearest-neighbor resize (numpy-only) for RGB uint8 images.

    Args:
        img: Source image (H, W, C=3), uint8.
        new_w: Target width.
        new_h: Target height.

    Returns:
        Resized image (new_h, new_w, 3), uint8.
    """
    h, w = int(img.shape[0]), int(img.shape[1])
    if h <= 0 or w <= 0 or new_w <= 0 or new_h <= 0:
        return np.zeros((max(new_h, 0), max(new_w, 0), img.shape[2]), dtype=img.dtype)

    # Generate target indices
    xs = np.linspace(0, max(w - 1, 0), new_w, dtype=np.float32)
    ys = np.linspace(0, max(h - 1, 0), new_h, dtype=np.float32)

    xi = np.clip(xs.round().astype(np.int64), 0, w - 1)
    yi = np.clip(ys.round().astype(np.int64), 0, h - 1)

    out = img[yi[:, None], xi[None, :], :]
    return out.astype(np.uint8, copy=False)


def _safe_crop(img: np.ndarray, x: int, y: int, w: int, h: int) -> np.ndarray:
    """Crop a region clamped to image bounds; pads with black if needed.

    Args:
        img: Source image (H, W, 3).
        x, y: Top-left of the region in source coordinates.
        w, h: Width/height of region.

    Returns:
        RGB uint8 array of shape (h, w, 3) where out-of-bounds areas are black.
    """
    H, W = img.shape[0], img.shape[1]
    x0 = max(0, x)
    y0 = max(0, y)
    x1 = min(W, x + w)
    y1 = min(H, y + h)
    out = np.zeros((h, w, img.shape[2]), dtype=img.dtype)
    if x0 >= x1 or y0 >= y1:
        return out
    dst_x0 = x0 - x
    dst_y0 = y0 - y
    out[dst_y0 : dst_y0 + (y1 - y0), dst_x0 : dst_x0 + (x1 - x0)] = img[y0:y1, x0:x1]
    return out


def segment_image_3x3(
    frame_rgb: np.ndarray,
    center_dims: Tuple[int, int],
    per_dims: Tuple[int, int],
    gaze: Tuple[float, float] = (0.5, 0.5),
) -> TileMap:
    """Segment an RGB frame into a 3x3 layout with a central high-res tile.

    The segmentation window is sized proportionally to the requested output tile
    sizes. A normalized gaze point (cx, cy) in [0,1] selects the window center.

    Args:
        frame_rgb: RGB uint8 image (H, W, 3).
        center_dims: (center_w, center_h) output size for the center tile.
        per_dims: (per_w, per_h) output size for peripheral tiles.
        gaze: (cx, cy) normalized gaze in [0,1], where (0.5, 0.5) is image center.

    Returns:
        Dict mapping cortical IDs to RGB tiles as numpy arrays.
    """
    if frame_rgb.ndim != 3 or frame_rgb.shape[2] != 3:
        raise ValueError("Expected RGB image of shape (H, W, 3)")
    if frame_rgb.dtype != np.uint8:
        frame = frame_rgb.astype(np.uint8, copy=False)
    else:
        frame = frame_rgb

    H, W = int(frame.shape[0]), int(frame.shape[1])
    cw, ch = int(center_dims[0]), int(center_dims[1])
    pw, ph = int(per_dims[0]), int(per_dims[1])
    cx = float(np.clip(gaze[0], 0.0, 1.0))
    cy = float(np.clip(gaze[1], 0.0, 1.0))

    # Compute proportional source window sizes based on requested output sizes
    total_w_units = 2 * pw + cw
    total_h_units = 2 * ph + ch
    # Avoid division by zero; fallback to equal thirds
    if total_w_units <= 0:
        total_w_units = 3
        pw = cw = 1
    if total_h_units <= 0:
        total_h_units = 3
        ph = ch = 1

    # Map tile output widths/heights to source widths/heights proportionally
    scale_w = W / float(total_w_units)
    scale_h = H / float(total_h_units)
    src_pw = max(1, int(round(pw * scale_w)))
    src_cw = max(1, int(round(cw * scale_w)))
    src_ph = max(1, int(round(ph * scale_h)))
    src_ch = max(1, int(round(ch * scale_h)))

    grid_w = src_pw + src_cw + src_pw
    grid_h = src_ph + src_ch + src_ph

    # Place the 3x3 window centered at gaze
    gx = int(round(cx * (W - 1)))
    gy = int(round(cy * (H - 1)))
    grid_x0 = gx - (src_cw // 2 + src_pw)
    grid_y0 = gy - (src_ch // 2 + src_ph)
    # Clamp window to fit inside the frame
    grid_x0 = max(0, min(grid_x0, W - grid_w))
    grid_y0 = max(0, min(grid_y0, H - grid_h))

    # Define tile specs: (key, col_width_src, row_height_src, out_w, out_h, col_offset_src, row_offset_src)
    layout = [
        ("iic600", src_pw, src_ph, pw, ph, 0, 0),
        ("iic700", src_cw, src_ph, cw, ph, src_pw, 0),
        ("iic800", src_pw, src_ph, pw, ph, src_pw + src_cw, 0),
        ("iic300", src_pw, src_ch, pw, ch, 0, src_ph),
        ("iic400", src_cw, src_ch, cw, ch, src_pw, src_ph),
        ("iic500", src_pw, src_ch, pw, ch, src_pw + src_cw, src_ph),
        ("iic000", src_pw, src_ph, pw, ph, 0, src_ph + src_ch),
        ("iic100", src_cw, src_ph, cw, ph, src_pw, src_ph + src_ch),
        ("iic200", src_pw, src_ph, pw, ph, src_pw + src_cw, src_ph + src_ch),
    ]

    tiles: TileMap = {}
    for key, sw, sh, out_w, out_h, ox, oy in layout:
        sx = grid_x0 + ox
        sy = grid_y0 + oy
        crop = _safe_crop(frame, sx, sy, sw, sh)
        tile = _resize_nn(crop, int(out_w), int(out_h))
        tiles[key] = tile

    return tiles


def build_mosaic_from_segments(
    tiles: TileMap,
    center_dims: Tuple[int, int],
    per_dims: Tuple[int, int],
    grid: int = 1,
) -> np.ndarray:
    """Assemble a 3x3 mosaic image from pre-segmented tiles.

    Args:
        tiles: Dict from segment_image_3x3 with keys like "iic400", etc.
        center_dims: (center_w, center_h).
        per_dims: (per_w, per_h).
        grid: Optional 1px grid spacing between tiles.

    Returns:
        Mosaic RGB uint8 image.
    """
    cw, ch = int(center_dims[0]), int(center_dims[1])
    pw, ph = int(per_dims[0]), int(per_dims[1])
    total_w = pw + grid + cw + grid + pw
    total_h = ph + grid + ch + grid + ph
    mosaic = np.zeros((total_h, total_w, 3), dtype=np.uint8)

    # Placement table: key -> (x0, y0)
    placements = {
        "iic600": (0, 0),
        "iic700": (pw + grid, 0),
        "iic800": (pw + grid + cw + grid, 0),
        "iic300": (0, ph + grid),
        "iic400": (pw + grid, ph + grid),
        "iic500": (pw + grid + cw + grid, ph + grid),
        "iic000": (0, ph + grid + ch + grid),
        "iic100": (pw + grid, ph + grid + ch + grid),
        "iic200": (pw + grid + cw + grid, ph + grid + ch + grid),
    }

    # Expected tile sizes for sanity; fall back to resize if mismatched
    expected_sizes = {
        "iic600": (pw, ph),
        "iic700": (cw, ph),
        "iic800": (pw, ph),
        "iic300": (pw, ch),
        "iic400": (cw, ch),
        "iic500": (pw, ch),
        "iic000": (pw, ph),
        "iic100": (cw, ph),
        "iic200": (pw, ph),
    }

    for key, (x0, y0) in placements.items():
        if key not in tiles:
            continue
        tile = tiles[key]
        ew, eh = expected_sizes[key]
        if tile.shape[1] != ew or tile.shape[0] != eh:
            tile = _resize_nn(tile, ew, eh)
        mosaic[y0 : y0 + tile.shape[0], x0 : x0 + tile.shape[1]] = tile

    return mosaic



