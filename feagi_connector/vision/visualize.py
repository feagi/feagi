"""
Visualization builders for FEAGI neuron bytes.

These helpers avoid OpenCV dependencies and return RGB uint8 numpy arrays.
"""

from __future__ import annotations

from typing import Dict, Tuple, Optional

import numpy as np


def build_neural_image(sensor_bytes: bytes, target_wh: Tuple[int, int]) -> np.ndarray:
    """Render a simple grayscale scatter image for a single cortical area.

    This scans for iic400 first; if unavailable, renders the first available area.
    """
    h, w = int(target_wh[1]), int(target_wh[0])
    img = np.zeros((h, w, 3), dtype=np.uint8)
    try:
        import feagi_rust_py_libs as frpl  # type: ignore
    except Exception:
        return img

    try:
        bs = frpl.data_serialization.FeagiByteStructure(sensor_bytes)
        mapped = frpl.data_structures.neurons.xyzp.CorticalMappedXYZPNeuronData.new_from_feagi_byte_structure(bs)
        # Prefer iic400
        target_id = frpl.data_structures.genomic.CorticalID.try_new_from_string("iic400")
        arrays = None
        try:
            arrays = mapped.get_neurons_of(target_id)
        except Exception:
            # fallback to first area
            for (cid_obj, neuron_arrays) in mapped.iter_full():
                arrays = neuron_arrays
                break
        if arrays is None:
            return img
        x_coords, y_coords, z_coords, potentials = arrays.copy_as_tuple_of_numpy_arrays()
        xs = np.asarray(x_coords, dtype=np.int32)
        ys = np.asarray(y_coords, dtype=np.int32)
        ps = np.asarray(potentials, dtype=np.float32)
        n = min(xs.size, ys.size, ps.size)
        for i in range(n):
            x = int(xs[i])
            y = int(ys[i])
            if 0 <= x < w and 0 <= y < h:
                p = float(ps[i])
                p = 0.0 if p < 0.0 else (1.0 if p > 1.0 else p)
                val = int(p * 255.0)
                # WORKAROUND: Rust bug - Y is flipped with off-by-one (height - y instead of height - 1 - y)
                # Compensate by flipping back: if Rust wrote (h - y), we reverse to get y
                img_y = h - 1 - y
                if 0 <= img_y < h:
                    img[img_y, x, :] = val
        return img
    except Exception:
        return img


def build_segmented_mosaic_with_gaze(
    sensor_bytes: bytes, 
    center_wh: Tuple[int, int], 
    per_wh: Tuple[int, int],
    eccentricity: Tuple[float, float],
    modulation: Tuple[float, float]
) -> np.ndarray:
    """Build a 3x3 segmented mosaic with tiles sized according to gaze parameters.
    
    Args:
        sensor_bytes: Encoded neuron data
        center_wh: Output dimensions for center tile
        per_wh: Output dimensions for peripheral tiles
        eccentricity: (x, y) center position (0-1)
        modulation: (x, y) center size as fraction of total (0-1)
    
    Layout:
        Row1: [iic600 | iic700 | iic800]
        Row2: [iic300 | iic400 | iic500]
        Row3: [iic000 | iic100 | iic200]
    """
    cw, ch = int(center_wh[0]), int(center_wh[1])
    pw, ph = int(per_wh[0]), int(per_wh[1])
    
    # Calculate tile sizes based on modulation (what fraction of source image each segment represents)
    mod_x, mod_y = modulation
    
    # Center gets modulation fraction, peripherals split the rest
    # For a square mosaic, make center tile proportional to modulation
    base_size = 200  # Base mosaic size
    center_display_w = int(base_size * mod_x)
    center_display_h = int(base_size * mod_y)
    periph_display_w = int(base_size * (1.0 - mod_x) / 2.0)
    periph_display_h = int(base_size * (1.0 - mod_y) / 2.0)
    
    return _build_mosaic_internal(sensor_bytes, center_display_w, center_display_h, periph_display_w, periph_display_h)


def build_segmented_mosaic(sensor_bytes: bytes, center_wh: Tuple[int, int], per_wh: Tuple[int, int]) -> np.ndarray:
    """Build a 3x3 segmented mosaic from neuron bytes using frpl segmentation.

    Layout:
        Row1: [iic600 | iic700 | iic800]
        Row2: [iic300 | iic400 | iic500]
        Row3: [iic000 | iic100 | iic200]
    """
    cw, ch = int(center_wh[0]), int(center_wh[1])
    pw, ph = int(per_wh[0]), int(per_wh[1])
    return _build_mosaic_internal(sensor_bytes, cw, ch, pw, ph)


def _build_mosaic_internal(sensor_bytes: bytes, cw: int, ch: int, pw: int, ph: int) -> np.ndarray:
    """Internal function to build mosaic with given tile dimensions.
    
    Args:
        cw, ch: Display dimensions for center tile
        pw, ph: Display dimensions for peripheral tiles
    """
    grid = 1
    total_w = pw + grid + cw + grid + pw
    total_h = ph + grid + ch + grid + ph
    mosaic = np.zeros((total_h, total_w, 3), dtype=np.uint8)

    try:
        import feagi_rust_py_libs as frpl  # type: ignore
    except Exception:
        return mosaic

    # Tile layout with display dimensions
    tiles = {
        "iic600": (0, 0, pw, ph),
        "iic700": (pw + grid, 0, cw, ph),
        "iic800": (pw + grid + cw + grid, 0, pw, ph),
        "iic300": (0, ph + grid, pw, ch),
        "iic400": (pw + grid, ph + grid, cw, ch),
        "iic500": (pw + grid + cw + grid, ph + grid, pw, ch),
        "iic000": (0, ph + grid + ch + grid, pw, ph),
        "iic100": (pw + grid, ph + grid + ch + grid, cw, ph),
        "iic200": (pw + grid + cw + grid, ph + grid + ch + grid, pw, ph),
    }
    
    # Original neuron coordinate dimensions (from FEAGI genome)
    # These are the dimensions neurons are encoded in
    neuron_dims = {
        "iic600": (16, 16), "iic700": (128, 16), "iic800": (16, 16),
        "iic300": (16, 128), "iic400": (128, 128), "iic500": (16, 128),
        "iic000": (16, 16), "iic100": (128, 16), "iic200": (16, 16),
    }

    try:
        bs = frpl.data_serialization.FeagiByteStructure(sensor_bytes)
        mapped = frpl.data_structures.neurons.xyzp.CorticalMappedXYZPNeuronData.new_from_feagi_byte_structure(bs)
        order = [
            "iic600", "iic700", "iic800",
            "iic300", "iic400", "iic500",
            "iic000", "iic100", "iic200",
        ]
        neuron_count = 0
        import logging
        logging.info(f"[MOSAIC-DEBUG] Available cortical IDs in data: {[str(cid) for (cid, _) in mapped.iter_full()]}")
        for cid_key in order:
            if cid_key not in tiles:
                continue
            x0, y0, tw, th = tiles[cid_key]
            nw, nh = neuron_dims.get(cid_key, (tw, th))  # Get original neuron dimensions
            
            # Calculate scale factors to map neuron coords to display tile size
            scale_x = float(tw) / float(nw) if nw > 0 else 1.0
            scale_y = float(th) / float(nh) if nh > 0 else 1.0
            
            try:
                cid = frpl.data_structures.genomic.CorticalID.try_new_from_string(cid_key)
                arrays = mapped.get_neurons_of(cid)
            except Exception as e:
                logging.debug(f"[MOSAIC] Failed to get neurons for {cid_key}: {e}")
                continue
            try:
                x_coords, y_coords, z_coords, potentials = arrays.copy_as_tuple_of_numpy_arrays()
            except Exception:
                try:
                    x_coords, y_coords, z_coords, potentials = arrays
                except Exception as e:
                    import logging
                    logging.debug(f"[MOSAIC] Failed to extract arrays for {cid_key}: {e}")
                    continue
            xs = np.asarray(x_coords, dtype=np.int32)
            ys = np.asarray(y_coords, dtype=np.int32)
            ps = np.asarray(potentials, dtype=np.float32)
            n = min(xs.size, ys.size, ps.size)
            neuron_count += n
            if n > 0:
                logging.info(f"[MOSAIC-DEBUG] {cid_key}: {n} neurons, tile@({x0},{y0}) display=({tw}x{th}), neuron_space=({nw}x{nh}), scale=({scale_x:.2f},{scale_y:.2f}), neuron_range x=[{xs.min()},{xs.max()}] y=[{ys.min()},{ys.max()}]")
            for i in range(n):
                # Scale neuron coordinates from neuron space to display space
                x_neuron = int(xs[i])
                y_neuron = int(ys[i])
                x_scaled = int(x_neuron * scale_x)
                y_scaled = int(y_neuron * scale_y)
                
                if 0 <= x_scaled < tw and 0 <= y_scaled < th:
                    p = float(ps[i])
                    p = 0.0 if p < 0.0 else (1.0 if p > 1.0 else p)
                    val = int(p * 255.0)
                    
                    # Calculate pixel block size (at least 1 pixel, scaled by the scale factor)
                    block_w = max(1, int(scale_x))
                    block_h = max(1, int(scale_y))
                    
                    # WORKAROUND: Rust bug - Y is flipped with off-by-one (height - y instead of height - 1 - y)
                    # Compensate by flipping back: if Rust wrote (h - y), we reverse to get y
                    mosaic_y_center = y0 + (th - 1 - y_scaled)
                    mosaic_x_center = x0 + x_scaled
                    
                    # Draw a block of pixels instead of a single pixel
                    for dy in range(block_h):
                        for dx in range(block_w):
                            mosaic_y = mosaic_y_center + dy
                            mosaic_x = mosaic_x_center + dx
                            if 0 <= mosaic_y < total_h and 0 <= mosaic_x < total_w:
                                mosaic[mosaic_y, mosaic_x, :] = val
        if neuron_count > 0:
            import logging
            logging.debug(f"[MOSAIC] ✅ Built mosaic: {total_w}x{total_h}, {neuron_count} neurons")
        
        # Draw visible grid lines (white) to separate the 3x3 tiles
        grid_color = 128  # Gray color for grid lines
        # Vertical lines
        mosaic[:, pw, :] = grid_color  # First vertical line
        mosaic[:, pw + grid + cw, :] = grid_color  # Second vertical line
        # Horizontal lines
        mosaic[ph, :, :] = grid_color  # First horizontal line
        mosaic[ph + grid + ch, :, :] = grid_color  # Second horizontal line
        
        return mosaic
    except Exception as e:
        import logging
        logging.error(f"[MOSAIC] ❌ Failed to build mosaic: {e}")
        import traceback
        logging.error(f"[MOSAIC] Traceback: {traceback.format_exc()}")
        return mosaic


