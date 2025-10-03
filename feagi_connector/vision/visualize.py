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
            # WORKAROUND: Rust bug in beta.28 swaps X/Y during encoding, so we swap them back here
            # TODO: Remove this swap once feagi_data_structures fixes the indexed_iter bug
            y = int(xs[i])  # Rust encoded Y into X position
            x = int(ys[i])  # Rust encoded X into Y position
            if 0 <= x < w and 0 <= y < h:
                p = float(ps[i])
                p = 0.0 if p < 0.0 else (1.0 if p > 1.0 else p)
                val = int(p * 255.0)
                img[y, x, :] = val
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
    
    # Use actual cortical dimensions from FEAGI genome
    # The mosaic display dimensions should match the neuron space dimensions
    # so that each neuron maps 1:1 to a pixel (or use integer scaling)
    center_display_w = cw
    center_display_h = ch
    periph_display_w = pw
    periph_display_h = ph
    
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
    # Interior separator thickness between tiles (visual grid lines)
    # Set to 10px so middle/corner tiles remain their true sizes (e.g., 16x16)
    grid = 5
    outer_border_thickness = 5
    total_w = pw + grid + cw + grid + pw
    total_h = ph + grid + ch + grid + ph
    # Initialize with gray background (gaps between segments)
    grid_color = 30
    mosaic = np.full((total_h, total_w, 3), grid_color, dtype=np.uint8)
    
    # Calculate middle segment positions (they are centered relative to iic400)
    # Coordinates are relative to mosaic origin; outer border is drawn later and does not
    # affect tile positions
    center_x0 = pw + grid
    center_y0 = ph + grid
    center_cx = center_x0 + cw // 2  # Center X of iic400
    center_cy = center_y0 + ch // 2  # Center Y of iic400
    
    top_mid_x0 = center_cx - pw // 2
    bot_mid_x0 = center_cx - pw // 2
    mid_left_y0 = center_cy - ph // 2
    mid_right_y0 = center_cy - ph // 2
    
    import logging
    logger = logging.getLogger(__name__)
    logger.info(f"[MOSAIC-FILL] cw={cw}, ch={ch}, pw={pw}, ph={ph}")
    logger.info(f"[MOSAIC-FILL] iic700: x=[{top_mid_x0}:{top_mid_x0+pw}] y=[0:{ph}] -> width={pw}, height={ph}")
    logger.info(f"[MOSAIC-FILL] iic300: x=[0:{pw}] y=[{mid_left_y0}:{mid_left_y0+ph}] -> width={pw}, height={ph}")
    
    # Fill segment areas with black (will be overwritten by neuron data)
    # Use the tiles dictionary positions directly to avoid overlap
    
    # Corner segments (peripheral dimensions)
    mosaic[0:ph, 0:pw, :] = 0  # iic600 (top-left)
    mosaic[0:ph, pw + grid + cw + grid: pw + grid + cw + grid + pw, :] = 0  # iic800 (top-right)
    mosaic[ph + grid + ch + grid : ph + grid + ch + grid + ph, 0:pw, :] = 0  # iic000 (bottom-left)
    mosaic[ph + grid + ch + grid : ph + grid + ch + grid + ph, pw + grid + cw + grid: pw + grid + cw + grid + pw, :] = 0  # iic200 (bottom-right)
    
    # Middle segments (centered, using their ACTUAL centered positions, not overlapping corners)
    mosaic[0:ph, top_mid_x0:top_mid_x0+pw, :] = 0  # iic700 (top-middle, centered horizontally)
    mosaic[ph + grid + ch + grid : ph + grid + ch + grid + ph, bot_mid_x0:bot_mid_x0+pw, :] = 0  # iic100 (bottom-middle, centered horizontally)
    
    # Middle-left and middle-right: These are in the MIDDLE row, NOT overlapping top/bottom
    # They should be at the center Y position (centered vertically relative to iic400)
    mosaic[mid_left_y0:mid_left_y0+ph, 0:pw, :] = 0  # iic300 (middle-left, centered vertically)
    mosaic[mid_right_y0:mid_right_y0+ph, pw + grid + cw + grid : pw + grid + cw + grid + pw, :] = 0  # iic500 (middle-right, centered vertically)
    
    # Center segment (full center dimensions)
    mosaic[center_y0:center_y0+ch, center_x0:center_x0+cw, :] = 0  # iic400 (center)

    try:
        import feagi_rust_py_libs as frpl  # type: ignore
    except Exception:
        return mosaic

    # Use the passed dimensions as the tile display sizes
    # cw, ch = center tile dimensions (e.g., 128x128)
    # pw, ph = peripheral tile dimensions (e.g., 16x16)
    # Middle segments use the peripheral dimensions for their display
    neuron_dims = {
        "iic600": (pw, ph), "iic700": (pw, ph), "iic800": (pw, ph),
        "iic300": (pw, ph), "iic400": (cw, ch), "iic500": (pw, ph),
        "iic000": (pw, ph), "iic100": (pw, ph), "iic200": (pw, ph),
    }
    
    # Center tile position and center point
    center_x0 = pw + grid
    center_y0 = ph + grid
    center_cx = center_x0 + cw // 2  # Center X of iic400
    center_cy = center_y0 + ch // 2  # Center Y of iic400
    
    # Middle segments: use peripheral dimensions, centered relative to center tile
    # Top-middle (iic700)
    top_mid_w, top_mid_h = pw, ph
    top_mid_x0 = center_cx - top_mid_w // 2
    
    # Bottom-middle (iic100)
    bot_mid_w, bot_mid_h = pw, ph
    bot_mid_x0 = center_cx - bot_mid_w // 2
    
    # Middle-left (iic300)
    mid_left_w, mid_left_h = pw, ph
    mid_left_y0 = center_cy - mid_left_h // 2
    
    # Middle-right (iic500)
    mid_right_w, mid_right_h = pw, ph
    mid_right_y0 = center_cy - mid_right_h // 2
    
    import logging
    logger = logging.getLogger(__name__)
    logger.debug(f"[MOSAIC-LAYOUT] cw={cw}, ch={ch}, pw={pw}, ph={ph}, grid={grid}")
    logger.debug(f"[MOSAIC-LAYOUT] iic400: pos=({center_x0},{center_y0}), size=({cw},{ch}), center=({center_cx},{center_cy})")
    logger.debug(f"[MOSAIC-LAYOUT] iic700: pos=({top_mid_x0},0), size=({top_mid_w},{top_mid_h}), center=({top_mid_x0 + top_mid_w//2},{top_mid_h//2})")
    logger.debug(f"[MOSAIC-LAYOUT] Are centers aligned? {top_mid_x0 + top_mid_w//2} == {center_cx}? {top_mid_x0 + top_mid_w//2 == center_cx}")
    
    tiles = {
        "iic600": (0, 0, pw, ph),
        "iic700": (top_mid_x0, 0, top_mid_w, top_mid_h),  # Top-middle: centered
        "iic800": (pw + grid + cw + grid, 0, pw, ph),
        "iic300": (0, mid_left_y0, mid_left_w, mid_left_h),  # Middle-left: centered
        "iic400": (center_x0, center_y0, cw, ch),
        "iic500": (pw + grid + cw + grid, mid_right_y0, mid_right_w, mid_right_h),  # Middle-right: centered
        "iic000": (0, ph + grid + ch + grid, pw, ph),
        "iic100": (bot_mid_x0, ph + grid + ch + grid, bot_mid_w, bot_mid_h),  # Bottom-middle: centered
        "iic200": (pw + grid + cw + grid, ph + grid + ch + grid, pw, ph),
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
                # WORKAROUND: Rust bug in beta.28 swaps X/Y during encoding, so we swap them back here
                # TODO: Remove this swap once feagi_data_structures fixes the indexed_iter bug
                y_neuron = int(xs[i])  # Rust encoded Y into X position
                x_neuron = int(ys[i])  # Rust encoded X into Y position
                
                # Scale neuron coordinates from neuron space to display space
                x_scaled = int(x_neuron * scale_x)
                y_scaled = int(y_neuron * scale_y)
                
                if 0 <= x_scaled < tw and 0 <= y_scaled < th:
                    p = float(ps[i])
                    p = 0.0 if p < 0.0 else (1.0 if p > 1.0 else p)
                    val = int(p * 255.0)
                    
                    # Calculate pixel block size (at least 1 pixel, scaled by the scale factor)
                    block_w = max(1, int(scale_x))
                    block_h = max(1, int(scale_y))
                    
                    # Position in mosaic
                    mosaic_y_center = y0 + y_scaled
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
        
        # Draw visible interior grid lines using the full gap area (no overlap with tiles)
        grid_color = 30  # Gray color for grid lines
        outer_border_thickness = 5
        
        # Vertical interior gaps
        mosaic[:, pw:pw + grid, :] = grid_color
        mosaic[:, pw + grid + cw: pw + grid + cw + grid, :] = grid_color
        
        # Horizontal interior gaps
        mosaic[ph:ph + grid, :, :] = grid_color
        mosaic[ph + grid + ch: ph + grid + ch + grid, :, :] = grid_color
        
        # Draw 5-pixel thick border around the entire frame
        mosaic[:outer_border_thickness, :, :] = grid_color  # Top border
        mosaic[-outer_border_thickness:, :, :] = grid_color  # Bottom border
        mosaic[:, :outer_border_thickness, :] = grid_color  # Left border
        mosaic[:, -outer_border_thickness:, :] = grid_color  # Right border
        
        return mosaic
    except Exception as e:
        import logging
        logging.error(f"[MOSAIC] ❌ Failed to build mosaic: {e}")
        import traceback
        logging.error(f"[MOSAIC] Traceback: {traceback.format_exc()}")
        return mosaic


