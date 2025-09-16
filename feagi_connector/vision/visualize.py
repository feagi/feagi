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
                img[y, x, :] = val
        return img
    except Exception:
        return img


def build_segmented_mosaic(sensor_bytes: bytes, center_wh: Tuple[int, int], per_wh: Tuple[int, int]) -> np.ndarray:
    """Build a 3x3 segmented mosaic from neuron bytes using frpl segmentation.

    Layout:
        Row1: [iic600 | iic700 | iic800]
        Row2: [iic300 | iic400 | iic500]
        Row3: [iic000 | iic100 | iic200]
    """
    cw, ch = int(center_wh[0]), int(center_wh[1])
    pw, ph = int(per_wh[0]), int(per_wh[1])
    grid = 1
    total_w = pw + grid + cw + grid + pw
    total_h = ph + grid + ch + grid + ph
    mosaic = np.zeros((total_h, total_w, 3), dtype=np.uint8)

    try:
        import feagi_rust_py_libs as frpl  # type: ignore
    except Exception:
        return mosaic

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

    try:
        bs = frpl.data_serialization.FeagiByteStructure(sensor_bytes)
        mapped = frpl.data_structures.neurons.xyzp.CorticalMappedXYZPNeuronData.new_from_feagi_byte_structure(bs)
        order = [
            "iic600", "iic700", "iic800",
            "iic300", "iic400", "iic500",
            "iic000", "iic100", "iic200",
        ]
        for cid_key in order:
            if cid_key not in tiles:
                continue
            x0, y0, tw, th = tiles[cid_key]
            try:
                cid = frpl.data_structures.genomic.CorticalID.try_new_from_string(cid_key)
                arrays = mapped.get_neurons_of(cid)
            except Exception:
                continue
            try:
                x_coords, y_coords, z_coords, potentials = arrays.copy_as_tuple_of_numpy_arrays()
            except Exception:
                try:
                    x_coords, y_coords, z_coords, potentials = arrays
                except Exception:
                    continue
            xs = np.asarray(x_coords, dtype=np.int32)
            ys = np.asarray(y_coords, dtype=np.int32)
            ps = np.asarray(potentials, dtype=np.float32)
            n = min(xs.size, ys.size, ps.size)
            for i in range(n):
                x = int(xs[i])
                y = int(ys[i])
                if 0 <= x < tw and 0 <= y < th:
                    p = float(ps[i])
                    p = 0.0 if p < 0.0 else (1.0 if p > 1.0 else p)
                    val = int(p * 255.0)
                    mosaic[y0 + y, x0 + x, :] = val
        return mosaic
    except Exception:
        return mosaic


