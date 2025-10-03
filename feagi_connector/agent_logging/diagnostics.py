"""
Diagnostics helpers: compact, reusable logging for FEAGI byte structures.

Provides human-readable summaries of per-area neuron counts using
feagi-rust-py-libs, when available.
"""

from __future__ import annotations

import logging
from typing import Optional


def log_sensor_area_counts(logger: logging.Logger, sensor_bytes: bytes) -> None:
    """Log compact per-area counts, if debug logging is enabled.

    Safe to call without feagi-rust-py-libs installed; will no-op on ImportError.
    """
    if not logger.isEnabledFor(logging.DEBUG):
        return
    try:
        import feagi_rust_py_libs as frpl  # type: ignore
    except Exception:
        return

    try:
        feagi_bs = frpl.data_serialization.FeagiByteStructure(sensor_bytes)
        mapped = frpl.data_structures.neurons.xyzp.CorticalMappedXYZPNeuronData.new_from_feagi_byte_structure(feagi_bs)
        area_counts = []
        total = 0
        for (cid_obj, neuron_arrays) in mapped.iter_full():
            try:
                cid = str(cid_obj.as_ascii_string()) if hasattr(cid_obj, "as_ascii_string") else str(cid_obj)
            except Exception:
                cid = str(cid_obj)
            if cid.startswith("CorticalID(") and cid.endswith(")"):
                cid = cid[len("CorticalID("):-1]
            try:
                x_coords, y_coords, z_coords, potentials = neuron_arrays
                count = len(x_coords)
            except Exception:
                count = 0
            total += int(count)
            area_counts.append((cid, int(count)))
        counts_str = ", ".join(f"{cid}={cnt}" for cid, cnt in area_counts)
        logger.debug(f"[RAW-DBG] areas={len(area_counts)}, total_points={total} :: {counts_str}")
    except Exception as e:
        logger.debug(f"[RAW-DBG] summary failed: {e}")


