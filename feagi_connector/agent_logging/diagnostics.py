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
        # Try new API first (FeagiByteContainer), return if not available
        if not hasattr(frpl.data_serialization, 'FeagiByteContainer'):
            return
        
        # Skip if empty bytes
        if not sensor_bytes or len(sensor_bytes) == 0:
            return
            
        try:
            feagi_bs = frpl.data_serialization.FeagiByteContainer()
            feagi_bs.load_bytes_and_verify(sensor_bytes)
            mapped = feagi_bs.try_create_new_struct_from_index(0)
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            # Handle Rust panics or invalid byte structures
            import sys
            logging.debug(f"[DIAG] Failed to deserialize sensor bytes: {sys.exc_info()[1]}")
            return
            
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


