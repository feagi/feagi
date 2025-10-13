"""
Zero-serialization helper for FEAGI neuron XYZP data.

Builds a compact SoA payload directly from FEAGI byte structure using
feagi-rust-py-libs for parsing.

Format per slot (little-endian):
  magic 'ZS1N' (4 bytes), version u8=1, num_areas u8, 2 bytes pad
  repeat num_areas times:
    cortical_id ascii[6]
    count u32
    X[count] u16, Y[count] u16, Z[count] u16, P[count] f32
"""

from __future__ import annotations

from typing import Optional
import struct


def build_zero_serialized_xyzp(sensor_bytes: bytes) -> Optional[bytes]:
    try:
        import feagi_rust_py_libs as frpl  # type: ignore
        import numpy as np
    except Exception:
        return None

    try:
        # Try new API first (FeagiByteContainer), return None if not available
        if not hasattr(frpl.data_serialization, 'FeagiByteContainer'):
            return None
        feagi_bs = frpl.data_serialization.FeagiByteContainer(sensor_bytes)
        mapped = frpl.data_structures.neurons.xyzp.CorticalMappedXYZPNeuronData.new_from_feagi_byte_structure(feagi_bs)

        areas = []
        for (cid_obj, neuron_arrays) in mapped.iter_full():
            # Normalize cortical id to 6-char ascii
            try:
                cid = str(cid_obj.as_ascii_string()) if hasattr(cid_obj, "as_ascii_string") else str(cid_obj)
            except Exception:
                cid = str(cid_obj)
            if cid.startswith("CorticalID(") and cid.endswith(")"):
                cid = cid[len("CorticalID("):-1]
            cid = cid.strip().strip("'\"")
            import re as _re
            cid_clean = _re.sub(r"[^A-Za-z0-9]", "", cid)
            m = _re.search(r"[A-Za-z]{3}\d{3}", cid_clean)
            if m:
                cid = m.group(0)
            else:
                cid = cid_clean[:6]
                if len(cid) < 6:
                    continue

            try:
                x_coords, y_coords, z_coords, potentials = neuron_arrays
            except Exception:
                continue

            xs = np.asarray(x_coords, dtype=np.uint16)
            ys = np.asarray(y_coords, dtype=np.uint16)
            zs = np.asarray(z_coords, dtype=np.uint16)
            ps = np.asarray(potentials, dtype=np.float32)

            count = min(len(xs), len(ys), len(zs), len(ps))
            if count == 0:
                continue
            areas.append((cid.encode("ascii", errors="ignore"), count, xs[:count], ys[:count], zs[:count], ps[:count]))

        if not areas:
            return None

        parts = []
        parts.append(b"ZS1N")
        parts.append(struct.pack("<BBH", 1, len(areas), 0))
        for cid_bytes, count, xs, ys, zs, ps in areas:
            parts.append(cid_bytes)
            parts.append(struct.pack("<I", count))
            parts.append(xs.tobytes(order="C"))
            parts.append(ys.tobytes(order="C"))
            parts.append(zs.tobytes(order="C"))
            parts.append(ps.tobytes(order="C"))

        return b"".join(parts)
    except Exception:
        return None


