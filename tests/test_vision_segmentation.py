import numpy as np
import pytest


# Require Rust libs for this integration-style test; skip if not available in the env
frpl = pytest.importorskip("feagi_rust_py_libs", reason="feagi_rust_py_libs is required for segmentation test")


def _cid_to_str(cid_obj) -> str:
    try:
        s = str(cid_obj.as_ascii_string())
    except Exception:
        s = str(cid_obj)
    # Normalize forms like "CorticalID(iic400)" and quoted strings like "'iic400'"
    if s.startswith("CorticalID(") and s.endswith(")"):
        s = s[len("CorticalID("):-1]
    # Strip any surrounding quotes accidentally included by backend stringification
    s = s.strip().strip("'\"")
    return s


def test_segmented_vision_coordinate_bounds_and_ids():
    """
    Encode a synthetic frame through SegmentedVisionProcessor and verify:
    - Expected cortical IDs are present (center plus some peripherals)
    - All neuron coordinates fall within the configured tile dimensions per area
    """
    from feagi_connector.vision.processor import SegmentedVisionProcessor

    # Use modest sizes to keep test fast and deterministic
    center_dims = (64, 64)
    per_dims = (32, 32)

    proc = SegmentedVisionProcessor(
        cortical_group_index=0,
        center_dims=center_dims,
        peripheral_dims=per_dims,
        number_of_channels=1,
    )

    # Build a simple uniform BGR frame (uint8)
    h, w = 128, 128
    frame_bgr = np.full((h, w, 3), 200, dtype=np.uint8)

    sensor_bytes = proc.process_frame(frame_bgr)

    # Decode bytes using frpl
    bs = frpl.data_serialization.FeagiByteStructure(sensor_bytes)
    mapped = frpl.data_structures.neurons.xyzp.CorticalMappedXYZPNeuronData.new_from_feagi_byte_structure(bs)

    # Expected dimensions per cortical area
    dims_by_id = {
        "iic400": center_dims,
        "iic600": per_dims, "iic700": per_dims, "iic800": per_dims,
        "iic300": per_dims,                    "iic500": per_dims,
        "iic000": per_dims, "iic100": per_dims, "iic200": per_dims,
    }

    present_ids = set()
    for (cid_obj, arrays) in mapped.iter_full():
        cid = _cid_to_str(cid_obj)
        present_ids.add(cid)

        # Extract numpy arrays robustly across API variants
        try:
            x_coords, y_coords, z_coords, potentials = arrays.copy_as_tuple_of_numpy_arrays()
        except Exception:
            x_coords, y_coords, z_coords, potentials = arrays

        # If we recognize this cortical id, verify coordinate bounds
        if cid in dims_by_id:
            w_tile, h_tile = dims_by_id[cid]
            if len(x_coords) > 0:
                assert np.max(x_coords) < w_tile, f"x out of bounds for {cid}: max={np.max(x_coords)} >= {w_tile}"
            if len(y_coords) > 0:
                assert np.max(y_coords) < h_tile, f"y out of bounds for {cid}: max={np.max(y_coords)} >= {h_tile}"

    # Basic presence checks: center tile must exist; expect at least some peripheral tiles
    assert "iic400" in present_ids, "Center cortical area iic400 not found in encoded neurons"
    peripheral_seen = {cid for cid in present_ids if cid in dims_by_id and cid != "iic400"}
    assert len(peripheral_seen) >= 3, f"Expected multiple peripheral areas, got: {sorted(peripheral_seen)}"


