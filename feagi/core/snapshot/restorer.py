"""
Brain snapshot restore utilities (phase 1).

Restores minimal snapshot content from a folder:
- Reads manifest.json to validate snapshot folder
- Reads connectome.json (structure summary) – currently informational
- Reads state.json and applies brain stats where safe

Future phases will restore neuron/synapse/memory SoA arrays and rebuild indexes.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict


class SnapshotRestoreError(Exception):
    pass


def _read_json(path: Path) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _blake2b_hex(path: Path) -> str:
    from hashlib import blake2b

    data = path.read_bytes()
    return blake2b(data, digest_size=32).hexdigest()


def restore_brain_snapshot(
    snapshot_root: Path,
    snapshot_id: str,
    state_manager,
    connectome_manager=None,
) -> bool:
    """
    Restore a minimal brain snapshot (phase 1) from snapshot_root/snapshot_id.

    Args:
        snapshot_root: Base directory containing snapshots
        snapshot_id: Snapshot directory name
        state_manager: FeagiStateManager instance
        connectome_manager: Optional ConnectomeManager for restoring SoA arrays

    Returns:
        True on success
    """
    if not isinstance(snapshot_root, Path):
        snapshot_root = Path(snapshot_root)
    snap_dir = snapshot_root / snapshot_id
    manifest_path = snap_dir / "manifest.json"
    if not snap_dir.exists() or not manifest_path.exists():
        raise SnapshotRestoreError(
            f"Snapshot not found or missing manifest: {snap_dir}"
        )

    manifest = _read_json(manifest_path)
    if manifest.get("schema_version") != "brain-snapshot-v1":
        raise SnapshotRestoreError("Unsupported snapshot schema_version")

    # Pre-restore: perform deterministic cleanup of existing connectome state
    try:
        cm = connectome_manager
        if cm is not None:
            # Clear cortical areas and mapping state
            if hasattr(cm, "cortical_areas") and isinstance(
                cm.cortical_areas, dict
            ):
                cm.cortical_areas.clear()
            if hasattr(cm, "cortical_mapping") and hasattr(
                cm.cortical_mapping, "clear"
            ):
                cm.cortical_mapping.clear()
            if hasattr(cm, "cortical_connections"):
                cm.cortical_connections = {}
            # Clear id/index mappings
            if hasattr(cm, "_neuron_id_to_index_map"):
                cm._neuron_id_to_index_map.clear()
            if hasattr(cm, "_index_to_neuron_id_map"):
                cm._index_to_neuron_id_map.clear()
            # Reset spatial structures if present
            if hasattr(cm, "_spatial_hash") and cm._spatial_hash:
                try:
                    cm._spatial_hash.clear()
                except Exception:
                    pass
            if hasattr(cm, "_spatial_index") and cm._spatial_index:
                try:
                    cm._spatial_index.clear()
                except Exception:
                    pass
            # Reset arrays to empty-valid state where possible
            try:
                na = getattr(cm, "neuron_array", None)
                if na is not None:
                    # Reset counters and masks
                    if hasattr(na, "next_index"):
                        na.next_index = 0
                    if hasattr(na, "neuron_count"):
                        na.neuron_count = 0
                    if hasattr(na, "free_indices"):
                        na.free_indices = set()
                    if hasattr(na, "valid_mask"):
                        na.valid_mask[:] = False
                    if hasattr(na, "is_active"):
                        na.is_active[:] = False
            except Exception:
                pass
            try:
                sa = getattr(cm, "synapse_array", None)
                if sa is not None:
                    for attr in ("next_slot",):
                        if hasattr(sa, attr):
                            setattr(sa, attr, 0)
                    for attr in ("free_slots",):
                        if hasattr(sa, attr):
                            setattr(sa, attr, set())
            except Exception:
                pass
            # Clear FCL caches
            fclm = getattr(cm, "fcl_manager", None)
            if fclm is not None and hasattr(fclm, "clear_all_window_caches"):
                try:
                    fclm.clear_all_window_caches()
                except Exception:
                    pass
            # Invalidate StateManager caches
            try:
                sm = state_manager
                if hasattr(sm, "invalidate_cortical_areas_cache"):
                    sm.invalidate_cortical_areas_cache()
                if hasattr(sm, "set_cortical_list"):
                    sm.set_cortical_list([])
            except Exception:
                pass
    except Exception:
        pass

    files = manifest.get("files", {})
    connectome_path = snap_dir / files.get("connectome", "connectome.json")
    state_path = snap_dir / files.get("state", "state.json")

    # Integrity verification if checksums present
    checksums = manifest.get("checksums", {})
    if isinstance(checksums, dict) and checksums:
        # Verify any known file listed in checksums
        def _verify(path: Path, key: str):
            expected = checksums.get(key)
            if expected:
                actual = _blake2b_hex(path)
                if actual != expected:
                    raise SnapshotRestoreError(f"Checksum mismatch for {key}")

        if connectome_path.exists():
            _verify(connectome_path, "connectome.json")
        if state_path.exists():
            _verify(state_path, "state.json")
        # Optional NPZ checks
        for category in ("neurons", "synapses", "memory"):
            entry = files.get(category)
            if isinstance(entry, list):
                for rel in entry:
                    p = snap_dir / rel
                    if p.exists():
                        _verify(p, rel)

    # Read summaries (connectome currently informational)
    # Rehydrate cortical areas from connectome.json
    if connectome_path.exists() and connectome_manager is not None:
        try:
            from feagi.bdu.models.cortical_area import CorticalArea

            connectome_data = _read_json(connectome_path)
            areas_data = connectome_data.get("cortical_areas", [])
            # Rehydrate cortical areas with proper names, types, and properties
            for idx, entry in enumerate(areas_data):
                try:
                    cid = entry.get("id", f"area_{idx}")
                    dims = entry.get("dimensions", [1, 1, 1])
                    pos = entry.get("position", [0, 0, 0])
                    if isinstance(dims, list) and len(dims) >= 3:
                        dims = dims[:3]
                    else:
                        dims = [1, 1, 1]
                    if isinstance(pos, list) and len(pos) >= 3:
                        pos = pos[:3]
                    elif isinstance(pos, dict):
                        pos = [
                            pos.get("x", 0),
                            pos.get("y", 0),
                            pos.get("z", 0),
                        ]
                    cidx_val = entry.get("cortical_idx")
                    cortical_idx = (
                        int(cidx_val) if isinstance(cidx_val, int) else idx
                    )
                    area_name = entry.get("name") or cid
                    area_type = (
                        entry.get("area_type") or entry.get("type") or "custom"
                    )
                    area_props = (
                        entry.get("properties")
                        or entry.get("parameters")
                        or {}
                    )
                    area = CorticalArea(
                        name=area_name,
                        dimensions=tuple(dims),
                        position=tuple(pos),
                        area_type=area_type,
                        properties=area_props,
                        cortical_id=cid,
                        cortical_idx=cortical_idx,
                    )
                    if hasattr(connectome_manager, "cortical_areas"):
                        # Store areas keyed by cortical_id (string), not index
                        connectome_manager.cortical_areas[cid] = area
                    if hasattr(connectome_manager, "_sync_cortical_mapping"):
                        connectome_manager._sync_cortical_mapping(
                            cid, cortical_idx
                        )
                except Exception:
                    continue
        except Exception:
            pass

    state_data: Dict[str, Any] = {}
    if state_path.exists():
        state_data = _read_json(state_path)

    # Apply minimal state: brain stats (non-destructive)
    try:
        stats = state_data.get("stats", {})
        if isinstance(stats, dict) and state_manager:
            current = state_manager.get_brain_stats() or {}
            merged = dict(current)
            for k in (
                "neuron_count",
                "memory_neuron_count",
                "non_memory_neuron_count",
            ):
                if k in stats and isinstance(stats[k], int):
                    merged[k] = stats[k]
            state_manager.set_brain_stats(merged)
    except Exception:
        pass

    # Reset cumulative activity counters on restore for safety
    try:
        if state_manager and hasattr(
            state_manager, "reset_cumulative_activity"
        ):
            state_manager.reset_cumulative_activity()
    except Exception:
        pass

    # Restore SoA arrays from NPZ if present and a connectome manager is provided
    try:
        import numpy as np  # noqa: F401

        if connectome_manager is not None:
            # Neurons
            neurons_entry = files.get("neurons")
            if isinstance(neurons_entry, list):
                for rel in neurons_entry:
                    if rel.endswith("neurons_soa.npz"):
                        npz = np.load(snap_dir / rel)
                        # Optional index map for sparse compaction
                        idx_map_path = snap_dir / "neurons" / "index_map.npy"
                        index_map = None
                        if idx_map_path.exists():
                            index_map = np.load(idx_map_path)
                            if index_map.dtype != np.uint32:
                                raise SnapshotRestoreError(
                                    "neurons/index_map.npy must be uint32"
                                )
                        if hasattr(connectome_manager, "neuron_array"):
                            na = connectome_manager.neuron_array
                            # Scatter or assign each field
                            for key in npz.files:
                                try:
                                    arr = npz[key]
                                    if key.endswith("__bitpacked"):
                                        continue
                                    bitpacked_flag = f"{key}__bitpacked"
                                    if bitpacked_flag in npz.files:
                                        # Unpack bit-packed booleans
                                        arr = np.unpackbits(arr, count=None)
                                    if index_map is not None:
                                        # Scatter into existing capacity (or ensure length)
                                        target = getattr(na, key, None)
                                        if target is None:
                                            setattr(na, key, arr)
                                        else:
                                            if (
                                                index_map.size
                                                > target.shape[0]
                                            ):
                                                raise SnapshotRestoreError(
                                                    (
                                                        f"Neuron target '{key}' smaller "
                                                        "than index map"
                                                    )
                                                )
                                            target[index_map] = arr[
                                                : index_map.size
                                            ]
                                    else:
                                        setattr(na, key, arr)
                                except Exception as e:
                                    raise SnapshotRestoreError(
                                        f"Failed to restore neuron array '{key}': {e}"
                                    ) from e
            # Synapses
            syn_entry = files.get("synapses")
            if isinstance(syn_entry, list):
                for rel in syn_entry:
                    if rel.endswith("synapses_soa.npz"):
                        npz = np.load(snap_dir / rel)
                        idx_map_path = snap_dir / "synapses" / "index_map.npy"
                        index_map = None
                        if idx_map_path.exists():
                            index_map = np.load(idx_map_path)
                            if index_map.dtype != np.uint32:
                                raise SnapshotRestoreError(
                                    "synapses/index_map.npy must be uint32"
                                )
                        if hasattr(connectome_manager, "synapse_array"):
                            sa = connectome_manager.synapse_array
                            for key in npz.files:
                                try:
                                    arr = npz[key]
                                    if key.endswith("__bitpacked"):
                                        continue
                                    if f"{key}__bitpacked" in npz.files:
                                        arr = np.unpackbits(arr, count=None)
                                    if index_map is not None:
                                        target = getattr(sa, key, None)
                                        if target is None:
                                            setattr(sa, key, arr)
                                        else:
                                            if (
                                                index_map.size
                                                > target.shape[0]
                                            ):
                                                raise SnapshotRestoreError(
                                                    (
                                                        f"Synapse target '{key}' smaller "
                                                        "than index map"
                                                    )
                                                )
                                            target[index_map] = arr[
                                                : index_map.size
                                            ]
                                    else:
                                        setattr(sa, key, arr)
                                except Exception as e:
                                    raise SnapshotRestoreError(
                                        f"Failed to restore synapse array '{key}': {e}"
                                    ) from e
            else:
                pass
            # Memory
            mem_entry = files.get("memory")
            if isinstance(mem_entry, list):
                for rel in mem_entry:
                    if rel.endswith("memory_soa.npz"):
                        npz = np.load(snap_dir / rel)
                        idx_map_path = snap_dir / "memory" / "index_map.npy"
                        index_map = None
                        if idx_map_path.exists():
                            index_map = np.load(idx_map_path)
                            if index_map.dtype != np.uint32:
                                raise SnapshotRestoreError(
                                    "memory/index_map.npy must be uint32"
                                )
                        if hasattr(connectome_manager, "memory_neuron_array"):
                            ma = connectome_manager.memory_neuron_array
                            for key in npz.files:
                                try:
                                    arr = npz[key]
                                    if key.endswith("__bitpacked"):
                                        continue
                                    if f"{key}__bitpacked" in npz.files:
                                        arr = np.unpackbits(arr, count=None)
                                    if index_map is not None:
                                        target = getattr(ma, key, None)
                                        if target is None:
                                            setattr(ma, key, arr)
                                        else:
                                            if (
                                                index_map.size
                                                > target.shape[0]
                                            ):
                                                raise SnapshotRestoreError(
                                                    (
                                                        f"Memory target '{key}' smaller "
                                                        "than index map"
                                                    )
                                                )
                                            target[index_map] = arr[
                                                : index_map.size
                                            ]
                                    else:
                                        setattr(ma, key, arr)
                                except Exception as e:
                                    raise SnapshotRestoreError(
                                        f"Failed to restore memory array '{key}': {e}"
                                    ) from e
            else:
                pass

            # Post-restore: rebuild coordinate indices and id/index mappings (best-effort)
            try:
                na = getattr(connectome_manager, "neuron_array", None)
                areas = getattr(connectome_manager, "cortical_areas", {})
                if na is not None and isinstance(areas, dict) and areas:
                    # Clear and rebuild neuron_id/index maps
                    try:
                        connectome_manager._neuron_id_to_index_map.clear()
                        connectome_manager._index_to_neuron_id_map.clear()
                    except Exception:
                        pass
                    # Try to load index_to_id mapping from snapshot
                    idx2id_arr = None
                    try:
                        idx2id_path = snap_dir / "neurons" / "index_to_id.npy"
                        if idx2id_path.exists():
                            import numpy as _np

                            idx2id_arr = _np.load(idx2id_path)
                    except Exception:
                        idx2id_arr = None
                    # Reindex per area
                    for cortical_id, area in areas.items():
                        try:
                            # Reset position maps
                            if hasattr(area, "_position_map"):
                                area._position_map.clear()
                            if hasattr(area, "_position_to_neurons"):
                                area._position_to_neurons.clear()
                            # Collect all neurons belonging to this area by cortical_idx
                            cidx = getattr(area, "cortical_idx", None)
                            if cidx is None:
                                continue
                            # neuron_array expected fields
                            idxs = []
                            try:
                                # Vectorized mask by cortical_idx if available
                                import numpy as _np

                                if hasattr(na, "cortical_idxs") and hasattr(
                                    na, "valid_mask"
                                ):
                                    mask = (
                                        _np.asarray(na.cortical_idxs)
                                        == int(cidx)
                                    ) & (
                                        _np.asarray(
                                            na.valid_mask, dtype=_np.bool_
                                        )
                                    )
                                    idxs = _np.flatnonzero(mask).tolist()
                            except Exception:
                                idxs = []
                            # Populate maps for each index
                            for i in idxs:
                                try:
                                    nid = -1
                                    if idx2id_arr is not None:
                                        # If index map was present, indices array may be sparse; fall back to direct position if size permits
                                        if int(i) < idx2id_arr.shape[0]:
                                            nid = int(idx2id_arr[int(i)])
                                    if nid < 0:
                                        # Fallback to neuron_array mapping if available
                                        nid = int(
                                            getattr(
                                                na, "index_to_id_map", {}
                                            ).get(int(i), -1)
                                        )
                                    if nid < 0:
                                        continue
                                    # Update global maps
                                    connectome_manager._neuron_id_to_index_map[
                                        nid
                                    ] = int(i)
                                    connectome_manager._index_to_neuron_id_map[
                                        int(i)
                                    ] = nid
                                    # Position
                                    pos = None
                                    try:
                                        x = int(na.coordinates_x[i])
                                        y = int(na.coordinates_y[i])
                                        z = int(na.coordinates_z[i])
                                        pos = (x, y, z)
                                    except Exception:
                                        pos = None
                                    if pos is not None:
                                        if hasattr(area, "_position_map"):
                                            area._position_map[nid] = pos
                                        if hasattr(
                                            area, "_position_to_neurons"
                                        ):
                                            lst = (
                                                area._position_to_neurons.get(
                                                    pos
                                                )
                                            )
                                            if lst is None:
                                                area._position_to_neurons[
                                                    pos
                                                ] = [nid]
                                            else:
                                                lst.append(nid)
                                except Exception:
                                    continue
                        except Exception:
                            # Skip area on error and continue rebuilding others
                            continue
                    # Refresh FCL window caches (safe conditional)
                    fclm = getattr(connectome_manager, "fcl_manager", None)
                    if fclm is not None and hasattr(
                        fclm, "clear_all_window_caches"
                    ):
                        try:
                            fclm.clear_all_window_caches()
                        except Exception:
                            pass
                    # Refresh StateManager cortical areas cache to synchronize with restored areas
                    try:
                        sm = state_manager
                        if sm and hasattr(sm, "set_cortical_list"):
                            # Build cortical list from restored areas
                            area_list = []
                            if hasattr(connectome_manager, "cortical_areas"):
                                for (
                                    cid,
                                    area,
                                ) in connectome_manager.cortical_areas.items():
                                    area_list.append(cid)
                            sm.set_cortical_list(area_list)
                        if sm and hasattr(
                            sm, "invalidate_cortical_areas_cache"
                        ):
                            sm.invalidate_cortical_areas_cache()
                        if sm and hasattr(sm, "get_cortical_areas_cache"):
                            # Force cache refresh
                            _ = sm.get_cortical_areas_cache(connectome_manager)
                    except Exception:
                        pass
            except Exception:
                # Non-fatal; core restore succeeded
                pass
    except SnapshotRestoreError:
        raise
    except Exception:
        # Best-effort for SoA restore to avoid breaking minimal snapshot restores
        pass

    return True
