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
    snapshot_root: Path, snapshot_id: str, state_manager, connectome_manager=None
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
    if connectome_path.exists():
        _ = _read_json(connectome_path)

    state_data: Dict[str, Any] = {}
    if state_path.exists():
        state_data = _read_json(state_path)

    # Apply minimal state: brain stats (non-destructive)
    try:
        stats = state_data.get("stats", {})
        if isinstance(stats, dict) and state_manager:
            current = state_manager.get_brain_stats() or {}
            merged = dict(current)
            for k in ("neuron_count", "memory_neuron_count", "non_memory_neuron_count"):
                if k in stats and isinstance(stats[k], int):
                    merged[k] = stats[k]
            state_manager.set_brain_stats(merged)
    except Exception:
        pass

    # Reset cumulative activity counters on restore for safety
    try:
        if state_manager and hasattr(state_manager, "reset_cumulative_activity"):
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
                                            if index_map.size > target.shape[0]:
                                                raise SnapshotRestoreError(
                                                    (
                                                        f"Neuron target '{key}' smaller "
                                                        "than index map"
                                                    )
                                                )
                                            target[index_map] = arr[: index_map.size]
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
                                            if index_map.size > target.shape[0]:
                                                raise SnapshotRestoreError(
                                                    (
                                                        f"Synapse target '{key}' smaller "
                                                        "than index map"
                                                    )
                                                )
                                            target[index_map] = arr[: index_map.size]
                                    else:
                                        setattr(sa, key, arr)
                                except Exception as e:
                                    raise SnapshotRestoreError(
                                        f"Failed to restore synapse array '{key}': {e}"
                                    ) from e
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
                                            if index_map.size > target.shape[0]:
                                                raise SnapshotRestoreError(
                                                    (
                                                        f"Memory target '{key}' smaller "
                                                        "than index map"
                                                    )
                                                )
                                            target[index_map] = arr[: index_map.size]
                                    else:
                                        setattr(ma, key, arr)
                                except Exception as e:
                                    raise SnapshotRestoreError(
                                        f"Failed to restore memory array '{key}': {e}"
                                    ) from e
    except SnapshotRestoreError:
        raise
    except Exception:
        # Best-effort for SoA restore to avoid breaking minimal snapshot restores
        pass

    return True 