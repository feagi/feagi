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
    snapshot_root: Path, snapshot_id: str, state_manager
) -> bool:
    """
    Restore a minimal brain snapshot (phase 1) from snapshot_root/snapshot_id.

    Args:
        snapshot_root: Base directory containing snapshots
        snapshot_id: Snapshot directory name
        state_manager: FeagiStateManager instance

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

    return True 