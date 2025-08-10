"""
Brain snapshot creation utilities.

This module creates a deterministic-on-content minimal snapshot of the brain
sufficient for packaging and transport. It currently writes:
- manifest.json
- connectome.json (structure summary)
- state.json (global brain and runtime markers)

Note: This is phase 1. Full neuron/synapse/memory SoA serialization to be added.
"""
from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict


def _now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _safe_mkdirs(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def _build_connectome_summary(connectome_manager) -> Dict[str, Any]:
    """
    Build a JSON-safe summary of the connectome. Avoids non-serializable objects.
    """
    try:
        areas = []
        if hasattr(connectome_manager, "cortical_areas") and isinstance(
            connectome_manager.cortical_areas, dict
        ):
            for area_id, area_obj in connectome_manager.cortical_areas.items():
                entry = {"id": str(area_id)}
                try:
                    dims = getattr(area_obj, "dimensions", None)
                    if dims is not None:
                        entry["dimensions"] = list(dims)
                except Exception:
                    pass
                areas.append(entry)
        mappings = {}
        if hasattr(connectome_manager, "cortical_mapping"):
            try:
                m = connectome_manager.cortical_mapping.get_all_mappings()
                mappings = {
                    str(k): (list(v) if not isinstance(v, dict) else v)
                    for k, v in m.items()
                }
            except Exception:
                mappings = {}
        return {"areas": areas, "mappings": mappings}
    except Exception:
        return {"areas": [], "mappings": {}}


def _build_state_summary(state_manager) -> Dict[str, Any]:
    try:
        stats = state_manager.get_brain_stats() if state_manager else {}
    except Exception:
        stats = {}
    if not isinstance(stats, dict):
        stats = {}
    try:
        counters = state_manager.get_cumulative_activity() if state_manager else {}
    except Exception:
        counters = {}
    if not isinstance(counters, dict):
        counters = {}
    return {
        "created_at": _now_iso(),
        "stats": stats,
        "cumulative_activity": counters,
    }


def create_brain_snapshot(
    connectome_manager,
    state_manager,
    output_dir: Path,
    snapshot_id: str | None = None,
) -> Path:
    """
    Create a minimal brain snapshot under output_dir and return its directory path.

    Args:
        connectome_manager: Active ConnectomeManager
        state_manager: FeagiStateManager
        output_dir: Root directory for snapshots
        snapshot_id: Optional externally provided ID; defaults to timestamp-based

    Returns:
        Path to the created snapshot directory
    """
    if not isinstance(output_dir, Path):
        output_dir = Path(output_dir)
    _safe_mkdirs(output_dir)

    if not snapshot_id:
        ts = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
        snapshot_id = f"brain-{ts}"

    snap_dir = output_dir / snapshot_id
    # Ensure unique snapshot directory by adding a numeric suffix if needed
    if snap_dir.exists():
        counter = 1
        base = snapshot_id
        while (output_dir / f"{base}-{counter}").exists():
            counter += 1
        snapshot_id = f"{base}-{counter}"
        snap_dir = output_dir / snapshot_id
    _safe_mkdirs(snap_dir)

    connectome_json = _build_connectome_summary(connectome_manager)
    state_json = _build_state_summary(state_manager)

    (snap_dir / "connectome.json").write_text(
        json.dumps(connectome_json, separators=(",", ":")), encoding="utf-8"
    )
    (snap_dir / "state.json").write_text(
        json.dumps(state_json, separators=(",", ":")), encoding="utf-8"
    )

    # Compute checksums for integrity verification
    from hashlib import blake2b
    c_bytes = (snap_dir / "connectome.json").read_bytes()
    s_bytes = (snap_dir / "state.json").read_bytes()
    checksums = {
        "connectome.json": blake2b(c_bytes, digest_size=32).hexdigest(),
        "state.json": blake2b(s_bytes, digest_size=32).hexdigest(),
    }

    manifest = {
        "schema_version": "brain-snapshot-v1",
        "created_at": _now_iso(),
        "files": {
            "connectome": "connectome.json",
            "state": "state.json",
        },
        "checksums": checksums,
    }
    (snap_dir / "manifest.json").write_text(
        json.dumps(manifest, separators=(",", ":")), encoding="utf-8"
    )

    return snap_dir 