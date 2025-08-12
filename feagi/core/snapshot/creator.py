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
            for idx, area_obj in connectome_manager.cortical_areas.items():
                entry = {
                    "id": getattr(area_obj, "cortical_id", str(idx)),
                    "index": int(getattr(area_obj, "cortical_idx", idx)),
                    "name": getattr(area_obj, "name", None),
                    "type": getattr(area_obj, "area_type", None),
                    "dimensions": list(getattr(area_obj, "dimensions", (1, 1, 1))),
                    "position": list(getattr(area_obj, "position", (0, 0, 0))),
                    "properties": getattr(area_obj, "properties", {}) or {},
                }
                areas.append(entry)
        # Include mapping graph if available
        mappings = {}
        if hasattr(connectome_manager, "cortical_mapping"):
            try:
                m = connectome_manager.cortical_mapping.get_all_mappings()
                mappings = {str(k): int(v) for k, v in m.items()}
            except Exception:
                mappings = {}
        # Include connection specs if available
        connections = {}
        if hasattr(connectome_manager, "cortical_connections"):
            try:
                # Copy shallow; caller persists JSON
                connections = dict(connectome_manager.cortical_connections)
            except Exception:
                connections = {}
        # Physiology/genome metadata if state manager present
        physiology = {}
        try:
            from feagi.core.state_manager import get_state_manager
            sm = get_state_manager()
            if hasattr(sm, "get_physiology"):
                physiology = sm.get_physiology() or {}
        except Exception:
            physiology = {}
        return {
            "areas": areas,
            "mappings": mappings,
            "connections": connections,
            "physiology": physiology,
        }
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
    # Optional: hierarchical genome and physiology
    genome_json = {}
    try:
        from feagi.api.core.services.core_api_service import CoreAPIService
        # connectome_manager is available; construct a minimal facade to fetch genome
        cas = CoreAPIService(connectome_manager, state_manager)
        g = cas.get_current_genome() or cas.get_genome()
        if isinstance(g, dict) and g:
            genome_json = g
    except Exception:
        genome_json = {}
    physiology_json = connectome_json.get("physiology", {})

    (snap_dir / "connectome.json").write_text(
        json.dumps(connectome_json, separators=(",", ":")), encoding="utf-8"
    )
    (snap_dir / "state.json").write_text(
        json.dumps(state_json, separators=(",", ":")), encoding="utf-8"
    )
    if genome_json:
        (snap_dir / "genome.json").write_text(
            json.dumps(genome_json, separators=(",", ":")), encoding="utf-8"
        )
    if physiology_json:
        (snap_dir / "physiology.json").write_text(
            json.dumps(physiology_json, separators=(",", ":")), encoding="utf-8"
        )

    # Compute checksums for integrity verification
    from hashlib import blake2b
    c_bytes = (snap_dir / "connectome.json").read_bytes()
    s_bytes = (snap_dir / "state.json").read_bytes()
    checksums = {
        "connectome.json": blake2b(c_bytes, digest_size=32).hexdigest(),
        "state.json": blake2b(s_bytes, digest_size=32).hexdigest(),
    }
    if genome_json:
        checksums["genome.json"] = blake2b(
            (snap_dir / "genome.json").read_bytes(), digest_size=32
        ).hexdigest()
    if physiology_json:
        checksums["physiology.json"] = blake2b(
            (snap_dir / "physiology.json").read_bytes(), digest_size=32
        ).hexdigest()

    # Optional: export SoA arrays if available
    files_entry = {
        "connectome": "connectome.json",
        "state": "state.json",
    }
    if genome_json:
        files_entry["genome"] = "genome.json"
    if physiology_json:
        files_entry["physiology"] = "physiology.json"
    try:
        import json as _json

        import numpy as np  # noqa: F401
        # Neurons
        if hasattr(connectome_manager, "neuron_array"):
            na = connectome_manager.neuron_array
            neurons_dir = snap_dir / "neurons"
            _safe_mkdirs(neurons_dir)
            neurons_npz = neurons_dir / "neurons_soa.npz"
            neurons_meta = neurons_dir / "neurons_meta.json"
            neuron_payload = {}
            neuron_meta: Dict[str, Any] = {}
            # Determine active indices (compact save)
            active_idx = None
            try:
                if hasattr(na, "valid_mask") and na.valid_mask is not None:
                    import numpy as _np
                    mask = _np.asarray(na.valid_mask, dtype=_np.bool_)
                    active_idx = _np.flatnonzero(mask)
                elif hasattr(na, "neuron_count") and hasattr(na, "next_index"):
                    import numpy as _np
                    used = int(na.neuron_count)
                    next_idx = int(na.next_index)
                    # Assume dense prefix if no deletions were made
                    prefix = min(used, next_idx)
                    active_idx = _np.arange(prefix, dtype=_np.int64)
            except Exception:
                active_idx = None
            for name in (
                "membrane_potentials",
                "resting_potentials",
                "thresholds",
                "excitability",
                "decay_rates",
                "refractory_periods",
                "refractory_counters",
                "coordinates_x",
                "coordinates_y",
                "coordinates_z",
                "cortical_idxs",
                "is_active",
                "valid_mask",
                "last_fired",
                "neuron_types",
                "enabled_flags",
            ):
                if hasattr(na, name):
                    arr = getattr(na, name)
                    try:
                        import numpy as _np
                        if arr.dtype == _np.bool_:
                            # Bit-pack booleans; optionally slice first
                            if active_idx is not None and active_idx.size:
                                arr = _np.asarray(arr, dtype=_np.bool_)[active_idx]
                            neuron_payload[name] = _np.packbits(arr)
                            neuron_payload[f"{name}__bitpacked"] = _np.array(
                                [1], dtype=_np.uint8
                            )
                            neuron_meta[name] = {
                                "dtype": "bool",
                                "shape": [int(arr.shape[0])],
                                "bitpacked": True,
                            }
                        else:
                            if active_idx is not None and active_idx.size:
                                sliced = _np.asarray(arr)[active_idx]
                                neuron_payload[name] = sliced
                                neuron_meta[name] = {
                                    "dtype": str(sliced.dtype),
                                    "shape": [int(s) for s in sliced.shape],
                                    "bitpacked": False,
                                }
                            else:
                                full = _np.asarray(arr)
                                neuron_payload[name] = full
                                neuron_meta[name] = {
                                    "dtype": str(full.dtype),
                                    "shape": [int(s) for s in full.shape],
                                    "bitpacked": False,
                                }
                    except Exception:
                        neuron_payload[name] = arr
            if neuron_payload:
                import numpy as _np
                # Persist optional index map if not prefix-dense
                if active_idx is not None:
                    # Check prefix-dense condition
                    is_prefix_dense = (
                        active_idx.size > 0 and (active_idx[0] == 0) and (
                            active_idx[-1] == active_idx.size - 1
                        )
                    )
                    if not is_prefix_dense:
                        idx_path = neurons_dir / "index_map.npy"
                        _np.save(idx_path, active_idx.astype(_np.uint32))
                        files_entry.setdefault("neurons", []).append(
                            "neurons/index_map.npy"
                        )
                        checksums["neurons/index_map.npy"] = blake2b(
                            idx_path.read_bytes(), digest_size=32
                        ).hexdigest()
                _np.savez_compressed(neurons_npz, **neuron_payload)
                # Write meta
                neurons_meta.write_text(
                    _json.dumps(neuron_meta, separators=(",", ":")),
                    encoding="utf-8",
                )
                files_entry.setdefault("neurons", []).append("neurons/neurons_soa.npz")
                files_entry.setdefault("neurons", []).append("neurons/neurons_meta.json")
                checksums["neurons/neurons_soa.npz"] = blake2b(
                    neurons_npz.read_bytes(), digest_size=32
                ).hexdigest()
                checksums["neurons/neurons_meta.json"] = blake2b(
                    neurons_meta.read_bytes(), digest_size=32
                ).hexdigest()
        # Synapses
        if hasattr(connectome_manager, "synapse_array"):
            sa = connectome_manager.synapse_array
            syn_dir = snap_dir / "synapses"
            _safe_mkdirs(syn_dir)
            syn_npz = syn_dir / "synapses_soa.npz"
            syn_meta = syn_dir / "synapses_meta.json"
            syn_payload = {}
            syn_meta_obj: Dict[str, Any] = {}
            # Build active index from next_slot and free_slots if available
            syn_active_idx = None
            try:
                import numpy as _np
                next_slot = int(getattr(sa, "next_slot", 0))
                free_slots = set(getattr(sa, "free_slots", []))
                if next_slot > 0:
                    syn_active_idx = _np.array(
                        [i for i in range(next_slot) if i not in free_slots],
                        dtype=_np.int64,
                    )
            except Exception:
                syn_active_idx = None
            for name in (
                "pre_neuron_ids",
                "post_neuron_ids",
                "weights",
                "delays",
                "types",
                "plasticity_coeffs",
                "conductances",
                "is_plastic_flags",
            ):
                if hasattr(sa, name):
                    arr = getattr(sa, name)
                    try:
                        import numpy as _np
                        if arr.dtype == _np.bool_:
                            if syn_active_idx is not None and syn_active_idx.size:
                                arr = _np.asarray(arr, dtype=_np.bool_)[syn_active_idx]
                            syn_payload[name] = _np.packbits(arr)
                            syn_payload[f"{name}__bitpacked"] = _np.array(
                                [1], dtype=_np.uint8
                            )
                            syn_meta_obj[name] = {
                                "dtype": "bool",
                                "shape": [int(arr.shape[0])],
                                "bitpacked": True,
                            }
                        else:
                            if syn_active_idx is not None and syn_active_idx.size:
                                sliced = _np.asarray(arr)[syn_active_idx]
                                syn_payload[name] = sliced
                                syn_meta_obj[name] = {
                                    "dtype": str(sliced.dtype),
                                    "shape": [int(s) for s in sliced.shape],
                                    "bitpacked": False,
                                }
                            else:
                                full = _np.asarray(arr)
                                syn_payload[name] = full
                                syn_meta_obj[name] = {
                                    "dtype": str(full.dtype),
                                    "shape": [int(s) for s in full.shape],
                                    "bitpacked": False,
                                }
                    except Exception:
                        syn_payload[name] = arr
            if syn_payload:
                import numpy as _np
                if syn_active_idx is not None:
                    is_prefix_dense = (
                        syn_active_idx.size > 0 and (syn_active_idx[0] == 0) and (
                            syn_active_idx[-1] == syn_active_idx.size - 1
                        )
                    )
                    if not is_prefix_dense:
                        idx_path = syn_dir / "index_map.npy"
                        _np.save(idx_path, syn_active_idx.astype(_np.uint32))
                        files_entry.setdefault("synapses", []).append(
                            "synapses/index_map.npy"
                        )
                        checksums["synapses/index_map.npy"] = blake2b(
                            idx_path.read_bytes(), digest_size=32
                        ).hexdigest()
                _np.savez_compressed(syn_npz, **syn_payload)
                syn_meta.write_text(
                    _json.dumps(syn_meta_obj, separators=(",", ":")),
                    encoding="utf-8",
                )
                files_entry.setdefault("synapses", []).append(
                    "synapses/synapses_soa.npz"
                )
                files_entry.setdefault("synapses", []).append(
                    "synapses/synapses_meta.json"
                )
                checksums["synapses/synapses_soa.npz"] = blake2b(
                    syn_npz.read_bytes(), digest_size=32
                ).hexdigest()
                checksums["synapses/synapses_meta.json"] = blake2b(
                    syn_meta.read_bytes(), digest_size=32
                ).hexdigest()
        # Memory
        if hasattr(connectome_manager, "memory_neuron_array"):
            ma = connectome_manager.memory_neuron_array
            mem_dir = snap_dir / "memory"
            _safe_mkdirs(mem_dir)
            mem_npz = mem_dir / "memory_soa.npz"
            mem_meta = mem_dir / "memory_meta.json"
            mem_payload = {}
            mem_meta_obj: Dict[str, Any] = {}
            # Active indices for memory neurons
            mem_active_idx = None
            try:
                import numpy as _np
                if hasattr(ma, "is_active"):
                    mem_active_idx = _np.flatnonzero(
                        _np.asarray(ma.is_active, dtype=_np.bool_)
                    )
            except Exception:
                mem_active_idx = None
            for name in (
                "lifespan_current",
                "lifespan_initial",
                "lifespan_growth_rate",
                "is_longterm_memory",
                "creation_burst",
                "last_activation_burst",
                "activation_count",
                "is_active",
            ):
                if hasattr(ma, name):
                    arr = getattr(ma, name)
                    try:
                        import numpy as _np
                        if arr.dtype == _np.bool_:
                            if mem_active_idx is not None and mem_active_idx.size:
                                arr = _np.asarray(arr, dtype=_np.bool_)[mem_active_idx]
                            mem_payload[name] = _np.packbits(arr)
                            mem_payload[f"{name}__bitpacked"] = _np.array(
                                [1], dtype=_np.uint8
                            )
                            mem_meta_obj[name] = {
                                "dtype": "bool",
                                "shape": [int(arr.shape[0])],
                                "bitpacked": True,
                            }
                        else:
                            if mem_active_idx is not None and mem_active_idx.size:
                                sliced = _np.asarray(arr)[mem_active_idx]
                                mem_payload[name] = sliced
                                mem_meta_obj[name] = {
                                    "dtype": str(sliced.dtype),
                                    "shape": [int(s) for s in sliced.shape],
                                    "bitpacked": False,
                                }
                            else:
                                full = _np.asarray(arr)
                                mem_payload[name] = full
                                mem_meta_obj[name] = {
                                    "dtype": str(full.dtype),
                                    "shape": [int(s) for s in full.shape],
                                    "bitpacked": False,
                                }
                    except Exception:
                        mem_payload[name] = arr
            if mem_payload:
                import numpy as _np
                if mem_active_idx is not None:
                    is_prefix_dense = (
                        mem_active_idx.size > 0 and (mem_active_idx[0] == 0) and (
                            mem_active_idx[-1] == mem_active_idx.size - 1
                        )
                    )
                    if not is_prefix_dense:
                        idx_path = mem_dir / "index_map.npy"
                        _np.save(idx_path, mem_active_idx.astype(_np.uint32))
                        files_entry.setdefault("memory", []).append(
                            "memory/index_map.npy"
                        )
                        checksums["memory/index_map.npy"] = blake2b(
                            idx_path.read_bytes(), digest_size=32
                        ).hexdigest()
                _np.savez_compressed(mem_npz, **mem_payload)
                mem_meta.write_text(
                    _json.dumps(mem_meta_obj, separators=(",", ":")),
                    encoding="utf-8",
                )
                files_entry.setdefault("memory", []).append("memory/memory_soa.npz")
                files_entry.setdefault("memory", []).append("memory/memory_meta.json")
                checksums["memory/memory_soa.npz"] = blake2b(
                    mem_npz.read_bytes(), digest_size=32
                ).hexdigest()
                checksums["memory/memory_meta.json"] = blake2b(
                    mem_meta.read_bytes(), digest_size=32
                ).hexdigest()
    except Exception as e:
        # SoA export is best-effort; core snapshot remains valid regardless
        raise RuntimeError(f"SoA export failed: {e}") from e

    manifest = {
        "schema_version": "brain-snapshot-v1",
        "created_at": _now_iso(),
        "files": files_entry,
        "checksums": checksums,
    }
    (snap_dir / "manifest.json").write_text(
        json.dumps(manifest, separators=(",", ":")), encoding="utf-8"
    )

    return snap_dir 