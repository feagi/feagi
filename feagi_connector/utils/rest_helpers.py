"""
HTTP REST helpers for FEAGI control-plane endpoints used by agents.

These helpers intentionally import 'requests' lazily to avoid hard dependencies
at import time. Callers should ensure 'requests' is available in their runtime
environment.

All functions are OS-agnostic and avoid hardcoding environment variables.
"""

from __future__ import annotations

from typing import Dict, List, Tuple


def _requests():
    try:
        import requests  # type: ignore
        return requests
    except Exception as e:
        raise RuntimeError("The 'requests' package is required for REST helpers.") from e


def get_cortical_dimensions(host: str, rest_port: int, cortical_ids: List[str]) -> Dict[str, Tuple[int, int]]:
    """Fetch cortical area dimensions for one or more cortical IDs.

    Args:
        host: FEAGI host, e.g., "127.0.0.1".
        rest_port: FEAGI REST port, e.g., 8000.
        cortical_ids: List of cortical IDs to query.

    Returns:
        Mapping of cortical_id -> (width, height). Missing entries are omitted.
    """
    if not cortical_ids:
        return {}
    requests = _requests()
    url = f"http://{host}:{int(rest_port)}/v1/cortical_area/multi/cortical_area_properties"
    try:
        r = requests.post(url, json={"cortical_ids": list(cortical_ids)}, timeout=5.0)
        if r.status_code != 200:
            return {}
        data = r.json() or []
        out: Dict[str, Tuple[int, int]] = {}
        for item in data:
            try:
                cid = str(item.get("cortical_id") or item.get("id") or "").strip()
                dims = item.get("dimensions") or item.get("cortical_dimensions") or [64, 64, 1]
                out[cid] = (int(dims[0]), int(dims[1]))
            except Exception:
                continue
        return out
    except Exception:
        return {}


def get_segmented_3x3_dimensions(host: str, rest_port: int) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    """Fetch center and peripheral cortical dimensions for 3x3 segmented vision.

    Returns ((center_w, center_h), (per_w, per_h)).
    """
    ids = [
        "iic400",  # center
        "iic600", "iic700", "iic800",
        "iic300", "iic500",
        "iic000", "iic100", "iic200",
    ]
    dims = get_cortical_dimensions(host, rest_port, ids)
    center = dims.get("iic400", (64, 64))
    # choose any available peripheral dims; default to center if none
    per = None
    for pid in ["iic600", "iic700", "iic800", "iic300", "iic500", "iic000", "iic100", "iic200"]:
        if pid in dims:
            per = dims[pid]
            break
    if per is None:
        per = center
    return (int(center[0]), int(center[1])), (int(per[0]), int(per[1]))


def set_simulation_timestep(host: str, rest_port: int, timestep_seconds: float) -> bool:
    """Set FEAGI simulation timestep via REST.

    Args:
        host: FEAGI host.
        rest_port: FEAGI REST port.
        timestep_seconds: Desired timestep in seconds (1/FPS).

    Returns True on HTTP 200, False otherwise.
    """
    requests = _requests()
    url = f"http://{host}:{int(rest_port)}/v1/burst_engine/simulation_timestep"
    try:
        r = requests.post(url, json={"simulation_timestep": float(timestep_seconds)}, timeout=5.0)
        return r.status_code == 200
    except Exception:
        return False


