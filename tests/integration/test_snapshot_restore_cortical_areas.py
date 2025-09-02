import json
from pathlib import Path

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from feagi.api.transport.universal_fastapi import UniversalFastAPIWrapper


@pytest.fixture()
def api_client(monkeypatch, tmp_path: Path):
    # Configure snapshot dirs
    out_dir = tmp_path / "snapshots"
    tmp_dir = tmp_path / "tmp"
    out_dir.mkdir(parents=True, exist_ok=True)
    tmp_dir.mkdir(parents=True, exist_ok=True)

    from feagi.config import toml_loader as tl

    orig_load = tl.load_feagi_config

    def _patched(cli_args=None):
        cfg = orig_load(cli_args=cli_args)
        cfg = dict(cfg)
        cfg.setdefault("snapshot", {})
        cfg["snapshot"].update({
            "output_dir": str(out_dir),
            "temp_dir": str(tmp_dir),
            "zip_compression": "deflate",
        })
        return cfg

    monkeypatch.setattr(tl, "load_feagi_config", _patched)

    # Real ConnectomeManager and CoreAPIService
    from feagi.bdu.connectome_manager import ConnectomeManager
    from feagi.api.core.services.core_api_service import CoreAPIService
    from feagi.api.rest.dependencies import (
        set_connectome_instance,
        set_core_api_service_instance,
    )
    from feagi.core.state_manager import FeagiStateManager, ServiceState

    cm = ConnectomeManager.instance(10_000)  # small capacity
    sm = FeagiStateManager.instance()
    # Initialize states for health readiness in test context
    sm.set_burst_engine_state(ServiceState.READY.value)
    sm.set_brain_readiness(True)

    cas = CoreAPIService(cm, sm)
    set_connectome_instance(cm)
    set_core_api_service_instance(cas)

    # Minimal app with routers
    app = FastAPI()
    wrapper = UniversalFastAPIWrapper()
    app.include_router(wrapper.create_router_for_module("cortical_area"), prefix="/v1/cortical_area")
    app.include_router(wrapper.create_router_for_module("snapshot"), prefix="/v1/snapshots")
    return TestClient(app)


def _list_cortical_ids(client: TestClient):
    r = client.get("/v1/cortical_area/cortical_area_id_list")
    assert r.status_code == 200
    body = r.json()
    # Support both wrapped and legacy shapes
    if isinstance(body, dict) and "cortical_ids" in body:
        return body["cortical_ids"]
    if isinstance(body, list):
        return body
    return []


def test_snapshot_restore_cortical_areas_end_to_end(api_client: TestClient, tmp_path: Path):
    client = api_client

    # Baseline IDs
    baseline_ids = _list_cortical_ids(client)

    # Snapshot and persist container
    r_snap = client.post("/v1/snapshots", json={"stateful": False, "compression": True})
    assert r_snap.status_code == 200
    sid = r_snap.json()["snapshot_id"]

    # Restore the snapshot
    r_restore = client.post(f"/v1/snapshots/{sid}/restore", json={"profile": "model", "mode": "load"})
    assert r_restore.status_code == 200

    # Validate IDs
    ids_after_restore = _list_cortical_ids(client)
    assert ids_after_restore == baseline_ids

    # Validate geometry endpoint returns structured data for each cortical_id
    geo = client.get("/v1/cortical_area/cortical_area/geometry")
    assert geo.status_code == 200
    geo_body = geo.json()
    if baseline_ids:
        assert isinstance(geo_body, dict) and geo_body
        for cid in baseline_ids:
            assert cid in geo_body
            entry = geo_body[cid]
            assert "cortical_id" in entry
            assert "cortical_dimensions" in entry and len(entry["cortical_dimensions"]) == 3
            assert "coordinates_3d" in entry and len(entry["coordinates_3d"]) == 3 