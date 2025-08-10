import json
from pathlib import Path

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient


@pytest.fixture()
def configure_snapshot_env(monkeypatch, tmp_path: Path):
    out_dir = tmp_path / "snapshots"
    temp_dir = tmp_path / "tmp"
    out_dir.mkdir(parents=True, exist_ok=True)
    temp_dir.mkdir(parents=True, exist_ok=True)

    from feagi.config import toml_loader as tl

    original_load = tl.load_feagi_config

    def _patched_load(cli_args=None):
        cfg = original_load(cli_args=cli_args)
        cfg = dict(cfg)
        cfg.setdefault("snapshot", {})
        cfg["snapshot"].update(
            {
                "output_dir": str(out_dir),
                "temp_dir": str(temp_dir),
                "zip_compression": "deflate",
            }
        )
        return cfg

    monkeypatch.setattr(tl, "load_feagi_config", _patched_load)
    return out_dir, temp_dir


@pytest.fixture()
def test_app(monkeypatch):
    # Minimal app with snapshot router
    from unittest.mock import MagicMock
    import feagi.api.transport.universal_fastapi as uf

    # Provide a fake core_api_service with state manager and connectome manager
    cmsvc = MagicMock()
    # Ensure state manager methods return real dicts for JSON serialization
    sm = MagicMock()
    sm.get_brain_stats.return_value = {}
    sm.get_cumulative_activity.return_value = {"bursts": 0, "neurons": 0}
    cmsvc.state_manager = sm
    # Patch the dependency used by manual router Depends
    monkeypatch.setattr(uf, "get_core_api_service", lambda: cmsvc)

    from feagi.api.transport.universal_fastapi import UniversalFastAPIWrapper

    app = FastAPI()
    router = UniversalFastAPIWrapper().create_router_for_module("snapshot")
    app.include_router(router, prefix="/v1/snapshots")
    return TestClient(app)


def test_create_zip_and_download_stream(configure_snapshot_env, test_app):
    client = test_app
    # Create zip without persisting artifact
    resp = client.post("/v1/snapshots", json={"format": "zip", "compression": "store", "persist_artifact": False})
    assert resp.status_code == 200
    payload = resp.json()
    sid = payload["snapshot_id"]
    # Download zip artifact (on-demand)
    dl = client.get(f"/v1/snapshots/{sid}/artifact/zip")
    assert dl.status_code == 200
    assert dl.headers.get("content-type", "").startswith("application/zip")


def test_create_fc_persist_and_download(configure_snapshot_env, test_app):
    client = test_app
    # Create fc and persist artifact
    resp = client.post("/v1/snapshots", json={"format": "fc", "compression": "store", "persist_artifact": True})
    assert resp.status_code == 200
    payload = resp.json()
    sid = payload["snapshot_id"]
    # Download fc (should be present)
    dl = client.get(f"/v1/snapshots/{sid}/artifact/fc")
    assert dl.status_code == 200
    cd = dl.headers.get("content-disposition", "")
    assert sid in cd


def test_restore_prefers_fc(configure_snapshot_env, test_app):
    client = test_app
    # Create fc and persist artifact
    resp = client.post("/v1/snapshots", json={"format": "fc", "compression": "store", "persist_artifact": True})
    assert resp.status_code == 200
    sid = resp.json()["snapshot_id"]
    # Restore should succeed
    rr = client.post(f"/v1/snapshots/{sid}/restore")
    assert rr.status_code == 200
    assert rr.json().get("success") is True


def test_list_and_delete_snapshots(configure_snapshot_env, test_app):
    out_dir, _ = configure_snapshot_env
    client = test_app
    # Create two snapshots
    r1 = client.post("/v1/snapshots", json={"format": "zip", "compression": "store", "persist_artifact": False})
    r2 = client.post("/v1/snapshots", json={"format": "fc", "compression": "store", "persist_artifact": True})
    assert r1.status_code == 200 and r2.status_code == 200
    sid1 = r1.json()["snapshot_id"]
    sid2 = r2.json()["snapshot_id"]
    # List should contain both
    ls = client.get("/v1/snapshots")
    assert ls.status_code == 200
    items = ls.json().get("snapshots", [])
    assert sid1 in items and sid2 in items
    # Delete one
    dl = client.delete(f"/v1/snapshots/{sid1}")
    assert dl.status_code == 200 and dl.json().get("deleted") is True
    # Ensure it is gone
    ls2 = client.get("/v1/snapshots")
    items2 = ls2.json().get("snapshots", [])
    assert sid1 not in items2 and sid2 in items2


def test_restore_folder_checksum_mismatch(configure_snapshot_env, test_app):
    out_dir, _ = configure_snapshot_env
    client = test_app
    # Create a folder-based snapshot (no persisted artifact)
    r = client.post("/v1/snapshots", json={"format": "zip", "compression": "store", "persist_artifact": False})
    assert r.status_code == 200
    sid = r.json()["snapshot_id"]
    snap_dir = Path(r.json()["path"])  # use server-returned path to avoid fixture mismatch
    # Corrupt state.json to break checksum
    (snap_dir / "state.json").write_text("{}", encoding="utf-8")
    # Attempt restore -> should fail with 400
    rr = client.post(f"/v1/snapshots/{sid}/restore")
    assert rr.status_code == 400 