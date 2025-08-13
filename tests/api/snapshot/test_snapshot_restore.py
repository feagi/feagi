import json
from pathlib import Path

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient


@pytest.fixture()
def minimal_snapshot(tmp_path: Path):
    root = tmp_path / "snapshots"
    sid = "brain-20250809-120000"
    d = root / sid
    d.mkdir(parents=True, exist_ok=True)
    (d / "connectome.json").write_text(json.dumps({"areas": [], "mappings": {}}), encoding="utf-8")
    state = {"stats": {"neuron_count": 10, "memory_neuron_count": 2, "non_memory_neuron_count": 8}}
    (d / "state.json").write_text(json.dumps(state), encoding="utf-8")
    manifest = {"schema_version": "brain-snapshot-v1", "files": {"connectome": "connectome.json", "state": "state.json"}}
    (d / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    return root, sid


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


def test_restore_snapshot_minimal(minimal_snapshot, configure_snapshot_env, monkeypatch):
    root, sid = minimal_snapshot
    out_dir, _ = configure_snapshot_env

    # Move minimal snapshot into configured root
    (out_dir / sid).mkdir(parents=True, exist_ok=True)
    for p in (root / sid).rglob("*"):
        if p.is_file():
            rel = p.relative_to(root / sid)
            dest = out_dir / sid / rel
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(p.read_bytes())

    # Build minimal app with snapshot router
    from unittest.mock import MagicMock
    import feagi.api.transport.universal_fastapi as uf

    monkeypatch.setattr(uf, "get_core_api_service", lambda: MagicMock(state_manager=MagicMock()))

    from feagi.api.transport.universal_fastapi import UniversalFastAPIWrapper

    app = FastAPI()
    router = UniversalFastAPIWrapper().create_router_for_module("snapshot")
    app.include_router(router, prefix="/v1/snapshot")

    client = TestClient(app)

    resp = client.post("/v1/snapshot/restore", json={"snapshot_id": sid})
    assert resp.status_code == 200
    data = resp.json()
    assert data.get("success") is True 