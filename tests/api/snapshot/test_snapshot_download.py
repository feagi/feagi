import io
import json
import os
from pathlib import Path
import zipfile

import pytest
from fastapi.testclient import TestClient
from fastapi import FastAPI


@pytest.fixture()
def fake_snapshot(tmp_path: Path):
    root = tmp_path / "snapshots"
    snap_id = "test-snap-001"
    snap_dir = root / snap_id
    (snap_dir / "neurons").mkdir(parents=True, exist_ok=True)
    (snap_dir / "synapses").mkdir(parents=True, exist_ok=True)
    (snap_dir / "memory").mkdir(parents=True, exist_ok=True)

    # Create some files
    (snap_dir / "connectome.json").write_text(json.dumps({"ok": True}), encoding="utf-8")
    (snap_dir / "state.json").write_text(json.dumps({"burst": 123}), encoding="utf-8")
    (snap_dir / "neurons" / "area_1.npz").write_bytes(b"NPZDATA1")
    (snap_dir / "synapses" / "global_coo.npz").write_bytes(b"NPZDATA2")
    (snap_dir / "memory" / "memory_soa.npz").write_bytes(b"NPZDATA3")

    manifest = {
        "schema_version": "brain-snapshot-v1",
        "files": {
            "connectome": "connectome.json",
            "state": "state.json",
            "neurons": ["neurons/area_1.npz"],
            "synapses": ["synapses/global_coo.npz"],
            "memory": ["memory/memory_soa.npz"],
        },
    }
    (snap_dir / "manifest.json").write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")
    return root, snap_id


@pytest.fixture()
def configure_snapshot_env(monkeypatch, tmp_path: Path):
    # Configure snapshot section via a temporary TOML file env var, but we leverage runtime monkeypatch on loader
    out_dir = tmp_path / "snapshots"
    temp_dir = tmp_path / "tmp"
    out_dir.mkdir(parents=True, exist_ok=True)
    temp_dir.mkdir(parents=True, exist_ok=True)

    # Monkeypatch the cached config loader to inject snapshot config
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


@pytest.mark.integration
def test_snapshot_download_endpoint(fake_snapshot, configure_snapshot_env, monkeypatch):
    root, snap_id = fake_snapshot
    out_dir, temp_dir = configure_snapshot_env

    # Move the prepared snapshot into configured root
    (out_dir / snap_id).mkdir(parents=True, exist_ok=True)
    # Copy files
    for path in (root / snap_id).rglob("*"):
        if path.is_file():
            rel = path.relative_to(root / snap_id)
            dest = (out_dir / snap_id / rel)
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(path.read_bytes())

    # Build a minimal app and include only the snapshot router
    # Patch dependency to provide a dummy CoreAPIService
    from unittest.mock import MagicMock
    import feagi.api.transport.universal_fastapi as uf
    monkeypatch.setattr(uf, "get_core_api_service", lambda: MagicMock())

    from feagi.api.transport.universal_fastapi import UniversalFastAPIWrapper

    app = FastAPI()
    snapshot_router = UniversalFastAPIWrapper().create_router_for_module("snapshot")
    app.include_router(snapshot_router, prefix="/v1/snapshot")

    client = TestClient(app)

    # Trigger endpoint
    resp = client.get(f"/v1/snapshot/{snap_id}/download")
    assert resp.status_code == 200
    assert resp.headers.get("content-type", "").startswith("application/zip")
    cd = resp.headers.get("content-disposition", "")
    assert "attachment" in cd.lower() and snap_id in cd

    # Verify zip content
    zbytes = io.BytesIO(resp.content)
    with zipfile.ZipFile(zbytes, mode="r") as zf:
        names = sorted(zf.namelist())
        # Must contain manifest and the files declared
        assert "manifest.json" in names
        assert "connectome.json" in names
        assert "state.json" in names
        assert "neurons/area_1.npz" in names
        assert "synapses/global_coo.npz" in names
        assert "memory/memory_soa.npz" in names

        # Check contents of a small JSON file
        with zf.open("state.json") as f:
            data = json.loads(f.read().decode("utf-8"))
            assert data["burst"] == 123 