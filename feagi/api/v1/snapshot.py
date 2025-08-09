"""
FEAGI v1 Snapshot API

Endpoints to manage and retrieve brain snapshots.

- Create snapshot: POST /v1/snapshots (Body: format/compression/persist)
- Download artifact: GET /v1/snapshots/{snapshot_id}/artifact/{fmt}
- Restore: POST /v1/snapshots/{snapshot_id}/restore
"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional

from pydantic import BaseModel, Field

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.api.v1.decorators import endpoint
from feagi.config.toml_loader import load_feagi_config
from feagi.core.snapshot import package_snapshot
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class SnapshotCreateRequest(BaseModel):
    format: str = Field("zip", pattern="^(zip|fc)$")
    compression: Optional[str] = Field(None, pattern="^(store|deflate)$")
    persist_artifact: bool = False


class SnapshotCreateResponse(BaseModel):
    snapshot_id: str
    path: str
    formats_available: Optional[Dict[str, str]] = None  # name->relative path within snapshot folder


class SnapshotRestoreResponse(BaseModel):
    success: bool


def snapshot_endpoint(methods, path, request_model=None, response_model=None, description=None):
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module="snapshot",
    )


class SnapshotAPI:
    def __init__(self, core_api_service: CoreAPIService) -> None:
        self.core_api_service = core_api_service

    @snapshot_endpoint("POST", "/", request_model=SnapshotCreateRequest, response_model=SnapshotCreateResponse)
    async def create_snapshot(self, request: SnapshotCreateRequest) -> Dict[str, Any]:
        """
        Create a snapshot folder and optionally persist an artifact (.fc or .zip).
        """
        try:
            config = load_feagi_config()
            snapshot_cfg = config.get("snapshot", {})
            output_dir = snapshot_cfg.get("output_dir")
            temp_dir = snapshot_cfg.get("temp_dir")
            default_zip_compression = snapshot_cfg.get("zip_compression", "deflate")
            if not output_dir:
                raise ValueError(
                    "Snapshot configuration missing required key: output_dir. "
                    "Add a [snapshot] section to feagi_configuration.toml with keys: "
                    "output_dir, temp_dir, zip_compression (or set FEAGI_CONFIG_PATH to your config)."
                )
            cm = self.core_api_service.get_connectome_manager()
            from feagi.core.snapshot import create_brain_snapshot
            snap_dir = create_brain_snapshot(
                connectome_manager=cm,
                state_manager=self.core_api_service.state_manager,
                output_dir=Path(output_dir),
            )
            snapshot_id = snap_dir.name
            formats_available: Dict[str, str] = {}

            fmt = request.format.lower()
            compression = (request.compression or (default_zip_compression if fmt == "zip" else "store")).lower()

            if request.persist_artifact:
                if fmt == "fc":
                    import json as _json
                    from feagi.core.snapshot.container import create_fc_snapshot

                    connectome_json = _json.loads((snap_dir / "connectome.json").read_text(encoding="utf-8"))
                    state_json = _json.loads((snap_dir / "state.json").read_text(encoding="utf-8"))
                    fc_path = create_fc_snapshot(
                        output_dir=snap_dir,
                        snapshot_id=snapshot_id,
                        connectome_json=connectome_json,
                        state_json=state_json,
                        compression=compression,
                    )
                    formats_available["fc"] = f"{snapshot_id}.fc"
                elif fmt == "zip":
                    if not temp_dir:
                        raise ValueError(
                            "Snapshot configuration missing required key: temp_dir. "
                            "Define it under [snapshot] in feagi_configuration.toml (or set FEAGI_CONFIG_PATH)."
                        )
                    # Build zip, then persist into snapshot folder as <id>.zip
                    zip_tmp = package_snapshot(snapshot_root=Path(output_dir), snapshot_id=snapshot_id, temp_dir=Path(temp_dir), compression=compression)
                    zip_final = snap_dir / f"{snapshot_id}.zip"
                    try:
                        Path(zip_tmp).replace(zip_final)
                    except Exception:
                        # If atomic move fails, copy then unlink
                        import shutil

                        shutil.copyfile(zip_tmp, zip_final)
                        Path(zip_tmp).unlink(missing_ok=True)
                    formats_available["zip"] = f"{snapshot_id}.zip"

            return {"snapshot_id": snapshot_id, "path": str(snap_dir), "formats_available": formats_available or None}
        except Exception as e:
            logger.error(f"Failed to create brain snapshot: {e}")
            raise ValueError(str(e))

    @snapshot_endpoint("POST", "/{snapshot_id}/restore", response_model=SnapshotRestoreResponse)
    async def restore_snapshot(self, snapshot_id: str) -> Dict[str, Any]:
        """
        Restore a snapshot by id. If <id>/<id>.fc exists, use it; else use folder manifest.
        """
        try:
            config = load_feagi_config()
            snapshot_cfg = config.get("snapshot", {})
            snapshot_root = snapshot_cfg.get("output_dir")
            if not snapshot_root:
                raise ValueError(
                    "Snapshot configuration missing required key: output_dir. "
                    "Define it under [snapshot] in feagi_configuration.toml (or set FEAGI_CONFIG_PATH)."
                )
            from feagi.core.snapshot import restore_fc_snapshot, restore_brain_snapshot

            fc_path = Path(snapshot_root) / snapshot_id / f"{snapshot_id}.fc"
            if fc_path.exists():
                ok = restore_fc_snapshot(Path(snapshot_root), snapshot_id, self.core_api_service.state_manager)
            else:
                ok = restore_brain_snapshot(
                    snapshot_root=Path(snapshot_root), snapshot_id=snapshot_id, state_manager=self.core_api_service.state_manager
                )
            return {"success": bool(ok)}
        except Exception as e:
            logger.error(f"Failed to restore brain snapshot '{snapshot_id}': {e}")
            raise ValueError(str(e))

    @snapshot_endpoint("GET", "/{snapshot_id}/artifact/{fmt}")
    async def get_snapshot_artifact(self, snapshot_id: str, fmt: str) -> Dict[str, Any]:
        """
        Download snapshot artifact as .fc or .zip.
        - .fc: served from <id>/<id>.fc; built on-demand if missing
        - .zip: packaged on-demand to temp and streamed (not persisted by default)
        """
        try:
            fmt_l = fmt.lower()
            if fmt_l not in ("fc", "zip"):
                raise ValueError("Unsupported artifact format; use 'fc' or 'zip'")
            config = load_feagi_config()
            snapshot_cfg = config.get("snapshot", {})
            snapshot_root = Path(snapshot_cfg.get("output_dir", ""))
            temp_dir = Path(snapshot_cfg.get("temp_dir", ""))
            if not snapshot_root:
                raise ValueError(
                    "Snapshot configuration missing required key: output_dir. "
                    "Define it under [snapshot] in feagi_configuration.toml (or set FEAGI_CONFIG_PATH)."
                )

            snap_dir = snapshot_root / snapshot_id
            if fmt_l == "fc":
                fc_path = snap_dir / f"{snapshot_id}.fc"
                if not fc_path.exists():
                    # Build .fc on demand
                    import json as _json
                    from feagi.core.snapshot.container import create_fc_snapshot

                    connectome_json = _json.loads((snap_dir / "connectome.json").read_text(encoding="utf-8"))
                    state_json = _json.loads((snap_dir / "state.json").read_text(encoding="utf-8"))
                    fc_path = create_fc_snapshot(
                        output_dir=snap_dir,
                        snapshot_id=snapshot_id,
                        connectome_json=connectome_json,
                        state_json=state_json,
                        compression="store",
                    )
                return {"path": str(fc_path), "filename": f"{snapshot_id}.fc"}
            else:
                # zip: package to temp and stream, then cleanup
                if not temp_dir:
                    raise ValueError(
                        "Snapshot configuration missing required key: temp_dir. "
                        "Define it under [snapshot] in feagi_configuration.toml (or set FEAGI_CONFIG_PATH)."
                    )
                zip_path = package_snapshot(snapshot_root=snapshot_root, snapshot_id=snapshot_id, temp_dir=temp_dir, compression=snapshot_cfg.get("zip_compression", "deflate"))
                return {"path": str(zip_path), "filename": f"{snapshot_id}.zip"}
        except Exception as e:
            logger.error(f"Failed to get snapshot artifact '{snapshot_id}:{fmt}': {e}")
            raise ValueError(str(e))


def create_snapshot_api(core_api_service: CoreAPIService) -> SnapshotAPI:
    return SnapshotAPI(core_api_service) 