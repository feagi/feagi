"""FEAGI v1 Snapshot API.

Endpoints to manage and retrieve brain snapshots.

- Create snapshot: POST /v1/snapshots (Body: format/compression/persist)
- Download artifact: GET /v1/snapshots/{snapshot_id}/artifact/{fmt}
- Restore: POST /v1/snapshots/{snapshot_id}/restore
"""

from __future__ import annotations

# Local imports used inside handlers (kept at module level for lint compliance)
import json as _json
from pathlib import Path
from typing import Any, Dict, Optional

from pydantic import BaseModel, Field

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.api.v1.decorators import endpoint
from feagi.config.toml_loader import load_feagi_config
from feagi.core.snapshot import package_snapshot
from feagi.core.snapshot.container import (  # noqa: E402
    create_fgc_snapshot,
    create_fgs_snapshot,
)
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class SnapshotCreateRequest(BaseModel):
    stateful: bool = Field(
        False,
        description=(
            "If true, create a stateful snapshot (.fgs). If false, "
            "model/stateless (.fgc)."
        ),
    )
    compression: bool = Field(
        True,
        description=(
            "If true, persist a single container file (.fgc/.fgs). If false, "
            "persist folder only."
        ),
    )

    model_config = {
        "json_schema_extra": {
            "example": {
                "stateful": False,
                "compression": True,
            },
        }
    }


class SnapshotCreateResponse(BaseModel):
    snapshot_id: str
    path: str
    # name->relative path within snapshot folder
    formats_available: Optional[Dict[str, str]] = None


class SnapshotRestoreResponse(BaseModel):
    success: bool
    mode_used: Optional[str] = None
    profile_used: Optional[str] = None


class SnapshotRestoreRequest(BaseModel):
    mode: Optional[str] = Field(
        None,
        pattern="^(mmap|load)$",
        description=(
            "Restore mode: 'mmap' (zero-copy, requires store-encoded arrays) or 'load'."
        ),
    )
    profile: Optional[str] = Field(
        None,
        pattern="^(model|stateful)$",
        description=(
            "Which profile to restore: 'model' or 'stateful'. Defaults to 'model'."
        ),
    )

    model_config = {
        "json_schema_extra": {
            "example": {"mode": "load", "profile": "model"},
        }
    }


def snapshot_endpoint(
    methods, path, request_model=None, response_model=None, description=None
):
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

    @snapshot_endpoint(
        "POST",
        "/",
        request_model=SnapshotCreateRequest,
        response_model=SnapshotCreateResponse,
        description=(
            "Create a brain snapshot.\n\n"
            "Body:\n"
            "- stateful: true → .fgs; false → .fgc\n"
            "- compression: true → persist container file (.fgc/.fgs); "
            "false → folder only\n\n"
            "Requires [snapshot] output_dir and temp_dir in feagi_configuration.toml."
        ),
    )
    async def create_snapshot(
        self, request: SnapshotCreateRequest
    ) -> Dict[str, Any]:
        """Create a snapshot folder and optionally persist an artifact (.fgc or
        .zip)."""
        try:
            config = load_feagi_config()
            snapshot_cfg = config.get("snapshot", {})
            output_dir = snapshot_cfg.get("output_dir")
            if not output_dir:
                raise ValueError(
                    (
                        "Snapshot configuration missing required key: output_dir. "
                        "Add a [snapshot] section to feagi_configuration.toml "
                        "with keys: "
                        "output_dir, temp_dir, zip_compression "
                        "(or set FEAGI_CONFIG_PATH to your config)."
                    )
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

            # Persist container if requested
            if request.compression:
                if request.stateful:
                    _ = create_fgs_snapshot(
                        output_dir=snap_dir,
                        snapshot_id=snapshot_id,
                        compression="store",
                    )
                    formats_available["fgs"] = f"{snapshot_id}.fgs"
                else:
                    _ = create_fgc_snapshot(
                        output_dir=snap_dir,
                        snapshot_id=snapshot_id,
                        compression="store",
                    )
                    formats_available["fgc"] = f"{snapshot_id}.fgc"

            return {
                "snapshot_id": snapshot_id,
                "path": str(snap_dir),
                "formats_available": formats_available or None,
            }
        except Exception as e:
            logger.error(f"Failed to create brain snapshot: {e}")
            raise ValueError(str(e)) from e

    @snapshot_endpoint(
        "POST",
        "/{snapshot_id}/restore",
        request_model=SnapshotRestoreRequest,
        response_model=SnapshotRestoreResponse,
        description=(
            "Restore a snapshot by id.\n\n"
            "Profile (default 'model'):\n"
            "- 'model' → uses <id>/<id>.fgc if present; otherwise restores "
            "from folder manifest.\n"
            "- 'stateful' → requires <id>/<id>.fgs.\n\n"
            "Mode (default from [snapshot].fc_restore_mode):\n"
            "- 'mmap' (zero-copy, requires store-encoded arrays)\n"
            "- 'load' (copies into RAM)."
        ),
    )
    async def restore_snapshot(
        self,
        snapshot_id: str,
        request: Optional[SnapshotRestoreRequest] = None,
    ) -> Dict[str, Any]:
        """Restore a snapshot by id.

        If <id>/<id>.fgc exists, use it; else use folder manifest.
        """
        try:
            config = load_feagi_config()
            snapshot_cfg = config.get("snapshot", {})
            snapshot_root = snapshot_cfg.get("output_dir")
            if not snapshot_root:
                raise ValueError(
                    (
                        "Snapshot configuration missing required key: output_dir. "
                        "Define it under [snapshot] in feagi_configuration.toml "
                        "(or set FEAGI_CONFIG_PATH)."
                    )
                )
            from feagi.core.snapshot import (
                restore_brain_snapshot,
                restore_fgc_snapshot,
                restore_fgs_snapshot,
            )

            fc_path = Path(snapshot_root) / snapshot_id / f"{snapshot_id}.fgc"
            fcs_path = Path(snapshot_root) / snapshot_id / f"{snapshot_id}.fgs"
            mode_default = (
                snapshot_cfg.get("fc_restore_mode") or "load"
            ).lower()
            if mode_default not in ("mmap", "load"):
                raise ValueError(
                    "Invalid snapshot.fc_restore_mode; use 'mmap' or 'load'"
                )
            mode = (
                request.mode.lower()
                if request and request.mode
                else mode_default
            )
            requested_profile = (
                request.profile.lower()
                if request and request.profile
                else "model"
            )

            if requested_profile == "stateful":
                if not fcs_path.exists():
                    raise ValueError(
                        "Requested profile 'stateful' but no .fgs artifact is available"
                    )
                if mode == "mmap":
                    from feagi.core.snapshot.container import (
                        validate_mmap_eligibility,
                    )

                    validate_mmap_eligibility(fcs_path)
                ok = restore_fgs_snapshot(
                    Path(snapshot_root),
                    snapshot_id,
                    self.core_api_service.state_manager,
                    self.core_api_service.get_connectome_manager(),
                )
            elif requested_profile == "model":
                if fc_path.exists():
                    if mode == "mmap":
                        from feagi.core.snapshot.container import (
                            validate_mmap_eligibility,
                        )

                        validate_mmap_eligibility(fc_path)
                    ok = restore_fgc_snapshot(
                        Path(snapshot_root),
                        snapshot_id,
                        self.core_api_service.state_manager,
                        self.core_api_service.get_connectome_manager(),
                    )
                else:
                    ok = restore_brain_snapshot(
                        snapshot_root=Path(snapshot_root),
                        snapshot_id=snapshot_id,
                        state_manager=self.core_api_service.state_manager,
                    )
            else:
                raise ValueError(
                    "Unsupported profile; use 'model' or 'stateful'"
                )

            #  Post-restore: load genome.json if present to back
            #  geometry/parameters
            try:
                gpath = Path(snapshot_root) / snapshot_id / "genome.json"
                if gpath.exists():
                    genome_obj = _json.loads(gpath.read_text(encoding="utf-8"))
                    gs = self.core_api_service._genome_service
                    # Cache genome for services and state
                    try:
                        self.core_api_service.state_manager.genome = genome_obj
                    except Exception:
                        pass
                    try:
                        cmgr = self.core_api_service._connectome_manager
                        if hasattr(cmgr, "genome"):
                            cmgr.genome = genome_obj
                    except Exception:
                        pass
                    # Apply physiology parameters backing
                    try:
                        gs._apply_genome_physiology_parameters(
                            genome_obj, self.core_api_service
                        )
                    except Exception:
                        pass
                    #  Backfill cortical area display names from genome when
                    #  missing
                    try:
                        cmgr = self.core_api_service._connectome_manager
                        blueprint = (
                            genome_obj.get("blueprint", {})
                            if isinstance(genome_obj, dict)
                            else {}
                        )
                        for cid, area in getattr(
                            cmgr, "cortical_areas", {}
                        ).items():
                            try:
                                #  If area.name is missing or equals id, try to
                                #  use genome cortical_name
                                if (
                                    not getattr(area, "name", None)
                                    or area.name == cid
                                ):
                                    g_def = blueprint.get(cid, {})
                                    g_name = g_def.get("cortical_name")
                                    if isinstance(g_name, str) and g_name:
                                        area.name = g_name
                            except Exception:
                                continue
                        #  Refresh cortical areas cache to propagate updated
                        #  names
                        sm = self.core_api_service.state_manager
                        sm.invalidate_cortical_areas_cache()
                        _ = sm.get_cortical_areas_cache(cmgr)
                    except Exception:
                        pass
                    #  Always update genome counter and timestamp on successful
                    #  restore
                    try:
                        sm = self.core_api_service.state_manager
                        sm.increment_genome_counter()
                        import time as _t

                        sm.set_genome_timestamp(int(_t.time() * 1000))
                    except Exception:
                        pass
            except Exception:
                # Genome load is best-effort; restore result remains valid
                pass

            # Post-restore: rebuild caches and cortical list in state
            try:
                cm = self.core_api_service.get_connectome_manager()
                # Rebuild cortical list from mapping
                ids = cm.get_all_cortical_ids()
                self.core_api_service.state_manager.set_cortical_list(ids)
                # Invalidate and refresh cortical areas cache
                sm = self.core_api_service.state_manager
                sm.invalidate_cortical_areas_cache()
                _ = sm.get_cortical_areas_cache(cm)
                # Initialize spatial hash cache if available
                if hasattr(cm, "initialize_spatial_hash_cache"):
                    cm.initialize_spatial_hash_cache()
            except Exception:
                pass

            return {
                "success": bool(ok),
                "mode_used": (
                    mode if (fc_path.exists() or fcs_path.exists()) else None
                ),
                "profile_used": requested_profile,
            }
        except Exception as e:
            logger.error(
                f"Failed to restore brain snapshot '{snapshot_id}': {e}"
            )
            raise ValueError(str(e)) from e

    @snapshot_endpoint("GET", "/{snapshot_id}/artifact/{fmt}")
    async def get_snapshot_artifact(
        self, snapshot_id: str, fmt: str
    ) -> Dict[str, Any]:
        """Download snapshot artifact as .fgc/.fgs or .zip.

        - .fgc: served from <id>/<id>.fgc; built on-demand if missing
        - .fgs: served from <id>/<id>.fgs (stateful)
        - .zip: packaged on-demand to temp and streamed (not persisted by default)
        """
        try:
            fmt_l = fmt.lower()
            if fmt_l not in ("fgc", "fgs", "zip"):
                raise ValueError(
                    "Unsupported artifact format; use 'fgc', 'fgs' or 'zip'"
                )
            config = load_feagi_config()
            snapshot_cfg = config.get("snapshot", {})
            snapshot_root = Path(snapshot_cfg.get("output_dir", ""))
            temp_dir = Path(snapshot_cfg.get("temp_dir", ""))
            if not snapshot_root:
                raise ValueError(
                    (
                        "Snapshot configuration missing required key: output_dir. "
                        "Define it under [snapshot] in feagi_configuration.toml "
                        "(or set FEAGI_CONFIG_PATH)."
                    )
                )

            snap_dir = snapshot_root / snapshot_id
            if fmt_l == "fgc":
                fc_path = snap_dir / f"{snapshot_id}.fgc"
                if not fc_path.exists():
                    c_path = snap_dir / "connectome.json"
                    s_path = snap_dir / "state.json"
                    connectome_json = _json.loads(
                        c_path.read_text(encoding="utf-8")
                    )
                    state_json = _json.loads(
                        s_path.read_text(encoding="utf-8")
                    )
                    fc_path = create_fgc_snapshot(
                        output_dir=snap_dir,
                        snapshot_id=snapshot_id,
                        connectome_json=connectome_json,
                        state_json=state_json,
                        compression="store",
                    )
                return {"path": str(fc_path), "filename": f"{snapshot_id}.fgc"}
            if fmt_l == "fgs":
                fcs_path = snap_dir / f"{snapshot_id}.fgs"
                if not fcs_path.exists():
                    raise ValueError(
                        "No stateful artifact available for this snapshot"
                    )
                return {
                    "path": str(fcs_path),
                    "filename": f"{snapshot_id}.fgs",
                }
            # zip: package to temp and stream, then cleanup
            if not temp_dir:
                raise ValueError(
                    (
                        "Snapshot configuration missing required key: temp_dir. "
                        "Define it under [snapshot] in feagi_configuration.toml "
                        "(or set FEAGI_CONFIG_PATH)."
                    )
                )
            zip_path = package_snapshot(
                snapshot_root=snapshot_root,
                snapshot_id=snapshot_id,
                temp_dir=temp_dir,
                compression=snapshot_cfg.get("zip_compression", "deflate"),
            )
            return {
                "path": str(zip_path),
                "filename": f"{snapshot_id}.zip",
            }
        except Exception as e:
            logger.error(
                f"Failed to get snapshot artifact '{snapshot_id}:{fmt}': {e}"
            )
            raise ValueError(str(e)) from e

    @snapshot_endpoint("GET", "/")
    async def list_snapshots(self) -> Dict[str, Any]:
        """List available snapshot ids (folder names under
        [snapshot].output_dir)."""
        try:
            config = load_feagi_config()
            root = Path(config.get("snapshot", {}).get("output_dir", ""))
            if not root or not root.exists():
                return {"snapshots": []}
            items = []
            for child in root.iterdir():
                if child.is_dir() and (child / "manifest.json").exists():
                    items.append(child.name)
            return {"snapshots": sorted(items)}
        except Exception as e:
            logger.error(f"Failed to list snapshots: {e}")
            raise ValueError(str(e)) from e

    @snapshot_endpoint("DELETE", "/{snapshot_id}")
    async def delete_snapshot(self, snapshot_id: str) -> Dict[str, Any]:
        """Delete a snapshot folder and all of its contents (artifacts
        included)."""
        try:
            config = load_feagi_config()
            root = Path(config.get("snapshot", {}).get("output_dir", ""))
            if not root:
                raise ValueError(
                    "Snapshot configuration missing required key: output_dir."
                )
            snap_dir = root / snapshot_id
            if not snap_dir.exists():
                return {"deleted": False}
            # Delete folder and its contents
            import shutil

            shutil.rmtree(snap_dir)
            return {"deleted": True}
        except Exception as e:
            logger.error(f"Failed to delete snapshot '{snapshot_id}': {e}")
            raise ValueError(str(e)) from e


def create_snapshot_api(core_api_service: CoreAPIService) -> SnapshotAPI:
    return SnapshotAPI(core_api_service)
