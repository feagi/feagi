"""
Snapshot packaging utilities (ZIP) - consolidated under feagi.core.snapshot.

Packages a snapshot directory into a ZIP archive based on manifest.json.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Dict, Iterable, List, Tuple
import zipfile


def _flatten_manifest_files(files_entry: Dict[str, object]) -> List[str]:
    flat: List[str] = []
    for _category, entry in files_entry.items():
        if entry is None:
            continue
        if isinstance(entry, str):
            flat.append(entry)
        elif isinstance(entry, Iterable):
            for item in entry:
                if isinstance(item, str):
                    flat.append(item)
                else:
                    raise ValueError("Manifest 'files' entries must be strings or lists of strings")
        else:
            raise ValueError("Manifest 'files' section has invalid entry type")
    return sorted(set(flat))


def _resolve_compression(compression: str) -> int:
    mode = compression.strip().lower()
    if mode == "deflate":
        return zipfile.ZIP_DEFLATED
    if mode == "store":
        return zipfile.ZIP_STORED
    raise ValueError(f"Unsupported zip compression: {compression}")


def _safe_relative_path(root: Path, target: Path) -> str:
    root_resolved = root.resolve()
    target_resolved = target.resolve()
    if not str(target_resolved).startswith(str(root_resolved)):
        raise ValueError(f"File '{target}' is outside snapshot root '{root}'")
    return str(target_resolved.relative_to(root_resolved)).replace(os.sep, "/")


def package_snapshot(snapshot_root: Path, snapshot_id: str, temp_dir: Path, compression: str) -> Path:
    if not isinstance(snapshot_root, Path):
        snapshot_root = Path(snapshot_root)
    if not isinstance(temp_dir, Path):
        temp_dir = Path(temp_dir)

    snapshot_dir = snapshot_root / snapshot_id
    manifest_path = snapshot_dir / "manifest.json"

    if not snapshot_dir.exists() or not snapshot_dir.is_dir():
        raise FileNotFoundError(f"Snapshot directory not found: {snapshot_dir}")
    if not manifest_path.exists():
        raise FileNotFoundError(f"Snapshot manifest.json not found: {manifest_path}")

    with open(manifest_path, "r", encoding="utf-8") as f:
        manifest = json.load(f)

    schema = manifest.get("schema_version")
    if not schema or not isinstance(schema, str):
        raise ValueError("manifest.json missing required 'schema_version'")

    files_entry = manifest.get("files")
    if not isinstance(files_entry, dict):
        raise ValueError("manifest.json missing required 'files' section")

    include_paths = _flatten_manifest_files(files_entry)
    if not include_paths:
        raise ValueError("manifest.json 'files' section is empty")

    files_to_add: List[Tuple[Path, str]] = []
    for rel in include_paths:
        abs_path = snapshot_dir / rel
        if not abs_path.exists():
            raise ValueError(f"Manifest lists missing file: {rel}")
        arcname = _safe_relative_path(snapshot_dir, abs_path)
        files_to_add.append((abs_path, arcname))

    temp_dir.mkdir(parents=True, exist_ok=True)
    zip_mode = _resolve_compression(compression)

    zip_path = temp_dir / f"{snapshot_id}--temp.snapshot.zip"
    counter = 1
    while zip_path.exists():
        zip_path = temp_dir / f"{snapshot_id}--temp{counter}.snapshot.zip"
        counter += 1

    with zipfile.ZipFile(zip_path, mode="w", compression=zip_mode, allowZip64=True) as zf:
        for abs_path, arcname in files_to_add:
            zf.write(abs_path, arcname)
        zf.write(manifest_path, "manifest.json")

    return zip_path.resolve() 