"""Filesystem boundary for binary FEAGI connectome artifacts."""

from __future__ import annotations

from pathlib import Path

CONNECTOME_ARTIFACT_EXTENSION = ".connectome"
CONNECTOME_ARTIFACT_MEDIA_TYPE = "application/octet-stream"


def is_connectome_artifact_file_name(file_name: str | None) -> bool:
    """Return whether a filename uses the required `.connectome` extension."""
    if not file_name:
        return False
    path = Path(file_name.replace("\\", "/"))
    return (
        bool(path.stem)
        and path.suffix.lower() == CONNECTOME_ARTIFACT_EXTENSION
    )


def connectome_artifact_stem(file_name: str) -> str:
    """Return the platform-independent stem of a connectome artifact."""
    if not is_connectome_artifact_file_name(file_name):
        raise ValueError("Connectome files must use the .connectome extension")
    normalized_name = Path(file_name.replace("\\", "/")).name
    return normalized_name[: -len(CONNECTOME_ARTIFACT_EXTENSION)]


def connectome_artifact_file_name(stem: str) -> str:
    """Build a cross-platform-safe connectome filename from a non-empty stem."""
    normalized_stem = stem.strip()
    if not normalized_stem:
        raise ValueError("Connectome artifact file name stem cannot be empty")
    safe_stem = "".join(
        "_" if character in {'"', "\r", "\n", "/", "\\"} else character
        for character in normalized_stem
    )
    return f"{safe_stem}{CONNECTOME_ARTIFACT_EXTENSION}"


def read_connectome_artifact(path: str | Path) -> bytes:
    """Read a binary `.connectome` artifact from the filesystem."""
    artifact_path = Path(path)
    if not is_connectome_artifact_file_name(artifact_path.name):
        raise ValueError("Connectome files must use the .connectome extension")
    return artifact_path.read_bytes()


def write_connectome_artifact(path: str | Path, artifact: bytes) -> None:
    """Write binary connectome bytes to a `.connectome` filesystem path."""
    artifact_path = Path(path)
    if not is_connectome_artifact_file_name(artifact_path.name):
        raise ValueError("Connectome files must use the .connectome extension")
    artifact_path.write_bytes(artifact)


__all__ = [
    "CONNECTOME_ARTIFACT_EXTENSION",
    "CONNECTOME_ARTIFACT_MEDIA_TYPE",
    "connectome_artifact_file_name",
    "connectome_artifact_stem",
    "is_connectome_artifact_file_name",
    "read_connectome_artifact",
    "write_connectome_artifact",
]
