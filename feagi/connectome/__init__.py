"""Binary connectome artifact I/O, migration, and live FEAGI operations."""

import json
from typing import Any

from feagi.connectome.api import (
    ConnectomeAPI,
    ConnectomeAPIError,
    ConnectomeMode,
)
from feagi.connectome.artifact import (
    CONNECTOME_ARTIFACT_EXTENSION,
    CONNECTOME_ARTIFACT_MEDIA_TYPE,
    connectome_artifact_file_name,
    connectome_artifact_stem,
    is_connectome_artifact_file_name,
    read_connectome_artifact,
    write_connectome_artifact,
)
from feagi.connectome.model import ConnectomeMigrationReport


def validate_and_migrate_connectome(
    artifact_bytes: bytes,
) -> tuple[bytes, ConnectomeMigrationReport]:
    """Validate and non-destructively migrate a connectome artifact.

    Args:
        artifact_bytes: Complete binary ``.connectome`` artifact.

    Returns:
        Migrated artifact bytes and the authoritative compatibility report.

    Raises:
        UnsupportedArtifactVersionError: The artifact was written by a newer
            FEAGI build than this one reads. Carries ``component``,
            ``artifact_version`` and ``supported_version``.
        ValueError: The artifact is corrupt or semantically invalid.
            ``UnsupportedArtifactVersionError`` is a ``ValueError`` subclass.
    """
    from feagi_rust_py_libs.connectome import (
        validate_and_migrate_connectome as rust_validate_and_migrate,
    )

    migrated_bytes, report_json = rust_validate_and_migrate(artifact_bytes)
    report_data = json.loads(report_json)
    if not isinstance(report_data, dict):
        raise ValueError("Connectome migration report must be a JSON object")
    report = ConnectomeMigrationReport(**report_data)
    return bytes(migrated_bytes), report


def supported_artifact_versions() -> dict[str, int]:
    """Return the highest artifact version this FEAGI build reads, per component.

    Returns:
        Mapping with the int-valued keys ``contract``, ``container``,
        ``connectome_schema`` and ``genome_schema``.
    """
    from feagi_rust_py_libs.connectome import (
        supported_artifact_versions as rust_supported_artifact_versions,
    )

    versions = rust_supported_artifact_versions()
    return {str(component): int(version) for component, version in versions.items()}


def __getattr__(name: str) -> Any:
    """Resolve ``UnsupportedArtifactVersionError`` from the Rust library on first use.

    The exception class lives in ``feagi_rust_py_libs`` and is imported lazily, like the
    other Rust-backed calls here, so importing this package does not require it.
    """
    if name == "UnsupportedArtifactVersionError":
        from feagi_rust_py_libs.connectome import UnsupportedArtifactVersionError

        return UnsupportedArtifactVersionError
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "CONNECTOME_ARTIFACT_EXTENSION",
    "CONNECTOME_ARTIFACT_MEDIA_TYPE",
    "ConnectomeAPI",
    "ConnectomeAPIError",
    "ConnectomeMigrationReport",
    "ConnectomeMode",
    "UnsupportedArtifactVersionError",
    "connectome_artifact_file_name",
    "connectome_artifact_stem",
    "is_connectome_artifact_file_name",
    "read_connectome_artifact",
    "supported_artifact_versions",
    "validate_and_migrate_connectome",
    "write_connectome_artifact",
]
