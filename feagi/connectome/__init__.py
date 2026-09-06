"""Binary connectome artifact I/O, migration, and live FEAGI operations."""

import json

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
        ValueError: The artifact is corrupt, unsupported, or semantically invalid.
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


__all__ = [
    "CONNECTOME_ARTIFACT_EXTENSION",
    "CONNECTOME_ARTIFACT_MEDIA_TYPE",
    "ConnectomeAPI",
    "ConnectomeAPIError",
    "ConnectomeMigrationReport",
    "ConnectomeMode",
    "connectome_artifact_file_name",
    "connectome_artifact_stem",
    "is_connectome_artifact_file_name",
    "read_connectome_artifact",
    "validate_and_migrate_connectome",
    "write_connectome_artifact",
]
