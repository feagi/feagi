"""Contract tests for binary connectome artifact filesystem I/O."""

from pathlib import Path

import pytest

from feagi.connectome import (
    CONNECTOME_ARTIFACT_EXTENSION,
    connectome_artifact_file_name,
    connectome_artifact_stem,
    is_connectome_artifact_file_name,
    read_connectome_artifact,
    write_connectome_artifact,
)


def test_filename_helpers_enforce_connectome_extension() -> None:
    """Connectome filenames are portable and extension-specific."""
    assert CONNECTOME_ARTIFACT_EXTENSION == ".connectome"
    assert is_connectome_artifact_file_name("brain.connectome")
    assert is_connectome_artifact_file_name(r"C:\brains\Brain.CONNECTOME")
    assert not is_connectome_artifact_file_name("brain.genome")
    assert connectome_artifact_stem(r"C:\brains\Brain.connectome") == "Brain"
    assert (
        connectome_artifact_file_name('Brain "A"\r\n')
        == "Brain _A_.connectome"
    )


def test_file_round_trip_preserves_binary_payload(tmp_path: Path) -> None:
    """Filesystem helpers preserve opaque binary connectome bytes."""
    artifact_path = tmp_path / "brain.connectome"
    artifact = b"\x00FEAGI-connectome\xff"

    write_connectome_artifact(artifact_path, artifact)

    assert read_connectome_artifact(artifact_path) == artifact


def test_file_helpers_reject_other_extensions(tmp_path: Path) -> None:
    """Connectome helpers reject files that violate the artifact contract."""
    invalid_path = tmp_path / "brain.bin"

    with pytest.raises(ValueError, match=r"\.connectome extension"):
        write_connectome_artifact(invalid_path, b"artifact")


def test_public_exports_include_artifact_and_migration_apis() -> None:
    """Wildcard exports retain the supported connectome surface."""
    namespace: dict[str, object] = {}

    exec("from feagi.connectome import *", namespace)

    assert "ConnectomeAPI" in namespace
    assert "validate_and_migrate_connectome" in namespace
    assert "read_connectome_artifact" in namespace
