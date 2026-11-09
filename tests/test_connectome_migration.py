"""Tests for the typed connectome migration adapter."""

import json
from types import ModuleType
import sys

import pytest

from feagi import connectome
from feagi.connectome import (
    supported_artifact_versions,
    validate_and_migrate_connectome,
)


def test_validate_and_migrate_connectome_returns_typed_report(monkeypatch) -> None:
    """The SDK must preserve migrated bytes and all compatibility metadata."""
    rust_module = ModuleType("feagi_rust_py_libs.connectome")
    report = {
        "valid": True,
        "compatible": True,
        "source_container_version": 1,
        "target_container_version": 2,
        "source_connectome_schema_version": 1,
        "target_connectome_schema_version": 1,
        "source_genome_schema_version": 2,
        "target_genome_schema_version": 3,
        "genome_sha256": "digest",
        "migration_steps": ["genome:v2_to_v3", "container:v1->v2"],
        "identifier_remaps": {"old": "new"},
        "warnings": [],
    }

    def rust_validate(payload: bytes) -> tuple[bytes, str]:
        assert payload == b"source"
        return b"migrated", json.dumps(report)

    rust_module.validate_and_migrate_connectome = rust_validate
    monkeypatch.setitem(sys.modules, "feagi_rust_py_libs.connectome", rust_module)

    artifact, migration_report = validate_and_migrate_connectome(b"source")

    assert artifact == b"migrated"
    assert migration_report.valid
    assert migration_report.target_genome_schema_version == 3
    assert migration_report.identifier_remaps == {"old": "new"}


def _install_rust_module(monkeypatch) -> ModuleType:
    """Install a stand-in ``feagi_rust_py_libs.connectome`` (outside the SDK's domain)."""
    rust_module = ModuleType("feagi_rust_py_libs.connectome")

    class UnsupportedArtifactVersionError(ValueError):
        """Stand-in for the Rust-defined exception."""

    rust_module.UnsupportedArtifactVersionError = UnsupportedArtifactVersionError
    monkeypatch.setitem(sys.modules, "feagi_rust_py_libs.connectome", rust_module)
    return rust_module


def test_unsupported_artifact_version_error_is_lazily_reexported(monkeypatch) -> None:
    """The class is the Rust one, resolved on access, and is a ValueError subclass."""
    rust_module = _install_rust_module(monkeypatch)

    assert (
        connectome.UnsupportedArtifactVersionError
        is rust_module.UnsupportedArtifactVersionError
    )
    assert issubclass(connectome.UnsupportedArtifactVersionError, ValueError)
    assert "UnsupportedArtifactVersionError" in connectome.__all__
    assert "UnsupportedArtifactVersionError" not in vars(connectome)


def test_unsupported_artifact_version_error_propagates_unchanged(monkeypatch) -> None:
    """The adapter must not wrap or swallow the Rust exception or its attributes."""
    rust_module = _install_rust_module(monkeypatch)
    raised = rust_module.UnsupportedArtifactVersionError(
        "Connectome validation failed: too new"
    )
    raised.component = "container"
    raised.artifact_version = 9
    raised.supported_version = 2

    def rust_validate(payload: bytes) -> tuple[bytes, str]:
        raise raised

    rust_module.validate_and_migrate_connectome = rust_validate

    with pytest.raises(connectome.UnsupportedArtifactVersionError) as caught:
        validate_and_migrate_connectome(b"future")

    assert caught.value is raised
    assert (caught.value.component, caught.value.artifact_version) == ("container", 9)
    assert caught.value.supported_version == 2


def test_plain_value_error_is_not_the_unsupported_version_error(monkeypatch) -> None:
    """Other validation failures stay plain ValueError."""
    rust_module = _install_rust_module(monkeypatch)

    def rust_validate(payload: bytes) -> tuple[bytes, str]:
        raise ValueError("Connectome validation failed: corrupt")

    rust_module.validate_and_migrate_connectome = rust_validate

    with pytest.raises(ValueError) as caught:
        validate_and_migrate_connectome(b"bad")

    assert not isinstance(caught.value, connectome.UnsupportedArtifactVersionError)


def test_supported_artifact_versions_passthrough(monkeypatch) -> None:
    """The wrapper returns the Rust mapping as a plain dict of ints."""
    rust_module = _install_rust_module(monkeypatch)
    rust_module.supported_artifact_versions = lambda: {
        "contract": 2,
        "container": 3,
        "connectome_schema": 1,
        "genome_schema": 4,
    }

    versions = supported_artifact_versions()

    assert versions == {
        "contract": 2,
        "container": 3,
        "connectome_schema": 1,
        "genome_schema": 4,
    }
    assert type(versions) is dict
    assert all(type(value) is int for value in versions.values())
    assert "supported_artifact_versions" in connectome.__all__


def test_unknown_attribute_still_raises_attribute_error() -> None:
    """The lazy hook resolves only the documented name."""
    with pytest.raises(AttributeError):
        connectome.does_not_exist  # noqa: B018
