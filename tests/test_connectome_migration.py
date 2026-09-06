"""Tests for the typed connectome migration adapter."""

import json
from types import ModuleType
import sys

from feagi.connectome import validate_and_migrate_connectome


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
