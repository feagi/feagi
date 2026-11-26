"""Contract test: the SDK report dataclass against the real native validator.

Unlike ``test_connectome_migration.py`` this does not stub ``feagi_rust_py_libs``.
It fails when the native report gains or loses a field the dataclass does not mirror.
"""

import dataclasses
import json
from pathlib import Path

import pytest

pytest.importorskip("feagi_rust_py_libs.connectome")

from feagi.connectome import (  # noqa: E402
    ConnectomeMigrationReport,
    validate_and_migrate_connectome,
)

FIXTURE = Path(__file__).parent / "fixtures" / "minimal_valid.connectome"


def test_native_report_keys_match_dataclass_fields() -> None:
    """Every key the native validator emits must be a dataclass field, and vice versa."""
    from feagi_rust_py_libs.connectome import (
        validate_and_migrate_connectome as rust_validate,
    )

    _, report_json = rust_validate(FIXTURE.read_bytes())
    native_keys = set(json.loads(report_json))
    field_names = {
        f.name for f in dataclasses.fields(ConnectomeMigrationReport)
    }
    assert native_keys == field_names


def test_validate_and_migrate_returns_report_for_real_artifact() -> None:
    """A valid artifact must yield migrated bytes and a typed report, not raise."""
    artifact, report = validate_and_migrate_connectome(FIXTURE.read_bytes())

    assert isinstance(artifact, bytes) and artifact
    assert isinstance(report, ConnectomeMigrationReport)
    assert report.valid
    assert report.blocking_errors == []
