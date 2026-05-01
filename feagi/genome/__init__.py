"""
FEAGI Genome Manipulation

Edit genomes on a running FEAGI instance via REST API.

Provides:
- validate_genome: Validate genome structure (Rust-backed, FEAGI 2.0 format)
- auto_fix_genome: Auto-fix common genome issues
- GenomeLoader: Load genome from local file
- GenomeAPI: Add/remove cortical areas, modify parameters via REST API

Example:
    from feagi.genome import GenomeLoader, GenomeAPI, validate_genome

    loader = GenomeLoader()
    genome = loader.load("my_brain.json")
    valid, errors = validate_genome(genome)

    api = GenomeAPI("http://localhost:8000")
    api.upload(genome)
    api.add_custom_cortical_area("my_area", (10, 10, 10), (0, 0, 0))
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


class GenomeLoader:
    """
    Load genome from local file.

    Supports absolute paths, relative paths, and filename-only (resolved against
    default genomes directory, same as FeagiEngine and CLI).
    """

    def load(self, path: str | Path) -> dict[str, Any]:
        """
        Load genome from file and return as dict.

        Args:
            path: Path to genome JSON file. Can be:
                - Absolute: /path/to/genome.json
                - Relative: ./genome.json
                - Filename only: genome.json (resolved against genomes dir)

        Returns:
            Genome dict (version, blueprint, neuron_morphologies, physiology, etc.)

        Raises:
            FileNotFoundError: If file does not exist
            json.JSONDecodeError: If file is not valid JSON
        """
        from feagi.paths import get_feagi_paths

        paths = get_feagi_paths()
        p = Path(path)

        if not p.is_absolute() and "/" not in str(path) and "\\" not in str(path):
            resolved = paths.resolve_path(str(path), "genome")
        else:
            resolved = Path(path)

        if not resolved.exists():
            raise FileNotFoundError(f"Genome file not found: {path}")

        with open(resolved, encoding="utf-8") as f:
            return json.load(f)


def validate_genome(genome: dict[str, Any]) -> tuple[bool, list[str]]:
    """
    Validate genome structure using Rust-backed FEAGI 2.0 validator.

    Args:
        genome: Genome dict with keys version, blueprint, neuron_morphologies, physiology.
                Supports both flat and hierarchical formats.

    Returns:
        Tuple of (valid: bool, errors: list[str]).
        When valid is False, errors contains validation messages.
        Warnings are appended to errors for backward compatibility.
    """
    from feagi_rust_py_libs.genome import validate_genome as _rust_validate

    result = _rust_validate(json.dumps(genome))
    all_issues = list(result.errors) + list(result.warnings)
    return result.valid, all_issues


def auto_fix_genome(genome: dict[str, Any]) -> tuple[dict[str, Any], int]:
    """
    Auto-fix common genome issues (zero dimensions, missing physiology, etc.).

    Args:
        genome: Genome dict to fix.

    Returns:
        Tuple of (fixed_genome: dict, num_fixes_applied: int).
    """
    from feagi_rust_py_libs.genome import auto_fix_genome as _rust_auto_fix

    fixed_json, num_fixes = _rust_auto_fix(json.dumps(genome))
    return json.loads(fixed_json), num_fixes


# ----------------------------------------------------------------------
# Schema-versioning aware API (ChainResult pipeline).
#
# These dataclasses mirror the PyO3 classes exposed by feagi-rust-py-libs
# but are plain Python so callers don't have to import the Rust module
# in their type hints. Fields match 1:1; see
# feagi-core/docs/GENOME_SCHEMA_VERSIONING.md for semantics.
# ----------------------------------------------------------------------


@dataclass(frozen=True)
class MigrationStepDiagnostics:
    """Per-hop migrator diagnostics (`vN -> vN+1`)."""

    from_version: int
    to_version: int
    transformations: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class NormalizationDiagnostics:
    """Per-version normalizer diagnostics (in-version cleanup)."""

    schema_version: int
    transformations: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class ChainResult:
    """
    Full report from `validate_and_repair_genome`.

    `blocking_errors` is non-empty when the latest-version validator
    found issues. The library never raises on validator errors; callers
    inspect this report and decide how to react (notify the user, drop
    problematic elements and run degraded, or reject the load). This bit
    also feeds into the FEAGI `/health` endpoint's `genome_validity`
    flag.
    """

    from_version: int
    to_version: int
    migrators_applied: list[str] = field(default_factory=list)
    normalizers_applied: list[str] = field(default_factory=list)
    per_step_diagnostics: list[MigrationStepDiagnostics] = field(
        default_factory=list
    )
    per_normalizer_diagnostics: list[NormalizationDiagnostics] = field(
        default_factory=list
    )
    advisory_warnings: list[str] = field(default_factory=list)
    blocking_errors: list[str] = field(default_factory=list)

    @property
    def is_blocking_clean(self) -> bool:
        return not self.blocking_errors


def validate_and_repair_genome(
    genome: dict[str, Any],
) -> tuple[dict[str, Any], ChainResult]:
    """
    Validate and repair a genome, returning the repaired dict plus the
    full chain report.

    The library never converts validator errors into exceptions. On
    return, `ChainResult.blocking_errors` describes any latest-version
    validation failures; the repaired genome is still returned so
    callers can decide whether to load it in degraded mode, surface the
    errors to the user, or reject the load outright. Only hard I/O /
    JSON parse / structural failures raise (propagated from Rust as
    `ValueError`).

    Args:
        genome: Genome dict in any supported schema version.

    Returns:
        Tuple of (repaired_genome: dict, report: ChainResult).
    """
    from feagi_rust_py_libs.genome import (
        validate_and_repair_genome as _rust_validate_and_repair,
    )

    repaired_json, report = _rust_validate_and_repair(json.dumps(genome))
    repaired = json.loads(repaired_json)

    py_report = ChainResult(
        from_version=int(report.from_version),
        to_version=int(report.to_version),
        migrators_applied=list(report.migrators_applied),
        normalizers_applied=list(report.normalizers_applied),
        per_step_diagnostics=[
            MigrationStepDiagnostics(
                from_version=int(step.from_version),
                to_version=int(step.to_version),
                transformations=list(step.transformations),
            )
            for step in report.per_step_diagnostics
        ],
        per_normalizer_diagnostics=[
            NormalizationDiagnostics(
                schema_version=int(n.schema_version),
                transformations=list(n.transformations),
            )
            for n in report.per_normalizer_diagnostics
        ],
        advisory_warnings=list(report.advisory_warnings),
        blocking_errors=list(report.blocking_errors),
    )
    return repaired, py_report


from feagi.genome.api import GenomeAPI, GenomeAPIError

__all__ = [
    "validate_genome",
    "auto_fix_genome",
    "validate_and_repair_genome",
    "ChainResult",
    "MigrationStepDiagnostics",
    "NormalizationDiagnostics",
    "GenomeLoader",
    "GenomeAPI",
    "GenomeAPIError",
]
