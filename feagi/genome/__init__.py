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


from feagi.genome.api import GenomeAPI, GenomeAPIError

__all__ = [
    "validate_genome",
    "auto_fix_genome",
    "GenomeLoader",
    "GenomeAPI",
    "GenomeAPIError",
]
