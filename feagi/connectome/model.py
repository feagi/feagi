"""Typed compatibility report for connectome artifact migration."""

from dataclasses import dataclass, field


@dataclass(frozen=True)
class ConnectomeMigrationReport:
    """Compatibility and migration details returned by FEAGI core.

    ``blocking_errors`` lists errors that put the brain in degraded mode; the
    artifact stays importable, so a non-empty list or ``valid=False`` never raises.
    """

    valid: bool
    compatible: bool
    source_container_version: int
    target_container_version: int
    source_connectome_schema_version: int
    target_connectome_schema_version: int
    source_genome_schema_version: int | None
    target_genome_schema_version: int | None
    genome_sha256: str | None
    migration_steps: list[str] = field(default_factory=list)
    identifier_remaps: dict[str, str] = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)
    # Errors that put the brain in degraded mode; the artifact stays importable.
    blocking_errors: list[str] = field(default_factory=list)


__all__ = ["ConnectomeMigrationReport"]
