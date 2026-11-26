"""Encoding boundary for external FEAGI genome artifacts."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Protocol

GENOME_ARTIFACT_EXTENSION = ".genome"
GENOME_ARTIFACT_MEDIA_TYPE = "application/vnd.feagi.genome+json"


class GenomeArtifactCodec(Protocol):
    """Convert external artifact bytes without performing schema migration."""

    @property
    def media_type(self) -> str:
        """Return the media type emitted by the codec."""

    def decode(self, artifact: bytes) -> dict[str, Any]:
        """Decode artifact bytes into a schema-bearing genome dictionary."""

    def encode(self, genome: dict[str, Any]) -> bytes:
        """Encode a schema-bearing genome dictionary as artifact bytes."""


class JsonGenomeArtifactCodec:
    """Encode and decode the current UTF-8 JSON artifact representation."""

    @property
    def media_type(self) -> str:
        """Return the JSON genome artifact media type."""
        return GENOME_ARTIFACT_MEDIA_TYPE

    def decode(self, artifact: bytes) -> dict[str, Any]:
        """Decode UTF-8 JSON bytes without interpreting schema version."""
        payload = json.loads(artifact)
        if not isinstance(payload, dict):
            raise ValueError("Genome artifact must contain a JSON object")
        return payload

    def encode(self, genome: dict[str, Any]) -> bytes:
        """Encode a genome dictionary as deterministic compact JSON bytes."""
        return json.dumps(
            genome,
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")


JSON_GENOME_ARTIFACT_CODEC = JsonGenomeArtifactCodec()


def decode_genome_artifact(artifact: bytes) -> dict[str, Any]:
    """Decode bytes using the current explicitly selected artifact codec."""
    return JSON_GENOME_ARTIFACT_CODEC.decode(artifact)


def encode_genome_artifact(genome: dict[str, Any]) -> bytes:
    """Encode a genome using the current explicitly selected artifact codec."""
    return JSON_GENOME_ARTIFACT_CODEC.encode(genome)


def is_genome_artifact_file_name(file_name: str | None) -> bool:
    """Return whether a filename uses the required `.genome` extension."""
    if not file_name:
        return False
    path = Path(file_name.replace("\\", "/"))
    return bool(path.stem) and path.suffix.lower() == GENOME_ARTIFACT_EXTENSION


def genome_artifact_stem(file_name: str) -> str:
    """Return the platform-independent stem of a valid artifact filename."""
    if not is_genome_artifact_file_name(file_name):
        raise ValueError("Genome files must use the .genome extension")
    normalized_name = Path(file_name.replace("\\", "/")).name
    return normalized_name[: -len(GENOME_ARTIFACT_EXTENSION)]


def genome_artifact_file_name(stem: str) -> str:
    """Build a cross-platform-safe artifact filename from a non-empty stem."""
    normalized_stem = stem.strip()
    if not normalized_stem:
        raise ValueError("Genome artifact file name stem cannot be empty")
    safe_stem = "".join(
        "_" if character in {'"', "\r", "\n", "/", "\\"} else character
        for character in normalized_stem
    )
    return f"{safe_stem}{GENOME_ARTIFACT_EXTENSION}"


def read_genome_artifact(path: str | Path) -> dict[str, Any]:
    """Read and decode a `.genome` artifact from the filesystem."""
    artifact_path = Path(path)
    if not is_genome_artifact_file_name(artifact_path.name):
        raise ValueError("Genome files must use the .genome extension")
    return decode_genome_artifact(artifact_path.read_bytes())


def write_genome_artifact(path: str | Path, genome: dict[str, Any]) -> None:
    """Encode and write a `.genome` artifact to the filesystem."""
    artifact_path = Path(path)
    if not is_genome_artifact_file_name(artifact_path.name):
        raise ValueError("Genome files must use the .genome extension")
    artifact_path.write_bytes(encode_genome_artifact(genome))


__all__ = [
    "GENOME_ARTIFACT_EXTENSION",
    "GENOME_ARTIFACT_MEDIA_TYPE",
    "JSON_GENOME_ARTIFACT_CODEC",
    "GenomeArtifactCodec",
    "JsonGenomeArtifactCodec",
    "decode_genome_artifact",
    "encode_genome_artifact",
    "genome_artifact_file_name",
    "genome_artifact_stem",
    "is_genome_artifact_file_name",
    "read_genome_artifact",
    "write_genome_artifact",
]
