"""Contract tests for encoding-neutral genome artifact I/O."""

import json

import pytest
from feagi.genome.artifact import (
    GENOME_ARTIFACT_MEDIA_TYPE,
    JsonGenomeArtifactCodec,
    decode_genome_artifact,
    encode_genome_artifact,
    genome_artifact_file_name,
    genome_artifact_stem,
    is_genome_artifact_file_name,
    read_genome_artifact,
    write_genome_artifact,
)


def test_json_codec_round_trip_preserves_schema_version() -> None:
    """The artifact codec must not reinterpret genome schema versions."""
    genome = {
        "genome_schema_version": 3,
        "version": "3.0",
        "blueprint": {},
    }

    encoded = encode_genome_artifact(genome)

    assert decode_genome_artifact(encoded) == genome
    assert JsonGenomeArtifactCodec().media_type == GENOME_ARTIFACT_MEDIA_TYPE


def test_json_codec_rejects_malformed_or_non_object_payloads() -> None:
    """Malformed JSON and non-object documents are invalid artifacts."""
    with pytest.raises(json.JSONDecodeError):
        decode_genome_artifact(b"not-json")

    with pytest.raises(ValueError, match="JSON object"):
        decode_genome_artifact(b"[]")


def test_filename_helpers_enforce_genome_extension() -> None:
    """Artifact filenames are portable and reject the legacy extension."""
    assert is_genome_artifact_file_name("brain.genome")
    assert is_genome_artifact_file_name(r"C:\brains\Vision.GENOME")
    assert not is_genome_artifact_file_name("brain.json")
    assert genome_artifact_stem(r"C:\brains\Vision.genome") == "Vision"
    assert genome_artifact_file_name('Brain "A"\r\n') == "Brain _A_.genome"


def test_file_round_trip_uses_codec_contract(tmp_path) -> None:
    """Filesystem helpers must route data through the selected codec."""
    artifact_path = tmp_path / "brain.genome"
    genome = {"genome_schema_version": 3, "blueprint": {"pwr": {}}}

    write_genome_artifact(artifact_path, genome)

    assert read_genome_artifact(artifact_path) == genome


def test_file_helpers_reject_json_extension(tmp_path) -> None:
    """Legacy `.json` filenames are not external genome artifacts."""
    legacy_path = tmp_path / "brain.json"

    with pytest.raises(ValueError, match=r"\.genome extension"):
        write_genome_artifact(legacy_path, {})
