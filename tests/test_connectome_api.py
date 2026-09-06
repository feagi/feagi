"""Contract tests for FEAGI connectome REST operations."""

from pathlib import Path
from unittest.mock import Mock, patch

import pytest

from feagi.connectome import ConnectomeAPI


@pytest.fixture
def connectome_api() -> ConnectomeAPI:
    """Create a connectome client with an explicit test timeout."""
    return ConnectomeAPI("http://test:8000", timeout=5.0)


def test_upload_uses_binary_multipart_artifact(
    connectome_api: ConnectomeAPI,
) -> None:
    """Upload sends artifact bytes to the connectome upload endpoint."""
    response = Mock(
        status_code=200,
        text="",
        content=b'{"message":"Connectome imported"}',
    )
    response.json.return_value = {"message": "Connectome imported"}

    with patch(
        "feagi.connectome.api.requests.request", return_value=response
    ) as request:
        result = connectome_api.upload(b"artifact", "brain.connectome")

    assert result == {"message": "Connectome imported"}
    kwargs = request.call_args.kwargs
    assert request.call_args.args[:2] == (
        "POST",
        "http://test:8000/v1/connectome/upload",
    )
    assert kwargs["files"]["file"][:2] == ("brain.connectome", b"artifact")
    assert kwargs["timeout"] == 5.0


def test_download_supports_full_and_lite_modes(
    connectome_api: ConnectomeAPI,
) -> None:
    """Download returns opaque artifact bytes and forwards persistence mode."""
    response = Mock(status_code=200, text="", content=b"artifact")

    with patch(
        "feagi.connectome.api.requests.request", return_value=response
    ) as request:
        result = connectome_api.download("lite")

    assert result == b"artifact"
    assert request.call_args.kwargs["params"] == {"mode": "lite"}


def test_validate_returns_typed_migration_report(
    connectome_api: ConnectomeAPI,
) -> None:
    """Validation converts the server report into the public report type."""
    report = {
        "valid": True,
        "compatible": True,
        "source_container_version": 3,
        "target_container_version": 3,
        "source_connectome_schema_version": 1,
        "target_connectome_schema_version": 1,
        "source_genome_schema_version": 3,
        "target_genome_schema_version": 3,
        "genome_sha256": "digest",
        "migration_steps": [],
        "identifier_remaps": {},
        "warnings": [],
    }
    response = Mock(status_code=200, text="", content=b"report")
    response.json.return_value = report

    with patch("feagi.connectome.api.requests.request", return_value=response):
        result = connectome_api.validate(b"artifact", "brain.connectome")

    assert result.valid
    assert result.target_container_version == 3
    assert result.genome_sha256 == "digest"


def test_upload_rejects_non_connectome_filename(
    connectome_api: ConnectomeAPI,
) -> None:
    """REST helpers enforce the public connectome filename contract."""
    with pytest.raises(ValueError, match=r"\.connectome extension"):
        connectome_api.upload(b"artifact", "brain.bin")


def test_file_upload_and_download_preserve_artifact_bytes(
    connectome_api: ConnectomeAPI,
    tmp_path: Path,
) -> None:
    """File conveniences read uploads and write downloaded binary bytes."""
    source = tmp_path / "source.connectome"
    destination = tmp_path / "downloaded.connectome"
    source.write_bytes(b"source")
    upload_response = Mock(status_code=200, text="", content=b"{}")
    upload_response.json.return_value = {"message": "imported"}
    download_response = Mock(
        status_code=200,
        text="",
        content=b"downloaded",
    )

    with patch(
        "feagi.connectome.api.requests.request",
        side_effect=[upload_response, download_response],
    ):
        connectome_api.upload_file(source)
        connectome_api.download_to_file(destination, mode="full")

    assert destination.read_bytes() == b"downloaded"


def test_migrate_file_writes_new_artifact_without_changing_source(
    connectome_api: ConnectomeAPI,
    tmp_path: Path,
) -> None:
    """File migration is non-destructive and writes returned artifact bytes."""
    source = tmp_path / "source.connectome"
    destination = tmp_path / "migrated.connectome"
    source.write_bytes(b"source")
    response = Mock(status_code=200, text="", content=b"migrated")

    with patch(
        "feagi.connectome.api.requests.request",
        return_value=response,
    ) as request:
        connectome_api.migrate_file(source, destination)

    assert source.read_bytes() == b"source"
    assert destination.read_bytes() == b"migrated"
    assert request.call_args.args[1].endswith("/v1/connectome/migrate")
