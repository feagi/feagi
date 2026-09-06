"""Tests for genome and connectome CLI startup options."""

from pathlib import Path
from unittest.mock import Mock, patch

import pytest

from feagi.cli.main import _build_parser, _handle_download_command, main


@pytest.mark.parametrize("command", ["start", "restart"])
def test_cli_accepts_connectome_artifact(command: str) -> None:
    """Start and restart expose the `.connectome` startup option."""
    args = _build_parser().parse_args(
        [command, "--connectome", "brain.connectome"]
    )

    assert args.connectome == "brain.connectome"
    assert args.genome is None


@pytest.mark.parametrize("command", ["start", "restart"])
def test_cli_rejects_genome_and_connectome_together(command: str) -> None:
    """A process cannot start from two mutually exclusive brain artifacts."""
    with pytest.raises(SystemExit):
        _build_parser().parse_args(
            [
                command,
                "--genome",
                "brain.genome",
                "--connectome",
                "brain.connectome",
            ]
        )


def _write_test_config(path: Path) -> None:
    """Write explicit API settings used by download command tests."""
    path.write_text('[api]\nhost = "test-host"\nport = 9000\n')


def test_download_genome_writes_server_artifact(tmp_path: Path) -> None:
    """Genome download validates and writes encoded `.genome` bytes."""
    config_path = tmp_path / "config.toml"
    output_path = tmp_path / "download.genome"
    _write_test_config(config_path)
    client = Mock()
    client.download_artifact.return_value = (
        b'{"genome_schema_version":3,"blueprint":{}}'
    )
    argv = [
        "download",
        "genome",
        "--config",
        str(config_path),
        "--output",
        str(output_path),
        "--timeout",
        "5",
    ]

    with patch("feagi.genome.GenomeAPI", return_value=client) as api:
        result = main(argv)

    assert result == 0
    assert output_path.read_bytes() == client.download_artifact.return_value
    api.assert_called_once_with("http://test-host:9000", timeout=5.0)


@pytest.mark.parametrize("mode", ["full", "lite"])
def test_download_connectome_forwards_selected_mode(
    mode: str,
    tmp_path: Path,
) -> None:
    """Connectome download forwards explicit full and lite persistence modes."""
    config_path = tmp_path / "config.toml"
    output_path = tmp_path / f"{mode}.connectome"
    _write_test_config(config_path)
    client = Mock()
    argv = [
        "download",
        "connectome",
        "--config",
        str(config_path),
        "--output",
        str(output_path),
        "--timeout",
        "5",
        "--mode",
        mode,
    ]

    with patch("feagi.connectome.ConnectomeAPI", return_value=client) as api:
        result = main(argv)

    assert result == 0
    client.download_to_file.assert_called_once_with(output_path, mode=mode)
    api.assert_called_once_with("http://test-host:9000", timeout=5.0)


def test_download_requires_matching_artifact_extension(tmp_path: Path) -> None:
    """Genome download rejects an output path with the wrong extension."""
    config_path = tmp_path / "config.toml"
    _write_test_config(config_path)
    args = _build_parser().parse_args(
        [
            "download",
            "genome",
            "--config",
            str(config_path),
            "--output",
            str(tmp_path / "download.json"),
            "--timeout",
            "5",
        ]
    )

    assert _handle_download_command(args) == 1
