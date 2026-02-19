"""
Integration tests for FEAGI config CLI entrypoint.
"""

from __future__ import annotations

from feagi.cli.main import main


def test_cli_config_show_outputs_contents(tmp_path, capsys):
    """Ensure main() prints the config file content."""
    config_path = tmp_path / "feagi_configuration.toml"
    config_contents = "[api]\nhost = \"127.0.0.1\"\nport = 8000\n"
    config_path.write_text(config_contents)

    result = main(["config", "show", "--config", str(config_path)])
    captured = capsys.readouterr()

    assert result == 0
    assert str(config_path) in captured.out
    assert config_contents in captured.out
