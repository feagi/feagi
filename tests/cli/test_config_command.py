"""
Tests for FEAGI config CLI commands.
"""

from __future__ import annotations

import argparse

from feagi.cli.main import _handle_config_command


def test_config_show_with_explicit_path(tmp_path, capsys):
    """Ensure config show prints the config file and contents."""
    config_path = tmp_path / "feagi_configuration.toml"
    config_contents = "[api]\nhost = \"127.0.0.1\"\nport = 8000\n"
    config_path.write_text(config_contents)

    args = argparse.Namespace(config_command="show", config=str(config_path))
    result = _handle_config_command(args)
    captured = capsys.readouterr()

    assert result == 0
    assert str(config_path) in captured.out
    assert config_contents in captured.out


def test_config_show_missing_file(tmp_path, capsys):
    """Ensure config show fails when file is missing."""
    missing_path = tmp_path / "missing.toml"
    args = argparse.Namespace(config_command="show", config=str(missing_path))
    result = _handle_config_command(args)
    captured = capsys.readouterr()

    assert result == 1
    assert "Config file not found" in captured.err
