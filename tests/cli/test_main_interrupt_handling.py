"""
Tests for top-level CLI interrupt handling.
"""

from __future__ import annotations

import argparse

import feagi.cli.main as cli_main


def test_main_handles_keyboard_interrupt_during_start(monkeypatch, capsys):
    """main() returns 130 with a clear message on console interrupt."""

    class FakeParser:
        """Minimal parser stub that returns a start-command namespace."""

        @staticmethod
        def parse_args(_argv):
            """Return parsed args for a `feagi start` flow."""
            return argparse.Namespace(command="start", version=False)

    def interrupting_start_handler(_args):
        """Simulate signal-driven KeyboardInterrupt in start command."""
        raise KeyboardInterrupt()

    monkeypatch.setattr(cli_main, "_build_parser", lambda: FakeParser())
    monkeypatch.setattr(cli_main, "_handle_start_command", interrupting_start_handler)

    result = cli_main.main(["start"])
    captured = capsys.readouterr()

    assert result == 130
    assert "Startup interrupted by console signal" in captured.err
