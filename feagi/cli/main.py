"""
FEAGI CLI main entry point.

Command-line tools for FEAGI development and utilities.
"""

from __future__ import annotations

import argparse
import sys

from feagi.cli import bv as bv_cli


def _build_parser() -> argparse.ArgumentParser:
    """Create the top-level CLI argument parser."""
    parser = argparse.ArgumentParser(
        prog="feagi",
        description="FEAGI CLI utilities.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    bv_parser = subparsers.add_parser(
        "bv",
        help="Brain Visualizer utilities.",
    )
    bv_subparsers = bv_parser.add_subparsers(dest="bv_command", required=True)
    bv_start = bv_subparsers.add_parser(
        "start",
        help="Launch Brain Visualizer using FEAGI configuration.",
    )
    bv_start.add_argument(
        "--config",
        default="feagi_configuration.toml",
        help="Path to FEAGI configuration TOML file.",
    )

    subparsers.add_parser(
        "create-agent",
        help="Scaffold a new agent (coming in Phase 4).",
    )
    subparsers.add_parser(
        "build-package",
        help="Build marketplace package (coming in Phase 4).",
    )

    return parser


def _handle_bv_command(args: argparse.Namespace) -> int:
    """Handle Brain Visualizer subcommands."""
    if args.bv_command == "start":
        try:
            pid = bv_cli.start_bv(args.config)
            print(f"Brain Visualizer launched (PID: {pid})")
            return 0
        except (bv_cli.BrainVisualizerLaunchError, FileNotFoundError) as exc:
            print(f"Failed to launch Brain Visualizer: {exc}", file=sys.stderr)
            return 1
    raise ValueError(f"Unsupported BV command: {args.bv_command}")


def main(argv: list[str] | None = None) -> int:
    """Run the FEAGI CLI entry point."""
    parser = _build_parser()
    args = parser.parse_args(argv)

    if args.command == "bv":
        return _handle_bv_command(args)

    print("FEAGI CLI v3.0.0")
    print("\nAvailable commands:")
    print("  feagi bv start       - Launch Brain Visualizer")
    print("  feagi create-agent   - Scaffold new agent (Coming in Phase 4)")
    print("  feagi build-package  - Build marketplace package (Coming in Phase 4)")
    print("\nFor more information, visit https://docs.feagi.org")
    return 0


if __name__ == "__main__":
    sys.exit(main())

