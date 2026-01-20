"""
FEAGI CLI main entry point.

Command-line tools for FEAGI development and utilities.
"""

# @ruff-skip: ssl install failure blocked required check - cleanup task: BV-CLI-001

from __future__ import annotations

import argparse
import sys

from feagi.cli import bv as bv_cli
from feagi.engine import FeagiEngine


def _build_parser() -> argparse.ArgumentParser:
    """Create the top-level CLI argument parser."""
    parser = argparse.ArgumentParser(
        prog="feagi",
        description="FEAGI CLI utilities.",
    )
    parser.add_argument(
        "--version",
        action="store_true",
        help="Show FEAGI CLI version and exit.",
    )
    subparsers = parser.add_subparsers(dest="command", required=False)

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
        default=None,
        help="Path to FEAGI configuration TOML file (default: uses ~/.feagi/config/feagi_configuration.toml).",
    )

    start_parser = subparsers.add_parser(
        "start",
        help="Start FEAGI using a config and genome/connectome.",
    )
    start_parser.add_argument(
        "--config",
        required=True,
        help="Path to FEAGI configuration TOML file.",
    )
    start_mode = start_parser.add_mutually_exclusive_group(required=True)
    start_mode.add_argument(
        "--genome",
        help="Path to genome JSON file.",
    )
    start_mode.add_argument(
        "--connectome",
        help="Path to connectome file.",
    )
    start_parser.add_argument(
        "--wait",
        action="store_true",
        help="Wait for FEAGI to report ready before returning.",
    )
    start_parser.add_argument(
        "--timeout",
        type=float,
        help="Seconds to wait for readiness (required with --wait).",
    )

    init_parser = subparsers.add_parser(
        "init",
        help="Initialize FEAGI environment and generate default configuration.",
    )
    init_parser.add_argument(
        "--config-only",
        action="store_true",
        help="Only generate config file, don't create all directories.",
    )
    init_parser.add_argument(
        "--force",
        action="store_true",
        help="Overwrite existing configuration file.",
    )
    init_parser.add_argument(
        "--output",
        help="Custom output path for config file.",
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


def _handle_init_command(args: argparse.Namespace) -> int:
    """Handle FEAGI init command."""
    from feagi.config import generate_default_config, init_feagi_environment
    from feagi.paths import get_feagi_paths
    from pathlib import Path
    
    try:
        if args.config_only:
            # Only generate config file
            output_path = Path(args.output) if args.output else None
            config_path = generate_default_config(output_path, force=args.force)
            print(f"Generated configuration file: {config_path}")
        else:
            # Initialize full environment
            env = init_feagi_environment()
            print("FEAGI environment initialized successfully!")
            print(f"\nConfiguration: {env['config_file']}")
            print(f"Genomes:       {env['genomes_dir']}")
            print(f"Connectomes:   {env['connectomes_dir']}")
            print(f"Logs:          {env['logs_dir']}")
            print(f"Cache:         {env['cache_dir']}")
            print("\nNext steps:")
            print("  1. Edit configuration if needed:")
            print(f"     {env['config_file']}")
            print("  2. Start Brain Visualizer:")
            print("     feagi bv start")
        return 0
    except FileExistsError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        print("Use --force to overwrite existing files.", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"Failed to initialize FEAGI environment: {exc}", file=sys.stderr)
        return 1


def _handle_start_command(args: argparse.Namespace) -> int:
    """Handle FEAGI start command."""
    if args.wait and args.timeout is None:
        print("Missing --timeout when using --wait", file=sys.stderr)
        return 1

    engine = FeagiEngine()
    engine.load_config(args.config)
    if args.genome:
        engine.load_genome(args.genome)
    if args.connectome:
        engine.load_connectome(args.connectome)

    if args.wait:
        started = engine.start(wait_for_ready=True, timeout=args.timeout)
    else:
        started = engine.start(wait_for_ready=False)

    if not started:
        print("Failed to start FEAGI", file=sys.stderr)
        return 1

    print("FEAGI started successfully")
    return 0


def main(argv: list[str] | None = None) -> int:
    """Run the FEAGI CLI entry point."""
    parser = _build_parser()
    args = parser.parse_args(argv)

    # Get version from package metadata
    try:
        from importlib.metadata import version
        pkg_version = version("feagi")
    except Exception:
        pkg_version = "unknown"

    # Handle --version flag
    if args.version:
        print(f"FEAGI CLI v{pkg_version}")
        
        # Try to detect installed Brain Visualizer package
        bv_version = None
        try:
            import platform
            system = platform.system().lower()
            if system == "darwin":
                bv_version = version("feagi-bv-macos")
            elif system.startswith("linux"):
                bv_version = version("feagi-bv-linux")
            elif system == "win32":
                bv_version = version("feagi-bv-windows")
        except Exception:
            pass
        
        if bv_version:
            print(f"Brain Visualizer v{bv_version}")
        else:
            print("Brain Visualizer: not installed (install with: pip install feagi[bv])")
        
        return 0

    # Handle commands
    if args.command == "bv":
        return _handle_bv_command(args)
    if args.command == "init":
        return _handle_init_command(args)
    if args.command == "start":
        return _handle_start_command(args)

    # No command specified - show help
    print(f"FEAGI CLI v{pkg_version}")
    print("\nAvailable commands:")
    print("  feagi bv start       - Launch Brain Visualizer")
    print("  feagi init           - Initialize FEAGI environment")
    print("  feagi start          - Start FEAGI with genome/connectome")
    print("  feagi create-agent   - Scaffold new agent (Coming in Phase 4)")
    print("  feagi build-package  - Build marketplace package (Coming in Phase 4)")
    print("\nFor more information, visit https://docs.feagi.org")
    return 0


if __name__ == "__main__":
    sys.exit(main())

