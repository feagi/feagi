"""
FEAGI CLI main entry point.

Command-line tools for FEAGI development and utilities.
"""

# ruff: noqa: E501

from __future__ import annotations

import argparse
import sys

from feagi.cli import bv as bv_cli
from feagi.engine import FeagiEngine


def _build_parser() -> argparse.ArgumentParser:
    """Create the top-level CLI argument parser."""
    from feagi.paths import get_feagi_paths
    
    paths = get_feagi_paths()
    
    epilog = (
        f"\nFEAGI Directories:\n"
        f"  Config:      {paths.config_dir}\n"
        f"  Logs:        {paths.logs_dir}\n"
        f"  Cache:       {paths.cache_dir}\n"
        f"  Genomes:     {paths.genomes_dir}\n"
        f"  Connectomes: {paths.connectomes_dir}\n"
    )
    
    parser = argparse.ArgumentParser(
        prog="feagi",
        description="FEAGI CLI utilities.",
        epilog=epilog,
        formatter_class=argparse.RawDescriptionHelpFormatter,
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
    
    # bv start
    bv_start = bv_subparsers.add_parser(
        "start",
        help="Launch Brain Visualizer using FEAGI configuration.",
    )
    bv_start.add_argument(
        "--config",
        default=None,
        help="Path to FEAGI configuration TOML file (default: uses ~/.feagi/config/feagi_configuration.toml).",
    )
    
    # bv stop
    bv_stop = bv_subparsers.add_parser(
        "stop",
        help="Stop running Brain Visualizer process.",
    )
    bv_stop.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Seconds to wait before force kill (default: 10).",
    )
    
    # bv status
    bv_subparsers.add_parser(
        "status",
        help="Check Brain Visualizer process status.",
    )
    
    # bv restart
    bv_restart = bv_subparsers.add_parser(
        "restart",
        help="Restart Brain Visualizer process.",
    )
    bv_restart.add_argument(
        "--config",
        default=None,
        help="Path to FEAGI configuration TOML file (default: uses ~/.feagi/config/feagi_configuration.toml).",
    )
    bv_restart.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for stop before force kill (default: 10).",
    )

    start_parser = subparsers.add_parser(
        "start",
        help="Start FEAGI using a config and genome/connectome.",
    )
    start_parser.add_argument(
        "--config",
        default=None,
        help="Path to FEAGI configuration TOML file (default: uses ~/.feagi/config/feagi_configuration.toml).",
    )
    start_mode = start_parser.add_mutually_exclusive_group(required=False)
    start_mode.add_argument(
        "--genome",
        help="Path to genome JSON file (default: uses barebones genome).",
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
    
    # feagi stop
    stop_parser = subparsers.add_parser(
        "stop",
        help="Stop running FEAGI process.",
    )
    stop_parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Seconds to wait before force kill (default: 10).",
    )
    
    # feagi status
    subparsers.add_parser(
        "status",
        help="Check FEAGI process status.",
    )
    
    # feagi restart
    restart_parser = subparsers.add_parser(
        "restart",
        help="Restart FEAGI process.",
    )
    restart_parser.add_argument(
        "--config",
        default=None,
        help="Path to FEAGI configuration TOML file (default: uses ~/.feagi/config/feagi_configuration.toml).",
    )
    restart_parser.add_argument(
        "--genome",
        help="Path to genome JSON file (default: uses barebones genome).",
    )
    restart_parser.add_argument(
        "--connectome",
        help="Path to connectome file.",
    )
    restart_parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for stop before force kill (default: 10).",
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

    return parser


def _handle_bv_command(args: argparse.Namespace) -> int:
    """Handle Brain Visualizer subcommands."""
    from feagi.cli.bv_process import BVProcessError
    
    if args.bv_command == "start":
        try:
            pid = bv_cli.start_bv(args.config)
            print(f"Brain Visualizer started (PID: {pid})")
            return 0
        except (bv_cli.BrainVisualizerLaunchError, BVProcessError, FileNotFoundError) as exc:
            print(f"Failed to start Brain Visualizer: {exc}", file=sys.stderr)
            return 1
    
    elif args.bv_command == "stop":
        try:
            timeout = args.timeout if hasattr(args, 'timeout') else 10.0
            stopped = bv_cli.stop_bv(timeout=timeout)
            if stopped:
                print("Brain Visualizer stopped successfully")
            else:
                print("Brain Visualizer is not running")
            return 0
        except BVProcessError as exc:
            print(f"Failed to stop Brain Visualizer: {exc}", file=sys.stderr)
            return 1
    
    elif args.bv_command == "status":
        try:
            status = bv_cli.status_bv()
            if status["running"]:
                print(f"Brain Visualizer is running (PID: {status['pid']})")
            else:
                print("Brain Visualizer is not running")
            print(f"PID file: {status['pid_file']}")
            return 0
        except Exception as exc:
            print(f"Failed to get status: {exc}", file=sys.stderr)
            return 1
    
    elif args.bv_command == "restart":
        try:
            timeout = args.timeout if hasattr(args, 'timeout') else 10.0
            pid = bv_cli.restart_bv(args.config, timeout=timeout)
            print(f"Brain Visualizer restarted (PID: {pid})")
            return 0
        except (bv_cli.BrainVisualizerLaunchError, BVProcessError, FileNotFoundError) as exc:
            print(f"Failed to restart Brain Visualizer: {exc}", file=sys.stderr)
            return 1
    
    raise ValueError(f"Unsupported BV command: {args.bv_command}")


def _handle_init_command(args: argparse.Namespace) -> int:
    """Handle FEAGI init command."""
    from feagi.config import generate_default_config, init_feagi_environment
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


def _handle_stop_command(args: argparse.Namespace) -> int:
    """Handle FEAGI stop command."""
    from feagi.cli.bv_process import BVProcessManager
    from feagi.cli.feagi_process import FeagiProcessError, FeagiProcessManager
    
    timeout = args.timeout if hasattr(args, 'timeout') else 10.0
    
    # Stop Brain Visualizer first (depends on FEAGI)
    bv_manager = BVProcessManager()
    if bv_manager.is_running():
        print("Stopping Brain Visualizer...")
        try:
            bv_manager.stop(timeout=timeout)
            print("Brain Visualizer stopped")
        except Exception as exc:
            print(f"Warning: Failed to stop Brain Visualizer: {exc}")
    
    # Stop FEAGI
    try:
        feagi_manager = FeagiProcessManager()
        stopped = feagi_manager.stop(timeout=timeout)
        if stopped:
            print("FEAGI stopped successfully")
        else:
            print("FEAGI is not running")
        return 0
    except FeagiProcessError as exc:
        print(f"Failed to stop FEAGI: {exc}", file=sys.stderr)
        return 1


def _handle_status_command(args: argparse.Namespace) -> int:
    """Handle FEAGI status command."""
    from feagi.cli.feagi_process import FeagiProcessManager
    
    try:
        manager = FeagiProcessManager()
        status = manager.get_status()
        if status["running"]:
            print(f"FEAGI is running (PID: {status['pid']})")
        else:
            print("FEAGI is not running")
        print(f"PID file: {status['pid_file']}")
        return 0
    except Exception as exc:
        print(f"Failed to get status: {exc}", file=sys.stderr)
        return 1


def _handle_restart_command(args: argparse.Namespace) -> int:
    """Handle FEAGI restart command."""
    from feagi.cli.feagi_process import FeagiProcessError, FeagiProcessManager
    from feagi.config import ensure_default_config
    
    # Stop existing instance
    try:
        manager = FeagiProcessManager()
        if manager.is_running():
            timeout = args.timeout if hasattr(args, 'timeout') else 10.0
            print("Stopping FEAGI...")
            manager.stop(timeout=timeout)
    except FeagiProcessError as exc:
        print(f"Failed to stop FEAGI: {exc}", file=sys.stderr)
        return 1
    
    # Start new instance
    config_path = args.config
    if config_path is None:
        config_path = str(ensure_default_config())
    
    # Enable daemon mode for CLI (detach process)
    import os
    os.environ["FEAGI_DAEMON_MODE"] = "1"

    engine = FeagiEngine()
    engine.load_config(config_path)
    config = engine.config
    if config is None:
        print("Failed to load FEAGI config.", file=sys.stderr)
        return 1
    timeouts_config = config.get("timeouts")
    if not isinstance(timeouts_config, dict):
        print("Config must define a [timeouts] section.", file=sys.stderr)
        return 1
    service_startup_timeout = timeouts_config.get("service_startup")
    if service_startup_timeout is None:
        print(
            "Config must define timeouts.service_startup.",
            file=sys.stderr
        )
        return 1
    try:
        service_startup_seconds = float(service_startup_timeout)
    except (TypeError, ValueError):
        print(
            "Config timeouts.service_startup must be a numeric value.",
            file=sys.stderr
        )
        return 1
    
    if hasattr(args, 'genome') and args.genome:
        engine.load_genome(args.genome)
    elif hasattr(args, 'connectome') and args.connectome:
        engine.load_connectome(args.connectome)

    started = engine.start(wait_for_ready=False)

    if not started:
        print("Failed to restart FEAGI", file=sys.stderr)
        return 1
    
    pid = engine.process.pid
    manager.store_pid(pid)
    
    # Verify process is still running
    import time
    time.sleep(service_startup_seconds)
    if not manager.is_running():
        print(
            "FEAGI process died immediately after start. "
            "Check logs for errors.",
            file=sys.stderr
        )
        manager._cleanup_pid_file()
        return 1
    
    print(f"FEAGI restarted successfully (PID: {pid})")
    return 0


def _handle_start_command(args: argparse.Namespace) -> int:
    """Handle FEAGI start command."""
    from feagi.cli.feagi_process import FeagiProcessManager
    from feagi.config import ensure_default_config
    
    if args.wait and args.timeout is None:
        print("Missing --timeout when using --wait", file=sys.stderr)
        return 1
    
    # Check if already running
    manager = FeagiProcessManager()
    if manager.is_running():
        pid = manager.get_pid()
        print(
            f"FEAGI is already running (PID: {pid})",
            file=sys.stderr
        )
        print("Stop it first with: feagi stop", file=sys.stderr)
        return 1
    
    # Use default config if none specified
    config_path = args.config
    if config_path is None:
        config_path = str(ensure_default_config())
    
    # Enable daemon mode for CLI (detach process)
    import os
    os.environ["FEAGI_DAEMON_MODE"] = "1"

    engine = FeagiEngine()
    engine.load_config(config_path)
    
    # Load genome/connectome if provided
    has_genome = False
    if args.genome:
        engine.load_genome(args.genome)
        has_genome = True
    elif args.connectome:
        engine.load_connectome(args.connectome)
        has_genome = True

    if args.wait:
        started = engine.start(wait_for_ready=True, timeout=args.timeout)
    else:
        started = engine.start(wait_for_ready=False)

    if not started:
        print("Failed to start FEAGI", file=sys.stderr)
        return 1
    
    # Store PID for process management
    pid = engine.process.pid
    manager.store_pid(pid)
    
    # Verify process is still running after a brief delay
    import time
    time.sleep(service_startup_seconds)
    if not manager.is_running():
        print(
            "FEAGI process died immediately after start. "
            "Check logs for errors.",
            file=sys.stderr
        )
        manager._cleanup_pid_file()
        return 1
    
    # Load barebones genome if no genome specified
    if not has_genome:
        print("No genome specified, loading barebones genome...")
        try:
            import requests
            
            # Get API config
            api_config = config.get("api")
            if not isinstance(api_config, dict):
                raise ValueError("Config must define an [api] section.")
            api_host = api_config.get("host")
            api_port = api_config.get("port")
            if not api_host:
                raise ValueError("Config must define api.host.")
            if api_port is None:
                raise ValueError("Config must define api.port.")
            try:
                api_port_int = int(api_port)
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    "Config api.port must be a numeric value."
                ) from exc
            
            api_url = f"http://{api_host}:{api_port_int}"
            
            # Wait for API to be ready
            time.sleep(service_startup_seconds)
            
            response = requests.post(
                f"{api_url}/v1/genome/upload/barebones",
                timeout=service_startup_seconds
            )
            if response.status_code == 200:
                print("Barebones genome loaded successfully")
            else:
                print(
                    f"Warning: Failed to load barebones genome "
                    f"(status {response.status_code})",
                    file=sys.stderr
                )
        except Exception as exc:
            print(
                f"Warning: Could not load barebones genome: {exc}",
                file=sys.stderr
            )
    
    print(f"FEAGI started successfully (PID: {pid})")
    return 0


def main(argv: list[str] | None = None) -> int:
    """Run the FEAGI CLI entry point."""
    parser = _build_parser()
    args = parser.parse_args(argv)

    # Get version from package metadata
    try:
        from importlib.metadata import version
        # Try feagi-core first (the actual package), then feagi (meta-package)
        try:
            pkg_version = version("feagi-core")
        except Exception:
            pkg_version = version("feagi")
    except Exception:
        # Fall back to module version
        try:
            import feagi
            pkg_version = feagi.__version__
        except Exception:
            pkg_version = "unknown"

    # Handle --version flag
    if args.version:
        print(f"FEAGI CLI v{pkg_version}")
        
        # Try to detect installed Brain Visualizer package
        bv_version = None
        try:
            # First try the meta package (most accurate)
            try:
                bv_version = version("feagi-bv")
            except Exception:
                pass
            
            # Fallback to platform-specific package if meta package not found
            if not bv_version:
                import platform
                system = platform.system().lower()
                if system == "darwin":
                    # Check for architecture-specific macOS packages
                    machine = platform.machine().lower()
                    if machine in ('arm64', 'aarch64'):
                        bv_version = version("feagi-bv-macos-arm64")
                    elif machine in ('x86_64', 'amd64'):
                        bv_version = version("feagi-bv-macos-x86_64")
                    # Fallback to old package name for backward compatibility
                    if not bv_version:
                        try:
                            bv_version = version("feagi-bv-macos")
                        except Exception:
                            pass
                elif system.startswith("linux"):
                    bv_version = version("feagi-bv-linux")
                elif system == "win32":
                    bv_version = version("feagi-bv-windows")
        except Exception:
            pass
        
        if bv_version:
            print(f"Brain Visualizer v{bv_version}")
        else:
            print('Brain Visualizer: not installed (install with: pip install "feagi[bv]")')
        
        return 0

    # Handle commands
    if args.command == "bv":
        return _handle_bv_command(args)
    if args.command == "init":
        return _handle_init_command(args)
    if args.command == "start":
        return _handle_start_command(args)
    if args.command == "stop":
        return _handle_stop_command(args)
    if args.command == "status":
        return _handle_status_command(args)
    if args.command == "restart":
        return _handle_restart_command(args)

    # No command specified - show help
    from feagi.paths import get_feagi_paths
    
    paths = get_feagi_paths()
    
    print(f"FEAGI CLI v{pkg_version}")
    print("\nAvailable commands:")
    print("  feagi start          - Start FEAGI")
    print("  feagi stop           - Stop FEAGI")
    print("  feagi status         - Check FEAGI status")
    print("  feagi restart        - Restart FEAGI")
    print("  feagi bv start       - Launch Brain Visualizer")
    print("  feagi bv stop        - Stop Brain Visualizer")
    print("  feagi bv status      - Check Brain Visualizer status")
    print("  feagi bv restart     - Restart Brain Visualizer")
    print("  feagi init           - Initialize FEAGI environment")
    print("\nFEAGI Directories:")
    print(f"  Config:      {paths.config_dir}")
    print(f"  Logs:        {paths.logs_dir}")
    print(f"  Cache:       {paths.cache_dir}")
    print(f"  Genomes:     {paths.genomes_dir}")
    print(f"  Connectomes: {paths.connectomes_dir}")
    print("\nFor more information: https://github.com/feagi/feagi/tree/main/docs")
    return 0


if __name__ == "__main__":
    sys.exit(main())

