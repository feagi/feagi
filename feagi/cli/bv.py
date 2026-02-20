"""
Brain Visualizer CLI utilities.

This module resolves the optional feagi-bv runtime package and launches
the platform-specific Brain Visualizer binary with FEAGI network settings.
"""

from __future__ import annotations

import importlib.util
import os
import platform
import sys
from pathlib import Path
from typing import Dict, Tuple

import requests
import toml

from feagi.cli.bv_process import BVProcessManager

BV_RELEASES_URL = "https://github.com/feagi/brain-visualizer/releases"


def _is_macos() -> bool:
    """True if running on macOS (darwin)."""
    return platform.system().lower() == "darwin"


def _print_macos_bv_instructions() -> None:
    """Print instructions for obtaining Brain Visualizer on macOS (not shipped on PyPI)."""
    print("Brain Visualizer is not installed from PyPI on macOS (size limits).")
    print("")
    print("Download the Brain Visualizer for macOS from:")
    print(f"  {BV_RELEASES_URL}")
    print("")
    print("Use release v2.2.1 or above. Download the asset for your architecture")
    print("(macos-arm64 or macos-x86_64), extract the archive, and launch the .app file.")


class BrainVisualizerLaunchError(RuntimeError):
    """Raised when Brain Visualizer launch prerequisites are not met."""


def _load_feagi_config(config_path: Path) -> Dict[str, object]:
    """Load FEAGI configuration from a TOML file."""
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")
    try:
        return toml.load(config_path)
    except Exception as exc:
        raise BrainVisualizerLaunchError(
            f"Failed to parse FEAGI config: {config_path}"
        ) from exc


def _extract_network_settings(
    config: Dict[str, object]
) -> Tuple[str, int, str, int]:
    """Extract FEAGI API and WebSocket settings from config."""
    api_config = config.get("api")
    ws_config = config.get("websocket")

    if not isinstance(api_config, dict) or not isinstance(ws_config, dict):
        raise BrainVisualizerLaunchError(
            "Missing required [api] or [websocket] sections in config."
        )

    api_host = api_config.get("host")
    api_port = api_config.get("port")
    ws_host = ws_config.get("host")
    ws_port = ws_config.get("visualization_port")

    if not api_host or not ws_host:
        raise BrainVisualizerLaunchError(
            "Config must define api.host and websocket.host."
        )
    if api_port is None or ws_port is None:
        raise BrainVisualizerLaunchError(
            "Config must define api.port and websocket.visualization_port."
        )

    try:
        api_port_int = int(api_port)
        ws_port_int = int(ws_port)
    except (TypeError, ValueError) as exc:
        raise BrainVisualizerLaunchError(
            "Config ports must be numeric values."
        ) from exc

    return str(api_host), api_port_int, str(ws_host), ws_port_int


def _resolve_bv_binary() -> Tuple[Path, Path]:
    """Resolve the Brain Visualizer binary from platform-specific package."""
    system = platform.system().lower()
    machine = platform.machine().lower()

    def _first_existing_path(candidates: Tuple[Path, ...]) -> Path | None:
        """Return the first existing path from a candidate list."""
        for candidate in candidates:
            if candidate.exists():
                return candidate
        return None

    def _candidate_paths(
        base_dir: Path, names: Tuple[str, ...], subdirs: Tuple[str, ...]
    ) -> Tuple[Path, ...]:
        """Build deterministic candidate paths under base and subdirs."""
        paths = []
        for subdir in subdirs:
            root = base_dir / subdir if subdir else base_dir
            for name in names:
                paths.append(root / name)
        return tuple(paths)

    # Determine platform-specific package name
    if system.startswith("linux"):
        package_name = "feagi_bv_linux"
    elif system == "darwin":
        if machine in ("arm64", "aarch64"):
            package_name = "feagi_bv_macos_arm64"
        elif machine in ("x86_64", "amd64"):
            package_name = "feagi_bv_macos_x86_64"
        else:
            raise BrainVisualizerLaunchError(
                f"Unsupported macOS architecture: {machine}"
            )
    elif system.startswith("windows"):
        package_name = "feagi_bv_windows"
    else:
        raise BrainVisualizerLaunchError(f"Unsupported platform: {system}")

    # Try to find the platform-specific package
    spec = importlib.util.find_spec(package_name)
    if spec is None or not spec.submodule_search_locations:
        raise BrainVisualizerLaunchError(
            f"{package_name} runtime package not found. "
            f'Install with: pip install "feagi[bv]"'
        )

    package_dir = Path(spec.submodule_search_locations[0])
    bin_dir = package_dir / "bin"

    if system == "darwin":
        app_names = (
            "BrainVisualizer-Remote.app",
            "BrainVisualizer-Remote-x86_64.app",
            "BrainVisualizer-Remote-arm64.app",
            "BrainVisualizer.app",
            "BrainVisualizer-x86_64.app",
            "BrainVisualizer-arm64.app",
        )
        binary_names = ("Brain Visualizer", "BrainVisualizer")
        app_roots = ("", "macos")
        candidates = []
        for root in app_roots:
            for app_name in app_names:
                app_dir = bin_dir / root / app_name
                for binary_name in binary_names:
                    candidates.append(
                        app_dir / "Contents" / "MacOS" / binary_name
                    )

        binary = _first_existing_path(tuple(candidates))
        if binary is None:
            archive_candidates = []
            for root in app_roots:
                archive_candidates.extend(
                    sorted(
                        (bin_dir / root).glob("BrainVisualizer-Remote*.zip")
                    )
                )
            if archive_candidates:
                archives = ", ".join(str(path) for path in archive_candidates)
                raise BrainVisualizerLaunchError(
                    "BV app bundle not found. Archives present: "
                    f"{archives}. Extract the app bundle or reinstall."
                )
            raise BrainVisualizerLaunchError(
                "BV app bundle not found. Expected macOS .app bundle under "
                f"{bin_dir}."
            )
        working_dir = binary.parent
        pck_file = None
    elif system.startswith("linux"):
        binary_names = ("BrainVisualizer", "BrainVisualizer-Remote")
        candidates = _candidate_paths(bin_dir, binary_names, ("", "linux"))
        binary = _first_existing_path(candidates)
        if binary is None:
            raise BrainVisualizerLaunchError(
                "BV binary not found. Expected Linux binary under "
                f"{bin_dir}."
            )
        working_dir = binary.parent
        pck_file = None
    else:
        windows_subdirs = ("", "windows")
        exe_pck_pairs = (
            ("BrainVisualizer.exe", "BrainVisualizer.pck"),
            ("BrainVisualizer-Remote.exe", "BrainVisualizer-Remote.pck"),
        )

        binary = None
        pck_file = None
        working_dir = None
        for subdir in windows_subdirs:
            root = bin_dir / subdir if subdir else bin_dir
            for exe_name, pck_name in exe_pck_pairs:
                exe_candidate = root / exe_name
                pck_candidate = root / pck_name
                if exe_candidate.exists() and pck_candidate.exists():
                    binary = exe_candidate
                    pck_file = pck_candidate
                    working_dir = root
                    break
            if binary is not None:
                break

        if binary is None or working_dir is None:
            raise BrainVisualizerLaunchError(
                "BV Windows runtime not found. Expected one of: "
                "BrainVisualizer.exe + BrainVisualizer.pck or "
                "BrainVisualizer-Remote.exe + BrainVisualizer-Remote.pck "
                f"under {bin_dir}."
            )

    if pck_file is not None and not pck_file.exists():
        raise BrainVisualizerLaunchError(f"BV data file not found: {pck_file}")

    return binary, working_dir


def _build_bv_env(api_url: str, ws_host: str, ws_port: int) -> Dict[str, str]:
    """Build environment variables required by Brain Visualizer."""
    env = os.environ.copy()
    env.update(
        {
            "FEAGI_MODE": "remote",
            "FEAGI_API_URL": api_url,
            "FEAGI_WS_HOST": ws_host,
            "FEAGI_WS_PORT": str(ws_port),
        }
    )
    return env


def _check_feagi_running(api_url: str) -> bool:
    """
    Check if FEAGI is running and responding.
    
    Args:
        api_url: FEAGI API URL (e.g., http://127.0.0.1:8000)
    
    Returns:
        True if FEAGI is responding, False otherwise
    """
    try:
        response = requests.get(
            f"{api_url}/v1/system/health_check",
            timeout=2.0
        )
        return response.status_code == 200
    except Exception:
        return False


def start_bv(config_path: str | None = None) -> int:
    """
    Launch Brain Visualizer using configuration from a TOML file.
    
    Args:
        config_path: Path to config file. If None, uses default config.
    
    Returns:
        Process ID of launched Brain Visualizer
    
    Raises:
        BrainVisualizerLaunchError: If prerequisites not met
        BVProcessError: If process management fails
    """
    if _is_macos():
        _print_macos_bv_instructions()
        sys.exit(0)
    
    from feagi.config import ensure_default_config
    
    # Use default config if none specified
    if config_path is None:
        config_path = str(ensure_default_config())
    
    config = _load_feagi_config(Path(config_path))
    api_host, api_port, ws_host, ws_port = _extract_network_settings(config)
    api_url = f"http://{api_host}:{api_port}"
    
    # Check if FEAGI is running
    if not _check_feagi_running(api_url):
        raise BrainVisualizerLaunchError(
            f"FEAGI is not running at {api_url}.\n"
            "Start FEAGI first:\n"
            "  feagi start                     "
            "(starts with barebones genome)\n"
            "  feagi start --genome <path>     "
            "(starts with custom genome)"
        )
    
    binary, working_dir = _resolve_bv_binary()
    env = _build_bv_env(api_url, ws_host, ws_port)
    
    # Use process manager for robust lifecycle management
    manager = BVProcessManager()
    return manager.start(binary, working_dir, env)


def stop_bv(timeout: float = 10.0) -> bool:
    """
    Stop Brain Visualizer process gracefully.
    
    Args:
        timeout: Seconds to wait before force kill
    
    Returns:
        True if stopped, False if not running
    
    Raises:
        BVProcessError: If stop fails
    """
    manager = BVProcessManager()
    return manager.stop(timeout=timeout)


def status_bv() -> Dict[str, object]:
    """
    Get Brain Visualizer process status.
    
    Returns:
        Dictionary with status information:
        - running: bool
        - pid: int or None
        - pid_file: str
    """
    manager = BVProcessManager()
    return manager.get_status()


def restart_bv(config_path: str | None = None, timeout: float = 10.0) -> int:
    """
    Restart Brain Visualizer process.
    
    Args:
        config_path: Path to config file. If None, uses default config.
        timeout: Seconds to wait for stop before force kill
    
    Returns:
        New process ID
    
    Raises:
        BrainVisualizerLaunchError: If prerequisites not met
        BVProcessError: If restart fails
    """
    if _is_macos():
        _print_macos_bv_instructions()
        sys.exit(0)
    
    from feagi.config import ensure_default_config
    
    # Use default config if none specified
    if config_path is None:
        config_path = str(ensure_default_config())
    
    config = _load_feagi_config(Path(config_path))
    api_host, api_port, ws_host, ws_port = _extract_network_settings(config)
    binary, working_dir = _resolve_bv_binary()
    api_url = f"http://{api_host}:{api_port}"
    env = _build_bv_env(api_url, ws_host, ws_port)
    
    manager = BVProcessManager()
    return manager.restart(binary, working_dir, env, stop_timeout=timeout)
