"""
Brain Visualizer runtime launcher.

Provides a Python API similar to FeagiEngine for starting BV.
"""

# @ruff-skip: ssl install failure blocked required check - cleanup task: BV-API-001

from __future__ import annotations

import os
import platform
import subprocess
from pathlib import Path
from typing import Dict, Optional, Tuple

import toml


class BrainVisualizerLaunchError(RuntimeError):
    """Raised when Brain Visualizer launch prerequisites are not met."""


class BrainVisualizer:
    """
    Brain Visualizer runtime launcher.

    Example:
        bv = BrainVisualizer()
        bv.load_config("feagi_configuration.toml")
        pid = bv.start()
    """

    def __init__(self) -> None:
        self._config_path: Optional[Path] = None
        self._process: Optional[subprocess.Popen] = None

    def load_config(self, config_path: str) -> "BrainVisualizer":
        """Load FEAGI configuration TOML file for BV connection settings."""
        path = Path(config_path)
        if not path.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")
        self._config_path = path
        return self

    def start(self) -> int:
        """Start Brain Visualizer and return its process ID."""
        if self._process is not None and self._process.poll() is None:
            return self._process.pid

        if self._config_path is None:
            raise BrainVisualizerLaunchError("Config not loaded. Call load_config().")

        config = _load_feagi_config(self._config_path)
        api_host, api_port, ws_host, ws_port = _extract_network_settings(config)
        binary, working_dir = _resolve_bv_binary()
        api_url = f"http://{api_host}:{api_port}"
        env = _build_bv_env(api_url, ws_host, ws_port)

        self._process = subprocess.Popen(
            [str(binary)],
            cwd=str(working_dir),
            env=env,
        )
        return self._process.pid


def _load_feagi_config(config_path: Path) -> Dict[str, object]:
    """Load FEAGI configuration from a TOML file."""
    try:
        return toml.load(config_path)
    except Exception as exc:
        raise BrainVisualizerLaunchError(
            f"Failed to parse FEAGI config: {config_path}"
        ) from exc


def _extract_network_settings(config: Dict[str, object]) -> Tuple[str, int, str, int]:
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
    """Resolve the Brain Visualizer binary from the feagi-bv package."""
    package_dir = Path(__file__).resolve().parent
    bin_dir = package_dir / "bin"
    system = platform.system().lower()

    if system == "windows":
        binary = bin_dir / "windows" / "BrainVisualizer.exe"
        working_dir = binary.parent
    elif system == "linux":
        binary = bin_dir / "linux" / "BrainVisualizer"
        working_dir = binary.parent
    elif system == "darwin":
        app_dir = bin_dir / "macos" / "BrainVisualizer.app"
        binary = app_dir / "Contents" / "MacOS" / "BrainVisualizer"
        working_dir = binary.parent
    else:
        raise BrainVisualizerLaunchError(f"Unsupported platform: {system}")

    if not binary.exists():
        raise BrainVisualizerLaunchError(f"BV binary not found: {binary}")

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
