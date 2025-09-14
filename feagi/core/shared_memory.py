"""Shared Memory Manager

Centralized coordinator for shared-memory file lifecycle.

- Uses pathlib for cross-OS path handling
- Stores runtime artifacts under feagi_core/tmp/bin
- Provides creation, lookup, and cleanup utilities

Design notes:
- Runtime-only artifacts; directory is git-ignored
- Naming convention: feagi-shared-mem-{agent_id}-<suffix>.bin
  Examples:
    - feagi-shared-mem-{agent_id}-video.bin         (agent → Brain Visualizer)
    - feagi-shared-mem-{agent_id}-neurons.bin       (agent → FEAGI)
    - feagi-shared-mem-visualization-stream.bin     (FEAGI → Brain Visualizer)
    - feagi-shared-mem-motor-stream.bin             (FEAGI → Controllers)
    - feagi-shared-mem-sensory-stream.bin           (Controllers → FEAGI)
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional


class SharedMemoryManager:
    """Manage shared-memory file paths and lifecycle.

    All paths are under the module-local tmp/bin folder to avoid OS-specific paths.
    """

    def __init__(self) -> None:
        self._base_dir: Path = self._resolve_base_dir()
        self.ensure_base_dir()

    @staticmethod
    def _resolve_base_dir() -> Path:
        # feagi_core/feagi/core/shared_memory.py → feagi_core
        core_root = Path(__file__).resolve().parents[2]
        return core_root / "tmp" / "bin"

    @property
    def base_dir(self) -> Path:
        return self._base_dir

    def ensure_base_dir(self) -> None:
        self._base_dir.mkdir(parents=True, exist_ok=True)

    def cleanup_all(self) -> None:
        """Remove all .bin files in the shared memory bin directory safely.

        Safety hardening:
        - Only operates within feagi_core/tmp/bin
        - Best-effort close by other processes is not required (we only remove files we created)
        - On permission or OS lock errors, truncate the file instead of raising
        - Files are created with 0o600 (owner read/write) when possible
        """
        if not self._base_dir.exists():
            return
        for p in self._base_dir.glob("*.bin"):
            try:
                p.unlink(missing_ok=True)
            except Exception:
                # If unlink fails (e.g., Windows file lock), try truncate
                try:
                    with p.open("r+b") as fh:
                        fh.truncate(0)
                except Exception:
                    # Suppress to avoid blocking startup/shutdown
                    pass

    # -------- Naming helpers --------
    def _agent_file_legacy(self, agent_id: str, suffix: str) -> Path:
        # Legacy naming used prior to capability-specific bins
        filename = f"feagi-shared-mem-{agent_id}-{suffix}.bin"
        return self._base_dir / filename

    def _agent_capability_file(self, agent_id: str, capability: str) -> Path:
        # New naming: feagi-shm-{agent_id}-{capability}.bin
        safe_cap = capability.replace(" ", "_")
        filename = f"feagi-shm-{agent_id}-{safe_cap}.bin"
        return self._base_dir / filename

    def _stream_file(self, stream_name: str) -> Path:
        # Normalize keys like "visualization-stream" to "visualization_stream"
        safe_key = stream_name.replace("-", "_")
        filename = f"feagi-shared-mem-{safe_key}.bin"
        return self._base_dir / filename

    # -------- Agent artifacts (capability-based) --------
    def create_agent_capability_files(self, agent_id: str, capabilities: Dict[str, bool]) -> Dict[str, str]:
        """Create per-capability SHM files for an agent.

        Known capabilities: 'video_stream', 'sensory', 'motor', 'neuron_visualization'

        Returns:
            Mapping of capability name -> absolute path string
        """
        self.ensure_base_dir()
        created: Dict[str, str] = {}
        if not isinstance(capabilities, dict):
            return created
        for cap in ("video_stream", "video_stream_raw", "video_stream_feagi", "sensory", "motor", "neuron_visualization"):
            if capabilities.get(cap, False):
                p = self._agent_capability_file(agent_id, cap)
                try:
                    p.touch(exist_ok=True)
                    try:
                        import os as _os
                        _os.chmod(str(p), 0o600)
                    except Exception:
                        pass
                except Exception:
                    pass
                created[cap] = str(p)
        return created

    def delete_agent_files(self, agent_id: str) -> None:
        # Remove legacy files
        for suffix in ("video", "neurons"):
            try:
                self._agent_file_legacy(agent_id, suffix).unlink(missing_ok=True)
            except Exception:
                pass
        # Remove capability-based files
        for cap in ("video_stream", "video_stream_raw", "video_stream_feagi", "sensory", "motor", "neuron_visualization"):
            try:
                self._agent_capability_file(agent_id, cap).unlink(missing_ok=True)
            except Exception:
                pass

    # -------- Core stream artifacts --------
    def create_stream_file(self, stream_key: str) -> str:
        """Create (touch) a core stream SHM file and return absolute path."""
        self.ensure_base_dir()
        p = self._stream_file(stream_key)
        try:
            p.touch(exist_ok=True)
            try:
                import os as _os
                _os.chmod(str(p), 0o600)
            except Exception:
                pass
        except Exception:
            pass
        return str(p)

    def delete_stream_file(self, stream_key: str) -> None:
        try:
            target = self._stream_file(stream_key)
            target.unlink(missing_ok=True)
        except Exception:
            # Fallback: truncate to zero length
            try:
                with target.open("r+b") as fh:
                    fh.truncate(0)
            except Exception:
                pass


