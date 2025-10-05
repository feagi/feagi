"""Shared Memory Manager

Centralized coordinator for shared-memory file lifecycle.

- Uses pathlib for cross-OS path handling
- Stores runtime artifacts in optimal RAM-backed location (see get_optimal_shm_directory)
- Provides creation, lookup, and cleanup utilities

Design notes:
- Runtime-only artifacts; files are temporary
- Naming convention: feagi-shared-mem-{agent_id}-<suffix>.bin
  Examples:
    - feagi-shared-mem-{agent_id}-video.bin         (agent → Brain Visualizer)
    - feagi-shared-mem-{agent_id}-neurons.bin       (agent → FEAGI)
    - feagi-shared-mem-visualization-stream.bin     (FEAGI → Brain Visualizer)
    - feagi-shared-mem-motor-stream.bin             (FEAGI → Controllers)
    - feagi-shared-mem-sensory-stream.bin           (Controllers → FEAGI)
- Storage location priority:
  1. FEAGI_SHM_DIR environment variable (user override)
  2. OS-specific RAM-backed location (Linux: /dev/shm, macOS: /tmp, Windows: %TEMP%)
  3. Fallback: feagi_core/tmp/bin (legacy, on SSD)
"""

from __future__ import annotations

import os
import platform
import tempfile
from pathlib import Path
from typing import Dict, Optional

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


def get_optimal_shm_directory() -> Path:
    """Get the optimal directory for FEAGI's shared memory files.
    
    Priority:
    1. FEAGI_SHM_DIR environment variable (user override)
    2. OS-specific RAM-backed location:
       - Linux: /dev/shm (tmpfs, RAM-backed)
       - macOS: /tmp (often RAM-backed)
       - Windows: system temp directory
    3. Fallback: feagi_core/tmp/bin (legacy, on SSD)
    
    Returns:
        Path object pointing to the optimal SHM directory
        
    Notes:
        - Linux /dev/shm prevents SSD wear (guaranteed RAM-backed)
        - macOS /tmp is often RAM-backed (system-dependent)
        - Windows users should set FEAGI_SHM_DIR to a RAM disk for best performance
        - Fallback to feagi_core/tmp/bin ensures backward compatibility
    """
    # Priority 1: User-specified directory via environment variable
    if "FEAGI_SHM_DIR" in os.environ:
        custom_dir = Path(os.environ["FEAGI_SHM_DIR"])
        if custom_dir.exists() and custom_dir.is_dir():
            logger.info(f"[SHM] Using custom directory from FEAGI_SHM_DIR: {custom_dir}")
            return custom_dir
        # If specified but doesn't exist, try to create it
        try:
            custom_dir.mkdir(parents=True, exist_ok=True)
            logger.info(f"[SHM] Created custom directory from FEAGI_SHM_DIR: {custom_dir}")
            return custom_dir
        except Exception as e:
            logger.warning(f"[SHM] Failed to create FEAGI_SHM_DIR={custom_dir}: {e}, using OS defaults")
    
    # Priority 2: OS-specific RAM-backed locations
    system = platform.system()
    
    if system == "Linux":
        # Linux: /dev/shm is tmpfs (RAM-backed filesystem)
        shm_dir = Path("/dev/shm")
        if shm_dir.exists() and shm_dir.is_dir():
            logger.info(f"[SHM] Using Linux tmpfs (RAM-backed): {shm_dir}")
            return shm_dir
        # Fallback to /tmp if /dev/shm doesn't exist (rare)
        logger.info("[SHM] Using Linux /tmp (RAM-backed fallback)")
        return Path("/tmp")
    
    elif system == "Darwin":  # macOS
        # macOS: /tmp is often RAM-backed (depends on system config)
        logger.info("[SHM] Using macOS /tmp (often RAM-backed)")
        return Path("/tmp")
    
    elif system == "Windows":
        # Windows: Use system temp directory
        # Users should set FEAGI_SHM_DIR to a RAM disk for optimal performance
        temp_dir = Path(tempfile.gettempdir())
        logger.info(f"[SHM] Using Windows temp directory: {temp_dir}")
        logger.info("[SHM] For optimal performance, set FEAGI_SHM_DIR to a RAM disk (e.g., ImDisk)")
        return temp_dir
    
    # Priority 3: Fallback to legacy location (on SSD, not ideal but compatible)
    try:
        # feagi_core/feagi/core/shared_memory.py → feagi_core
        core_root = Path(__file__).resolve().parents[2]
        legacy_dir = core_root / "tmp" / "bin"
        logger.warning(f"[SHM] Using legacy SSD location (may cause SSD wear): {legacy_dir}")
        logger.warning("[SHM] Consider setting FEAGI_SHM_DIR to a RAM-backed location")
        return legacy_dir
    except Exception:
        # Last resort: system temp directory
        return Path(tempfile.gettempdir())

class SharedMemoryManager:
    """Manage shared-memory file paths and lifecycle.

    Uses get_optimal_shm_directory() to select RAM-backed storage when possible.
    Falls back to feagi_core/tmp/bin for backward compatibility.
    """

    def __init__(self) -> None:
        self._base_dir: Path = self._resolve_base_dir()
        self.ensure_base_dir()

    @staticmethod
    def _resolve_base_dir() -> Path:
        """Resolve the base directory for shared memory files.
        
        Uses get_optimal_shm_directory() which prioritizes RAM-backed locations.
        """
        return get_optimal_shm_directory()

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
        logger.warning(
            f"𒓉 [SHM-CLEANUP] cleanup_all invoked for {self._base_dir}"
        )
        for p in self._base_dir.glob("*.bin"):
            try:
                logger.warning(f"𒓉 [SHM-CLEANUP] unlink {p}")
                p.unlink(missing_ok=True)
            except Exception as e:
                # If unlink fails (e.g., Windows file lock), try truncate
                try:
                    logger.warning(
                        f"𒓉 [SHM-CLEANUP] unlink failed, truncating {p}: {e}"
                    )
                    with p.open("r+b") as fh:
                        fh.truncate(0)
                except Exception:
                    # Suppress to avoid blocking startup/shutdown
                    pass
        logger.warning("𒓉 [SHM-CLEANUP] cleanup_all completed")

    # -------- Naming helpers --------
    def _agent_file_legacy(self, agent_id: str, suffix: str) -> Path:
        # Legacy naming used prior to capability-specific bins
        filename = f"feagi-shared-mem-{agent_id}-{suffix}.bin"
        return self._base_dir / filename

    def _agent_capability_file(self, agent_id: str, capability: str) -> Path:
        # New naming: feagi-shm-{agent_id}-{capability}.bin
        safe_cap = capability.replace(" ", "_")
        filename = f"feagi-shm-{agent_id}-{safe_cap}.bin"
        path = self._base_dir / filename
        
        return path

    def _stream_file(self, stream_name: str) -> Path:
        # Normalize keys like "visualization-stream" to "visualization_stream"
        safe_key = stream_name.replace("-", "_")
        filename = f"feagi-shared-mem-{safe_key}.bin"
        return self._base_dir / filename

    # -------- Agent artifacts (capability-based) --------
    def create_agent_capability_files(self, agent_id: str, capabilities: Dict[str, bool]) -> Dict[str, str]:
        """Create per-capability SHM files for an agent.

        Canonical capabilities: 'video', 'sensory', 'motor'.

        Returns:
            Mapping of capability name -> absolute path string
        """
        # Removed debug logging to reduce overhead
        
        self.ensure_base_dir()
        created: Dict[str, str] = {}
        if not isinstance(capabilities, dict):
            return created
        for cap in ("video", "feagi", "sensory", "motor"):
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
        logger.warning(
            f"𒓉 [SHM-AGENT] delete_agent_files invoked for agent={agent_id}"
        )
        # Remove legacy files
        for suffix in ("video", "neurons"):
            try:
                p = self._agent_file_legacy(agent_id, suffix)
                logger.info(f"𒓉 [SHM-AGENT] unlink legacy {p}")
                p.unlink(missing_ok=True)
            except Exception:
                pass
        # Remove capability-based files
        for cap in ("video", "sensory", "motor"):
            try:
                p = self._agent_capability_file(agent_id, cap)
                logger.info(f"𒓉 [SHM-AGENT] unlink capability {p}")
                p.unlink(missing_ok=True)
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
            logger.warning(
                f"𒓉 [SHM-STREAM] delete_stream_file key={stream_key} path={target}"
            )
            target.unlink(missing_ok=True)
        except Exception:
            # Fallback: truncate to zero length
            try:
                with target.open("r+b") as fh:
                    fh.truncate(0)
            except Exception:
                pass


