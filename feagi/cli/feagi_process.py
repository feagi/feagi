"""
FEAGI Process Management

Robust lifecycle management for FEAGI processes including:
- PID tracking and storage
- Process state monitoring
- Clean shutdown handling
- Prevention of duplicate instances
"""

from __future__ import annotations

import os
import signal
import sys
import time
from typing import Optional

from feagi.paths import get_feagi_paths
from feagi.cli.process_utils import check_process_exists_windows, force_kill_process


class FeagiProcessError(RuntimeError):
    """Raised when FEAGI process management fails."""


class FeagiProcessManager:
    """
    Manages FEAGI process lifecycle.
    
    Features:
    - PID file management in ~/.feagi/cache/
    - Process state monitoring
    - Clean shutdown with timeout
    - Prevention of duplicate instances
    """
    
    def __init__(self):
        """Initialize process manager."""
        self.paths = get_feagi_paths()
        self.pid_file = self.paths.cache_dir / "feagi.pid"
        self.paths.ensure_cache_dir()
    
    def store_pid(self, pid: int) -> None:
        """
        Store FEAGI process PID.
        
        Args:
            pid: Process ID to store
        
        Raises:
            FeagiProcessError: If PID file write fails
        """
        try:
            self.pid_file.write_text(f"{pid}\n")
        except OSError as exc:
            raise FeagiProcessError(
                f"Failed to write PID file: {exc}"
            ) from exc
    
    def get_pid(self) -> Optional[int]:
        """
        Get stored PID from file.
        
        Returns:
            Process ID or None if not found
        """
        if not self.pid_file.exists():
            return None
        
        try:
            pid_str = self.pid_file.read_text().strip()
            return int(pid_str)
        except (ValueError, OSError):
            return None
    
    def is_running(self) -> bool:
        """
        Check if FEAGI process is running.
        
        Returns:
            True if running, False otherwise
        """
        pid = self.get_pid()
        if pid is None:
            return False
        return self._is_process_running(pid)
    
    def _is_permission_error(self, exc: BaseException) -> bool:
        """Check if exception is permission-related (e.g. WinError 5 Access denied)."""
        if isinstance(exc, PermissionError):
            return True
        if isinstance(exc, OSError):
            return getattr(exc, "winerror", None) == 5 or getattr(
                exc, "errno", None
            ) in (5, 13)
        return False

    def stop(
        self, timeout: float = 10.0, force_clear_pid: bool = False
    ) -> tuple[bool, bool]:
        """
        Stop FEAGI process gracefully.

        Args:
            timeout: Seconds to wait before force kill
            force_clear_pid: If True, remove PID file when process cannot be
                killed (e.g. Access denied on Windows). Unblocks feagi start.

        Returns:
            Tuple (success, cleared_only). success True if stopped or PID
            cleared. cleared_only True when PID file was cleared because
            process could not be killed.

        Raises:
            FeagiProcessError: If stop fails and force_clear_pid is False
        """
        pid = self.get_pid()
        if pid is None:
            return False, False

        if not self.is_running():
            self._cleanup_pid_file()
            return False, False

        # Try graceful shutdown first (SIGTERM)
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            self._cleanup_pid_file()
            return False, False
        except Exception as exc:
            if self._is_permission_error(exc):
                try:
                    force_kill_process(pid)
                    time.sleep(0.5)
                except ProcessLookupError:
                    self._cleanup_pid_file()
                    return True, False
                except Exception as kill_exc:
                    if self._is_permission_error(kill_exc) and force_clear_pid:
                        self._cleanup_pid_file()
                        return True, True
                    raise FeagiProcessError(
                        f"Failed to stop PID {pid}: {kill_exc}. "
                        f"Try: feagi stop --force"
                    ) from kill_exc
            else:
                raise FeagiProcessError(
                    f"Failed to send SIGTERM to PID {pid}: {exc}"
                ) from exc

        # Wait for graceful shutdown
        start_time = time.time()
        while time.time() - start_time < timeout:
            if not self._is_process_running(pid):
                self._cleanup_pid_file()
                return True, False
            time.sleep(0.1)

        # Force kill if still running (SIGKILL on Unix, taskkill on Windows)
        try:
            force_kill_process(pid)
            time.sleep(0.5)
        except ProcessLookupError:
            pass
        except Exception as exc:
            if self._is_permission_error(exc) and force_clear_pid:
                self._cleanup_pid_file()
                return True, True
            raise FeagiProcessError(
                f"Failed to force kill PID {pid}: {exc}. "
                f"Try: feagi stop --force"
            ) from exc

        self._cleanup_pid_file()

        # Final verification
        if self._is_process_running(pid):
            if force_clear_pid:
                self._cleanup_pid_file()
                return True, True
            raise FeagiProcessError(
                f"Failed to stop FEAGI (PID: {pid})"
            )

        return True, False
    
    def get_status(self) -> dict[str, object]:
        """
        Get detailed process status.
        
        Returns:
            Dictionary with status information:
            - running: bool
            - pid: int or None
            - pid_file: str
        """
        pid = self.get_pid()
        running = self.is_running() if pid else False
        
        return {
            "running": running,
            "pid": pid,
            "pid_file": str(self.pid_file),
        }
    
    def _cleanup_pid_file(self) -> None:
        """Remove PID file."""
        if self.pid_file.exists():
            try:
                self.pid_file.unlink()
            except OSError:
                pass
    
    @staticmethod
    def _is_process_running(pid: int) -> bool:
        """Check if process with given PID is running."""
        try:
            os.kill(pid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            # On Windows, os.kill(pid,0) can raise PermissionError for
            # non-existent PIDs (reused) or processes we can't access.
            # Use tasklist to verify - if not found, treat as gone.
            if sys.platform == "win32" and not check_process_exists_windows(pid):
                return False
            return True
        except Exception:
            return False
