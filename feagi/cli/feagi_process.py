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
import time
from typing import Optional

from feagi.paths import get_feagi_paths


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
    
    def stop(self, timeout: float = 10.0) -> bool:
        """
        Stop FEAGI process gracefully.
        
        Args:
            timeout: Seconds to wait before force kill
        
        Returns:
            True if stopped successfully, False if not running
        
        Raises:
            FeagiProcessError: If stop fails
        """
        pid = self.get_pid()
        if pid is None:
            return False
        
        if not self.is_running():
            self._cleanup_pid_file()
            return False
        
        # Try graceful shutdown first (SIGTERM)
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            self._cleanup_pid_file()
            return False
        except Exception as exc:
            raise FeagiProcessError(
                f"Failed to send SIGTERM to PID {pid}: {exc}"
            ) from exc
        
        # Wait for graceful shutdown
        start_time = time.time()
        while time.time() - start_time < timeout:
            if not self._is_process_running(pid):
                self._cleanup_pid_file()
                return True
            time.sleep(0.1)
        
        # Force kill if still running (SIGKILL)
        try:
            os.kill(pid, signal.SIGKILL)
            time.sleep(0.5)
        except ProcessLookupError:
            pass
        except Exception as exc:
            raise FeagiProcessError(
                f"Failed to force kill PID {pid}: {exc}"
            ) from exc
        
        self._cleanup_pid_file()
        
        # Final verification
        if self._is_process_running(pid):
            raise FeagiProcessError(
                f"Failed to stop FEAGI (PID: {pid})"
            )
        
        return True
    
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
            # Send signal 0 to check if process exists
            os.kill(pid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            # Process exists but we don't have permission
            return True
        except Exception:
            return False
