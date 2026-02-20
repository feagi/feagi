"""
Brain Visualizer Process Management

Robust lifecycle management for Brain Visualizer processes including:
- PID tracking and storage
- Process state monitoring
- Clean shutdown handling
- Prevention of duplicate instances
"""

from __future__ import annotations

import os
import platform
import signal
import subprocess
import time
from pathlib import Path
from typing import Optional

from feagi.paths import get_feagi_paths
from feagi.cli.process_utils import force_kill_process


class BVProcessError(RuntimeError):
    """Raised when Brain Visualizer process management fails."""


class BVProcessManager:
    """
    Manages Brain Visualizer process lifecycle.
    
    Features:
    - PID file management in ~/.feagi/cache/
    - Process state monitoring
    - Clean shutdown with timeout
    - Prevention of duplicate instances
    """
    
    def __init__(self):
        """Initialize process manager."""
        self.paths = get_feagi_paths()
        self.pid_file = self.paths.cache_dir / "bv.pid"
        self.paths.ensure_cache_dir()
        self._stdout_fd: Optional[int] = None
        self._stderr_fd: Optional[int] = None

    def _open_log_fds(self) -> tuple[int, int]:
        """Open log file descriptors for BV stdout/stderr."""
        log_dir = self.paths.create_log_run_dir(component="bv", retention=10)
        stdout_path = log_dir / "bv.log"
        stderr_path = log_dir / "bv_error.log"
        stdout_fd = os.open(
            str(stdout_path),
            os.O_WRONLY | os.O_CREAT | os.O_TRUNC,
            0o644,
        )
        stderr_fd = os.open(
            str(stderr_path),
            os.O_WRONLY | os.O_CREAT | os.O_TRUNC,
            0o644,
        )
        return stdout_fd, stderr_fd

    def _close_log_fds(self) -> None:
        """Close any open log file descriptors."""
        for fd_attr in ("_stdout_fd", "_stderr_fd"):
            fd = getattr(self, fd_attr)
            if fd is None:
                continue
            try:
                os.close(fd)
            except OSError:
                pass
            setattr(self, fd_attr, None)

    @staticmethod
    def _windows_creation_flags() -> int:
        """Return Windows creation flags that isolate BV from console signals."""
        if platform.system().lower().startswith("windows"):
            return (
                subprocess.CREATE_NEW_PROCESS_GROUP
                | subprocess.DETACHED_PROCESS
            )
        return 0
    
    def start(
        self,
        binary: Path,
        working_dir: Path,
        env: dict[str, str],
    ) -> int:
        """
        Start Brain Visualizer process.
        
        Args:
            binary: Path to BV binary
            working_dir: Working directory for process
            env: Environment variables
        
        Returns:
            Process ID
        
        Raises:
            BVProcessError: If already running or start fails
        """
        # Check for existing process
        if self.is_running():
            existing_pid = self.get_pid()
            raise BVProcessError(
                f"Brain Visualizer already running (PID: {existing_pid}). "
                "Stop it first with: feagi bv stop"
            )
        
        # Clean up stale PID file
        if self.pid_file.exists():
            self.pid_file.unlink()
        
        # Start process
        stdout_fd = None
        stderr_fd = None
        try:
            stdout_fd, stderr_fd = self._open_log_fds()
            self._stdout_fd = stdout_fd
            self._stderr_fd = stderr_fd
            process = subprocess.Popen(
                [str(binary)],
                cwd=str(working_dir),
                env=env,
                stdout=stdout_fd,
                stderr=stderr_fd,
                start_new_session=True,  # Detach from parent
                creationflags=self._windows_creation_flags(),
            )
        except Exception as exc:
            if stdout_fd is not None:
                try:
                    os.close(stdout_fd)
                except OSError:
                    pass
            if stderr_fd is not None:
                try:
                    os.close(stderr_fd)
                except OSError:
                    pass
            raise BVProcessError(
                f"Failed to start Brain Visualizer: {exc}"
            ) from exc
        
        # Verify process started
        time.sleep(0.5)
        if not self._is_process_running(process.pid):
            raise BVProcessError(
                "Brain Visualizer process died immediately after start"
            )
        
        # Store PID
        self._write_pid(process.pid)
        
        return process.pid
    
    def stop(self, timeout: float = 10.0) -> bool:
        """
        Stop Brain Visualizer process gracefully.
        
        Args:
            timeout: Seconds to wait before force kill
        
        Returns:
            True if stopped successfully, False if not running
        
        Raises:
            BVProcessError: If stop fails
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
            raise BVProcessError(
                f"Failed to send SIGTERM to PID {pid}: {exc}"
            ) from exc
        
        # Wait for graceful shutdown
        start_time = time.time()
        while time.time() - start_time < timeout:
            if not self._is_process_running(pid):
                self._cleanup_pid_file()
                self._close_log_fds()
                return True
            time.sleep(0.1)
        
        # Force kill if still running (SIGKILL on Unix, taskkill on Windows)
        try:
            force_kill_process(pid)
            time.sleep(0.5)
        except ProcessLookupError:
            pass
        except Exception as exc:
            raise BVProcessError(
                f"Failed to force kill PID {pid}: {exc}"
            ) from exc
        
        self._cleanup_pid_file()
        self._close_log_fds()
        
        # Final verification
        if self._is_process_running(pid):
            raise BVProcessError(
                f"Failed to stop Brain Visualizer (PID: {pid})"
            )
        
        return True
    
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
        Check if Brain Visualizer process is running.
        
        Returns:
            True if running, False otherwise
        """
        pid = self.get_pid()
        if pid is None:
            return False
        return self._is_process_running(pid)
    
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
    
    def restart(
        self,
        binary: Path,
        working_dir: Path,
        env: dict[str, str],
        stop_timeout: float = 10.0,
    ) -> int:
        """
        Restart Brain Visualizer process.
        
        Args:
            binary: Path to BV binary
            working_dir: Working directory
            env: Environment variables
            stop_timeout: Seconds to wait for stop
        
        Returns:
            New process ID
        
        Raises:
            BVProcessError: If restart fails
        """
        if self.is_running():
            self.stop(timeout=stop_timeout)
        
        return self.start(binary, working_dir, env)
    
    def _write_pid(self, pid: int) -> None:
        """Write PID to file."""
        try:
            self.pid_file.write_text(f"{pid}\n")
        except OSError as exc:
            raise BVProcessError(f"Failed to write PID file: {exc}") from exc
    
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
