"""
Cross-platform process utilities for FEAGI CLI.

Provides force-kill and related helpers that work on Windows and Unix.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys


def check_process_exists_windows(pid: int) -> bool:
    """
    Check if a process exists on Windows using tasklist.

    More reliable than os.kill(pid, 0) when PID may have been reused
    or when we get PermissionError from a non-FEAGI process.

    Returns:
        True if process exists, False if not found.
    """
    if sys.platform != "win32":
        return True  # Caller should use os.kill on Unix
    result = subprocess.run(
        ["tasklist", "/FI", f"PID eq {pid}", "/NH"],
        capture_output=True,
        timeout=5,
        text=True,
    )
    if result.returncode != 0:
        return False
    out = (result.stdout or "").strip().lower()
    # "INFO: No tasks are running..." = process not found
    if "no tasks" in out or "info:" in out:
        return False
    return str(pid) in out


def force_kill_process(pid: int) -> None:
    """
    Force kill a process by PID.

    On Windows, SIGKILL does not exist; use taskkill /F /PID.
    On Unix, use SIGKILL.

    Raises:
        ProcessLookupError: Process no longer exists (caller may treat as success).
        OSError: Other OS error (e.g. permission).
        subprocess.TimeoutExpired: taskkill timed out (Windows).
    """
    if sys.platform == "win32":
        result = subprocess.run(
            ["taskkill", "/F", "/PID", str(pid)],
            capture_output=True,
            timeout=10,
            text=True,
        )
        if result.returncode == 0:
            return
        stderr_lower = (result.stderr or "").lower()
        # 128 = process not found; 1 with "not found" = process gone
        # 1 with "access denied" = process exists but we cannot kill it
        if result.returncode == 128 or "not found" in stderr_lower:
            raise ProcessLookupError(pid)
        if result.returncode == 1 and (
            "access" in stderr_lower and "denied" in stderr_lower
        ):
            raise OSError(
                result.returncode,
                f"taskkill access denied: {result.stderr or result.stdout}",
            )
        # Other 1 or unknown: treat as process gone (e.g. already exited)
        if result.returncode == 1:
            raise ProcessLookupError(pid)
        raise OSError(
            result.returncode,
            f"taskkill failed: {result.stderr or result.stdout or result.returncode}",
        )
    else:
        os.kill(pid, signal.SIGKILL)
