"""
Cross-platform process utilities for FEAGI CLI.

Provides force-kill and related helpers that work on Windows and Unix.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys


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
        # Process already gone: taskkill often returns 128 or 1 with "not found"
        stderr_lower = (result.stderr or "").lower()
        if result.returncode in (1, 128) or "not found" in stderr_lower:
            raise ProcessLookupError(pid)
        raise OSError(
            result.returncode,
            f"taskkill failed: {result.stderr or result.stdout or result.returncode}",
        )
    else:
        os.kill(pid, signal.SIGKILL)
