"""
Cross-platform process utilities for FEAGI CLI.

Provides force-kill and related helpers that work on Windows and Unix.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
from typing import Any

_PROCESS_QUERY_LIMITED_INFORMATION = 0x1000
_ERROR_ACCESS_DENIED = 5
_STILL_ACTIVE = 259


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


def _pid_alive_posix(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    except OSError:
        return False
    return True


def _pid_alive_windows(pid: int, kernel32: Any = None) -> bool:
    """Windows liveness probe; ``kernel32`` is injectable for tests."""
    import ctypes
    from ctypes import wintypes

    if kernel32 is None:
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)  # type: ignore[attr-defined]
        kernel32.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
        kernel32.OpenProcess.restype = wintypes.HANDLE
        kernel32.GetExitCodeProcess.argtypes = [
            wintypes.HANDLE,
            ctypes.POINTER(wintypes.DWORD),
        ]
        kernel32.GetExitCodeProcess.restype = wintypes.BOOL
        kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
        kernel32.CloseHandle.restype = wintypes.BOOL
        get_last_error = ctypes.get_last_error  # type: ignore[attr-defined]
    else:
        get_last_error = kernel32.get_last_error

    handle = kernel32.OpenProcess(_PROCESS_QUERY_LIMITED_INFORMATION, False, pid)
    if not handle:
        return bool(get_last_error() == _ERROR_ACCESS_DENIED)
    try:
        code = wintypes.DWORD()
        if not kernel32.GetExitCodeProcess(handle, ctypes.byref(code)):
            return False
        return int(code.value) == _STILL_ACTIVE
    finally:
        kernel32.CloseHandle(handle)


def pid_alive(pid: object) -> bool:
    """
    Return True if a process with this PID exists, without disturbing it.

    Never use os.kill(pid, 0) on Windows: Python's Windows os.kill calls
    TerminateProcess for any signal other than CTRL_C_EVENT / CTRL_BREAK_EVENT,
    so the "probe" would kill the target. Windows uses OpenProcess +
    GetExitCodeProcess instead.
    """
    if not isinstance(pid, int) or isinstance(pid, bool) or pid <= 0:
        return False
    if sys.platform == "win32":
        return _pid_alive_windows(pid)
    return _pid_alive_posix(pid)
