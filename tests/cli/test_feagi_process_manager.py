"""
Tests for FEAGI Process Manager

Tests process lifecycle management including:
- PID file management
- Graceful shutdown
- Windows permission error handling (--force)
"""

import subprocess
import sys
from unittest.mock import patch

import pytest

from feagi.cli.feagi_process import FeagiProcessError, FeagiProcessManager


@pytest.fixture
def manager(tmp_path):
    """Create a process manager with temporary PID file location."""
    manager = FeagiProcessManager()
    manager.pid_file = tmp_path / "feagi.pid"
    return manager


@pytest.fixture
def dummy_process():
    """Create a long-running dummy process for testing (cross-platform)."""
    proc = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(30)"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    yield proc
    try:
        proc.kill()
        proc.wait(timeout=1)
    except Exception:
        pass


def test_stop_not_running(manager):
    """Test stop returns (False, False) when process not running."""
    manager.pid_file.write_text("999999\n")
    success, cleared = manager.stop()
    assert success is False
    assert cleared is False


def test_stop_graceful(manager, dummy_process):
    """Test graceful stop with SIGTERM."""
    manager.pid_file.write_text(f"{dummy_process.pid}\n")
    success, cleared = manager.stop(timeout=2.0)
    assert success is True
    assert cleared is False
    assert not manager.pid_file.exists()


def test_stop_force_clear_pid_on_permission_error(manager):
    """Test force_clear_pid clears PID file when both SIGTERM and force_kill fail."""
    manager.pid_file.write_text("12345\n")
    perm_err = PermissionError(5, "Access is denied")
    with patch("feagi.cli.feagi_process.os.kill", side_effect=perm_err):
        with patch(
            "feagi.cli.feagi_process.force_kill_process",
            side_effect=perm_err,
        ):
            success, cleared = manager.stop(force_clear_pid=True)
    assert success is True
    assert cleared is True
    assert not manager.pid_file.exists()


def test_stop_raises_on_permission_error_without_force(manager):
    """Test stop raises when permission error and force_clear_pid is False."""
    manager.pid_file.write_text("12345\n")
    perm_err = PermissionError(5, "Access is denied")
    with patch("feagi.cli.feagi_process.os.kill", side_effect=perm_err):
        with patch(
            "feagi.cli.feagi_process.force_kill_process",
            side_effect=perm_err,
        ):
            with pytest.raises(FeagiProcessError, match="feagi stop --force"):
                manager.stop(force_clear_pid=False)


def test_stop_tries_force_kill_when_sigterm_fails(manager, dummy_process):
    """Test that force_kill is tried when SIGTERM fails with permission error."""
    manager.pid_file.write_text(f"{dummy_process.pid}\n")
    perm_err = PermissionError(5, "Access is denied")
    with patch("feagi.cli.feagi_process.os.kill", side_effect=perm_err):
        success, cleared = manager.stop(timeout=2.0)
    assert success is True
    assert cleared is False
    assert not manager.pid_file.exists()
