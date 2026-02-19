"""
Tests for Brain Visualizer Process Manager

Tests the robust process lifecycle management including:
- PID file management
- Process state tracking
- Graceful shutdown
- Prevention of duplicate instances
"""

import os
import signal
import subprocess
import time
from pathlib import Path

import pytest

from feagi.cli.bv_process import BVProcessError, BVProcessManager


@pytest.fixture
def manager(tmp_path):
    """Create a process manager with temporary PID file location."""
    manager = BVProcessManager()
    # Override PID file location for testing
    manager.pid_file = tmp_path / "test_bv.pid"
    return manager


@pytest.fixture
def dummy_process():
    """Create a long-running dummy process for testing."""
    # Use sleep command as a dummy process
    proc = subprocess.Popen(
        ["sleep", "30"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    yield proc
    # Cleanup
    try:
        proc.kill()
        proc.wait(timeout=1)
    except Exception:
        pass


def test_get_pid_no_file(manager):
    """Test get_pid returns None when PID file doesn't exist."""
    assert manager.get_pid() is None


def test_get_pid_with_file(manager):
    """Test get_pid reads PID from file."""
    manager.pid_file.write_text("12345\n")
    assert manager.get_pid() == 12345


def test_get_pid_invalid_content(manager):
    """Test get_pid handles invalid PID file content."""
    manager.pid_file.write_text("not_a_number\n")
    assert manager.get_pid() is None


def test_is_running_no_process(manager):
    """Test is_running returns False when no process."""
    assert manager.is_running() is False


def test_is_running_with_process(manager, dummy_process):
    """Test is_running returns True for running process."""
    manager._write_pid(dummy_process.pid)
    assert manager.is_running() is True


def test_is_running_dead_process(manager):
    """Test is_running returns False for dead process."""
    # Use a PID that definitely doesn't exist
    manager._write_pid(999999)
    assert manager.is_running() is False


def test_get_status_not_running(manager):
    """Test get_status when process not running."""
    status = manager.get_status()
    assert status["running"] is False
    assert status["pid"] is None
    assert "pid_file" in status


def test_get_status_running(manager, dummy_process):
    """Test get_status when process is running."""
    manager._write_pid(dummy_process.pid)
    status = manager.get_status()
    assert status["running"] is True
    assert status["pid"] == dummy_process.pid


def test_stop_not_running(manager):
    """Test stop returns False when process not running."""
    assert manager.stop() is False


def test_stop_graceful(manager, dummy_process):
    """Test graceful stop with SIGTERM."""
    manager._write_pid(dummy_process.pid)
    assert manager.stop(timeout=2.0) is True
    assert not manager.is_running()
    assert not manager.pid_file.exists()


def test_stop_force_kill(manager):
    """Test force kill when SIGTERM doesn't work."""
    # Create a process that ignores SIGTERM
    proc = subprocess.Popen(
        ["python", "-c", "import signal, time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(30)"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        manager._write_pid(proc.pid)
        # Stop with short timeout to trigger force kill
        assert manager.stop(timeout=0.5) is True
        assert not manager.is_running()
        assert not manager.pid_file.exists()
    finally:
        # Ensure cleanup
        try:
            os.kill(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def test_cleanup_stale_pid_file(manager):
    """Test cleanup of stale PID file."""
    # Write PID of non-existent process
    manager._write_pid(999999)
    manager._cleanup_pid_file()
    assert not manager.pid_file.exists()


def test_write_pid(manager):
    """Test PID file writing."""
    manager._write_pid(12345)
    assert manager.pid_file.exists()
    assert manager.pid_file.read_text().strip() == "12345"


def test_is_process_running_valid(dummy_process):
    """Test _is_process_running with valid PID."""
    assert BVProcessManager._is_process_running(dummy_process.pid) is True


def test_is_process_running_invalid():
    """Test _is_process_running with invalid PID."""
    assert BVProcessManager._is_process_running(999999) is False


def test_prevent_duplicate_start(manager, dummy_process):
    """Test prevention of starting when already running."""
    manager._write_pid(dummy_process.pid)
    
    with pytest.raises(BVProcessError, match="already running"):
        manager.start(
            binary=Path("/bin/echo"),
            working_dir=Path("/tmp"),
            env={},
        )
