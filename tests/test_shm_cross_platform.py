"""
Tests for cross-platform shared memory directory selection and functionality.

Tests cover:
- OS-specific directory selection (Linux, macOS, Windows)
- Environment variable override (FEAGI_SHM_DIR)
- Directory creation and fallback behavior
- SharedFrameWriter initialization with different paths
- ShmBytesWriter initialization with different paths
"""

import os
import platform
import tempfile
from pathlib import Path
from unittest import mock

import numpy as np
import pytest

from feagi_connector.utils.shm import (
    SharedFrameWriter,
    ShmBytesWriter,
    get_shm_directory,
)


class TestGetShmDirectory:
    """Tests for get_shm_directory() function."""

    def test_env_var_override_existing_dir(self, tmp_path):
        """Test that FEAGI_SHM_DIR environment variable takes priority."""
        custom_dir = tmp_path / "custom_shm"
        custom_dir.mkdir()
        
        with mock.patch.dict(os.environ, {"FEAGI_SHM_DIR": str(custom_dir)}):
            result = get_shm_directory()
            assert result == custom_dir

    def test_env_var_override_creates_dir(self, tmp_path):
        """Test that FEAGI_SHM_DIR creates the directory if it doesn't exist."""
        custom_dir = tmp_path / "new_shm_dir"
        assert not custom_dir.exists()
        
        with mock.patch.dict(os.environ, {"FEAGI_SHM_DIR": str(custom_dir)}):
            result = get_shm_directory()
            assert result == custom_dir
            assert custom_dir.exists()
            assert custom_dir.is_dir()

    def test_env_var_invalid_fallback(self):
        """Test that invalid FEAGI_SHM_DIR falls back to OS defaults."""
        with mock.patch.dict(os.environ, {"FEAGI_SHM_DIR": "/nonexistent/path/that/cannot/be/created"}):
            result = get_shm_directory()
            # Should fall back to OS-specific default
            assert result.exists()
            assert result.is_dir()

    @mock.patch("platform.system", return_value="Linux")
    def test_linux_uses_dev_shm(self, mock_system):
        """Test that Linux uses /dev/shm when available."""
        with mock.patch.dict(os.environ, {}, clear=True):
            if Path("/dev/shm").exists():
                result = get_shm_directory()
                assert result == Path("/dev/shm")

    @mock.patch("platform.system", return_value="Linux")
    def test_linux_fallback_to_tmp(self, mock_system):
        """Test that Linux falls back to /tmp if /dev/shm doesn't exist."""
        with mock.patch.dict(os.environ, {}, clear=True):
            with mock.patch("pathlib.Path.exists", return_value=False):
                result = get_shm_directory()
                assert result == Path("/tmp")

    @mock.patch("platform.system", return_value="Darwin")
    def test_macos_uses_tmp(self, mock_system):
        """Test that macOS uses /tmp."""
        with mock.patch.dict(os.environ, {}, clear=True):
            result = get_shm_directory()
            assert result == Path("/tmp")

    @mock.patch("platform.system", return_value="Windows")
    def test_windows_uses_temp(self, mock_system):
        """Test that Windows uses system temp directory."""
        with mock.patch.dict(os.environ, {}, clear=True):
            result = get_shm_directory()
            expected = Path(tempfile.gettempdir())
            assert result == expected

    @mock.patch("platform.system", return_value="UnknownOS")
    def test_unknown_os_uses_temp(self, mock_system):
        """Test that unknown OS uses system temp directory."""
        with mock.patch.dict(os.environ, {}, clear=True):
            result = get_shm_directory()
            expected = Path(tempfile.gettempdir())
            assert result == expected


class TestSharedFrameWriter:
    """Tests for SharedFrameWriter with cross-platform path handling."""

    def test_init_with_explicit_path(self, tmp_path):
        """Test SharedFrameWriter initialization with explicit path."""
        test_path = tmp_path / "explicit_video.bin"
        writer = SharedFrameWriter(path=test_path)
        assert writer.path == test_path

    def test_init_without_path_uses_shm_dir(self):
        """Test SharedFrameWriter initialization without path uses get_shm_directory()."""
        writer = SharedFrameWriter()
        shm_dir = get_shm_directory()
        assert writer.path.parent == shm_dir
        assert writer.path.name == "feagi_video_shm--temp.bin"

    def test_init_with_custom_env_var(self, tmp_path):
        """Test SharedFrameWriter respects FEAGI_SHM_DIR environment variable."""
        custom_dir = tmp_path / "env_shm"
        custom_dir.mkdir()
        
        with mock.patch.dict(os.environ, {"FEAGI_SHM_DIR": str(custom_dir)}):
            writer = SharedFrameWriter()
            assert writer.path.parent == custom_dir

    def test_write_frame_creates_file(self, tmp_path):
        """Test that writing a frame creates the SHM file."""
        test_path = tmp_path / "video_test.bin"
        writer = SharedFrameWriter(path=test_path)
        
        # Create a test frame
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        frame[:, :] = [255, 0, 0]  # Red frame
        
        writer.write_frame(frame)
        
        assert test_path.exists()
        assert test_path.stat().st_size > 0

    def test_write_multiple_frames(self, tmp_path):
        """Test writing multiple frames to ring buffer."""
        test_path = tmp_path / "video_multi.bin"
        writer = SharedFrameWriter(path=test_path, num_slots=3)
        
        # Write frames
        for i in range(5):
            frame = np.full((50, 50, 3), i * 50, dtype=np.uint8)
            writer.write_frame(frame)
        
        assert writer._frame_seq == 5
        assert writer._write_index in [0, 1, 2]  # Should wrap around


class TestShmBytesWriter:
    """Tests for ShmBytesWriter with cross-platform path handling."""

    def test_init_with_explicit_path(self, tmp_path):
        """Test ShmBytesWriter initialization with explicit path."""
        test_path = tmp_path / "explicit_bytes.bin"
        writer = ShmBytesWriter(path=test_path)
        assert writer.path == test_path

    def test_init_without_path_uses_shm_dir(self):
        """Test ShmBytesWriter initialization without path uses get_shm_directory()."""
        writer = ShmBytesWriter()
        shm_dir = get_shm_directory()
        assert writer.path.parent == shm_dir
        assert writer.path.name == "feagi_bytes_shm--temp.bin"

    def test_init_with_custom_env_var(self, tmp_path):
        """Test ShmBytesWriter respects FEAGI_SHM_DIR environment variable."""
        custom_dir = tmp_path / "env_bytes_shm"
        custom_dir.mkdir()
        
        with mock.patch.dict(os.environ, {"FEAGI_SHM_DIR": str(custom_dir)}):
            writer = ShmBytesWriter()
            assert writer.path.parent == custom_dir

    def test_write_bytes(self, tmp_path):
        """Test writing bytes to SHM."""
        test_path = tmp_path / "bytes_test.bin"
        writer = ShmBytesWriter(path=test_path, num_slots=4, slot_size=1024)
        
        # Write test data
        test_data = b"Hello, FEAGI!"
        writer.write(test_data)
        
        assert test_path.exists()
        assert test_path.stat().st_size > 0

    def test_write_multiple_payloads(self, tmp_path):
        """Test writing multiple byte payloads."""
        test_path = tmp_path / "bytes_multi.bin"
        writer = ShmBytesWriter(path=test_path, num_slots=4, slot_size=1024)
        
        # Write multiple payloads
        for i in range(10):
            data = f"Payload {i}".encode()
            writer.write(data)
        
        assert writer._frame_seq == 10


class TestCrossPlatformBehavior:
    """Integration tests for cross-platform behavior."""

    def test_shm_directory_is_writable(self):
        """Test that the selected SHM directory is writable."""
        shm_dir = get_shm_directory()
        assert shm_dir.exists()
        assert os.access(shm_dir, os.W_OK)

    def test_real_os_detection(self):
        """Test that real OS is detected correctly."""
        actual_system = platform.system()
        shm_dir = get_shm_directory()
        
        if actual_system == "Linux":
            # Linux should prefer /dev/shm if it exists
            if Path("/dev/shm").exists():
                assert shm_dir == Path("/dev/shm")
            else:
                assert shm_dir == Path("/tmp")
        
        elif actual_system == "Darwin":
            # macOS should use /tmp
            assert shm_dir == Path("/tmp")
        
        elif actual_system == "Windows":
            # Windows should use temp directory
            assert shm_dir == Path(tempfile.gettempdir())

    def test_end_to_end_frame_write(self, tmp_path):
        """End-to-end test: write frame with auto-detected SHM dir."""
        # Set custom SHM dir for testing
        custom_dir = tmp_path / "e2e_shm"
        custom_dir.mkdir()
        
        with mock.patch.dict(os.environ, {"FEAGI_SHM_DIR": str(custom_dir)}):
            writer = SharedFrameWriter()
            
            # Write a frame
            frame = np.random.randint(0, 256, (100, 100, 3), dtype=np.uint8)
            writer.write_frame(frame)
            
            # Verify file is in custom dir
            assert writer.path.parent == custom_dir
            assert writer.path.exists()
            
            writer.close()


@pytest.mark.skipif(
    platform.system() == "Windows",
    reason="SSD wear test only meaningful on systems with /dev/shm"
)
class TestSSDWearMitigation:
    """Tests to verify SSD wear mitigation strategies."""

    def test_linux_uses_ram_backed_storage(self):
        """Verify that Linux uses RAM-backed /dev/shm to prevent SSD wear."""
        if platform.system() == "Linux" and Path("/dev/shm").exists():
            with mock.patch.dict(os.environ, {}, clear=True):
                shm_dir = get_shm_directory()
                assert shm_dir == Path("/dev/shm")
                
                # Verify it's a tmpfs (RAM-backed)
                # This is the key to preventing SSD wear
                import subprocess
                result = subprocess.run(
                    ["df", "-T", "/dev/shm"],
                    capture_output=True,
                    text=True
                )
                assert "tmpfs" in result.stdout

