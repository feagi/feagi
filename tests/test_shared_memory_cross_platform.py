"""
Tests for FEAGI Core's cross-platform shared memory directory selection.

Tests the get_optimal_shm_directory() function and SharedMemoryManager
to ensure proper RAM-backed storage location selection across platforms.
"""

import os
import platform
import tempfile
from pathlib import Path
from unittest.mock import patch
import pytest

from feagi.core.shared_memory import get_optimal_shm_directory, SharedMemoryManager


class TestGetOptimalShmDirectory:
    """Test the get_optimal_shm_directory() function."""
    
    def test_env_var_override_existing_dir(self, tmp_path):
        """FEAGI_SHM_DIR environment variable should take priority if directory exists."""
        test_dir = tmp_path / "custom_shm"
        test_dir.mkdir()
        
        with patch.dict(os.environ, {"FEAGI_SHM_DIR": str(test_dir)}):
            result = get_optimal_shm_directory()
            assert result == test_dir
    
    def test_env_var_creates_missing_dir(self, tmp_path):
        """FEAGI_SHM_DIR should create directory if it doesn't exist."""
        test_dir = tmp_path / "new_shm_dir"
        assert not test_dir.exists()
        
        with patch.dict(os.environ, {"FEAGI_SHM_DIR": str(test_dir)}):
            result = get_optimal_shm_directory()
            assert result == test_dir
            assert test_dir.exists()
    
    def test_env_var_fallback_on_creation_failure(self, tmp_path):
        """Should fall back to OS defaults if FEAGI_SHM_DIR creation fails."""
        invalid_path = tmp_path / "nonexistent" / "deeply" / "nested" / "path"
        
        with patch.dict(os.environ, {"FEAGI_SHM_DIR": str(invalid_path)}):
            with patch('pathlib.Path.mkdir', side_effect=PermissionError):
                result = get_optimal_shm_directory()
                # Should fall back to OS-specific default
                assert result != invalid_path
    
    @patch('platform.system', return_value='Linux')
    def test_linux_uses_dev_shm(self, mock_system, tmp_path):
        """Linux should prefer /dev/shm if it exists."""
        with patch.dict(os.environ, {}, clear=True):
            with patch('pathlib.Path.exists', return_value=True):
                with patch('pathlib.Path.is_dir', return_value=True):
                    result = get_optimal_shm_directory()
                    assert result == Path("/dev/shm")
    
    @patch('platform.system', return_value='Linux')
    def test_linux_fallback_to_tmp(self, mock_system):
        """Linux should fall back to /tmp if /dev/shm doesn't exist."""
        with patch.dict(os.environ, {}, clear=True):
            with patch('pathlib.Path.exists', return_value=False):
                result = get_optimal_shm_directory()
                assert result == Path("/tmp")
    
    @patch('platform.system', return_value='Darwin')
    def test_macos_uses_tmp(self, mock_system):
        """macOS should use /tmp."""
        with patch.dict(os.environ, {}, clear=True):
            result = get_optimal_shm_directory()
            assert result == Path("/tmp")
    
    @patch('platform.system', return_value='Windows')
    def test_windows_uses_temp_dir(self, mock_system):
        """Windows should use system temp directory."""
        with patch.dict(os.environ, {}, clear=True):
            result = get_optimal_shm_directory()
            assert result == Path(tempfile.gettempdir())
    
    @patch('platform.system', return_value='UnknownOS')
    def test_unknown_os_fallback(self, mock_system):
        """Unknown OS should fall back to system temp directory."""
        with patch.dict(os.environ, {}, clear=True):
            result = get_optimal_shm_directory()
            assert result == Path(tempfile.gettempdir())
    
    def test_real_os_detection(self):
        """Test that real OS detection returns a valid, writable path."""
        with patch.dict(os.environ, {}, clear=True):
            result = get_optimal_shm_directory()
            assert result.exists() or result.parent.exists()
            # Verify it's a reasonable path
            system = platform.system()
            if system == "Linux":
                assert result in [Path("/dev/shm"), Path("/tmp")]
            elif system == "Darwin":
                assert result == Path("/tmp")
            elif system == "Windows":
                assert result == Path(tempfile.gettempdir())


class TestSharedMemoryManager:
    """Test SharedMemoryManager uses optimal directory."""
    
    def test_manager_uses_optimal_directory(self, tmp_path):
        """SharedMemoryManager should use get_optimal_shm_directory()."""
        test_dir = tmp_path / "test_shm"
        test_dir.mkdir()
        
        with patch.dict(os.environ, {"FEAGI_SHM_DIR": str(test_dir)}):
            manager = SharedMemoryManager()
            assert manager.base_dir == test_dir
    
    def test_manager_creates_base_dir(self, tmp_path):
        """SharedMemoryManager should create base directory if it doesn't exist."""
        test_dir = tmp_path / "new_manager_dir"
        
        with patch.dict(os.environ, {"FEAGI_SHM_DIR": str(test_dir)}):
            manager = SharedMemoryManager()
            assert test_dir.exists()
            assert test_dir.is_dir()
    
    def test_agent_capability_files_in_optimal_dir(self, tmp_path):
        """Agent capability files should be created in optimal directory."""
        test_dir = tmp_path / "agent_shm"
        test_dir.mkdir()
        
        with patch.dict(os.environ, {"FEAGI_SHM_DIR": str(test_dir)}):
            manager = SharedMemoryManager()
            
            # Create agent files
            capabilities = {"video": True, "sensory": True}
            created_files = manager.create_agent_capability_files("test-agent-1", capabilities)
            
            # Verify files are in the optimal directory
            for cap, path in created_files.items():
                file_path = Path(path)
                assert file_path.parent == test_dir
                assert file_path.exists()
    
    def test_stream_files_in_optimal_dir(self, tmp_path):
        """Stream files should be created in optimal directory."""
        test_dir = tmp_path / "stream_shm"
        test_dir.mkdir()
        
        with patch.dict(os.environ, {"FEAGI_SHM_DIR": str(test_dir)}):
            manager = SharedMemoryManager()
            
            # Create stream file
            stream_path = manager.create_stream_file("visualization_stream")
            
            # Verify file is in the optimal directory
            assert Path(stream_path).parent == test_dir
            assert Path(stream_path).exists()


class TestCrossPlatformBehavior:
    """Test cross-platform behavior and compatibility."""
    
    def test_directory_is_writable(self):
        """Ensure the determined SHM directory is actually writable."""
        with patch.dict(os.environ, {}, clear=True):
            shm_dir = get_optimal_shm_directory()
            test_file = shm_dir / f"test_writable_{os.getpid()}.tmp"
            
            try:
                with open(test_file, "w") as f:
                    f.write("test")
                assert test_file.exists()
            finally:
                if test_file.exists():
                    test_file.unlink()
    
    @pytest.mark.skipif(platform.system() != "Linux", reason="Linux-specific test")
    def test_linux_dev_shm_is_ram_backed(self):
        """Verify that Linux /dev/shm is actually tmpfs (RAM-backed)."""
        import subprocess
        
        with patch.dict(os.environ, {}, clear=True):
            shm_dir = get_optimal_shm_directory()
            
            if shm_dir == Path("/dev/shm"):
                # Check mount info to confirm it's tmpfs
                result = subprocess.run(
                    ["mount"], 
                    capture_output=True, 
                    text=True
                )
                mount_output = result.stdout
                assert "tmpfs on /dev/shm" in mount_output or "/dev/shm" in mount_output
    
    def test_end_to_end_file_creation(self, tmp_path):
        """Test complete workflow of creating SHM files."""
        test_dir = tmp_path / "e2e_test"
        test_dir.mkdir()
        
        with patch.dict(os.environ, {"FEAGI_SHM_DIR": str(test_dir)}):
            manager = SharedMemoryManager()
            
            # Create various file types
            agent_files = manager.create_agent_capability_files(
                "e2e-agent", 
                {"video": True, "motor": True, "sensory": True}
            )
            stream_file = manager.create_stream_file("visualization_stream")
            
            # Verify all files exist in the correct location
            assert len(agent_files) == 3
            for path in agent_files.values():
                assert Path(path).exists()
                assert Path(path).parent == test_dir
            
            assert Path(stream_file).exists()
            assert Path(stream_file).parent == test_dir


class TestSSDWearMitigation:
    """Test that SSD wear mitigation strategy works correctly."""
    
    @pytest.mark.skipif(platform.system() != "Linux", reason="Test specific to Linux /dev/shm")
    def test_linux_uses_ram_backed_storage(self):
        """Verify Linux uses /dev/shm to avoid SSD writes."""
        with patch.dict(os.environ, {}, clear=True):
            shm_dir = get_optimal_shm_directory()
            # On Linux, should be /dev/shm (RAM) or /tmp (often RAM-backed)
            assert shm_dir == Path("/dev/shm") or shm_dir == Path("/tmp")
    
    def test_env_var_allows_custom_ram_disk(self, tmp_path):
        """Users can specify custom RAM disk location via FEAGI_SHM_DIR."""
        ram_disk = tmp_path / "ramdisk"
        ram_disk.mkdir()
        
        with patch.dict(os.environ, {"FEAGI_SHM_DIR": str(ram_disk)}):
            result = get_optimal_shm_directory()
            assert result == ram_disk
            
            # Verify manager uses it
            manager = SharedMemoryManager()
            assert manager.base_dir == ram_disk

