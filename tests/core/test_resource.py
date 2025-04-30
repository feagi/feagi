"""
Tests for FEAGI resource management functionality.
"""

import os
import pytest
from unittest.mock import patch, MagicMock

import psutil
import multiprocessing


@pytest.fixture
def mock_cpu_count():
    """Fixture to mock CPU count."""
    with patch('multiprocessing.cpu_count', return_value=8):
        yield


@pytest.fixture
def mock_memory_info():
    """Fixture to mock system memory information."""
    mock_memory = MagicMock()
    mock_memory.total = 16 * 1024 * 1024 * 1024  # 16 GB
    mock_memory.available = 8 * 1024 * 1024 * 1024  # 8 GB
    
    with patch('psutil.virtual_memory', return_value=mock_memory):
        yield


@pytest.fixture
def mock_gpu_info():
    """Fixture to mock GPU information."""
    mock_gpu = MagicMock()
    mock_gpu.name = "Test GPU"
    mock_gpu.memory_total = 8 * 1024 * 1024 * 1024  # 8 GB
    mock_gpu.memory_free = 4 * 1024 * 1024 * 1024  # 4 GB
    
    with patch('feagi.core.resource_mgr.get_gpu_info', return_value=[mock_gpu]):
        yield


def test_get_system_resources(mock_cpu_count, mock_memory_info):
    """Test retrieving system resources."""
    from feagi.core.resource_mgr import get_system_resources
    
    resources = get_system_resources()
    assert resources['cpu_count'] == 8
    assert resources['memory_total'] == 16 * 1024 * 1024 * 1024
    assert resources['memory_available'] == 8 * 1024 * 1024 * 1024


@pytest.mark.parametrize(
    "process_priority,expected_cores",
    [
        (1, 4),  # Priority 1 should get half of the cores
        (2, 2),  # Priority 2 should get a quarter
        (3, 1),  # Priority 3 should get minimal
    ]
)
def test_allocate_cpu_cores(mock_cpu_count, process_priority, expected_cores):
    """Test CPU core allocation based on process priority."""
    from feagi.core.resource_mgr import allocate_cpu_cores
    
    allocated = allocate_cpu_cores(process_priority)
    assert allocated == expected_cores


def test_resource_manager_singleton():
    """Test ResourceManager is a singleton."""
    from feagi.core.resource_mgr import ResourceManager
    
    manager1 = ResourceManager()
    manager2 = ResourceManager()
    
    assert manager1 is manager2


def test_resource_manager_process_management():
    """Test process management in ResourceManager."""
    from feagi.core.resource_mgr import ResourceManager
    
    manager = ResourceManager()
    
    # Mock process
    mock_process = MagicMock()
    mock_process.is_alive.return_value = True
    mock_process.pid = 12345
    
    # Register process
    manager.register_process("test_process", mock_process, priority=1)
    
    # Check process registration
    assert "test_process" in manager.processes
    assert manager.processes["test_process"]["process"] is mock_process
    assert manager.processes["test_process"]["priority"] == 1
    
    # Check process status
    status = manager.get_process_status("test_process")
    assert status["name"] == "test_process"
    assert status["is_alive"] is True
    assert status["priority"] == 1


def test_resource_manager_memory_tracking():
    """Test memory tracking in ResourceManager."""
    from feagi.core.resource_mgr import ResourceManager
    
    manager = ResourceManager()
    
    # Mock process
    with patch('psutil.Process') as mock_psutil_process:
        mock_process = MagicMock()
        mock_process.memory_info.return_value.rss = 1024 * 1024 * 100  # 100 MB
        mock_psutil_process.return_value = mock_process
        
        memory_usage = manager.get_process_memory_usage(os.getpid())
        assert memory_usage == 1024 * 1024 * 100  # 100 MB 