"""
Tests for the integration between ResourceManager and backend selection.
"""

import unittest
import pytest
from unittest.mock import patch, MagicMock

from feagi.core.resource_mgr import ResourceManager
from feagi.core.backend import BackendType, get_backend, get_available_backends
from feagi.core.backend.interface import determine_best_backend


class TestResourceBackendIntegration(unittest.TestCase):
    
    def setUp(self):
        # Clear any existing instance
        ResourceManager._instances = {}
    
    def tearDown(self):
        # Clean up
        ResourceManager._instances = {}
    
    @patch('feagi.core.resource_mgr.ResourceManager._detect_resources')
    def test_gpu_detection_integration(self, mock_detect):
        """Test that backend selection uses ResourceManager for GPU detection."""
        # Mock ResourceManager to report GPU available
        mock_detect.return_value = {
            "cpu_count": 8,
            "gpu_available": True,
            "gpu_count": 1,
            "metal_available": False,
            "webgpu_available": False,
            "memory": 16000000000,
        }
        # Patch _BACKENDS to include CUDA
        from feagi.core.backend import interface as backend_interface
        backend_interface._BACKENDS[BackendType.CUDA] = MagicMock()
        backend_interface._BACKENDS[BackendType.CPU] = MagicMock()
        # Create ResourceManager instance with mocked resources
        resource_mgr = ResourceManager()
        # Test that backends are correctly detected
        backends = get_available_backends()
        self.assertIn(BackendType.CPU, backends)
        self.assertIn(BackendType.CUDA, backends)
        # Test that best backend is CUDA
        best = determine_best_backend()
        self.assertEqual(best, BackendType.CUDA)
    
    @patch('feagi.core.resource_mgr.ResourceManager._detect_resources')
    def test_metal_detection_integration(self, mock_detect):
        """Test that backend selection correctly detects Metal."""
        # Mock ResourceManager to report Metal available
        mock_detect.return_value = {
            "cpu_count": 8,
            "gpu_available": False,
            "gpu_count": 0,
            "metal_available": True,
            "webgpu_available": False,
            "memory": 16000000000,
        }
        # Patch _BACKENDS to include METAL
        from feagi.core.backend import interface as backend_interface
        backend_interface._BACKENDS[BackendType.METAL] = MagicMock()
        backend_interface._BACKENDS[BackendType.CPU] = MagicMock()
        # Create ResourceManager instance with mocked resources
        resource_mgr = ResourceManager()
        # Test backends detection
        backends = get_available_backends()
        self.assertIn(BackendType.CPU, backends)
        self.assertIn(BackendType.METAL, backends)
        # Test that best backend is Metal
        best = determine_best_backend()
        self.assertEqual(best, BackendType.METAL)
    
    @patch('feagi.core.resource_mgr.ResourceManager._detect_resources')
    def test_webgpu_detection_integration(self, mock_detect):
        """Test that backend selection correctly detects WebGPU."""
        # Mock ResourceManager to report WebGPU available
        mock_detect.return_value = {
            "cpu_count": 8,
            "gpu_available": False,
            "gpu_count": 0,
            "metal_available": False,
            "webgpu_available": True,
            "memory": 16000000000,
        }
        # Patch _BACKENDS to include WEBGPU
        from feagi.core.backend import interface as backend_interface
        backend_interface._BACKENDS[BackendType.WEBGPU] = MagicMock()
        backend_interface._BACKENDS[BackendType.CPU] = MagicMock()
        # Create ResourceManager instance with mocked resources
        resource_mgr = ResourceManager()
        # Test backends detection
        backends = get_available_backends()
        self.assertIn(BackendType.CPU, backends)
        self.assertIn(BackendType.WEBGPU, backends)
        # Test best backend detection
        best = determine_best_backend()
        self.assertEqual(best, BackendType.WEBGPU)


if __name__ == '__main__':
    unittest.main() 