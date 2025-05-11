"""
Unit tests for the backend interface and backend selection.
"""

import pytest
import numpy as np

from feagi.core.backend.interface import (
    BackendType,
    BackendInterface,
    BackendCapability,
    get_available_backends,
    get_backend,
    determine_best_backend,
    register_backend,
)
from feagi.core.backend.cpu import CPUBackend


class TestBackendTypes:
    """Tests for backend type enumeration."""
    
    def test_backend_type_values(self):
        """Test that BackendType enum has expected values."""
        assert BackendType.CPU.value == "cpu"
        assert BackendType.WEBGPU.value == "webgpu"
        assert BackendType.CUDA.value == "cuda"
        assert BackendType.METAL.value == "metal"
        assert BackendType.AUTO.value == "auto"


class TestBackendCapabilities:
    """Tests for backend capability enumeration."""
    
    def test_backend_capability_values(self):
        """Test that BackendCapability enum has expected values."""
        assert BackendCapability.MATRIX_MULTIPLICATION.value == "matrix_multiplication"
        assert BackendCapability.CONVOLUTION.value == "convolution"
        assert BackendCapability.ELEMENT_WISE_OPERATIONS.value == "element_wise_operations"
        assert BackendCapability.BITMAP_OPERATIONS.value == "bitmap_operations"
        assert BackendCapability.RANDOM_NUMBER_GENERATION.value == "random_number_generation"
        assert BackendCapability.SPARSE_OPERATIONS.value == "sparse_operations"


class TestBackendRegistry:
    """Tests for backend registration and discovery."""
    
    def test_cpu_backend_registration(self):
        """Test that CPU backend is registered correctly."""
        # CPU backend should already be registered
        backends = get_available_backends()
        assert BackendType.CPU in backends
    
    def test_register_backend(self, monkeypatch):
        """Test registering a custom backend."""
        # Create a dummy backend class
        class DummyBackend:
            pass
        
        # Patch the _BACKENDS dictionary
        monkeypatch.setattr("feagi.core.backend.interface._BACKENDS", {})
        
        # Register it
        register_backend(BackendType.WEBGPU, DummyBackend)
        
        # Verify it was registered
        from feagi.core.backend.interface import _BACKENDS
        assert BackendType.WEBGPU in _BACKENDS
        assert _BACKENDS[BackendType.WEBGPU] == DummyBackend


class TestBackendSelection:
    """Tests for backend selection logic."""
    
    def test_determine_best_backend_cuda(self, monkeypatch):
        """Test that CUDA is preferred when available."""
        # Create a mock CUDA backend class
        class MockCUDABackend:
            pass
            
        # Mock the _BACKENDS dictionary to include CUDA
        monkeypatch.setattr(
            "feagi.core.backend.interface._BACKENDS",
            {BackendType.CPU: None, BackendType.CUDA: MockCUDABackend, BackendType.METAL: None}
        )
        
        monkeypatch.setattr(
            "feagi.core.backend.interface.get_available_backends",
            lambda: [BackendType.CPU, BackendType.CUDA, BackendType.METAL]
        )
        
        # Mock the ResourceManager's resources to indicate CUDA is available
        mock_resources = {
            "gpu_available": True,
            "gpu_count": 1,
            "webgpu_available": False,
            "metal_available": False
        }
        
        class MockResourceManager:
            @classmethod
            def get_instance(cls):
                return cls()
                
            @property
            def resources(self):
                return mock_resources
        
        monkeypatch.setattr(
            "feagi.core.resource_mgr.ResourceManager",
            MockResourceManager
        )
        
        assert determine_best_backend() == BackendType.CUDA
    
    def test_determine_best_backend_metal(self, monkeypatch):
        """Test that Metal is preferred when CUDA is not available."""
        # Create mock backend classes
        class MockMetalBackend:
            pass
            
        # Mock the _BACKENDS dictionary to include Metal but not CUDA
        monkeypatch.setattr(
            "feagi.core.backend.interface._BACKENDS",
            {BackendType.CPU: None, BackendType.METAL: MockMetalBackend}
        )
        
        monkeypatch.setattr(
            "feagi.core.backend.interface.get_available_backends",
            lambda: [BackendType.CPU, BackendType.METAL]
        )
        
        # Mock the ResourceManager's resources to indicate Metal is available but not CUDA
        mock_resources = {
            "gpu_available": False,
            "gpu_count": 0,
            "webgpu_available": False,
            "metal_available": True
        }
        
        class MockResourceManager:
            @classmethod
            def get_instance(cls):
                return cls()
                
            @property
            def resources(self):
                return mock_resources
        
        monkeypatch.setattr(
            "feagi.core.resource_mgr.ResourceManager",
            MockResourceManager
        )
        
        assert determine_best_backend() == BackendType.METAL
    
    def test_determine_best_backend_webgpu(self, monkeypatch):
        """Test that WebGPU is preferred when CUDA and Metal are not available."""
        # Create mock backend classes
        class MockWebGPUBackend:
            pass
            
        # Mock the _BACKENDS dictionary to include WebGPU but not CUDA or Metal
        monkeypatch.setattr(
            "feagi.core.backend.interface._BACKENDS",
            {BackendType.CPU: None, BackendType.WEBGPU: MockWebGPUBackend}
        )
        
        monkeypatch.setattr(
            "feagi.core.backend.interface.get_available_backends",
            lambda: [BackendType.CPU, BackendType.WEBGPU]
        )
        
        # Mock the ResourceManager's resources to indicate WebGPU is available but not CUDA or Metal
        mock_resources = {
            "gpu_available": False,
            "gpu_count": 0,
            "webgpu_available": True,
            "metal_available": False
        }
        
        class MockResourceManager:
            @classmethod
            def get_instance(cls):
                return cls()
                
            @property
            def resources(self):
                return mock_resources
        
        monkeypatch.setattr(
            "feagi.core.resource_mgr.ResourceManager",
            MockResourceManager
        )
        
        assert determine_best_backend() == BackendType.WEBGPU
    
    def test_determine_best_backend_cpu(self, monkeypatch):
        """Test that CPU is used when no other backends are available."""
        # Create mock backend classes
        class MockCPUBackend:
            pass
            
        # Mock the _BACKENDS dictionary to include only CPU
        monkeypatch.setattr(
            "feagi.core.backend.interface._BACKENDS",
            {BackendType.CPU: MockCPUBackend}
        )
        
        monkeypatch.setattr(
            "feagi.core.backend.interface.get_available_backends",
            lambda: [BackendType.CPU]
        )
        
        # Mock the ResourceManager's resources to indicate no GPU acceleration is available
        mock_resources = {
            "gpu_available": False,
            "gpu_count": 0,
            "webgpu_available": False,
            "metal_available": False
        }
        
        class MockResourceManager:
            @classmethod
            def get_instance(cls):
                return cls()
                
            @property
            def resources(self):
                return mock_resources
        
        monkeypatch.setattr(
            "feagi.core.resource_mgr.ResourceManager",
            MockResourceManager
        )
        
        assert determine_best_backend() == BackendType.CPU


class TestCPUBackend:
    """Tests for the CPU backend implementation."""
    
    def test_cpu_backend_initialization(self):
        """Test that the CPU backend initializes properly."""
        backend = CPUBackend()
        assert backend.initialize() is True
        assert backend.is_initialized is True
    
    def test_cpu_backend_capabilities(self):
        """Test that the CPU backend supports expected capabilities."""
        backend = CPUBackend()
        backend.initialize()
        
        assert backend.supports_capability(BackendCapability.MATRIX_MULTIPLICATION) is True
        assert backend.supports_capability(BackendCapability.ELEMENT_WISE_OPERATIONS) is True
        assert backend.supports_capability(BackendCapability.BITMAP_OPERATIONS) is True
        assert backend.supports_capability(BackendCapability.RANDOM_NUMBER_GENERATION) is True
    
    def test_create_tensor(self):
        """Test creating a tensor with the CPU backend."""
        backend = CPUBackend()
        backend.initialize()
        
        # Create an empty tensor
        tensor = backend.create_tensor((2, 3))
        assert isinstance(tensor, np.ndarray)
        assert tensor.shape == (2, 3)
        assert tensor.dtype == np.float32
        assert np.all(tensor == 0)
        
        # Create a tensor with data
        data = np.ones((2, 3))
        tensor = backend.create_tensor((2, 3), data=data)
        assert np.array_equal(tensor, data)
    
    def test_to_numpy_from_numpy(self):
        """Test converting to and from NumPy arrays."""
        backend = CPUBackend()
        backend.initialize()
        
        # These methods should be no-ops for the CPU backend
        arr = np.array([[1, 2], [3, 4]])
        
        tensor = backend.from_numpy(arr)
        assert np.array_equal(tensor, arr)
        
        arr2 = backend.to_numpy(tensor)
        assert np.array_equal(arr2, arr)


# Create a pytest fixture for the config object
@pytest.fixture
def mock_config():
    """Fixture for a mock config object."""
    class MockConfig:
        def get(self, key, default=None):
            return default
    return MockConfig()


class TestBackendInstantiation:
    """Tests for backend instantiation."""
    
    @pytest.fixture(autouse=True)
    def setup(self, monkeypatch):
        """Setup for backend instantiation tests."""
        # Clear the backend instances before each test
        monkeypatch.setattr("feagi.core.backend.interface._BACKEND_INSTANCES", {})
        # Ensure the backend registry is correct and not polluted by previous tests
        from feagi.core.backend.cpu import CPUBackend
        from feagi.core.backend.interface import BackendType
        monkeypatch.setattr("feagi.core.backend.interface._BACKENDS", {BackendType.CPU: CPUBackend})
    
    def test_get_backend_explicit(self):
        """Test getting a backend with explicit backend type."""
        backend = get_backend(BackendType.CPU)
        assert isinstance(backend, CPUBackend)
        assert backend.is_initialized is True
    
    def test_get_backend_auto(self, monkeypatch, mock_config):
        """Test getting a backend with auto selection."""
        monkeypatch.setattr("feagi.core.backend.interface.config", mock_config)
        monkeypatch.setattr(
            "feagi.core.backend.interface.determine_best_backend",
            lambda: BackendType.CPU
        )
        
        backend = get_backend()
        assert isinstance(backend, CPUBackend) 