"""
Global pytest fixtures and configuration for FEAGI tests.
"""

import pytest
import os
import logging
from pathlib import Path
import numpy as np
import sys
import importlib.util
from unittest.mock import MagicMock
from types import ModuleType

# Configure logging for tests
logging.basicConfig(level=logging.WARNING)

# Mock modules that are causing import issues
MOCK_MODULES = ['wgpu', 'wgpu._coreutils']
for mod_name in MOCK_MODULES:
    sys.modules[mod_name] = MagicMock(spec=ModuleType(mod_name))

if 'wgpu._coreutils' in sys.modules:
    # Create a mock WGPULogger class
    class MockWGPULogger:
        def __init__(self):
            pass
        
    # Set up the logger as an instance of WGPULogger
    sys.modules['wgpu._coreutils'].WGPULogger = MockWGPULogger
    sys.modules['wgpu._coreutils'].logger = MockWGPULogger()

# Add ZMQ test utilities for handling ZMQ mocking
# This keeps all ZMQ related mocking in one place
try:
    # Try to import from the dedicated ZMQ test utils
    from tests.api.zmq.zmq_test_utils import ZMQ_AVAILABLE, setup_mocks
    # Set up ZMQ mocks if needed
    setup_mocks()
except ImportError:
    # If we can't import, create minimal mocks here
    ZMQ_AVAILABLE = importlib.util.find_spec("zmq") is not None
    if not ZMQ_AVAILABLE:
        # Create basic mock zmq module
        zmq_module = ModuleType("zmq")
        sys.modules['zmq'] = zmq_module
        
        # Add essential constants
        zmq_module.REQ = 3
        zmq_module.REP = 4
        zmq_module.PUB = 1
        zmq_module.SUB = 2

def pytest_ignore_collect(collection_path, config):
    """
    Hook to ignore test collection for modules with issues.
    
    This function allows for skipping collections of specific test modules based on
    availability of dependencies or to avoid duplicate test collection.
    
    Args:
        collection_path: Path to the file being collected
        config: Pytest configuration object
        
    Returns:
        bool: True if the file should be ignored, False otherwise
    """
    # Skip modules with duplicate test names to avoid collection errors
    path_str = str(collection_path)
    
    # Skip duplicate test files
    if "api/rest/v2/test_genome.py" in path_str and os.path.exists("tests/api/rest/v1/test_genome.py"):
        return True
    
    # Defer to the ZMQ utility function if it exists
    try:
        from tests.api.zmq.zmq_test_utils import pytest_ignore_collect as zmq_ignore_collect
        return zmq_ignore_collect(collection_path, config)
    except ImportError:
        # If we can't import the ZMQ utilities, use basic check
        if not ZMQ_AVAILABLE:
            # Skip tests in ZMQ-related directories
            for rel_path in ["api/zmq", "api/protocols", "api/grpc", "api/websocket"]:
                if rel_path in path_str:
                    return True
        
        return False

@pytest.fixture(scope="session")
def project_root():
    """Return the project root directory."""
    return Path(__file__).parent.parent


@pytest.fixture(scope="session")
def test_data_dir(project_root):
    """Return the test data directory."""
    data_dir = project_root / "tests" / "data"
    data_dir.mkdir(exist_ok=True)
    return data_dir


@pytest.fixture
def minimal_config():
    """Create a minimal FeagiConfig for testing."""
    from feagi.utils.config import FeagiConfig
    config = FeagiConfig()
    config.set('connectome.max_neurons', 100)
    config.set('connectome.max_synapses_per_neuron', 10)
    config.set('connectome.fcl_window_size', 3)
    return config


@pytest.fixture
def medium_config():
    """Create a medium-sized FeagiConfig for testing."""
    from feagi.utils.config import FeagiConfig
    config = FeagiConfig()
    config.set('connectome.max_neurons', 1000)
    config.set('connectome.max_synapses_per_neuron', 100)
    return config


@pytest.fixture
def large_config():
    """Create a large FeagiConfig for performance testing."""
    from feagi.utils.config import FeagiConfig
    config = FeagiConfig()
    config.set('connectome.max_neurons', 100000)
    config.set('connectome.max_synapses_per_neuron', 1000)
    return config


@pytest.fixture(autouse=True)
def set_test_environment():
    """Set environment variables for testing."""
    os.environ["FEAGI_TESTING"] = "true"
    os.environ.setdefault("FEAGI_LOG_LEVEL", "WARNING")
    
    # Get backend setting from environment or use CPU as default for tests
    backend = os.environ.get("FEAGI_BACKEND", "cpu")
    os.environ["FEAGI_BACKEND"] = backend
    
    yield
    
    # Clean up environment after tests
    if "FEAGI_TESTING" in os.environ:
        del os.environ["FEAGI_TESTING"]


@pytest.fixture
def random_seed():
    """Set a fixed random seed for reproducible tests."""
    seed = 42
    np.random.seed(seed)
    return seed


@pytest.fixture
def temp_genome_path(tmp_path):
    """Create a temporary path for genome files."""
    genome_dir = tmp_path / "genomes"
    genome_dir.mkdir()
    return genome_dir


@pytest.fixture
def skip_if_no_gpu():
    """Skip a test if no GPU is available."""
    try:
        import torch
        if not torch.cuda.is_available():
            pytest.skip("No GPU available")
    except ImportError:
        pytest.skip("PyTorch not installed, cannot check for GPU") 