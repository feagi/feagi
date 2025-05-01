"""
Global pytest fixtures and configuration for FEAGI tests.
"""

import pytest
import os
import logging
from pathlib import Path
import numpy as np
from feagi.utils.config import FeagiConfig

# Configure logging for tests
logging.basicConfig(level=logging.WARNING)


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
    config = FeagiConfig()
    config.set('connectome.max_neurons', 100)
    config.set('connectome.max_synapses_per_neuron', 10)
    config.set('connectome.fcl_window_size', 3)
    return config


@pytest.fixture
def medium_config():
    """Create a medium-sized FeagiConfig for testing."""
    config = FeagiConfig()
    config.set('connectome.max_neurons', 1000)
    config.set('connectome.max_synapses_per_neuron', 100)
    return config


@pytest.fixture
def large_config():
    """Create a large FeagiConfig for performance testing."""
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