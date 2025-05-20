"""
Fixtures and configuration for FEAGI REST API tests.

This module provides fixtures for testing the FEAGI REST API,
including mocks for the connectome manager, core API service, etc.
"""

import logging
import sys
import pytest
import os
import json
import tempfile
from typing import Dict, Any, Callable, Optional, Set, List, Tuple
from unittest.mock import MagicMock, patch, create_autospec

# Create comprehensive mock for ZMQ to allow tests to run without circular dependencies
def setup_zmq_mocks():
    """Set up comprehensive ZMQ mocks for testing."""
    # Create core ZMQ module structure
    mock_zmq = MagicMock()
    
    # Add common ZMQ constants
    mock_zmq.POLLIN = 1
    mock_zmq.POLLOUT = 2
    mock_zmq.DEALER = 5
    mock_zmq.ROUTER = 6
    mock_zmq.PUSH = 8
    mock_zmq.PULL = 9
    mock_zmq.EAGAIN = 35
    mock_zmq.LINGER = 17
    mock_zmq.RCVTIMEO = 27
    mock_zmq.SNDHWM = 23
    
    # Create ZMQ auth module
    mock_auth_thread = MagicMock()
    mock_auth_thread.ThreadAuthenticator = MagicMock()
    mock_auth = MagicMock()
    mock_auth.thread = mock_auth_thread
    
    # Create ZMQ asyncio module
    mock_asyncio = MagicMock()
    mock_asyncio.Context = MagicMock()
    mock_asyncio.Socket = MagicMock()
    mock_asyncio.Poller = MagicMock()
    
    # Populate the auth and asyncio modules
    mock_zmq.auth = mock_auth
    mock_zmq.asyncio = mock_asyncio
    
    # Create error class
    class ZMQError(Exception):
        def __init__(self, errno):
            self.errno = errno
            super().__init__(f"ZMQ Error {errno}")
    mock_zmq.ZMQError = ZMQError
    
    # Set up in sys.modules
    sys.modules['zmq'] = mock_zmq
    sys.modules['zmq.auth'] = mock_auth
    sys.modules['zmq.auth.thread'] = mock_auth_thread
    sys.modules['zmq.asyncio'] = mock_asyncio
    
    # Mock feagi_connector to prevent circular dependencies
    sys.modules['feagi_connector'] = MagicMock()
    sys.modules['feagi_connector.zmq'] = MagicMock()
    sys.modules['feagi_connector.zmq.client'] = MagicMock()
    sys.modules['feagi_connector.zmq.rest_client'] = MagicMock()
    sys.modules['feagi_connector.zmq_rest_client'] = MagicMock()

# Apply mocks before other imports
setup_zmq_mocks()

# Async mock for async methods
class AsyncMock:
    """Mock for async methods."""
    
    def __init__(self, return_value=None):
        self.return_value = return_value
        
    def __call__(self, *args, **kwargs):
        return self
        
    def __await__(self):
        async def _await_me():
            return self.return_value
        return _await_me().__await__()

# Import needed components including create_rest_app before it's needed
from fastapi.testclient import TestClient
from feagi.api.rest.app import create_rest_app
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.api.rest.dependencies import get_core_api, get_connectome, set_connectome_instance, set_core_api_service

# Configure test logging
logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger(__name__)

# ======= SHARED MOCK CONFIGURATIONS =======
# These can be imported and extended by test modules

DEFAULT_BURST_ENGINE_CONFIG = {
    "burst_duration": 10.0,
    "inter_burst_interval": 5.0,
    "maximum_firing_rate": 100.0,
    "refractory_period": 5.0,
    "threshold": 0.5,
    "decay_rate": 0.1,
    "firing_threshold": 0.7,
    "membrane_potential_decay": 0.05
}

DEFAULT_BURST_ENGINE_STATS = {
    "average_burst_time": 8.5,
    "max_burst_time": 12.3,
    "min_burst_time": 7.1,
    "total_bursts": 1000, 
    "average_active_neurons": 500,
    "memory_usage": 128.5
}

DEFAULT_HEALTH_CHECK = {
    "burst_engine": True, 
    "connected_agents": None,
    "influxdb_availability": False,
    "neuron_count_max": 0,
    "synapse_count_max": 0, 
    "latest_changes_saved_externally": False,
    "genome_availability": False,
    "genome_validity": None,
    "brain_readiness": None
}

MOCK_CORTICAL_AREAS = [
    {"id": "1", "name": "Test Area 1", "dimensions": [10, 10, 1], "type": "sensory"},
    {"id": "2", "name": "Test Area 2", "dimensions": [5, 5, 1], "type": "motor"}
]

MOCK_BRAIN_REGIONS = {
    "root": {"id": "root", "name": "Root Region", "parent": None, "children": ["region1", "region2"]},
    "region1": {"id": "region1", "name": "Region 1", "parent": "root", "children": []},
    "region2": {"id": "region2", "name": "Region 2", "parent": "root", "children": []}
}

MOCK_GENOME = {
    "id": "test-genome",
    "title": "Test Genome",
    "version": "1.0",
    "cortical_areas": {
        "area1": {
            "name": "Area 1",
            "type": "sensory",
            "dimensions": [10, 10, 1],
            "position": [0, 0, 0]
        },
        "area2": {
            "name": "Area 2",
            "type": "motor",
            "dimensions": [5, 5, 1],
            "position": [20, 0, 0]
        }
    },
    "blueprint": {},
    "brain_regions": {"test_region": {}}
}

# ======= CORE MOCKING FUNCTIONS =======

@pytest.fixture(scope="session")
def fcl_manager_mock():
    """Create a fully mocked FCL Manager."""
    mock = MagicMock()
    
    # Add common methods
    mock.get_window_size.return_value = 20
    mock.set_window_size.return_value = True
    mock.get_data.return_value = {"timestamps": [], "neurons": []}
    mock.clear.return_value = True
    
    return mock

@pytest.fixture(scope="session")
def connectome_manager_mock(fcl_manager_mock):
    """Create a fully mocked ConnectomeManager to avoid expensive initialization."""
    # Create a comprehensive mock - using simple MagicMock instead of create_autospec to avoid issues
    mock_cm = MagicMock()
    
    # Add the most commonly used attributes and methods
    mock_cm.max_neurons = 10_000_000
    mock_cm.max_synapses = 100_000_000
    mock_cm.cortical_areas = {
        "1": MagicMock(name="Area 1", dimensions=[10, 10, 1], position=[0, 0, 0], type="sensory", id="1"),
        "2": MagicMock(name="Area 2", dimensions=[5, 5, 1], position=[20, 0, 0], type="motor", id="2")
    }
    mock_cm.area_neuron_map = {
        "1": set(range(100)),
        "2": set(range(100, 150))
    }
    mock_cm.brain_regions = MOCK_BRAIN_REGIONS
    mock_cm.neurons = {i: {"membrane_potential": 0.0, "position": (i%10, i//10, 0)} for i in range(150)}
    mock_cm.get_cortical_areas.return_value = list(mock_cm.cortical_areas.values())
    mock_cm.get_neurons_by_area.side_effect = lambda area_id: mock_cm.area_neuron_map.get(area_id, set())
    mock_cm.get_brain_state.return_value = {
        "neurons": 150,
        "synapses": 1000,
        "timestep": 0
    }
    mock_cm.get_synapse_count.return_value = 1000
    mock_cm.fcl_manager = fcl_manager_mock
    mock_cm.is_initialized = True
    
    # Advanced functionality for brain activity tests
    mock_cm.get_activity.return_value = {
        "timestep": 100,
        "active_neurons": [1, 2, 3, 4, 5],
        "areas": {"1": {"active_count": 5}}
    }
    mock_cm.get_area_activity.return_value = {
        "area_id": "1",
        "active_neurons": [1, 2, 3, 4, 5],
        "average_activity": 0.5
    }
    
    return mock_cm

@pytest.fixture(scope="session")
def core_api_mock(connectome_manager_mock):
    """Create a fully mocked CoreAPIService with comprehensive behaviors."""
    # Create a comprehensive mock - using simple MagicMock instead of create_autospec
    mock = MagicMock()
    
    # Set default return values for commonly used methods
    mock.get_burst_engine_config.return_value = DEFAULT_BURST_ENGINE_CONFIG
    mock.get_burst_engine_stats.return_value = DEFAULT_BURST_ENGINE_STATS
    mock.health_check.return_value = DEFAULT_HEALTH_CHECK
    mock.get_cortical_areas.return_value = MOCK_CORTICAL_AREAS
    mock.get_brain_state.return_value = {
        "neurons": 150,
        "synapses": 1000,
        "timestep": 100,
        "active_neurons": 25
    }
    mock.get_genome.return_value = MOCK_GENOME
    mock.get_connectome_manager.return_value = connectome_manager_mock
    mock.list_brain_regions.return_value = {"regions": MOCK_BRAIN_REGIONS}
    
    # Add specific behaviors for update methods
    mock.update_burst_engine_config.side_effect = lambda params: {
        **DEFAULT_BURST_ENGINE_CONFIG,
        **params
    }
    
    # Add behaviors for brain stimulation
    mock.stimulate_neurons.return_value = {
        "success": True,
        "stimulated": 5
    }
    
    # Add behaviors for cortical area creation
    mock.create_cortical_area.side_effect = lambda name, area_type, dimensions, position: {
        "id": "new_area",
        "name": name,
        "type": area_type,
        "dimensions": dimensions,
        "position": position
    }
    
    # Genome API specific behaviors
    mock.get_data_path.return_value = "/tmp/feagi_test"
    mock.get_temp_path.return_value = "/tmp/feagi_test"
    mock.load_genome.return_value = True
    mock.get_genome_filename.return_value = "test_genome.json"
    mock.get_genome_counter.return_value = 1
    mock.reset_genome.return_value = True
    mock.has_pending_amalgamation.return_value = False
    mock.initiate_amalgamation.return_value = True
    mock.initiate_amalgamation_by_filename.return_value = True
    mock.get_amalgamation_history.return_value = {"202304050123_A": "completed"}
    mock.get_cortical_templates.return_value = {"templates": [{"name": "Test Template"}]}
    
    # Mapping API specific behaviors
    mock.get_mapping_stats.return_value = {
        "source_id": "1",
        "target_id": "2",
        "synapse_count": 1000,
        "average_weight": 0.7,
        "connectivity_ratio": 0.8,
        "mapping_type": "one-to-one"
    }
    
    # Add more default behaviors for common endpoints
    mock.get_activity.return_value = {
        "timestep": 100,
        "active_neurons": [1, 2, 3, 4, 5],
        "areas": {"1": {"active_count": 5}}
    }
    
    # In the core_api_mock fixture, add this method to support essential genome upload
    mock.load_genome.return_value = True
    mock.get_burst_engine.return_value = MagicMock()
    # Add a specific method to process the essential genome
    mock.process_essential_genome = MagicMock(return_value={"success": True, "data": {"genome_id": "essential"}, "timestamp": 1234567890})
    
    return mock

# ======= CLIENT FACTORY SYSTEM =======

# Store client instances by group to avoid recreating them
_client_cache: Dict[str, TestClient] = {}

@pytest.fixture(scope="session")
def client_factory(core_api_mock, connectome_manager_mock):
    """Factory fixture to create TestClients with different configurations.
    
    This allows test modules to create customized clients while reusing
    the expensive initialization parts.
    """
    def _create_client(
        group_name: str = "default",
        core_api_customizer: Optional[Callable[[MagicMock], None]] = None,
        connectome_customizer: Optional[Callable[[MagicMock], None]] = None,
        app_overrides: Optional[Dict[Callable, Callable]] = None
    ) -> TestClient:
        """Create a TestClient with custom configuration.
        
        Args:
            group_name: Name of the test group to cache the client under
            core_api_customizer: Function to customize the core API mock
            connectome_customizer: Function to customize the connectome mock
            app_overrides: Additional dependency overrides for the app
            
        Returns:
            A configured TestClient
        """
        # Check cache first
        if group_name in _client_cache:
            return _client_cache[group_name]
        
        # Clone the mocks to prevent cross-contamination between test groups
        mock_core_api = MagicMock()
        for key, val in core_api_mock.__dict__.items():
            if not key.startswith('_'):
                setattr(mock_core_api, key, val)
                
        mock_connectome = MagicMock()
        for key, val in connectome_manager_mock.__dict__.items():
            if not key.startswith('_'):
                setattr(mock_connectome, key, val)
        
        # Apply customizations if provided
        if core_api_customizer:
            core_api_customizer(mock_core_api)
            
        if connectome_customizer:
            connectome_customizer(mock_connectome)
        
        # Create app with deep mocking to prevent actual initialization
        with patch('feagi.api.rest.app.CoreAPIService', return_value=mock_core_api):
            with patch('feagi.api.rest.app.ConnectomeManager', return_value=mock_connectome):
                test_app = create_rest_app(connectome=mock_connectome)
        
        # Set up dependency overrides
        overrides = {
            get_core_api: lambda: mock_core_api,
            get_connectome: lambda: mock_connectome
        }
        
        # Add any additional overrides
        if app_overrides:
            overrides.update(app_overrides)
            
        test_app.dependency_overrides.update(overrides)
        
        # Create and cache the client
        client = TestClient(test_app)
        _client_cache[group_name] = client
        return client
    
    return _create_client

@pytest.fixture
def client(client_factory):
    """Default client using standard configuration."""
    return client_factory("default")

# ======= SPECIALIZED CLIENTS FOR DIFFERENT TEST GROUPS =======

@pytest.fixture(scope="module")
def burst_engine_client(client_factory):
    """Client specialized for burst engine tests."""
    def _customize_core_api(mock):
        # Add any burst-engine specific customizations
        mock.get_burst_engine_config.return_value = {
            **DEFAULT_BURST_ENGINE_CONFIG,
            "burst_duration": 15.0,  # Different from default for testing
            "inter_burst_interval": 7.5
        }
        mock.get_burst_engine_stats.return_value = {
            **DEFAULT_BURST_ENGINE_STATS,
            "total_bursts": 2000,  # Different from default for testing
            "average_active_neurons": 750
        }
        mock.update_burst_engine_config.side_effect = lambda params: {
            **mock.get_burst_engine_config.return_value,
            **params
        }
    
    return client_factory("burst_engine", core_api_customizer=_customize_core_api)

@pytest.fixture(scope="module")
def genome_client(client_factory):
    """Client specialized for genome tests."""
    def _customize_core_api(mock):
        # Add genome-specific responses
        mock.get_genome.return_value = {
            **MOCK_GENOME,
            "genome_title": "Extended Test Genome",  # Changed for this test group
            "cortical_areas": {
                "area1": {"name": "Area 1", "type": "sensory", "dimensions": [20, 20, 2]},
                "area2": {"name": "Area 2", "type": "motor", "dimensions": [10, 10, 2]},
                "area3": {"name": "Area 3", "type": "associative", "dimensions": [15, 15, 2]}
            }
        }
        # Create a temporary directory for test files
        temp_dir = tempfile.mkdtemp(prefix='feagi_test_')
        genome_dir = os.path.join(temp_dir, 'defaults', 'genome')
        os.makedirs(genome_dir, exist_ok=True)
        
        # Configure mock return values
        mock.get_data_path.return_value = temp_dir
        mock.get_temp_path.return_value = temp_dir
    
    return client_factory("genome", core_api_customizer=_customize_core_api)

@pytest.fixture(scope="module")
def brain_state_client(client_factory):
    """Client specialized for brain state tests."""
    def _customize_connectome(mock):
        # Add brain state specifics
        mock.get_brain_state.return_value = {
            "neurons": 250,
            "synapses": 2500,
            "timestep": 100,
            "active_neurons": 25
        }
        mock.get_activity.return_value = {
            "timestep": 100,
            "active_neurons": [1, 2, 3, 4, 5],
            "areas": {
                "1": {"active_count": 5, "average_activity": 0.5},
                "2": {"active_count": 3, "average_activity": 0.3}
            }
        }
    
    def _customize_core_api(mock):
        # Add brain state specific behaviors
        mock.get_brain_state.return_value = {
            "neurons": 250, 
            "synapses": 2500,
            "timestep": 100,
            "active_neurons": 25
        }
        mock.stimulate_neurons.return_value = {
            "success": True,
            "stimulated": 5,
            "message": "Neurons successfully stimulated"
        }
    
    return client_factory("brain_state", 
                          core_api_customizer=_customize_core_api,
                          connectome_customizer=_customize_connectome)

@pytest.fixture(scope="module")
def cortical_area_client(client_factory):
    """Client specialized for cortical area tests."""
    def _customize_core_api(mock):
        # Add cortical area specifics
        mock.get_cortical_areas.return_value = [
            {"id": "1", "name": "Visual Cortex", "dimensions": [20, 20, 5], "type": "sensory"},
            {"id": "2", "name": "Motor Cortex", "dimensions": [10, 10, 5], "type": "motor"},
            {"id": "3", "name": "Associative Cortex", "dimensions": [15, 15, 5], "type": "associative"}
        ]
        mock.create_cortical_area.side_effect = lambda name, area_type, dimensions, position: {
            "id": f"new_{name.lower().replace(' ', '_')}",
            "name": name,
            "type": area_type,
            "dimensions": dimensions,
            "position": position,
            "parameters": {}
        }
    
    return client_factory("cortical_area", core_api_customizer=_customize_core_api)

@pytest.fixture(scope="module")
def region_client(client_factory):
    """Client specialized for brain region tests."""
    def _customize_core_api(mock):
        # Add region specifics
        mock.list_brain_regions.return_value = {
            "regions": {
                **MOCK_BRAIN_REGIONS,
                "region3": {"id": "region3", "name": "Region 3", "parent": "root", "children": ["region4"]},
                "region4": {"id": "region4", "name": "Region 4", "parent": "region3", "children": []}
            }
        }
    
    return client_factory("region", core_api_customizer=_customize_core_api)

@pytest.fixture(scope="module")
def mapping_client(client_factory):
    """Client specialized for cortical mapping tests."""
    def _customize_core_api(mock):
        # Set up mock data for cortical areas
        mock.get_cortical_areas.return_value = [
            {
                "id": "101",
                "name": "Visual Cortex",
                "type": "sensory",
                "dimensions": {"width": 20, "height": 20, "depth": 5},
                "coordinates": {"x": 0, "y": 0, "z": 0},
                "parameters": {},
                "neuron_count": 2000
            },
            {
                "id": "102",
                "name": "Motor Cortex",
                "type": "motor",
                "dimensions": {"width": 10, "height": 10, "depth": 5},
                "coordinates": {"x": 30, "y": 0, "z": 0},
                "parameters": {},
                "neuron_count": 500
            }
        ]
        
        # Set up mock data for genome
        mock.get_genome.return_value = {
            **MOCK_GENOME,
            "connectivity": {
                "1": {
                    "source_id": "101",
                    "target_id": "102",
                    "mapping_type": "one-to-one",
                    "weight_multiplier": 1.0,
                    "connection_probability": 0.8
                },
                "2": {
                    "source_id": "102",
                    "target_id": "101",
                    "mapping_type": "probabilistic",
                    "weight_multiplier": 0.5,
                    "connection_probability": 0.3
                }
            }
        }
        
        # Set up mock data for other endpoints
        mock.get_genome_filename.return_value = "test_genome.json"
        mock.get_mapping_stats.return_value = {
            "source_id": "101",
            "target_id": "102",
            "synapse_count": 1000,
            "average_weight": 0.7,
            "connectivity_ratio": 0.8,
            "mapping_type": "one-to-one"
        }
        mock.apply_mapping.return_value = {
            "message": "Mapping applied successfully",
            "source_id": "101",
            "target_id": "102",
            "mapping_type": "one-to-one",
            "connections_created": 1000
        }
        mock.get_mapping_templates.return_value = {
            "templates": [
                {
                    "id": "one-to-one",
                    "name": "One-to-One",
                    "description": "Direct one-to-one mapping between areas",
                    "parameters": {}
                },
                {
                    "id": "gaussian",
                    "name": "Gaussian",
                    "description": "Gaussian probability distribution",
                    "parameters": {
                        "sigma": 1.5,
                        "max_distance": 5
                    }
                }
            ]
        }
    
    return client_factory("mapping", core_api_customizer=_customize_core_api)

# ======= PYTEST CONFIGURATION HOOKS =======

def pytest_configure(config):
    """Register custom markers."""
    config.addinivalue_line("markers", "api: mark a test as an API test")
    config.addinivalue_line("markers", 
                           "api_group(name): mark a test as belonging to a specific API test group")
    config.addinivalue_line("markers",
                           "long_running: mark a test as being long-running")

def pytest_collection_modifyitems(config, items):
    """
    Auto-classify tests into groups based on file names if not explicitly marked.
    
    This allows test modules to be organized properly without requiring explicit markers
    on every test.
    """
    for item in items:
        # Add default classifications based on filename if not already classified
        if not any(mark.name == "api_group" for mark in item.iter_markers()):
            item_path = item.path.name
            
            if "burst_engine" in item_path:
                item.add_marker(pytest.mark.api_group("burst_engine"))
            elif "genome" in item_path:
                item.add_marker(pytest.mark.api_group("genome"))
            elif "brain" in item_path:
                item.add_marker(pytest.mark.api_group("brain_state"))
            elif "cortical" in item_path or "mapping" in item_path:
                item.add_marker(pytest.mark.api_group("mapping"))
            elif "region" in item_path:
                item.add_marker(pytest.mark.api_group("region"))
            else:
                # Default group
                item.add_marker(pytest.mark.api_group("default"))

@pytest.fixture(autouse=True)
def use_appropriate_client(request):
    """
    Automatically select the appropriate test client based on test group.
    
    This fixture runs for every test and determines which client fixture to use.
    """
    # Check if the test has an api_group marker
    group_marker = request.node.get_closest_marker("api_group")
    if group_marker:
        group_name = group_marker.args[0] if group_marker.args else "default"
        
        # Select the appropriate client fixture
        if group_name == "burst_engine":
            request.getfixturevalue("burst_engine_client")
        elif group_name == "genome":
            request.getfixturevalue("genome_client")
        elif group_name == "brain_state":
            request.getfixturevalue("brain_state_client")
        elif group_name == "mapping":
            request.getfixturevalue("mapping_client")
        elif group_name == "region":
            request.getfixturevalue("region_client")
        elif group_name == "cortical_area":
            request.getfixturevalue("cortical_area_client")

# Client fixtures already defined earlier 