"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
Fixtures and configuration for FEAGI REST API tests.

This module provides fixtures for testing the FEAGI REST API,
including mocks for the connectome manager, core API service, etc.
"""

import logging
import os
import tempfile
from typing import Callable, Dict, Optional
from unittest.mock import MagicMock, patch

import pytest
from fastapi import FastAPI

# Mock main FEAGI objects for REST API testing
from fastapi.testclient import TestClient

# Import from pytest root conftest - no need to recreate ZMQ mocks here
# ZMQ mocks are already set up in tests/conftest.py


# Get mocks from main conftest if needed and reuse them
# The main conftest already sets up:
# - sys.modules['zmq'] and related modules
# - sys.modules['feagi_connector'] and related modules


@pytest.fixture
def mock_connectome_manager():
    """Create a mock ConnectomeManager for REST API tests."""
    mock_cm = MagicMock()

    # Set up basic cortical areas data
    mock_cm.get_all_cortical_areas.return_value = [
        {
            "id": "visual_cortex",
            "name": "Visual Cortex",
            "type": "sensory",
            "dimensions": [10, 10, 1],
            "position": [0, 0, 0],
            "parameters": {"growth_rate": 0.01},
        },
        {
            "id": "motor_cortex",
            "name": "Motor Cortex",
            "type": "motor",
            "dimensions": [10, 10, 1],
            "position": [20, 0, 0],
            "parameters": {"growth_rate": 0.01},
        },
    ]

    # Set up mappings data
    mock_cm.get_all_cortical_mappings.return_value = [
        {
            "source_id": "visual_cortex",
            "destination_id": "motor_cortex",
            "mapping_type": "one_to_one",
            "parameters": {"synaptic_weight": 0.5},
        }
    ]

    # Add detailed area mock implementations
    def get_cortical_area(area_id: str):
        areas = {item["id"]: item for item in mock_cm.get_all_cortical_areas()}
        return areas.get(area_id)

    mock_cm.get_cortical_area.side_effect = get_cortical_area

    # Add genome mocks
    mock_cm.get_genome_template.return_value = {"version": "2.0", "areas": []}

    # Ensure fcl_manager exists
    mock_cm.fcl_manager = MagicMock()

    return mock_cm


@pytest.fixture
def mock_core_api_service(mock_connectome_manager):
    """Create a mock CoreAPIService for REST API tests."""
    mock_service = MagicMock()

    # Set up connectome-related methods
    mock_service.get_cortical_areas.return_value = (
        mock_connectome_manager.get_all_cortical_areas()
    )
    mock_service.get_cortical_area.side_effect = (
        mock_connectome_manager.get_cortical_area
    )
    mock_service.get_cortical_mappings.return_value = (
        mock_connectome_manager.get_all_cortical_mappings()
    )

    # Set up genome-related methods
    mock_service.get_genome_template.return_value = (
        mock_connectome_manager.get_genome_template()
    )
    mock_service.load_genome.return_value = True

    # Set up system config methods
    mock_service.get_configuration.return_value = {
        "burst_engine": {"rate": 60},
        "learning": {"enabled": True, "rate": 0.01},
    }

    # Set up state methods
    mock_service.get_state.return_value = {"status": "running"}
    mock_service.genome_is_loaded.return_value = True

    return mock_service


@pytest.fixture
def test_app(mock_core_api_service, mock_connectome_manager):
    """Create a test FastAPI application with mocked dependencies."""
    # Import the universal wrapper functions directly instead of old routers
    from feagi.api.transport.universal_fastapi import (
        get_burst_engine_router,
        get_connectome_router,
        get_cortical_area_router,
        get_cortical_mapping_router,
        get_evolution_router,
        get_feagi_agent_router,
        get_genome_router,
        get_inputs_router,
        get_insights_router,
        get_monitoring_router,
        get_morphology_router,
        get_network_router,
        get_neuroplasticity_router,
        get_outputs_router,
        get_region_router,
        get_simulation_router,
        get_system_router,
        get_training_router,
    )

    # Create FastAPI app
    app = FastAPI(title="FEAGI API Test")

    # Patch dependencies to use mocks
    with patch("feagi.api.rest.dependencies.get_connectome") as mock_get_connectome:
        mock_get_connectome.return_value = mock_connectome_manager

        with patch(
            "feagi.api.rest.dependencies._connectome_instance", mock_connectome_manager
        ):
            # Include all v1 routers using the universal wrapper
            app.include_router(
                get_system_router(), prefix="/v1/system", tags=["SYSTEM"]
            )
            app.include_router(
                get_genome_router(), prefix="/v1/genome", tags=["GENOME"]
            )
            app.include_router(
                get_cortical_area_router(),
                prefix="/v1/cortical_area",
                tags=["CORTICAL AREAS"],
            )
            app.include_router(
                get_connectome_router(), prefix="/v1/connectome", tags=["CONNECTOME"]
            )
            app.include_router(
                get_burst_engine_router(),
                prefix="/v1/burst_engine",
                tags=["BURST ENGINE"],
            )
            app.include_router(
                get_neuroplasticity_router(),
                prefix="/v1/neuroplasticity",
                tags=["NEUROPLASTICITY"],
            )
            app.include_router(
                get_region_router(), prefix="/v1/region", tags=["BRAIN REGIONS"]
            )
            app.include_router(
                get_morphology_router(),
                prefix="/v1/morphology",
                tags=["NEURON MORPHOLOGIES"],
            )
            app.include_router(
                get_monitoring_router(), prefix="/v1/monitoring", tags=["MONITORING"]
            )
            app.include_router(
                get_simulation_router(), prefix="/v1/simulation", tags=["SIMULATION"]
            )
            app.include_router(
                get_feagi_agent_router(), prefix="/v1/agent", tags=["FEAGI AGENT"]
            )
            app.include_router(
                get_insights_router(), prefix="/v1/insight", tags=["INSIGHTS"]
            )
            app.include_router(
                get_training_router(), prefix="/v1/training", tags=["TRAINING"]
            )
            app.include_router(
                get_cortical_mapping_router(),
                prefix="/v1/cortical_mapping",
                tags=["CORTICAL MAPPINGS"],
            )
            app.include_router(
                get_network_router(), prefix="/v1/network", tags=["NETWORK"]
            )
            app.include_router(
                get_inputs_router(), prefix="/v1/input", tags=["INPUT MANAGEMENT"]
            )
            app.include_router(
                get_outputs_router(), prefix="/v1/output", tags=["OUTPUT MANAGEMENT"]
            )
            app.include_router(
                get_evolution_router(), prefix="/v1/evolution", tags=["EVOLUTIONARY"]
            )

            # Create and return TestClient
            client = TestClient(app)
            yield client


@pytest.fixture
def temp_directory():
    """Create a temporary directory for file operations during tests."""
    with tempfile.TemporaryDirectory() as temp_dir:
        yield temp_dir


# Async mock for async methods
async def async_mock(*args, **kwargs):
    return MagicMock()


# Import needed components including create_rest_app before it's needed
from feagi.api.rest.app import create_rest_app
from feagi.api.rest.dependencies import get_connectome, get_core_api

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
    "membrane_potential_decay": 0.05,
}

DEFAULT_BURST_ENGINE_STATS = {
    "average_burst_time": 8.5,
    "max_burst_time": 12.3,
    "min_burst_time": 7.1,
    "total_bursts": 1000,
    "average_active_neurons": 500,
    "memory_usage": 128.5,
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
    "brain_readiness": None,
}

MOCK_CORTICAL_AREAS = [
    {"id": "1", "name": "Test Area 1", "dimensions": [10, 10, 1], "type": "sensory"},
    {"id": "2", "name": "Test Area 2", "dimensions": [5, 5, 1], "type": "motor"},
]

MOCK_BRAIN_REGIONS = {
    "root": {
        "id": "root",
        "name": "Root Region",
        "parent": None,
        "children": ["region1", "region2"],
    },
    "region1": {"id": "region1", "name": "Region 1", "parent": "root", "children": []},
    "region2": {"id": "region2", "name": "Region 2", "parent": "root", "children": []},
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
            "position": [0, 0, 0],
        },
        "area2": {
            "name": "Area 2",
            "type": "motor",
            "dimensions": [5, 5, 1],
            "position": [20, 0, 0],
        },
    },
    "blueprint": {},
    "brain_regions": {"test_region": {}},
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
        "1": MagicMock(
            name="Area 1",
            dimensions=[10, 10, 1],
            position=[0, 0, 0],
            type="sensory",
            id="1",
        ),
        "2": MagicMock(
            name="Area 2",
            dimensions=[5, 5, 1],
            position=[20, 0, 0],
            type="motor",
            id="2",
        ),
    }
    mock_cm.area_neuron_map = {"1": set(range(100)), "2": set(range(100, 150))}
    mock_cm.brain_regions = MOCK_BRAIN_REGIONS
    mock_cm.neurons = {
        i: {"membrane_potential": 0.0, "position": (i % 10, i // 10, 0)}
        for i in range(150)
    }
    mock_cm.get_cortical_areas.return_value = list(mock_cm.cortical_areas.values())
    mock_cm.get_neurons_by_area.side_effect = (
        lambda area_id: mock_cm.area_neuron_map.get(area_id, set())
    )
    mock_cm.get_brain_state.return_value = {
        "neurons": 150,
        "synapses": 1000,
        "timestep": 0,
    }
    mock_cm.get_synapse_count.return_value = 1000
    mock_cm.fcl_manager = fcl_manager_mock
    mock_cm.is_initialized = True

    # Advanced functionality for brain activity tests
    mock_cm.get_activity.return_value = {
        "timestep": 100,
        "active_neurons": [1, 2, 3, 4, 5],
        "areas": {"1": {"active_count": 5}},
    }
    mock_cm.get_area_activity.return_value = {
        "area_id": "1",
        "active_neurons": [1, 2, 3, 4, 5],
        "average_activity": 0.5,
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
        "active_neurons": 25,
    }
    mock.get_genome.return_value = MOCK_GENOME
    mock.get_connectome_manager.return_value = connectome_manager_mock
    mock.list_brain_regions.return_value = {"regions": MOCK_BRAIN_REGIONS}

    # Add specific behaviors for update methods
    mock.update_burst_engine_config.side_effect = lambda params: {
        **DEFAULT_BURST_ENGINE_CONFIG,
        **params,
    }

    # Add behaviors for brain stimulation
    mock.stimulate_neurons.return_value = {"success": True, "stimulated": 5}

    # Add behaviors for cortical area creation
    mock.create_cortical_area.side_effect = (
        lambda name, area_type, dimensions, position: {
            "id": "new_area",
            "name": name,
            "type": area_type,
            "dimensions": dimensions,
            "position": position,
        }
    )

    # Genome API specific behaviors
    mock.get_data_path.return_value = os.path.join(tempfile.gettempdir(), "feagi_test")
    mock.get_temp_path.return_value = os.path.join(tempfile.gettempdir(), "feagi_test")
    mock.load_genome.return_value = {
        "success": True,
        "genome_id": "test_genome",
        "loaded_at": "2024-01-01T00:00:00",
    }
    mock.get_genome_filename.return_value = "test_genome.json"
    mock.get_genome_counter.return_value = 1
    mock.reset_genome.return_value = True
    mock.has_pending_amalgamation.return_value = False
    mock.initiate_amalgamation.return_value = True
    mock.initiate_amalgamation_by_filename.return_value = True
    mock.get_amalgamation_history.return_value = {"202304050123_A": "completed"}
    mock.get_cortical_templates.return_value = {
        "templates": [{"name": "Test Template"}]
    }

    # Mapping API specific behaviors
    mock.get_mapping_stats.return_value = {
        "source_id": "1",
        "target_id": "2",
        "synapse_count": 1000,
        "average_weight": 0.7,
        "connectivity_ratio": 0.8,
        "mapping_type": "one-to-one",
    }

    # Add more default behaviors for common endpoints
    mock.get_activity.return_value = {
        "timestep": 100,
        "active_neurons": [1, 2, 3, 4, 5],
        "areas": {"1": {"active_count": 5}},
    }

    # In the core_api_mock fixture, add this method to support essential genome upload
    mock.get_burst_engine.return_value = MagicMock()
    # Add a specific method to process the essential genome
    mock.process_essential_genome = MagicMock(
        return_value={
            "success": True,
            "data": {"genome_id": "essential"},
            "timestamp": 1234567890,
        }
    )

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
        app_overrides: Optional[Dict[Callable, Callable]] = None,
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
            if not key.startswith("_"):
                setattr(mock_core_api, key, val)

        mock_connectome = MagicMock()
        for key, val in connectome_manager_mock.__dict__.items():
            if not key.startswith("_"):
                setattr(mock_connectome, key, val)

        # Apply customizations if provided
        if core_api_customizer:
            core_api_customizer(mock_core_api)

        if connectome_customizer:
            connectome_customizer(mock_connectome)

        # Create app with deep mocking to prevent actual initialization
        with patch("feagi.api.rest.app.CoreAPIService", return_value=mock_core_api):
            with patch(
                "feagi.api.rest.app.ConnectomeManager", return_value=mock_connectome
            ):
                test_app = create_rest_app(connectome=mock_connectome)

        # Set up dependency overrides
        overrides = {
            get_core_api: lambda: mock_core_api,
            get_connectome: lambda: mock_connectome,
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
            "inter_burst_interval": 7.5,
        }
        mock.get_burst_engine_stats.return_value = {
            **DEFAULT_BURST_ENGINE_STATS,
            "total_bursts": 2000,  # Different from default for testing
            "average_active_neurons": 750,
        }
        mock.update_burst_engine_config.side_effect = lambda params: {
            **mock.get_burst_engine_config.return_value,
            **params,
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
                "area1": {
                    "name": "Area 1",
                    "type": "sensory",
                    "dimensions": [20, 20, 2],
                },
                "area2": {"name": "Area 2", "type": "motor", "dimensions": [10, 10, 2]},
                "area3": {
                    "name": "Area 3",
                    "type": "associative",
                    "dimensions": [15, 15, 2],
                },
            },
        }
        # Create a temporary directory for test files
        temp_dir = tempfile.mkdtemp(prefix="feagi_test_")
        genome_dir = os.path.join(temp_dir, "defaults", "genome")
        os.makedirs(genome_dir, exist_ok=True)

        # Configure mock return values
        mock.get_data_path.return_value = temp_dir
        mock.get_temp_path.return_value = temp_dir

        # Add load_genome method that returns a proper dictionary
        mock.load_genome.return_value = {
            "success": True,
            "genome_id": "test_genome",
            "loaded_at": "2024-01-01T00:00:00",
        }
        mock.get_genome_counter.return_value = 1
        mock.get_burst_engine.return_value = MagicMock()

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
            "active_neurons": 25,
        }
        mock.get_activity.return_value = {
            "timestep": 100,
            "active_neurons": [1, 2, 3, 4, 5],
            "areas": {
                "1": {"active_count": 5, "average_activity": 0.5},
                "2": {"active_count": 3, "average_activity": 0.3},
            },
        }

    def _customize_core_api(mock):
        # Add brain state specific behaviors
        mock.get_brain_state.return_value = {
            "neurons": 250,
            "synapses": 2500,
            "timestep": 100,
            "active_neurons": 25,
        }
        mock.stimulate_neurons.return_value = {
            "success": True,
            "stimulated": 5,
            "message": "Neurons successfully stimulated",
        }

    return client_factory(
        "brain_state",
        core_api_customizer=_customize_core_api,
        connectome_customizer=_customize_connectome,
    )


@pytest.fixture(scope="module")
def cortical_area_client(client_factory):
    """Client specialized for cortical area tests."""

    def _customize_core_api(mock):
        # Add cortical area specifics
        mock.get_cortical_areas.return_value = [
            {
                "id": "1",
                "name": "Visual Cortex",
                "dimensions": [20, 20, 5],
                "type": "sensory",
            },
            {
                "id": "2",
                "name": "Motor Cortex",
                "dimensions": [10, 10, 5],
                "type": "motor",
            },
            {
                "id": "3",
                "name": "Associative Cortex",
                "dimensions": [15, 15, 5],
                "type": "associative",
            },
        ]
        mock.create_cortical_area.side_effect = (
            lambda name, area_type, dimensions, position: {
                "id": f"new_{name.lower().replace(' ', '_')}",
                "name": name,
                "type": area_type,
                "dimensions": dimensions,
                "position": position,
                "parameters": {},
            }
        )

    return client_factory("cortical_area", core_api_customizer=_customize_core_api)


@pytest.fixture(scope="module")
def region_client(client_factory):
    """Client specialized for brain region tests."""

    def _customize_core_api(mock):
        # Add region specifics
        mock.list_brain_regions.return_value = {
            "regions": {
                **MOCK_BRAIN_REGIONS,
                "region3": {
                    "id": "region3",
                    "name": "Region 3",
                    "parent": "root",
                    "children": ["region4"],
                },
                "region4": {
                    "id": "region4",
                    "name": "Region 4",
                    "parent": "region3",
                    "children": [],
                },
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
                "neuron_count": 2000,
            },
            {
                "id": "102",
                "name": "Motor Cortex",
                "type": "motor",
                "dimensions": {"width": 10, "height": 10, "depth": 5},
                "coordinates": {"x": 30, "y": 0, "z": 0},
                "parameters": {},
                "neuron_count": 500,
            },
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
                    "connection_probability": 0.8,
                },
                "2": {
                    "source_id": "102",
                    "target_id": "101",
                    "mapping_type": "probabilistic",
                    "weight_multiplier": 0.5,
                    "connection_probability": 0.3,
                },
            },
        }

        # Set up mock data for other endpoints
        mock.get_genome_filename.return_value = "test_genome.json"
        mock.get_mapping_stats.return_value = {
            "source_id": "101",
            "target_id": "102",
            "synapse_count": 1000,
            "average_weight": 0.7,
            "connectivity_ratio": 0.8,
            "mapping_type": "one-to-one",
        }
        mock.apply_mapping.return_value = {
            "message": "Mapping applied successfully",
            "source_id": "101",
            "target_id": "102",
            "mapping_type": "one-to-one",
            "connections_created": 1000,
        }
        mock.get_mapping_templates.return_value = {
            "templates": [
                {
                    "id": "one-to-one",
                    "name": "One-to-One",
                    "description": "Direct one-to-one mapping between areas",
                    "parameters": {},
                },
                {
                    "id": "gaussian",
                    "name": "Gaussian",
                    "description": "Gaussian probability distribution",
                    "parameters": {"sigma": 1.5, "max_distance": 5},
                },
            ]
        }

    return client_factory("mapping", core_api_customizer=_customize_core_api)


# ======= PYTEST CONFIGURATION HOOKS =======


def pytest_configure(config):
    """Register custom markers."""
    config.addinivalue_line("markers", "api: mark a test as an API test")
    config.addinivalue_line(
        "markers",
        "api_group(name): mark a test as belonging to a specific API test group",
    )
    config.addinivalue_line(
        "markers", "long_running: mark a test as being long-running"
    )


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
