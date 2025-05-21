"""Tests for the CoreAPIService class."""

import pytest
from unittest.mock import MagicMock, patch
import numpy as np

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.bdu.connectome_manager import ConnectomeManager


@pytest.fixture
def connectome_manager():
    """Create a mock ConnectomeManager for testing."""
    cm = MagicMock(spec=ConnectomeManager)
    
    # Set up basic mocks for cortical area methods
    cm.cortical_areas = {
        1: MagicMock(name="Area 1", position=(0, 0, 0), dimensions=(10, 10, 10), type="sensory", cortical_id="area1"),
        2: MagicMock(name="Area 2", position=(20, 0, 0), dimensions=(10, 10, 10), type="motor", cortical_id="area2")
    }
    cm.cortical_neuron_map = {
        1: {1, 2, 3},
        2: {4, 5, 6}
    }
    
    # Mock neurons
    cm.neurons = {
        1: {"cortical_id": "area1", "position": (1, 1, 1), "membrane_potential": 0.5, "threshold": 1.0},
        2: {"cortical_id": "area1", "position": (2, 2, 2), "membrane_potential": 0.3, "threshold": 1.0},
        3: {"cortical_id": "area1", "position": (3, 3, 3), "membrane_potential": 0.8, "threshold": 1.0},
        4: {"cortical_id": "area2", "position": (1, 1, 1), "membrane_potential": 0.1, "threshold": 1.0},
        5: {"cortical_id": "area2", "position": (2, 2, 2), "membrane_potential": 0.9, "threshold": 1.0},
        6: {"cortical_id": "area2", "position": (3, 3, 3), "membrane_potential": 0.4, "threshold": 1.0},
    }
    
    # Mock methods
    cm.get_neurons_by_area = lambda cortical_id: list(cm.cortical_neuron_map.get(cortical_id, []))
    cm.get_neuron_property = lambda neuron_id, property_name: cm.neurons[neuron_id].get(property_name)
    cm.set_neuron_property = lambda neuron_id, property_name, value: cm.neurons[neuron_id].update({property_name: value})
    
    # Mock for FCL manager
    fcl_manager = MagicMock()
    cm.fcl_manager = fcl_manager
    
    return cm


@pytest.fixture
def core_api_service(connectome_manager):
    """Create a CoreAPIService instance with a mock ConnectomeManager."""
    # Create CoreAPIService
    service = CoreAPIService(connectome_manager)
    return service


def test_get_cortical_areas(core_api_service, connectome_manager):
    """Test getting all cortical areas."""
    # Call the method
    result = core_api_service.get_cortical_areas()
    
    # Verify the result is a list
    assert isinstance(result, list)
    
    # Verify the correct number of areas
    assert len(result) == 2  # We mocked 2 areas


def test_get_cortical_area(core_api_service, connectome_manager):
    """Test getting a specific cortical area by ID."""
    # Get cortical area 1
    result = core_api_service.get_cortical_area("1")
    
    # Verify the result is a dictionary
    assert isinstance(result, dict)
    
    # Verify the area ID
    assert result["id"] == "1"


def test_reset_fcl(core_api_service, connectome_manager):
    """Test resetting the FCL."""
    # Call the method
    result = core_api_service.reset_fcl()
    
    # Verify the result
    assert result is True
    
    # Verify the FCL manager's reset method was called
    assert connectome_manager.fcl_manager.reset.called


def test_get_burst_engine_config(core_api_service, connectome_manager):
    """Test getting the burst engine configuration."""
    # Call the method
    result = core_api_service.get_burst_engine_config()
    
    # Verify the result is a dictionary
    assert isinstance(result, dict) 