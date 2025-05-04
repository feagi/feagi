"""Configuration for REST API tests."""

import sys
import logging
import pytest
from unittest.mock import MagicMock

# Configure logging for tests
logging.basicConfig(level=logging.WARNING)

# Mock modules that are causing import issues
MOCK_MODULES = ['wgpu', 'wgpu._coreutils']
for mod_name in MOCK_MODULES:
    sys.modules[mod_name] = MagicMock()

# Create a mock WGPULogger class
class MockWGPULogger:
    def __init__(self):
        pass
    
# Set up the logger as an instance of WGPULogger
if 'wgpu._coreutils' in sys.modules:
    sys.modules['wgpu._coreutils'].WGPULogger = MockWGPULogger
    sys.modules['wgpu._coreutils'].logger = MockWGPULogger()

# Mock the FEAGI class for testing
class MockFEAGI:
    def __init__(self):
        self.connectome_manager = MagicMock()
        self.connectome_manager.get_cortical_areas.return_value = [
            {"id": "1", "name": "Test Area 1"},
            {"id": "2", "name": "Test Area 2"}
        ]
    
    def get_cortical_areas(self):
        """Mock implementation of get_cortical_areas."""
        return self.connectome_manager.get_cortical_areas()
    
    def get_brain_state(self):
        """Mock implementation of get_brain_state."""
        return {"status": "initialized"}
        
    def get_configuration(self):
        """Mock implementation of get_configuration."""
        return {"api_version": "1.0.0"}
        
    def get_simulation_status(self):
        """Mock implementation of get_simulation_status."""
        return {"running": False, "current_burst": 0}
    
    def save_brain_state(self, path):
        """Mock implementation of save_brain_state."""
        return True
        
    def load_brain_state(self, path):
        """Mock implementation of load_brain_state."""
        return True
        
    def start_simulation(self):
        """Mock implementation of start_simulation."""
        return True
        
    def stop_simulation(self):
        """Mock implementation of stop_simulation."""
        return True
        
    def update_configuration(self, config):
        """Mock implementation of update_configuration."""
        return True
        
    def get_burst_engine_config(self):
        """Mock implementation of get_burst_engine_config."""
        return {
            "burst_duration": 1.0,
            "inter_burst_interval": 1.0,
            "maximum_firing_rate": 1.0,
            "refractory_period": 2.0,
            "threshold": 0.5
        }
        
    def update_burst_engine_config(self, config):
        """Mock implementation of update_burst_engine_config."""
        # Return True to indicate success
        return True
        
    def get_burst_engine_stats(self):
        """Mock implementation of get_burst_engine_stats."""
        return {
            "average_burst_time": 0.5,
            "average_processing_time": 8.5,
            "burst_duration": 10.0,
            "current_burst": 0,
            "inter_burst_interval": 5.0,
            "last_burst_time": 0.1,
            "max_burst_time": 1.0,
            "min_burst_time": 0.01
        }
        
    def get_input_sources(self):
        """Mock implementation of get_input_sources."""
        return [
            {
                "id": "camera1",
                "name": "Front Camera",
                "type": "camera",
                "target_area_id": "1",
                "properties": {
                    "resolution": "640x480"
                }
            },
            {
                "id": "microphone1",
                "name": "Microphone",
                "type": "audio",
                "target_area_id": "2",
                "properties": {
                    "channels": 2,
                    "sample_rate": 44100
                }
            }
        ]
        
    def get_input_source(self, source_id):
        """Mock implementation of get_input_source."""
        if source_id == "camera1":
            return {
                "id": "camera1",
                "name": "Front Camera",
                "type": "camera",
                "target_area_id": "1",
                "properties": {
                    "resolution": "640x480"
                }
            }
        return None

# Replace the FEAGI import if needed
if 'feagi.core.feagi' in sys.modules:
    sys.modules['feagi.core.feagi'].FEAGI = MockFEAGI

@pytest.fixture
def mock_core_api():
    """Create a mock CoreAPIService with all the necessary methods."""
    mock = MagicMock()
    
    # Configuration
    mock.get_configuration.return_value = {"api_version": "1.0.0"}
    mock.update_configuration.return_value = True
    
    # Brain state
    mock.get_brain_state.return_value = {"status": "initialized"}
    mock.save_brain_state.return_value = True
    mock.load_brain_state.return_value = True
    
    # Simulation
    mock.get_simulation_status.return_value = {"running": False, "current_burst": 0}
    mock.start_simulation.return_value = True
    mock.stop_simulation.return_value = True
    
    # Burst engine
    mock.get_burst_engine_config.return_value = {
        "burst_duration": 1.0,
        "inter_burst_interval": 1.0,
        "maximum_firing_rate": 1.0,
        "refractory_period": 2.0,
        "threshold": 0.5
    }
    mock.update_burst_engine_config.return_value = True
    mock.get_burst_engine_stats.return_value = {
        "average_burst_time": 0.5,
        "average_processing_time": 8.5,
        "burst_duration": 10.0,
        "current_burst": 0,
        "inter_burst_interval": 5.0,
        "last_burst_time": 0.1,
        "max_burst_time": 1.0,
        "min_burst_time": 0.01
    }
    
    # Cortical areas
    mock.get_cortical_areas.return_value = [
        {"id": "1", "name": "Test Area 1"},
        {"id": "2", "name": "Test Area 2"}
    ]
    
    # Inputs
    mock.get_input_sources.return_value = [
        {
            "id": "camera1",
            "name": "Front Camera",
            "type": "camera",
            "target_area_id": "1",
            "properties": {
                "resolution": "640x480"
            }
        },
        {
            "id": "microphone1",
            "name": "Microphone",
            "type": "audio",
            "target_area_id": "2",
            "properties": {
                "channels": 2,
                "sample_rate": 44100
            }
        }
    ]
    
    def mock_get_input_source(source_id):
        if source_id == "camera1":
            return {
                "id": "camera1",
                "name": "Front Camera",
                "type": "camera",
                "target_area_id": "1",
                "properties": {
                    "resolution": "640x480"
                }
            }
        return None
    
    mock.get_input_source.side_effect = mock_get_input_source
    mock.create_input_source.return_value = {"id": "new-source", "name": "New Source"}
    mock.update_input_source.return_value = True
    mock.delete_input_source.return_value = True
    
    # Insights
    mock.get_activity_summary.return_value = {"active_areas": 2, "active_neurons": 100}
    mock.get_activity_heatmap.return_value = {"data": []}
    mock.get_neuron_activity.return_value = {"data": []}
    mock.get_network_analytics.return_value = {"neuron_count": 1000, "synapse_count": 10000}
    
    return mock

@pytest.fixture
def client():
    """Create a test client for the REST API."""
    from fastapi.testclient import TestClient
    from feagi.api.rest.app import app
    
    # Patch the dependency for testing
    from feagi.api.rest.app import get_core_api
    app.dependency_overrides[get_core_api] = lambda: mock_core_api()
    
    return TestClient(app) 