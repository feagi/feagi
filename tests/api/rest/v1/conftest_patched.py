"""
Lightweight conftest for API tests that avoids heavy dependencies.

This module provides a minimal test framework that:
1. Avoids actual imports of heavy dependencies like PyTorch and ConnectomeManager
2. Creates lightweight mocks for all critical components
3. Sets up client fixtures needed for testing different API groups
"""

import pytest
import logging
import os
import json
import tempfile
from typing import Dict, Any, Callable, Optional, List, Set, Union
from unittest.mock import MagicMock, patch
from fastapi.testclient import TestClient
from fastapi import FastAPI, Depends, HTTPException, status, Query, UploadFile, Response, File, Form

# Configure test logging
logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger(__name__)

# ======= MOCK CLASSES =======
# Create minimal mock classes to substitute for heavy imports

class MockConnectomeManager(MagicMock):
    """Lightweight mock of ConnectomeManager."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.max_neurons = 10_000_000
        self.max_synapses = 100_000_000
        self.cortical_areas = {
            "1": MagicMock(name="Area 1", dimensions=[10, 10, 1], position=[0, 0, 0], type="sensory", id="1"),
            "2": MagicMock(name="Area 2", dimensions=[5, 5, 1], position=[20, 0, 0], type="motor", id="2")
        }
        self.area_neuron_map = {
            "1": set(range(100)),
            "2": set(range(100, 150))
        }
        self.is_initialized = True
        self.mappings = {
            "mapping1": {
                "id": "mapping1",
                "source_id": "1",
                "target_id": "2",
                "type": "direct",
                "enabled": True
            }
        }
        
    def get_mapping(self, mapping_id):
        """Get a specific mapping."""
        return self.mappings.get(mapping_id)
    
    def create_mapping(self, source_id, target_id, mapping_type="direct", properties=None):
        """Create a new mapping."""
        mapping_id = f"mapping_{source_id}_{target_id}"
        self.mappings[mapping_id] = {
            "id": mapping_id,
            "source_id": source_id,
            "target_id": target_id,
            "type": mapping_type,
            "enabled": True,
            "properties": properties or {}
        }
        return self.mappings[mapping_id]
    
    def update_mapping(self, mapping_id, **kwargs):
        """Update a mapping."""
        if mapping_id in self.mappings:
            self.mappings[mapping_id].update(kwargs)
            return self.mappings[mapping_id]
        return None
    
    def delete_mapping(self, mapping_id):
        """Delete a mapping."""
        if mapping_id in self.mappings:
            del self.mappings[mapping_id]
            return True
        return False
    
    def get_all_mappings(self):
        """Get all mappings."""
        return list(self.mappings.values())
        
class MockCoreAPIService(MagicMock):
    """Lightweight mock of CoreAPIService."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.connectome = MockConnectomeManager()
        self.burst_engine_running = False
        self.burst_engine_paused = False
        self.current_genome = {
            "genome_id": "test_genome",
            "genome_title": "Test Genome",
            "genome_description": "Test genome for API testing",
            "cortical_areas": {
                "area1": {
                    "name": "Area 1",
                    "type": "sensory",
                    "coordinates": {"x": 0, "y": 0, "z": 0},
                    "dimensions": {"x": 10, "y": 10, "z": 1}
                },
                "area2": {
                    "name": "Area 2",
                    "type": "motor",
                    "coordinates": {"x": 20, "y": 0, "z": 0},
                    "dimensions": {"x": 5, "y": 5, "z": 1}
                }
            },
            "blueprint": {},
            "brain_regions": {}
        }
        self.amalgamation_history = {
            "202304050123_A": {
                "id": "202304050123_A",
                "timestamp": "2023-04-05T01:23:45Z",
                "changes": ["Added cortical area X", "Modified cortical area Y"]
            }
        }
        self.brain_state = {
            "running": False,
            "paused": False,
            "burst_counter": 0,
            "neuron_count": 1000,
            "synapse_count": 5000,
            "memory_usage": {
                "total_mb": 2048,
                "used_mb": 500,
                "available_mb": 1548
            }
        }
        
    # === Input Sources API Methods ===
    
    def get_input_sources(self):
        """Get all registered input sources."""
        return [
            {
                "id": "camera1",
                "name": "Front Camera",
                "type": "camera",
                "target_area_id": "101",
                "properties": {
                    "resolution": "640x480"
                }
            },
            {
                "id": "microphone1",
                "name": "Microphone",
                "type": "audio",
                "target_area_id": "102",
                "properties": {
                    "sample_rate": 44100
                }
            }
        ]
    
    def get_input_source(self, source_id):
        """Get a specific input source by ID."""
        sources = {s["id"]: s for s in self.get_input_sources()}
        return sources.get(source_id)
    
    def create_input_source(self, data):
        """Create a new input source."""
        return {
            "id": "new_source_1",
            "name": data["name"],
            "type": data["type"],
            "target_area_id": data.get("target_area_id"),
            "properties": data.get("properties", {})
        }
    
    def update_input_source(self, source_id, data):
        """Update an existing input source."""
        return {
            "id": source_id,
            "name": data.get("name", "Updated Source"),
            "type": data.get("type", "generic"),
            "target_area_id": data.get("target_area_id", "101"),
            "properties": data.get("properties", {})
        }
    
    def delete_input_source(self, source_id):
        """Delete an input source."""
        return True
    
    def send_input_data(self, source_id, data):
        """Send data from an input source."""
        return {
            "message": "Data received successfully",
            "neuron_count": len(data.get("values", [])),
            "timestamp": "2025-05-19T18:30:00.000Z"
        }
        
    # === Genome API Methods ===
    
    def get_genome(self):
        """Get the current genome."""
        return self.current_genome
    
    def get_cortical_area(self, area_id):
        """Get a cortical area from the genome."""
        return self.current_genome.get("cortical_areas", {}).get(area_id)
    
    def set_genome(self, genome):
        """Set the current genome."""
        self.current_genome = genome
        return {"message": "Genome set successfully"}
    
    def reset_genome(self):
        """Reset the current genome."""
        self.current_genome = {
            "genome_id": "reset_genome",
            "genome_title": "Reset Genome",
            "genome_description": "Reset genome for testing",
            "cortical_areas": {},
            "blueprint": {},
            "brain_regions": {}
        }
        return {"message": "Genome reset successfully"}
    
    def get_default_genomes(self):
        """Get list of available default genomes."""
        return {
            "files": [
                "barebones_genome.json",
                "essential_genome.json",
                "tutorial_genome.json"
            ]
        }
    
    def get_amalgamation_history(self):
        """Get the amalgamation history."""
        return {"amalgamations": self.amalgamation_history}
    
    def create_amalgamation(self, data):
        """Create a new amalgamation."""
        amalgamation_id = f"{data.get('genome_id', 'unknown')}_{len(self.amalgamation_history) + 1}"
        self.amalgamation_history[amalgamation_id] = {
            "id": amalgamation_id,
            "timestamp": "2025-06-01T12:00:00Z",
            "changes": [f"Added {len(data.get('cortical_areas', {}))} cortical areas"]
        }
        return {
            "amalgamation_id": amalgamation_id,
            "message": "Amalgamation created successfully"
        }
    
    def get_cortical_templates(self):
        """Get the available cortical templates."""
        return {
            "templates": [
                {
                    "id": "template1",
                    "name": "Test Template",
                    "description": "Test template for API testing",
                    "parameters": {
                        "size": {"default": 10, "min": 1, "max": 100},
                        "type": {"options": ["sensory", "association", "motor"]}
                    }
                }
            ]
        }
    
    def get_circuit_library(self):
        """Get the circuit library."""
        return {
            "circuits": [
                {
                    "id": "circuit1",
                    "name": "Test Circuit",
                    "description": "Test circuit for API testing",
                    "components": [
                        {"id": "comp1", "type": "neuron", "parameters": {}}
                    ]
                }
            ]
        }
    
    # === Brain State API Methods ===
    
    def get_brain_state(self):
        """Get the current brain state."""
        return self.brain_state
    
    def set_brain_state(self, state):
        """Update the brain state."""
        self.brain_state.update(state)
        return self.brain_state
    
    # === Burst Engine API Methods ===
    
    def start_burst_engine(self):
        """Start the burst engine."""
        self.burst_engine_running = True
        self.burst_engine_paused = False
        self.brain_state["running"] = True
        self.brain_state["paused"] = False
        return {"message": "Burst engine started"}
    
    def stop_burst_engine(self):
        """Stop the burst engine."""
        self.burst_engine_running = False
        self.burst_engine_paused = False
        self.brain_state["running"] = False
        self.brain_state["paused"] = False
        return {"message": "Burst engine stopped"}
    
    def pause_burst_engine(self):
        """Pause the burst engine."""
        if self.burst_engine_running:
            self.burst_engine_paused = True
            self.brain_state["paused"] = True
            return {"message": "Burst engine paused"}
        return {"error": "Burst engine not running"}
    
    def resume_burst_engine(self):
        """Resume the burst engine."""
        if self.burst_engine_running and self.burst_engine_paused:
            self.burst_engine_paused = False
            self.brain_state["paused"] = False
            return {"message": "Burst engine resumed"}
        return {"error": "Burst engine not paused"}
    
    def get_burst_engine_status(self):
        """Get the burst engine status."""
        return {
            "running": self.burst_engine_running,
            "paused": self.burst_engine_paused,
            "burst_counter": self.brain_state["burst_counter"]
        }

# ======= CREATE REST APP =======

def create_mock_app():
    """Create a minimal FastAPI app for testing."""
    # Create a minimal FastAPI app
    app = FastAPI()
    
    # Add dependency injectors for core API and connectome
    core_api = MockCoreAPIService()
    connectome = core_api.connectome
    
    # Create dependency functions
    def get_core_api():
        return core_api
        
    def get_connectome():
        return connectome
    
    # ===== API ENDPOINTS =====
    
    # === Input Sources API Routes ===
    
    @app.get("/v1/inputs/sources")
    def get_input_sources(_core_api=Depends(get_core_api)):
        """Get all registered input sources."""
        return {
            "sources": _core_api.get_input_sources()
        }

    @app.get("/v1/inputs/sources/{source_id}")
    def get_input_source(source_id: str, _core_api=Depends(get_core_api)):
        """Get a specific input source by ID."""
        source = _core_api.get_input_source(source_id)
        if not source:
            raise HTTPException(status_code=404, detail="Input source not found")
        return source

    @app.post("/v1/inputs/sources")
    def create_input_source(data: dict, _core_api=Depends(get_core_api)):
        """Create a new input source."""
        # Validate required fields
        if "name" not in data or "type" not in data:
            raise HTTPException(status_code=400, detail="Missing required fields")
        return _core_api.create_input_source(data)
        
    @app.put("/v1/inputs/sources/{source_id}")
    def update_input_source(source_id: str, data: dict, _core_api=Depends(get_core_api)):
        """Update an existing input source."""
        if not _core_api.get_input_source(source_id):
            raise HTTPException(status_code=404, detail="Input source not found")
        return _core_api.update_input_source(source_id, data)
        
    @app.delete("/v1/inputs/sources/{source_id}")
    def delete_input_source(source_id: str, _core_api=Depends(get_core_api)):
        """Delete an input source."""
        if not _core_api.get_input_source(source_id):
            raise HTTPException(status_code=404, detail="Input source not found")
        _core_api.delete_input_source(source_id)
        return {"message": f"Input source {source_id} deleted successfully"}
        
    @app.post("/v1/inputs/sources/{source_id}/data")
    def send_input_data(source_id: str, data: dict, _core_api=Depends(get_core_api)):
        """Send data from an input source."""
        if not _core_api.get_input_source(source_id):
            raise HTTPException(status_code=404, detail="Input source not found")
        
        # Validate that data contains the required fields
        if "values" not in data:
            raise HTTPException(status_code=400, detail="Missing 'values' field in data")
        
        return _core_api.send_input_data(source_id, data)
    
    # === Genome API Routes ===
    
    @app.get("/v1/genome/download")
    def download_genome(_core_api=Depends(get_core_api)):
        """Download the current genome."""
        return _core_api.get_genome()
    
    @app.get("/v1/genome/download/region/{region_id}")
    def download_genome_from_region(region_id: str, _core_api=Depends(get_core_api)):
        """Download a genome from a specific brain region."""
        # Mock implementation - return the main genome for any region
        return _core_api.get_genome()
    
    @app.post("/v1/genome/upload/barebones")
    def upload_barebones_genome(_core_api=Depends(get_core_api)):
        """Upload the barebones genome."""
        _core_api.reset_genome()
        return {"message": "Barebones genome loaded successfully"}
    
    @app.post("/v1/genome/upload/essential")
    def upload_essential_genome(_core_api=Depends(get_core_api)):
        """Upload the essential genome."""
        _core_api.reset_genome()
        return {"message": "Essential genome loaded successfully"}
    
    @app.post("/v1/genome/upload/file")
    async def upload_genome_file(file: UploadFile = File(...), _core_api=Depends(get_core_api)):
        """Upload a genome file."""
        try:
            content = await file.read()
            genome_data = json.loads(content)
            _core_api.set_genome(genome_data)
            return {"message": f"Genome loaded successfully from {file.filename}"}
        except json.JSONDecodeError:
            raise HTTPException(status_code=400, detail="Invalid JSON format")
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"Error loading genome: {str(e)}")
    
    @app.post("/v1/genome/upload/string")
    def upload_genome_string(data: dict, _core_api=Depends(get_core_api)):
        """Upload a genome as a string."""
        try:
            if "genome" not in data:
                raise HTTPException(status_code=400, detail="Missing 'genome' field in request")
                
            genome_data = json.loads(data["genome"])
            _core_api.set_genome(genome_data)
            return {"message": "Genome loaded successfully from string"}
        except json.JSONDecodeError:
            raise HTTPException(status_code=400, detail="Invalid JSON format")
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"Error loading genome: {str(e)}")
    
    @app.get("/v1/genome/default_files")
    def genome_default_files(_core_api=Depends(get_core_api)):
        """List available default genomes."""
        return _core_api.get_default_genomes()
    
    @app.get("/v1/genome/number")
    def get_genome_number(_core_api=Depends(get_core_api)):
        """Get the current genome number."""
        return {"genome_number": 42}  # Mock a genome number
    
    @app.post("/v1/genome/reset")
    def reset_genome(_core_api=Depends(get_core_api)):
        """Reset the genome."""
        return _core_api.reset_genome()
    
    @app.post("/v1/genome/amalgamation")
    def amalgamation_by_payload(data: dict, _core_api=Depends(get_core_api)):
        """Initiate an amalgamation by payload."""
        return _core_api.create_amalgamation(data)
    
    @app.get("/v1/genome/amalgamation/history")
    def amalgamation_history(_core_api=Depends(get_core_api)):
        """Get amalgamation history."""
        return _core_api.get_amalgamation_history()
    
    @app.get("/v1/genome/cortical_template")
    def cortical_template(_core_api=Depends(get_core_api)):
        """Get cortical templates."""
        return _core_api.get_cortical_templates()
    
    @app.get("/v1/genome/circuits")
    def get_circuit_library(_core_api=Depends(get_core_api)):
        """Get the circuit library."""
        return _core_api.get_circuit_library()
    
    # === Brain State API Routes ===
    
    @app.get("/v1/brain/state")
    def get_brain_state(_core_api=Depends(get_core_api)):
        """Get the current brain state."""
        return _core_api.get_brain_state()
    
    @app.post("/v1/brain/state")
    def set_brain_state(data: dict, _core_api=Depends(get_core_api)):
        """Update the brain state."""
        return _core_api.set_brain_state(data)
    
    # === Burst Engine API Routes ===
    
    @app.post("/v1/burst_engine/start")
    def start_burst_engine(_core_api=Depends(get_core_api)):
        """Start the burst engine."""
        return _core_api.start_burst_engine()
    
    @app.post("/v1/burst_engine/stop")
    def stop_burst_engine(_core_api=Depends(get_core_api)):
        """Stop the burst engine."""
        return _core_api.stop_burst_engine()
    
    @app.post("/v1/burst_engine/pause")
    def pause_burst_engine(_core_api=Depends(get_core_api)):
        """Pause the burst engine."""
        return _core_api.pause_burst_engine()
    
    @app.post("/v1/burst_engine/resume")
    def resume_burst_engine(_core_api=Depends(get_core_api)):
        """Resume the burst engine."""
        return _core_api.resume_burst_engine()
    
    @app.get("/v1/burst_engine/status")
    def get_burst_engine_status(_core_api=Depends(get_core_api)):
        """Get the burst engine status."""
        return _core_api.get_burst_engine_status()
    
    # === Cortical Mapping API Routes ===
    
    @app.get("/v1/mapping")
    def get_all_mappings(_connectome=Depends(get_connectome)):
        """Get all cortical mappings."""
        return {"mappings": _connectome.get_all_mappings()}
    
    @app.get("/v1/mapping/{mapping_id}")
    def get_mapping(mapping_id: str, _connectome=Depends(get_connectome)):
        """Get a specific mapping."""
        mapping = _connectome.get_mapping(mapping_id)
        if not mapping:
            raise HTTPException(status_code=404, detail="Mapping not found")
        return mapping
    
    @app.post("/v1/mapping")
    def create_mapping(data: dict, _connectome=Depends(get_connectome)):
        """Create a new mapping."""
        if "source_id" not in data or "target_id" not in data:
            raise HTTPException(status_code=400, detail="Missing required fields")
        
        mapping = _connectome.create_mapping(
            source_id=data["source_id"],
            target_id=data["target_id"],
            mapping_type=data.get("type", "direct"),
            properties=data.get("properties", {})
        )
        return mapping
    
    @app.put("/v1/mapping/{mapping_id}")
    def update_mapping(mapping_id: str, data: dict, _connectome=Depends(get_connectome)):
        """Update a mapping."""
        mapping = _connectome.get_mapping(mapping_id)
        if not mapping:
            raise HTTPException(status_code=404, detail="Mapping not found")
        
        # Update the mapping
        updated_mapping = _connectome.update_mapping(mapping_id, **data)
        return updated_mapping
    
    @app.delete("/v1/mapping/{mapping_id}")
    def delete_mapping(mapping_id: str, _connectome=Depends(get_connectome)):
        """Delete a mapping."""
        mapping = _connectome.get_mapping(mapping_id)
        if not mapping:
            raise HTTPException(status_code=404, detail="Mapping not found")
        
        _connectome.delete_mapping(mapping_id)
        return {"message": f"Mapping {mapping_id} deleted successfully"}
          
    return app

# ======= CLIENT FACTORY SYSTEM =======

# Store client instances by group to avoid recreating them
_client_cache: Dict[str, TestClient] = {}

@pytest.fixture(scope="session")
def client_factory():
    """Factory fixture to create TestClients with different configurations."""
    def _create_client(group_name: str = "default"):
        """Create a TestClient with custom configuration."""
        # Check cache first
        if group_name in _client_cache:
            return _client_cache[group_name]
        
        # Create the app
        app = create_mock_app()
        
        # Create and cache the client
        client = TestClient(app)
        _client_cache[group_name] = client
        return client
    
    return _create_client

@pytest.fixture
def client(client_factory):
    """Default client using standard configuration."""
    return client_factory("default")

# ======= SPECIALIZED CLIENTS FOR DIFFERENT TEST GROUPS =======

@pytest.fixture(scope="module")
def inputs_client(client_factory):
    """Client specialized for input sources tests."""
    return client_factory("inputs")

@pytest.fixture(scope="module")
def genome_client(client_factory):
    """Client specialized for genome tests."""
    return client_factory("genome")

@pytest.fixture(scope="module")
def brain_client(client_factory):
    """Client specialized for brain state tests."""
    return client_factory("brain_state")

@pytest.fixture(scope="module")
def burst_engine_client(client_factory):
    """Client specialized for burst engine tests."""
    return client_factory("burst_engine")

@pytest.fixture(scope="module")
def mapping_client(client_factory):
    """Client specialized for cortical mapping tests."""
    return client_factory("mapping")

@pytest.fixture(scope="module")
def region_client(client_factory):
    """Client specialized for region tests."""
    return client_factory("region")

@pytest.fixture(scope="module")
def system_client(client_factory):
    """Client specialized for system tests."""
    return client_factory("system")

@pytest.fixture(scope="module")
def simulation_client(client_factory):
    """Client specialized for simulation tests."""
    return client_factory("simulation")

@pytest.fixture(scope="module")
def insights_client(client_factory):
    """Client specialized for insights tests."""
    return client_factory("insights")

# ======= PYTEST CONFIGURATION HOOKS =======

def pytest_configure(config):
    """Register custom markers."""
    config.addinivalue_line("markers", "api: mark a test as an API test")
    config.addinivalue_line("markers", 
                           "api_group(name): mark a test as belonging to a specific API test group")

def pytest_collection_modifyitems(config, items):
    """Auto-classify tests into groups based on file names if not explicitly marked."""
    for item in items:
        # Add default classifications based on filename if not already classified
        if not any(mark.name == "api_group" for mark in item.iter_markers()):
            item_path = item.path.name
            
            if "inputs" in item_path:
                item.add_marker(pytest.mark.api_group("inputs"))
            elif "genome" in item_path:
                item.add_marker(pytest.mark.api_group("genome"))
            elif "brain" in item_path:
                item.add_marker(pytest.mark.api_group("brain_state"))
            elif "burst_engine" in item_path:
                item.add_marker(pytest.mark.api_group("burst_engine"))
            elif "cortical_mapping" in item_path:
                item.add_marker(pytest.mark.api_group("mapping"))
            elif "region" in item_path:
                item.add_marker(pytest.mark.api_group("region"))
            elif "system" in item_path:
                item.add_marker(pytest.mark.api_group("system"))
            elif "simulation" in item_path:
                item.add_marker(pytest.mark.api_group("simulation"))
            elif "insights" in item_path:
                item.add_marker(pytest.mark.api_group("insights"))
            else:
                # Default group
                item.add_marker(pytest.mark.api_group("default"))

@pytest.fixture(autouse=True)
def use_appropriate_client(request):
    """Automatically select the appropriate test client based on test group."""
    # Check if the test has an api_group marker
    group_marker = request.node.get_closest_marker("api_group")
    if group_marker:
        group_name = group_marker.args[0] if group_marker.args else "default"
        
        # Select the appropriate client fixture
        if group_name == "inputs":
            request.getfixturevalue("inputs_client")
        elif group_name == "genome":
            request.getfixturevalue("genome_client")
        elif group_name == "brain_state":
            request.getfixturevalue("brain_client")
        elif group_name == "burst_engine":
            request.getfixturevalue("burst_engine_client")
        elif group_name == "mapping":
            request.getfixturevalue("mapping_client")
        elif group_name == "region":
            request.getfixturevalue("region_client")
        elif group_name == "system":
            request.getfixturevalue("system_client")
        elif group_name == "simulation":
            request.getfixturevalue("simulation_client")
        elif group_name == "insights":
            request.getfixturevalue("insights_client") 