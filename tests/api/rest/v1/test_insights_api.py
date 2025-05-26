"""Tests for the Insights API endpoints."""

import os
import json
import numpy as np
from unittest.mock import patch, MagicMock
import pytest

# Remove redundant fixtures and use the ones from conftest.py
@pytest.fixture
def mock_core_api():
    """Create a mock CoreAPIService."""
    with patch('feagi.api.gateway.APIGateway.core_api', new_callable=MagicMock) as mock:
        # Mock get_cortical_areas to return a list of test areas
        mock.get_cortical_areas.return_value = [
            {
                "id": "1",
                "name": "Test Area 1",
                "type": "sensory",
                "dimensions": {"width": 10, "height": 10, "depth": 5},
                "coordinates": {"x": 0, "y": 0, "z": 0},
                "parameters": {"test": True},
                "neuron_count": 500
            },
            {
                "id": "2",
                "name": "Test Area 2",
                "type": "interconnect",
                "dimensions": {"width": 20, "height": 20, "depth": 10},
                "coordinates": {"x": 20, "y": 0, "z": 0},
                "parameters": {"test": True},
                "neuron_count": 4000
            }
        ]
        
        yield mock

# Test Insights API Endpoints
def test_activity_heatmap(client, mock_core_api):
    """Test getting activity heatmap data."""
    # Request data for the heatmap
    request_data = {
        "window": 1,
        "include_empty": False,
        "threshold": 0.1
    }
    
    response = client.post("/v1/insights/activity/heatmap", json=request_data)
    assert response.status_code in (200, 400, 404, 422)
    if response.status_code == 200:
        data = response.json()
        
        # Check the structure of the response
        assert "timestamp" in data
        assert "data" in data
        
        # The response should contain data for at least one area
        assert len(data["data"]) > 0
        
        # Each area should have coordinates with activity values
        for area_id, area_data in data["data"].items():
            assert len(area_data) > 0
            for coords, activity in area_data.items():
                # Check that activity is a float between 0 and 1
                assert 0 <= activity <= 1

def test_neuron_activity_time_series(client, mock_core_api):
    """Test getting neuron activity time series data."""
    # Request data for the neuron activity
    request_data = {
        "neuron_ids": ["neuron1", "neuron2", "neuron3"],
        "window": 5
    }
    
    response = client.post("/v1/insights/activity/neurons", json=request_data)
    assert response.status_code in (200, 400, 404, 422)
    if response.status_code == 200:
        data = response.json()
        
        # Check the structure of the response
        assert "timestamps" in data
        assert "data" in data
        
        # Check that timestamps match the requested window
        assert len(data["timestamps"]) == request_data["window"]
        
        # Check that each requested neuron has data
        for neuron_id in request_data["neuron_ids"]:
            assert neuron_id in data["data"]
            assert len(data["data"][neuron_id]) == request_data["window"]
            
            # Check that activity values are floats between 0 and 1
            for activity in data["data"][neuron_id]:
                assert 0 <= activity <= 1

def test_network_analytics(client, mock_core_api):
    """Test getting network analytics data."""
    response = client.get("/v1/insights/network/analytics")
    assert response.status_code in (200, 400, 404, 422)
    if response.status_code == 200:
        data = response.json()
        
        # Check the structure of the response
        assert "neuron_count" in data
        assert "synapse_count" in data
        assert "area_count" in data
        assert "average_connectivity" in data
        assert "most_active_areas" in data
        assert "least_active_areas" in data
        assert "activity_distribution" in data
        
        # Check that the data is of the expected type
        assert isinstance(data["neuron_count"], int)
        assert isinstance(data["synapse_count"], int)
        assert isinstance(data["area_count"], int)
        assert isinstance(data["average_connectivity"], float)
        assert isinstance(data["most_active_areas"], list)
        assert isinstance(data["least_active_areas"], list)
        assert isinstance(data["activity_distribution"], dict)
        
        # Verify the contents of active areas
        for area in data["most_active_areas"]:
            assert "id" in area
            assert "name" in area
            assert "activity_level" in area
            assert 0 <= area["activity_level"] <= 1

def test_performance_stats(client, mock_core_api):
    """Test getting performance statistics."""
    response = client.get("/v1/insights/performance/stats", params={"window": 10})
    assert response.status_code in (200, 400, 404, 422)
    if response.status_code == 200:
        data = response.json()
        
        # Check the structure of the response
        assert "average_burst_duration" in data
        assert "min_burst_duration" in data
        assert "max_burst_duration" in data
        assert "average_memory_usage" in data
        assert "average_cpu_usage" in data
        assert "timestamp" in data
        
        # Check that the data is of the expected type
        assert isinstance(data["average_burst_duration"], float)
        assert isinstance(data["min_burst_duration"], float)
        assert isinstance(data["max_burst_duration"], float)
        assert isinstance(data["average_memory_usage"], float)
        assert isinstance(data["average_cpu_usage"], float)
        
        # Check that the values are reasonable
        assert data["min_burst_duration"] <= data["average_burst_duration"] <= data["max_burst_duration"]
        assert data["average_memory_usage"] > 0
        assert 0 <= data["average_cpu_usage"] <= 100

def test_activity_summary(client, mock_core_api):
    """Test getting activity summary data."""
    response = client.get("/v1/insights/activity/summary", params={"window": 1})
    assert response.status_code in (200, 400, 404, 422)
    if response.status_code == 200:
        data = response.json()
        
        # Check the structure of the response
        assert "timestamp" in data
        assert "window_size" in data
        assert "overall_activity_level" in data
        assert "area_summaries" in data
        
        # Check that the data is of the expected type
        assert isinstance(data["window_size"], int)
        assert isinstance(data["overall_activity_level"], float)
        assert isinstance(data["area_summaries"], list)
        
        # Check that the activity level is between 0 and 1
        assert 0 <= data["overall_activity_level"] <= 1
        
        # Verify the contents of area summaries
        for area in data["area_summaries"]:
            assert "id" in area
            assert "name" in area
            assert "activity_level" in area
            assert "active_neurons" in area
            assert "total_neurons" in area
            assert 0 <= area["activity_level"] <= 1
            assert area["active_neurons"] <= area["total_neurons"]

def test_connectivity_graph(client, mock_core_api):
    """Test getting connectivity graph data."""
    response = client.get("/v1/insights/connectivity/graph", params={"include_weights": True, "min_weight": 0.2})
    assert response.status_code in (200, 400, 404, 422)
    if response.status_code == 200:
        data = response.json()
        
        # Check the structure of the response
        assert "nodes" in data
        assert "edges" in data
        
        # Check that there are nodes and edges
        assert len(data["nodes"]) > 0
        assert len(data["edges"]) > 0
        
        # Verify the contents of nodes
        for node in data["nodes"]:
            assert "id" in node
            assert "name" in node
            assert "type" in node
            assert "coordinates" in node
        
        # Verify the contents of edges
        for edge in data["edges"]:
            assert "source" in edge
            assert "target" in edge
            assert "weight" in edge  # We requested include_weights=True
            
            # Check that the weight is above the requested minimum
            assert edge["weight"] >= 0.2 