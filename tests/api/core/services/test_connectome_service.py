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

"""Tests for the ConnectomeService class."""

import pytest
from unittest.mock import MagicMock, patch

from feagi.api.core.services.connectome.connectome_service import ConnectomeService
from feagi.bdu.connectome_manager import ConnectomeManager


@pytest.fixture
def mock_connectome_manager():
    """Create a mock ConnectomeManager for testing."""
    cm = MagicMock(spec=ConnectomeManager)
    
    # Mock neurons and synapses
    cm.neurons = {
        1: {"cortical_id": "area01", "position": (1, 1, 1)},
        2: {"cortical_id": "area01", "position": (2, 2, 2)},
        3: {"cortical_id": "area02", "position": (1, 1, 1)},
        4: {"cortical_id": "area02", "position": (2, 2, 2)}
    }
    
    cm.synapses = {
        (1, 2): {"weight": 0.7, "delay": 1},
        (1, 3): {"weight": 0.5, "delay": 2},
        (2, 4): {"weight": 0.8, "delay": 1},
        (3, 4): {"weight": 0.6, "delay": 3}
    }
    
    # Mock methods
    cm.get_neuron_connectivity = MagicMock()
    cm.get_connection_stats = MagicMock()
    cm.get_connection_matrix = MagicMock()
    cm.add_connection = MagicMock()
    cm.remove_connection = MagicMock()
    cm.update_connection_weight = MagicMock()
    cm.get_area_to_area_connectivity = MagicMock()
    cm.analyze_network_properties = MagicMock()
    
    return cm


@pytest.fixture
def mock_state_manager():
    """Create a mock state manager for testing."""
    sm = MagicMock()
    sm.is_genome_loaded.return_value = True
    return sm


@pytest.fixture
def connectome_service(mock_connectome_manager, mock_state_manager):
    """Create a ConnectomeService instance with mocked dependencies."""
    return ConnectomeService(mock_connectome_manager, mock_state_manager)


@pytest.fixture
def connectome_service_no_state(mock_connectome_manager):
    """Create a ConnectomeService instance without state manager."""
    return ConnectomeService(mock_connectome_manager, None)


class TestConnectomeService:
    """Test cases for the ConnectomeService."""

    def test_init(self, mock_connectome_manager, mock_state_manager):
        """Test ConnectomeService initialization."""
        service = ConnectomeService(mock_connectome_manager, mock_state_manager)
        
        assert service._connectome_manager == mock_connectome_manager
        assert service.state_manager == mock_state_manager

    def test_get_neuron_connectivity_success(self, connectome_service, mock_connectome_manager):
        """Test getting neuron connectivity successfully."""
        # Mock genome validation to succeed
        connectome_service._validate_genome_loaded = MagicMock(return_value=True)
        
        # Mock neuron existence
        mock_connectome_manager._neuron_id_to_index = {1: 0, 2: 1, 3: 2}
        
        # Mock connectivity data
        mock_connectome_manager.get_incoming_connections.return_value = [(2, 0.5), (3, 0.7)]
        mock_connectome_manager.get_outgoing_connections.return_value = [(3, 0.8)]
        
        result = connectome_service.get_neuron_connectivity("1", direction="both")
        
        assert isinstance(result, dict)
        assert result["neuron_id"] == "1"
        assert result["direction"] == "both"
        assert len(result["incoming_connections"]) == 2
        assert len(result["outgoing_connections"]) == 1

    def test_get_neuron_connectivity_incoming_only(self, connectome_service, mock_connectome_manager):
        """Test getting incoming connectivity only."""
        # Mock genome validation to succeed
        connectome_service._validate_genome_loaded = MagicMock(return_value=True)
        
        # Mock neuron existence
        mock_connectome_manager._neuron_id_to_index = {1: 0}
        mock_connectome_manager.get_incoming_connections.return_value = [(2, 0.5)]
        
        result = connectome_service.get_neuron_connectivity("1", direction="incoming")
        
        assert isinstance(result, dict)
        assert result["direction"] == "incoming"
        assert "incoming_connections" in result
        assert "outgoing_connections" not in result

    def test_get_neuron_connectivity_nonexistent_neuron(self, connectome_service, mock_connectome_manager):
        """Test getting connectivity for nonexistent neuron."""
        mock_connectome_manager.get_neuron_connectivity.return_value = None
        
        result = connectome_service.get_neuron_connectivity("nonexistent")
        
        assert result is None

    def test_get_neuron_connectivity_with_exception(self, connectome_service, mock_connectome_manager):
        """Test getting neuron connectivity when an exception occurs."""
        mock_connectome_manager.get_neuron_connectivity.side_effect = Exception("Database error")
        
        result = connectome_service.get_neuron_connectivity("1")
        
        assert result is None

    def test_get_connection_stats_success(self, connectome_service, mock_connectome_manager):
        """Test getting connection statistics successfully."""
        # Mock genome validation to succeed
        connectome_service._validate_genome_loaded = MagicMock(return_value=True)
        
        # Mock the attributes that get_connection_stats expects
        mock_connectome_manager._neuron_id_to_index = {1: 0, 2: 1, 3: 2}  # 3 neurons
        mock_connectome_manager._outgoing_connections = {
            1: [(2, 0.5), (3, 0.7)],  # 2 connections from neuron 1
            2: [(3, 0.8)],            # 1 connection from neuron 2
            3: []                     # 0 connections from neuron 3
        }
        mock_connectome_manager.cortical_areas = {1: MagicMock(), 2: MagicMock()}  # 2 areas
        
        result = connectome_service.get_connection_stats()
        
        assert isinstance(result, dict)
        assert result["total_neurons"] == 3
        assert result["total_synapses"] == 3  # Total connections across all neurons
        assert result["total_cortical_areas"] == 2

    def test_get_connection_stats_empty(self, connectome_service, mock_connectome_manager):
        """Test getting connection statistics when no connections exist."""
        # Mock genome validation to succeed
        connectome_service._validate_genome_loaded = MagicMock(return_value=True)
        
        # Mock empty data structures
        mock_connectome_manager._neuron_id_to_index = {}
        mock_connectome_manager._outgoing_connections = {}
        mock_connectome_manager.cortical_areas = {}
        
        result = connectome_service.get_connection_stats()
        
        assert isinstance(result, dict)
        assert result["total_neurons"] == 0
        assert result["total_synapses"] == 0
        assert result["total_cortical_areas"] == 0

    def test_get_connection_matrix_success(self, connectome_service, mock_connectome_manager):
        """Test getting connection matrix between areas."""
        # Mock genome validation to succeed
        connectome_service._validate_genome_loaded = MagicMock(return_value=True)
        
        # Mock neurons in areas
        mock_connectome_manager.get_neurons_by_area.side_effect = lambda area_id: {
            1: [1, 2],  # Source area has neurons 1, 2
            2: [3, 4]   # Target area has neurons 3, 4
        }.get(area_id, [])
        
        # Mock connections: neuron 1 -> neuron 3 with weight 0.8
        mock_connectome_manager.get_outgoing_connections.side_effect = lambda neuron_id: {
            1: [(3, 0.8)],  # Neuron 1 connects to neuron 3
            2: []           # Neuron 2 has no outgoing connections
        }.get(neuron_id, [])
        
        result = connectome_service.get_connection_matrix("1", "2")
        
        assert isinstance(result, dict)
        assert result["source_area"] == "1"
        assert result["target_area"] == "2"
        assert result["connection_count"] == 1  # Only neuron 1 -> neuron 3

    def test_get_connection_matrix_no_connections(self, connectome_service, mock_connectome_manager):
        """Test getting connection matrix when no connections exist."""
        mock_connectome_manager.get_connection_matrix.return_value = None
        
        result = connectome_service.get_connection_matrix("area01", "area03")
        
        assert result is None

    def test_add_connection_success(self, connectome_service, mock_connectome_manager):
        """Test successfully adding a connection."""
        # Mock genome validation to succeed
        connectome_service._validate_genome_loaded = MagicMock(return_value=True)
        
        # Mock neuron existence
        mock_connectome_manager._neuron_id_to_index = {1: 0, 2: 1}
        mock_connectome_manager.add_connection = MagicMock()
        
        result = connectome_service.add_connection("1", "2", 0.75)
        
        assert result is True
        mock_connectome_manager.add_connection.assert_called_once_with(1, 2, 0.75)

    def test_add_connection_failure(self, connectome_service, mock_connectome_manager):
        """Test adding a connection when it fails."""
        mock_connectome_manager.add_connection.return_value = False
        
        result = connectome_service.add_connection("1", "nonexistent")
        
        assert result is False

    def test_add_connection_with_default_weight(self, connectome_service, mock_connectome_manager):
        """Test adding connection with default weight."""
        # Mock genome validation to succeed
        connectome_service._validate_genome_loaded = MagicMock(return_value=True)
        
        # Mock neuron existence
        mock_connectome_manager._neuron_id_to_index = {1: 0, 2: 1}
        mock_connectome_manager.add_connection = MagicMock()
        
        result = connectome_service.add_connection("1", "2")  # No weight specified
        
        assert result is True
        mock_connectome_manager.add_connection.assert_called_once_with(1, 2, 1.0)  # Default weight

    def test_remove_connection_success(self, connectome_service, mock_connectome_manager):
        """Test successfully removing a connection."""
        # Mock genome validation to succeed
        connectome_service._validate_genome_loaded = MagicMock(return_value=True)
        
        mock_connectome_manager.remove_connection = MagicMock()
        
        result = connectome_service.remove_connection("1", "2")
        
        assert result is True
        mock_connectome_manager.remove_connection.assert_called_once_with(1, 2)  # Integer args

    def test_remove_connection_nonexistent(self, connectome_service, mock_connectome_manager):
        """Test removing a nonexistent connection."""
        # Mock genome validation to succeed but connection removal to fail
        connectome_service._validate_genome_loaded = MagicMock(return_value=True)
        
        # Mock that the removal raises an exception (which gets caught)
        mock_connectome_manager.remove_connection = MagicMock(side_effect=Exception("Connection not found"))
        
        result = connectome_service.remove_connection("1", "nonexistent")
        
        assert result is False

    def test_update_connection_weight_success(self, connectome_service, mock_connectome_manager):
        """Test successfully updating connection weight."""
        # Mock genome validation to succeed
        connectome_service._validate_genome_loaded = MagicMock(return_value=True)
        
        mock_connectome_manager.update_connection_weight = MagicMock()
        
        result = connectome_service.update_connection_weight("1", "2", 0.85)
        
        assert result is True
        mock_connectome_manager.update_connection_weight.assert_called_once_with(1, 2, 0.85)  # Integer args

    def test_update_connection_weight_nonexistent(self, connectome_service, mock_connectome_manager):
        """Test updating weight of a nonexistent connection."""
        # Mock genome validation to succeed but weight update to fail
        connectome_service._validate_genome_loaded = MagicMock(return_value=True)
        
        # Mock that the update raises an exception (which gets caught)
        mock_connectome_manager.update_connection_weight = MagicMock(side_effect=Exception("Connection not found"))
        
        result = connectome_service.update_connection_weight("1", "nonexistent", 0.5)
        
        assert result is False

    def test_get_area_to_area_connectivity_success(self, connectome_service, mock_connectome_manager):
        """Test getting area-to-area connectivity matrix."""
        # Mock genome validation to succeed
        connectome_service._validate_genome_loaded = MagicMock(return_value=True)
        
        # Mock cortical areas
        mock_connectome_manager.cortical_areas = {1: MagicMock(), 2: MagicMock()}
        
        # Mock neurons in areas
        mock_connectome_manager.get_neurons_by_area.side_effect = lambda area_id: {
            1: [1, 2],  # Area 1 has neurons 1, 2
            2: [3, 4]   # Area 2 has neurons 3, 4
        }.get(area_id, [])
        
        # Mock outgoing connections
        mock_connectome_manager.get_outgoing_connections.side_effect = lambda neuron_id: {
            1: [(3, 0.5)],  # Neuron 1 -> neuron 3
            2: [(4, 0.7)],  # Neuron 2 -> neuron 4
            3: [],
            4: []
        }.get(neuron_id, [])
        
        result = connectome_service.get_area_to_area_connectivity()
        
        assert isinstance(result, dict)
        assert "1" in result  # Area 1 should be in result
        assert "2" in result  # Area 2 should be in result

    def test_get_area_to_area_connectivity_empty(self, connectome_service, mock_connectome_manager):
        """Test getting area-to-area connectivity when no areas exist."""
        mock_connectome_manager.get_area_to_area_connectivity.return_value = {}
        
        result = connectome_service.get_area_to_area_connectivity()
        
        assert isinstance(result, dict)
        assert len(result) == 0

    def test_analyze_network_properties_success(self, connectome_service, mock_connectome_manager):
        """Test analyzing network properties."""
        # Mock genome validation to succeed
        connectome_service._validate_genome_loaded = MagicMock(return_value=True)
        
        # Mock network data
        mock_connectome_manager._neuron_id_to_index = {1: 0, 2: 1, 3: 2}
        mock_connectome_manager._outgoing_connections = {
            1: [(2, 0.5), (3, 0.7)],
            2: [(3, 0.8)],
            3: []
        }
        mock_connectome_manager.get_outgoing_connections.side_effect = lambda neuron_id: {
            1: [(2, 0.5), (3, 0.7)],
            2: [(3, 0.8)],
            3: []
        }.get(neuron_id, [])
        mock_connectome_manager.get_incoming_connections.side_effect = lambda neuron_id: {
            1: [],
            2: [(1, 0.5)],
            3: [(1, 0.7), (2, 0.8)]
        }.get(neuron_id, [])
        
        result = connectome_service.analyze_network_properties()
        
        assert isinstance(result, dict)
        assert "total_neurons" in result
        assert "total_connections" in result
        assert "average_out_degree" in result
        assert result["total_neurons"] == 3

    def test_analyze_network_properties_small_network(self, connectome_service, mock_connectome_manager):
        """Test analyzing a very small network."""
        # Mock genome validation to succeed
        connectome_service._validate_genome_loaded = MagicMock(return_value=True)
        
        # Mock minimal network
        mock_connectome_manager._neuron_id_to_index = {1: 0}
        mock_connectome_manager._outgoing_connections = {1: []}
        mock_connectome_manager.get_outgoing_connections.return_value = []
        mock_connectome_manager.get_incoming_connections.return_value = []
        
        result = connectome_service.analyze_network_properties()
        
        assert isinstance(result, dict)
        assert result["total_neurons"] == 1
        assert result["total_connections"] == 0

    def test_error_handling_in_methods(self, connectome_service, mock_connectome_manager):
        """Test that methods handle exceptions gracefully."""
        # Make connectome manager methods raise exceptions
        mock_connectome_manager.get_connection_stats.side_effect = Exception("Network error")
        mock_connectome_manager.add_connection.side_effect = Exception("Add error")
        
        # Test that methods don't crash with exceptions
        stats = connectome_service.get_connection_stats()
        assert isinstance(stats, dict)
        assert len(stats) == 0  # Should return empty dict on error
        
        result = connectome_service.add_connection("1", "2")
        assert result is False  # Should return False on error

    def test_connection_operations_workflow(self, connectome_service, mock_connectome_manager):
        """Test a complete workflow of connection operations."""
        # Mock genome validation to succeed
        connectome_service._validate_genome_loaded = MagicMock(return_value=True)
        
        # Mock neuron existence
        mock_connectome_manager._neuron_id_to_index = {1: 0, 2: 1, 3: 2}
        mock_connectome_manager.add_connection = MagicMock()
        mock_connectome_manager.update_connection_weight = MagicMock()
        mock_connectome_manager.remove_connection = MagicMock()
        
        # Add a connection
        result1 = connectome_service.add_connection("1", "2", 0.5)
        assert result1 is True
        
        # Update its weight
        result2 = connectome_service.update_connection_weight("1", "2", 0.8)
        assert result2 is True
        
        # Remove the connection
        result3 = connectome_service.remove_connection("1", "2")
        assert result3 is True

    def test_parameter_validation(self, connectome_service, mock_connectome_manager):
        """Test parameter validation in connection operations."""
        # Mock genome validation to succeed
        connectome_service._validate_genome_loaded = MagicMock(return_value=True)
        
        # Mock neuron existence
        mock_connectome_manager._neuron_id_to_index = {1: 0, 2: 1}
        mock_connectome_manager.add_connection = MagicMock()
        
        # Test with valid parameters
        result = connectome_service.add_connection("1", "2", 0.5)
        assert result is True

    def test_direction_parameter_handling(self, connectome_service, mock_connectome_manager):
        """Test direction parameter handling in connectivity queries."""
        # Mock genome validation to succeed
        connectome_service._validate_genome_loaded = MagicMock(return_value=True)
        
        # Mock neuron existence
        mock_connectome_manager._neuron_id_to_index = {1: 0}
        
        # Test incoming direction - should only call get_incoming_connections
        mock_connectome_manager.get_incoming_connections.return_value = [(2, 0.5)]
        mock_connectome_manager.get_outgoing_connections.return_value = []
        
        result = connectome_service.get_neuron_connectivity("1", "incoming")
        
        assert result["direction"] == "incoming"
        assert "incoming_connections" in result
        # Should not include outgoing connections for incoming-only query
        assert "outgoing_connections" not in result

    def test_complex_data_structures(self, connectome_service, mock_connectome_manager):
        """Test handling of complex connection data structures."""
        # Mock genome validation to succeed
        connectome_service._validate_genome_loaded = MagicMock(return_value=True)
        
        # Mock complex connectivity between areas
        mock_connectome_manager.get_neurons_by_area.side_effect = lambda area_id: {
            1: [1, 2, 3],  # Area 1 has 3 neurons
            2: [4, 5, 6]   # Area 2 has 3 neurons
        }.get(area_id, [])
        
        # Mock complex connection pattern
        mock_connectome_manager.get_outgoing_connections.side_effect = lambda neuron_id: {
            1: [(4, 0.1), (5, 0.2)],  # Neuron 1 connects to neurons 4,5
            2: [(6, 0.3)],            # Neuron 2 connects to neuron 6
            3: [(4, 0.4), (6, 0.5)]   # Neuron 3 connects to neurons 4,6
        }.get(neuron_id, [])
        
        result = connectome_service.get_connection_matrix("1", "2")
        
        assert isinstance(result, dict)
        assert result["connection_count"] == 5  # Total connections from area 1 to area 2 