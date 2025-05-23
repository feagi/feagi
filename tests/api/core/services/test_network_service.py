"""Tests for the NetworkService class."""

import pytest
from unittest.mock import MagicMock

from feagi.api.core.services.network.network_service import NetworkService
from feagi.bdu.connectome_manager import ConnectomeManager


@pytest.fixture
def mock_connectome_manager():
    """Create a mock ConnectomeManager for testing."""
    return MagicMock(spec=ConnectomeManager)


@pytest.fixture
def mock_state_manager():
    """Create a mock state manager for testing."""
    sm = MagicMock()
    sm.network_stats = {
        "total_bytes_sent": 1024000,
        "total_bytes_received": 2048000,
        "connections_active": 5,
        "protocols": {
            "zmq": {"active": True, "connections": 3},
            "rest": {"active": True, "connections": 2}
        }
    }
    return sm


@pytest.fixture
def network_service(mock_connectome_manager, mock_state_manager):
    """Create a NetworkService instance with mocked dependencies."""
    return NetworkService(mock_connectome_manager, mock_state_manager)


@pytest.fixture
def network_service_no_state(mock_connectome_manager):
    """Create a NetworkService instance without state manager."""
    return NetworkService(mock_connectome_manager, None)


class TestNetworkService:
    """Test cases for the NetworkService."""

    def test_init(self, mock_connectome_manager, mock_state_manager):
        """Test NetworkService initialization."""
        service = NetworkService(mock_connectome_manager, mock_state_manager)
        
        assert service._connectome_manager == mock_connectome_manager
        assert service.state_manager == mock_state_manager

    def test_get_network_status_with_state_manager(self, network_service, mock_state_manager):
        """Test getting network status with state manager."""
        result = network_service.get_network_status()
        
        assert isinstance(result, dict)
        assert result["status"] == "healthy"
        assert result["total_bytes_sent"] == 1024000
        assert result["total_bytes_received"] == 2048000
        assert result["connections_active"] == 5

    def test_get_network_status_without_state_manager(self, network_service_no_state):
        """Test getting network status without state manager."""
        result = network_service_no_state.get_network_status()
        
        assert isinstance(result, dict)
        assert result["status"] == "unknown"
        assert result["total_bytes_sent"] == 0
        assert result["total_bytes_received"] == 0

    def test_get_bandwidth_usage_success(self, network_service, mock_state_manager):
        """Test successfully getting bandwidth usage."""
        result = network_service.get_bandwidth_usage(60)
        
        assert isinstance(result, dict)
        assert result["time_window"] == 60
        assert "bytes_per_second" in result
        assert "peak_usage" in result

    def test_get_bandwidth_usage_without_state_manager(self, network_service_no_state):
        """Test getting bandwidth usage without state manager."""
        result = network_service_no_state.get_bandwidth_usage(30)
        
        assert isinstance(result, dict)
        assert result["time_window"] == 30
        assert result["bytes_per_second"] == 0

    def test_get_connection_statistics_success(self, network_service, mock_state_manager):
        """Test successfully getting connection statistics."""
        result = network_service.get_connection_statistics()
        
        assert isinstance(result, dict)
        assert result["active_connections"] == 5
        assert "protocol_breakdown" in result
        assert result["protocol_breakdown"]["zmq"] == 3
        assert result["protocol_breakdown"]["rest"] == 2

    def test_get_connection_statistics_without_state_manager(self, network_service_no_state):
        """Test getting connection statistics without state manager."""
        result = network_service_no_state.get_connection_statistics()
        
        assert isinstance(result, dict)
        assert result["active_connections"] == 0

    def test_test_connectivity_success(self, network_service):
        """Test successful connectivity test."""
        result = network_service.test_connectivity()
        
        assert isinstance(result, dict)
        assert result["status"] == "success"
        assert "latency" in result

    def test_test_connectivity_with_target(self, network_service):
        """Test connectivity test with specific target."""
        result = network_service.test_connectivity("localhost:8080")
        
        assert isinstance(result, dict)
        assert result["target"] == "localhost:8080"
        assert result["status"] in ["success", "failed"]

    def test_test_connectivity_failure(self, network_service):
        """Test connectivity test failure."""
        # Simulate network failure
        with pytest.raises(Exception):
            # This would normally test actual network connectivity
            pass
        
        # Test the fallback behavior
        result = network_service.test_connectivity("unreachable:9999")
        assert isinstance(result, dict)

    def test_get_protocol_status_success(self, network_service, mock_state_manager):
        """Test successfully getting protocol status."""
        result = network_service.get_protocol_status()
        
        assert isinstance(result, dict)
        assert "zmq" in result
        assert "rest" in result
        assert result["zmq"]["active"] is True
        assert result["rest"]["active"] is True

    def test_get_protocol_status_without_state_manager(self, network_service_no_state):
        """Test getting protocol status without state manager."""
        result = network_service_no_state.get_protocol_status()
        
        assert isinstance(result, dict)
        # Should return default protocol status

    def test_reset_network_statistics_success(self, network_service, mock_state_manager):
        """Test successfully resetting network statistics."""
        result = network_service.reset_network_statistics()
        
        assert result is True

    def test_reset_network_statistics_without_state_manager(self, network_service_no_state):
        """Test resetting network statistics without state manager."""
        result = network_service_no_state.reset_network_statistics()
        
        assert result is False

    def test_configure_bandwidth_limits_success(self, network_service, mock_state_manager):
        """Test successfully configuring bandwidth limits."""
        limits = {
            "max_upload_mbps": 100,
            "max_download_mbps": 200,
            "burst_allowance": 50
        }
        
        result = network_service.configure_bandwidth_limits(limits)
        
        assert isinstance(result, dict)
        assert result["success"] is True
        assert result["limits"] == limits

    def test_configure_bandwidth_limits_invalid(self, network_service):
        """Test configuring bandwidth limits with invalid data."""
        limits = {
            "max_upload_mbps": -10,  # Invalid negative value
            "max_download_mbps": "invalid"  # Invalid type
        }
        
        result = network_service.configure_bandwidth_limits(limits)
        
        assert isinstance(result, dict)
        assert result["success"] is False

    def test_configure_bandwidth_limits_without_state_manager(self, network_service_no_state):
        """Test configuring bandwidth limits without state manager."""
        limits = {"max_upload_mbps": 100}
        
        result = network_service_no_state.configure_bandwidth_limits(limits)
        
        assert isinstance(result, dict)
        assert result["success"] is False

    def test_get_message_queue_status_success(self, network_service, mock_state_manager):
        """Test successfully getting message queue status."""
        mock_state_manager.message_queues = {
            "zmq_publisher": {"pending": 5, "processed": 1000},
            "rest_requests": {"pending": 2, "processed": 500}
        }
        
        result = network_service.get_message_queue_status()
        
        assert isinstance(result, dict)
        assert "zmq_publisher" in result
        assert "rest_requests" in result
        assert result["zmq_publisher"]["pending"] == 5

    def test_get_message_queue_status_empty(self, network_service, mock_state_manager):
        """Test getting message queue status when no queues exist."""
        mock_state_manager.message_queues = {}
        
        result = network_service.get_message_queue_status()
        
        assert isinstance(result, dict)
        assert len(result) == 0

    def test_get_message_queue_status_without_state_manager(self, network_service_no_state):
        """Test getting message queue status without state manager."""
        result = network_service_no_state.get_message_queue_status()
        
        assert isinstance(result, dict)
        assert len(result) == 0

    def test_bandwidth_calculation(self, network_service):
        """Test bandwidth calculation logic."""
        # Test bandwidth calculation with different time windows
        result_30 = network_service.get_bandwidth_usage(30)
        result_60 = network_service.get_bandwidth_usage(60)
        
        assert result_30["time_window"] == 30
        assert result_60["time_window"] == 60
        # Bandwidth calculation should account for time window

    def test_protocol_health_checking(self, network_service, mock_state_manager):
        """Test protocol health checking."""
        # Test with healthy protocols
        status = network_service.get_protocol_status()
        assert all(protocol["active"] for protocol in status.values())
        
        # Test with some unhealthy protocols
        mock_state_manager.network_stats["protocols"]["zmq"]["active"] = False
        status = network_service.get_protocol_status()
        # Should reflect the updated status

    def test_connection_limit_validation(self, network_service):
        """Test connection limit validation."""
        valid_limits = {
            "max_upload_mbps": 100,
            "max_download_mbps": 200
        }
        
        invalid_limits = {
            "max_upload_mbps": -50,  # Negative
            "max_download_mbps": "unlimited"  # Wrong type
        }
        
        valid_result = network_service.configure_bandwidth_limits(valid_limits)
        invalid_result = network_service.configure_bandwidth_limits(invalid_limits)
        
        assert valid_result["success"] is True
        assert invalid_result["success"] is False

    def test_error_handling_in_methods(self, network_service, mock_state_manager):
        """Test that methods handle exceptions gracefully."""
        # Make state manager raise exceptions
        mock_state_manager.network_stats = None
        
        # Test that methods don't crash with exceptions
        status = network_service.get_network_status()
        assert isinstance(status, dict)
        
        stats = network_service.get_connection_statistics()
        assert isinstance(stats, dict)

    def test_network_monitoring_workflow(self, network_service, mock_state_manager):
        """Test complete network monitoring workflow."""
        # Get initial status
        status = network_service.get_network_status()
        assert status["status"] == "healthy"
        
        # Check bandwidth usage
        bandwidth = network_service.get_bandwidth_usage(60)
        assert "bytes_per_second" in bandwidth
        
        # Test connectivity
        connectivity = network_service.test_connectivity()
        assert "status" in connectivity
        
        # Check protocol status
        protocols = network_service.get_protocol_status()
        assert isinstance(protocols, dict)
        
        # Reset statistics
        reset_result = network_service.reset_network_statistics()
        assert reset_result is True

    def test_queue_monitoring(self, network_service, mock_state_manager):
        """Test message queue monitoring."""
        # Setup queue data
        mock_state_manager.message_queues = {
            "high_priority": {"pending": 0, "processed": 1000},
            "normal_priority": {"pending": 10, "processed": 500},
            "low_priority": {"pending": 50, "processed": 200}
        }
        
        result = network_service.get_message_queue_status()
        
        assert len(result) == 3
        assert result["high_priority"]["pending"] == 0
        assert result["normal_priority"]["pending"] == 10
        assert result["low_priority"]["pending"] == 50

    def test_bandwidth_limit_edge_cases(self, network_service):
        """Test bandwidth limit configuration edge cases."""
        # Test with zero limits
        zero_limits = {"max_upload_mbps": 0, "max_download_mbps": 0}
        result = network_service.configure_bandwidth_limits(zero_limits)
        assert result["success"] is True  # Zero might be valid for unlimited
        
        # Test with very large limits
        large_limits = {"max_upload_mbps": 999999, "max_download_mbps": 999999}
        result = network_service.configure_bandwidth_limits(large_limits)
        assert result["success"] is True
        
        # Test with empty limits
        empty_limits = {}
        result = network_service.configure_bandwidth_limits(empty_limits)
        assert isinstance(result, dict) 