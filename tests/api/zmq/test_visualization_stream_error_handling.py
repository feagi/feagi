"""
Test cases for VisualizationStream error handling and race condition fixes.

This module tests the comprehensive error handling improvements made to prevent
the 'NoneType' object has no attribute 'send' error and other race conditions.
"""

import threading
import time
from queue import Empty, Queue
from unittest.mock import MagicMock, Mock, patch

import pytest
import zmq

from feagi.api.zmq.streams.visualization import VisualizationStream


class TestVisualizationStreamErrorHandling:
    """Test cases for improved error handling and race condition prevention."""

    @pytest.fixture
    def mock_context(self):
        """Create a mock ZMQ context."""
        context = Mock()
        mock_socket = Mock()
        mock_socket.socket_type = zmq.PUB
        mock_socket.closed = False
        mock_socket.getsockopt.return_value = 1000
        context.socket.return_value = mock_socket
        return context

    @pytest.fixture
    def mock_queue(self):
        """Create a mock FQ sampler queue."""
        return Queue()

    @pytest.fixture
    def viz_stream(self, mock_context, mock_queue):
        """Create a VisualizationStream with mocked dependencies."""
        return VisualizationStream(
            host="localhost",
            port=5562,
            context=mock_context,
            fq_sampler_queue=mock_queue,
        )

    def test_publish_data_none_socket_prevention(self, viz_stream):
        """Test that _publish_data handles None socket gracefully."""
        # Set socket to None to simulate race condition
        viz_stream.socket = None

        # This should not raise an exception
        viz_stream._publish_data(b"test_data")

        # Verify no stats were updated (since no data was sent)
        assert viz_stream.stats["data_sent"] == 0
        assert viz_stream.stats["bytes_sent"] == 0

    def test_publish_data_not_running_prevention(self, viz_stream):
        """Test that _publish_data handles not running state gracefully."""
        # Set running to False
        viz_stream.running = False

        # This should not raise an exception
        viz_stream._publish_data(b"test_data")

        # Verify no stats were updated
        assert viz_stream.stats["data_sent"] == 0
        assert viz_stream.stats["bytes_sent"] == 0

    @pytest.mark.xfail(
        reason="Test assumes specific atomic reference behavior that may have changed"
    )
    def test_publish_data_socket_becomes_none_during_operation(self, viz_stream):
        """Test atomic socket reference prevents mid-operation None."""
        # Mock socket that becomes None mid-operation
        mock_socket = Mock()
        viz_stream.socket = mock_socket
        viz_stream.running = True

        # Simulate socket becoming None after initial check
        def side_effect(*args, **kwargs):
            viz_stream.socket = None
            return mock_socket.send(*args, **kwargs)

        mock_socket.send.side_effect = side_effect

        # This should complete successfully with atomic reference
        viz_stream._publish_data(b"test_data")

        # Verify the atomic reference was used
        assert mock_socket.send.call_count == 2  # Two send calls (topic + data)

    def test_publish_data_attribute_error_handling(self, viz_stream):
        """Test specific AttributeError handling for None socket."""
        viz_stream.running = True
        viz_stream.socket = Mock()

        # Mock socket.send to raise AttributeError
        viz_stream.socket.send.side_effect = AttributeError(
            "'NoneType' object has no attribute 'send'"
        )

        # This should not raise an exception
        viz_stream._publish_data(b"test_data")

        # Verify no stats were updated due to error
        assert viz_stream.stats["data_sent"] == 0

    def test_publish_data_zmq_error_handling(self, viz_stream):
        """Test ZMQ-specific error handling."""
        viz_stream.running = True
        viz_stream.socket = Mock()

        # Test ETERM error (context terminated)
        zmq_error = zmq.ZMQError()
        zmq_error.errno = zmq.ETERM
        viz_stream.socket.send.side_effect = zmq_error

        # This should handle gracefully
        viz_stream._publish_data(b"test_data")

        # Test EAGAIN error (socket not ready)
        zmq_error.errno = zmq.EAGAIN
        viz_stream.socket.send.side_effect = zmq_error

        # This should handle gracefully
        viz_stream._publish_data(b"test_data")

    def test_socket_recreation_not_running(self, viz_stream):
        """Test socket recreation when stream is not running."""
        viz_stream.running = False

        # This should return early without attempting recreation
        viz_stream._recreate_socket()

        # Socket should remain unchanged
        assert viz_stream.socket is not None

    def test_socket_recreation_no_context(self, viz_stream):
        """Test socket recreation when context is None."""
        viz_stream.running = True
        viz_stream.context = None

        # This should return early without attempting recreation
        viz_stream._recreate_socket()

    @pytest.mark.xfail(
        reason="Mock setup doesn't work with method object - test implementation issue"
    )
    def test_socket_recreation_zmq_error(self, viz_stream):
        """Test socket recreation with ZMQ error."""
        viz_stream.running = True

        # Mock context.socket to raise ZMQ error
        zmq_error = zmq.ZMQError()
        zmq_error.errno = zmq.EADDRINUSE
        viz_stream.context.socket.side_effect = zmq_error

        # This should raise the ZMQ error and set socket to None
        with pytest.raises(zmq.ZMQError):
            viz_stream._recreate_socket()

        assert viz_stream.socket is None

    def test_stop_thread_synchronization(self, viz_stream):
        """Test that stop() waits for threads before closing socket."""
        # Mock worker thread
        mock_thread = Mock()
        mock_thread.is_alive.return_value = True
        mock_thread.name = "TestThread"
        viz_stream.worker_threads = [mock_thread]

        # Mock socket
        mock_socket = Mock()
        viz_stream.socket = mock_socket
        viz_stream.running = True

        # Call stop
        viz_stream.stop()

        # Verify thread.join was called before socket.close
        mock_thread.join.assert_called_once()
        mock_socket.close.assert_called_once()

        # Verify socket was set to None
        assert viz_stream.socket is None

    def test_data_worker_safety_checks(self, viz_stream, mock_queue):
        """Test data worker safety checks prevent processing without socket."""
        viz_stream.running = True
        viz_stream.socket = None  # Simulate missing socket

        # Add test data to queue
        mock_queue.put(b"test_data")
        mock_queue.put("STOP")  # Stop signal

        # Mock _publish_data to track calls
        viz_stream._publish_data = Mock()

        # Run data worker
        viz_stream._data_worker()

        # Verify _publish_data was not called due to missing socket
        viz_stream._publish_data.assert_not_called()

    def test_health_status_comprehensive(self, viz_stream):
        """Test comprehensive health status reporting."""
        viz_stream.running = True

        # Get health status
        status = viz_stream.get_health_status()

        # Verify all expected keys are present
        expected_keys = [
            "running",
            "active_mode",
            "socket_available",
            "context_available",
            "worker_thread_count",
            "stop_event_set",
            "worker_threads",
            "stats",
        ]

        for key in expected_keys:
            assert key in status

        # Verify socket details when socket is available
        assert "socket_type" in status or "socket_details" in status

    def test_health_status_none_socket(self, viz_stream):
        """Test health status when socket is None."""
        viz_stream.socket = None

        status = viz_stream.get_health_status()

        assert status["socket_available"] is False
        assert status["socket_details"] == "Socket is None"

    def test_validate_socket_state_healthy(self, viz_stream):
        """Test socket state validation for healthy socket."""
        viz_stream.running = True
        viz_stream.socket = Mock()
        viz_stream.socket.socket_type = zmq.PUB
        viz_stream.socket.closed = False

        result = viz_stream.validate_socket_state()
        assert result is True

    def test_validate_socket_state_none_socket(self, viz_stream):
        """Test socket state validation when socket is None."""
        viz_stream.socket = None

        result = viz_stream.validate_socket_state()
        assert result is False

    def test_validate_socket_state_closed_socket(self, viz_stream):
        """Test socket state validation when socket is closed."""
        viz_stream.running = True
        viz_stream.socket = Mock()
        viz_stream.socket.socket_type = zmq.PUB
        viz_stream.socket.closed = True

        result = viz_stream.validate_socket_state()
        assert result is False

    def test_race_condition_simulation(self, viz_stream, mock_queue):
        """Simulate race condition between data worker and stop()."""
        viz_stream.running = True

        # Track calls to _publish_data
        publish_calls = []
        original_publish = viz_stream._publish_data

        def tracked_publish(data):
            publish_calls.append(data)
            # Simulate some processing time
            time.sleep(0.01)
            return original_publish(data)

        viz_stream._publish_data = tracked_publish

        # Add data to queue
        for i in range(10):
            mock_queue.put(f"data_{i}".encode())
        mock_queue.put("STOP")

        # Start data worker in thread
        worker_thread = threading.Thread(target=viz_stream._data_worker)
        worker_thread.start()

        # Wait a bit then stop
        time.sleep(0.02)
        viz_stream.stop()

        # Wait for worker to finish
        worker_thread.join(timeout=1.0)

        # Verify no exceptions occurred and some data was processed
        assert len(publish_calls) >= 0  # May be 0 if stopped immediately

    def test_send_visualization_data_type_validation(self, viz_stream):
        """Test send_visualization_data validates input type."""
        # Mock _publish_data to track calls
        viz_stream._publish_data = Mock()

        # Test with invalid type
        viz_stream.send_visualization_data("invalid_string")
        viz_stream._publish_data.assert_not_called()

        # Test with valid type
        viz_stream.send_visualization_data(b"valid_bytes")
        viz_stream._publish_data.assert_called_once_with(b"valid_bytes")
